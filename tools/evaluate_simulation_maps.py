#!/usr/bin/env python3
"""Compare native occupancy contours with independent MATLAB scene geometry."""
import argparse,csv,json,os,time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import cKDTree
from contourpy import contour_generator
from simulation_geometry import reference_geometry,verify_feature_model,polyline_samples
from evaluate_2d import rotation


def load_contour_grid(folder):
    return (np.loadtxt(folder/'occupancy.txt'),np.loadtxt(folder/'normalization.txt'),
            yaml.safe_load((folder/'map.yaml').read_text()))


def contours(folder,threshold,alignment,grid=None):
    m,n,meta=load_contour_grid(folder) if grid is None else grid
    origin=np.asarray(meta['origin'][:2]);h=meta['resolution']
    if meta['origin'][2]!=0:raise ValueError('Expected native unrotated lattice')
    # bilinear(M)/bilinear(N)=logit(p) has exactly the same zero set as bilinear(M-logit(p)*N).
    field=np.ma.array(m-n*np.log(threshold/(1-threshold)),mask=n<=1e-9)
    lines=contour_generator(x=origin[0]+np.arange(m.shape[1])*h,y=origin[1]+np.arange(m.shape[0])*h,z=field,corner_mask=False).lines(0.)
    points=np.concatenate([polyline_samples(line,.01) for line in lines if len(line)>1]) if lines else np.empty((0,2))
    return points@rotation(alignment['yaw_rad']).T+alignment['translation_m']


def compare(predicted,reference,observed,reference_tree=None):
    if not len(predicted):
        return dict(predicted_points=0,forward_mae_m=None,backward_mae_m=None,symmetric_capped_mae_m=1.,precision_010=0.,recall_010=0.,f1_010=0.,precision_020=0.,recall_020=0.,f1_020=0.)
    tree=cKDTree(reference) if reference_tree is None else reference_tree
    forward=tree.query(predicted,workers=1)[0]
    backward=cKDTree(predicted).query(reference[observed],workers=1)[0]
    result=dict(predicted_points=len(predicted),forward_mae_m=float(forward.mean()),forward_p95_m=float(np.percentile(forward,95)),
                backward_mae_m=float(backward.mean()),backward_p95_m=float(np.percentile(backward,95)),
                symmetric_capped_mae_m=float((np.minimum(forward,1.).mean()+np.minimum(backward,1.).mean())/2))
    for label,t in [('010',.1),('020',.2)]:
        precision=float(np.mean(forward<=t));recall=float(np.mean(backward<=t))
        result['precision_'+label]=precision;result['recall_'+label]=recall;result['f1_'+label]=2*precision*recall/(precision+recall) if precision+recall else 0.
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--converted',type=Path,required=True);p.add_argument('--results',type=Path,required=True);p.add_argument('--wait',action='store_true')
    a=p.parse_args();out=Path(os.environ['OCCUPANCY_RUN_DIR']);inventory=json.loads((a.converted/'inventory.json').read_text());rows=[];geometries={}
    for scene in inventory:
        name=scene['scene'];source=a.converted/name
        with np.load(source/'geometry.npz') as d:geometry=dict(d)
        with np.load(source/'raw_selected_scans.npz') as d:scans=d['scans']
        gt=np.loadtxt(source/'Pose_GT.txt')
        reference,observed,description=reference_geometry(geometry,scans,gt)
        reference_tree=cKDTree(reference)
        description['model_check']=verify_feature_model(geometry,scans,gt)
        geometries[name]=description;np.savez_compressed(out/(name+'_reference.npz'),points=reference,observed=observed)
        (out/'geometry_validation.json').write_text(json.dumps(geometries,indent=2))
        for case in scene['cases']:
            result=a.results/name/case['name'];evaluation=result/'evaluation/metrics.json'
            while not evaluation.exists():
                status=result/'run_status.json'
                if status.exists() and json.loads(status.read_text()).get('status')=='failed':break
                if not a.wait:raise ValueError('Missing completed trajectory evaluation: '+str(evaluation))
                time.sleep(2)
            if not evaluation.exists():continue
            alignment=json.loads(evaluation.read_text())['shared_alignment']
            for method in ['initial','joint','reintegrated']:
                grid=load_contour_grid(result/'optimized'/method)
                for threshold in [.5,.65]:
                    points=contours(result/'optimized'/method,threshold,alignment,grid)
                    metrics=compare(points,reference,observed,reference_tree)
                    rows.append(dict(scene=name,case=case['name'],kind=case['kind'],method=method,probability_threshold=threshold,**metrics))
            (out/'metrics.json').write_text(json.dumps(rows,indent=2))
            with (out/'metrics.csv').open('w',newline='') as f:
                keys=list(dict.fromkeys(k for row in rows for k in row));w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
            print(name,case['name'],'three maps evaluated',flush=True)
    (out/'protocol.json').write_text(json.dumps(dict(geometry='Original Fourier radial boundaries, rectangles and outer border, excluding overlapping hidden boundaries',precision_target='all true boundary geometry',recall_target='true boundary within 0.10 m of geometry-projected raw GT hits',thresholds=[.5,.65],geometry_spacing_m=.005,prediction_spacing_m=.01,capped_distance_m=1.,alignment='same initial first-pose SE2 as trajectory evaluation; no map ICP'),indent=2))

if __name__=='__main__':main()
