#!/usr/bin/env python3
"""Exercise real two-stage execution, exact sample offsets and native map outputs."""
import json,os,struct,subprocess,sys
from pathlib import Path
import numpy as np
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from backend_config import load_parameters

def main():
    root=Path(os.environ['SDF_BA_RUN_ROOT']);data=root/'input';data.mkdir()
    angles=np.linspace(-np.pi,np.pi,90,endpoint=False)
    truth=np.array([[0,0,0],[.12,.03,.02],[.25,.06,.04],[.38,.04,.05]])
    poses=truth.copy();poses[1:,:2]+=[[.02,-.01],[-.01,.02],[.03,.01]];poses[1:,2]+=[.004,-.003,.006]
    scans=[]
    for x,y,yaw in truth:
        dx,dy=np.cos(angles+yaw),np.sin(angles+yaw)
        with np.errstate(divide='ignore'):
            rx=np.where(abs(dx)<1e-12,np.inf,np.where(dx>0,(5-x)/dx,(-4-x)/dx))
            ry=np.where(abs(dy)<1e-12,np.inf,np.where(dy>0,(4-y)/dy,(-3-y)/dy))
            scans.append(np.minimum(rx,ry))
    scans=np.asarray(scans);np.savetxt(data/'Range.txt',scans);np.savetxt(data/'Pose.txt',poses)
    p=load_parameters(ROOT/'config.txt',dict(ModeMulti='true',DownRate=2,DownTime=2,MaxIter=4,Scale=.2,NumBeam=len(angles),MinAngle=angles[0],MaxAngle=angles[-1],MinRange=0,MaxRange=20,ModeSparseSolver='false',MinDelta=0,MinDeltaPose=0,MinMeanDeltaFirst=0,MinMeanDeltaPoseFirst=0,WeightSmoothN=1,WeightSmoothNSecond=1))
    config=root/'config.txt';config.write_text(''.join(f'{k} {v}\n' for k,v in p.items()))
    command=[sys.executable,str(ROOT/'tools/txt_pipeline.py'),'--dataset',str(data),'--backend-config',str(config),'--output',str(root/'pipeline'),'--display-resolution','.05']
    with (root/'pipeline.log').open('w') as log:subprocess.run(command,check=True,stdout=log,stderr=subprocess.STDOUT)
    result=root/'pipeline/optimized';summary=json.loads((result/'summary.json').read_text())
    assert summary['mode']=='2d_coarse_to_fine' and summary['coarse_iterations']==summary['fine_iterations']==2
    assert summary['normalization_updates']==summary['iterations']==4
    assert yaml.safe_load((result/'coarse/map.yaml').read_text())['resolution']==.4
    for kind in ('initial','joint','reintegrated'):
        assert yaml.safe_load((result/kind/'map.yaml').read_text())['resolution']==.2
    np.testing.assert_array_equal(np.loadtxt(result/'joint/poses.txt'),np.loadtxt(result/'reintegrated/poses.txt'))
    np.testing.assert_array_equal(np.loadtxt(result/'joint/poses.txt')[0],poses[0])
    # Saved backend profiles may contain comments, just like source profiles.
    from export_maps import export_result
    saved_config=root/'pipeline/backend_config.txt'
    saved_config.write_text('# Profile for display replay\n'+saved_config.read_text()+'\n# End of profile\n')
    export_result(root/'pipeline',.1,root/'commented_profile_display')
    assert (root/'commented_profile_display/display_manifest.json').is_file()
    # Cached display maps survive a new viewer output and invalidate on source edits.
    import export_maps
    from unittest.mock import patch
    os.environ['SDF_BA_STORAGE_ROOT']=str(root)
    cached=export_maps.prepare_display(root/'pipeline',.05,root/'viewer_a')
    with patch.object(export_maps,'export_result',side_effect=AssertionError('Cache was not reused')):
        assert export_maps.prepare_display(root/'pipeline',.05,root/'viewer_b')==cached
    source_ranges=root/'pipeline/Range.txt';changed=np.loadtxt(source_ranges);changed[0,0]-=.15
    np.savetxt(source_ranges,changed)
    refreshed=export_maps.prepare_display(root/'pipeline',.05,root/'viewer_c')
    assert refreshed!=cached and cached.is_dir()
    fresh=export_result(root/'pipeline',.05,root/'fresh_after_edit')
    for method in ('initial','optimized','reintegrated'):
        with np.load(refreshed/method/'grid.npz') as a,np.load(fresh/method/'grid.npz') as b:
            for key in a.files:np.testing.assert_array_equal(a[key],b[key])
    other_resolution=export_maps.prepare_display(root/'pipeline',.1,root/'viewer_d')
    assert other_resolution!=refreshed
    (refreshed/'initial/grid.npz').unlink()
    repaired=export_maps.prepare_display(root/'pipeline',.05,root/'viewer_e')
    assert repaired!=refreshed and (repaired/'initial/grid.npz').is_file()
    # A leading frame absent from the pose stream must be skipped explicitly.
    blocks=[np.array([[99.,99.]])]+[np.column_stack([r*np.cos(angles),r*np.sin(angles)]) for r in scans]
    with (root/'samples.bin').open('wb') as f:
        f.write(struct.pack('<Q',len(blocks)))
        for points in blocks:f.write(struct.pack('<Q',len(points)));f.write(np.asarray(points,dtype='<f8').tobytes());f.write(np.full(len(points),.847297860387203,dtype='<f8').tobytes())
    p.update(ModeMulti='false',MaxIter='0');config.write_text(''.join(f'{k} {v}\n' for k,v in p.items()))
    command=[str(ROOT/'build/occupancy_slam_2d'),'--samples',str(root/'samples.bin'),'--sample-start','1','--poses',str(data/'Pose.txt'),'--config',str(config),'--output',str(root/'exact')]
    with (root/'samples.log').open('w') as log:subprocess.run(command,check=True,stdout=log,stderr=subprocess.STDOUT)
    assert np.loadtxt(root/'exact/initial/integration_counts.txt').sum()==len(angles)*len(poses)
    bad=command.copy();bad[4]='0';bad[-1]=str(root/'bad_offset');test=subprocess.run(bad,capture_output=True,text=True)
    assert test.returncode!=0 and 'frame count mismatch' in test.stderr
    (root/'verification.json').write_text(json.dumps(dict(multiresolution_through_txt_wrapper=True,coarse_and_fine_iterations=2,all_three_maps=True,fixed_final_pose_reintegration=True,sample_offset=True,invalid_offset_rejected=True),indent=2))
    print('PASS: TXT configuration, two stages, maps, fixed poses, exact samples and offset checks')
if __name__=='__main__':main()
