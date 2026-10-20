#!/usr/bin/env python3
"""Convert all supplied MATLAB simulation scenes and initializations to C++ input."""
import argparse,json,os,re
from pathlib import Path
import numpy as np
from matlab_simulation import numeric_variables,extract_endpoints,raw_scans
from txt_pipeline import absolute_odometry
from backend_config import load_parameters
from project_paths import ROOT
from export_matlab_samples import export_scene


def convert(source,output):
    output.mkdir(parents=True,exist_ok=False); inventory=[]
    for folder in sorted(source.glob('Simu*_Data')):
        name=folder.name.removesuffix('_Data'); full=int(name[4:]); count=full//10
        raw=folder/('Raw_'+name+'.mat')
        scanfile=next(folder.glob('Scan_Simu*.mat'))
        small_gt=folder/f'Pose_GT_Simu{count}.mat'
        gt=numeric_variables(small_gt,['Pose_GT'])['Pose_GT']
        data=numeric_variables(raw,['groundtruth','feature','rectF','borderF','fsN'])
        np.testing.assert_allclose(gt,data['groundtruth'].T[::10],atol=1e-12,rtol=0)
        odom_gt=numeric_variables(folder/f'Odom_Simu{count}_GT.mat',['Odom'])['Odom']
        np.testing.assert_allclose(absolute_odometry(odom_gt),gt,atol=1e-10,rtol=0)
        hits,receipt=extract_endpoints(scanfile)
        if len(hits)!=count: raise ValueError('Scan/GT count mismatch')
        if receipt['max_stored_free_sample_replay_error_m']>1e-10: raise ValueError('MATLAB free sampling does not match ray model')
        scene=output/name;scene.mkdir()
        np.savez_compressed(scene/'Endpoints.npz',endpoints=hits)
        rows=np.column_stack([np.repeat(np.arange(count),hits.shape[1]),hits.reshape(-1,2)])
        np.savetxt(scene/'Endpoints.txt',rows,fmt=['%d','%.17g','%.17g'])
        np.savetxt(scene/'Pose_GT.txt',gt,fmt='%.17g')
        np.savetxt(scene/'selected_raw_indices.txt',np.arange(0,full,10),fmt='%d')
        np.savez_compressed(scene/'geometry.npz',**{k:data[k] for k in ['feature','rectF','borderF','fsN']})
        rawscan=raw_scans(raw,np.arange(0,full,10))
        np.savez_compressed(scene/'raw_selected_scans.npz',scans=np.stack(rawscan))
        exact=export_scene(scanfile,scene/'samples.bin',count)
        (scene/'samples_receipt.json').write_text(json.dumps(exact,indent=2))
        params=load_parameters(ROOT/'config/backend/simulation_multires_extended.txt',dict(NumBeam=hits.shape[1]))
        (scene/'backend_config.txt').write_text(''.join(f'{k} {v}\n' for k,v in params.items()))
        receipt.update(source_directory=str(folder),scan_file=str(scanfile),ground_truth_file=str(small_gt),
            raw_frame_selection='0,10,20,...; exact match to supplied low-rate GT',observation_format='samples.bin: exact stored Scan.xy/Odd; Endpoints retained for independent-resolution display',exact_samples=exact['samples'],
            frames=count,beams=hits.shape[1],range_min_m=float(np.linalg.norm(hits,axis=2).min()),range_max_m=float(np.linalg.norm(hits,axis=2).max()),
            observations_at_resolution_02=int(np.floor(np.linalg.norm(hits,axis=2)/np.float32(.2)).sum()))
        cases=[]
        for path in sorted(folder.glob('Odom_*_Noise_*.mat')):
            trial=path.stem.rsplit('_',1)[1]; case=scene/('odom_'+trial);case.mkdir()
            odom=numeric_variables(path,['Odom'])['Odom']; pose=absolute_odometry(odom)
            if len(pose)!=count: raise ValueError('Noisy odometry length mismatch')
            np.savetxt(case/'Pose.txt',pose,fmt='%.17g');np.savetxt(case/'Odom.txt',odom,fmt='%.17g')
            np.savetxt(case/'timestamps.txt',np.arange(count),fmt='%.17g')
            info=dict(name=case.name,kind='noisy_odometry',source=str(path),frames=count,scan_start=0,timestamps='synthetic frame indices',pose_source='integrated input odometry; first row absolute',optimization_uses_odometry_constraints=False)
            (case/'input_manifest.json').write_text(json.dumps(info,indent=2));cases.append(info)
        for path in sorted(folder.glob('*Carto_Sync_*.mat')):
            trial=re.search(r'Sync_(\d+)',path.name)[1];case=scene/('carto_'+trial);case.mkdir()
            d=numeric_variables(path,['Pose','timestamps']);pose=d['Pose'];times=d['timestamps'].ravel()
            start=count-len(pose)
            if start not in (0,1) or len(times)!=count: raise ValueError('Unrecognized Cartographer alignment')
            # Missing first pose: do not synthesize it from GT. Match observations from row 1 onward.
            if np.max(np.abs(pose[0]-gt[start]))>1e-9: raise ValueError('Cartographer anchored first pose differs from expected frame; inspect association')
            if np.any(np.diff(times)<=0): raise ValueError('Cartographer timestamps are not increasing')
            np.savetxt(case/'Pose.txt',pose,fmt='%.17g');np.savetxt(case/'timestamps.txt',times[start:],fmt='%.17g')
            np.savetxt(case/'gt_indices.txt',np.arange(start,count),fmt='%d')
            info=dict(name=case.name,kind='cartographer',source=str(path),frames=len(pose),scan_start=start,timestamps='provided timestamps with matching leading frame omitted when pose absent',pose_source='provided Cartographer Pose, unchanged; no GT-filled pose',optimization_uses_odometry_constraints=False)
            (case/'input_manifest.json').write_text(json.dumps(info,indent=2));cases.append(info)
        receipt['cases']=cases
        receipt['not_used_as_initialization']=[p.name for p in folder.glob('Pose_*_First.mat')]
        receipt['not_used_as_observations']=[p.name for p in folder.glob('SelectScan*.mat')]
        (scene/'conversion.json').write_text(json.dumps(receipt,indent=2))
        inventory.append(dict(scene=name,frames=count,beams=hits.shape[1],cases=cases,observations=exact['samples']))
        print(name,count,'frames',len(cases),'initializations',exact['samples'],'exact observations; conversion max error',receipt['max_stored_free_sample_replay_error_m'],flush=True)
    (output/'inventory.json').write_text(json.dumps(inventory,indent=2))
    return inventory


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();convert(a.source,a.output)

if __name__=='__main__':main()
