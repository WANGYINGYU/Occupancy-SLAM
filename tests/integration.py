#!/usr/bin/env python3
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
from make_fixtures import make_bags,make_pcds

ROOT=Path(__file__).resolve().parents[1]
run=Path(os.environ['SDF_BA_RUN_ROOT'])/'integration'
run.mkdir()

def command(name,args,success=True):
    with (run/f'{name}.log').open('w') as log:
        result=subprocess.run([str(v) for v in args],stdout=log,stderr=subprocess.STDOUT)
    if (result.returncode==0)!=success:raise RuntimeError(f'{name}: unexpected exit {result.returncode}; see {run/name}.log')
    print(name,'PASS',flush=True)

command('unit',[sys.executable,ROOT/'tests/test_scan_io.py'])
laser,cloud=make_bags(run)
cases=[('laser_tf',laser,'tf'),('laser_odom',laser,'odom'),('cloud_tf',cloud,'tf'),('cloud_scan_match',cloud,'scan-match')]
reports={}
for i,(name,bag,source) in enumerate(cases):
    output=run/name
    command(name,[sys.executable,ROOT/'tools/rosbag_pipeline.py','--bag',bag,'--output',output,'--odom-source',source,'--iterations','3','--resolution','.3','--beam-stride','2','--bins','360','--max-range','15','--tf-max-gap','1.1','--domain-id',90+i])
    summary=json.loads((output/'summary.json').read_text());reports[name]=summary
    assert summary['accepted_scans']>=20
    poses=np.loadtxt(output/'optimized/poses.txt')
    assert np.isfinite(poses).all() and poses.shape==(summary['accepted_scans'],3)
    if source in ('tf','odom'):
        prior=np.loadtxt(output/'input_odometry.txt')
        np.testing.assert_allclose(prior[0],[.4,-.1,.3],atol=1e-9)
np.testing.assert_allclose(np.loadtxt(run/'laser_tf/input_odometry.txt'),np.loadtxt(run/'laser_odom/input_odometry.txt'),atol=1e-10)
np.testing.assert_allclose(np.loadtxt(run/'laser_tf/Range.txt'),np.loadtxt(run/'cloud_tf/Range.txt'),atol=2e-6)
fixture=make_pcds(run/'pcd_fixture')
backend=ROOT/'build/occupancy_slam_3d'
command('pcd_native',[backend,'--dataset',fixture,'--output',run/'pcd_native','--voxel','.4','--point-voxel','.1','--free-samples','3','--iterations','8','--check-jacobians','1'])
command('pcd_manifest',[backend,'--poses',fixture/'odometry.txt','--manifest',fixture/'scans.txt','--output',run/'pcd_manifest','--voxel','.4','--point-voxel','.1','--free-samples','3','--iterations','8'])
summary=json.loads((run/'pcd_native/summary.json').read_text());reports['pcd_native']=summary
assert summary['final_cost']<summary['initial_cost'] and summary['accepted_steps']>0
result=np.loadtxt(run/'pcd_native/trajectory.tum')
np.testing.assert_allclose(result,np.loadtxt(run/'pcd_manifest/trajectory.tum'),atol=1e-10)
initial=np.loadtxt(fixture/'odometry.txt')
np.testing.assert_allclose(result[0],initial[0],atol=1e-10)
assert np.linalg.norm(result[1:,1:4]-initial[1:,1:4])>1e-5
assert np.linalg.norm(result[1:,4:]-initial[1:,4:])>1e-5
trace=np.genfromtxt(run/'pcd_native/iterations.csv',delimiter=',',names=True)
assert np.all(trace['cost_after']<=trace['cost_before'])
command('reject_bad_option',[backend,'--dataset',fixture,'--output',run/'bad','--voxel','0'],False)
command('reject_existing_output',[backend,'--dataset',fixture,'--output',run/'pcd_native','--max-frames','2','--point-stride','20'],False)
wrong=initial.copy();wrong[2,0]=wrong[1,0]
np.savetxt(run/'bad_poses.txt',wrong)
command('reject_duplicate_time',[backend,'--dataset',fixture,'--poses',run/'bad_poses.txt','--output',run/'bad_times'],False)
(run/'summary.json').write_text(json.dumps(reports,indent=2))
print('All integration checks passed:',run,flush=True)
