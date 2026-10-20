#!/usr/bin/env python3
"""Numerical compatibility, effective configuration, headless exports and 3D I/O."""
import argparse,json,os,subprocess,sys
from pathlib import Path
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tests')]
from make_fixtures import make_pcds
from export_maps import export_result
from result_scene import load_scene
from rviz_viewer import parser
run=Path(os.environ['SDF_BA_RUN_ROOT'])
parser_args=argparse.ArgumentParser(description=__doc__)
parser_args.add_argument('--reference-2d-pipeline',type=Path)
parser_args.add_argument('--reference-binaries',type=Path)
options=parser_args.parse_args()
base=options.reference_2d_pipeline or run/'input'
old=options.reference_binaries
records={'compared_reference_binaries':bool(old)}
def invoke(name,cmd,success=True):
    with (run/(name+'.log')).open('w') as log:r=subprocess.run([str(x) for x in cmd],stdout=log,stderr=subprocess.STDOUT)
    assert (r.returncode==0)==success,(name,(run/(name+'.log')).read_text()[-2000:])
    return run/name
if options.reference_2d_pipeline is None:
    invoke('prepare',['bash',ROOT/'tools/run_2d.sh','txt','--dataset',ROOT/'Data/Intel','--backend-config',ROOT/'config/backend/intel.txt','--output',base,'--start','2','--scan-stride','3','--max-scans','8','--resolution','.5','--iterations','0'])
params=dict(line.split() for line in (base/'backend_config.txt').read_text().splitlines() if line.strip())
params.update(MaxIter='2',MinDelta='0',MinDeltaPose='0',SolverFirstMaxIter='56',SolverFirstTolerance='0.005',ModeSparseSolver='true')
config=run/'compatible.txt';config.write_text(''.join(f'{k} {v}\n' for k,v in params.items()))
common=['--ranges',base/'Range.txt','--poses',base/'Pose.txt','--odom',base/'Odom.txt','--timestamps',base/'timestamps.txt','--config',config]
for name,exe in ([('old2d',old/'occupancy_slam_2d')] if old else [])+[('new2d',ROOT/'build/occupancy_slam_2d')]:invoke(name,[exe,*common,'--output',run/name])
if old:
    for method in ('initial','joint','reintegrated'):
        for file in ('poses.txt','occupancy.txt','normalization.txt'):
            np.testing.assert_allclose(np.loadtxt(run/'new2d'/method/file),np.loadtxt(run/'old2d'/method/file),rtol=0,atol=1e-12)
        np.testing.assert_array_equal(np.asarray(Image.open(run/'new2d'/method/'map.pgm')),np.asarray(Image.open(run/'old2d'/method/'map.pgm')))
report=json.loads((run/'new2d/summary.json').read_text());assert report['normalization_updates']==report['iterations']==2
assert report['objective_increased'] and report['normalization_model'].startswith('paper approximation')
records['2d_matching_effective_solver']=report
for name,iterations,tolerance in [('low',1,.9),('high',500,1e-6)]:
    p=params|dict(SolverFirstMaxIter=str(iterations),SolverFirstTolerance=str(tolerance));f=run/(name+'.txt');f.write_text(''.join(f'{k} {v}\n' for k,v in p.items()))
    invoke(name,[ROOT/'build/occupancy_slam_2d',*common[:-2],'--config',f,'--output',run/name])
assert not np.array_equal(np.loadtxt(run/'low/poses.txt'),np.loadtxt(run/'high/poses.txt'))
records['solver_settings_affect_output']=True
invoke('decrease',[ROOT/'build/occupancy_slam_2d',*common,'--output',run/'decrease','--step-policy','decrease'])
trace=np.genfromtxt(run/'decrease/iterations.csv',delimiter=',',names=True,dtype=None,encoding='utf8');trace=np.atleast_1d(trace)
assert np.all(trace['frozen_n_total_after']<=trace['total_before']+1e-8)
records['decrease']=json.loads((run/'decrease/summary.json').read_text())
invoke('budget',[ROOT/'build/occupancy_slam_2d',*common,'--output',run/'budget','--max-observations','1'],False)
assert json.loads((run/'budget/run_status.json').read_text())['status']=='failed'
fixture=make_pcds(run/'pcd_fixture')
for name,exe in ([('old3d',old/'occupancy_slam_3d')] if old else [])+[('new3d',ROOT/'build/occupancy_slam_3d')]:
    invoke(name,[exe,'--dataset',fixture,'--output',run/name,'--voxel','.4','--point-voxel','.1','--iterations','3','--check-jacobians','1'])
if old:
    for method in ('initial','joint','reintegrated'):
        for file in ('trajectory.tum','map_variables.txt'):
            np.testing.assert_allclose(np.loadtxt(run/'new3d'/method/file),np.loadtxt(run/'old3d'/method/file),atol=1e-12,rtol=0)
records['3d_compatibility']=json.loads((run/'new3d/summary.json').read_text())
invoke('manifest3d',['bash',ROOT/'tools/run_3d.sh','--poses',fixture/'odometry.txt','--manifest',fixture/'scans.txt','--output',run/'manifest3d','--voxel','.4','--point-voxel','.1','--iterations','3'])
np.testing.assert_allclose(np.loadtxt(run/'manifest3d/trajectory.tum'),np.loadtxt(run/'new3d/trajectory.tum'),atol=1e-12,rtol=0)
# Independent display exports remain possible without any ROS imports.
sys.modules['rclpy']=None
for h in (.05,.1):
    out=export_result(base,h,run/f'export_{h}')
    args=parser().parse_args(['2d','--result',str(base),'--display-resolution',str(h)])
    fresh=load_scene(args);args.display_maps=out;saved=load_scene(args)
    for method in fresh['maps']:
        np.testing.assert_array_equal(fresh['maps'][method]['display_grid']['values'],saved['maps'][method]['display_grid']['values'])
records['headless_exports_005_010']=True
(run/'verification.json').write_text(json.dumps(records,indent=2));print(json.dumps(records,indent=2))
