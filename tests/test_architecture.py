#!/usr/bin/env python3
"""Self-contained pipeline, configuration, map reintegration and 3D I/O checks."""
import json,os,subprocess,sys
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'tests')]
from make_fixtures import make_pcds, make_txt
from export_maps import export_result
from result_scene import load_scene
from rviz_viewer import parser
from result_io import read_pcd

def check2d(pipeline, result):
    config=dict(line.split() for line in (pipeline/'backend_config.txt').read_text().splitlines() if line.strip())
    meta=yaml.safe_load((result/'map.yaml').read_text())
    methods=json.loads((result/'map_outputs.json').read_text())
    np.testing.assert_array_equal(np.loadtxt(result/'joint/poses.txt'),np.loadtxt(result/'reintegrated/poses.txt'))
    ranges=np.loadtxt(pipeline/'Range.txt')
    angles=np.linspace(float(config['MinAngle']),float(config['MaxAngle']),ranges.shape[1])
    total=0
    for method in ('initial','reintegrated'):
        folder=result/method;poses=np.loadtxt(folder/'poses.txt')
        expected=np.zeros_like(np.loadtxt(folder/'occupancy.txt')); counts=np.zeros_like(expected)
        scale=meta['resolution']; origin=np.asarray(meta['origin'][:2])
        for scan,pose in zip(ranges,poses):
            samples=[];targets=[]
            for distance,angle in zip(scan,angles):
                if not float(config['MinRange'])<distance<float(config['MaxRange']):continue
                count=int(np.floor(distance/scale))
                if not count:continue
                hit=distance*np.array([np.cos(angle),np.sin(angle)])
                samples.append(np.vstack([hit]+[hit/count*(count-k) for k in range(1,count)]))
                targets.extend([methods['hit_log_odds']]+[methods['free_log_odds']]*(count-1))
            c,s=np.cos(pose[2]),np.sin(pose[2]); local=np.vstack(samples)
            world=local@np.array([[c,s],[-s,c]])+pose[:2]
            indices=np.floor((world-origin)/scale).astype(int)
            # Accumulate per scan to match the summation grouping of the upstream core.
            scan_sum=np.zeros_like(expected);scan_count=np.zeros_like(expected)
            np.add.at(scan_sum,(indices[:,1],indices[:,0]),targets)
            np.add.at(scan_count,(indices[:,1],indices[:,0]),1)
            expected+=scan_sum;counts+=scan_count
        np.testing.assert_array_equal(counts,np.loadtxt(folder/'integration_counts.txt'))
        np.testing.assert_allclose(expected,np.loadtxt(folder/'occupancy.txt'),atol=1e-10,rtol=1e-12)
        total=int(counts.sum())
    before=np.loadtxt(result/'initial/occupancy.txt');joint=np.loadtxt(result/'joint/occupancy.txt');replay=np.loadtxt(result/'reintegrated/occupancy.txt')
    assert not np.array_equal(before,replay) and not np.array_equal(joint,replay)
    return dict(frames=len(ranges),replayed_hit_and_free_observations=total,independent_replay_checked=True)


def check3d(result):
    assert (result/'joint/trajectory.tum').read_bytes()==(result/'reintegrated/trajectory.tum').read_bytes()
    a=np.loadtxt(result/'initial/map_variables.txt');b=np.loadtxt(result/'joint/map_variables.txt');c=np.loadtxt(result/'reintegrated/map_variables.txt')
    np.testing.assert_array_equal(a[:,:3],b[:,:3]);np.testing.assert_array_equal(a[:,:3],c[:,:3])
    assert np.linalg.norm(a[:,3]-c[:,3])>1e-3 and np.linalg.norm(b[:,3]-c[:,3])>1e-3
    for name in ('initial','joint','reintegrated'):
        values=np.loadtxt(result/name/'map_variables.txt')[:,3]
        _,prob=read_pcd(result/name/'occupancy_vertices.pcd')
        np.testing.assert_allclose(prob,1/(1+np.exp(-np.clip(values,-30,30))),atol=3e-8,rtol=0)
    # A transformed point cloud cannot distinguish the two final map estimators.
    np.testing.assert_array_equal(read_pcd(result/'joint/optimized_points.pcd')[0],read_pcd(result/'reintegrated/optimized_points.pcd')[0])
    return dict(frames=len(np.loadtxt(result/'trajectory.tum')),vertices=len(b),joint_vs_replay_log_odds_l2=float(np.linalg.norm(b[:,3]-c[:,3])))


def check_trilinear_reintegration(root):
    root.mkdir()
    dataset=root/'fixture';(dataset/'PCD').mkdir(parents=True)
    rng=np.random.default_rng(913)
    local=(rng.normal(size=(45,3))*.6+[2,1,.5]).astype(np.float32)
    rotations=Rotation.from_euler('xyz',[[.14,-.19,.31],[.18,-.16,.36]])
    poses=np.column_stack(([1000,1001],[[2,4,1],[2.3,4.05,1.03]],rotations.as_quat()))
    np.savetxt(dataset/'odometry.txt',poses,fmt='%.17g')
    for i in range(2):
        path=dataset/'PCD'/f'full{i}.pcd'
        path.write_bytes((f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(local)}\nHEIGHT 1\nPOINTS {len(local)}\nDATA binary\n').encode()+local.astype('<f4').tobytes())
    reports=[]
    for iterations in (0,3):
        output=root/f'fixture_{iterations}'
        command=[str(ROOT/'build/occupancy_slam_3d'),'--dataset',str(dataset),'--output',str(output),'--point-voxel','0','--free-samples','2','--voxel','.5','--iterations',str(iterations)]
        with (root/f'fixture_{iterations}.log').open('w') as f:subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,check=True)
        world=np.loadtxt(output/'world_from_grid.txt')
        for method in ('initial','reintegrated'):
            folder=output/method; data=np.loadtxt(folder/'map_variables.txt');trajectory=np.loadtxt(folder/'trajectory.tum')
            lookup={tuple(row.astype(int)):i for i,row in enumerate(data[:,:3])}
            sums=np.zeros(len(data));counts=np.zeros(len(data))
            for row in trajectory:
                rot=Rotation.from_quat(row[4:]).as_matrix()
                for point in local.astype(float):
                    distance=np.linalg.norm(point);end=max(0,distance-.5)
                    observations=[(point,np.log(.7/.3),1)]
                    if end>.3:
                        observations += [(point*(end/distance)*j/3,np.log(.4/.6),.5) for j in (1,2)]
                    for p,target,weight in observations:
                        world_point=rot@p+row[1:4]
                        grid_point=world[:3,:3].T@(world_point-world[:3,3])/.5
                        base=np.floor(grid_point).astype(int);frac=grid_point-base
                        for z in (0,1):
                            for y in (0,1):
                                for x in (0,1):
                                    key=tuple(base+[x,y,z]);w=np.prod(np.where([x,y,z],frac,1-frac))*weight
                                    idx=lookup[key];counts[idx]+=w;sums[idx]+=w*target
            expected=np.divide(sums,counts,out=np.zeros_like(sums),where=counts>0)
            np.testing.assert_allclose(counts,np.loadtxt(folder/'integration_weights.txt'),atol=5e-14,rtol=1e-10)
            np.testing.assert_allclose(expected,data[:,3],atol=5e-12,rtol=1e-10)
        if iterations==0:
            for name in ('joint','reintegrated'):
                np.testing.assert_array_equal(np.loadtxt(output/'initial/map_variables.txt'),np.loadtxt(output/name/'map_variables.txt'))
        reports.append(dict(iterations=iterations,independent_trilinear_replay=True,nonidentity_world_frame=True))
    return reports



run=Path(os.environ['OCCUPANCY_RUN_DIR'])
base=run/'input'
records={}
def invoke(name,cmd,success=True):
    with (run/(name+'.log')).open('w') as log:r=subprocess.run([str(x) for x in cmd],stdout=log,stderr=subprocess.STDOUT)
    assert (r.returncode==0)==success,(name,(run/(name+'.log')).read_text()[-2000:])
    return run/name
fixture2d = make_txt(run/'txt_fixture')
invoke('prepare',['bash',ROOT/'tools/occupancy-slam','txt','--dataset',fixture2d,'--backend-config',fixture2d/'backend.txt','--output',base,'--start','2','--scan-stride','3','--max-scans','8','--resolution','.5','--iterations','0'])
params=dict(line.split() for line in (base/'backend_config.txt').read_text().splitlines() if line.strip())
params.update(MaxIter='2',MinDelta='0',MinDeltaPose='0',SolverFirstMaxIter='56',SolverFirstTolerance='0.005',ModeSparseSolver='true')
# Avoid exact grid boundaries in the independent Python/C++ floor-bin check.
# Otherwise a one-ulp transform difference can assign a boundary hit to either cell.
params.update(FixedGrid='true',Sizei='32',Sizej='36',OriginX='-8.013',OriginY='-7.017')
config=run/'solver.txt';config.write_text(''.join(f'{k} {v}\n' for k,v in params.items()))
common=['--ranges',base/'Range.txt','--poses',base/'Pose.txt','--odom',base/'Odom.txt','--timestamps',base/'timestamps.txt','--config',config]
invoke('backend2d',[ROOT/'build/occupancy_slam_2d',*common,'--output',run/'backend2d'])
report=json.loads((run/'backend2d/summary.json').read_text());assert report['normalization_updates']==report['iterations']==2
assert report['normalization_model'].startswith('paper approximation')
records['2d_effective_solver']=report
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
invoke('backend3d',[ROOT/'build/occupancy_slam_3d','--dataset',fixture,'--output',run/'backend3d','--voxel','.4','--point-voxel','.1','--iterations','3','--check-jacobians','1'])
records['3d_backend']=json.loads((run/'backend3d/summary.json').read_text())
invoke('manifest3d',['bash',ROOT/'tools/occupancy-slam','pcd','--poses',fixture/'odometry.txt','--manifest',fixture/'scans.txt','--output',run/'manifest3d','--voxel','.4','--point-voxel','.1','--iterations','3'])
np.testing.assert_allclose(np.loadtxt(run/'manifest3d/trajectory.tum'),np.loadtxt(run/'backend3d/trajectory.tum'),atol=1e-12,rtol=0)
records['2d_reintegration']=check2d(base,run/'backend2d')
records['3d_reintegration']=check3d(run/'backend3d')
records['trilinear_reintegration']=check_trilinear_reintegration(run/'map_checks')
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
