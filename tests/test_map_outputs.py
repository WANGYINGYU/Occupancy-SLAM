#!/usr/bin/env python3
"""Verify true replay, preserved poses, raw map variables and before/after exports."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from result_io import read_pcd


def check2d(pipeline, old):
    result=pipeline/'optimized'
    config=dict(line.split() for line in (pipeline/'backend_config.txt').read_text().splitlines() if line.strip())
    meta=yaml.safe_load((result/'map.yaml').read_text())
    methods=json.loads((result/'map_outputs.json').read_text())
    np.testing.assert_array_equal(np.loadtxt(result/'poses.txt'),np.loadtxt(old/'poses.txt'))
    np.testing.assert_array_equal(np.loadtxt(result/'occupancy.txt'),np.loadtxt(old/'occupancy.txt'))
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
    return dict(frames=len(ranges),replayed_hit_and_free_observations=total,unchanged_joint_result=True,independent_replay_checked=True)


def check3d(result,old):
    np.testing.assert_array_equal(np.loadtxt(result/'trajectory.tum'),np.loadtxt(old/'trajectory.tum'))
    np.testing.assert_array_equal(read_pcd(result/'occupancy_vertices.pcd')[1],read_pcd(old/'occupancy_vertices.pcd')[1])
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
    return dict(frames=len(np.loadtxt(result/'trajectory.tum')),vertices=len(b),unchanged_joint_result=True,joint_vs_replay_log_odds_l2=float(np.linalg.norm(b[:,3]-c[:,3])))


def synthetic(root):
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


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('pipeline','result3d','old2d','old3d','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();a.output.mkdir()
    report={'2d':check2d(a.pipeline,a.old2d),'3d':check3d(a.result3d,a.old3d),'synthetic':synthetic(a.output)}
    (a.output/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':main()
