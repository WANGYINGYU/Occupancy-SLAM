#!/usr/bin/env python3
"""Legacy Range/Pose/Odom text input to the C++ 2D backend and standard viewer outputs."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import numpy as np
from pipeline_config import parse_config, copy_input_config
from result_io import read_matrix
from geometry import se2, pose2
from backend_config import read_parameters, load_parameters
from backend_runner import run_backend
from run_context import RunContext
from project_paths import ROOT, backend_path



def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,help='Input YAML; CLI overrides YAML')
    p.add_argument('--dataset',type=Path,help='Directory with Range.txt, Pose.txt and optional Odom.txt')
    for key in ('ranges','poses','odom','timestamps'):
        p.add_argument('--'+key,type=Path)
    p.add_argument('--backend-config',type=Path,default=ROOT/'config.txt',help='Original key/value config.txt format')
    p.add_argument('--backend',type=Path,default=backend_path('occupancy_slam_2d'))
    p.add_argument('--output',type=Path,default=Path('pipeline'))
    p.add_argument('--pose-source',choices=('auto','file','odom'),default='auto')
    p.add_argument('--start',type=int,default=0)
    p.add_argument('--scan-stride',type=int,default=1)
    p.add_argument('--max-scans',type=int,default=0)
    p.add_argument('--scan-period',type=float,default=.1,help='Synthetic timestamp interval when no timestamp file exists')
    p.add_argument('--resolution',type=float,help='Override backend Scale (not display resolution)')
    p.add_argument('--iterations',type=int,help='Override backend MaxIter')
    p.add_argument('--odom-weight',type=float,help='Override backend WeightO')
    p.add_argument('--step-policy',choices=('legacy','decrease'),default='legacy')
    p.add_argument('--max-observations',type=int,default=20000000)
    p.add_argument('--display-resolution',type=float,default=.05)
    return p


def absolute_odometry(increments):
    """Original convention: row 0 is an absolute pose; later rows are local increments."""
    poses = np.empty_like(increments)
    poses[0] = increments[0]
    for i in range(1,len(increments)):
        poses[i] = pose2(se2(poses[i-1]) @ se2(increments[i]))
    return poses


def selected_odometry(increments,indices):
    absolute = absolute_odometry(increments)
    result = np.empty((len(indices),3))
    result[0] = absolute[indices[0]]
    for i in range(1,len(indices)):
        result[i] = pose2(np.linalg.inv(se2(absolute[indices[i-1]])) @ se2(absolute[indices[i]]))
    return result


def run(a):
    if a.start < 0 or a.scan_stride < 1 or a.max_scans < 0 or not math.isfinite(a.scan_period) or a.scan_period <= 0:
        raise ValueError('Invalid frame selection or scan period')
    if not math.isfinite(a.display_resolution) or a.display_resolution<=0 or a.max_observations<1: raise ValueError('Invalid display resolution/observation budget')
    if not a.backend.is_file():
        raise ValueError(f'Build the C++ backend first: {a.backend}')
    for key,filename in (('ranges','Range.txt'),('poses','Pose.txt'),('odom','Odom.txt'),('timestamps','timestamps.txt')):
        if getattr(a,key) is None and a.dataset and (a.dataset/filename).is_file():
            setattr(a,key,a.dataset/filename)
    if a.ranges is None:
        raise ValueError('Supply --ranges or a --dataset containing Range.txt')
    params=load_parameters(a.backend_config,dict(Scale=a.resolution,MaxIter=a.iterations,WeightO=a.odom_weight))
    ranges = read_matrix(a.ranges,int(params['NumBeam']))
    odom = read_matrix(a.odom,3) if a.odom else None
    source = a.pose_source
    if source == 'auto':
        source = 'odom' if params['PosefromOdom'] == 'true' or (a.poses is None and odom is not None) else 'file'
    if source == 'odom':
        if odom is None: raise ValueError('Odometry initialization requires Odom.txt or --odom')
        poses = absolute_odometry(odom)
    else:
        if a.poses is None: raise ValueError('Pose initialization requires Pose.txt or --poses')
        poses = read_matrix(a.poses,3)
    if len(ranges) != len(poses) or (odom is not None and len(odom) != len(ranges)):
        raise ValueError('Range/Pose/Odom row counts differ')
    if float(params['WeightO']) > 0 and odom is None:
        raise ValueError('WeightO > 0 requires an incremental Odom.txt; set WeightO 0 for pose-only input')
    times = read_matrix(a.timestamps,1).ravel() if a.timestamps else np.arange(len(ranges))*a.scan_period
    if len(times) != len(ranges) or np.any(np.diff(times) <= 0):
        raise ValueError('Timestamps must match scans and strictly increase')
    indices = np.arange(a.start,len(ranges),a.scan_stride)
    if a.max_scans: indices = indices[:a.max_scans]
    if len(indices) < 2: raise ValueError('Need at least two selected scans')
    selected = ranges[indices]
    valid = (selected > float(params['MinRange'])) & (selected < float(params['MaxRange']))
    if np.any(valid.sum(axis=1) < 2):
        raise ValueError('A selected scan has fewer than two valid returns; check range limits/config')
    # Selection is performed here; the batch C++ backend is single-resolution/full-grid.
    params.update(EvaluateGT='false',PosefromOdom='false',ModeOdom='true' if odom is not None else 'false',
                  ModeKeyFrame='false',ModeShowMap='false')
    out = a.output.expanduser().resolve()
    if out.exists() and any(out.iterdir()): raise ValueError(f'Output directory must be empty: {out}')
    out.mkdir(parents=True,exist_ok=True)
    with RunContext(out) as context:
        copy_input_config(a,out)
        (out/'source_backend_config.txt').write_text(a.backend_config.read_text())
        (out/'arguments.json').write_text(json.dumps({k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},indent=2))
        for name,values in (('Range.txt',selected),('Pose.txt',poses[indices]),('timestamps.txt',times[indices])):
            np.savetxt(out/name,values,fmt='%.17g')
        np.savetxt(out/'selected_indices.txt',indices,fmt='%d')
        if odom is not None: np.savetxt(out/'Odom.txt',selected_odometry(odom,indices),fmt='%.17g')
        run_backend(out,params,a,dict(input_mode='txt',pose_source=source,pose_convention='T_world_sensor',odometry_source='input increments' if odom is not None else None,timestamps_source='measured' if a.timestamps else 'synthetic',selected_indices='selected_indices.txt'))
        summary = dict(input_mode='txt',input_scans=len(ranges),accepted_scans=len(indices),pose_source=source,
                       timestamps_source=str(a.timestamps) if a.timestamps else 'synthetic frame index * scan_period; not measured time',
                       odometry_convention='first row absolute, later local SE2 increments; composed across selected frames',
                       optimized_directory=str(out/'optimized'))
        (out/'summary.json').write_text(json.dumps(summary,indent=2))
        print(f'Completed: {out}; open with tools/view_rviz.sh 2d --result {out}',flush=True)


if __name__ == '__main__':
    try:
        run(parse_config(parser(),path_keys=('dataset','ranges','poses','odom','timestamps','backend_config','backend')))
    except (OSError,ValueError,RuntimeError,subprocess.CalledProcessError) as exc:
        sys.exit(f'TXT pipeline: {exc}')
