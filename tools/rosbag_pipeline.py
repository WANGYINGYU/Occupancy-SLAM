#!/usr/bin/env python3
"""Read ROS1/ROS2 bags, run ROS2 slam_toolbox, then the original C++ 2D optimizer."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import yaml
from scan_io import bag_inputs, stamp_ns, se2, pose2
from pipeline_config import parse_config, copy_input_config
from slam_frontend import run_slam
from backend_config import load_parameters
from backend_runner import run_backend
from run_context import RunContext
from project_paths import ROOT, backend_path



def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, help='YAML input settings; CLI overrides YAML')
    p.add_argument('--bag', type=Path, required=True)
    p.add_argument('--output', default='pipeline')
    p.add_argument('--scan-topic', default='auto')
    p.add_argument('--odom-source', choices=['auto', 'tf', 'odom', 'scan-match'], default='auto')
    p.add_argument('--odom-topic', default='/odom')
    p.add_argument('--tf-topic', default='/tf')
    p.add_argument('--tf-static-topic', default='/tf_static')
    p.add_argument('--odom-frame', default='odom')
    p.add_argument('--tf-max-gap', type=float, default=0.2)
    p.add_argument('--projection-frame', help='TF target frame for height slicing/reprojection (default: sensor frame)')
    p.add_argument('--min-height', type=float, default=-0.2)
    p.add_argument('--max-height', type=float, default=0.2)
    p.add_argument('--bins', type=int, default=720)
    p.add_argument('--min-range', type=float, default=0.1)
    p.add_argument('--max-range', type=float, default=30.)
    p.add_argument('--icp-distance', type=float, default=0.8)
    p.add_argument('--beam-stride', type=int, default=1)
    p.add_argument('--scan-stride', type=int, default=1)
    p.add_argument('--start', type=int, default=0)
    p.add_argument('--max-scans', type=int, default=0)
    p.add_argument('--resolution', type=float)
    p.add_argument('--iterations', type=int)
    p.add_argument('--odom-weight', type=float)
    p.add_argument('--backend', type=Path, default=backend_path('occupancy_slam_2d'))
    p.add_argument('--slam-config', type=Path, help='Optional slam_toolbox YAML; required frame/time settings are overridden')
    p.add_argument('--scan-timeout', type=float, default=3.)
    p.add_argument('--startup-timeout', type=float, default=30.)
    p.add_argument('--min-accepted-fraction', type=float, default=0.8)
    p.add_argument('--domain-id', type=int, default=87)
    p.add_argument('--backend-config',type=Path,default=ROOT/'config/backend/bag.txt')
    p.add_argument('--frontend-resolution',type=float)
    p.add_argument('--step-policy',choices=('legacy','decrease'),default='legacy')
    p.add_argument('--max-observations',type=int,default=20000000)
    p.add_argument('--display-resolution',type=float,default=.05)
    return p


def check_args(a):
    if min(a.bins, a.beam_stride, a.scan_stride) < 1 or a.bins < 2 or min(a.start, a.max_scans, a.iterations) < 0:
        raise ValueError('Invalid count/stride')
    for key, val in vars(a).items():
        if isinstance(val, float) and not math.isfinite(val):
            raise ValueError(f'Nonfinite argument: {key}')
    if a.min_height >= a.max_height or not 0 <= a.min_range < a.max_range or min(a.resolution, a.scan_timeout, a.startup_timeout, a.icp_distance, a.tf_max_gap) <= 0:
        raise ValueError('Invalid geometry/timeout parameters')
    if not 0 < a.min_accepted_fraction <= 1 or a.odom_weight < 0 or not 0 <= a.domain_id <= 232:
        raise ValueError('Invalid acceptance, odometry weight or ROS domain')
    topics = [a.scan_topic,a.odom_topic,a.tf_topic,a.tf_static_topic]
    if any(not t or any(c.isspace() for c in t) for t in topics):
        raise ValueError('Input topics must be nonempty names without whitespace')
    if len({t.lstrip('/') for t in topics}) != len(topics):
        raise ValueError('Scan, odometry, TF and static TF input topics must be distinct')
    if a.frontend_resolution is not None and a.frontend_resolution<=0: raise ValueError('frontend-resolution must be positive')
    if a.display_resolution<=0 or a.max_observations<1: raise ValueError('Invalid map resolution/observation budget')
    if not a.backend.is_file():
        raise ValueError(f'Build the C++ backend first: {a.backend}')




def main():
    args = parse_config(parser(), path_keys=('bag','backend','slam_config','backend_config'))
    backend_params=load_parameters(args.backend_config,dict(Scale=args.resolution,MaxIter=args.iterations,WeightO=args.odom_weight))
    args.resolution=float(backend_params['Scale']);args.iterations=int(backend_params['MaxIter']);args.odom_weight=float(backend_params['WeightO'])
    check_args(args)
    out = Path(args.output).expanduser().resolve()
    if out.exists() and any(out.iterdir()):
        raise ValueError(f'Output directory must be empty: {out}')
    out.mkdir(parents=True, exist_ok=True)
    with RunContext(out) as context:
        (out/'arguments.json').write_text(json.dumps({k: str(v) if isinstance(v, Path) else v for k,v in vars(args).items()}, indent=2))
        copy_input_config(args,out)
        print('Reading scans and preparing odometry prior...', flush=True)
        scans, priors, source, diagnostics = bag_inputs(args)
        (out/'arguments.json').write_text(json.dumps({k: str(v) if isinstance(v, Path) else v for k,v in vars(args).items()}, indent=2))
        print(f'{len(scans)} scans, input frame={scans[0].frame}, odometry source={source}', flush=True)
        np.savetxt(out/'input_odometry.txt', priors, fmt='%.12g')
        (out/'scan_odometry_diagnostics.json').write_text(json.dumps(diagnostics, indent=2))
        context.update(stage="slam_toolbox")
        accepted, poses = run_slam(scans, priors, args, out)
        scans = [scans[i] for i in accepted]
        np.savetxt(out/'Range.txt', np.array([np.where(np.isfinite(s.ranges), s.ranges, 0.) for s in scans]), fmt='%.9g')
        np.savetxt(out/'Pose.txt', poses, fmt='%.12g')
        with (out/'timestamps.txt').open('w') as f:
            for scan in scans:
                seconds, ns = divmod(scan.time, 1_000_000_000)
                f.write(f'{seconds}.{ns:09d}\n')
        # These relative constraints come from slam_toolbox, as requested. They are
        # correlated with its initialization, so WeightO is configurable and modest.
        increments = np.zeros_like(poses)
        increments[0] = poses[0]
        for i in range(1,len(poses)):
            increments[i] = pose2(np.linalg.inv(se2(poses[i-1])) @ se2(poses[i]))
        np.savetxt(out/'Odom.txt', increments, fmt='%.12g')
        s = scans[0]
        backend_params.update(NumBeam=str(len(s.ranges)),MinAngle=str(s.angle_min),MaxAngle=str(s.angle_min+(len(s.ranges)-1)*s.angle_increment),MinRange=str(s.range_min),MaxRange=str(s.range_max))
        run_backend(out,backend_params,args,dict(input_mode='bag',pose_source='slam_toolbox causal poses',pose_convention='T_map_scan',scan_frame=s.frame,odometry_source='relative increments of the same causal poses; correlated prior',frontend_prior='input_odometry.txt',frame_selection='scan_acceptance.json'))
        (out/'summary.json').write_text(json.dumps({'bag': str(Path(args.bag).resolve()), 'odometry_source': source, 'input_topics': args.resolved_input_topics,
            'input_scans': len(priors), 'accepted_scans': len(accepted), 'scan_frame': s.frame,
            'pose_semantics': 'slam_toolbox causal map-to-scan-frame poses; not final graph poses after later loop closures',
            'optimized_directory': str(out/'optimized')}, indent=2))
        print(f'Completed: {out / "optimized"}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, RuntimeError, OSError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
