#!/usr/bin/env python3
"""Configured PCD + absolute sensor odometry entry to the 3D C++ backend."""
import argparse,json,subprocess,sys
from pathlib import Path
from pipeline_config import parse_config,copy_input_config
from project_paths import ROOT,backend_path

def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path)
    for key in ('dataset','poses','manifest'):p.add_argument('--'+key,type=Path)
    p.add_argument('--backend',type=Path,default=backend_path('occupancy_slam_3d'))
    p.add_argument('--output',type=Path,default=Path('optimized'))
    for key,value in dict(start=0,stride=1,max_frames=0,point_stride=1,free_samples=4,iterations=10,max_vertices=2000000,max_observations=20000000,linear_iterations=500,check_jacobians=0).items():
        p.add_argument('--'+key.replace('_','-'),type=int,default=value)
    for key,value in dict(voxel=.5,point_voxel=.3,min_range=.3,max_range=60.,max_time_diff=.05,smooth=.01,odom_translation_weight=5.,odom_rotation_weight=10.,linear_tolerance=1e-6).items():
        p.add_argument('--'+key.replace('_','-'),type=float,default=value)
    return p

def main():
    a=parse_config(parser(),path_keys=('dataset','poses','manifest','backend'))
    if a.dataset is None and (a.poses is None or a.manifest is None): raise ValueError('Supply --dataset, or both --poses and --manifest')
    out=a.output.expanduser().resolve()
    if out.exists() and any(out.iterdir()):raise ValueError(f'Output directory must be empty: {out}')
    # C++ owns the directory until complete. Wrapper metadata is written afterwards,
    # including on failure, so partial outputs carry an explicit state.
    command=[str(a.backend),'--output',str(out)]
    for key,value in vars(a).items():
        if key not in ('config','backend','output') and value is not None:command+=['--'+key.replace('_','-'),str(value)]
    status='running';error=None
    try:
        out.parent.mkdir(parents=True,exist_ok=True)
        with Path(str(out)+'.backend.log').open('w') as log: subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
        status='completed'
    except BaseException as exc:
        status='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed';error=str(exc);raise
    finally:
        out.mkdir(parents=True,exist_ok=True);copy_input_config(a,out)
        (out/'arguments.json').write_text(json.dumps({k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()},indent=2))
        (out/'backend_command.json').write_text(json.dumps(command,indent=2))
        (out/'run_status.json').write_text(json.dumps(dict(status=status,error=error),indent=2))
        (out/'input_manifest.json').write_text(json.dumps(dict(schema_version=1,mode='3d',pose_convention='T_world_sensor',pose_source='absolute input odometry',odometry_source='relative increments of input poses',selected_scans='selected_scans.txt'),indent=2))
    print(f'Completed: {out}')
if __name__=='__main__':
    try:main()
    except (OSError,ValueError,RuntimeError,subprocess.CalledProcessError) as exc:sys.exit(str(exc))
