#!/usr/bin/env python3
"""Launch experiments with data-disk working, output, temporary and cache paths.

This routes normal relative outputs and temp APIs. It cannot override explicit
hardcoded paths or container filesystems; audit those before replaying old runs.
"""
import argparse
import json
import os
from pathlib import Path
import re
import sys
import time

DATA = Path(os.environ.get('OCCUPANCY_DATA_ROOT','/media/yingyuwang/Data'))
STORAGE = DATA / 'experiments/sdf_ba'


def require_data_disk():
    if not DATA.is_mount() or DATA.stat().st_dev == DATA.parent.stat().st_dev:
        raise RuntimeError(f'Data disk is not mounted: {DATA}; no SSD fallback')
    if os.statvfs(DATA).f_flag & os.ST_RDONLY:
        raise RuntimeError(f'Data disk is read-only: {DATA}')
    if not STORAGE.resolve().is_relative_to(DATA.resolve()):
        raise RuntimeError('Experiment storage resolves outside the data disk')


def environment(run_dir):
    paths = {
        'TMPDIR': run_dir / 'scratch/tmp',
        'TMP': run_dir / 'scratch/tmp',
        'TEMP': run_dir / 'scratch/tmp',
        'ROS_LOG_DIR': run_dir / 'logs/ros',
        'ROS_HOME': run_dir / 'scratch/ros_home',
        'XDG_CACHE_HOME': run_dir / 'scratch/cache',
        'MPLCONFIGDIR': run_dir / 'scratch/cache/matplotlib',
        'JOBLIB_TEMP_FOLDER': run_dir / 'scratch/joblib',
        'NUMBA_CACHE_DIR': run_dir / 'scratch/cache/numba',
        'PIP_CACHE_DIR': run_dir / 'scratch/cache/pip',
        'CUDA_CACHE_PATH': run_dir / 'scratch/cache/cuda',
        'TORCH_EXTENSIONS_DIR': run_dir / 'scratch/cache/torch_extensions',
        'PYTHONPYCACHEPREFIX': run_dir / 'scratch/cache/pycache',
    }
    for path in paths.values():
        if not path.resolve().is_relative_to(DATA.resolve()):
            raise RuntimeError(f'Generated path escapes data disk: {path}')
        path.mkdir(parents=True, exist_ok=True)
    result = dict(os.environ)
    result.update({key: str(path) for key, path in paths.items()})
    result.update(SDF_BA_STORAGE_ROOT=str(STORAGE), SDF_BA_RUN_ROOT=str(run_dir),
                  SDF_BA_OUTPUT_DIR=str(run_dir), PYTHONDONTWRITEBYTECODE='1')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', default=time.strftime('%Y%m%d_%H%M%S') + f'_{os.getpid()}')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('a command after -- is required')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', args.name):
        parser.error('--name must be a single safe directory name')
    require_data_disk()
    run_dir = STORAGE / 'codex_runs' / args.name
    if not run_dir.resolve().is_relative_to(DATA.resolve()):
        raise RuntimeError('Run path resolves outside the data disk')
    run_dir.mkdir(parents=True, exist_ok=False)
    env = environment(run_dir)
    (run_dir / 'launcher.json').write_text(json.dumps(dict(
        command=command, cwd=str(run_dir),
        storage_root=str(STORAGE), time=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        generated_paths={key: env[key] for key in (
            'TMPDIR', 'ROS_LOG_DIR', 'XDG_CACHE_HOME', 'PYTHONPYCACHEPREFIX')},
    ), indent=2) + '\n')
    os.chdir(run_dir)
    print(f'Experiment storage: {run_dir}', file=sys.stderr, flush=True)
    os.execvpe(command[0], command, env)


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError) as exc:
        sys.exit(str(exc))
