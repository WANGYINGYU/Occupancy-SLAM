#!/usr/bin/env python3
"""Run a command with persistent outputs, logs and temporary files in a workspace."""
import argparse
import json
import os
from pathlib import Path
import re
import sys
import time
from workspace import require_workspace


def environment(run_dir, workspace):
    paths = {
        'TMPDIR': run_dir/'scratch/tmp',
        'TMP': run_dir/'scratch/tmp',
        'TEMP': run_dir/'scratch/tmp',
        'ROS_LOG_DIR': run_dir/'logs/ros',
        'ROS_HOME': run_dir/'scratch/ros_home',
        'XDG_CACHE_HOME': run_dir/'scratch/cache',
        'MPLCONFIGDIR': run_dir/'scratch/cache/matplotlib',
        'JOBLIB_TEMP_FOLDER': run_dir/'scratch/joblib',
        'NUMBA_CACHE_DIR': run_dir/'scratch/cache/numba',
        'PIP_CACHE_DIR': run_dir/'scratch/cache/pip',
        'PYTHONPYCACHEPREFIX': run_dir/'scratch/cache/pycache',
    }
    for path in paths.values():
        if not path.resolve().is_relative_to(workspace):
            raise RuntimeError(f'Generated path escapes the workspace: {path}')
        path.mkdir(parents=True, exist_ok=True)
    result = dict(os.environ)
    result.update({key: str(path) for key, path in paths.items()})
    result.update(OCCUPANCY_WORKSPACE=str(workspace), OCCUPANCY_RUN_DIR=str(run_dir),
                  PYTHONDONTWRITEBYTECODE='1')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, help='Output root (default: OCCUPANCY_WORKSPACE or ~/occupancy-slam-workspace)')
    parser.add_argument('--name', default=time.strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('a command after -- is required')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', args.name):
        parser.error('--name must be a single safe directory name')
    if args.workspace is not None:
        os.environ['OCCUPANCY_WORKSPACE'] = str(args.workspace.expanduser().resolve())
    workspace = require_workspace()
    run_dir = workspace/'runs'/args.name
    if not run_dir.resolve().is_relative_to(workspace):
        raise RuntimeError('Run path resolves outside the workspace')
    run_dir.mkdir(parents=True, exist_ok=False)
    env = environment(run_dir, workspace)
    (run_dir/'launcher.json').write_text(json.dumps(dict(
        command=command, cwd=str(run_dir), workspace=str(workspace),
        time=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        generated_paths={key:env[key] for key in ('TMPDIR','ROS_LOG_DIR','XDG_CACHE_HOME','PYTHONPYCACHEPREFIX')},
    ), indent=2)+'\n')
    os.chdir(run_dir)
    print(f'Run directory: {run_dir}', file=sys.stderr, flush=True)
    os.execvpe(command[0], command, env)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError) as exc:
        sys.exit(str(exc))
