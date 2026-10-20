#!/usr/bin/env python3
"""Build/run Occupancy-SLAM containers with a configurable output workspace."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
IMAGE_ROOT = Path('/opt/occupancy-slam')
INPUT_OPTIONS = {
    '--dataset', '--bag', '--poses', '--ranges', '--odom', '--timestamps',
    '--manifest', '--config', '--backend-config', '--slam-config', '--backend',
    '--result', '--initial', '--source', '--converted', '--samples', '--results',
    '--maps', '--ground-truth', '--indices',
}


def parser():
    p = argparse.ArgumentParser(description=__doc__, epilog=(
        'Wrapper options precede the command. Example: tools/docker.sh --gui view 2d '
        '--result /absolute/pipeline. Paths on the CLI are mounted automatically; '
        'use --mount-input for additional paths referenced inside YAML/manifests.'))
    p.add_argument('--image', default=os.environ.get('OCCUPANCY_DOCKER_IMAGE', 'occupancy-slam:jazzy'))
    p.add_argument('--run-name', help='Fresh experiment directory name')
    p.add_argument('--mount-input', type=Path, action='append', default=[])
    p.add_argument('--gui', action='store_true', help='Forward this Linux X11/XWayland display using Mesa software rendering')
    p.add_argument('command', choices=('build', 'txt', 'bag', 'pcd', 'view', 'export', 'inspect', 'test', 'exec'))
    p.add_argument('arguments', nargs=argparse.REMAINDER)
    return p


def absolute(path, caller):
    p = Path(path).expanduser()
    return (p if p.is_absolute() else caller / p).resolve()


def bind(source, target=None, readonly=False):
    source = str(source); target = str(target or source)
    if ',' in source or ',' in target:
        raise ValueError('Docker bind paths containing commas are unsupported')
    return ['--mount', f'type=bind,src={source},dst={target}' + (',readonly' if readonly else '')]


def prepare_arguments(arguments, caller, storage, run):
    args = list(arguments); inputs = []
    for i, item in enumerate(args):
        key, sep, value = item.partition('=')
        if key not in INPUT_OPTIONS and key != '--output':
            continue
        if not sep:
            if i + 1 >= len(args):
                raise ValueError(f'Missing value for {key}')
            value = args[i+1]
        if key == '--output':
            if not absolute(value, run).is_relative_to(storage):
                raise ValueError('Output must be inside OCCUPANCY_WORKSPACE')
            continue
        path = absolute(value, caller)
        if path.is_relative_to(IMAGE_ROOT):
            continue  # A path to the image's own config/tools, not a host input.
        if not path.exists():
            raise ValueError(f'Input does not exist: {path}')
        inputs.append(path if path.is_dir() else path.parent)
        if sep: args[i] = key + '=' + str(path)
        else: args[i+1] = str(path)
    return args, inputs


def execute(command, run, name):
    (run/(name + '_command.json')).write_text(json.dumps(command, indent=2))
    print(f'{name} output: {run}', flush=True)
    started = time.monotonic()
    with (run/(name + '.log')).open('w') as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            for line in process.stdout:
                sys.stdout.write(line); sys.stdout.flush(); log.write(line); log.flush()
            code = process.wait()
        except KeyboardInterrupt:
            process.send_signal(signal.SIGINT)
            try: code = process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.terminate(); code = process.wait(timeout=10)
    (run/(name + '_status.json')).write_text(json.dumps(dict(exit_code=code, wall_seconds=time.monotonic()-started), indent=2))
    return code


def main():
    a = parser().parse_args()
    caller = Path(os.environ.get('OCCUPANCY_DOCKER_CALLER_DIR', os.getcwd())).resolve()
    # Preserve the caller before the launcher's chdir. Never write outputs in the checkout.
    if not os.environ.get('OCCUPANCY_RUN_DIR'):
        env = dict(os.environ, OCCUPANCY_DOCKER_CALLER_DIR=str(caller))
        launcher = Path(os.environ.get('OCCUPANCY_RUNNER', str(ROOT/'tools/run.py')))
        name = a.run_name or f'occupancy_docker_{a.command}_{time.strftime("%Y%m%d_%H%M%S")}_{os.getpid()}'
        command = [sys.executable, str(launcher), '--name', name, '--', sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]]
        os.execvpe(command[0], command, env)
    from workspace import require_workspace, current_run
    storage = require_workspace(); run = current_run()
    if storage.is_relative_to(IMAGE_ROOT) or IMAGE_ROOT.is_relative_to(storage):
        raise ValueError('Workspace must not overlap the container application directory')
    if a.command == 'build':
        return execute(['docker', 'build', '--progress=plain', '--tag', a.image, *a.arguments, str(ROOT)], run, 'build')
    args, inputs = prepare_arguments(a.arguments, caller, storage, run)
    if a.command == 'inspect' and args and args[0] not in ('--help', '-h'):
        bag = absolute(args[0], caller)
        if not bag.exists(): raise ValueError(f'Bag does not exist: {bag}')
        args[0] = str(bag); inputs.append(bag if bag.is_dir() else bag.parent)
    inputs += [absolute(p, caller) for p in a.mount_input]
    # A bundled YAML can reference sibling backend/slam profiles.
    config_root = ROOT/'config'
    if any(path.is_relative_to(config_root) for path in inputs): inputs.append(config_root)
    inputs = sorted(set(inputs), key=lambda p: len(p.parts))
    mounts = []
    for path in inputs:
        if not path.exists(): raise ValueError(f'Mount input does not exist: {path}')
        if path == Path('/') or path.is_relative_to(IMAGE_ROOT):
            raise ValueError(f'Refusing input mount over system/image paths: {path}')
        if any(path.is_relative_to(p) for p in mounts): continue
        if run.is_relative_to(path):
            raise ValueError(f'Input mount would cover this run output; use a narrower directory: {path}')
        mounts.append(path)
    command = ['docker', 'run', '--rm', '--init', '--read-only', '--network', 'host',
               '--user', f'{os.getuid()}:{os.getgid()}', '--cap-drop', 'ALL',
               '--security-opt', 'no-new-privileges', '--workdir', str(run)]
    for group in sorted(set(os.getgroups()) - {os.getgid()}):
        command += ['--group-add', str(group)]
    command += bind(storage)
    for path in mounts: command += bind(path, readonly=True)
    for name, target in [('tmp', '/tmp'), ('var_tmp', '/var/tmp'), ('shm', '/dev/shm')]:
        folder = run/'scratch'/name; folder.mkdir(parents=True, exist_ok=True)
        command += bind(folder, target)
    # This is the container user's actual writable home, backed by this run.
    # RViz uses ~/.rviz2 even when XDG_CONFIG_HOME is set. Host HOME is untouched.
    container_home = run/'scratch/user_home'
    container_home.mkdir(parents=True, exist_ok=True)
    command += bind(container_home, '/run/occupancy/home')
    command += ['--env', 'HOME=/run/occupancy/home']
    for name in ('XDG_CONFIG_HOME', 'XDG_RUNTIME_DIR'):
        folder = run/'scratch'/name.lower(); folder.mkdir(parents=True, exist_ok=True, mode=0o700)
        command += ['--env', f'{name}={folder}']
    keys = ['OCCUPANCY_RUN_DIR', 'OCCUPANCY_WORKSPACE', 'TMPDIR', 'TMP', 'TEMP',
            'ROS_LOG_DIR', 'ROS_HOME', 'XDG_CACHE_HOME', 'MPLCONFIGDIR', 'JOBLIB_TEMP_FOLDER',
            'NUMBA_CACHE_DIR', 'PIP_CACHE_DIR', 'PYTHONPYCACHEPREFIX']
    for key in keys:
        if key in os.environ: command += ['--env', f'{key}={os.environ[key]}']
    for key in ('OCCUPANCY_INPUT_CALLER_DIR', 'OCCUPANCY_VIEWER_CALLER_DIR'):
        command += ['--env', f'{key}={caller}']
    command += ['--env', 'PYTHONDONTWRITEBYTECODE=1']
    if a.gui:
        display = os.environ.get('DISPLAY')
        if not display: raise ValueError('--gui requires a local X11/XWayland DISPLAY')
        # FamilyLocal Xauthority entries are keyed by the host name.
        command += ['--hostname', socket.gethostname()]
        command += bind('/tmp/.X11-unix', readonly=True)
        command += ['--env', f'DISPLAY={display}', '--env', 'QT_QPA_PLATFORM=xcb', '--env', 'LIBGL_ALWAYS_SOFTWARE=1']
        authority = Path(os.environ.get('XAUTHORITY', str(Path.home()/'.Xauthority')))
        if authority.is_file():
            command += bind(authority, '/run/occupancy/Xauthority', readonly=True)
            command += ['--env', 'XAUTHORITY=/run/occupancy/Xauthority']
    elif a.command == 'view' and '--no-rviz' not in args and '--help' not in args:
        raise ValueError('Use --gui before view for RViz, or --no-rviz to publish topics only')
    image_info = subprocess.check_output(['docker', 'image', 'inspect', a.image, '--format', '{{.Id}}'], text=True).strip()
    (run/'image.json').write_text(json.dumps(dict(image=a.image, id=image_info, input_mounts=[str(p) for p in mounts]), indent=2))
    command += [a.image, a.command, *args]
    return execute(command, run, 'container')


if __name__ == '__main__':
    try: sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        sys.exit(f'Docker: {exc}')
