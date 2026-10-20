#!/usr/bin/env python3
"""Install optional ROS runtime in the current data-disk run, without sudo."""
import os
from pathlib import Path
import subprocess
import sys

root = Path(os.environ['SDF_BA_RUN_ROOT'])
os.chdir(root)
subprocess.run([sys.executable, '-m', 'venv', '--system-site-packages', str(root/'venv')], check=True)
subprocess.run([str(root/'venv/bin/python'), '-m', 'pip', 'install', 'rosbags>=0.10,<0.12'], check=True)
packages = ['libceres4t64', 'ros-jazzy-bond', 'ros-jazzy-bondcpp', 'ros-jazzy-smclib', 'ros-jazzy-slam-toolbox']
subprocess.run(['apt-get', 'download', *packages], check=True)
prefix = root/'sysroot'
for package in sorted(root.glob('*.deb')):
    subprocess.run(['dpkg-deb', '-x', str(package), str(prefix)], check=True)
(root/'setup.bash').write_text(f'''source /opt/ros/jazzy/setup.bash
export AMENT_PREFIX_PATH="{prefix}/opt/ros/jazzy:$AMENT_PREFIX_PATH"
export LD_LIBRARY_PATH="{prefix}/opt/ros/jazzy/lib:{prefix}/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
export PYTHONPATH="{prefix}/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"
export PATH="{root}/venv/bin:$PATH"
''')
print('Runtime environment:', root/'setup.bash')
