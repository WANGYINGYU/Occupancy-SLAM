#!/usr/bin/env bash
# Load the system ROS2 installation (or an explicit alternate installation).
_occupancy_project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_occupancy_setup="${OCCUPANCY_RUNTIME_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"
if [[ ! -f "$_occupancy_setup" ]]; then
  echo "ROS2 setup not found: $_occupancy_setup. Install ROS2 Jazzy or set OCCUPANCY_RUNTIME_SETUP." >&2
  return 1
fi
source "$_occupancy_setup"
_occupancy_rviz="${OCCUPANCY_RVIZ_PREFIX:-$_occupancy_project/build/rviz_install}"
if [[ -f "$_occupancy_rviz/share/occupancy_slam_rviz/local_setup.bash" ]]; then
  source "$_occupancy_rviz/share/occupancy_slam_rviz/local_setup.bash"
fi
unset _occupancy_project _occupancy_setup _occupancy_rviz
