#!/usr/bin/env bash
# Source inside the data-disk launcher so ROS logs and Python caches follow it.
_occupancy_project="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "${OCCUPANCY_RUNTIME_SETUP:-$_occupancy_project/.runtime/setup.bash}" ]]; then
  source "${OCCUPANCY_RUNTIME_SETUP:-$_occupancy_project/.runtime/setup.bash}"
else
  source /opt/ros/jazzy/setup.bash
fi
_occupancy_rviz="${OCCUPANCY_RVIZ_PREFIX:-$_occupancy_project/build/rviz_install}"
if [[ -f "$_occupancy_rviz/share/occupancy_slam_rviz/local_setup.bash" ]]; then
  source "$_occupancy_rviz/share/occupancy_slam_rviz/local_setup.bash"
fi
unset _occupancy_project _occupancy_rviz
