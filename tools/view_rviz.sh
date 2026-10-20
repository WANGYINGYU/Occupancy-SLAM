#!/usr/bin/env bash
# Saved-result viewer. Outputs and caches use the configured workspace.
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export OCCUPANCY_VIEWER_CALLER_DIR="${OCCUPANCY_VIEWER_CALLER_DIR:-$PWD}"
launcher="${OCCUPANCY_RUNNER:-$project_dir/tools/run.py}"
if [[ -z "${OCCUPANCY_RUN_DIR:-}" && -f "$launcher" && " $* " != *" --help "* ]]; then
  exec python3 "$launcher" --name "occupancy_rviz_$(date +%Y%m%d_%H%M%S)_$$" -- bash "$project_dir/tools/view_rviz.sh" "$@"
fi
source "$project_dir/tools/env.sh"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$project_dir/config/fastdds_udp.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$project_dir/config/fastdds_udp.xml"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export QT_X11_NO_MITSHM=1
exec python3 "$project_dir/tools/rviz_viewer.py" "$@"
