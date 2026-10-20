#!/usr/bin/env bash
# Saved-result viewer. Local workstation runs automatically use the data disk.
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export OCCUPANCY_VIEWER_CALLER_DIR="${OCCUPANCY_VIEWER_CALLER_DIR:-$PWD}"
launcher="${OCCUPANCY_DATA_LAUNCHER:-$project_dir/tools/run_on_data.py}"
if [[ -z "${SDF_BA_RUN_ROOT:-}" && -f "$launcher" && " $* " != *" --help "* ]]; then
  exec python3 "$launcher" --name "occupancy_rviz_$(date +%Y%m%d_%H%M%S)_$$" -- bash "$project_dir/tools/view_rviz.sh" "$@"
fi
source "$project_dir/tools/env.sh"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$project_dir/config/fastdds_udp.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$project_dir/config/fastdds_udp.xml"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export QT_X11_NO_MITSHM=1
exec python3 "$project_dir/tools/rviz_viewer.py" "$@"
