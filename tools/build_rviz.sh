#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$project_dir/tools/env.sh"
install_dir="${OCCUPANCY_RVIZ_PREFIX:-$project_dir/build/rviz_install}"
cmake -S "$project_dir/ros2/occupancy_slam_rviz" -B "$project_dir/build/rviz_plugin" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$install_dir"
cmake --build "$project_dir/build/rviz_plugin" --parallel 2
cmake --install "$project_dir/build/rviz_plugin"
