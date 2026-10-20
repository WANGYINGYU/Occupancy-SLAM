#!/usr/bin/env bash
# Reopen the measured Cartographer / NC20 examples retained on this workstation.
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
data_root="${OCCUPANCY_DEMO_ROOT:-/media/yingyuwang/Data/experiments/sdf_ba/codex_runs}"
case "${1:-}" in
  2d)
    result="$data_root/occupancy_dual_map_2d_20261019_01/pipeline"
    initial="$result/optimized/initial"
    domain=88 ;;
  3d)
    result="$data_root/occupancy_dual_map_3d_20261019_01/optimized"
    initial="$result/initial"
    domain=89 ;;
  *) echo "Usage: $0 {2d|3d} [viewer options, e.g. --method initial]"; exit 2 ;;
esac
if [[ ! -d "$result" || ! -d "$initial" ]]; then
  echo 'Saved demo results are missing; use view_rviz.sh --result with your output directory.' >&2
  exit 1
fi
mode="$1"
shift
exec bash "$project_dir/tools/view_rviz.sh" "$mode" --result "$result" --initial "$initial" --domain-id "$domain" "$@"
