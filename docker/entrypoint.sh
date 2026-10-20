#!/usr/bin/env bash
set -eo pipefail
source /opt/occupancy-slam/tools/env.sh
case "${1:---help}" in
  exec) shift; exec "$@" ;;
  test) shift; exec bash /opt/occupancy-slam/tools/test.sh "$@" ;;
  *) exec bash /opt/occupancy-slam/tools/occupancy-slam "$@" ;;
esac
