#!/usr/bin/env bash
set -eo pipefail
source /opt/occupancy-slam/tools/env.sh
case "${1:---help}" in
  exec) shift; exec "$@" ;;
  test)
    if [[ ! -f /opt/occupancy-slam/tests/run.sh ]]; then
      echo "Tests are not included in the runtime image. Build with --build-arg BUILD_TESTING=ON; see docs/VALIDATION.md." >&2
      exit 2
    fi
    shift; exec bash /opt/occupancy-slam/tests/run.sh "$@" ;;
  *) exec bash /opt/occupancy-slam/tools/occupancy-slam "$@" ;;
esac
