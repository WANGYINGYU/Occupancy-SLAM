#!/usr/bin/env bash
set -eo pipefail
project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -z "${SDF_BA_RUN_ROOT:-}" ]]; then
 exec python3 "$project_dir/tools/run_on_data.py" --name "occupancy_test_$(date +%Y%m%d_%H%M%S)_$$" -- bash "$project_dir/tools/test.sh" "$@"
fi
suite="${1:-unit}"; if [[ $# -gt 0 ]]; then shift; fi
case "$suite" in
 unit) python3 - "$project_dir" <<'INNER'
import os,sys,subprocess
from pathlib import Path
run=Path(os.environ['SDF_BA_RUN_ROOT'])
(run/'CTestTestfile.cmake').write_text('include("'+sys.argv[1]+'/build/CTestTestfile.cmake")\n')
raise SystemExit(subprocess.call(['ctest','--test-dir',str(run),'--output-on-failure','--output-junit',str(run/'ctest.xml')]))
INNER
 ;;
 inputs|integration) source "$project_dir/tools/env.sh"; if [[ "$suite" == inputs ]]; then script=test_inputs_2d.py; else script=integration.py; fi; exec python3 "$project_dir/tests/$script" "$@" ;;
 architecture) exec python3 "$project_dir/tests/test_architecture.py" "$@" ;;
 *) echo 'Usage: tools/test.sh {unit|inputs|integration|architecture}'; exit 2 ;;
esac
