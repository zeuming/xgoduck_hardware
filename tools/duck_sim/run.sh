#!/bin/sh
set -eu
duck_sim_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
duck_project_root=$(CDPATH= cd -- "$duck_sim_dir/../.." && pwd)
if [ -n "${DUCK_SIM_PYTHON:-}" ]; then
  duck_python=$DUCK_SIM_PYTHON
elif [ -x "$duck_sim_dir/.venv/bin/python" ]; then
  duck_python="$duck_sim_dir/.venv/bin/python"
elif command -v python3.12 >/dev/null 2>&1; then
  duck_python=$(command -v python3.12)
else
  duck_python=$(command -v python3)
fi
duck_extra="$duck_project_root/output/duck_sim_runtime"
if "$duck_python" -c 'import sys; sys.exit(sys.version_info[:2] != (3,12))'; then
  duck_extra="$duck_extra:$duck_project_root/output/x5_recovery/mac-model-test-runtime/lib/python3.12/site-packages"
fi
export PYTHONPATH="$duck_extra${PYTHONPATH:+:$PYTHONPATH}"
exec "$duck_python" "$duck_sim_dir/server.py" "$@"
