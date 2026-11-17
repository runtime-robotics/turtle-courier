#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export PYTHONPATH="/app${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=1
exec python3 /app/run.py "$@"
