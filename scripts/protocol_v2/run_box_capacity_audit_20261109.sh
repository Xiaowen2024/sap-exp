#!/usr/bin/env bash
set -euo pipefail
task_root=/root/SIM-EXP/audits/20261109
task_python=/root/autodl-tmp/venvs/mjwarp317-cu12/bin/python
export WARP_CACHE_PATH=/root/autodl-tmp/warp-cache-cu12
export PYTHONPATH=$task_root/box-code
mkdir -p "$task_root/results/three-box/mujoco"
"$task_python" -u "$task_root/box-code/run_threebox_strict_v4_mjwarp.py" --manifest "$task_root/three_box_push_strict_v4.json" --out "$task_root/results/three-box/mujoco"
touch "$task_root/results/three-box/COMPLETE"
