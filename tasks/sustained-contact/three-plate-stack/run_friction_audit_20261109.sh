#!/usr/bin/env bash
set -euo pipefail
task_root=/root/SIM-EXP/audits/20261109
task_python=/root/autodl-tmp/venvs/mjwarp317-cu12/bin/python
export WARP_CACHE_PATH=/root/autodl-tmp/warp-cache-cu12
export PYTHONPATH=/root/sap-fix-gamma/src:$task_root/code
mkdir -p "$task_root/results/three-plate/sap" "$task_root/results/three-plate/mujoco"
"$task_python" /root/sap-fix-gamma/local_experiments/diagnostics/sap_preflight.py --compare-devices --json "$task_root/results/preflight.json"
"$task_python" "$task_root/code/run_threeplate_strict_v3_mjwarp.py" --manifest "$task_root/three_plate_stack_strict_v3p1.json" --out "$task_root/results/three-plate/mujoco"
"$task_python" /root/sap-fix-gamma/local_experiments/strict_v3/run_three_plate_strict_v3.py --manifest "$task_root/three_plate_stack_strict_v3p1.json" --simexp-scripts "$task_root/code" --preflight-json "$task_root/results/preflight.json" --out "$task_root/results/three-plate/sap"
touch "$task_root/results/three-plate/COMPLETE"
