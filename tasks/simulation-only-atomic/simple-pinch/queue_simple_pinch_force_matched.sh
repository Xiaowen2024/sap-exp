#!/usr/bin/env bash
# Wait for an otherwise idle GPU, then run the declared force-matched pair
# sequentially.  This never kills, suspends, or shares a user GPU process.
set -euo pipefail

ROOT=/root/SIM-EXP
RUNNERS=$ROOT/runners
OUT=$ROOT/exp-results/protocol-v2-native-default/aligned-generalizability-v2/simple-pinch
LOG=$OUT/queue.log
mkdir -p "$OUT"

idle_checks=0
while :; do
  active=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | sed '/^$/d' | wc -l | tr -d ' ')
  if [ "$active" -eq 0 ]; then
    idle_checks=$((idle_checks + 1))
  else
    idle_checks=0
  fi
  if [ "$idle_checks" -ge 2 ]; then
    break
  fi
  printf '%s waiting for an idle GPU (%s compute processes)\n' "$(date -Is)" "$active" >> "$LOG"
  sleep 60
done

printf '%s starting MuJoCo Warp Simple Pinch rerun\n' "$(date -Is)" >> "$LOG"
PYTHONPATH="$RUNNERS" /root/.venvs/mjwarp317/bin/python "$RUNNERS/run_simple_pinch_mjwarp_force_matched.py" \
  --manifest "$ROOT/tasks/simulation-only-atomic/simple-pinch/manifests/simple_pinch_force_matched_v1.json" \
  --source "$RUNNERS/run_static_friction_v2.py" \
  --output "$OUT/mujoco_warp" >> "$LOG" 2>&1

printf '%s starting SAP Warp Simple Pinch rerun\n' "$(date -Is)" >> "$LOG"
PYTHONPATH="/root/SAP-WARP-INTERNAL-db8d236/src:$RUNNERS" /root/.venvs/mjwarp317/bin/python "$RUNNERS/run_simple_pinch_sap_force_matched.py" \
  --manifest "$ROOT/tasks/simulation-only-atomic/simple-pinch/manifests/simple_pinch_force_matched_v1.json" \
  --private-runner /root/SAP-WARP-INTERNAL-db8d236/local_experiments/run_simple_pinch_b1_b5.py \
  --output "$OUT/sap_warp" >> "$LOG" 2>&1

printf '%s completed both engines\n' "$(date -Is)" >> "$LOG"
