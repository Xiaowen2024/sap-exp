#!/usr/bin/env bash
# Run only after Simple Pinch has completed and the shared GPU is idle.
set -euo pipefail
ROOT=/root/SIM-EXP
RUNNERS=$ROOT/runners
OUT=$ROOT/exp-results/protocol-v2-native-default/aligned-generalizability-v2/round-socket
LOG=$OUT/queue.log
SIMPLE=$ROOT/exp-results/protocol-v2-native-default/aligned-generalizability-v2/simple-pinch/sap_warp/summary.json
mkdir -p "$OUT"

while :; do
  if [ -f "$SIMPLE" ] && grep -q '"complete": true' "$SIMPLE"; then
    active=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | sed '/^$/d' | wc -l | tr -d ' ')
    if [ "$active" -eq 0 ]; then
      break
    fi
  fi
  printf '%s waiting for completed Simple Pinch and an idle GPU\n' "$(date -Is)" >> "$LOG"
  sleep 60
done

printf '%s starting MuJoCo Warp Round Socket strict-v2\n' "$(date -Is)" >> "$LOG"
PYTHONPATH="$RUNNERS" /root/.venvs/mjwarp317/bin/python "$RUNNERS/run_round_socket_mjwarp_strict.py" \
  --manifest "$ROOT/sim_protocol/manifests/round_socket_strict_v2.json" \
  --source "$RUNNERS/run_round_socket_b1_b5_mjwarp.py" \
  --output "$OUT/mujoco_warp" >> "$LOG" 2>&1

printf '%s starting SAP Warp Round Socket strict-v2\n' "$(date -Is)" >> "$LOG"
PYTHONPATH="/root/SAP-WARP-INTERNAL-db8d236/src:$RUNNERS" /root/.venvs/mjwarp317/bin/python "$RUNNERS/run_round_socket_sap_strict.py" \
  --manifest "$ROOT/sim_protocol/manifests/round_socket_strict_v2.json" \
  --private-runner /root/SAP-WARP-INTERNAL-db8d236/local_experiments/run_round_socket_b1b5.py \
  --output "$OUT/sap_warp" >> "$LOG" 2>&1

printf '%s completed both engines\n' "$(date -Is)" >> "$LOG"
