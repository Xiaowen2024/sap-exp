#!/bin/bash
# Ball bounce v2 rerun with the decimated IPC ball mesh; waits for the three-box lane (pid $2) to free GPU memory.
set -u
OUT=${1:-/root/genesis_ipc/results}; WAITPID=${2:-0}
PY=/root/autodl-tmp/genesis-runtime/bin/python-genesis; M=/root/SIM-EXP/protocol/manifests
cd /root/SIM-EXP
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
while kill -0 "$WAITPID" 2>/dev/null; do sleep 60; done
log "start ball-bounce-v2"
"$PY" -u tasks/dynamic-impact/genesis/run_impact_genesis_ipc.py --task ball --manifest $M/gauge_ball_bounce_dynamic_impact_v2.json --out "$OUT/ball-bounce-v2" > "$OUT/ball-bounce-v2.log" 2>&1
log "ball-bounce-v2 exit=$?"
log ALLDONE3
