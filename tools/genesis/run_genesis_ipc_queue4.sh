#!/bin/bash
# Ball bounce v2 and v3 with the 162-vertex farthest-point hull (IPC track representation change a2), two lanes.
set -u
OUT=${1:-/root/genesis_ipc/results}
PY=/root/autodl-tmp/genesis-runtime/bin/python-genesis; MROOT=/root/SIM-EXP
cd /root/SIM-EXP
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
run() { local name=$1; shift; log "start $name"; "$PY" -u "$@" --out "$OUT/$name" > "$OUT/$name.log" 2>&1; log "$name exit=$?"; }
( run ball-bounce-v2 common/run_impact_genesis_ipc.py --task ball --manifest $MROOT/tasks/real-data-simple/ball-bounce/manifests/gauge_ball_bounce_dynamic_impact_v2.json ) &
( run ball-bounce-v3 common/run_impact_genesis_ipc.py --task ball --manifest $MROOT/tasks/real-data-simple/ball-bounce/manifests/gauge_ball_bounce_dynamic_impact_v3.json ) &
wait
log ALLDONE4
