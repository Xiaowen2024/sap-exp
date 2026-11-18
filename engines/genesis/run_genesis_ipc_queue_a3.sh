#!/bin/bash
# Genesis IPC a3 rerun (registry genesis-ipc-20261116-a3).  Three lanes (GPU shared with other jobs; CUDA OOM deaths are
# retried by the isolation layer, never scored).
# Ball bounce v2 is not run (a3); v3 is report-only.
#   setsid nohup bash run_genesis_ipc_queue_a3.sh /root/genesis_ipc/a3/results < /dev/null > /root/genesis_ipc/a3/queue.out 2>&1 &
set -u
OUT=${1:-/root/genesis_ipc/a3/results}
PY=${PY:-/root/autodl-tmp/genesis-runtime/bin/python-genesis}
ROOT=${SIMEXP_ROOT:-/root/SIM-EXP}
M=$ROOT/sim_protocol/manifests
mkdir -p "$OUT"
cd "$ROOT/scripts/genesis"
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
run() { local name=$1; shift; log "start $name"; "$PY" -u "$@" --out "$OUT/$name" > "$OUT/$name.log" 2>&1; log "$name exit=$?"; }
log "a3 queue start (registry $(grep -o '"revision": "genesis-ipc[^"]*"' $M/genesis_ipc_rerun.json | head -1))"
TOSS="$OUT/cube-toss/genesis_ipc_cube_dynamic_impact_v2.json"
( run three-plate run_threeplate_genesis_ipc.py --manifest $M/three_plate_stack_strict_v3p1.json
  until [ -f "$OUT/cube-toss.done" ]; do sleep 60; done
  run cube-tail run_tail_genesis_ipc.py --manifest $M/contactnets_cube_tail_sustained_v3.json --cube-toss-artifact "$TOSS" ) &
( run simple-pinch run_pinch_genesis_ipc.py --manifest $M/simple_pinch_force_controlled_v3.json
  run gauge-slope run_slope_genesis_ipc.py --manifest $M/gauge_slope_slider_dynamic_friction_v3.json
  run mit-push run_mitpush_genesis_ipc.py --manifest $M/mit_push_dynamic_friction_v3.json ) &
( run cube-toss run_impact_genesis_ipc.py --task cube --manifest $M/contactnets_cube_toss_dynamic_impact_v2.json
  touch "$OUT/cube-toss.done"
  run three-box run_threebox_genesis_ipc.py --manifest $M/three_box_push_strict_v4.json
  run ball-bounce-v3 run_impact_genesis_ipc.py --task ball --manifest $M/gauge_ball_bounce_dynamic_impact_v3.json ) &
wait
log ALLDONE
