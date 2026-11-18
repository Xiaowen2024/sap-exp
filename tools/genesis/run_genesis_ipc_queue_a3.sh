#!/bin/bash
# Genesis IPC a3 rerun (registry genesis-ipc-20261116-a3).  Three lanes (GPU shared with other jobs; CUDA OOM deaths are
# retried by the isolation layer, never scored).
# Ball bounce v2 is not run (a3); v3 is report-only.
#   setsid nohup bash run_genesis_ipc_queue_a3.sh /root/genesis_ipc/a3/results < /dev/null > /root/genesis_ipc/a3/queue.out 2>&1 &
set -u
OUT=${1:-/root/genesis_ipc/a3/results}
PY=${PY:-/root/autodl-tmp/genesis-runtime/bin/python-genesis}
ROOT=${SIMEXP_ROOT:-/root/SIM-EXP}
MROOT=$ROOT
mkdir -p "$OUT"
cd "$ROOT"
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
run() { local name=$1; shift; log "start $name"; "$PY" -u "$@" --out "$OUT/$name" > "$OUT/$name.log" 2>&1; log "$name exit=$?"; }
log "a3 queue start (registry $(grep -o '"revision": "genesis-ipc[^"]*"' $MROOT/protocol/registries/genesis_ipc_rerun.json | head -1))"
TOSS="$OUT/cube-toss/genesis_ipc_cube_dynamic_impact_v2.json"
( run three-plate tasks/simulation-only-atomic/three-plate-stack/genesis/run_threeplate_genesis_ipc.py --manifest $MROOT/tasks/simulation-only-atomic/three-plate-stack/manifests/three_plate_stack_strict_v3p1.json
  until [ -f "$OUT/cube-toss.done" ]; do sleep 60; done
  run cube-tail tasks/real-data-simple/cube-toss-tail/genesis/run_tail_genesis_ipc.py --manifest $MROOT/tasks/real-data-simple/cube-toss-tail/manifests/contactnets_cube_tail_sustained_v3.json --cube-toss-artifact "$TOSS" ) &
( run simple-pinch tasks/simulation-only-atomic/simple-pinch/genesis/run_pinch_genesis_ipc.py --manifest $MROOT/tasks/simulation-only-atomic/simple-pinch/manifests/simple_pinch_force_controlled_v3.json
  run gauge-slope tasks/real-data-simple/slope-slider/genesis/run_slope_genesis_ipc.py --manifest $MROOT/tasks/real-data-simple/slope-slider/manifests/gauge_slope_slider_dynamic_friction_v3.json
  run mit-push tasks/real-data-simple/mit-push/genesis/run_mitpush_genesis_ipc.py --manifest $MROOT/tasks/real-data-simple/mit-push/manifests/mit_push_dynamic_friction_v3.json ) &
( run cube-toss common/run_impact_genesis_ipc.py --task cube --manifest $MROOT/tasks/real-data-simple/cube-toss/manifests/contactnets_cube_toss_dynamic_impact_v2.json
  touch "$OUT/cube-toss.done"
  run three-box tasks/simulation-only-atomic/three-box-push/genesis/run_threebox_genesis_ipc.py --manifest $MROOT/tasks/simulation-only-atomic/three-box-push/manifests/three_box_push_strict_v4.json
  run ball-bounce-v3 common/run_impact_genesis_ipc.py --task ball --manifest $MROOT/tasks/real-data-simple/ball-bounce/manifests/gauge_ball_bounce_dynamic_impact_v3.json ) &
wait
log ALLDONE
