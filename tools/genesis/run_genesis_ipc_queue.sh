#!/bin/bash
# Genesis + IPC preliminary run (registry protocol/registries/genesis_ipc_rerun.json), five lanes on one GPU.
# Usage (server): setsid nohup bash run_genesis_ipc_queue.sh /root/genesis_ipc/results < /dev/null > /root/genesis_ipc/queue.out 2>&1 &
set -u
OUT=${1:-/root/genesis_ipc/results}
PY=${PY:-/root/autodl-tmp/genesis-runtime/bin/python-genesis}
ROOT=${SIMEXP_ROOT:-/root/SIM-EXP}
MROOT=$ROOT
cd "$ROOT"
mkdir -p "$OUT"
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
run() {  # name, script, args...
  local name=$1; shift
  log "start $name"
  "$PY" -u "$@" --out "$OUT/$name" > "$OUT/$name.log" 2>&1
  log "$name exit=$?"
}
log "preflight"
"$PY" -u tools/genesis/genesis_ipc_preflight.py --out "$OUT/preflight.json" > "$OUT/preflight.log" 2>&1 || { log "PREFLIGHT FAILED; stop"; exit 1; }
log "preflight pass"
( run three-plate tasks/simulation-only-atomic/three-plate-stack/genesis/run_threeplate_genesis_ipc.py --manifest $MROOT/tasks/simulation-only-atomic/three-plate-stack/manifests/three_plate_stack_strict_v3p1.json ) &
( run three-box tasks/simulation-only-atomic/three-box-push/genesis/run_threebox_genesis_ipc.py --manifest $MROOT/tasks/simulation-only-atomic/three-box-push/manifests/three_box_push_strict_v4.json ) &
( run simple-pinch tasks/simulation-only-atomic/simple-pinch/genesis/run_pinch_genesis_ipc.py --manifest $MROOT/tasks/simulation-only-atomic/simple-pinch/manifests/simple_pinch_force_controlled_v3.json ) &
( run cube-toss common/run_impact_genesis_ipc.py --task cube --manifest $MROOT/tasks/real-data-simple/cube-toss/manifests/contactnets_cube_toss_dynamic_impact_v2.json
  run cube-tail tasks/real-data-simple/cube-toss-tail/genesis/run_tail_genesis_ipc.py --manifest $MROOT/tasks/real-data-simple/cube-toss-tail/manifests/contactnets_cube_tail_sustained_v3.json \
      --cube-toss-artifact "$OUT/cube-toss/genesis_ipc_cube_dynamic_impact_v2.json"
  run ball-bounce-v2 common/run_impact_genesis_ipc.py --task ball --manifest $MROOT/tasks/real-data-simple/ball-bounce/manifests/gauge_ball_bounce_dynamic_impact_v2.json
  run ball-bounce-v3 common/run_impact_genesis_ipc.py --task ball --manifest $MROOT/tasks/real-data-simple/ball-bounce/manifests/gauge_ball_bounce_dynamic_impact_v3.json ) &
( run gauge-slope tasks/real-data-simple/slope-slider/genesis/run_slope_genesis_ipc.py --manifest $MROOT/tasks/real-data-simple/slope-slider/manifests/gauge_slope_slider_dynamic_friction_v3.json
  run mit-push tasks/real-data-simple/mit-push/genesis/run_mitpush_genesis_ipc.py --manifest $MROOT/tasks/real-data-simple/mit-push/manifests/mit_push_dynamic_friction_v3.json ) &
wait
log ALLDONE
