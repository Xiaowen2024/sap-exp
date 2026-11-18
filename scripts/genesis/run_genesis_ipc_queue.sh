#!/bin/bash
# Genesis + IPC preliminary run (registry sim_protocol/manifests/genesis_ipc_rerun.json), five lanes on one GPU.
# Usage (server): setsid nohup bash run_genesis_ipc_queue.sh /root/genesis_ipc/results < /dev/null > /root/genesis_ipc/queue.out 2>&1 &
set -u
OUT=${1:-/root/genesis_ipc/results}
PY=${PY:-/root/autodl-tmp/genesis-runtime/bin/python-genesis}
ROOT=${SIMEXP_ROOT:-/root/SIM-EXP}
M=$ROOT/sim_protocol/manifests
cd "$ROOT/scripts/genesis"
mkdir -p "$OUT"
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
run() {  # name, script, args...
  local name=$1; shift
  log "start $name"
  "$PY" -u "$@" --out "$OUT/$name" > "$OUT/$name.log" 2>&1
  log "$name exit=$?"
}
log "preflight"
"$PY" -u genesis_ipc_preflight.py --out "$OUT/preflight.json" > "$OUT/preflight.log" 2>&1 || { log "PREFLIGHT FAILED; stop"; exit 1; }
log "preflight pass"
( run three-plate run_threeplate_genesis_ipc.py --manifest $M/three_plate_stack_strict_v3p1.json ) &
( run three-box run_threebox_genesis_ipc.py --manifest $M/three_box_push_strict_v4.json ) &
( run simple-pinch run_pinch_genesis_ipc.py --manifest $M/simple_pinch_force_controlled_v3.json ) &
( run cube-toss run_impact_genesis_ipc.py --task cube --manifest $M/contactnets_cube_toss_dynamic_impact_v2.json
  run cube-tail run_tail_genesis_ipc.py --manifest $M/contactnets_cube_tail_sustained_v3.json \
      --cube-toss-artifact "$OUT/cube-toss/genesis_ipc_cube_dynamic_impact_v2.json"
  run ball-bounce-v2 run_impact_genesis_ipc.py --task ball --manifest $M/gauge_ball_bounce_dynamic_impact_v2.json
  run ball-bounce-v3 run_impact_genesis_ipc.py --task ball --manifest $M/gauge_ball_bounce_dynamic_impact_v3.json ) &
( run gauge-slope run_slope_genesis_ipc.py --manifest $M/gauge_slope_slider_dynamic_friction_v3.json
  run mit-push run_mitpush_genesis_ipc.py --manifest $M/mit_push_dynamic_friction_v3.json ) &
wait
log ALLDONE
