#!/bin/bash
# Genesis RigidSolver full run (registry protocol/manifests/genesis_rigid_rerun.json), three lanes on one GPU.
# Usage (server): setsid nohup bash run_genesis_rigid_queue.sh /root/genesis_rigid/results < /dev/null > /root/genesis_rigid/queue.out 2>&1 &
set -u
OUT=${1:-/root/genesis_rigid/results}
PY=${PY:-/root/autodl-tmp/genesis-runtime/bin/python-genesis}
ROOT=${SIMEXP_ROOT:-/root/SIM-EXP}
M=$ROOT/protocol/manifests
cd "$ROOT"
mkdir -p "$OUT"
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
run() {  # name, script, args...
  local name=$1; shift
  log "start $name"
  "$PY" "$@" --out "$OUT/$name" > "$OUT/$name.log" 2>&1
  log "$name exit=$?"
}

log "preflight"
"$PY" engines/genesis/genesis_rigid_preflight.py --out "$OUT/preflight.json" > "$OUT/preflight.log" 2>&1 || { log "PREFLIGHT FAILED; stop"; exit 1; }
log "preflight pass"

( run three-box tasks/static-friction/three-box-push/genesis/run_threebox_genesis_rigid.py --manifest $M/three_box_push_strict_v4.json
  run three-plate tasks/sustained-contact/three-plate-stack/genesis/run_threeplate_genesis_rigid.py --manifest $M/three_plate_stack_strict_v3p1.json
  run pendulum-v2 tasks/energy-conservation/pendulum/genesis/run_pendulum_v2_genesis_rigid.py --manifest $M/pendulum_energy_v2.json
  run pendulum-gauge-real tasks/energy-conservation/pendulum/genesis/run_gauge_pendulum_real_genesis_rigid.py --manifest $M/pendulum_gauge_real_v1.json ) &
( run simple-pinch tasks/static-friction/simple-pinch/genesis/run_pinch_genesis_rigid.py --manifest $M/simple_pinch_force_controlled_v3.json
  run gauge-slope tasks/dynamic-friction/slope-slider/genesis/run_slope_genesis_rigid.py --manifest $M/gauge_slope_slider_dynamic_friction_v3.json ) &
( run cube-toss tasks/dynamic-impact/genesis/run_impact_genesis_rigid.py --task cube --manifest $M/contactnets_cube_toss_dynamic_impact_v2.json
  run cube-tail tasks/sustained-contact/cube-toss-tail/genesis/run_tail_genesis_rigid.py --manifest $M/contactnets_cube_tail_sustained_v3.json \
      --cube-toss-artifact "$OUT/cube-toss/genesis_rigid_cube_dynamic_impact_v2.json"
  run ball-bounce-v2 tasks/dynamic-impact/genesis/run_impact_genesis_rigid.py --task ball --manifest $M/gauge_ball_bounce_dynamic_impact_v2.json
  run ball-bounce-v3 tasks/dynamic-impact/genesis/run_impact_genesis_rigid.py --task ball --manifest $M/gauge_ball_bounce_dynamic_impact_v3.json
  run mit-push tasks/dynamic-friction/mit-push/genesis/run_mitpush_genesis_rigid.py --manifest $M/mit_push_dynamic_friction_v3.json ) &
wait
log ALLDONE
