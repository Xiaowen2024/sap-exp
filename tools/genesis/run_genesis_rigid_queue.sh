#!/bin/bash
# Genesis RigidSolver full run (registry protocol/registries/genesis_rigid_rerun.json), three lanes on one GPU.
# Usage (server): setsid nohup bash run_genesis_rigid_queue.sh /root/genesis_rigid/results < /dev/null > /root/genesis_rigid/queue.out 2>&1 &
set -u
OUT=${1:-/root/genesis_rigid/results}
PY=${PY:-/root/autodl-tmp/genesis-runtime/bin/python-genesis}
ROOT=${SIMEXP_ROOT:-/root/SIM-EXP}
MROOT=$ROOT
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
"$PY" tools/genesis/genesis_rigid_preflight.py --out "$OUT/preflight.json" > "$OUT/preflight.log" 2>&1 || { log "PREFLIGHT FAILED; stop"; exit 1; }
log "preflight pass"

( run three-box tasks/simulation-only-atomic/three-box-push/genesis/run_threebox_genesis_rigid.py --manifest $MROOT/tasks/simulation-only-atomic/three-box-push/manifests/three_box_push_strict_v4.json
  run three-plate tasks/simulation-only-atomic/three-plate-stack/genesis/run_threeplate_genesis_rigid.py --manifest $MROOT/tasks/simulation-only-atomic/three-plate-stack/manifests/three_plate_stack_strict_v3p1.json
  run pendulum-v2 tasks/simulation-only-atomic/pendulum-ideal/genesis/run_pendulum_v2_genesis_rigid.py --manifest $MROOT/tasks/simulation-only-atomic/pendulum-ideal/manifests/pendulum_energy_v2.json
  run pendulum-gauge-real tasks/real-data-simple/gauge-pendulum/genesis/run_gauge_pendulum_real_genesis_rigid.py --manifest $MROOT/tasks/real-data-simple/gauge-pendulum/manifests/pendulum_gauge_real_v1.json ) &
( run simple-pinch tasks/simulation-only-atomic/simple-pinch/genesis/run_pinch_genesis_rigid.py --manifest $MROOT/tasks/simulation-only-atomic/simple-pinch/manifests/simple_pinch_force_controlled_v3.json
  run gauge-slope tasks/real-data-simple/slope-slider/genesis/run_slope_genesis_rigid.py --manifest $MROOT/tasks/real-data-simple/slope-slider/manifests/gauge_slope_slider_dynamic_friction_v3.json ) &
( run cube-toss common/run_impact_genesis_rigid.py --task cube --manifest $MROOT/tasks/real-data-simple/cube-toss/manifests/contactnets_cube_toss_dynamic_impact_v2.json
  run cube-tail tasks/real-data-simple/cube-toss-tail/genesis/run_tail_genesis_rigid.py --manifest $MROOT/tasks/real-data-simple/cube-toss-tail/manifests/contactnets_cube_tail_sustained_v3.json \
      --cube-toss-artifact "$OUT/cube-toss/genesis_rigid_cube_dynamic_impact_v2.json"
  run ball-bounce-v2 common/run_impact_genesis_rigid.py --task ball --manifest $MROOT/tasks/real-data-simple/ball-bounce/manifests/gauge_ball_bounce_dynamic_impact_v2.json
  run ball-bounce-v3 common/run_impact_genesis_rigid.py --task ball --manifest $MROOT/tasks/real-data-simple/ball-bounce/manifests/gauge_ball_bounce_dynamic_impact_v3.json
  run mit-push tasks/real-data-simple/mit-push/genesis/run_mitpush_genesis_rigid.py --manifest $MROOT/tasks/real-data-simple/mit-push/manifests/mit_push_dynamic_friction_v3.json ) &
wait
log ALLDONE
