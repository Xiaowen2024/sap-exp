#!/bin/bash
# Relaunch after amendment a1 (two lanes; three-box and ball lanes of the first queue keep running).
set -u
OUT=${1:-/root/genesis_ipc/results}
PY=${PY:-/root/autodl-tmp/genesis-runtime/bin/python-genesis}
ROOT=${SIMEXP_ROOT:-/root/SIM-EXP}
M=$ROOT/protocol/manifests
cd "$ROOT"
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
run() { local name=$1; shift; log "start $name"; "$PY" -u "$@" --out "$OUT/$name" > "$OUT/$name.log" 2>&1; log "$name exit=$?"; }
( run cube-toss tasks/real-data-simple/shared/genesis/run_impact_genesis_ipc.py --task cube --manifest $M/contactnets_cube_toss_dynamic_impact_v2.json
  run cube-tail tasks/real-data-simple/cube-toss-tail/genesis/run_tail_genesis_ipc.py --manifest $M/contactnets_cube_tail_sustained_v3.json \
      --cube-toss-artifact "$OUT/cube-toss/genesis_ipc_cube_dynamic_impact_v2.json"
  run gauge-slope tasks/real-data-simple/slope-slider/genesis/run_slope_genesis_ipc.py --manifest $M/gauge_slope_slider_dynamic_friction_v3.json
  run mit-push tasks/real-data-simple/mit-push/genesis/run_mitpush_genesis_ipc.py --manifest $M/mit_push_dynamic_friction_v3.json ) &
( run simple-pinch tasks/simulation-only-atomic/simple-pinch/genesis/run_pinch_genesis_ipc.py --manifest $M/simple_pinch_force_controlled_v3.json
  run three-plate tasks/simulation-only-atomic/three-plate-stack/genesis/run_threeplate_genesis_ipc.py --manifest $M/three_plate_stack_strict_v3p1.json ) &
wait
log ALLDONE2
