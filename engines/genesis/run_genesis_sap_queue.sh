#!/bin/bash
# Run only after preflight passed. Serial tasks avoid the other jobs already using the shared GPU.
set -eu
OUT=${1:-/root/genesis_sap/a1}
PY=${PY:-/root/autodl-tmp/genesis-runtime/bin/python-genesis}
ROOT=${SIMEXP_ROOT:-/root/SIM-EXP}
M=$ROOT/protocol/manifests
cd "$ROOT"
mkdir -p "$OUT"
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
run() {
 local name=$1; shift
 log "start $name"
 if "$PY" -u "$@" --out "$OUT/$name" > "$OUT/$name.log" 2>&1; then
  log "$name exit=0"
 else
  local rc=$?; log "$name exit=$rc; queue stopped"; exit "$rc"
 fi
}
"$PY" -c 'import json,sys;sys.exit(0 if json.load(open(sys.argv[1]))["all_pass"] else 1)' "$OUT/preflight.json"
run three-box tasks/static-friction/three-box-push/genesis/run_threebox_genesis_sap.py --manifest "$M/three_box_push_strict_v4.json"
run three-plate tasks/sustained-contact/three-plate-stack/genesis/run_threeplate_genesis_sap.py --manifest "$M/three_plate_stack_strict_v3p1.json"
run simple-pinch tasks/static-friction/simple-pinch/genesis/run_pinch_genesis_sap.py --manifest "$M/simple_pinch_force_controlled_v3.json"
run cube-toss tasks/dynamic-impact/genesis/run_impact_genesis_sap.py --task cube --manifest "$M/contactnets_cube_toss_dynamic_impact_v2.json"
run cube-tail tasks/sustained-contact/cube-toss-tail/genesis/run_tail_genesis_sap.py --manifest "$M/contactnets_cube_tail_sustained_v3.json" --cube-toss-artifact "$OUT/cube-toss/genesis_sap_cube_dynamic_impact_v2.json"
run gauge-slope tasks/dynamic-friction/slope-slider/genesis/run_slope_genesis_sap.py --manifest "$M/gauge_slope_slider_dynamic_friction_v3.json"
run mit-push tasks/dynamic-friction/mit-push/genesis/run_mitpush_genesis_sap.py --manifest "$M/mit_push_dynamic_friction_v3.json"
log ALLDONE
