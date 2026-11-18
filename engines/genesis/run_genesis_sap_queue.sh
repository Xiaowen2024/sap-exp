#!/bin/bash
# Run only after preflight passed. Serial tasks avoid the other jobs already using the shared GPU.
set -eu
OUT=${1:-/root/genesis_sap/a1}
PY=${PY:-/root/autodl-tmp/genesis-runtime/bin/python-genesis}
ROOT=${SIMEXP_ROOT:-/root/SIM-EXP}
M=$ROOT/sim_protocol/manifests
cd "$ROOT/scripts/genesis"
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
run three-box run_threebox_genesis_sap.py --manifest "$M/three_box_push_strict_v4.json"
run three-plate run_threeplate_genesis_sap.py --manifest "$M/three_plate_stack_strict_v3p1.json"
run simple-pinch run_pinch_genesis_sap.py --manifest "$M/simple_pinch_force_controlled_v3.json"
run cube-toss run_impact_genesis_sap.py --task cube --manifest "$M/contactnets_cube_toss_dynamic_impact_v2.json"
run cube-tail run_tail_genesis_sap.py --manifest "$M/contactnets_cube_tail_sustained_v3.json" --cube-toss-artifact "$OUT/cube-toss/genesis_sap_cube_dynamic_impact_v2.json"
run gauge-slope run_slope_genesis_sap.py --manifest "$M/gauge_slope_slider_dynamic_friction_v3.json"
run mit-push run_mitpush_genesis_sap.py --manifest "$M/mit_push_dynamic_friction_v3.json"
log ALLDONE
