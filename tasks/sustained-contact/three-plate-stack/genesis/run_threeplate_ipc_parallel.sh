#!/bin/bash
# Three-plate a3, cell-parallel on one large GPU: 5 selection shards at once, freeze by the manifest rule, then the 12
# held-out cells (at most $PAR at once, longest first), then merge into the standard artifact.
set -u
OUT=${1:-/root/genesis_ipc/a3/results}; PAR=${PAR:-8}
PY=/root/autodl-tmp/genesis-runtime/bin/python-genesis; ROOT=/root/SIM-EXP; M=$ROOT/protocol/manifests/three_plate_stack_strict_v3p1.json
T=$OUT/three-plate; mkdir -p $T/shards $T/logs; cd $ROOT
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
log "three-plate parallel start (RTX 4080 32 GB; 5 selection shards)"
for k in 1 2 3 4 5; do "$PY" -u tasks/sustained-contact/three-plate-stack/genesis/run_threeplate_genesis_ipc.py --manifest $M --out $T --select-index $k > $T/logs/select_$k.log 2>&1 & done
wait
for k in 1 2 3 4 5; do [ -f $T/shards/select_$k.json ] || { log "three-plate select $k missing; abort"; exit 1; }; done
W=$("$PY" -c "
import json, sys; sys.path.insert(0, '$ROOT/runners')
from strict_v3_common import plate_choose
rows = [json.load(open('$T/shards/select_%d.json' % i)) for i in range(1, 6)]
print(plate_choose(rows) + 1)" 2>/dev/null | tail -1)
log "three-plate selection done; frozen index $W"
CELLS="dt_0p25 thickness_20 thickness_5 thickness_0p5 mass_0p667 mass_0p167 radius_125 radius_75 offset_0p5 offset_1p0 dt_0p5 dt_1p0"
echo $CELLS | tr " " "\n" | xargs -P $PAR -I{} sh -c "\"$PY\" -u tasks/sustained-contact/three-plate-stack/genesis/run_threeplate_genesis_ipc.py --manifest $M --out $T --heldout-cells {} --frozen-index $W > $T/logs/held_{}.log 2>&1"
"$PY" -u tasks/sustained-contact/three-plate-stack/genesis/run_threeplate_genesis_ipc.py --manifest $M --out $T --merge > $T/logs/merge.log 2>&1
log "three-plate exit=$? (merged)"
