#!/bin/bash
# Re-run, at most PAR at a time, the socket-v8 IPC shards whose process died while the GPU was oversubscribed
# (18 processes, 31/32 GB).  Crashed shards are moved to crashed/ (kept).  A shard that crashes again when run with
# low concurrency counts as an IPC failure.  Candidate 1 (native d_hat 10 mm > clearance: start pose has no solution) is
# deterministic and not re-run.
set -u
OUT=${1:-/root/genesis_ipc/socket_v8/run}; PAR=${PAR:-3}
PY=/root/autodl-tmp/genesis-runtime/bin/python-genesis; M=/root/SIM-EXP/protocol/manifests/round_socket_strict_v8_draft.json
SUB="R_c0p3_x1p50,R_c1p0_x1p10,R_c0p1_x1p10,R_c0p3_x0p85,R_c1p0_x2p20,I_c0p3_rim_x2p20_g64,I_c0p3_wedgehold_g8,I_c1p0_wedgehold_g64,I_c0p1_wedgehold_g8,I_c0p3_wedgeslide_m0.1,E_c0p3_out_x1p5,E_c0p1_in_tilt0p5fe,E_c0p3_in_off0p5c,W_c0p3_shallow_slide1,W_c0p3_shallow_hold2,C_c0p3_free_centred"
cd /root/SIM-EXP; mkdir -p $OUT/logs/rerun
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
crashed() { "$PY" - "$1" 2>/dev/null <<'P'
import json, sys
r = json.load(open(sys.argv[1])); f = (r.get("ipc") or {}).get("failed") or ""
print("yes" if f and "cannot convert float NaN" not in f else "no")
P
}
move_crashed() {  # $1 = phase dir glob
  for f in $1; do [ -f "$f" ] || continue; if [ "$(crashed $f)" = yes ]; then d=$OUT/crashed/${f#$OUT/shards/}; mkdir -p $(dirname $d); mv $f $d; echo $f; fi; done
}
log "rerun of crashed shards start (PAR=$PAR, RTX 3080 Ti idle)"
JOBS=$(move_crashed "$OUT/shards/selection/cand[2-5]/*.json" | sed -E 's|.*/cand([0-9])/(.*)\.json|\1 \2|')
log "selection shards re-run: $(echo $JOBS | tr '\n' ' ')"
echo "$JOBS" | grep . | xargs -P $PAR -L 1 sh -c "\"$PY\" -u tasks/simulation-only-composite/round-socket/genesis/run_socket_v8_genesis_ipc.py --manifest $M --out $OUT --phase selection --cand \$0 --cells \$1 > $OUT/logs/rerun/sel_\$0_\$1.log 2>&1"
"$PY" -u tasks/simulation-only-composite/round-socket/genesis/run_socket_v8_genesis_ipc.py --manifest $M --out $OUT --merge > $OUT/logs/rerun/merge_selection.log 2>&1
W=$(grep -o 'frozen candidate [0-9]*' $OUT/logs/rerun/merge_selection.log | head -1 | awk '{print $3}')
log "after rerun: frozen candidate $W"
move_crashed "$OUT/shards/frozen_B5/cand$W/*.json" > /dev/null
TODO=$(for c in $(echo $SUB | tr ',' ' '); do [ -f $OUT/shards/frozen_B5/cand$W/$c.json ] || echo $c; done)
log "frozen_B5 cand$W shards to run: $(echo $TODO | tr '\n' ' ')"
echo "$TODO" | grep . | xargs -P $PAR -I{} sh -c "\"$PY\" -u tasks/simulation-only-composite/round-socket/genesis/run_socket_v8_genesis_ipc.py --manifest $M --out $OUT --phase frozen_B5 --cells {} > $OUT/logs/rerun/B5_{}.log 2>&1"
"$PY" -u tasks/simulation-only-composite/round-socket/genesis/run_socket_v8_genesis_ipc.py --manifest $M --out $OUT --merge > $OUT/logs/rerun/merge.log 2>&1
log "rerun done exit=$?"
