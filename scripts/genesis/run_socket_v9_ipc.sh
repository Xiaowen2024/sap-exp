#!/bin/bash
# Round socket strict-v9-draft, Genesis IPC: full run plan (selection 15, frozen B5 61, robustness 24, frozen B1 61), cell-parallel.
# Order after selection: B5 first, then robustness, then B1 (native default, expected not meaningful), so the main result
# arrives first. PAR=3 on the 12-GB RTX 3080 Ti (the v8 pilot re-run showed no crash at 3; 18 processes did crash).
set -u
OUT=${1:-/root/genesis_ipc/socket_v9/run}; PAR=${PAR:-3}
PY=/root/autodl-tmp/genesis-runtime/bin/python-genesis; M=/root/SIM-EXP/sim_protocol/manifests/round_socket_strict_v9_draft.json
R="$PY -u run_socket_v8_genesis_ipc.py --manifest $M --out $OUT"
mkdir -p $OUT/logs; cd /root/SIM-EXP/scripts/genesis
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
sha256sum $M > $OUT/manifest.sha256; md5sum run_socket_v8_genesis_ipc.py genesis_ipc_common.py /root/SIM-EXP/runners/socket_v8_common.py > $OUT/code.md5
log "socket v9 IPC start (PAR=$PAR)"
SEL="W_c0p3_shallow_slide2 W_c0p3_shallow_hold1 R_c0p3_x1p10"
for k in 1 2 3 4 5; do for c in $SEL; do echo "$k $c"; done; done | \
  xargs -P $PAR -L 1 sh -c "$R --phase selection --cand \$0 --cells \$1 > $OUT/logs/sel_cand\$0_\$1.log 2>&1"
log "selection done"
$R --merge > $OUT/logs/merge_selection.log 2>&1
log "frozen: $(grep -o 'frozen candidate [0-9]*.*' $OUT/logs/merge_selection.log | head -1)"
{ for ph in frozen_B5 robust_dt_0.25ms robust_dt_1ms robust_repeat frozen_B1; do
    $R --phase $ph --list 2>/dev/null | awk '{print $1, $3}'
  done; } > $OUT/queue.txt
log "queue: $(wc -l < $OUT/queue.txt) shards"
xargs -P $PAR -L 1 sh -c "$R --phase \$0 --cells \$1 > $OUT/logs/\$0_\$1.log 2>&1" < $OUT/queue.txt
$R --merge > $OUT/logs/merge.log 2>&1
log "socket v9 IPC done exit=$? (merged)"
