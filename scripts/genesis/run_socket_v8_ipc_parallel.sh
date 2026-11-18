#!/bin/bash
# Round-socket v8 (DRAFT) Genesis-IPC pilot, cell-parallel.  Time-boxed plan (2026-11-16, fixed before any IPC socket
# result): full selection (5 candidates x 3 conditions); frozen matrix B5 and B1 on a 16-condition labelled subset;
# no robustness phase.  Subset is in $OUT/plan.json.
set -u
OUT=${1:-/root/genesis_ipc/socket_v8/run}; PAR=${PAR:-10}
PY=/root/autodl-tmp/genesis-runtime/bin/python-genesis; M=/root/SIM-EXP/sim_protocol/manifests/round_socket_strict_v8_draft.json
SUB="R_c0p3_x1p50,R_c1p0_x1p10,R_c0p1_x1p10,R_c0p3_x0p85,R_c1p0_x2p20,I_c0p3_rim_x2p20_g64,I_c0p3_wedgehold_g8,I_c1p0_wedgehold_g64,I_c0p1_wedgehold_g8,I_c0p3_wedgeslide_m0.1,E_c0p3_out_x1p5,E_c0p1_in_tilt0p5fe,E_c0p3_in_off0p5c,W_c0p3_shallow_slide1,W_c0p3_shallow_hold2,C_c0p3_free_centred"
mkdir -p $OUT/logs; cd /root/SIM-EXP/scripts/genesis
log() { echo "$(date -Is) $*" >> "$OUT/status.txt"; }
cat > $OUT/plan.json <<J
{"written": "$(date -Is)", "manifest": "round_socket_strict_v8_draft.json (DRAFT; pilot like the SAP / MuJoCo 4.0 v8 pilots)",
 "selection": "all 3 selection conditions x 5 candidates", "frozen_B5_subset": "$SUB", "frozen_B1_subset": "same",
 "robustness": "not run (time box)", "reason": "IPC ~0.5 s/step at R=8 (6000 steps per condition); full matrix would take >100 GPU-hours",
 "subset_rule": "chosen from the SAP / MuJoCo 4.0 v8 pilot verdicts before any IPC socket result: every labelled family, conditions where the two engines differ, and one or two where both are correct"}
J
log "socket v8 IPC pilot start"
SEL="W_c0p3_shallow_slide2 W_c0p3_shallow_hold1 R_c0p3_x1p10"
"$PY" -u run_socket_v8_genesis_ipc.py --manifest $M --out $OUT --phase selection --cand 1 > $OUT/logs/sel_cand1.log 2>&1 &
for k in 2 3 4 5; do for c in $SEL; do
  "$PY" -u run_socket_v8_genesis_ipc.py --manifest $M --out $OUT --phase selection --cand $k --cells $c > $OUT/logs/sel_cand${k}_$c.log 2>&1 &
done; done
wait
log "socket selection done"
"$PY" -u run_socket_v8_genesis_ipc.py --manifest $M --out $OUT --merge > $OUT/logs/merge_selection.log 2>&1
log "socket frozen: $(grep -o 'frozen candidate [0-9]*.*' $OUT/logs/merge_selection.log | head -1)"
"$PY" -u run_socket_v8_genesis_ipc.py --manifest $M --out $OUT --phase frozen_B1 --cells $SUB > $OUT/logs/frozen_B1.log 2>&1 &
echo $SUB | tr "," "\n" | xargs -P $PAR -I{} sh -c "\"$PY\" -u run_socket_v8_genesis_ipc.py --manifest $M --out $OUT --phase frozen_B5 --cells {} > $OUT/logs/frozen_B5_{}.log 2>&1"
wait
"$PY" -u run_socket_v8_genesis_ipc.py --manifest $M --out $OUT --merge > $OUT/logs/merge.log 2>&1
log "socket v8 IPC pilot exit=$? (merged)"
