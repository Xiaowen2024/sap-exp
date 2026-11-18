#!/usr/bin/env bash
# Exploratory screen on the dedicated RTX 4090 task directory.
set -u
TASK_ROOT=${TASK_ROOT:-/root/autodl-tmp/rp}
PY=${IPC_PY:-/root/autodl-tmp/genesis-runtime/bin/python-genesis}
OUT=${1:-$TASK_ROOT/results/genesis_ipc_placement}
cd "$TASK_ROOT" || exit 1
export PYTHONPATH="$TASK_ROOT/scripts"
export OMP_NUM_THREADS=1
SCENE=data/seg_scene_s9_v5.json
PB=data/task_0050_user_0016_scene_0009_cfg_0002_freeze.npz
run() {
    label=$1
    shift
    mkdir -p "$OUT/$label"
    "$PY" -m real_playback.rp_placement_ipc --scene "$SCENE" --playback "$PB" \
        --out "$OUT/$label" --label "$label" "$@" > "$OUT/$label/console.log" 2>&1
    code=$?
    echo "$code" > "$OUT/$label/exit_code.txt"
    echo "$label exit=$code"
}
run drive_strength1 --no-objects --duration 0.5 --params '{"d_hat":0.0001,"eps_v":0.001,"newton_tol":0.0001,"strength":1}'
run strict_native --duration 0.05
run strict_dh01_strength1 --duration 0.05 --params '{"d_hat":0.0001,"eps_v":0.001,"newton_tol":0.0001,"strength":1}'
run opened32_dh01_strength1 --extra-width-mm 32 --duration 0.25 --params '{"d_hat":0.0001,"eps_v":0.001,"newton_tol":0.0001,"strength":1}'
echo done > "$OUT/SCREEN_DONE"
