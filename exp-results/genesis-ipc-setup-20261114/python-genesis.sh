#!/usr/bin/env bash
set -eu
rt=/root/autodl-tmp/genesis-runtime
venv=/root/autodl-tmp/venvs/genesis146-ipc029-py312
mkdir -p /dev/shm/genesis-workspaces /dev/shm/genesis-cache /root/genesis146-extra/triton-cache
export PYTHONDONTWRITEBYTECODE=1 TMPDIR=/dev/shm/genesis-workspaces XDG_CACHE_HOME=/dev/shm/genesis-cache GS_CACHE_FILE_PATH=/dev/shm/genesis-cache/genesis QD_OFFLINE_CACHE_FILE_PATH=/dev/shm/genesis-cache/quadrants TRITON_CACHE_DIR=/root/genesis146-extra/triton-cache
exec "$rt/glibc235/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2" --library-path "$rt/glibc235/lib/x86_64-linux-gnu:$rt/glibc235/usr/lib/x86_64-linux-gnu:$rt/cxx/lib:/root/genesis146-cublas/nvidia/cublas/lib:$venv/lib" "$venv/bin/python" "$@"
