#!/usr/bin/env python3
"""Held-out headline table: SAP vs MuJoCo Warp 4.0 vs Genesis Rigid vs Genesis IPC (preliminary)."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from compare_genesis_rigid import E, LOSS_NAME, TASKS, count, frozen, loss


def cell(p: Path, kind: str):
    if not p.exists():
        return "not run", ""
    d = json.loads(p.read_text())
    if d.get("complete") is False:
        n = len(d.get("heldout") or [])
        return (f"incomplete ({n} held-out cells)" if n else "incomplete"), frozen(d) if d.get("frozen_candidate") else ""
    return (count(d) if kind == "count" else loss(d)), frozen(d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rigid", type=Path, default=E / "genesis-rigid-20261114")
    ap.add_argument("--ipc", type=Path, default=E / "genesis-ipc-prelim-20261115")
    a = ap.parse_args()
    print("| Task | Held-out metric | SAP | MuJoCo Warp 4.0 | Genesis Rigid | Genesis IPC (prelim) | IPC frozen |")
    print("|---|---|---|---|---|---|---|")
    for tid, kind, sap, mj, gn in TASKS:
        if tid.startswith("pendulum"):
            continue
        gi = gn.replace("genesis_rigid_", "genesis_ipc_")
        s, m, r = (cell(p, kind)[0] for p in (E / sap, E / mj, a.rigid / gn))
        i, fi = cell(a.ipc / gi, kind)
        metric = "pass / valid-correct" if kind == "count" else "mean " + LOSS_NAME[tid]
        print(f"| {tid} | {metric} | {s} | {m} | {r} | {i} | `{fi}` |")


if __name__ == "__main__":
    main()
