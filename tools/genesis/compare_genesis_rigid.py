#!/usr/bin/env python3
"""Held-out headline table: SAP vs MuJoCo Warp 4.0 vs Genesis RigidSolver (same metric, same denominators)."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import math
from pathlib import Path

ROOT = REPO_ROOT
E = ROOT / "exp-results"
TASKS = [  # id, kind, SAP, MuJoCo Warp 4.0, Genesis (relative to the Genesis results dir)
    ("three-plate", "count", "contact-correction-20261109/three-plate/sap/sap_three_plate_strict_v3.json",
     "mujoco-warp-4p0/three-plate/mjwarp_three_plate_strict_v3.json", "three-plate/genesis_rigid_three_plate_strict_v3.json"),
    ("three-box", "count", "aligned-generalizability-strict-v3/v4/sap_three_box_strict_v4.json",
     "mujoco-warp-4p0/three-box/mjwarp_three_box_strict_v4.json", "three-box/genesis_rigid_three_box_strict_v4.json"),
    ("simple-pinch", "count", "simple-pinch-v3/sap_simple_pinch_v3_inertia_fixed.json",
     "mujoco-warp-4p0/simple-pinch/mjwarp_simple_pinch_v3.json", "simple-pinch/genesis_rigid_simple_pinch_v3.json"),
    ("cube-tail", "count", "cube-tail-v3/sap_cube_tail_v3.json",
     "mujoco-warp-4p0/cube-tail-v3/mjwarp_cube_tail_v3.json", "cube-tail/genesis_rigid_cube_tail_v3.json"),
    ("cube-toss", "loss", "dynamic-impact-v2/sap_cube_dynamic_impact_v2.json",
     "mujoco-warp-4p0/cube-toss/mjwarp_cube_dynamic_impact_v2.json", "cube-toss/genesis_rigid_cube_dynamic_impact_v2.json"),
    ("ball-bounce-v2", "loss", "dynamic-impact-v2/sap_ball_dynamic_impact_v2.json",
     "mujoco-warp-4p0/bounce-v2/mjwarp_ball_dynamic_impact_v2.json", "ball-bounce-v2/genesis_rigid_ball_dynamic_impact_v2.json"),
    ("ball-bounce-v3", "loss", "dynamic-impact-v3/sap_ball_dynamic_impact_v3.json",
     "mujoco-warp-4p0/bounce-v3/mjwarp_ball_dynamic_impact_v3.json", "ball-bounce-v3/genesis_rigid_ball_dynamic_impact_v3.json"),
    ("gauge-slope", "loss", "gauge-slope-v3/sap_gauge_slope_v3.json",
     "mujoco-warp-4p0/gauge-slope/mjwarp_gauge_slope_v3.json", "gauge-slope/genesis_rigid_gauge_slope_v3.json"),
    ("mit-push", "loss", "mit-push-v3/sap_mit_push_v3.json",
     "mujoco-warp-4p0/mit-push/mjwarp_mit_push_v3.json", "mit-push/genesis_rigid_mit_push_v3.json"),
    ("pendulum-v2", "loss", "pendulum-v2/sap_pendulum_v2.json",
     "mujoco-warp-4p0/pendulum/mjwarp_pendulum_v2.json", "pendulum-v2/genesis_rigid_pendulum_v2.json"),
    ("pendulum-gauge-real", "loss", "gauge-real-317-400-20261110/sap/pendulum/sap_gauge_pendulum_real.json",
     "gauge-real-317-400-20261110/400/pendulum/mjwarp_gauge_pendulum_real.json",
     "pendulum-gauge-real/genesis_rigid_gauge_pendulum_real.json"),
]
LOSS_NAME = {"cube-toss": "pos RMSE+0.05·orient", "ball-bounce-v2": "RMSE+0.05·apex", "ball-bounce-v3": "RMSE+0.05·apex",
             "gauge-slope": "traj RMSE (m)", "mit-push": "trans+0.05·yaw", "pendulum-v2": "max rel. energy drift",
             "pendulum-gauge-real": "PD-ratio err + EL err"}


def count(d):
    held = d.get("heldout") or []
    k = sum(r.get("pass_count", r.get("agree_and_valid_count", 0)) for r in held)
    n = sum(r.get("n", r.get("R", 20)) for r in held)
    return f"{k}/{n}"


def loss(d):
    held = d.get("heldout") or []
    vals = [r.get("loss") for r in held]
    ok = [v for v in vals if v is not None and math.isfinite(v)]
    if not held:
        return "not run"
    s = f"{sum(ok) / len(ok):.4g}" if ok else "—"
    return s + (f" ({len(ok)}/{len(vals)} cells valid)" if len(ok) < len(vals) else "")


def frozen(d):
    c = d.get("frozen_candidate")
    if isinstance(c, dict):
        return c.get("text") or c.get("id")
    return str(c)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--genesis", type=Path, default=E / "genesis-rigid-20261114")
    a = ap.parse_args()
    print("| Task | Held-out metric | SAP | MuJoCo Warp 4.0 | Genesis Rigid | Genesis frozen |")
    print("|---|---|---|---|---|---|")
    for tid, kind, sap, mj, gn in TASKS:
        cells = []
        for p in (E / sap, E / mj, a.genesis / gn):
            if not p.exists():
                cells.append(("missing", "")); continue
            d = json.loads(p.read_text())
            if d.get("complete") is False:
                cells.append(("incomplete", frozen(d))); continue
            cells.append((count(d) if kind == "count" else loss(d), frozen(d)))
        metric = "pass / valid-correct" if kind == "count" else "mean " + LOSS_NAME[tid]
        print(f"| {tid} | {metric} | {cells[0][0]} | {cells[1][0]} | {cells[2][0]} | `{cells[2][1]}` |")


if __name__ == "__main__":
    main()
