#!/usr/bin/env python3
"""Audit: how much of the cube-toss final-orientation error is cube symmetry?

Replays the 32 held-out ContactNets trajectories (IDs 1, 3, ..., 63) with the
frozen MuJoCo Warp B=5 setting (= literal MuJoCo default, Euler) on CPU
MuJoCo 3.17, deterministically (no perturbation), and reports the raw
quaternion angle next to the symmetry-reduced angle and the effect on the
published composite loss (position RMSE + 0.05 * final orientation error).

This is a metric audit on the CPU reference engine, not a replacement for the
MuJoCo Warp / SAP Warp R=20 artifacts, which must be regenerated with final
quaternions saved.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import json
import sys
from pathlib import Path

import numpy as np

ROOT = REPO_ROOT
sys.path.insert(0, str(ROOT / "common"))
from cube_orientation_metrics import _self_test, cube_symmetric_qangle, raw_qangle  # noqa: E402
from run_dynamic_impact_cube_budget import SRC, make_model, simulate  # noqa: E402

HELDOUT = list(range(1, 64, 2))
OUT = ROOT / "exp-results" / "dynamic-impact-v1" / "cube-toss" / "orientation-symmetry-audit.json"


def main() -> None:
    _self_test()
    data = np.load(SRC, allow_pickle=True)
    states, lengths = data["state"], data["lengths"]
    model = make_model({"id": "default", "kind": "default", "integrator": "Euler"})
    rows = []
    for i in HELDOUT:
        L = int(lengths[i]); real = states[i, :L]
        sim, _, _ = simulate(model, real, L)
        pos_rmse = float(np.sqrt(np.mean(np.sum((sim[:, :3] - real[:, :3])**2, axis=1))))
        rows.append({
            "sample": i,
            "position_rmse_m": pos_rmse,
            "final_orientation_raw_rad": raw_qangle(sim[-1, 3:7], real[-1, 3:7]),
            "final_orientation_sym_rad": cube_symmetric_qangle(sim[-1, 3:7], real[-1, 3:7]),
        })
    raw = np.array([r["final_orientation_raw_rad"] for r in rows])
    sym = np.array([r["final_orientation_sym_rad"] for r in rows])
    pos = np.array([r["position_rmse_m"] for r in rows])
    summary = {
        "engine": "MuJoCo 3.17 CPU (reference for metric audit)",
        "config": "literal default, Euler (frozen MuJoCo Warp B=5)",
        "heldout_samples": len(rows),
        "mean_position_rmse_m": float(pos.mean()),
        "mean_final_orientation_raw_rad": float(raw.mean()),
        "mean_final_orientation_sym_rad": float(sym.mean()),
        "samples_where_raw_exceeds_sym_by_gt_0p1rad": int(np.sum(raw - sym > 0.1)),
        "composite_loss_raw": float(pos.mean() + 0.05 * raw.mean()),
        "composite_loss_sym": float(pos.mean() + 0.05 * sym.mean()),
        "orientation_share_of_composite_raw": float(0.05 * raw.mean() / (pos.mean() + 0.05 * raw.mean())),
        "orientation_share_of_composite_sym": float(0.05 * sym.mean() / (pos.mean() + 0.05 * sym.mean())),
    }
    OUT.write_text(json.dumps({"summary": summary, "rows": rows}, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
