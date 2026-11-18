#!/usr/bin/env python3
"""Fresh R=20 candidate screen in the available public SAP runtime.

This is a new protocol revision, not a reproduction of the unrecoverable
internal artifact.  Each rollout receives the declared x/y lateral
perturbation before the public equal-budget runner is invoked.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np


ROOT = Path("/home/xw/sim-exp-gpu")
PUBLIC = ROOT / "sap-sim-round"
SOURCE = PUBLIC / "scripts" / "run_round_socket_equal_budget.py"
OUT = ROOT / "results" / "round-sap-public-r20-selection"
R = 20
CANDIDATES = [
    {"id": "pair_k1e4_tau3ms_gap0p1", "dt_ms": .25, "kpair_N_m": 1e4, "taupair_ms": 3., "rigid_gap_mm": .1, "preset": "drake"},
    {"id": "pair_k3e4_tau3ms_gap0p1", "dt_ms": .25, "kpair_N_m": 3e4, "taupair_ms": 3., "rigid_gap_mm": .1, "preset": "drake"},
    {"id": "pair_k1e5_tau3ms_gap0p1", "dt_ms": .25, "kpair_N_m": 1e5, "taupair_ms": 3., "rigid_gap_mm": .1, "preset": "drake"},
    {"id": "pair_k1e4_tau10ms_gap0p1", "dt_ms": .25, "kpair_N_m": 1e4, "taupair_ms": 10., "rigid_gap_mm": .1, "preset": "drake"},
]


spec = importlib.util.spec_from_file_location("round_public", SOURCE)
runner = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(runner)
runner.base.PIECES = PUBLIC / "collision"
runner.OUT = OUT
original_scene = runner.base.material_scene


def execute(candidate: dict, candidate_index: int) -> dict:
    rng = np.random.default_rng(20264029 + candidate_index * 1000)
    perturb = rng.uniform(-.00005, .00005, size=(R, 2))
    rows = []
    for repeat in range(R):
        y = float(perturb[repeat, 1])
        x_delta = float(perturb[repeat, 0])

        def patched_scene(radius: float, offset: float, rep: str, peg_mesh: Path):
            scene = original_scene(radius, offset + x_delta, rep, peg_mesh)
            scene["bodies"][1]["transform"]["p"][1] = y
            return scene

        runner.base.material_scene = patched_scene
        row = runner.run(candidate, candidate_index * R + repeat + 1, 2.0, 1.0, "cuda:0")
        row.update({"seed": 20264029 + candidate_index * 1000 + repeat,
                    "perturb_xy_mm": (perturb[repeat] * 1000.0).tolist()})
        rows.append(row)
        print(candidate["id"], repeat, row["credible"], flush=True)
    runner.base.material_scene = original_scene
    return {"candidate": candidate, "R": R, "clearance_mm": 2.0, "offset_mm": 1.0,
            "tilt_deg": 1.0, "duration_s": 2.0,
            "credible_count": sum(row["credible"] for row in rows),
            "credible_rate": float(np.mean([row["credible"] for row in rows])),
            "pmax_over_clearance_mean": float(np.mean([row["pmax_over_c"] for row in rows])),
            "final_radial_mm_mean": float(np.mean([row["final_radial_mm"] for row in rows])),
            "rollouts": rows}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for index, candidate in enumerate(CANDIDATES):
        rows.append(execute(candidate, index))
        (OUT / "selection.json").write_text(json.dumps({"protocol": "round-socket-public-r20-selection-v1",
            "note": "fresh public-runtime revision; not a reproduction of the unavailable internal runner",
            "candidates": rows}, indent=2) + "\n")


if __name__ == "__main__":
    main()
