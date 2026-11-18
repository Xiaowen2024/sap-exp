"""Engine-independent pieces of the simple-pinch force-controlled-v3 run (shared by the SAP and MuJoCo Warp drivers)."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import time
from typing import Any, Callable

import numpy as np

from strict_v3_common import json_safe, load_manifest, wilson  # noqa: F401

G = 9.81
GATE_S, LIFT_S, HOLD_S, LIFT_M = 0.8, 1.0, 0.25, 0.060
RETAIN_RISE_M = 0.057
PRELOAD_TOL = 0.05
PAD_KD, PAD_Z_KE, PAD_Z_KD = 300.0, 2.0e4, 300.0
SAMPLE_S = 0.01


def perturbations(seed: int, R: int) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    return {"yz": rng.uniform(-4e-4, 4e-4, (R, 2)), "pitch": rng.uniform(-4e-3, 4e-3, R),
            "load_scale": 1.0 + rng.uniform(-5e-3, 5e-3, R)}


def expected(cell: dict[str, Any]) -> tuple[str, float]:
    r = 2 * cell["mu"] * cell["per_pad_force_N"] / (cell["mass_kg"] * G)
    lab = "retain" if r >= 1.2 - 1e-6 else ("release" if r <= 0.8 + 1e-6 else "boundary")
    return lab, r


def lift_target(t: float) -> tuple[float, float]:
    """(z, vz) of the pad lift command at time t since the end of the gate."""
    if t < LIFT_S:
        return LIFT_M * t / LIFT_S, LIFT_M / LIFT_S
    return LIFT_M, 0.0


def summarize(cell: dict[str, Any], *, gate_fn: np.ndarray, gate_duty: np.ndarray, t: np.ndarray,
              cube_rise: np.ndarray, rel_z: np.ndarray, finite: np.ndarray, truncated: int) -> dict[str, Any]:
    """gate_fn [S,R,2] per-pad normal force over the last 0.2 s of the gate; gate_duty [S,R,2];
    t [T] since gate end; cube_rise [T,R]; rel_z [T,R] cube minus pad height change."""
    lab, r = expected(cell)
    F = cell["per_pad_force_N"]
    fn = gate_fn.mean(axis=0)                                     # [R,2]
    pre_err = np.abs(fn - F).max(axis=1) / F
    duty_ok = gate_duty.all(axis=(0, 2))
    final = cube_rise[-1]
    observed = np.where(final >= RETAIN_RISE_M, "retain", "release")
    hold = t >= LIFT_S - 1e-9
    creep = np.array([abs(np.polyfit(t[hold], rel_z[hold, w], 1)[0]) for w in range(cube_rise.shape[1])])
    valid = finite & duty_ok & (pre_err <= PRELOAD_TOL) & (truncated == 0)
    agree = observed == lab
    R = len(final)
    rows = [{"rollout": w, "observed": str(observed[w]), "agrees": bool(agree[w]), "valid": bool(valid[w]),
             "gate_preload_N": fn[w].tolist(), "preload_rel_error": float(pre_err[w]), "gate_duty_ok": bool(duty_ok[w]),
             "rise_mm": float(final[w] * 1e3), "hold_slip_creep_mm_s": float(creep[w] * 1e3)} for w in range(R)]
    k = int((agree & valid).sum())
    return {"expected": lab, "ideal_margin_r": r, "R": R, "agree_and_valid_count": k, "accuracy": k / R,
            "accuracy_wilson_95ci": wilson(k, R), "observed_retain": int((observed == "retain").sum()),
            "valid_count": int(valid.sum()), "gate_preload_mean_N": float(fn.mean()),
            "preload_rel_error_max": float(pre_err.max()), "rise_mm_median": float(np.median(final) * 1e3),
            "lift_follow_error_mm_mean": float(np.mean(np.abs(LIFT_M - final)) * 1e3),
            "hold_slip_creep_mm_s_mean": float(creep.mean() * 1e3), "truncated": int(truncated), "rollouts": rows}


def choose(sel_rows: list[list[dict[str, Any]]], walls: list[float]) -> int:
    def key(i):
        rows = sel_rows[i]
        ret = [r for r in rows if r["expected"] == "retain"]
        return (-sum(r["agree_and_valid_count"] for r in rows), np.mean([r["lift_follow_error_mm_mean"] for r in ret]),
                np.mean([r["hold_slip_creep_mm_s_mean"] for r in ret]), walls[i])
    return min(range(len(sel_rows)), key=key)


def execute(manifest: dict[str, Any], engine_key: str, parse: Callable, run_cell: Callable, log: Callable = print) -> dict[str, Any]:
    R, seed0 = manifest["R"], manifest["seed0"]
    cands = [parse(t) for t in manifest["candidate_rule"][engine_key]]
    for c in list(manifest["selection"]) + list(manifest["held_out_conditions"]):
        if expected(c)[0] != c["expected"]:
            raise ValueError(f"label mismatch for {c['id']}")
    fmt = lambda r: (f'{r["agree_and_valid_count"]:2d}/20 retain={r["observed_retain"]:2d} valid={r["valid_count"]:2d} '
                     f'preload={r["gate_preload_mean_N"]:.3f}N (err<={r["preload_rel_error_max"]:.1%}) rise={r["rise_mm_median"]:.1f}mm '
                     f'creep={r["hold_slip_creep_mm_s_mean"]:.3f}mm/s')
    selection, walls = [], []
    for i, c in enumerate(cands):
        rows = []
        for k, cell in enumerate(manifest["selection"]):
            t0 = time.perf_counter(); r = run_cell(c, cell, seed0 + k, R); r["wall_s"] = time.perf_counter() - t0
            r.update({"cell": cell, "candidate": c}); rows.append(r)
            log(f"select {i + 1} {c['id']} {cell['id']} {r['expected']} {fmt(r)}")
        selection.append(rows); walls.append(sum(r["wall_s"] for r in rows))
    win = choose(selection, walls); frozen = cands[win]
    log(f"frozen {frozen}")
    held = []
    for j, cell in enumerate(manifest["held_out_conditions"]):
        t0 = time.perf_counter(); r = run_cell(frozen, cell, seed0 + 1000 + j, R); r["wall_s"] = time.perf_counter() - t0
        r.update({"cell": cell, "candidate": frozen}); held.append(r)
        log(f"held {cell['id']} {r['expected']} {fmt(r)}")
    return {"selection_candidates": selection, "frozen_index": win + 1, "frozen_candidate": frozen, "heldout": held,
            "heldout_accuracy_total": [sum(r["agree_and_valid_count"] for r in held), R * len(held)]}
