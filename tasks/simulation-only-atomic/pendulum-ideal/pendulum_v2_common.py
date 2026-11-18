"""Shared logic for the pendulum energy-conservation-v2 run (SAP Warp and MuJoCo Warp)."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import math
import time

import numpy as np

from strict_v3_common import json_safe, load_manifest  # noqa: F401

G, R_BOB, SAMPLE_S, DURATION_S = 9.81, 0.02, 0.01, 10.0


def i_com(m):
    return 0.4 * m * R_BOB ** 2


def ellipk(k):
    a, b = 1.0, math.sqrt(1 - k * k)
    for _ in range(40):
        a, b = (a + b) / 2, math.sqrt(a * b)
    return math.pi / (2 * a)


def exact_period(cell):
    m, L = cell["mass_kg"], cell["length_m"]; Ip = m * L * L + i_com(m)
    return 4 * math.sqrt(Ip / (m * G * L)) * ellipk(math.sin(math.radians(cell["amplitude_deg"]) / 2))


def theta0(cell, seed, R):
    return math.radians(cell["amplitude_deg"]) + np.random.default_rng(seed).uniform(-3e-3, 3e-3, R)


def metrics(cell, q, v):
    m, L = cell["mass_kg"], cell["length_m"]; Ip = m * L * L + i_com(m)
    rows = []
    T_ex = exact_period(cell)
    for r in range(q.shape[1]):
        if not (np.isfinite(q[:, r]).all() and np.isfinite(v[:, r]).all()):
            rows.append({"finite": False}); continue
        e = 0.5 * Ip * v[:, r] ** 2 + m * G * L * (1 - np.cos(q[:, r])); e0 = e[0]
        # period from successive upward zero crossings of the angle
        z = np.flatnonzero((q[:-1, r] < 0) & (q[1:, r] >= 0))
        tz = [(i + (-q[i, r]) / (q[i + 1, r] - q[i, r])) * SAMPLE_S for i in z]
        T = float(np.mean(np.diff(tz))) if len(tz) > 1 else math.nan
        rows.append({"finite": True, "max_rel_energy_drift": float(np.max(np.abs(e - e0)) / e0), "final_energy_ratio": float(e[-1] / e0),
                     "period_s": T, "period_rel_error": float(abs(T - T_ex) / T_ex) if np.isfinite(T) else math.nan})
    ok = [x for x in rows if x["finite"]]
    mean = lambda k: float(np.nanmean([x[k] for x in ok])) if ok else None
    return {"rows": rows, "n_finite": len(ok), "loss": mean("max_rel_energy_drift") if len(ok) == len(rows) else math.inf,
            "final_energy_ratio": mean("final_energy_ratio"), "period_rel_error": mean("period_rel_error"), "exact_period_s": T_ex}


def run_cell(cand, cell, seed, R, simulate):
    t0 = time.perf_counter()
    q, v = simulate(cand, cell, theta0(cell, seed, R))
    out = metrics(cell, q, v); out.update({"cell": cell, "candidate": cand, "R": R, "wall_s": time.perf_counter() - t0})
    return out


def execute(man, key, parse, simulate, log=print):
    R, s0 = man["R"], man["seed0"]
    cands = [parse(t) for t in man["candidate_rule"][key]]
    fmt = lambda r: f'loss={r["loss"]:.3e} final_E={r["final_energy_ratio"]} period_err={r["period_rel_error"]}'
    sel = []
    for i, c in enumerate(cands):
        r = run_cell(c, man["selection"], s0, R, simulate); r["candidate_index"] = i + 1; sel.append(r)
        log(f"select {i + 1} {c['id']} {fmt(r)}")
    valid = [r for r in sel if math.isfinite(r["loss"])]
    win = min(valid, key=lambda r: (r["loss"], r["wall_s"])); log(f"frozen {win['candidate']}")
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(win["candidate"], cell, s0 + 1000 + j, R, simulate); held.append(r); log(f"held {cell['id']} {fmt(r)}")
    return {"selection_candidates": sel, "frozen_index": win["candidate_index"], "frozen_candidate": win["candidate"], "heldout": held}
