"""Shared logic for the round-socket strict-v4 run (SAP Warp and MuJoCo Warp).

Engine simulate(cand, cell, init_xy [R,2]) -> dict(final_xyz [R,3] (m), peak_pen [R] (m), final_pen [R] (m),
finite [R], truncated int).
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import math, time
import numpy as np
from strict_v3_common import json_safe, load_manifest, wilson  # noqa: F401

R_BORE, PEG_HALF, PEG_MASS, N_WALL = 0.012, 0.025, 0.08, 32
ASSET_DIR = "assets/round_socket_exact"
FILES = ["socket_base.obj"] + [f"socket_wall_{k:02d}.obj" for k in range(N_WALL)]


def peg_radius(cell):
    return R_BORE - cell["clearance_mm"] * 1e-3


def peg_inertia(cell):
    r = peg_radius(cell); h = 2 * PEG_HALF; m = PEG_MASS
    return (m * (3 * r * r + h * h) / 12, m * (3 * r * r + h * h) / 12, m * r * r / 2)


def init_xy(seed, R):
    return np.random.default_rng(seed).uniform(-5e-5, 5e-5, (R, 2))


ALL_GATES = ("entered", "radial", "not_escaped", "no_truncation", "peak_pen", "final_pen")


def summarize(cell, sim, gate_terms=ALL_GATES):
    c = cell["clearance_mm"]; x, y, z = (sim["final_xyz"][:, i] * 1e3 for i in range(3))
    rad = np.hypot(x, y); pk = sim["peak_pen"] * 1e3; pf = sim["final_pen"] * 1e3
    rows = []
    for w in range(len(x)):
        fin = bool(sim["finite"][w]); entered = bool(fin and z[w] <= 45 and rad[w] <= 3); escaped = bool((not fin) or rad[w] > 50 or z[w] < -10 or z[w] > 160)
        g = {"entered": entered, "radial": bool(fin and rad[w] <= c), "not_escaped": not escaped, "no_truncation": int(sim["truncated"]) == 0,
             "peak_pen": bool(fin and pk[w] <= 0.1 * c), "final_pen": bool(fin and pf[w] <= 0.1 * c)}
        row = {"rollout": w, "credible": all(g[k] for k in gate_terms), "gates": g, "final_xyz_mm": [float(x[w]), float(y[w]), float(z[w])],
               "final_radial_mm": float(rad[w]), "peak_pen_mm": float(pk[w]), "final_pen_mm": float(pf[w])}
        for k in ("peak_pen_vertical", "peak_pen_lateral"):
            if k in sim:
                row[k + "_mm"] = float(sim[k][w] * 1e3)
        rows.append(row)
    k = sum(r["credible"] for r in rows); n = len(rows)
    fails = {g: sum(1 for r in rows if not r["gates"][g]) for g in rows[0]["gates"]}
    fin = [r for r in rows if np.isfinite(r["final_radial_mm"])]
    diag = {f"{key}_mm_mean": float(np.mean(sim[key][np.isfinite(sim[key])]) * 1e3) for key in ("peak_pen_vertical", "peak_pen_lateral") if key in sim}
    for name, sub in (("credible", [r for r in rows if r["credible"]]), ("entered", [r for r in rows if r["gates"]["entered"]])):
        for key in ("peak_pen_mm", "final_pen_mm", "peak_pen_vertical_mm", "peak_pen_lateral_mm"):
            v = [r[key] for r in sub if key in r and np.isfinite(r[key])]
            diag[f"{name}_{key}_mean"] = float(np.mean(v)) if v else None
            diag[f"{name}_{key}_max"] = float(np.max(v)) if v else None
        diag[f"{name}_count"] = len(sub)
    return {"R": n, "credible_count": k, **diag, "credible_rate": k / n, "credible_wilson_95ci": wilson(k, n), "failed_gate_counts": fails,
            "pmax_over_clearance_mean": float(np.mean([r["peak_pen_mm"] for r in fin]) / c) if fin else None,
            "final_radial_mm_mean": float(np.mean([r["final_radial_mm"] for r in fin])) if fin else None,
            "final_z_mm_mean": float(np.mean([r["final_xyz_mm"][2] for r in fin])) if fin else None, "truncated": int(sim["truncated"]), "rollouts": rows}


def run_cell(cand, cell, seed, R, simulate, gate_terms=ALL_GATES):
    t0 = time.perf_counter(); sim = simulate(cand, cell, init_xy(seed, R))
    out = summarize(cell, sim, gate_terms); out.update({"cell": cell, "candidate": cand, "wall_s": time.perf_counter() - t0}); return out


def execute(man, key, parse, simulate, log=print):
    R, s0 = man["R"], man["seed0"]; cands = [parse(t) for t in man["candidate_rule"][key]]
    gates = tuple(man["fixed"].get("gate_terms", ALL_GATES))
    F = lambda v, s: "None" if v is None else format(v, s)
    fmt = lambda r: (f'{r["credible_count"]}/{r["R"]} pen|ok={F(r.get("credible_peak_pen_mm_mean"), ".3f")}mm (vert {F(r.get("credible_peak_pen_vertical_mm_mean"), ".3f")} lat {F(r.get("credible_peak_pen_lateral_mm_mean"), ".3f")}) fails={ {k: v for k, v in r["failed_gate_counts"].items() if v} } '
                     f'pmax/c={F(r["pmax_over_clearance_mean"], ".3f")} vert={F(r.get("peak_pen_vertical_mm_mean"), ".3f")}mm lat={F(r.get("peak_pen_lateral_mm_mean"), ".3f")}mm radial={F(r["final_radial_mm_mean"], ".3f")}mm z={F(r["final_z_mm_mean"], ".1f")}mm')
    sel = []
    for i, c in enumerate(cands):
        r = run_cell(c, man["selection"], s0, R, simulate, gates); r["candidate_index"] = i + 1; sel.append(r); log(f"select {i + 1} {c['id']} {fmt(r)}")
    big = 9e9
    use_pen = "peak_pen" in gates                     # v5 (penetration gates removed) keeps penetration out of the tie-break too
    win = min(sel, key=lambda r: (-r["credible_rate"],
                                  (r["pmax_over_clearance_mean"] if r["pmax_over_clearance_mean"] is not None else big) if use_pen else 0.0,
                                  r["final_radial_mm_mean"] if r["final_radial_mm_mean"] is not None else big, r["wall_s"]))
    log(f"frozen {win['candidate']}")
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(win["candidate"], cell, s0 + 1000 + j, R, simulate, gates); held.append(r); log(f"held {cell['id']} {fmt(r)}")
    out = {"gate_terms": list(gates), "selection_candidates": sel, "frozen_index": win["candidate_index"], "frozen_candidate": win["candidate"],
           "heldout": held, "heldout_credible_total": [sum(r["credible_count"] for r in held), R * len(held)]}
    if man.get("diagnostic_all_candidates_heldout"):
        # exploratory, not part of the frozen-setting result: every non-frozen candidate on every held-out cell (same seeds)
        diag = {}
        for i, c in enumerate(cands):
            if i + 1 == win["candidate_index"]:
                continue
            rows = []
            for j, cell in enumerate(man["held_out_conditions"]):
                r = run_cell(c, cell, s0 + 1000 + j, R, simulate, gates); rows.append(r); log(f"diag {c['id']} {cell['id']} {fmt(r)}")
            diag[c["id"]] = rows
        out["diagnostic_heldout_all_candidates"] = diag
    return out
