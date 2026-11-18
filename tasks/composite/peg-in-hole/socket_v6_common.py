"""Shared logic for the round-socket strict-v6 run (SAP Warp and MuJoCo Warp): analytically labelled peg-in-hole.

Every cell carries an analytic label (peg_hole_statics.py): inserted / held_in_wedge / held_on_rim.  Each rollout is scored
correct iff its outcome matches the label; penetration is reported, never part of correctness.

Engine simulate(cand, cell, init_xy [R,2]) must use `initial_pose(cell)` for the peg (centre position in m, wxyz quaternion),
add init_xy to the centre's world x/y, and return the v4 keys (final_xyz, peak_pen, final_pen, peak_pen_vertical,
peak_pen_lateral, finite, truncated) plus traj_pos [R,S,3] (m) and traj_quat [R,S,4] (wxyz) sampled every SAMPLE_S from t = 0.
"""
from __future__ import annotations

import math
import time

import numpy as np

from socket_v4_common import (ASSET_DIR, FILES, N_WALL, PEG_HALF, PEG_MASS, R_BORE, json_safe, load_manifest,  # noqa: F401
                              peg_inertia, peg_radius, wilson)

SAMPLE_S = 0.01
SEAT_Z_MM, SEAT_TOL_MM, BORE_DEPTH_MM = 35.0, 1.0, 30.0
SETTLE_WINDOW_S, EPS_Z_MM, V_TH_MM_S = 0.5, 0.5, 5.0
UPRIGHT_MARGIN_DEG = 0.5
HOLD_DISP_MM, HOLD_TILT_DEG = 0.5, 0.5          # strict (rigid-ideal) hold, reported only
WEDGE_DROP_MM, WEDGE_TILT_DEG = 5.0, 5.0        # held_in_wedge: never drops > 5 mm below the start, tilt change <= 5 deg, settled
RIM_DZ_MM, RIM_TILT_DEG = 1.0, 5.0              # held_on_rim: final |z - z0| <= 1 mm, tilt change <= 5 deg, settled


def _qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def initial_pose(cell):
    """Peg centre (m, world) and orientation (wxyz) from the cell's tilt-plane pose: in-plane direction e = (cos az, sin az, 0),
    peg axis (-sin(tilt) e, cos(tilt)), i.e. R = Rz(az) Ry(-tilt)."""
    az = math.radians(cell.get("azimuth_deg", 0.0)); t = math.radians(cell["pose"]["tilt_deg"])
    e = np.array([math.cos(az), math.sin(az), 0.0])
    pos = cell["pose"]["centre_inplane_mm"] * 1e-3 * e + np.array([0.0, 0.0, cell["pose"]["centre_z_mm"] * 1e-3])
    q = _qmul(np.array([math.cos(az / 2), 0, 0, math.sin(az / 2)]), np.array([math.cos(-t / 2), 0, math.sin(-t / 2), 0]))
    return pos, q / np.linalg.norm(q)


def perturbation(cell, seed, R):
    """U(+-0.05 mm) perpendicular to the tilt plane only (keeps the analytic contact geometry)."""
    az = math.radians(cell.get("azimuth_deg", 0.0)); s = np.random.default_rng(seed).uniform(-5e-5, 5e-5, R)
    return np.column_stack([-math.sin(az) * s, math.cos(az) * s])


def tilt_deg(q):
    w, x, y, z = (q[..., i] for i in range(4))
    return np.degrees(np.arccos(np.clip(np.abs(1.0 - 2.0 * (x * x + y * y)), 0.0, 1.0)))


def summarize(cell, sim):
    c = cell["clearance_mm"]; exp = cell["expected"]; P = sim["traj_pos"] * 1e3; Qt = sim["traj_quat"]; R = P.shape[0]
    n_win = int(round(SETTLE_WINDOW_S / SAMPLE_S)); upright_lim = math.degrees(math.atan(2 * c / BORE_DEPTH_MM)) + UPRIGHT_MARGIN_DEG
    rows = []
    for w in range(R):
        p, q = P[w], Qt[w]
        row = {"rollout": w}
        for k in ("peak_pen", "final_pen", "peak_pen_vertical", "peak_pen_lateral"):
            if k in sim:
                row[k + "_mm"] = float(sim[k][w] * 1e3)
        if not (bool(sim["finite"][w]) and np.isfinite(p).all() and np.isfinite(q).all() and int(sim["truncated"]) == 0):
            row.update(outcome="invalid", invalid_reason="non_finite_or_truncated", correct=False); rows.append(row); continue
        zf, rad = float(p[-1, 2]), float(np.hypot(p[-1, 0], p[-1, 1]))
        if zf < -10.0 and rad <= 50.0:
            row.update(outcome="invalid", invalid_reason="tunneled_through_socket", correct=False, final_z_mm=zf); rows.append(row); continue
        tl = tilt_deg(q); seg = p[-(n_win + 1):]
        zr = float(seg[:, 2].max() - seg[:, 2].min()); vmax = float(np.linalg.norm(np.diff(seg, axis=0), axis=1).max() / SAMPLE_S)
        settled = zr <= EPS_Z_MM and vmax <= V_TH_MM_S
        disp = float(np.linalg.norm(p - p[0], axis=1).max()); dtilt = float(np.abs(tl - tl[0]).max())
        inserted = zf <= SEAT_Z_MM + SEAT_TOL_MM and rad <= c and float(tl[-1]) <= upright_lim and settled
        held_strict = disp <= HOLD_DISP_MM and dtilt <= HOLD_TILT_DEG and settled
        drop = float(p[0, 2] - p[:, 2].min())
        held_wedge = drop <= WEDGE_DROP_MM and dtilt <= WEDGE_TILT_DEG and settled
        held_rim = abs(zf - float(p[0, 2])) <= RIM_DZ_MM and dtilt <= RIM_TILT_DEG and settled
        held = held_wedge if exp == "held_in_wedge" else held_rim if exp == "held_on_rim" else held_strict
        outcome = "inserted" if inserted else "ejected" if rad > 50.0 else "stuck"
        correct = inserted if exp == "inserted" else held
        row.update(outcome=outcome, held_in_place=held, held_strict=held_strict, max_drop_mm=drop, correct=correct, final_z_mm=zf, final_radial_mm=rad, final_tilt_deg=float(tl[-1]),
                   max_displacement_mm=disp, max_tilt_change_deg=dtilt, settled=settled,
                   penetration_assisted=bool(inserted and row.get("peak_pen_lateral_mm", 0.0) > 0.5 * c))
        rows.append(row)
    cnt = {o: sum(r["outcome"] == o for r in rows) for o in ("inserted", "stuck", "ejected", "invalid")}

    def stat(sub, key):
        v = [r[key] for r in sub if r.get(key) is not None and np.isfinite(r[key])]
        return (float(np.mean(v)), float(np.max(v))) if v else (None, None)
    out = {"R": R, "expected": exp, "correct_count": sum(r["correct"] for r in rows), "outcomes": cnt,
           "held_in_place_count": sum(r.get("held_in_place", False) for r in rows),
           "held_strict_count": sum(r.get("held_strict", False) for r in rows),
           "invalid_reasons": {k: sum(r.get("invalid_reason") == k for r in rows) for k in ("non_finite_or_truncated", "tunneled_through_socket")},
           "penetration_assisted_count": sum(r.get("penetration_assisted", False) for r in rows), "upright_limit_deg": upright_lim,
           "truncated": int(sim["truncated"]), "rollouts": rows}
    out["correct_rate"] = out["correct_count"] / R; out["correct_wilson_95ci"] = wilson(out["correct_count"], R)
    valid = [r for r in rows if r["outcome"] != "invalid"]
    for key in ("peak_pen_mm", "final_pen_mm", "peak_pen_vertical_mm", "peak_pen_lateral_mm", "final_z_mm", "final_tilt_deg",
                "max_displacement_mm", "max_tilt_change_deg", "max_drop_mm"):
        out[f"{key}_mean"], out[f"{key}_max"] = stat(valid, key)
    return out


def run_cell(cand, cell, seed, R, simulate):
    t0 = time.perf_counter(); sim = simulate(cand, cell, perturbation(cell, seed, R))
    out = summarize(cell, sim); out.update({"cell": cell, "candidate": cand, "seed": seed, "wall_s": time.perf_counter() - t0}); return out


def fmt(r):
    F = lambda v, f=".3f": "None" if v is None else format(v, f)
    o = r["outcomes"]
    return (f'expected={r["expected"]} correct={r["correct_count"]}/{r["R"]} ins/stuck/ej/inv={o["inserted"]}/{o["stuck"]}/{o["ejected"]}/{o["invalid"]} '
            f'held={r["held_in_place_count"]} disp_max={F(r["max_displacement_mm_max"])}mm dtilt_max={F(r["max_tilt_change_deg_max"], ".2f")}deg '
            f'z={F(r["final_z_mm_mean"], ".1f")} pen base/wall={F(r["peak_pen_vertical_mm_mean"])}/{F(r["peak_pen_lateral_mm_mean"])}mm '
            f'assisted={r["penetration_assisted_count"]} wall={r["wall_s"]:.0f}s')


def execute(man, key, parse, simulate, log=print):
    R, s0 = man["R"], man["seed0"]; cands = [parse(t) for t in man["candidate_rule"][key]]
    sels = man["selection"] if isinstance(man["selection"], list) else [man["selection"]]
    sel = []
    for i, c in enumerate(cands):
        rs = []
        for j, cell in enumerate(sels):
            r = run_cell(c, cell, s0 + j, R, simulate); rs.append(r); log(f"select {i + 1} {c['id']} {cell['id']} {fmt(r)}")
        sel.append({"candidate": c, "candidate_index": i + 1, "cells": rs, "correct_count": sum(r["correct_count"] for r in rs),
                    "invalid": sum(r["outcomes"]["invalid"] for r in rs),
                    "wall_pen_mean": float(np.mean([r["peak_pen_lateral_mm_mean"] for r in rs if r["peak_pen_lateral_mm_mean"] is not None] or [9e9])),
                    "wall_s": sum(r["wall_s"] for r in rs)})
    win = min(sel, key=lambda s: (-s["correct_count"], s["invalid"], s["wall_pen_mean"], s["wall_s"]))
    log(f"frozen {win['candidate']} (selection correct {win['correct_count']}/{R * len(sels)})")
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(win["candidate"], cell, s0 + 1000 + j, R, simulate); held.append(r); log(f"held {cell['id']} {fmt(r)}")
    by = {}
    for r in held:
        b = by.setdefault(r["expected"], [0, 0]); b[0] += r["correct_count"]; b[1] += r["R"]
    return {"selection_candidates": sel, "frozen_index": win["candidate_index"], "frozen_candidate": win["candidate"], "heldout": held,
            "heldout_correct_total": [sum(r["correct_count"] for r in held), R * len(held)], "heldout_correct_by_label": by,
            "heldout_outcome_totals": {o: sum(r["outcomes"][o] for r in held) for o in ("inserted", "stuck", "ejected", "invalid")}}
