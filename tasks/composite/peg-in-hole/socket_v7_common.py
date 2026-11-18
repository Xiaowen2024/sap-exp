"""Shared logic for the round-socket strict-v7 run (DRAFT; SAP Warp and MuJoCo Warp).

Same cells, labels and engine interface as strict-v6 (socket_v6_common).  Two rule changes, both motivated by the v6 audit:

1. Radial tolerance.  v6 required final radial <= clearance exactly, while depth (+1 mm) and tilt (+0.5 deg) had tolerances.  A seated
   peg resting against the bore wall overshoots by its static wall penetration (0.3-10.2 um in every v6 case), which flipped otherwise
   identical rollouts to "not inserted".  v7: final radial <= clearance + RADIAL_TOL_MM.
2. Selection tie-break.  v6 ranked candidates by correct count, invalid count, mean peak bore-wall penetration, wall time.  Base-impact
   penetration was invisible to the rule and wall time is machine dependent.  v7: correct count, invalid count, mean peak penetration
   over ALL peg contacts (base and wall) on the selection cells, then the declared candidate order.  With equal correctness the
   stiffest (least-penetrating) candidate is frozen.

Reported only (never part of correctness): at_depth (final centre z within the seat tolerance, whatever the other checks say) and
radial_excess_um, so "reached the seat but unsettled / over the radial bound" is separable from "did not insert".
"""
from __future__ import annotations

import math
import time

import numpy as np

from socket_v6_common import (BORE_DEPTH_MM, EPS_Z_MM, HOLD_DISP_MM, HOLD_TILT_DEG, RIM_DZ_MM, RIM_TILT_DEG, SAMPLE_S, SEAT_TOL_MM,  # noqa: F401
                              SEAT_Z_MM, SETTLE_WINDOW_S, UPRIGHT_MARGIN_DEG, V_TH_MM_S, WEDGE_DROP_MM, WEDGE_TILT_DEG, fmt, initial_pose,
                              json_safe, load_manifest, peg_inertia, peg_radius, perturbation, tilt_deg, wilson, ASSET_DIR, FILES, PEG_HALF, PEG_MASS)

RADIAL_TOL_MM = 0.01


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
        at_depth = zf <= SEAT_Z_MM + SEAT_TOL_MM
        inserted = at_depth and rad <= c + RADIAL_TOL_MM and float(tl[-1]) <= upright_lim and settled
        held_strict = disp <= HOLD_DISP_MM and dtilt <= HOLD_TILT_DEG and settled
        drop = float(p[0, 2] - p[:, 2].min())
        held_wedge = drop <= WEDGE_DROP_MM and dtilt <= WEDGE_TILT_DEG and settled
        held_rim = abs(zf - float(p[0, 2])) <= RIM_DZ_MM and dtilt <= RIM_TILT_DEG and settled
        held = held_wedge if exp == "held_in_wedge" else held_rim if exp == "held_on_rim" else held_strict
        outcome = "inserted" if inserted else "ejected" if rad > 50.0 else "stuck"
        correct = inserted if exp == "inserted" else held
        row.update(outcome=outcome, held_in_place=held, held_strict=held_strict, max_drop_mm=drop, correct=correct, final_z_mm=zf, final_radial_mm=rad,
                   final_tilt_deg=float(tl[-1]), max_displacement_mm=disp, max_tilt_change_deg=dtilt, settled=settled, at_depth=at_depth,
                   radial_excess_um=max(0.0, rad - c) * 1e3 if at_depth else None,
                   penetration_assisted=bool(inserted and row.get("peak_pen_lateral_mm", 0.0) > 0.5 * c))
        rows.append(row)
    cnt = {o: sum(r["outcome"] == o for r in rows) for o in ("inserted", "stuck", "ejected", "invalid")}

    def stat(sub, key):
        v = [r[key] for r in sub if r.get(key) is not None and np.isfinite(r[key])]
        return (float(np.mean(v)), float(np.max(v))) if v else (None, None)
    out = {"R": R, "expected": exp, "correct_count": sum(r["correct"] for r in rows), "outcomes": cnt,
           "held_in_place_count": sum(r.get("held_in_place", False) for r in rows),
           "held_strict_count": sum(r.get("held_strict", False) for r in rows),
           "at_depth_count": sum(r.get("at_depth", False) for r in rows),
           "invalid_reasons": {k: sum(r.get("invalid_reason") == k for r in rows) for k in ("non_finite_or_truncated", "tunneled_through_socket")},
           "penetration_assisted_count": sum(r.get("penetration_assisted", False) for r in rows), "upright_limit_deg": upright_lim,
           "radial_tol_mm": RADIAL_TOL_MM, "truncated": int(sim["truncated"]), "rollouts": rows}
    out["correct_rate"] = out["correct_count"] / R; out["correct_wilson_95ci"] = wilson(out["correct_count"], R)
    valid = [r for r in rows if r["outcome"] != "invalid"]
    for key in ("peak_pen_mm", "final_pen_mm", "peak_pen_vertical_mm", "peak_pen_lateral_mm", "final_z_mm", "final_tilt_deg",
                "max_displacement_mm", "max_tilt_change_deg", "max_drop_mm", "radial_excess_um"):
        out[f"{key}_mean"], out[f"{key}_max"] = stat(valid, key)
    return out


def run_cell(cand, cell, seed, R, simulate):
    t0 = time.perf_counter(); sim = simulate(cand, cell, perturbation(cell, seed, R))
    out = summarize(cell, sim); out.update({"cell": cell, "candidate": cand, "seed": seed, "wall_s": time.perf_counter() - t0}); return out


def selection_key(s):
    """Higher correct count, fewer invalid rollouts, lower mean peak penetration over all peg contacts, then declared candidate order."""
    return (-s["correct_count"], s["invalid"], s["peak_pen_mean"], s["candidate_index"])


def execute(man, key, parse, simulate, log=print):
    tol = man["fixed"].get("radial_tolerance_mm")
    if tol is None or abs(float(tol) - RADIAL_TOL_MM) > 1e-12:
        raise SystemExit(f"manifest fixed.radial_tolerance_mm ({tol}) must equal socket_v7_common.RADIAL_TOL_MM ({RADIAL_TOL_MM})")
    R, s0 = man["R"], man["seed0"]; cands = [parse(t) for t in man["candidate_rule"][key]]
    sels = man["selection"] if isinstance(man["selection"], list) else [man["selection"]]
    sel = []
    for i, c in enumerate(cands):
        rs = []
        for j, cell in enumerate(sels):
            r = run_cell(c, cell, s0 + j, R, simulate); rs.append(r); log(f"select {i + 1} {c['id']} {cell['id']} {fmt(r)}")
        sel.append({"candidate": c, "candidate_index": i + 1, "cells": rs, "correct_count": sum(r["correct_count"] for r in rs),
                    "invalid": sum(r["outcomes"]["invalid"] for r in rs),
                    "peak_pen_mean": float(np.mean([r["peak_pen_mm_mean"] for r in rs if r["peak_pen_mm_mean"] is not None] or [9e9])),
                    "wall_pen_mean": float(np.mean([r["peak_pen_lateral_mm_mean"] for r in rs if r["peak_pen_lateral_mm_mean"] is not None] or [9e9])),
                    "wall_s": sum(r["wall_s"] for r in rs)})
    win = min(sel, key=selection_key)
    log(f"frozen {win['candidate']} (selection correct {win['correct_count']}/{R * len(sels)}, mean peak penetration {win['peak_pen_mean']:.3f} mm)")
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(win["candidate"], cell, s0 + 1000 + j, R, simulate); held.append(r); log(f"held {cell['id']} {fmt(r)}")
    by = {}
    for r in held:
        b = by.setdefault(r["expected"], [0, 0]); b[0] += r["correct_count"]; b[1] += r["R"]
    return {"selection_candidates": sel, "frozen_index": win["candidate_index"], "frozen_candidate": win["candidate"], "heldout": held,
            "heldout_correct_total": [sum(r["correct_count"] for r in held), R * len(held)], "heldout_correct_by_label": by,
            "heldout_cells_all_correct": [sum(r["correct_count"] == r["R"] for r in held), len(held)],
            "heldout_at_depth_total": sum(r["at_depth_count"] for r in held),
            "heldout_outcome_totals": {o: sum(r["outcomes"][o] for r in held) for o in ("inserted", "stuck", "ejected", "invalid")}}
