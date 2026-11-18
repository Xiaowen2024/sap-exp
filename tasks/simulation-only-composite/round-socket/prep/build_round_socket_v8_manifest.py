#!/usr/bin/env python3
"""Generate the round-socket strict-v8 DRAFT manifest (every label is computed here from peg_hole_statics; nothing is hand-entered).

Design rule: every test is a provable statement of the rigid-body Coulomb model (an "admissible set" for some observable), and test
points are placed by their margin to an analytic boundary, not on a parameter grid.  Free-fall release only (no controller to match).

  E  entry map              - the task question: dropped from above the bore with a lateral offset or tilt, does the peg go in?
                              labelled where theory is certain (clear free entry / rests on the rim); elsewhere the outcome is recorded
                              and only the universal checks apply
  W  wedge friction ladder  - peg starts in two-point contact: below mu* it must slide in, above the unique-hold edge it must stay
  R  rim offset ladder      - upright peg on the rim: the outer boundary of the entry map (tips in / rests), exact threshold
  I  invariances            - outcome must not change with gravity scale, mass, or mirror image
  C  centred free-entry control and ladder calibration
Universal checks on every rollout: pose inside the non-penetrating set, mechanical energy never above its initial value, final pose is a
static equilibrium the rigid model allows at that friction.  Every regularisation rung of both engines is run on every condition.

Usage: python3 build_round_socket_v8_manifest.py [--out tasks/simulation-only-composite/round-socket/manifests/round_socket_strict_v8_draft.json]
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import linprog

import peg_hole_statics as ps

R_BORE, N_SIDES, Z_TOP, Z_BASE, LP, AZ = 12.0, 32, 40.0, 10.0, 50.0, 5.625
DEV_C, HELD_C = 0.3, (0.1, 1.0)                       # half-decade steps in clearance / regularisation length; 0.1 and 0.3 mm were never run, 1.0 mm was a v6 clearance (new tilts and frictions here)
DT_MS, DUR_S, MASS_KG = 0.5, 3.0, 0.08
MU_NEUTRAL = 0.5                                      # blocks whose label does not depend on friction use one value (v6 duplicated them)
RIM_RATIOS_BELOW, RIM_RATIOS_ABOVE = (0.85,), (1.10, 1.50, 2.20)   # margin ladder about the exact threshold; kept only while the centre is >= 1 mm inside the bore
G_SCALES, MASS_SCALES = (8, 64), (0.1, 10.0)
SLIDE_LADDER, HOLD_LADDER = (0.5, 0.8), (1.25, 2.0)     # mu / mu* on the must-slide side, mu / unique-hold edge on the must-stay side
STICK = [("stick", "stick")]


# ---------------------------------------------------------------------------------------------- analytic helpers
def rim_threshold_polygon(c):
    """Smallest centre offset (along the tilt-plane direction, through a flat-face centre of the 32-gon) at which an upright peg on
    the rim top is statically stable: the centre must not lie inside the innermost point of (peg disc minus polygonal bore)."""
    r = R_BORE - c; ang = np.radians(np.arange(N_SIDES) * 360.0 / N_SIDES); nrm = np.stack([np.cos(ang), np.sin(ang)], axis=1)
    phi = np.linspace(0, 2 * np.pi, 200001)

    def xmin(d):
        pts = np.stack([d + r * np.cos(phi), r * np.sin(phi)], axis=1)
        outside = (pts @ nrm.T).max(axis=1) > R_BORE
        return pts[outside, 0].min()
    lo, hi = 0.5, R_BORE
    for _ in range(50):
        mid = 0.5 * (lo + hi); lo, hi = (lo, mid) if mid >= xmin(mid) else (mid, hi)
    return hi


def lean_mu_threshold(c):
    """Smallest mu at which a seated peg can also rest leaning at the largest geometric tilt (three contacts: raised bottom corner on
    one wall, opposite side on the rim corner, lowest corner on the base).  2D statics LP; approximate (the lowest corner sits
    0.02-0.07 mm above the base at free_entry_tilt)."""
    th = ps.free_entry_tilt(c); p = ps.wedge_pose(c, th); r = p["r"]; a, n = p["a"], p["n"]; A, B, G = p["A"], p["B"], p["G"]; C = p["Pb"] - r * n
    cont = [(A, np.array([-1.0, 0.0]), np.array([0.0, 1.0])), (B, n, a), (C, np.array([0.0, 1.0]), np.array([1.0, 0.0]))]
    cross = lambda u, v: u[0] * v[1] - u[1] * v[0]

    def feasible(mu):
        Aeq = np.zeros((3, 6)); Aub = []
        for k, (P, nn, tt) in enumerate(cont):
            for j, d in enumerate((nn, tt)):
                Aeq[0, 2 * k + j] = d[0]; Aeq[1, 2 * k + j] = d[1]; Aeq[2, 2 * k + j] = cross(P - G, d)
            for s in (1, -1):
                row = np.zeros(6); row[2 * k] = -mu; row[2 * k + 1] = s; Aub.append(row)
        return linprog(np.zeros(6), A_ub=np.array(Aub), b_ub=np.zeros(6), A_eq=Aeq, b_eq=[0.0, 1.0, 0.0], bounds=[(0, None), (None, None)] * 3).status == 0
    lo, hi = 0.0, 5.0
    for _ in range(50):
        mid = 0.5 * (lo + hi); lo, hi = (lo, mid) if feasible(mid) else (mid, hi)
    return th, hi


def hold_window(c, tilt):
    """[lo, hi] of mu for which (stick, stick) is the ONLY consistent contact mode at release (None if there is no such mu)."""
    ms = ps.mu_star(c, tilt); grid = np.geomspace(ms * 1.001, 6.0, 240)
    ok = [ps.release_modes(c, tilt, float(m)) == STICK for m in grid]; idx = [i for i, o in enumerate(ok) if o]
    if not idx:
        return ms, None, None

    def refine(a, b, want_low):
        for _ in range(30):
            m = math.sqrt(a * b); good = ps.release_modes(c, tilt, m) == STICK
            a, b = ((a, m) if good else (m, b)) if want_low else ((m, b) if good else (a, m))
        return b if want_low else a
    lo = refine(grid[idx[0] - 1] if idx[0] > 0 else ms, grid[idx[0]], True)
    hi = refine(grid[idx[-1]], grid[idx[-1] + 1] if idx[-1] + 1 < len(grid) else 6.0, False)
    return ms, lo, hi


def wedge_tilt_for_depth(c, min_depth_mm):
    """Largest integer tilt whose bottom corner is at least min_depth_mm below the rim top and that has a unique-hold window."""
    for t in range(40, 1, -1):
        p = ps.wedge_pose(c, float(t))
        if p["valid"] and p["A_depth_below_rim"] >= min_depth_mm and hold_window(c, float(t))[1] is not None:
            return float(t)
    raise SystemExit(f"no wedge tilt for c={c}, depth>={min_depth_mm}")


def base(cid, c, mu, category, expected, pose, basis, split, **extra):
    d = {"id": cid, "split": split, "block": cid[0], "category": category, "clearance_mm": c, "mu": round(float(mu), 4), "dt_ms": DT_MS,
         "duration_s": DUR_S, "azimuth_deg": AZ, "gravity_scale": 1, "mass_kg": MASS_KG, "expected": expected, "pose": pose, "label_basis": basis}
    d.update(extra); return d


def wedge_pose_dict(c, tilt):
    p = ps.wedge_pose(c, tilt)
    return {"centre_inplane_mm": round(float(p["G"][0]), 4), "centre_z_mm": round(float(p["G"][1]), 4), "tilt_deg": tilt}, p


# ---------------------------------------------------------------------------------------------- blocks
def build():
    cells, geom = [], {}
    for c in (DEV_C,) + HELD_C:
        split = "dev" if c == DEV_C else "heldout"; tag = f"c{str(c).replace('.', 'p')}"
        lean_tilt, lean_mu = lean_mu_threshold(c); thr = rim_threshold_polygon(c); thr_round = ps.rim_stable_offset(c)
        geom[tag] = {"clearance_mm": c, "peg_radius_mm": R_BORE - c, "rim_threshold_polygon_mm": round(thr, 4), "rim_threshold_round_bore_mm": round(thr_round, 4),
                     "max_geometric_lean_deg": round(lean_tilt, 3), "lean_rest_possible_above_mu": round(lean_mu, 3), "wedges": {}}

        def final_state(mu):
            lean = mu >= lean_mu / 1.1
            return {"centre_z_mm": 35.0, "tilt_deg": [0.0] + ([round(lean_tilt, 3)] if lean else []), "radial_mm_max": c, "at_rest": True,
                    "note": "flat on the base is always admissible" + ("; leaning at the largest geometric tilt is also statically admissible at this mu" if lean else "; flat is the only rest state")}

        # ---- W: wedge friction ladder (largest integer tilt whose bottom corner is still >= 1 mm inside the bore)
        for name, depth in (("shallow", 1.0),):      # deeper wedges have no unique-hold friction at 0.1-0.3 mm clearance (hold is never the only consistent mode), so only the shallow wedge is used
            tilt = wedge_tilt_for_depth(c, depth); pose, p = wedge_pose_dict(c, tilt); ms, lo, hi = hold_window(c, tilt)
            fe = ps.free_entry_tilt(c); path = [ps.mu_star(c, float(t)) for t in np.arange(fe + 0.05, tilt + 1e-9, 0.25) if ps.wedge_pose(c, float(t))["valid"]]
            ms_path = min(path + [ms])
            geom[tag]["wedges"][name] = {"tilt_deg": tilt, "corner_depth_below_rim_mm": round(float(p["A_depth_below_rim"]), 3), "mu_star_static": round(ms, 4),
                                         "mu_star_min_along_slide_path": round(ms_path, 4), "unique_hold_window": [round(lo, 4), round(hi, 4)],
                                         "admissible_flip_band": [round(ms_path, 4), round(lo, 4)], "band_ratio": round(lo / ms_path, 2)}
            ins = [f * ms_path for f in SLIDE_LADDER]
            band = [math.sqrt(ms * lo)]                                # one point in the middle of the non-unique band
            held = sorted({round(min(f * lo, hi / 1.25), 6) for f in HOLD_LADDER})
            for k, mu in enumerate(ins):
                assert ps.certain_slide(c, tilt, mu) and 1.2 * mu < ms_path, (c, tilt, mu)
                cells.append(base(f"W_{tag}_{name}_slide{k + 1}", c, mu, "wedge", "inserted", pose, {"mu_over_mu_star": round(mu / ms, 3), "reason": "no static solution even at 1.2 mu, at release and at every tilt on the way down"}, split,
                                  geometry=f"{tag}_{name}", final_state_set=final_state(mu)))
            for k, mu in enumerate(band):
                modes = ps.release_modes(c, tilt, float(mu)); assert ("stick", "stick") in modes and modes != STICK, (c, tilt, mu, modes)
                cells.append(base(f"W_{tag}_{name}_band{k + 1}", c, mu, "wedge", None, pose, {"mu_over_mu_star": round(mu / ms, 3), "consistent_release_modes": modes,
                                  "reason": "rigid-body + Coulomb is non-unique here (hold and slide are both consistent); recorded to locate the flip, never scored"}, split, geometry=f"{tag}_{name}"))
            for k, mu in enumerate(held):
                assert ps.release_modes(c, tilt, mu / 1.2) == STICK and ps.release_modes(c, tilt, mu * 1.2) == STICK, (c, tilt, mu)
                cells.append(base(f"W_{tag}_{name}_hold{k + 1}", c, mu, "wedge", "held_in_wedge", pose, {"mu_over_unique_hold_lo": round(mu / lo, 3), "reason": "(stick, stick) is the only consistent mode for every mu in [mu/1.2, 1.2 mu]"}, split,
                                  geometry=f"{tag}_{name}", final_state_set={"same_as_release": True, "displacement_mm": 0.0, "tilt_change_deg": 0.0}))
            if name == "shallow":                                                                    # ---- L: load scaling on the mid hold cell
                mu = held[min(1, len(held) - 1)]; hold_id = f"W_{tag}_{name}_hold{min(1, len(held) - 1) + 1}"
                for g in G_SCALES:
                    cells.append(base(f"I_{tag}_wedgehold_g{g}", c, mu, "invariance", "held_in_wedge", pose, {"base_cell": hold_id, "reason": "the release-mode analysis is homogeneous in the load: scaling gravity cannot change the label"}, split,
                                      geometry=f"{tag}_{name}", gravity_scale=g, final_state_set={"same_as_release": True, "displacement_mm": 0.0, "tilt_change_deg": 0.0}))
                if split == "dev":
                    for s in MASS_SCALES[1:]:
                        cells.append(base(f"I_{tag}_wedgehold_m{s:g}", c, mu, "invariance", "held_in_wedge", pose, {"base_cell": hold_id, "reason": "rigid-body outcome is independent of mass (inertia scales with it)"}, split,
                                          geometry=f"{tag}_{name}", mass_kg=MASS_KG * s, final_state_set={"same_as_release": True, "displacement_mm": 0.0, "tilt_change_deg": 0.0}))
                    for s in MASS_SCALES[:1]:
                        cells.append(base(f"I_{tag}_wedgeslide_m{s:g}", c, ins[1], "invariance", "inserted", pose, {"base_cell": f"W_{tag}_{name}_slide2", "reason": "rigid-body outcome is independent of mass (inertia scales with it)"}, split,
                                          geometry=f"{tag}_{name}", mass_kg=MASS_KG * s, final_state_set=final_state(ins[1])))
                for lab, mu_m, base_id, exp in ((("hold", mu, hold_id, "held_in_wedge"), ("slide", ins[1], f"W_{tag}_{name}_slide2", "inserted")) if split == "dev" else ()):
                    cells.append(base(f"I_{tag}_wedge{lab}_mirror", c, mu_m, "invariance", exp, pose, {"base_cell": base_id, "reason": "mirror image of the base cell (tilt plane rotated by 180 deg onto the opposite flat face)"}, split,
                                      geometry=f"{tag}_{name}", azimuth_deg=AZ + 180.0, final_state_set=final_state(mu_m) if exp == "inserted" else {"same_as_release": True, "displacement_mm": 0.0, "tilt_change_deg": 0.0}))

        # ---- R: rim offset sweep (friction-independent label -> one mu)
        for ratio in RIM_RATIOS_BELOW + RIM_RATIOS_ABOVE:
            d = ratio * thr; r = R_BORE - c
            if d > R_BORE - 1.0:
                continue
            pose = {"centre_inplane_mm": round(d, 4), "centre_z_mm": Z_TOP + LP / 2, "tilt_deg": 0.0}
            stable = ratio > 1; rid = f"R_{tag}_x{ratio:.2f}".replace(".", "p")
            cells.append(base(rid, c, MU_NEUTRAL, "rim_rest", "held_on_rim" if stable else "tips_off_rim", pose,
                              {"offset_over_threshold": ratio, "support_margin_mm": round(d - thr, 4), "reason": "upright peg on the rim top: stable iff its centre lies outside the innermost point of the support region (polygon-exact threshold)"},
                              split, geometry=f"{tag}_rim", final_state_set={"same_as_release": True, "displacement_mm": 0.0, "tilt_change_deg": 0.0} if stable else {"leaves_release_pose": True, "end_state": "not labelled"}))
        for ratio in (2.2,):                                                                          # ---- I: load scaling on one rim hold
            d = ratio * thr; pose = {"centre_inplane_mm": round(d, 4), "centre_z_mm": Z_TOP + LP / 2, "tilt_deg": 0.0}
            for g in G_SCALES[-1:]:
                cells.append(base(f"I_{tag}_rim_x{ratio:.2f}_g{g}".replace(".", "p"), c, MU_NEUTRAL, "invariance", "held_on_rim", pose, {"base_cell": f"R_{tag}_x{ratio:.2f}".replace(".", "p"), "reason": "static stability does not depend on the load magnitude"},
                                  split, geometry=f"{tag}_rim", gravity_scale=g, final_state_set={"same_as_release": True, "displacement_mm": 0.0, "tilt_change_deg": 0.0}))

        # ---- E: entry map (drop from 2 mm above the rim with a lateral offset or a tilt)
        r = R_BORE - c; fe = ps.free_entry_tilt(c)

        def entry_pose(bottom_offset, tilt):
            t = math.radians(tilt); zb = Z_TOP + 2.0 + r * math.sin(t)
            return {"centre_inplane_mm": round(bottom_offset - LP / 2 * math.sin(t), 4), "centre_z_mm": round(zb + LP / 2 * math.cos(t), 4), "tilt_deg": round(tilt, 4)}
        fs_any = {"must_be_a_rigid_equilibrium": True, "note": "theory does not single out one end state here; the final pose must still pass the equilibrium check"}
        scan = [o for o in np.linspace(-c, c, 401) if ps.free_entry_clear(c, float(o), 0.5 * fe, margin=0.25 * c)[0]]; assert scan, c
        tilt_off = float(0.5 * (scan[0] + scan[-1]))                                                    # mid-point of the bottom offsets that keep 0.25 c from both walls
        for lab, off, tilt in (("off0p5c", 0.5 * c, 0.0), ("tilt0p5fe", tilt_off, 0.5 * fe)):           # certain: clears both walls all the way down
            ok, _ = ps.free_entry_clear(c, off, tilt, margin=0.25 * c); assert ok, (c, off, tilt)
            cells.append(base(f"E_{tag}_in_{lab}", c, MU_NEUTRAL, "entry_map", "inserted", entry_pose(off, tilt), {"bottom_offset_mm": round(off, 4), "wall_margin_mm": round(0.25 * c, 4),
                              "reason": "lowered vertically the peg stays at least 0.25 x clearance from both walls down to the base, so no two-point contact can form"}, split, geometry=f"{tag}_entry", final_state_set=final_state(MU_NEUTRAL)))
        cells.append(base(f"E_{tag}_out_x1p5", c, MU_NEUTRAL, "entry_map", "held_on_rim", entry_pose(1.5 * thr, 0.0), {"offset_over_threshold": 1.5,
                          "reason": "upright peg dropped 2 mm flat onto the rim, centre 1.5 x beyond the stability threshold: a flat vertical impact gives no horizontal impulse, so it rests there"}, split, geometry=f"{tag}_entry",
                          final_state_set={"upright_on_rim": True, "centre_z_mm": Z_TOP + LP / 2}))
        for mu in (0.1, 0.5):                                                                           # uncertain: recorded, universal checks only
            for f in (0.5,):
                off = c + f * (thr - c)
                cells.append(base(f"E_{tag}_rimland_f{f:.2f}_mu{mu}".replace(".", "p"), c, mu, "entry_map", None, entry_pose(off, 0.0), {"bottom_offset_mm": round(off, 4), "fraction_from_clearance_to_rim_threshold": f,
                                  "reason": "one edge lands on the rim with the centre over the bore: the peg tips in and then slides or wedges depending on friction and impact; no unique rigid prediction"}, split, geometry=f"{tag}_entry", final_state_set=fs_any))
            for k in (2.0,):
                cells.append(base(f"E_{tag}_tilt_{k:g}fe_mu{mu}".replace(".", "p"), c, mu, "entry_map", None, entry_pose(0.0, k * fe), {"tilt_over_free_entry_tilt": k,
                                  "reason": "tilt beyond the free-entry limit: two-point contact forms on the way down; no unique rigid prediction"}, split, geometry=f"{tag}_entry", final_state_set=fs_any))

        # ---- C: free-entry control (not part of any headline number)
        pose = {"centre_inplane_mm": 0.0, "centre_z_mm": Z_TOP + 2.0 + LP / 2, "tilt_deg": 0.0}
        cells.append(base(f"C_{tag}_free_centred", c, MU_NEUTRAL, "control_free_entry", "inserted", pose, {"reason": "centred upright peg 2 mm above the rim: free fall of 32 mm to the base",
                          "time_to_base_ms": round(math.sqrt(2 * 0.032 / 9.81) * 1e3, 2), "impact_speed_m_s": round(math.sqrt(2 * 9.81 * 0.032), 4)}, split, geometry=f"{tag}_free", final_state_set=final_state(MU_NEUTRAL)))
    return cells, geom


RULES = {
    "unit_of_analysis": "the CONDITION, not the rollout. R = 8 rollouts per condition (perturbation U(+-0.05 mm) perpendicular to the tilt plane, identical arrays for all engines) exist only to detect instability. A condition is 'correct' iff every rollout of every repeat matches the label, 'wrong' iff none does, 'unstable' otherwise. No rollout-level totals or rollout-level confidence intervals are reported.",
    "outcome_taxonomy": {
        "held": "max drop <= 5 mm AND max tilt change <= 5 deg AND settled",
        "seated_settled": "final centre z <= 36 mm AND settled",
        "seated_unsettled": "final centre z <= 36 mm AND NOT settled (reported separately from never reaching the seat)",
        "partial": "inside or on the socket, neither held nor seated",
        "tipped_or_ejected": "rim cells that left the release pose without seating, or centre radial > 50 mm",
        "invalid": "non-finite state, contact-buffer truncation, or tunnelling through the socket"},
    "settled": "over the last 0.5 s: centre z range <= 0.5 mm, centre speed <= 5 mm/s AND tilt range <= 0.5 deg (v6 ignored rotation). Creep rate (mm/s) and tilt rate (deg/s) over that window are reported for every rollout.",
    "correct_rule": {
        "inserted": "seated_settled AND final radial <= clearance + 0.01 mm AND final tilt within 0.5 deg of a member of final_state_set.tilt_deg (flat only, or flat / leaning where the cell's mu makes a leaning rest admissible)",
        "held_in_wedge / held_on_rim": "held",
        "tips_off_rim": "NOT held",
        "unlabelled (expected = null)": "never scored; outcome recorded to locate the flip"},
    "fidelity_metrics_reported_for_every_condition": "max displacement and max tilt change (analytic value 0 for every 'held' label), strict-hold count (<= 0.5 mm and <= 0.5 deg), final z / tilt / radial excess for seated pegs, time from release to first base contact for control cells (analytic 80.8 ms). These are reported next to the label result and are never hidden behind a tolerance.",
    "penetration": "computed from the peg POSE by one shared routine against the analytic socket (base plane, 32-gon bore, rim top), every step, identically for all engines; split by region (base / bore wall / rim top), not by contact normal. Reported: peak, 99th percentile over time, time above 0.1 mm. Engine-reported contact depth is kept only as a secondary column; any engine contact deeper than pose-based penetration + 1 mm is flagged as a spurious contact and counted.",
    "universal_checks_every_rollout": {
        "non_penetration": "pose-based penetration (see 'penetration'): distance of the peg pose outside the set of non-penetrating poses, in mm and as a fraction of the clearance",
        "energy": "m g z + kinetic energy (from the logged per-step pose) must never exceed its initial value by more than 1 % of m g x 32 mm; violations are counted and the largest gain reported",
        "admissible_rest": "the final pose is classified against the rigid equilibrium classes: seated flat; seated leaning (allowed only if mu >= lean_rest_possible_above_mu); two-point wedge at the final tilt (allowed only if mu >= mu*(final tilt)); upright on the rim (allowed only if the final centre offset >= the rim threshold); lying on its side on the socket top; left the socket. A rest in a class the rigid model forbids at this mu is an 'inadmissible rest'; a pose matching no class is listed as 'unclassified'. This is the only correctness statement made for unlabelled conditions."},
    "criteria_freeze": "every threshold above is fixed at declaration. Smoke tests may use the selection conditions only. If a label is contested after the run, the condition is dropped for all engines, never relabelled. Any other change makes a new revision."}

LADDER = {
    "principle": "candidates are defined by a measured observable, not by engine parameters, so both engines search the same range of regularisation. Each rung is one fixed configuration.",
    "calibration_cell": {"clearance_mm": 1.0, "pose": {"centre_inplane_mm": 0.0, "centre_z_mm": 67.0, "tilt_deg": 0.0}, "mu": MU_NEUTRAL, "dt_ms": DT_MS, "note": "32-mm centred drop; not a scored condition"},
    "rungs_target_peak_base_penetration_mm": [0.15, 0.3, 0.6, 1.2],
    "tolerance": "pose-based peak base penetration within 10 % of the target on the calibration cell",
    "damping_alignment": "after first base contact the peg must stay within 0.05 mm of its final height from 0.3 s onward (no long ringing)",
    "sap_warp": "bisect ke; tau = smallest of {1.5, 3, 6, 12, 24} ms that meets damping_alignment; gap = 0.1 mm",
    "mujoco_warp": "bisect solref timeconst with dampratio = 1; solimp = 0.95 0.95 0.001 0.5 2; integrator = implicitfast, cone = elliptic, solver = Newton. timeconst < 2 * dt is not allowed (MuJoCo's own limit): such a rung is recorded as 'unreachable at this dt' and is not replaced.",
    "native_default": "each engine's literal defaults (candidate 1, B = 1)"}

SELECTION_IDS = ["W_c0p3_shallow_slide2", "W_c0p3_shallow_hold1", "R_c0p3_x1p10"]
ROBUST_SUFFIXES = ["shallow_slide2", "shallow_hold1", "x1p10", "in_tilt0p5fe"]     # per held-out clearance: one must-slide, one must-stay, one rim hold, one tilted entry

PLAN = {
    "structure": "task_runner_sop.md phases: B = 1 audit, B = 5 selection on a few selection-only conditions, then the frozen configuration on the disjoint frozen matrix. Candidates are never run on the frozen matrix.",
    "1_selection": {"conditions": SELECTION_IDS,
                    "why_these": "three labelled conditions at the dev clearance (0.3 mm), each close to an analytic boundary and together pulling in opposite directions: a wedge that must slide in and seat (0.8 mu*), a wedge that must stay (1.25 x unique-hold edge), a rim rest just outside the stability threshold (x1.10). A configuration that is too soft or too sticky fails at least one of them.",
                    "candidates_in_declared_order": ["native default", "rung 0.3 mm", "rung 0.6 mm", "rung 0.15 mm", "rung 1.2 mm"],
                    "selection_rule": "most selection conditions 'correct'; then fewest 'unstable'; then fewest inadmissible rests; then lowest mean pose-based peak penetration over the selection conditions; then declared order. B = k is the same rule over the first k candidates (kept in the raw artifact); B = 1 is the native default, B = 5 the frozen configuration.",
                    "smoke": "code smoke tests may use these selection conditions only; no frozen condition is run before the configuration is frozen"},
    "2_frozen_matrix": {"conditions": "every condition with role = 'frozen' (same clearance as selection: transfer to other poses / frictions / loads; clearances 0.1 and 1.0 mm: transfer to unseen geometry). Reported separately for the two groups.",
                        "configurations": ["frozen B = 5 winner", "native default (B = 1), unless it is the winner"]},
    "3_robustness": {"dt": "frozen B = 5 at dt = 0.25 and 1.0 ms on the robustness subset (a configuration that violates an engine limit at that dt is recorded as not runnable, not retuned)",
                     "repeat": "frozen B = 5 run a second time on the robustness subset to expose run-to-run non-determinism"},
    "4_attribution_control": "MuJoCo only, selection conditions only: the 0.6-mm rung's solref with native integrator / cone / solver, to separate stiffness from solver options"}

REPORTS = {
    "0_selection_table": "per engine: all five candidates on the selection conditions (correct / wrong / unstable, pose-based penetration), the frozen index and what changed from B = 1.",
    "1_hardness_correctness_curve": "selection conditions only (three conditions, descriptive): outcome versus measured pose-based penetration for the five candidates, both engines on the same axis. Next to it: stop time after first base contact on the calibration cell, against 2 ms and 10 ms marks.",
    "2_frozen_result": "B = 1 and frozen B = 5 on the frozen matrix: labelled conditions correct / wrong / unstable per block, split into same-clearance and unseen-clearance groups; inadmissible rests on every condition including the unlabelled ones.",
    "3_boundary_location": "W: per engine and clearance, which of the five friction points slide / hold and whether the flip lies inside admissible_flip_band [mu*, unique-hold lower edge]. Non-monotone sweeps are flagged. R: the offset interval containing the flip, and its distance from the analytic threshold in mm and in units of the clearance.",
    "4_invariance": "I: largest gravity scale (1, 8, 64) and mass scale at which the outcome still equals the base cell's; whether the mirror image gives the same outcome.",
    "5_mechanism_diagnostics": "R: innermost peg contact point two steps after release versus the analytic support edge (support-edge error, mm). All blocks: spurious-contact count; pose-based versus engine-reported penetration; for MuJoCo the residual against kappa * v * timeconst.",
    "6_robustness": "dt = 0.25 / 0.5 / 1.0 ms and the repeat run on the robustness subset."}

NOT_COVERED = [
    "Geometric interference (a peg larger than the bore being squeezed in): with a flat-ended peg and a sharp-edged bore the block is face-on-face, so it cannot probe bore-wall penetration. It needs a tapered or chamfered asset; block L (load scaling of self-locked holds) is the available substitute.",
    "Slide-phase trajectory reference, restitution, impact force and duration (levels 2-4): out of scope by decision; restitution is covered by the bounce tasks.",
    "Free fall only: no gripper compliance, no applied insertion force, no controller. Impact at 0.2-0.8 m/s is not how a robot inserts; static penetration under a 10-50 N push is not exercised except through gravity scaling in block I. A compliantly held, driven insertion is the planned next task.",
    "Blindness: clearances 0.1 and 0.3 mm were never run (1.0 mm was, with other tilts and frictions), and the blocks were designed knowing v6's findings (SAP's rim support gap, MuJoCo's soft-contact behaviour). This is disclosed, not removable.",
    "W resolution: the rigid model itself is non-unique over a factor 2-5 in mu between mu* and the unique-hold edge, so a flip anywhere inside that band is consistent with theory. Only R has a sharp analytic boundary.",
    "No real data: every reference is the rigid-body Coulomb limit."]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, default=REPO_ROOT / "tasks/simulation-only-composite/round-socket/manifests/round_socket_strict_v8_draft.json")
    a = ap.parse_args(); cells, geom = build()
    ids = {c["id"] for c in cells}; assert set(SELECTION_IDS) <= ids, set(SELECTION_IDS) - ids
    robust = [c["id"] for c in cells if c["split"] == "heldout" and any(c["id"].endswith(sfx) for sfx in ROBUST_SUFFIXES)]
    assert len(robust) == 2 * len(ROBUST_SUFFIXES), robust
    for c in cells:
        c["role"] = "selection" if c["id"] in SELECTION_IDS else "frozen"
        assert c["role"] == "frozen" or c["expected"] is not None
    PLAN["3_robustness"]["conditions"] = robust
    n_sel, n_fro = len(SELECTION_IDS), len(cells) - len(SELECTION_IDS)
    runs = {"selection": n_sel * 5, "frozen_B5": n_fro, "frozen_B1_native": n_fro, "robustness_dt": 2 * len(robust), "robustness_repeat": len(robust), "mujoco_attribution_control": n_sel}
    runs["total_sap"] = sum(v for k, v in runs.items() if k != "mujoco_attribution_control"); runs["total_mujoco"] = runs["total_sap"] + n_sel
    man = {"protocol": "aligned-frozen-generalizability-v1", "task": "round-socket", "revision": "strict-v8-draft",
           "status": "DRAFT 2026-11-15: not declared, not run. Needs runner support (see implementation_required) and the open decisions before declaration.",
           "supersedes": "none yet. strict-v6 results and the strict-v7 draft (v6 cells with two rule fixes) are kept. v8 is a redesign after the v6 audit.",
           "generator": "tasks/simulation-only-composite/round-socket/prep/build_round_socket_v8_manifest.py (labels from tasks/simulation-only-composite/round-socket/peg_hole_statics.py)",
           "R": 8, "seed0": 88100, "seed_rule": "condition j (order of this file): seed0 + j; the same perturbation array for every engine, rung and repeat",
           "engines": {"default": ["SAP Warp (internal, fix branch)", "MuJoCo Warp 4.0"], "open_decision": "whether MuJoCo Warp 3.17 is also run (v6 found ~0.3 % of 4.0 rollouts with a ~60-mm spurious contact, none in 3.17)"},
           "design_rule": "every test is a provable statement of the rigid-body Coulomb model; points are placed by margin to an analytic boundary; anything the theory cannot decide is recorded but not scored",
           "fixed": {"geometry": "assets/round_socket_exact (32-gon bore, inscribed radius 12.000 mm, base z 0-10 mm, wall z 10-40 mm, outer radius 50 mm); peg = cylinder radius 12 mm - clearance, half-length 25 mm, solid-cylinder inertia",
                     "frame": "all poses in the tilt plane through a flat-face centre of the bore (azimuth 5.625 deg); centre_inplane_mm along that direction; tilt_deg leans the peg top toward the negative in-plane direction",
                     "release": "at rest, gravity only (gravity = 9.81 m/s^2 x gravity_scale), 3-s run. W: exact two-point contact. R: bottom face on the rim top. C: lowest point 2 mm above the rim top",
                     "dt_ms": DT_MS, "radial_tolerance_mm": 0.01},
           "rules": RULES, "regularisation_ladder": LADDER, "run_plan": PLAN, "cell_runs_per_engine": runs, "reports": REPORTS, "not_covered": NOT_COVERED,
           "implementation_required": ["socket_v8_common.py: outcome taxonomy, condition-level scoring, flip location, shared pose-based penetration",
                                       "both runners: per-step pose logging, gravity_scale, mass_kg and azimuth per condition, calibration stage for the ladder, selection -> freeze -> frozen-matrix driver, contact-point dump for block R, engine-contact-depth column",
                                       "optional: batch conditions that share a model (R offsets, L scales where the engine allows) as worlds; must be shown on dev to leave results unchanged"],
           "geometry_summary": geom,
           "counts": {"conditions": len(cells), "by_block": {b: sum(c["block"] == b for c in cells) for b in "EWRIC"}, "by_split": {s: sum(c["split"] == s for c in cells) for s in ("dev", "heldout")},
                      "by_role": {"selection": n_sel, "frozen_same_clearance": sum(c["role"] == "frozen" and c["split"] == "dev" for c in cells), "frozen_unseen_clearance": sum(c["split"] == "heldout" for c in cells)},
                      "labelled": sum(c["expected"] is not None for c in cells), "unlabelled_band": sum(c["expected"] is None for c in cells)},
           "conditions": cells}
    a.out.write_text(json.dumps(man, indent=1) + "\n"); print("wrote", a.out); print(json.dumps(man["counts"])); print(json.dumps(runs)); print("robustness subset:", robust)


if __name__ == "__main__":
    main()
