"""Shared logic for the round-socket strict-v8 run (DRAFT manifest; SAP Warp and MuJoCo Warp).

Everything engine-independent lives here so both engines are scored by the same code:
  * pose-based penetration (one routine, against the analytic socket: base slab, 32-gon bore wall, rim top, rim edge)
  * per-rollout trajectory metrics from the per-step pose (energy, settled, creep, drop, tilt, first base contact)
  * outcome taxonomy, label matching, condition-level verdict (correct / wrong / unstable)
  * admissible-rest classification of the final pose against the rigid equilibrium classes
  * regularisation-ladder calibration (bisection on an engine parameter, observable = pose-based peak base penetration)
  * run plan: calibration -> selection (5 candidates x 3 conditions) -> freeze (B = 1 .. 5) -> frozen matrix (B = 5 and B = 1)
    -> robustness (dt 0.25 / 1.0 ms, repeat) -> MuJoCo attribution control

Engine interface (provided by the runner):
  simulate(cand, cell, init_xy [R, 2] m, dump_contacts: bool) -> dict
      pos [R, S, 3] m and quat [R, S, 4] wxyz, the peg pose at every step t = k dt, k = 0 .. S - 1 (S = steps + 1)
      engine_depth [R, S] m: deepest engine-reported peg contact computed at pose k (>= 0; nan where not available)
      truncated: int (> 0 if any contact buffer overflowed)
      contacts (only if dump_contacts): list over rollouts of [n, 8] arrays (px py pz nx ny nz depth fn) in m / N, from the
      contact set used in the second step after release
  runnable(cand, dt_s) -> (bool, reason)
  ladder: dict(param_name, lo, hi, increasing (pen increases with the parameter), log (bisect in log space),
               make(param, extra) -> cand, extras: list of extra settings tried in order (SAP: tau), 'native': cand,
               'attribution'(cand) -> cand or None)

Choices the manifest leaves open are fixed here as module constants and written into every result file (INTERPRETATION).
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import json
import math
import time
from pathlib import Path

import numpy as np

import peg_hole_statics as ps
from strict_v3_common import json_safe, load_manifest  # noqa: F401

ASSET_DIR = "assets/round_socket_exact"
FILES = ["socket_base.obj"] + [f"socket_wall_{k:02d}.obj" for k in range(32)]

# --- socket geometry (mm), verified against assets/round_socket_exact: 32-gon bore inscribed 12.000 (vertices 12.058), outer
# 32-gon inscribed 50.000, face normals at 5.625 + k * 11.25 deg, base z 0-10, wall z 10-40
N_SIDES, FACE0, BORE_IN, OUTER_IN, Z_BASE, Z_TOP, PEG_HALF = 32, math.radians(5.625), 12.0, 50.0, 10.0, 40.0, 25.0
DTH = 2 * math.pi / N_SIDES
BORE_VERT = BORE_IN / math.cos(DTH / 2); OUTER_VERT = OUTER_IN / math.cos(DTH / 2)

# --- thresholds stated by the manifest
HELD_DROP_MM, HELD_TILT_DEG = 5.0, 5.0
SEAT_Z_MM = 36.0
SETTLE_WINDOW_S, SETTLE_Z_MM, SETTLE_V_MM_S, SETTLE_TILT_DEG = 0.5, 0.5, 5.0, 0.5
STRICT_DISP_MM, STRICT_TILT_DEG = 0.5, 0.5
TILT_MATCH_DEG = 0.5
EJECT_RADIAL_MM = 50.0
TUNNEL_RADIAL_MM = 38.0          # centre below z = 0 this far inside the outer radius can only be reached through the socket
ENERGY_FRAC, ENERGY_DROP_M = 0.01, 0.032
SPURIOUS_MM = 1.0
PEN_TIME_ABOVE_MM = 0.1
CALIB_TOL = 0.10
DAMP_BAND_MM, DAMP_FROM_S = 0.05, 0.3
SAP_TAUS_MS = (1.5, 3.0, 6.0, 12.0, 24.0)

# --- interpretation of points the manifest leaves open (written into every result)
SPEED_SAMPLE_S = 0.01              # settled: centre speed from 10-ms differences (per-step differences measure contact jitter, not motion)
REST_TOL = {"seated_lean_deg": 0.5, "rim_dz_mm": 1.0, "rim_tilt_deg": 1.0, "lying_tilt_deg": 85.0, "lying_dz_mm": 1.5, "wedge_gap_mm": 0.25}
CALIB_BISECT_S = 0.4               # peak base penetration of the 32-mm drop happens at ~81-85 ms; full 3-s run only for damping check
TRAJ_SAVE_S = 0.005                # stored trajectory resolution (metrics use every step)
INTERPRETATION = {
    "speed_sample_s": SPEED_SAMPLE_S,
    "rest_class_tolerances": REST_TOL,
    "calibration": "single rollout, no perturbation; pose-based peak base penetration from a 0.4-s run during bisection, then one full 3-s run "
                   "for the damping check. Damping window: t >= 0.3 s after release. SAP: for each tau in {1.5,3,6,12,24} ms in order, bisect ke "
                   "(log space) to the target; the first tau whose 3-s run passes damping is the rung. MuJoCo: bisect timeconst (log space) in "
                   "[2 dt, 0.2 s] with dampratio 1; damping is recorded (dampratio is fixed, so a failure has no remedy). A rung whose target "
                   "is out of reach is 'unreachable' and not replaced.",
    "penetration": "peg surface sampled at 180 angles x 26 heights on the side (end circles included) plus the bore top edge and outer top edge "
                   "(32 x 24 points each) tested inside the peg; depth = distance to exit the socket solid (or the peg). Regions: base (z < 10), "
                   "bore_wall, rim_top (exit through z = 40 or the outer face), rim_edge (bore top edge inside the peg). Sampling error <= 2 um.",
    "energy": "E = m g_scaled z + 0.5 m v^2 + 0.5 w.I w from forward differences of consecutive step poses; limit 1 % of m g_scaled 32 mm",
    "outcome_priority": "invalid > held > seated_settled > seated_unsettled > tipped_or_ejected > partial",
    "tipped_or_ejected": "centre radial > 50 mm (any cell), or a rim cell (expected held_on_rim / tips_off_rim) that is neither held nor seated",
    "tunnelling": "centre below z = 0 (or final below -10 mm) at radial <= 38 mm (outer radius minus peg radius); a peg that rolls off the outer edge is tipped_or_ejected",
    "seated": "final centre z <= 36 mm AND final radial <= 12 mm (inside the bore)",
    "verdict": "frozen matrix verdicts use the first run only; the repeat run is reported in robustness",
    "tips_off_rim": "an invalid rollout does not count as matching any label",
    "attribution_control": "0.6-mm rung solref and solimp kept; integrator / cone / solver set to MuJoCo defaults",
    "support_edge": "R and rim-invariance cells: innermost (smallest in-plane coordinate) peg contact with normal force > 0 in the second step, minus "
                    "the analytic innermost point of the peg disc outside the polygonal bore",
}


# ------------------------------------------------------------------------------------------------ cell helpers
def peg_radius_mm(cell):
    return BORE_IN - cell["clearance_mm"]


def peg_mass(cell):
    return float(cell.get("mass_kg", 0.08))


def peg_inertia(cell):
    r = peg_radius_mm(cell) * 1e-3; h = 2 * PEG_HALF * 1e-3; m = peg_mass(cell)
    return (m * (3 * r * r + h * h) / 12, m * (3 * r * r + h * h) / 12, m * r * r / 2)


def gravity(cell):
    return 9.81 * float(cell.get("gravity_scale", 1))


def _qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def initial_pose(cell):
    """Centre (m) and wxyz quaternion: in-plane direction e = (cos az, sin az, 0), axis (-sin(tilt) e, cos(tilt)), R = Rz(az) Ry(-tilt)."""
    az = math.radians(cell.get("azimuth_deg", 0.0)); t = math.radians(cell["pose"]["tilt_deg"])
    e = np.array([math.cos(az), math.sin(az), 0.0])
    pos = cell["pose"]["centre_inplane_mm"] * 1e-3 * e + np.array([0.0, 0.0, cell["pose"]["centre_z_mm"] * 1e-3])
    q = _qmul(np.array([math.cos(az / 2), 0, 0, math.sin(az / 2)]), np.array([math.cos(-t / 2), 0, math.sin(-t / 2), 0]))
    return pos, q / np.linalg.norm(q)


def perturbation(cell, seed, R):
    """U(+-0.05 mm) perpendicular to the tilt plane (same array for every engine, rung and repeat)."""
    az = math.radians(cell.get("azimuth_deg", 0.0)); s = np.random.default_rng(seed).uniform(-5e-5, 5e-5, R)
    return np.column_stack([-math.sin(az) * s, math.cos(az) * s])


def is_rim_cell(cell):
    return cell.get("expected") in ("held_on_rim", "tips_off_rim") or str(cell.get("geometry", "")).endswith("_rim")


def geom_tag(c):
    return {0.3: "c0p3", 0.1: "c0p1", 1.0: "c1p0"}.get(round(float(c), 4), f"c{str(c).replace('.', 'p')}")


# ------------------------------------------------------------------------------------------------ rotations
def quat_to_mat(q):
    """[N, 4] wxyz -> [N, 3, 3] (normalised)."""
    q = q / np.linalg.norm(q, axis=-1, keepdims=True); w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], -2)


def tilt_deg(q):
    w, x, y, z = (q[..., i] for i in range(4)); n = w * w + x * x + y * y + z * z
    return np.degrees(np.arccos(np.clip(np.abs(1.0 - 2.0 * (x * x + y * y) / n), 0.0, 1.0)))


# ------------------------------------------------------------------------------------------------ pose-based penetration
_N_ANG, _N_H, _N_EDGE = 180, 26, 24


def _peg_points(r):
    a = np.linspace(0, 2 * np.pi, _N_ANG, endpoint=False); h = np.linspace(-PEG_HALF, PEG_HALF, _N_H)
    A, H = np.meshgrid(a, h); return np.stack([r * np.cos(A).ravel(), r * np.sin(A).ravel(), H.ravel()], 1)


def _ring(vert_r):
    v = [np.array([vert_r * math.cos(k * DTH), vert_r * math.sin(k * DTH)]) for k in range(N_SIDES + 1)]   # vertices at k * 11.25 deg
    s = np.linspace(0, 1, _N_EDGE, endpoint=False)
    pts = np.concatenate([v[k][None] * (1 - s[:, None]) + v[k + 1][None] * s[:, None] for k in range(N_SIDES)])
    return np.column_stack([pts, np.full(len(pts), Z_TOP)])


_BORE_RING, _OUTER_RING = _ring(BORE_VERT), _ring(OUTER_VERT)


def _socket_depth(P):
    """P [..., 3] mm -> (depth >= 0, region code 0 base / 1 bore_wall / 2 rim_top) of each point inside the socket solid."""
    x, y, z = P[..., 0], P[..., 1], P[..., 2]
    rho = np.hypot(x, y); th = np.arctan2(y, x); k = np.round((th - FACE0) / DTH); proj = rho * np.cos(th - (FACE0 + k * DTH))
    bore_out = proj - BORE_IN; outer_in = OUTER_IN - proj
    in_base = (z > 0) & (z < Z_BASE) & (outer_in > 0)
    under_wall = bore_out > 0
    d_base = np.minimum(np.minimum(z, outer_in), np.where(under_wall, np.minimum(np.hypot(bore_out, Z_BASE - z), Z_TOP - z), Z_BASE - z))
    in_wall = (z >= Z_BASE) & (z < Z_TOP) & under_wall & (outer_in > 0)
    d_top = np.minimum(Z_TOP - z, outer_in); d_wall = np.minimum(bore_out, d_top)
    depth = np.where(in_base, d_base, np.where(in_wall, d_wall, 0.0))
    region = np.where(in_base, 0, np.where(d_top <= bore_out, 2, 1))
    return np.maximum(depth, 0.0), region


def _pen_chunk(pos_mm, quat, r_mm, pb):
    out = {}; Rm = quat_to_mat(quat); c = pos_mm
    W = np.einsum("nij,pj->npi", Rm, pb) + c[:, None, :]
    d, reg = _socket_depth(W)
    for code, name in ((0, "base"), (1, "bore_wall"), (2, "rim_top")):
        out[name] = np.where(reg == code, d, 0.0).max(axis=1)
    for ring, name in ((_BORE_RING, "rim_edge"), (_OUTER_RING, "rim_top")):
        B = np.einsum("nji,npj->npi", Rm, ring[None, :, :] - c[:, None, :])      # socket edge points in the peg frame
        dp = np.minimum(r_mm - np.hypot(B[..., 0], B[..., 1]), PEG_HALF - np.abs(B[..., 2]))
        out[name] = np.maximum(out.get(name, 0.0), np.maximum(dp, 0.0).max(axis=1))
    return out


def pose_penetration(pos_mm, quat, r_mm, chunk=128, threads=8):
    """Per pose (arrays [N]): penetration depth (mm) by region: base, bore_wall, rim_top, rim_edge, and total = max."""
    from concurrent.futures import ThreadPoolExecutor
    N = len(pos_mm); pb = _peg_points(r_mm); names = ("base", "bore_wall", "rim_top", "rim_edge")
    sls = [slice(i, min(N, i + chunk)) for i in range(0, N, chunk)]
    with ThreadPoolExecutor(threads) as ex:
        parts = list(ex.map(lambda sl: _pen_chunk(np.asarray(pos_mm[sl], float), np.asarray(quat[sl], float), r_mm, pb), sls))
    out = {k: np.concatenate([p[k] for p in parts]) if parts else np.zeros(0) for k in names}
    out["total"] = np.maximum.reduce([out[k] for k in names])
    return out


# ------------------------------------------------------------------------------------------------ analytic references
def rim_support_xmin(c, d, n_phi=20001):
    """Innermost in-plane coordinate (mm) of the upright peg's support region (peg disc minus the polygonal bore), centre offset d."""
    r = BORE_IN - c; phi = np.linspace(0, 2 * np.pi, n_phi); a = FACE0
    px = d * math.cos(a) + r * np.cos(phi); py = d * math.sin(a) + r * np.sin(phi)
    th = np.arctan2(py, px); k = np.round((th - FACE0) / DTH); proj = np.hypot(px, py) * np.cos(th - (FACE0 + k * DTH))
    m = proj > BORE_IN
    return float((px[m] * math.cos(a) + py[m] * math.sin(a)).min()) if m.any() else None


_MU_STAR_CACHE: dict = {}


def mu_star_at(c, tilt):
    key = (round(c, 4), round(float(tilt), 2))
    if key not in _MU_STAR_CACHE:
        _MU_STAR_CACHE[key] = ps.mu_star(c, key[1])
    return _MU_STAR_CACHE[key]


def _poly_proj(xy):
    th = math.atan2(xy[1], xy[0]); k = round((th - FACE0) / DTH); return math.hypot(xy[0], xy[1]) * math.cos(th - (FACE0 + k * DTH))


def wedge_gaps(c, p_mm, q):
    """Signed gaps (mm, < 0 = penetration) of the two wedge contacts in the peg's own lean plane: A = lowest-side bottom corner vs the
    far bore wall, B = near bore top edge vs the peg side.  None if the peg does not lean."""
    r = BORE_IN - c; Rm = quat_to_mat(np.asarray(q, float)[None])[0]; a3 = Rm[:, 2]; h = a3[:2]; hn = np.linalg.norm(h)
    if hn < 1e-9:
        return None, None
    e2 = -h / hn; e = np.array([e2[0], e2[1], 0.0]); u = e - np.dot(e, a3) * a3; u /= np.linalg.norm(u)
    A = np.asarray(p_mm, float) - PEG_HALF * a3 + r * u
    gA = BORE_IN - _poly_proj(A[:2])
    th = math.atan2(-e2[1], -e2[0]); k = round((th - FACE0) / DTH); rb = BORE_IN / math.cos(th - (FACE0 + k * DTH))
    B = np.array([-rb * e2[0], -rb * e2[1], Z_TOP]); v = B - np.asarray(p_mm, float); ax = float(np.dot(v, a3))
    if abs(ax) > PEG_HALF:
        return gA, math.inf
    gB = float(np.linalg.norm(v - ax * a3) - r)
    return float(gA), gB


def classify_rest(cell, man, p_mm, q, settled):
    """Rigid equilibrium class of the final pose and whether the rigid model allows it at this mu."""
    c, mu = cell["clearance_mm"], cell["mu"]; g = man["geometry_summary"][geom_tag(c)]
    z, tl = float(p_mm[2]), float(tilt_deg(q)); rad = float(math.hypot(p_mm[0], p_mm[1])); r = BORE_IN - c
    if rad > EJECT_RADIAL_MM or z < 0:
        return "left_socket", True
    if not settled:
        return "not_at_rest", None
    if z <= SEAT_Z_MM and tl <= TILT_MATCH_DEG:
        return "seated_flat", True
    if z <= SEAT_Z_MM and abs(tl - g["max_geometric_lean_deg"]) <= REST_TOL["seated_lean_deg"]:
        return "seated_leaning", bool(mu >= g["lean_rest_possible_above_mu"])
    if abs(z - (Z_TOP + PEG_HALF)) <= REST_TOL["rim_dz_mm"] and tl <= REST_TOL["rim_tilt_deg"]:
        return "upright_on_rim", bool(rad >= g["rim_threshold_polygon_mm"])
    if tl >= REST_TOL["lying_tilt_deg"] and abs(z - (Z_TOP + r)) <= REST_TOL["lying_dz_mm"]:
        return "lying_on_socket_top", True
    if 0.5 < tl < 89.0:                                                  # two-point wedge: bottom corner on the far wall AND rim corner on the side
        gA, gB = wedge_gaps(c, p_mm, q)
        if gA is not None and abs(gA) <= REST_TOL["wedge_gap_mm"] and abs(gB) <= REST_TOL["wedge_gap_mm"]:
            ms = mu_star_at(c, tl) if ps.wedge_pose(c, tl)["valid"] else None
            return "two_point_wedge", bool(ms is not None and mu >= ms)
    return "unclassified", None


# ------------------------------------------------------------------------------------------------ per-rollout metrics
def rollout_metrics(cell, man, dt, pos, quat, eng_depth, truncated):
    """pos [S, 3] m, quat [S, 4], eng_depth [S] m -> metrics dict (mm, deg, s)."""
    exp = cell.get("expected"); c = cell["clearance_mm"]; P = pos * 1e3; S = len(P); t = np.arange(S) * dt
    row = {}
    if not (np.isfinite(P).all() and np.isfinite(quat).all()) or truncated:
        row.update(outcome="invalid", invalid_reason="non_finite" if not np.isfinite(P).all() else "contact_buffer_truncated", correct=False if exp else None,
                   rest_class="invalid", rest_admissible=None)
        return row, None
    rad_t = np.hypot(P[:, 0], P[:, 1]); zf, radf = float(P[-1, 2]), float(rad_t[-1])
    tunnel = (zf < -10.0 and radf <= TUNNEL_RADIAL_MM) or bool(((P[:, 2] < 0) & (rad_t <= TUNNEL_RADIAL_MM)).any())
    pen = pose_penetration(P, quat, peg_radius_mm(cell))
    tl = tilt_deg(quat)
    # --- energy (forward differences)
    m = peg_mass(cell); gs = gravity(cell); I = np.diag(peg_inertia(cell))
    v = np.diff(pos, axis=0) / dt; qn = quat / np.linalg.norm(quat, axis=1, keepdims=True)
    q0c = qn[:-1] * np.array([1, -1, -1, -1]); dq = _qmul_vec(qn[1:], q0c)
    sgn = np.where(dq[:, 0] < 0, -1.0, 1.0)[:, None]; w = 2 * dq[:, 1:4] * sgn / dt
    Rm = quat_to_mat(qn[:-1]); wb = np.einsum("nji,nj->ni", Rm, w)
    E = m * gs * 0.5 * (pos[1:, 2] + pos[:-1, 2]) + 0.5 * m * (v * v).sum(1) + 0.5 * (wb * wb * np.diag(I)[None]).sum(1)
    E0 = m * gs * pos[0, 2]; lim = ENERGY_FRAC * m * gs * ENERGY_DROP_M; gain = E - E0
    # --- settled / creep (last 0.5 s)
    nw = int(round(SETTLE_WINDOW_S / dt)); seg = P[-(nw + 1):]; tseg = t[-(nw + 1):]; tlseg = tl[-(nw + 1):]
    ks = max(1, int(round(SPEED_SAMPLE_S / dt))); sub = seg[::ks]
    vmax = float(np.linalg.norm(np.diff(sub, axis=0), axis=1).max() / (ks * dt)) if len(sub) > 1 else 0.0
    zr, trg = float(seg[:, 2].max() - seg[:, 2].min()), float(tlseg.max() - tlseg.min())
    settled = zr <= SETTLE_Z_MM and vmax <= SETTLE_V_MM_S and trg <= SETTLE_TILT_DEG
    creep = float(np.polyfit(tseg, seg[:, 2], 1)[0]); trate = float(np.polyfit(tseg, tlseg, 1)[0])
    drop = float(P[0, 2] - P[:, 2].min()); dtilt = float(np.abs(tl - tl[0]).max()); disp = float(np.linalg.norm(P - P[0], axis=1).max())
    held = drop <= HELD_DROP_MM and dtilt <= HELD_TILT_DEG and settled
    seated = zf <= SEAT_Z_MM and radf <= BORE_IN
    if tunnel:
        outcome = "invalid"
    elif held:
        outcome = "held"
    elif seated:
        outcome = "seated_settled" if settled else "seated_unsettled"
    elif radf > EJECT_RADIAL_MM or is_rim_cell(cell):
        outcome = "tipped_or_ejected"
    else:
        outcome = "partial"
    fs = cell.get("final_state_set") or {}
    if outcome == "invalid":
        correct = False if exp else None
    elif exp == "inserted":
        correct = bool(outcome == "seated_settled" and radf <= c + man["fixed"]["radial_tolerance_mm"]
                       and any(abs(float(tl[-1]) - tt) <= TILT_MATCH_DEG for tt in fs.get("tilt_deg", [0.0])))
    elif exp in ("held_in_wedge", "held_on_rim"):
        correct = outcome == "held"
    elif exp == "tips_off_rim":
        correct = outcome != "held"
    else:
        correct = None
    rest, adm = ("invalid", None) if outcome == "invalid" else classify_rest(cell, man, P[-1], quat[-1], settled)
    ed = np.nan_to_num(eng_depth * 1e3, nan=0.0); spur = ed > pen["total"] + SPURIOUS_MM
    base_hit = np.nonzero(pen["base"] > 1e-4)[0]
    row.update(outcome=outcome, invalid_reason="tunnelled" if tunnel else None, correct=correct, settled=bool(settled), held=bool(held),
               strict_hold=bool(disp <= STRICT_DISP_MM and dtilt <= STRICT_TILT_DEG and settled),
               final_z_mm=zf, final_radial_mm=radf, final_tilt_deg=float(tl[-1]), radial_excess_um=max(0.0, radf - c) * 1e3 if seated else None,
               max_drop_mm=drop, max_tilt_change_deg=dtilt, max_displacement_mm=disp,
               settle_z_range_mm=zr, settle_speed_max_mm_s=vmax, settle_tilt_range_deg=trg, creep_rate_mm_s=creep, tilt_rate_deg_s=trate,
               energy_violation_steps=int((gain > lim).sum()), energy_max_gain_J=float(gain.max()), energy_limit_J=lim,
               first_base_contact_ms=float(t[base_hit[0]] * 1e3) if len(base_hit) else None,
               rest_class=rest, rest_admissible=adm, inadmissible_rest=adm is False, unclassified_rest=rest == "unclassified",
               spurious_contact_steps=int(spur.sum()), engine_depth_max_mm=float(ed.max()))
    for k in ("base", "bore_wall", "rim_top", "rim_edge", "total"):
        row[f"pen_{k}_peak_mm"] = float(pen[k].max())
    row["pen_total_p99_mm"] = float(np.percentile(pen["total"], 99)); row["pen_time_above_0p1mm_s"] = float((pen["total"] > PEN_TIME_ABOVE_MM).sum() * dt)
    row["pen_over_clearance"] = row["pen_total_peak_mm"] / c
    return row, pen


def _qmul_vec(a, b):
    w1, x1, y1, z1 = a.T; w2, x2, y2, z2 = b.T
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], 1)


def support_edge(cell, contacts_list):
    """Support-edge error per rollout for R cells (mm): innermost force-carrying peg contact (in-plane) minus the analytic edge."""
    if not is_rim_cell(cell) or cell["pose"]["tilt_deg"] != 0.0 or contacts_list is None:
        return None
    a = math.radians(cell.get("azimuth_deg", 0.0)); e = np.array([math.cos(a), math.sin(a)])
    ref = rim_support_xmin(cell["clearance_mm"], cell["pose"]["centre_inplane_mm"]); out = []
    for C in contacts_list:
        C = np.asarray(C).reshape(-1, 8)
        f = C[C[:, 7] > 0] if len(C) else C
        xin = (f[:, 0:2] * 1e3) @ e if len(f) else np.array([])
        xall = (C[:, 0:2] * 1e3) @ e if len(C) else np.array([])
        out.append({"n_contacts": int(len(C)), "n_force": int(len(f)), "innermost_force_mm": float(xin.min()) if len(xin) else None,
                    "innermost_any_mm": float(xall.min()) if len(xall) else None, "analytic_edge_mm": ref,
                    "support_edge_error_mm": float(xin.min() - ref) if len(xin) and ref is not None else None})
    return out


# ------------------------------------------------------------------------------------------------ condition-level run
def run_condition(cand, cell, seed, man, simulate, traj_dir=None, tag=""):
    R = man["R"]; t0 = time.perf_counter(); xy = perturbation(cell, seed, R)
    sim = simulate(cand, cell, xy, dump_contacts=is_rim_cell(cell)); t_sim = time.perf_counter() - t0
    dt = cell["dt_ms"] * 1e-3; rows = []
    for w in range(R):
        row, _ = rollout_metrics(cell, man, dt, sim["pos"][w], sim["quat"][w], sim["engine_depth"][w], int(sim["truncated"]))
        row["rollout"] = w; rows.append(row)
    se = support_edge(cell, sim.get("contacts"))
    if se is not None:
        for row, s in zip(rows, se):
            row["support_edge"] = s
    out = summarize_condition(cell, rows)
    out.update(candidate=cand["id"], seed=seed, sim_wall_s=t_sim, wall_s=time.perf_counter() - t0, truncated=int(sim["truncated"]))
    if traj_dir is not None:
        Path(traj_dir).mkdir(parents=True, exist_ok=True); ks = max(1, int(round(TRAJ_SAVE_S / dt)))
        np.savez_compressed(Path(traj_dir) / f"{tag}{cell['id']}.npz", t=np.arange(sim["pos"].shape[1])[::ks] * dt,
                            pos=sim["pos"][:, ::ks].astype(np.float32), quat=sim["quat"][:, ::ks].astype(np.float32),
                            engine_depth=sim["engine_depth"][:, ::ks].astype(np.float32), xy=xy, meta=json.dumps({"cell": cell, "candidate": cand}))
    return out


def summarize_condition(cell, rows):
    exp = cell.get("expected"); R = len(rows); ok = [r["correct"] for r in rows]
    if exp is None:
        verdict = None
    else:
        k = sum(bool(o) for o in ok); verdict = "correct" if k == R else "wrong" if k == 0 else "unstable"
    cnt = {o: sum(r["outcome"] == o for r in rows) for o in ("held", "seated_settled", "seated_unsettled", "partial", "tipped_or_ejected", "invalid")}
    rest = {}
    for r in rows:
        rest[r["rest_class"]] = rest.get(r["rest_class"], 0) + 1

    def st(key, sub=None):
        v = [r[key] for r in (sub if sub is not None else rows) if r.get(key) is not None and np.isfinite(r[key])]
        return {"mean": float(np.mean(v)), "max": float(np.max(v)), "min": float(np.min(v))} if v else None
    seated = [r for r in rows if r["outcome"] in ("seated_settled", "seated_unsettled")]
    out = {"id": cell["id"], "block": cell["block"], "split": cell["split"], "role": cell["role"], "clearance_mm": cell["clearance_mm"], "mu": cell["mu"],
           "dt_ms": cell["dt_ms"], "expected": exp, "verdict": verdict, "correct_rollouts": sum(bool(o) for o in ok) if exp else None, "R": R,
           "outcomes": cnt, "rest_classes": rest, "inadmissible_rests": sum(r.get("inadmissible_rest", False) for r in rows),
           "unclassified_rests": sum(r.get("unclassified_rest", False) for r in rows), "strict_hold_count": sum(r.get("strict_hold", False) for r in rows),
           "energy_violation_rollouts": sum(r.get("energy_violation_steps", 0) > 0 for r in rows),
           "spurious_contact_rollouts": sum(r.get("spurious_contact_steps", 0) > 0 for r in rows),
           "rollouts": rows}
    for key in ("max_displacement_mm", "max_tilt_change_deg", "max_drop_mm", "final_z_mm", "final_tilt_deg", "final_radial_mm", "pen_total_peak_mm",
                "pen_base_peak_mm", "pen_bore_wall_peak_mm", "pen_rim_top_peak_mm", "pen_rim_edge_peak_mm", "pen_total_p99_mm", "pen_time_above_0p1mm_s",
                "engine_depth_max_mm", "energy_max_gain_J", "creep_rate_mm_s", "tilt_rate_deg_s", "first_base_contact_ms"):
        out[key] = st(key)
    out["seated_final_z_mm"] = st("final_z_mm", seated); out["seated_final_tilt_deg"] = st("final_tilt_deg", seated)
    out["seated_radial_excess_um"] = st("radial_excess_um", seated)
    se = [r["support_edge"]["support_edge_error_mm"] for r in rows if r.get("support_edge") and r["support_edge"].get("support_edge_error_mm") is not None]
    if se:
        out["support_edge_error_mm"] = {"mean": float(np.mean(se)), "min": float(np.min(se)), "max": float(np.max(se))}
    return out


def fmt(r):
    F = lambda d, k="max", f=".3f": "None" if d is None else format(d[k], f)
    o = r["outcomes"]
    s = (f'{r["id"]} exp={r["expected"]} verdict={r["verdict"]} ok={r["correct_rollouts"]}/{r["R"]} '
         f'held/seat/unset/part/tip/inv={o["held"]}/{o["seated_settled"]}/{o["seated_unsettled"]}/{o["partial"]}/{o["tipped_or_ejected"]}/{o["invalid"]} '
         f'rest={r["rest_classes"]} inadm={r["inadmissible_rests"]} disp={F(r["max_displacement_mm"])} dtilt={F(r["max_tilt_change_deg"], f=".2f")} '
         f'z={F(r["final_z_mm"], "mean", ".2f")} pen={F(r["pen_total_peak_mm"])} (base {F(r["pen_base_peak_mm"])} wall {F(r["pen_bore_wall_peak_mm"])} '
         f'edge {F(r["pen_rim_edge_peak_mm"])}) eng={F(r["engine_depth_max_mm"])} spur={r["spurious_contact_rollouts"]} Eviol={r["energy_violation_rollouts"]} '
         f'wall={r["wall_s"]:.0f}s')
    if "support_edge_error_mm" in r:
        s += f' edge_err={r["support_edge_error_mm"]["mean"]:.2f}mm'
    return s


# ------------------------------------------------------------------------------------------------ ladder calibration
def calibration_cell(man):
    cc = man["regularisation_ladder"]["calibration_cell"]
    return {"id": "calibration", "split": "calibration", "block": "C", "category": "calibration", "role": "calibration", "clearance_mm": cc["clearance_mm"],
            "mu": cc["mu"], "dt_ms": cc["dt_ms"], "duration_s": 3.0, "azimuth_deg": man["conditions"][0]["azimuth_deg"], "gravity_scale": 1, "mass_kg": 0.08,
            "expected": None, "pose": cc["pose"], "geometry": geom_tag(cc["clearance_mm"]) + "_free"}


def calib_probe(cand, cell, simulate, duration):
    c = dict(cell, duration_s=duration); sim = simulate(cand, c, np.zeros((1, 2)), dump_contacts=False)
    P, Q = sim["pos"][0] * 1e3, sim["quat"][0]; dt = cell["dt_ms"] * 1e-3; t = np.arange(len(P)) * dt
    if not np.isfinite(P).all():
        return {"peak_base_pen_mm": math.inf, "finite": False}
    pen = pose_penetration(P, Q, peg_radius_mm(cell)); out = {"peak_base_pen_mm": float(pen["base"].max()), "peak_total_pen_mm": float(pen["total"].max()), "finite": True}
    hit = np.nonzero(pen["base"] > 1e-4)[0]
    out["first_base_contact_ms"] = float(t[hit[0]] * 1e3) if len(hit) else None
    if duration >= 1.0:
        zf = P[-1, 2]; late = t >= DAMP_FROM_S; dev = np.abs(P[late, 2] - zf)
        out.update(final_z_mm=float(zf), damping_max_dev_mm=float(dev.max()), damping_pass=bool(dev.max() <= DAMP_BAND_MM))
        if len(hit):                                     # stop time: after first base contact, first time |vz| < 5 mm/s for 10 ms
            vz = np.abs(np.diff(P[:, 2])) / dt; k0 = hit[0]; nk = max(1, int(round(0.01 / dt)))
            stop = next((k for k in range(k0, len(vz) - nk) if (vz[k:k + nk] < SETTLE_V_MM_S).all()), None)
            out["stop_time_after_contact_ms"] = float((stop - k0) * dt * 1e3) if stop is not None else None
    return out


def calibrate_rung(target, ladder, cell, simulate, runnable, log):
    """Bisect the ladder parameter so the pose-based peak base penetration is within CALIB_TOL of target; returns a rung record."""
    dt = cell["dt_ms"] * 1e-3; tries = []
    for extra in ladder.get("extras", [None]):
        lo, hi = ladder["lo"](dt), ladder["hi"]; inc = ladder["increasing"]
        f = lambda p: calib_probe(ladder["make"](p, extra), cell, simulate, CALIB_BISECT_S)
        plo, phi_ = f(lo), f(hi); rec = {"extra": extra, "bracket": [[lo, plo["peak_base_pen_mm"]], [hi, phi_["peak_base_pen_mm"]]]}
        pmin, pmax = sorted([plo["peak_base_pen_mm"], phi_["peak_base_pen_mm"]])
        if not (pmin <= target * (1 + CALIB_TOL) and pmax >= target * (1 - CALIB_TOL)):
            rec["status"] = "unreachable"; tries.append(rec); log(f"calib target {target} extra {extra}: unreachable bracket {rec['bracket']}"); continue
        a, b, best = lo, hi, None
        for it in range(40):
            mid = math.sqrt(a * b) if ladder.get("log", True) else 0.5 * (a + b); pm = f(mid)["peak_base_pen_mm"]
            if best is None or abs(pm - target) < abs(best[1] - target):
                best = (mid, pm)
            if abs(pm - target) <= 0.02 * target:
                break
            if (pm < target) == inc:
                a = mid
            else:
                b = mid
        rec["param"], rec["peak_base_pen_mm"] = best
        if abs(best[1] - target) > CALIB_TOL * target:
            rec["status"] = "not_converged"; tries.append(rec); continue
        cand = ladder["make"](best[0], extra); full = calib_probe(cand, cell, simulate, 3.0); rec["full_run"] = full
        ok, why = runnable(cand, dt); rec["candidate"] = cand
        rec["status"] = "ok" if full["damping_pass"] else "damping_fail"
        log(f"calib target {target} extra {extra}: param {best[0]:.6g} pen {best[1]:.4f} mm damping_dev {full['damping_max_dev_mm']:.4f} mm -> {rec['status']}")
        tries.append(rec)
        if full["damping_pass"] or not ladder.get("damping_selects_extra", False):
            break
    good = [t for t in tries if t["status"] == "ok"]
    if ladder.get("damping_selects_extra", False):
        chosen = good[0] if good else None
    else:
        chosen = next((t for t in tries if t["status"] in ("ok", "damping_fail")), None)
    return {"target_mm": target, "tries": tries, "status": chosen["status"] if chosen else (tries[-1]["status"] if tries else "unreachable"),
            "candidate": chosen["candidate"] if chosen else None}


# ------------------------------------------------------------------------------------------------ selection and the full plan
def selection_key(s):
    """strict-v8-draft rule: most conditions correct, then fewest unstable, inadmissible rests, mean peak penetration, declared order."""
    return (-s["n_correct"], s["n_unstable"], s["n_inadmissible"], s["mean_peak_pen_mm"], s["index"])


def selection_key_v9(s):
    """strict-v9-draft rule: most correct rollouts summed over the selection conditions, then fewest inadmissible rests, mean peak
    penetration, declared order. Deviations are not used."""
    return (-s["n_correct_rollouts"], s["n_inadmissible"], s["mean_peak_pen_mm"], s["index"])


def selection_key_for(man):
    return selection_key if man["revision"].startswith("strict-v8") else selection_key_v9


def hardness_vs_dt(man, ladder, simulate, runnable, log=print):
    """Supplementary phase 5 (strict-v9): the ladder calibration repeated at each dt on the calibration cell only.
    Per dt: the hardest configuration that is runnable (ladder bound) and its peak base penetration; if it fails damping_alignment the
    parameter is stepped toward softer (x1.25) until it passes. Every rung is bisected at that dt. Wall time per simulated second
    comes from the 3-s probe. Nothing here touches a candidate, a frozen configuration or a scored condition."""
    spec = man["run_plan"]["5_hardness_vs_time_step"]; base = calibration_cell(man); out = {"dt": {}}
    targets = man["regularisation_ladder"]["rungs_target_peak_base_penetration_mm"]; inc = ladder["increasing"]
    for dtm in spec["dt_ms"]:
        cell = dict(base, dt_ms=dtm); dt = dtm * 1e-3; rec = {"dt_ms": dtm}
        lo, hi = ladder["lo"](dt), ladder["hi"]; p = lo if inc else hi; steps = []
        hardest = None
        for extra in ladder.get("extras", [None]):
            q = p
            for _ in range(30):
                cand = ladder["make"](q, extra); ok, why = runnable(cand, dt)
                if not ok:
                    steps.append({"param": q, "extra": extra, "runnable": False, "reason": why}); break
                t0 = time.perf_counter(); full = calib_probe(cand, cell, simulate, 3.0); wall = time.perf_counter() - t0
                steps.append({"param": q, "extra": extra, "candidate": cand, "wall_s_per_sim_s": wall / 3.0, **full})
                if full.get("damping_pass"):
                    hardest = steps[-1]; break
                q = q * 1.25 if inc else q / 1.25
                if (inc and q > hi) or ((not inc) and q < lo):
                    break
            if hardest is not None:
                break
        rec["hardest_probe_steps"] = steps
        rec["min_peak_base_pen_mm"] = hardest["peak_base_pen_mm"] if hardest else None
        rec["min_pen_candidate"] = hardest["candidate"] if hardest else None
        rec["wall_s_per_sim_s"] = hardest["wall_s_per_sim_s"] if hardest else (steps[0].get("wall_s_per_sim_s") if steps else None)
        t0 = time.perf_counter(); nat = calib_probe(ladder["native"], cell, simulate, 3.0)
        rec["native"] = dict(nat, wall_s_per_sim_s=(time.perf_counter() - t0) / 3.0)
        rec["rungs"] = {}
        for tgt in targets:
            rg = calibrate_rung(tgt, ladder, cell, simulate, runnable, log); rec["rungs"][str(tgt)] = rg
        log(f"[hardness_dt] dt {dtm} ms: min pen {rec['min_peak_base_pen_mm']} mm, rungs "
            f"{ {k: v['status'] for k, v in rec['rungs'].items()} }, wall/sim-s {rec['wall_s_per_sim_s']}")
        out["dt"][str(dtm)] = rec
    out["largest_dt_per_rung_ms"] = {}
    for tgt in targets:
        ok = [float(d) for d, r in out["dt"].items() if r["rungs"][str(tgt)]["status"] == "ok"]
        out["largest_dt_per_rung_ms"][str(tgt)] = max(ok) if ok else "not reachable in the tested range"
    return out


def run_native_matrix(man, ladder, simulate, out_dir, log=print, label="B1"):
    """Frozen matrix with the native default (B = 1). It does not depend on selection, so it can run in its own process; the result
    is merged into result.json['frozen_matrix'][label] afterwards (merge_native_matrix)."""
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True); jl = open(out_dir / "runs.jsonl", "a")
    conds = man["conditions"]; idx = {c["id"]: j for j, c in enumerate(conds)}; rows = []; res = {"interpretation": INTERPRETATION}
    for c in [c for c in conds if c["role"] == "frozen"]:
        r = run_condition(ladder["native"], c, man["seed0"] + idx[c["id"]], man, simulate, out_dir / "traj" / f"frozen_{label}", "")
        r["phase"] = f"frozen_{label}"; log(f"[frozen_{label}] {ladder['native']['id']} {fmt(r)}"); jl.write(json.dumps(json_safe(r)) + "\n"); jl.flush()
        rows.append(r); res["frozen_matrix"] = {label: {"candidate_index": 1, "conditions": rows}}
        (out_dir / "result.json").write_text(json.dumps(json_safe(res), indent=1) + "\n")
    res["frozen_matrix"][label]["tally"] = tally(rows); log(f"frozen {label} tally {res['frozen_matrix'][label]['tally']}")
    (out_dir / "result.json").write_text(json.dumps(json_safe(res), indent=1) + "\n"); jl.close(); return res


def execute(man, engine, simulate, runnable, ladder, out_dir, log=print, phases=("calibration", "selection", "frozen", "robustness", "attribution"),
            only_conditions=None, frozen_configs=("B5", "B1")):
    out_dir = Path(out_dir); out_dir.mkdir(parents=True, exist_ok=True); traj = out_dir / "traj"; jl = open(out_dir / "runs.jsonl", "a")
    conds = man["conditions"]; idx = {c["id"]: j for j, c in enumerate(conds)}; s0 = man["seed0"]
    seed = lambda cid: s0 + idx[cid]
    res = {"interpretation": INTERPRETATION, "engine": engine}

    def save():
        (out_dir / "result.json").write_text(json.dumps(json_safe(res), indent=1) + "\n")

    def run(cand, cell, tag, phase):
        r = run_condition(cand, cell, seed(cell["id"]), man, simulate, traj / tag, "")
        r["phase"] = phase; log(f"[{phase}] {cand['id']} {fmt(r)}"); jl.write(json.dumps(json_safe(r)) + "\n"); jl.flush(); return r

    # ---- 0. ladder calibration
    cal = calibration_cell(man); rungs = {}
    if "calibration" in phases:
        for tgt in man["regularisation_ladder"]["rungs_target_peak_base_penetration_mm"]:
            rungs[tgt] = calibrate_rung(tgt, ladder, cal, simulate, runnable, log); res["calibration"] = rungs; save()
        nat = calib_probe(ladder["native"], cal, simulate, 3.0); res["calibration_native"] = nat
        log(f"calib native: {nat}"); save()
    else:
        prev = json.loads((out_dir / "result.json").read_text()); rungs = {float(k): v for k, v in prev["calibration"].items()}; res["calibration"] = prev["calibration"]
        res["calibration_native"] = prev.get("calibration_native")
    order = man["run_plan"]["1_selection"]["candidates_in_declared_order"]
    cands = []
    for i, name in enumerate(order):
        if name == "native default":
            cands.append({"index": i + 1, "name": name, "candidate": ladder["native"], "status": "ok"})
        else:
            tgt = float(name.split()[1]); rg = rungs[tgt]
            cands.append({"index": i + 1, "name": name, "candidate": rg["candidate"], "status": rg["status"]})
    res["candidates"] = cands; save()
    if "selection" not in phases:
        return res
    # ---- 1. selection
    sel_ids = man["run_plan"]["1_selection"]["conditions"]; sel = []
    for cd in cands:
        rec = {"index": cd["index"], "name": cd["name"], "candidate": cd["candidate"], "status": cd["status"]}
        if cd["candidate"] is None:
            rec.update(runnable=False, reason=f"rung {cd['status']}"); sel.append(rec); continue
        ok, why = runnable(cd["candidate"], 0.5e-3)
        if not ok:
            rec.update(runnable=False, reason=why); sel.append(rec); continue
        rs = [run(cd["candidate"], conds[idx[c]], f"selection/cand{cd['index']}", "selection") for c in sel_ids]
        rec.update(runnable=True, conditions=rs, n_correct=sum(r["verdict"] == "correct" for r in rs), n_unstable=sum(r["verdict"] == "unstable" for r in rs),
                   n_correct_rollouts=sum(r["correct_rollouts"] for r in rs), n_rollouts=sum(r["R"] for r in rs),
                   n_inadmissible=sum(r["inadmissible_rests"] for r in rs), mean_peak_pen_mm=float(np.mean([r["pen_total_peak_mm"]["mean"] for r in rs])))
        sel.append(rec); res["selection"] = sel; save()
    res["selection"] = sel
    run_ok = [s for s in sel if s.get("runnable")]; skey = selection_key_for(man); res["selection_rule"] = skey.__doc__.strip()
    res["B_k_frozen_index"] = {f"B{k}": min([s for s in run_ok if s["index"] <= k], key=skey)["index"] if any(s["index"] <= k for s in run_ok) else None
                               for k in range(1, len(order) + 1)}
    win = min(run_ok, key=skey); res["frozen_index"] = win["index"]; res["frozen_candidate"] = win["candidate"]; res["frozen_name"] = win["name"]
    log(f"frozen B=5: candidate {win['index']} ({win['name']}) {win['candidate']} correct {win['n_correct']}/3, "
        f"rollouts {win['n_correct_rollouts']}/{win['n_rollouts']}"); save()
    if "frozen" not in phases:
        return res
    # ---- 2. frozen matrix
    frozen = [c for c in conds if c["role"] == "frozen" and (only_conditions is None or c["id"] in only_conditions)]
    configs = ([("B5", win)] if "B5" in frozen_configs else []) + \
              ([("B1", sel[0])] if "B1" in frozen_configs and win["index"] != 1 and sel[0].get("runnable") else [])
    res["frozen_matrix"] = {}
    for lab, s in configs:
        rows = []
        for c in frozen:
            rows.append(run(s["candidate"], c, f"frozen_{lab}", f"frozen_{lab}")); res["frozen_matrix"][lab] = {"candidate_index": s["index"], "conditions": rows}; save()
        res["frozen_matrix"][lab]["tally"] = tally(rows)
        log(f"frozen {lab} tally {res['frozen_matrix'][lab]['tally']}"); save()
    if "robustness" not in phases:
        return res
    # ---- 3. robustness
    rob_ids = man["run_plan"]["3_robustness"]["conditions"]; rob = {}
    for dtm in (0.25, 1.0):
        ok, why = runnable(win["candidate"], dtm * 1e-3); key = f"dt_{dtm:g}ms"
        if not ok:
            rob[key] = {"runnable": False, "reason": why}; continue
        rob[key] = {"runnable": True, "conditions": [run(win["candidate"], dict(conds[idx[c]], dt_ms=dtm), f"robust_{key}", f"robust_{key}") for c in rob_ids]}
        res["robustness"] = rob; save()
    rep = [run(win["candidate"], conds[idx[c]], "robust_repeat", "robust_repeat") for c in rob_ids]
    first = {r["id"]: r for r in res["frozen_matrix"]["B5"]["conditions"]}
    rob["repeat"] = {"conditions": rep, "verdict_changes": [r["id"] for r in rep if first.get(r["id"], {}).get("verdict") != r["verdict"]],
                     "final_pose_max_diff_mm": {r["id"]: _pose_diff(first.get(r["id"]), r) for r in rep}}
    res["robustness"] = rob; save()
    # ---- 4. attribution control
    if "attribution" in phases and ladder.get("attribution") is not None:
        rg = rungs.get(0.6) or rungs.get("0.6")
        if rg and rg.get("candidate"):
            ca = ladder["attribution"](rg["candidate"])
            res["attribution_control"] = {"candidate": ca, "conditions": [run(ca, conds[idx[c]], "attribution", "attribution") for c in sel_ids]}
        else:
            res["attribution_control"] = {"runnable": False, "reason": "0.6-mm rung unavailable"}
        save()
    # ---- 5. hardness versus time step (strict-v9 supplementary; calibration cell only)
    if "hardness_dt" in phases and "5_hardness_vs_time_step" in man["run_plan"]:
        res["hardness_vs_time_step"] = hardness_vs_dt(man, ladder, simulate, runnable, log); save()
    jl.close(); return res


def _pose_diff(a, b):
    if not a:
        return None
    d = [math.dist([ra["final_z_mm"], ra["final_radial_mm"]], [rb["final_z_mm"], rb["final_radial_mm"]])
         for ra, rb in zip(a["rollouts"], b["rollouts"]) if ra.get("final_z_mm") is not None and rb.get("final_z_mm") is not None]
    return float(max(d)) if d else None


def tally(rows):
    out = {}
    for grp, sub in (("same_clearance", [r for r in rows if r["split"] == "dev"]), ("unseen_clearance", [r for r in rows if r["split"] == "heldout"])):
        g = {}
        for blk in sorted({r["block"] for r in sub}):
            b = [r for r in sub if r["block"] == blk and r["expected"] is not None]
            g[blk] = {v: sum(r["verdict"] == v for r in b) for v in ("correct", "wrong", "unstable")}
            g[blk]["correct_rollouts"] = sum(r["correct_rollouts"] for r in b); g[blk]["rollouts"] = sum(r["R"] for r in b)
        lab = [r for r in sub if r["expected"] is not None]
        g["all_labelled"] = {v: sum(r["verdict"] == v for r in lab) for v in ("correct", "wrong", "unstable")}
        g["inadmissible_rest_conditions"] = sum(r["inadmissible_rests"] > 0 for r in sub)
        out[grp] = g
    return out
