"""Shared helpers for strict-v3 frozen-generalizability runs (SAP Warp and MuJoCo Warp).

Both engine drivers import this module so that, for a given manifest seed, the
R=20 initial-state perturbations and force scales are numerically identical
across engines (task_runner_sop.md section 3: "matched seed IDs").
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def load_manifest(path: Path) -> tuple[dict[str, Any], str]:
    raw = Path(path).read_bytes()
    return json.loads(raw), hashlib.sha256(raw).hexdigest()


def wilson(k: int, n: int) -> list[float]:
    if n == 0:
        return [math.nan, math.nan]
    z = 1.959963984540054
    p = k / n
    den = 1 + z * z / n
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    centre = (p + z * z / (2 * n)) / den
    return [centre - half, centre + half]


def cell_seed(seed0: int, phase: str, index: int) -> int:
    """selection candidate i -> seed0 + i ; held-out cell j -> seed0 + 1000 + j.

    Selection uses the same seed for every candidate (common random numbers),
    so candidates differ only in their contact parameters.
    """
    return seed0 if phase == "selection" else seed0 + 1000 + index


def plate_perturbations(seed: int, R: int) -> tuple[np.ndarray, np.ndarray]:
    """Three-plate: xy ~ U(+-0.1 mm), yaw ~ U(+-3 mrad), per plate, per rollout."""
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-1e-4, 1e-4, size=(R, 3, 2))
    yaw = rng.uniform(-3e-3, 3e-3, size=(R, 3))
    return xy, yaw


def box_perturbations(seed: int, R: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Three-box: xy ~ U(+-0.4 mm), yaw ~ U(+-5 mrad) per box, force scale 1 +- 0.5%."""
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-4e-4, 4e-4, size=(R, 3, 2))
    yaw = rng.uniform(-5e-3, 5e-3, size=(R, 3))
    scale = 1.0 + rng.uniform(-5e-3, 5e-3, size=R)
    return xy, yaw, scale


def json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.generic):
        return json_safe(value.item())
    return value


# ---------------------------------------------------------------- three-plate
PLATE_TAIL_S = 1.0     # load averaging window (last 1 s)
PLATE_SETTLE_S = 0.2   # settling window (last 0.2 s)


def plate_summary(z: np.ndarray, v: np.ndarray, w: np.ndarray, fn: np.ndarray, cnt: np.ndarray,
                  *, h: float, mass: float, g: float, sample_dt: float, truncated: int) -> dict[str, Any]:
    """Engine-independent three-plate gate evaluation.

    z [T,R,3] plate centre heights; v,w [T,R,3,3] linear/angular velocity;
    fn,cnt [T,R,3] interface normal load / contact count, bottom-up.
    """
    T = len(z)
    tail = np.arange(T) >= T - round(PLATE_TAIL_S / sample_dt)
    last = np.arange(T) >= T - round(PLATE_SETTLE_S / sample_dt)
    gaps = np.empty_like(z)
    gaps[:, :, 0] = z[:, :, 0] - h / 2
    gaps[:, :, 1] = z[:, :, 1] - z[:, :, 0] - h
    gaps[:, :, 2] = z[:, :, 2] - z[:, :, 1] - h
    overlap = np.maximum(0.0, -gaps)
    peak = overlap.max(axis=(0, 2))
    order = np.all((z[:, :, 2] > z[:, :, 1]) & (z[:, :, 1] > z[:, :, 0]) & (z[:, :, 0] > 0), axis=0)
    speed = np.linalg.norm(v, axis=3)
    omega = np.linalg.norm(w, axis=3)
    settled = np.all((speed[last] < .01) & (omega[last] < .05), axis=(0, 2))
    expected = np.array([3, 2, 1], float) * mass * g
    fmean = fn[tail].mean(axis=0)
    ferr = np.abs(fmean - expected) / expected
    finite = np.isfinite(z).all(axis=(0, 2)) & np.isfinite(fn).all(axis=(0, 2))
    gates = {
        "order": order,
        "overlap": peak <= .05 * h,
        "load": np.all(ferr <= .01, axis=1),
        "settling": settled,
        "finite": finite,
    }
    passed = np.logical_and.reduce(list(gates.values())) & (truncated == 0)
    R = z.shape[1]
    zref = np.array([.5, 1.5, 2.5]) * h
    rows = []
    for r in range(R):
        rows.append({
            "rollout": r, "passes": bool(passed[r]),
            **{f"gate_{k}": bool(val[r]) for k, val in gates.items()},
            "max_overlap_mm": float(peak[r] * 1e3), "max_overlap_over_h": float(peak[r] / h),
            "load_error_by_interface": ferr[r].tolist(), "load_error_max": float(ferr[r].max()),
            "contact_duty_by_interface": (cnt[tail, r] > 0).mean(axis=0).tolist(),
            "height_bias_mm": ((z[-1, r] - zref) * 1e3).tolist(),
            "tail_speed_max_m_s": float(speed[last, r].max()), "tail_omega_max_rad_s": float(omega[last, r].max()),
        })
    k = int(passed.sum())
    return {
        "R": R, "pass_count": k, "pass_fraction": k / R, "pass_wilson_95ci": wilson(k, R),
        "failed_gate_counts": {name: int((~val).sum()) for name, val in gates.items()},
        "truncated_contacts": int(truncated),
        "max_overlap_mm_mean": float(peak.mean() * 1e3), "max_overlap_over_h_mean": float((peak / h).mean()),
        "load_error_max_mean": float(ferr.max(axis=1).mean()), "load_error_mean": float(ferr.mean()),
        "tail_speed_max_mean_m_s": float(speed[last].max(axis=(0, 2)).mean()),
        "contact_duty_mean": float((cnt[tail] > 0).mean()),
        "rollouts": rows,
    }


def plate_choose(rows: list[dict[str, Any]]) -> int:
    """Manifest rule: pass fraction, then overlap/h, then load error, then tail motion, then wall time."""
    key = lambda i: (-rows[i]["pass_fraction"], rows[i]["max_overlap_over_h_mean"],
                     rows[i]["load_error_max_mean"], rows[i]["tail_speed_max_mean_m_s"], rows[i]["wall_s"])
    return min(range(len(rows)), key=key)


# ------------------------------------------------------------------ three-box
BOX_SETTLE_S, BOX_RAMP_S, BOX_HOLD_S = 0.6, 0.4, 0.65
BOX_TAIL_S = 0.4          # creep slope window: last 0.4 s of the hold
BOX_SLIP_M = 1e-3         # >= 1 mm support-relative displacement = slip
BOX_DUTY_MIN = 0.99


def box_force_N(cell: dict[str, Any], g: float) -> float:
    if "force_N" in cell:
        return float(cell["force_N"])
    n = 2 if cell["pushed_body"] == "middle" else 1
    return float(cell["force_ratio"]) * n * cell["mu"] * cell["mass_kg"] * g


def box_threshold_N(cell: dict[str, Any], g: float) -> float:
    n = 2 if cell["pushed_body"] == "middle" else 1
    return n * cell["mu"] * cell["mass_kg"] * g


def box_summary(t: np.ndarray, rel_x: np.ndarray, duty: np.ndarray, fn: np.ndarray, *, expected: str,
                finite: np.ndarray, truncated: int) -> dict[str, Any]:
    """rel_x [T,R] support-relative x displacement since force onset; duty/fn [T,R] for the pushed interface
    over the force window (ramp+hold)."""
    tail = t >= t[-1] - BOX_TAIL_S + 1e-9
    final = np.abs(rel_x[-1])
    slip = final >= BOX_SLIP_M
    creep = np.array([abs(np.polyfit(t[tail], rel_x[tail, r], 1)[0]) for r in range(rel_x.shape[1])])
    duty_r = duty.mean(axis=0)
    observed = np.where(slip, "release", "retain")
    agree = observed == expected
    valid = finite & (duty_r >= BOX_DUTY_MIN) & (truncated == 0)
    R = rel_x.shape[1]
    rows = [{"rollout": r, "observed": str(observed[r]), "agrees": bool(agree[r]), "valid": bool(valid[r]),
             "final_rel_disp_mm": float(final[r] * 1e3), "tail_creep_um_s": float(creep[r] * 1e6),
             "contact_duty": float(duty_r[r]), "Fn_tail_mean_N": float(fn[tail, r].mean())} for r in range(R)]
    k = int((agree & valid).sum())
    return {
        "R": R, "expected": expected, "agree_count": int(agree.sum()), "valid_count": int(valid.sum()),
        "agree_and_valid_count": k, "accuracy": k / R, "accuracy_wilson_95ci": wilson(k, R),
        "observed_retain": int((observed == "retain").sum()), "observed_release": int((observed == "release").sum()),
        "final_rel_disp_mm_mean": float(final.mean() * 1e3), "tail_creep_um_s_mean": float(creep.mean() * 1e6),
        "contact_duty_mean": float(duty_r.mean()), "contact_duty_min": float(duty_r.min()),
        "Fn_tail_mean_N": float(fn[tail].mean()), "truncated_contacts": int(truncated),
        "rollouts": rows,
    }


def box_choose(rows: list[dict[str, Any]]) -> int:
    """UNIFIED-B5-SOP: valid normal gate (and correct hold) first, then low tail creep, then wall time."""
    key = lambda i: (-rows[i]["agree_and_valid_count"], rows[i]["tail_creep_um_s_mean"], rows[i]["wall_s"])
    return min(range(len(rows)), key=key)


# ------------------------------------------------------- three-box strict-v4
# Rigid-body event thresholds for a centred stack of three identical boxes (half-width a) with a
# horizontal force at the COM of the pushed box (1 = middle, 2 = top).  Interface i lies under box i
# (0 = ground).  All thresholds are in units of m*g and are independent of a.
V4_TILT_TIP_DEG = 10.0
V4_SLIP_M = 1e-3


def box_v4_thresholds(pushed: int, mu: float) -> list[tuple[str, int, float]]:
    out = []
    h_push = 1 + 2 * pushed                      # pushed COM height / a
    for i in range(pushed + 1):
        above = 3 - i                            # boxes resting on interface i
        out.append(("slip", i, mu * above))
        out.append(("tip", i, above / (h_push - 2 * i)))
    return out


def box_v4_force_mg(cell: dict[str, Any]) -> float:
    """force_mg (F/(m g)) or force_ratio relative to the pushed interface's Coulomb threshold."""
    if "force_mg" in cell:
        return float(cell["force_mg"])
    pushed = 1 if cell["pushed_body"] == "middle" else 2
    return float(cell["force_ratio"]) * cell["mu"] * (3 - pushed)


def box_v4_expected(cell: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    """Analytic label and margins.  retain: F <= (1-m) T1.  event: F >= (1+m) T1 and F <= Tc/(1+mc),
    T1 = smallest threshold, Tc = smallest threshold of the *other* event class, m = 0.2,
    mc = cell.get('competing_margin', 0.2)."""
    pushed = 1 if cell["pushed_body"] == "middle" else 2
    F = box_v4_force_mg(cell)
    th = sorted(box_v4_thresholds(pushed, cell["mu"]), key=lambda x: x[2])
    k1, i1, t1 = th[0]
    info = {"force_mg": F, "first_threshold": {"kind": k1, "interface": i1, "T_mg": t1},
            "thresholds_mg": [{"kind": k, "interface": i, "T_mg": t} for k, i, t in th]}
    m, mc = 0.2, float(cell.get("competing_margin", 0.2))
    if F <= (1 - m) * t1 + 1e-12:
        info["margin"] = 1 - F / t1
        return "retain", info
    other = [t for k, _, t in th if k != k1]
    tc = min(other) if other else float("inf")
    if F >= (1 + m) * t1 - 1e-12 and F <= tc / (1 + mc) + 1e-12:
        info["margin"] = F / t1 - 1; info["competing_margin"] = tc / F - 1
        return k1, info
    info["margin"] = None
    return "boundary", info


def _quat_z_axis(q_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = q_wxyz[..., 0], q_wxyz[..., 1], q_wxyz[..., 2], q_wxyz[..., 3]
    return np.stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)], axis=-1)


def _quat_rotate(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    w = q_wxyz[..., :1]; u = q_wxyz[..., 1:]
    t = 2 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def box_v4_kinematics(pos: np.ndarray, quat_wxyz: np.ndarray, half: float) -> dict[str, np.ndarray]:
    """pos [T,R,3,3], quat [T,R,3,4] (wxyz) -> tilt [T,R,3] (deg), face x at every interface [T,R,3]
    (interface 0 = bottom face of box 0 in world x; interface i = bottom face of box i minus top face of box i-1)."""
    zax = _quat_z_axis(quat_wxyz)
    tilt = np.degrees(np.arccos(np.clip(zax[..., 2], -1, 1)))
    down = _quat_rotate(quat_wxyz, np.array([0, 0, -half]))
    up = _quat_rotate(quat_wxyz, np.array([0, 0, half]))
    bot = pos + down; top = pos + up
    face = np.empty(pos.shape[:3])
    face[..., 0] = bot[..., 0, 0]
    face[..., 1] = bot[..., 1, 0] - top[..., 0, 0]
    face[..., 2] = bot[..., 2, 0] - top[..., 1, 0]
    return {"tilt_deg": tilt, "face_x": face}


def box_v4_summary(t: np.ndarray, pos: np.ndarray, quat: np.ndarray, duty: np.ndarray, fn: np.ndarray, *,
                   half: float, pushed: int, expected: str, finite: np.ndarray, truncated: int | np.ndarray) -> dict[str, Any]:
    """t [T] seconds since force onset; pos/quat over the force window; duty/fn [T,R] pushed interface."""
    k = box_v4_kinematics(pos, quat, half)
    face = k["face_x"] - k["face_x"][0]                       # change since force onset
    face_max = np.abs(face[..., : pushed + 1]).max(axis=2)  # any interface at/below the pushed box
    tilt = k["tilt_deg"].max(axis=2)
    com = (pos[..., pushed, 0] - pos[..., pushed - 1, 0]); com = com - com[0]
    R = pos.shape[1]
    finite = (np.asarray(finite, dtype=bool)
              & np.isfinite(pos).all(axis=(0, 2, 3))
              & np.isfinite(quat).all(axis=(0, 2, 3)))
    capacity_bad = np.broadcast_to(np.asarray(truncated) != 0, (R,)).copy()
    big = len(t) + 1
    t_slip = np.array([np.argmax(face_max[:, r] >= V4_SLIP_M) if (face_max[:, r] >= V4_SLIP_M).any() else big for r in range(R)])
    t_tip = np.array([np.argmax(tilt[:, r] > V4_TILT_TIP_DEG) if (tilt[:, r] > V4_TILT_TIP_DEG).any() else big for r in range(R)])
    observed = np.where((t_tip < big) & (t_tip <= t_slip), "tip", np.where(t_slip < big, "slip", "retain"))
    observed = np.where(finite, observed, "invalid")
    first = np.minimum(t_slip, t_tip)
    duty_pre = np.array([duty[: max(1, min(first[r], len(t))), r].mean() for r in range(R)])
    valid = finite & (duty_pre >= BOX_DUTY_MIN) & ~capacity_bad
    agree = observed == expected
    tail = t >= t[-1] - BOX_TAIL_S + 1e-9
    creep = np.array([abs(np.polyfit(t[tail], face_max[tail, r], 1)[0]) for r in range(R)])
    rows = [{"rollout": r, "observed": str(observed[r]), "agrees": bool(agree[r]), "valid": bool(valid[r]),
             "finite": bool(finite[r]), "capacity_ok": bool(not capacity_bad[r]),
             "contact_gate_pass": bool(duty_pre[r] >= BOX_DUTY_MIN),
             "face_disp_final_mm": float(face_max[-1, r] * 1e3), "com_rel_disp_final_mm": float(abs(com[-1, r]) * 1e3),
             "tilt_max_deg": float(tilt[:, r].max()), "hold_face_creep_um_s": float(creep[r] * 1e6),
             "event_time_s": (float(t[first[r]]) if first[r] < big else None),
             "duty_pre_event": float(duty_pre[r]), "Fn_mean_N": float(fn[:, r].mean())} for r in range(R)]
    kk = int((agree & valid).sum())
    cnt = {c: int((observed == c).sum()) for c in ("retain", "slip", "tip", "invalid")}
    return {"R": R, "expected": expected, "observed_counts": cnt, "agree_and_valid_count": kk,
            "accuracy": kk / R, "accuracy_wilson_95ci": wilson(kk, R), "valid_count": int(valid.sum()),
            "face_disp_final_mm_median": float(np.median(face_max[-1]) * 1e3),
            "com_rel_disp_final_mm_median": float(np.median(np.abs(com[-1])) * 1e3),
            "tilt_max_deg_median": float(np.median(tilt.max(axis=0))),
            "hold_face_creep_um_s_mean": float(creep.mean() * 1e6), "duty_pre_event_min": float(duty_pre.min()),
            "finite_count": int(finite.sum()), "classification_correct_finite_count": int((agree & finite).sum()),
            "capacity_overflow_worlds": int(capacity_bad.sum()),
            "truncated_contacts": int(capacity_bad.sum()), "rollouts": rows}


def box_v4_choose(rows_by_candidate: list[list[dict[str, Any]]], walls: list[float]) -> int:
    """rows_by_candidate[i] = [summary of each selection cell].  Rank: total valid+correct rollouts over all
    selection cells, then mean hold creep on retain cells, then wall time."""
    def key(i):
        rows = rows_by_candidate[i]
        correct = sum(r["agree_and_valid_count"] for r in rows)
        creep = np.mean([r["hold_face_creep_um_s_mean"] for r in rows if r["expected"] == "retain"] or [0.0])
        return (-correct, creep, walls[i])
    return min(range(len(rows_by_candidate)), key=key)
