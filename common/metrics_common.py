"""Shared evaluation metrics and backend helpers for static-friction-v2 and dynamic-friction-v2.

Supports both:
  - Standalone CPU MuJoCo (mujoco 3.17.0)
  - GPU Batched MuJoCo Warp (mujoco_warp 3.17.0 / warp 1.18.0)
"""
from __future__ import annotations

import math
from typing import Any

import mujoco
import numpy as np

try:
    import mujoco_warp as mjw
    import warp as wp
    HAS_MJWARP = True
except ImportError:
    mjw = None
    wp = None
    HAS_MJWARP = False


def dtw_distance(s1: np.ndarray, s2: np.ndarray, window: int | None = None) -> float:
    """Compute Euclidean Dynamic Time Warping (DTW) average per-step distance."""
    a = np.asarray(s1, dtype=float)
    b = np.asarray(s2, dtype=float)
    if a.ndim == 1:
        a = a[:, None]
    if b.ndim == 1:
        b = b[:, None]
    n, m = len(a), len(b)
    if n == 0 or m == 0:
        return float("nan")
    w = max(window or max(n, m), abs(n - m))
    cost = np.full((n + 1, m + 1), np.inf, dtype=float)
    cost[0, 0] = 0.0
    for i in range(1, n + 1):
        j_start = max(1, i - w)
        j_end = min(m, i + w)
        for j in range(j_start, j_end + 1):
            dist = float(np.linalg.norm(a[i - 1] - b[j - 1]))
            cost[i, j] = dist + min(cost[i - 1, j], cost[i, j - 1], cost[i - 1, j - 1])
    return float(cost[n, m] / max(n, m))


def trajectory_rmse(sim: np.ndarray, ref: np.ndarray) -> float:
    """Framewise Euclidean RMSE between aligned trajectories."""
    a = np.asarray(sim, dtype=float)
    b = np.asarray(ref, dtype=float)
    n = min(len(a), len(b))
    if n == 0:
        return float("nan")
    diff = a[:n] - b[:n]
    if diff.ndim == 1:
        return float(np.sqrt(np.mean(diff ** 2)))
    return float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1))))


def fit_quadratic_acceleration(t: np.ndarray, s: np.ndarray) -> tuple[float, float, float]:
    """Fit s(t) = s0 + v0*t + 0.5*a*t^2 via OLS over a declared sliding window.

    Returns (a_hat, v0_hat, s0_hat). Avoids finite-difference noise amplification.
    """
    tt = np.asarray(t, dtype=float)
    ss = np.asarray(s, dtype=float)
    if len(tt) < 3:
        return float("nan"), float("nan"), float("nan")
    t_rel = tt - tt[0]
    X = np.column_stack([np.ones_like(t_rel), t_rel, 0.5 * (t_rel ** 2)])
    beta, _, _, _ = np.linalg.lstsq(X, ss, rcond=None)
    return float(beta[2]), float(beta[1]), float(beta[0])


def fit_linear_slope(t: np.ndarray, x: np.ndarray) -> float:
    """OLS slope of x(t) over the evaluation window (e.g., tail creep rate)."""
    tt = np.asarray(t, dtype=float)
    xx = np.asarray(x, dtype=float)
    if len(tt) < 2:
        return float("nan")
    t_rel = tt - np.mean(tt)
    denom = float(np.sum(t_rel ** 2))
    if denom <= 1e-18:
        return 0.0
    return float(np.sum(t_rel * (xx - np.mean(xx))) / denom)


def warp_contact_records(data: Any, world_id: int) -> list[dict[str, Any]]:
    """Read solved Warp contact forces directly from its EFC buffers.

    A CPU ``MjData`` restored through ``get_data_into`` contains kinematics but
    not Warp's solved EFC force vector.  Contact-local force component ``j``
    belongs to ``data.efc.force[world, contact.efc_address[contact, j]]``.
    """
    n = int(data.nacon.numpy()[0])
    if n == 0:
        return []
    wid = data.contact.worldid.numpy()[:n]
    geom = data.contact.geom.numpy()[:n]
    addr = data.contact.efc_address.numpy()[:n]
    dim = data.contact.dim.numpy()[:n]
    frame = data.contact.frame.numpy()[:n]
    force = data.efc.force.numpy()[world_id]
    out: list[dict[str, Any]] = []
    for i in np.flatnonzero(wid == world_id):
        local = np.zeros(3, dtype=float)
        for j in range(min(3, int(dim[i]))):
            row = int(addr[i, j])
            if row >= 0:
                local[j] = float(force[row])
        normal_local = np.array([local[0], 0.0, 0.0], dtype=float)
        world_force = np.asarray(frame[i], dtype=float).T @ local
        world_normal = np.asarray(frame[i], dtype=float).T @ normal_local
        out.append({
            "geom1": int(geom[i, 0]), "geom2": int(geom[i, 1]),
            "normal": max(0.0, float(local[0])), "tangent": local[1:3],
            "world_force": world_force,
            "world_normal": world_normal,
            "world_tangent": world_force - world_normal,
        })
    return out


def wilson_ci(successes: int, total: int, z: float = 1.959963984540054) -> tuple[float, float]:
    """95% Wilson score confidence interval for a binomial proportion."""
    if total <= 0:
        return (float("nan"), float("nan"))
    p = successes / total
    denom = 1.0 + (z ** 2) / total
    center = (p + (z ** 2) / (2.0 * total)) / denom
    half = (z * math.sqrt((p * (1.0 - p) + (z ** 2) / (4.0 * total)) / total)) / denom
    return (max(0.0, center - half), min(1.0, center + half))


def compute_binary_outcome_stats(
    records: list[dict[str, Any]],
    group_key: str = "condition_group",
) -> dict[str, Any]:
    """Compute pooled and macro-averaged TP/FP/FN/TN, FPR, FNR, balanced accuracy, and Wilson CIs.

    Each record in ``records`` must have:
      - ``expected_positive``: bool (e.g., True for eject / release / fail-band, False for retain / pass-band)
      - ``simulated_positive``: bool
      - ``condition_id``: str
      - ``condition_group``: str (optional, for macro-averaging across physical groups)
    """
    tp = sum(1 for r in records if r["expected_positive"] and r["simulated_positive"])
    fp = sum(1 for r in records if (not r["expected_positive"]) and r["simulated_positive"])
    fn = sum(1 for r in records if r["expected_positive"] and (not r["simulated_positive"]))
    tn = sum(1 for r in records if (not r["expected_positive"]) and (not r["simulated_positive"]))
    n_pos = tp + fn
    n_neg = tn + fp
    total = n_pos + n_neg
    agree = tp + tn

    fpr = fp / n_neg if n_neg > 0 else 0.0
    fnr = fn / n_pos if n_pos > 0 else 0.0
    bal_acc = 1.0 - 0.5 * (fpr + fnr) if (n_pos > 0 and n_neg > 0) else (agree / total if total > 0 else float("nan"))
    agree_ci = wilson_ci(agree, total)
    fpr_ci = wilson_ci(fp, n_neg) if n_neg > 0 else (0.0, 0.0)
    fnr_ci = wilson_ci(fn, n_pos) if n_pos > 0 else (0.0, 0.0)

    # Condition-level fractions
    by_cond: dict[str, list[dict[str, Any]]] = {}
    for r in records:
        by_cond.setdefault(str(r.get("condition_id", "default")), []).append(r)

    condition_summaries = {}
    for cid, items in by_cond.items():
        m = len(items)
        k_agree = sum(1 for x in items if x["expected_positive"] == x["simulated_positive"])
        k_pos = sum(1 for x in items if x["simulated_positive"])
        condition_summaries[cid] = {
            "M": m,
            "expected_positive": bool(items[0]["expected_positive"]),
            "positive_fraction": k_pos / m,
            "agreement_fraction_p_hat": k_agree / m,
            "agreement_wilson_95ci": list(wilson_ci(k_agree, m)),
        }

    # Macro-average across physical condition groups
    by_group: dict[str, list[dict[str, Any]]] = {}
    for r in records:
        by_group.setdefault(str(r.get(group_key, "all")), []).append(r)

    group_agreements = []
    group_fprs = []
    group_fnrs = []
    for gname, gitems in by_group.items():
        gtp = sum(1 for r in gitems if r["expected_positive"] and r["simulated_positive"])
        gfp = sum(1 for r in gitems if (not r["expected_positive"]) and r["simulated_positive"])
        gfn = sum(1 for r in gitems if r["expected_positive"] and (not r["simulated_positive"]))
        gtn = sum(1 for r in gitems if (not r["expected_positive"]) and (not r["simulated_positive"]))
        g_agree = (gtp + gtn) / len(gitems)
        group_agreements.append(g_agree)
        if (gtn + gfp) > 0:
            group_fprs.append(gfp / (gtn + gfp))
        if (gtp + gfn) > 0:
            group_fnrs.append(gfn / (gtp + gfn))

    macro_fpr = float(np.mean(group_fprs)) if group_fprs else 0.0
    macro_fnr = float(np.mean(group_fnrs)) if group_fnrs else 0.0
    macro_bal_acc = 1.0 - 0.5 * (macro_fpr + macro_fnr)

    return {
        "total_rollouts": total,
        "positive_denominator": n_pos,
        "negative_denominator": n_neg,
        "TP": tp,
        "FP": fp,
        "FN": fn,
        "TN": tn,
        "pooled_agreement": agree / total if total > 0 else float("nan"),
        "pooled_agreement_wilson_95ci": list(agree_ci),
        "pooled_FPR": fpr,
        "pooled_FPR_wilson_95ci": list(fpr_ci),
        "pooled_FNR": fnr,
        "pooled_FNR_wilson_95ci": list(fnr_ci),
        "pooled_balanced_accuracy": bal_acc,
        "macro_group_count": len(by_group),
        "macro_agreement": float(np.mean(group_agreements)) if group_agreements else float("nan"),
        "macro_FPR": macro_fpr,
        "macro_FNR": macro_fnr,
        "macro_balanced_accuracy": macro_bal_acc,
        "per_condition": condition_summaries,
    }


def spearman_rank_corr(x: list[float] | np.ndarray, y: list[float] | np.ndarray) -> float:
    """Compute Spearman rank correlation |rho_s| without requiring scipy."""
    xa = np.asarray(x, dtype=float)
    ya = np.asarray(y, dtype=float)
    mask = np.isfinite(xa) & np.isfinite(ya)
    xa, ya = xa[mask], ya[mask]
    if len(xa) < 2:
        return float("nan")
    rx = np.argsort(np.argsort(xa)).astype(float)
    ry = np.argsort(np.argsort(ya)).astype(float)
    if np.std(rx) <= 1e-12 or np.std(ry) <= 1e-12:
        return 0.0
    return float(np.corrcoef(rx, ry)[0, 1])


def summarize_budget_curve(
    candidates: list[dict[str, Any]],
    pass_key: str = "passes_criterion",
    loss_key: str = "primary_loss",
) -> dict[str, Any]:
    """Compute the only active budget endpoints: Pass@1 and Pass@5."""
    passes = [bool(c.get(pass_key, False)) for c in candidates]
    losses = [float(c.get(loss_key, float("inf"))) for c in candidates]
    first_pass = next((i + 1 for i, p in enumerate(passes) if p), None)

    def best_at(b: int) -> dict[str, Any]:
        sub = candidates[: min(b, len(candidates))]
        if not sub:
            return {"pass": False, "best_loss": float("nan"), "best_config": None}
        sub_losses = [float(c.get(loss_key, float("inf"))) for c in sub]
        passing_ids = [i for i, c in enumerate(sub) if bool(c.get(pass_key, False))]
        eligible_ids = passing_ids if passing_ids else list(range(len(sub)))
        idx = min(eligible_ids, key=lambda i: (sub_losses[i], i))
        return {
            "budget_B": b,
            "evaluated_count": len(sub),
            "Pass_at_B": any(bool(c.get(pass_key, False)) for c in sub),
            "passing_fraction_within_B": sum(1 for c in sub if c.get(pass_key, False)) / len(sub),
            "best_loss_at_B": float(sub_losses[idx]),
            "best_candidate_index_1based": idx + 1,
            "best_candidate_tag": sub[idx].get("config_tag", f"cand_{idx+1}"),
            "selected_passes_criterion": bool(sub[idx].get(pass_key, False)),
            "no_candidate_passed": not bool(passing_ids),
        }

    return {
        "total_candidates": len(candidates),
        "B_first_pass": first_pass,
        "Pass@1": best_at(1)["Pass_at_B"],
        "Pass@5": best_at(5)["Pass_at_B"],
        "stage_B1": best_at(1),
        "stage_B5": best_at(5),
    }
