"""Engine-independent pieces of the dynamic-impact-v2 runs (GAUGE ball bounce, ContactNets cube toss).

Both engine drivers import this module, so trial selection, start states, the R=20 perturbation arrays,
reference trajectories, cell transforms (dt, mass, offset, yaw, apex re-release) and metrics are identical
across engines.  An engine only has to provide

    simulate(cand, *, task, dt, mass_scale, init, n_samples, steps_per_sample) -> dict(pos [W,S,3],
             quat [W,S,4] (wxyz), finite [W] bool, truncated int)

where init is [W,13] = position, quaternion (wxyz), world linear velocity, body angular velocity.
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
from typing import Any, Callable

import numpy as np

from strict_v3_common import json_safe, load_manifest  # noqa: F401  (re-exported)

BALL_FPS, BALL_GROUND, BALL_RADIUS = 30.0, 0.013297, 0.03
CUBE_FPS, CUBE_HALF = 148.0, 0.0524


# ------------------------------------------------------------------ quaternions (wxyz)
def qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.moveaxis(a, -1, 0); bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw], -1)


def quat_from_rotvec(v: np.ndarray) -> np.ndarray:
    a = np.linalg.norm(v, axis=-1, keepdims=True)
    s = np.where(a > 1e-15, np.sin(a / 2) / np.maximum(a, 1e-15), 0.5)
    return np.concatenate([np.cos(a / 2), s * v], -1)


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = np.moveaxis(q, -1, 0)
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], -2)


# ------------------------------------------------------------------ data
def ball_trials(simexp_root: Path) -> tuple[list[str], list[np.ndarray]]:
    d = Path(simexp_root) / "exp-results/gauge-rigid-reproduction/data/bouncing-ball"
    ps = sorted(d.glob("*.json"), key=lambda p: int(p.stem))
    xyz = [np.column_stack([json.loads(p.read_text())["ball"][a] for a in "xyz"]) / 1000.0 for p in ps]
    if len(xyz) != 22:
        raise RuntimeError(f"expected 22 GAUGE trials, got {len(xyz)}")
    return [p.stem for p in ps], xyz


def ball_peaks(z: np.ndarray) -> tuple[list[int], list[float]]:
    g = BALL_GROUND + BALL_RADIUS
    ids = [i for i in range(1, len(z) - 1) if z[i] >= z[i - 1] and z[i] > z[i + 1] and z[i] > g]
    return ids, [float(z[i] - g) for i in ids]


def cube_data(npz: Path) -> tuple[np.ndarray, np.ndarray, list[int]]:
    d = np.load(npz)
    if str(d.get("angular_velocity_frame", "")) != "body":
        raise RuntimeError("use contactnets_si_v2.npz (corrected body-frame angular velocity)")
    s, L = d["state"], d["lengths"]
    eligible = [i for i, l in enumerate(L) if s[i, 0, 2] > 2 * CUBE_HALF and np.nanmin(s[i, :l, 2]) < .09]
    return s, L, eligible


def trial_ids(task: str, which: str, eligible: list[int] | None = None) -> list[int]:
    if task == "ball":
        return list(range(0, 22, 2)) if which == "selection" else list(range(1, 22, 2))
    return eligible[::2][:32] if which == "selection" else eligible[1::2][:32]


# ------------------------------------------------------------------ cell -> start states and references
def cell_problem(task: str, cell: dict[str, Any], ids: list[int], data: Any) -> list[dict[str, Any]]:
    """Per trial: start state [13] (pos, quat wxyz, v_world, omega_body) and reference (pos [L,3], quat [L,4]|None)."""
    out = []
    for i in ids:
        if task == "ball":
            ref = data[i].copy()
            start = cell.get("start", "frame 0")
            k0 = 0
            if start.startswith("1st") or start.startswith("2nd"):
                pk, _ = ball_peaks(ref[:, 2]); k0 = pk[0 if start.startswith("1st") else 1]
            v = np.zeros(3) if k0 == 0 else (ref[k0 + 1] - ref[k0 - 1]) * BALL_FPS / 2
            ref = ref[k0:]
            off = np.r_[cell.get("offset_m", [0, 0]), 0.0]
            ref = ref + off
            x0 = np.r_[ref[0], 1, 0, 0, 0, v, 0, 0, 0]
            out.append({"trial": int(i), "start_frame": int(k0), "x0": x0, "ref_pos": ref, "ref_quat": None})
        else:
            s, L = data
            ref = s[i, :int(L[i])].copy()
            yaw = math.radians(cell.get("yaw_deg", 0.0))
            if yaw:
                qz = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
                Rz = quat_to_mat(qz)
                ref[:, 0:3] = ref[:, 0:3] @ Rz.T
                ref[:, 3:7] = qmul(np.broadcast_to(qz, ref[:, 3:7].shape), ref[:, 3:7])
                ref[:, 7:10] = ref[:, 7:10] @ Rz.T          # body angular velocity unchanged
            out.append({"trial": int(i), "start_frame": 0, "x0": ref[0, :13].copy(), "ref_pos": ref[:, 0:3], "ref_quat": ref[:, 3:7]})
    return out


def perturb(task: str, x0: np.ndarray, seed: int, R: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    x = np.repeat(x0[None], R, 0).astype(np.float64)
    if task == "ball":
        x[:, 0:3] += rng.normal(0, 5e-4, (R, 3))
        return x
    n = rng.normal(size=(R, 12))
    x[:, 0:3] += 5e-4 * n[:, 0:3]
    x[:, 3:7] = qmul(quat_from_rotvec(np.deg2rad(0.5) * n[:, 3:6]), x[:, 3:7])
    x[:, 3:7] /= np.linalg.norm(x[:, 3:7], axis=1, keepdims=True)
    x[:, 7:10] += 0.01 * n[:, 6:9]
    x[:, 10:13] += 0.05 * n[:, 9:12]
    return x


def sampling(task: str, dt_ms: float, ref_len: int) -> tuple[int, int, int]:
    """(steps_per_sample, frames_per_sample, n_samples) so that sample s is at reference frame s*frames_per_sample."""
    frame = 1000.0 / (BALL_FPS if task == "ball" else CUBE_FPS)
    ratio = frame / dt_ms
    if ratio >= 1 - 1e-9:
        sps, fps_ = int(round(ratio)), 1
        if abs(sps - ratio) > 1e-3:
            raise ValueError(f"dt {dt_ms} ms does not divide the frame period")
    else:
        sps, fps_ = 1, int(round(1 / ratio))
    return sps, fps_, (ref_len - 1) // fps_ + 1


# ------------------------------------------------------------------ metrics
def dtw(a: np.ndarray, b: np.ndarray) -> float:
    n, m = len(a), len(b)
    d = np.linalg.norm(a[:, None] - b[None], axis=2)
    c = np.full((n + 1, m + 1), np.inf); l = np.zeros((n + 1, m + 1), int); c[0, 0] = 0
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            opts = ((c[i - 1, j], l[i - 1, j]), (c[i, j - 1], l[i, j - 1]), (c[i - 1, j - 1], l[i - 1, j - 1]))
            p = min(opts, key=lambda x: x[0]); c[i, j] = p[0] + d[i - 1, j - 1]; l[i, j] = p[1] + 1
    return float(c[n, m] / max(1, l[n, m]))


def ball_metrics(sim_pos: np.ndarray, ref_pos: np.ndarray) -> dict[str, Any]:
    rmse = float(np.sqrt(np.mean(np.sum((sim_pos - ref_pos) ** 2, axis=1))))
    _, ps = ball_peaks(sim_pos[:, 2]); _, pr = ball_peaks(ref_pos[:, 2])
    apex = float(np.mean([abs(a - b) / max(abs(b), 1e-6) for a, b in zip(ps[:3], pr[:3])])) if ps and pr else math.nan
    e = lambda h: [math.sqrt(h[k + 1] / h[k]) for k in range(len(h) - 1) if h[k] > 0]
    return {"rmse_m": rmse, "apex_rel_error": apex, "dtw_m": dtw(sim_pos, ref_pos),
            "sim_apex_m": ps[:4], "ref_apex_m": pr[:4], "sim_e": e(ps[:4]), "ref_e": e(pr[:4])}


def cube_metrics(sim_pos, sim_quat, ref_pos, ref_quat, sym_qangle: Callable) -> dict[str, Any]:
    raw = float(2 * np.arccos(np.clip(abs(np.dot(sim_quat[-1], ref_quat[-1])), -1, 1)))
    up = lambda q: int(np.argmax(np.abs(quat_to_mat(q)[2]))) * 2 + int(quat_to_mat(q)[2, np.argmax(np.abs(quat_to_mat(q)[2]))] < 0)
    return {"position_rmse_m": float(np.sqrt(np.mean(np.sum((sim_pos - ref_pos) ** 2, axis=1)))),
            "position_dtw_m": dtw(sim_pos, ref_pos),
            "final_position_error_m": float(np.linalg.norm(sim_pos[-1] - ref_pos[-1])),
            "final_orientation_sym_rad": float(sym_qangle(sim_quat[-1], ref_quat[-1])),
            "final_orientation_raw_rad": raw, "up_face_agree": float(up(sim_quat[-1]) == up(ref_quat[-1]))}


def _mean(v):
    a = np.asarray([x for x in v if x is not None and np.isfinite(x)], float)
    return float(a.mean()) if len(a) else None


def run_cell(task: str, cand: dict[str, Any], cell: dict[str, Any], ids: list[int], data: Any, seeds: list[int],
             R: int, simulate: Callable, sym_qangle: Callable | None = None) -> dict[str, Any]:
    """One (candidate, cell): batch all trials x R in one simulate() call; per-trial and pooled metrics."""
    probs = cell_problem(task, cell, ids, data)
    t0 = time.perf_counter()
    init = np.concatenate([perturb(task, p["x0"], sd, R) for p, sd in zip(probs, seeds)])
    L = max(len(p["ref_pos"]) for p in probs)
    sps, fps_, S = sampling(task, cell["dt_ms"], L)
    frame_s = 1.0 / (BALL_FPS if task == "ball" else CUBE_FPS)
    dt_exact = frame_s / sps if fps_ == 1 else frame_s * fps_      # manifest dt_ms is rounded; use the exact divisor
    sim = simulate(cand, task=task, dt=dt_exact, mass_scale=float(cell.get("mass_scale", 1.0)),
                   init=init, n_samples=S, steps_per_sample=sps)
    trials = []
    for t, p in enumerate(probs):
        rows = []
        for r in range(R):
            w = t * R + r
            n = (len(p["ref_pos"]) - 1) // fps_ + 1
            rp = p["ref_pos"][::fps_][:n]; sp = sim["pos"][w, :n]
            ok = bool(sim["finite"][w]) and np.isfinite(sp).all()
            if task == "ball":
                m = ball_metrics(sp, rp) if ok else {"rmse_m": math.nan, "apex_rel_error": math.nan}
            else:
                m = cube_metrics(sp, sim["quat"][w, :n], rp, p["ref_quat"][::fps_][:n], sym_qangle) if ok else \
                    {"position_rmse_m": math.nan, "final_orientation_sym_rad": math.nan}
            m["finite"] = ok; rows.append(m)
        if task == "ball":
            rm, ap = _mean([x["rmse_m"] for x in rows]), _mean([x["apex_rel_error"] for x in rows])
            valid = all(x["finite"] and np.isfinite(x["apex_rel_error"]) for x in rows)
            loss = rm + 0.05 * ap if (valid and rm is not None and ap is not None) else math.inf
        else:
            rm, ap = _mean([x["position_rmse_m"] for x in rows]), _mean([x["final_orientation_sym_rad"] for x in rows])
            valid = all(x["finite"] for x in rows)
            loss = rm + 0.05 * ap if (valid and rm is not None and ap is not None) else math.inf
        trials.append({"trial": p["trial"], "start_frame": p["start_frame"], "valid": valid, "loss": loss,
                       "initial_speed_m_s": float(np.linalg.norm(p["x0"][7:10])),
                       "initial_spin_rad_s": float(np.linalg.norm(p["x0"][10:13])), "rollouts": rows})
    keys = ["rmse_m", "apex_rel_error", "dtw_m"] if task == "ball" else \
        ["position_rmse_m", "final_orientation_sym_rad", "final_orientation_raw_rad", "position_dtw_m",
         "final_position_error_m", "up_face_agree"]
    pooled = {k: _mean([x.get(k) for tr in trials for x in tr["rollouts"]]) for k in keys}
    all_valid = all(tr["valid"] for tr in trials) and int(sim["truncated"]) == 0
    loss = float(np.mean([tr["loss"] for tr in trials])) if all_valid else math.inf
    return {"cell": cell, "candidate": cand, "R": R, "n_trials": len(trials), "loss": loss, "all_valid": all_valid,
            "truncated": int(sim["truncated"]), "pooled": pooled, "frames_per_sample": fps_,
            "steps_per_sample": sps, "dt_exact_s": dt_exact, "wall_s": time.perf_counter() - t0, "trials": trials}


def execute(task: str, manifest: dict[str, Any], engine_key: str, parse: Callable, simulate: Callable,
            data: Any, sym_qangle: Callable | None, log: Callable = print) -> dict[str, Any]:
    R, seed0 = manifest["R"], manifest["seed0"]
    elig = data[2] if task == "cube" else None
    payload = data[1] if task == "ball" else (data[0], data[1])
    sel_ids, held_ids = trial_ids(task, "selection", elig), trial_ids(task, "heldout", elig)
    cands = [parse(t) for t in manifest["candidate_rule"][engine_key]]
    selection = []
    for i, c in enumerate(cands):
        r = run_cell(task, c, manifest["selection"], sel_ids, payload, [seed0 + k for k in range(len(sel_ids))],
                     R, simulate, sym_qangle)
        r["candidate_index"] = i + 1; selection.append(r)
        log(f"select {i + 1} {c['id']} loss={r['loss']:.5g} pooled={ {k: (None if v is None else round(v, 5)) for k, v in r['pooled'].items()} } wall={r['wall_s']:.0f}s")
    valid = [r for r in selection if math.isfinite(r["loss"])]
    win = min(valid, key=lambda r: (r["loss"], r["wall_s"])) if valid else None
    held = []
    if win:
        frozen = win["candidate"]; log(f"frozen {frozen}")
        for j, cell in enumerate(manifest["held_out_conditions"]):
            r = run_cell(task, frozen, cell, held_ids, payload, [seed0 + 1000 + 100 * j + k for k in range(len(held_ids))],
                         R, simulate, sym_qangle)
            held.append(r)
            log(f"held {cell['id']} loss={r['loss']:.5g} pooled={ {k: (None if v is None else round(v, 5)) for k, v in r['pooled'].items()} }")
    return {"selection_candidates": selection, "frozen_index": (win["candidate_index"] if win else None),
            "frozen_candidate": (win["candidate"] if win else None), "heldout": held}
