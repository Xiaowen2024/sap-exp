"""Shared logic for the MIT Push dynamic-friction-v3 run (SAP Warp and MuJoCo Warp).

Engine run_trial(cand, trial, cell, init_xy [R,2], steps_per_sample, dt, n_samples, frames_per_sample) returns
pose [R,S,3] (x, y, yaw in the object-0 frame), twist [R,S,3], wrench [R,S,2] (force ON the pusher), duty [R,S],
finite [R], truncated int, tracking_ok bool, tracking dict.
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

from strict_v3_common import json_safe, load_manifest  # noqa: F401

REF_DT = 1.0 / 180.0
WINDOWS = [(0.12, 0.20), (0.22, 0.32), (0.40, 0.50), (0.60, 0.72)]
MASS = 0.9827
INERTIA = {"hex": (0.00075, 0.00075, 0.001497), "rect1": (0.00057, 0.00057, 0.001131)}
MU_TABLE, MU_PUSH = 0.22, 0.25
HALF_Z = 0.00635                     # released meshes are 12.7 mm thick
TIP_R, TIP_HH, TIP_Z = 0.00475, 0.015, HALF_Z + 0.010


def load_trial(root, filename, shape):
    raw = json.loads((Path(root) / "exp-results/mit-push-v2/assets" / filename).read_text())
    tip, obj, ft = (np.asarray(raw[k], float) for k in ("tip_pose", "object_pose", "ft_wrench"))
    t0, t1 = max(tip[0, 0], obj[0, 0], ft[0, 0]), min(tip[-1, 0], obj[-1, 0], ft[-1, 0])
    tg = np.arange(t0, t1, REF_DT); f = lambda a, c: np.interp(tg, a[:, 0], a[:, c])
    ox, oy, oth = f(obj, 1), f(obj, 2), np.interp(tg, obj[:, 0], np.unwrap(obj[:, 3]))
    a0 = oth[0]; Rm = np.array([[math.cos(-a0), -math.sin(-a0)], [math.sin(-a0), math.cos(-a0)]])
    oxy = np.column_stack([ox - ox[0], oy - oy[0]]) @ Rm.T
    txy = np.column_stack([f(tip, 1) - ox[0], f(tip, 2) - oy[0]]) @ Rm.T
    fxy = np.column_stack([f(ft, 1), f(ft, 2)]) @ Rm.T
    t = tg - tg[0]; pose = np.column_stack([oxy, oth - a0])
    tw = np.column_stack([np.gradient(pose[:, 0], t), np.gradient(pose[:, 1], t), np.gradient(pose[:, 2], t)])
    return {"shape": shape, "file": filename, "t": t, "tip": txy, "pose": pose, "twist": tw, "wrench": fxy}


def tip_target(trial, t):
    tt = trial["t"]; x = np.interp(t, tt, trial["tip"][:, 0]); y = np.interp(t, tt, trial["tip"][:, 1])
    h = REF_DT
    vx = (np.interp(t + h, tt, trial["tip"][:, 0]) - np.interp(t - h, tt, trial["tip"][:, 0])) / (2 * h)
    vy = (np.interp(t + h, tt, trial["tip"][:, 1]) - np.interp(t - h, tt, trial["tip"][:, 1])) / (2 * h)
    return float(x), float(y), float(vx), float(vy)


def init_xy(seed, R):
    return np.random.default_rng(seed).normal(0.0, 2e-4, (R, 2))


def sampling(dt_ms, n_ref):
    ratio = REF_DT * 1000.0 / dt_ms
    if ratio >= 1 - 1e-9:
        sps, fps_ = int(round(ratio)), 1
    else:
        sps, fps_ = 1, int(round(1 / ratio))
    return sps, fps_, (n_ref - 1) // fps_ + 1


def metrics(sim, trial, fps_, mass_scale):
    ref_pose, ref_tw, ref_w = (trial[k][::fps_] for k in ("pose", "twist", "wrench"))
    t = trial["t"][::fps_]; S = sim["pose"].shape[1]
    rows = []
    for r in range(sim["pose"].shape[0]):
        if not sim["finite"][r]:
            rows.append({"finite": False}); continue
        p, tw, w = sim["pose"][r], sim["twist"][r], sim["wrench"][r] / mass_scale
        trans = float(np.linalg.norm(p[-1, :2] - ref_pose[S - 1, :2])); yaw = float(abs(p[-1, 2] - ref_pose[S - 1, 2]))
        win = []
        for a, b in WINDOWS:
            m = (t[:S] >= a) & (t[:S] <= b)
            if m.any():
                win.append({"window_s": [a, b], "twist_error": float(np.linalg.norm(tw[m].mean(0) - ref_tw[:S][m].mean(0))),
                            "wrench_error_N": float(np.linalg.norm(w[m].mean(0) - ref_w[:S][m].mean(0)))})
        rows.append({"finite": True, "final_translation_error_m": trans, "final_yaw_error_rad": yaw, "loss": trans + 0.05 * yaw,
                     "twist_error_mean": float(np.mean([x["twist_error"] for x in win])) if win else None,
                     "wrench_error_mean_N": float(np.mean([x["wrench_error_N"] for x in win])) if win else None,
                     "push_contact_duty": float(sim["duty"][r].mean()), "windows": win})
    ok = [x for x in rows if x["finite"]]
    mean = lambda k: float(np.mean([x[k] for x in ok])) if ok else None
    return {"rows": rows, "n_finite": len(ok), "final_translation_error_m": mean("final_translation_error_m"),
            "final_yaw_error_rad": mean("final_yaw_error_rad"), "loss_mean": mean("loss"), "twist_error_mean": mean("twist_error_mean"),
            "wrench_error_mean_N": mean("wrench_error_mean_N"), "push_contact_duty": mean("push_contact_duty"),
            "ref_final_pose": trial["pose"][-1].tolist()}


def run_cell(cand, cell, seed, R, root, run_trial):
    t0 = time.perf_counter()
    trial = load_trial(root, cell["trial_file"], cell["shape"])
    sps, fps_, S = sampling(cell["dt_ms"], len(trial["t"]))
    dt = REF_DT / sps if fps_ == 1 else REF_DT * fps_
    sim = run_trial(cand, trial, cell, init_xy(seed, R), sps, dt, S, fps_)
    m = metrics(sim, trial, fps_, float(cell.get("mass_scale", 1.0)))
    gate = m["n_finite"] == R and int(sim["truncated"]) == 0 and bool(sim.get("tracking_ok", True))
    return {"cell": cell, "candidate": cand, "R": R, "gate": gate, "loss": m["loss_mean"] if gate else math.inf,
            "truncated": int(sim["truncated"]), "tracking": sim.get("tracking"), "dt_exact_s": dt, "wall_s": time.perf_counter() - t0, **m}


def execute(man, key, parse, run_trial, root, log=print):
    R, s0 = man["R"], man["seed0"]
    cands = [parse(t) for t in man["candidate_rule"][key]]
    fmt = lambda r: (f'gate={r["gate"]} loss={r["loss_mean"]} trans={r["final_translation_error_m"]} yaw={r["final_yaw_error_rad"]} '
                     f'twist={r["twist_error_mean"]} wrench={r["wrench_error_mean_N"]}N duty={r["push_contact_duty"]} track={r["tracking"]}')
    sel = []
    for i, c in enumerate(cands):
        r = run_cell(c, man["selection"], s0, R, root, run_trial); r["candidate_index"] = i + 1; sel.append(r)
        log(f"select {i + 1} {c['id']} {fmt(r)}")
    valid = [r for r in sel if math.isfinite(r["loss"])]
    win = min(valid, key=lambda r: (r["loss"], r["wall_s"])) if valid else None
    held = []
    if win:
        log(f"frozen {win['candidate']}")
        for j, cell in enumerate(man["held_out_conditions"]):
            r = run_cell(win["candidate"], cell, s0 + 1000 + j, R, root, run_trial); held.append(r); log(f"held {cell['id']} {fmt(r)}")
    return {"selection_candidates": sel, "frozen_index": win["candidate_index"] if win else None,
            "frozen_candidate": win["candidate"] if win else None, "heldout": held}
