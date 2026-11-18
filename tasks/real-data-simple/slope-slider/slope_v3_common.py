"""Engine-independent pieces of the GAUGE slope-slider dynamic-friction-v3 run.

Engine run_case(cand, case, cell, init_pos [R,3], n_frames, steps_per_frame, dt) -> dict(pos [R,F,3], finite [R],
truncated int, extra dict) where case comes from load_case() and board motion from board_state().
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

from strict_v3_common import json_safe, load_manifest  # noqa: F401

FPS = 30.0
SPECS = {("wood", "task-1"): 9, ("plastic", "task-1"): 9, ("metal", "task-1"): 9, ("wood", "task-2"): 19,
         ("plastic", "task-2"): 17, ("metal", "task-2"): 19, ("wood", "task-3"): 46, ("plastic", "task-3"): 46,
         ("metal", "task-3"): 60}


def qmul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw])


def conj(q):
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def quat_zyx_wxyz(e):
    x, y, z = [math.radians(v) / 2.0 for v in e]
    cx, sx, cy, sy, cz, sz = math.cos(x), math.sin(x), math.cos(y), math.sin(y), math.cos(z), math.sin(z)
    return np.array([cx * cy * cz + sx * sy * sz, sx * cy * cz - cx * sy * sz, cx * sy * cz + sx * cy * sz, cx * cy * sz - sx * sy * cz])


def load_case(root: Path, material: str, subtask: str) -> dict[str, Any]:
    root = Path(root) / "exp-results/gauge-rigid-reproduction"
    meta = json.loads((root / "metadata" / "slope slider.json").read_text())
    info = meta["tasks"][subtask]; frms = SPECS[(material, subtask)]
    a = meta["assets"]["cube"]["material"][material]; b = meta["assets"]["board"]["material"]["wood"]
    mu = 2.0 * a["friction"] * b["friction"] / (a["friction"] + b["friction"])
    files = sorted((root / "data/full-table5/slope-slider" / material / subtask).glob("*.json"), key=lambda p: int(p.stem))
    cubes, bp, bq = [], [], []
    for f in files:
        raw = json.loads(f.read_text())
        cubes.append(np.column_stack([raw["cube"][k] for k in "xyz"])[:frms] / 1000.0)
        bp.append(np.column_stack([raw["board"][k] for k in "xyz"])[:frms] / 1000.0)
        q = np.column_stack([raw["board"][k] for k in ("qw", "qx", "qy", "qz")])[:frms]
        for i in range(1, len(q)):
            if np.dot(q[i], q[i - 1]) < 0:
                q[i:] *= -1
        bq.append(q)
    mq = np.mean(bq, axis=0); mq /= np.linalg.norm(mq, axis=-1, keepdims=True)
    return {"material": material, "subtask": subtask, "frames": frms, "kinematic": bool(info["kinematic"]["board"]),
            "mass_kg": float(a["mass"]), "mass_board_kg": float(b["mass"]), "mu": float(mu),
            "p_m": np.array(info["translation"]["cube"][material], float), "p_f": np.array(info["translation"]["board"][material], float),
            "q_m": quat_zyx_wxyz(info["orientation"]["cube"][material]), "q_f": quat_zyx_wxyz(info["orientation"]["board"][material]),
            "trials": np.stack(cubes), "mean_cube": np.mean(cubes, axis=0), "board_pos": np.mean(bp, axis=0), "board_quat": mq}


def board_state(case: dict[str, Any], t: float) -> dict[str, Any]:
    """Board pose/velocity at time t (s) for the kinematic task, plus SAP joint targets (dx, dz, theta_y)."""
    pos, quat = case["board_pos"], case["board_quat"]
    tf = min(t * FPS, len(pos) - 1 - 1e-9); i = int(tf); a = tf - i
    p = case["p_f"] + (1 - a) * pos[i] + a * pos[i + 1] - pos[0]
    v = (pos[i + 1] - pos[i]) * FPS
    q0, q1 = quat[i], quat[i + 1]
    if np.dot(q0, q1) < 0:
        q1 = -q1
    q = (1 - a) * q0 + a * q1; q /= np.linalg.norm(q)
    rel = qmul(q, conj(quat[0]))
    if rel[0] < 0:
        rel = -rel
    mesh_q = qmul(rel, case["q_f"])
    d = qmul(q1, conj(q0))
    if d[0] < 0:
        d = -d
    th = 2 * math.atan2(np.linalg.norm(d[1:]), d[0])
    w = d[1:] * th / np.linalg.norm(d[1:]) * FPS if th > 1e-10 else np.zeros(3)
    w_local = qmul(qmul(conj(mesh_q), np.r_[0.0, w]), mesh_q)[1:]
    theta = 2 * math.atan2(rel[2], rel[0])                       # signed rotation about world y
    return {"p": p, "q": mesh_q, "v": v, "w": w, "w_local": w_local,
            "dx": float(p[0] - case["p_f"][0]), "dz": float(p[2] - case["p_f"][2]), "theta": float(theta),
            "vdx": float(v[0]), "vdz": float(v[2]), "wy": float(w[1])}


def init_positions(case, seed, R):
    rng = np.random.default_rng(seed)
    tr = case["trials"]; n = len(tr)
    delta = np.stack([0.25 * (tr[r % n, 0] - case["mean_cube"][0]) for r in range(R)])
    return case["p_m"][None] + delta + rng.uniform(-2e-4, 2e-4, (R, 3))


def metrics(sim, case):
    ref = case["mean_cube"]; t = np.arange(len(ref)) / FPS
    fit_start = 35 if (case["material"] == "metal" and case["subtask"] == "task-3") else 2
    s_ref = np.linalg.norm(ref - ref[0], axis=1); a_ref = np.polyfit(t[fit_start:], s_ref[fit_start:], 2)[0] * 2
    rows = []
    for r in range(len(sim["pos"])):
        p = sim["pos"][r]
        if not sim["finite"][r] or not np.isfinite(p).all():
            rows.append({"finite": False}); continue
        s = np.linalg.norm(p - p[0], axis=1)
        rows.append({"finite": True, "rmse_m": float(np.sqrt(np.mean(np.sum((p - ref) ** 2, axis=1)))),
                     "terminal_error_m": float(np.linalg.norm(p[-1] - ref[-1])), "slide_m": float(s[-1]),
                     "accel_error_m_s2": float(abs(np.polyfit(t[fit_start:], s[fit_start:], 2)[0] * 2 - a_ref)),
                     "static_held": bool(s[-1] < 0.010)})
    ok = [x for x in rows if x["finite"]]
    mean = lambda k: float(np.mean([x[k] for x in ok])) if ok else None
    return {"rows": rows, "n_finite": len(ok), "rmse_m": mean("rmse_m"), "terminal_error_m": mean("terminal_error_m"),
            "slide_m": mean("slide_m"), "ref_slide_m": float(s_ref[-1]), "accel_error_m_s2": mean("accel_error_m_s2"),
            "static_held_fraction": mean("static_held") if case["subtask"] == "task-1" else None}


def run_cell(cand, cell, seed, R, root, run_case):
    t0 = time.perf_counter()
    case = load_case(root, cell["material"], cell["subtask"])
    if cell.get("mass_scale", 1.0) != 1.0:
        case = {**case, "mass_kg": case["mass_kg"] * cell["mass_scale"]}
    frame = 1.0 / FPS; dt_ms = cell["dt_ms"]
    spf = int(round(frame * 1000.0 / dt_ms)); dt = frame / spf
    sim = run_case(cand, case, init_positions(case, seed, R), case["frames"], spf, dt)
    m = metrics(sim, case)
    gate = (m["n_finite"] == R) and int(sim["truncated"]) == 0 and bool(sim.get("tracking_ok", True))
    return {"cell": cell, "candidate": cand, "R": R, "gate": gate, "loss": m["rmse_m"] if gate else math.inf,
            "truncated": int(sim["truncated"]), "tracking": sim.get("tracking"), "dt_exact_s": dt, "wall_s": time.perf_counter() - t0, **m}


def execute(manifest, engine_key, parse, run_case, root, log=print):
    R, seed0 = manifest["R"], manifest["seed0"]
    cands = [parse(t) for t in manifest["candidate_rule"][engine_key]]
    fmt = lambda r: (f'gate={r["gate"]} rmse={r["rmse_m"]} term={r["terminal_error_m"]} slide={r["slide_m"]} (real {r["ref_slide_m"]:.3f}) '
                     f'static={r["static_held_fraction"]} track={r["tracking"]}')
    sel = []
    for i, c in enumerate(cands):
        rows = [run_cell(c, cell, seed0 + 100 * s, R, root, run_case) for s, cell in enumerate(manifest["selection"])]
        loss = float(np.mean([r["loss"] for r in rows])) if all(r["gate"] for r in rows) else math.inf
        sel.append({"candidate": c, "candidate_index": i + 1, "loss": loss, "cells": rows, "wall_s": sum(r["wall_s"] for r in rows)})
        for r in rows:
            log(f"select {i + 1} {c['id']} {r['cell']['id']} {fmt(r)}")
    valid = [s for s in sel if math.isfinite(s["loss"])]
    win = min(valid, key=lambda s: (s["loss"], s["wall_s"])) if valid else None
    held = []
    if win:
        log(f"frozen {win['candidate']}")
        for j, cell in enumerate(manifest["held_out_conditions"]):
            r = run_cell(win["candidate"], cell, seed0 + 1000 + 100 * j, R, root, run_case); held.append(r)
            log(f"held {cell['id']} {fmt(r)}")
    return {"selection_candidates": sel, "frozen_index": win["candidate_index"] if win else None,
            "frozen_candidate": win["candidate"] if win else None, "heldout": held}
