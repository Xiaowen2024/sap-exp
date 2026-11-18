"""Engine-independent pieces of the cube-toss resting-tail sustained-contact-v2 run.

Engine simulate(cand, *, dt, mass_scale, mu, scale, init, n_samples, steps_per_sample) must return
pos [W,S,3], quat [W,S,4] (wxyz), speed [W,S], omega [W,S] (|linear|, |angular|), fn [W,S] (total support
normal force), ncon [W,S], finite [W], truncated int.
"""
from __future__ import annotations

import math
import time
from typing import Any, Callable

import numpy as np

import impact_v2_common as ic
from strict_v3_common import json_safe, load_manifest, wilson  # noqa: F401

G = 9.81
MASS, HALF = 0.37, ic.CUBE_HALF
DURATION_S = 6.0
FRAME = 1.0 / ic.CUBE_FPS
SAMPLE = 2 * FRAME                         # 13.5 ms, an integer number of steps at every declared dt
SETTLE_S, MIN_TAIL_S = 0.2, 1.0
GATE = {"load": 0.01, "overlap_m": 1e-3, "drift_m_s": 1e-4, "yaw_rad_s": 1e-3}


def starts(cell, ids, data, seeds, R):
    s, L, _ = data
    probs = ic.cell_problem("cube", {"yaw_deg": 0}, ids, (s, L))
    init = np.concatenate([ic.perturb("cube", p["x0"], sd, R) for p, sd in zip(probs, seeds)])
    sc = float(cell.get("scale", 1.0))
    if sc != 1.0:
        init[:, 2] += max(0.0, (sc - 1.0) * HALF * math.sqrt(3))
    return probs, init


def corners_min_z(pos, quat, half):
    R = ic.quat_to_mat(quat)                                       # [...,3,3]
    signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float) * half
    z = pos[..., None, 2] + np.einsum("...ij,kj->...ki", R, signs)[..., 2]
    return z.min(axis=-1)


def rollout_metrics(t, pos, quat, speed, omega, fn, ncon, finite, mass, half, standoff=False):
    """standoff=True adds the mirror of the overlap gate for engines that rest separated (IPC barrier): the lowest
    corner may not float more than GATE['overlap_m'] above the floor over the tail window.  Off by default; engines that
    penetrate at rest are unaffected either way."""
    if not finite:
        return {"settled": False, "finite": False}
    good = (speed < 0.01) & (omega < 0.05)
    k = int(round(SETTLE_S / SAMPLE))
    start = next((i for i in range(len(t) - k) if good[i:i + k].all() and t[-1] - t[i] >= MIN_TAIL_S), None)
    if start is None:
        return {"settled": False, "finite": True, "contact_duty": float((ncon > 0).mean())}
    sl = slice(start, None); T = t[-1] - t[start]
    load_err = abs(float(fn[sl].mean()) - mass * G) / (mass * G)
    overlap = max(0.0, -float(corners_min_z(pos[sl], quat[sl], half).min()))
    drift = float(np.linalg.norm(pos[-1, :2] - pos[start, :2]) / T)
    qa, qb = quat[start], quat[-1]
    rot = float(2 * np.arccos(np.clip(abs(np.dot(qa, qb)), -1, 1))) / T
    gate = {"load": load_err <= GATE["load"], "overlap": overlap <= GATE["overlap_m"],
            "drift": drift <= GATE["drift_m_s"], "rot": rot <= GATE["yaw_rad_s"]}
    out = {"settled": True, "finite": True, "settle_time_s": float(t[start]), "tail_s": float(T),
           "load_error": load_err, "overlap_mm": overlap * 1e3, "drift_mm_s": drift * 1e3, "rot_drift_mrad_s": rot * 1e3,
           "contact_duty": float((ncon[sl] > 0).mean()), "contacts_mean": float(ncon[sl].mean())}
    if standoff:
        sep = max(0.0, float(corners_min_z(pos[sl], quat[sl], half).max()))
        gate["standoff"] = sep <= GATE["overlap_m"]; out["standoff_mm"] = sep * 1e3
    return {**out, "passes": bool(all(gate.values())), "gate": gate}


def run_cell(cand, cell, ids, data, seeds, R, simulate, standoff=False):
    t0 = time.perf_counter()
    probs, init = starts(cell, ids, data, seeds, R)
    dt = cell["dt_ms"] * 1e-3
    sps = int(round(SAMPLE / dt)); S = int(round(DURATION_S / SAMPLE)) + 1
    dt = SAMPLE / sps
    sim = simulate(cand, dt=dt, mass_scale=float(cell.get("mass_scale", 1.0)), mu=float(cell.get("mu", 0.18)),
                   scale=float(cell.get("scale", 1.0)), init=init, n_samples=S, steps_per_sample=sps)
    t = np.arange(S) * SAMPLE
    mass = MASS * float(cell.get("mass_scale", 1.0)); half = HALF * float(cell.get("scale", 1.0))
    rows = []
    for w in range(len(init)):
        m = rollout_metrics(t, sim["pos"][w], sim["quat"][w], sim["speed"][w], sim["omega"][w], sim["fn"][w], sim["ncon"][w],
                            bool(sim["finite"][w]), mass, half, standoff=standoff)
        m["trial"] = probs[w // R]["trial"]; m["rollout"] = w % R
        if int(sim["truncated"]):
            m["passes"] = False
        rows.append(m)
    n = len(rows); k = sum(bool(r.get("passes")) for r in rows); st = [r for r in rows if r.get("settled")]
    mean = lambda key: (float(np.mean([r[key] for r in st])) if st else None)
    fails = {g: sum(1 for r in st if not r["gate"][g]) for g in ("load", "overlap", "drift", "rot") + (("standoff",) if standoff else ())}
    return {"cell": cell, "candidate": cand, "n": n, "pass_count": k, "pass_fraction": k / n, "pass_wilson_95ci": wilson(k, n),
            "settled_count": len(st), "failed_gate_counts": {"not_settled": n - len(st), **fails},
            "load_error_mean": mean("load_error"), "overlap_mm_mean": mean("overlap_mm"), "drift_mm_s_mean": mean("drift_mm_s"),
            "rot_drift_mrad_s_mean": mean("rot_drift_mrad_s"), "settle_time_s_mean": mean("settle_time_s"),
            **({"standoff_mm_mean": mean("standoff_mm")} if standoff else {}),
            "truncated": int(sim["truncated"]), "dt_exact_s": dt, "wall_s": time.perf_counter() - t0, "rollouts": rows}


def execute(manifest, engine_key, parse, simulate, data, log=print, standoff=False):
    R, seed0 = manifest["R"], manifest["seed0"]
    sel_ids, held_ids = ic.trial_ids("cube", "selection", data[2]), ic.trial_ids("cube", "heldout", data[2])
    cands = [parse(t) for t in manifest["candidate_rule"][engine_key]]
    fmt = lambda r: (f'{r["pass_count"]}/{r["n"]} settled={r["settled_count"]} fails={r["failed_gate_counts"]} '
                     f'load={r["load_error_mean"]} overlap={r["overlap_mm_mean"]}mm drift={r["drift_mm_s_mean"]}mm/s')
    sel = []
    for i, c in enumerate(cands):
        r = run_cell(c, manifest["selection"], sel_ids, data, [seed0 + k for k in range(len(sel_ids))], R, simulate, standoff)
        r["candidate_index"] = i + 1; sel.append(r); log(f"select {i + 1} {c['id']} {fmt(r)} wall={r['wall_s']:.0f}s")
    key = lambda r: (-r["pass_fraction"], r["load_error_mean"] if r["load_error_mean"] is not None else 9e9,
                     r["drift_mm_s_mean"] if r["drift_mm_s_mean"] is not None else 9e9, r["wall_s"])
    win = min(sel, key=key); frozen = win["candidate"]; log(f"frozen {frozen}")
    held = []
    for j, cell in enumerate(manifest["held_out_conditions"]):
        r = run_cell(frozen, cell, held_ids, data, [seed0 + 1000 + 100 * j + k for k in range(len(held_ids))], R, simulate, standoff)
        held.append(r); log(f"held {cell['id']} {fmt(r)}")
    return {"selection_candidates": sel, "frozen_index": win["candidate_index"], "frozen_candidate": frozen, "heldout": held,
            "heldout_pass_total": [sum(r["pass_count"] for r in held), sum(r["n"] for r in held)]}
