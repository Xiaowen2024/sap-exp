"""Shared logic for the GAUGE pendulum strict-v3 run (SAP Warp and MuJoCo Warp).

Scene = the GAUGE pendulum (released ball-6.obj, 0.3175 kg, ball centre 0.278 m below a world-y hinge at the metadata
anchor, no damping, released from rest at the trial's measured angle ~90 deg).  Metrics = the GAUGE paper's:
PD = 1 / dominant FFT frequency of the 300-frame x trajectory (raw FFT, as in the paper); EL = (E_start - E_end) / E_start over 3000 frames, using
peak potential energy (release peak and last swing peak).  Real reference for PD = the trials' own PD with the same
code; EL target = 0 (the paper's undamped ideal; the real rig is damped).

Engine simulate(cand, cell, th0 [W]) -> (Q [S, W] hinge angle, finite [W]) sampled every 1/30 s, S = cell frames.
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

FPS, PD_FRAMES = 30, 300
L, MASS, RAD = 0.278, 0.3175, 0.03
ANCHOR = np.array([0.37396, 0.49059, 0.445133])
MESH = "exp-results/gauge-rigid-reproduction/assets/ball-6.obj"
DATA = "exp-results/gauge-rigid-reproduction/data/pendulum"


def ball_inertia(mass_scale=1.0):
    return 0.4 * MASS * mass_scale * RAD ** 2


def load_trials(root):
    out = []
    for i in range(1, 21):
        raw = json.loads((Path(root) / DATA / f"{i}.json").read_text())
        assert raw["FPS"] == FPS and raw["translation unit"] == "mm"
        out.append({"id": i, "p": np.column_stack([raw["ball"][a] for a in "xyz"]) / 1000.0})
    return out


def trial_ids(spec):
    return list(range(1, 21, 2)) if spec == "odd trials" else list(range(2, 21, 2))


def release_angle(trial):
    """Hinge angle about +y: R_y(th) (0,0,-L) = (-L sin th, 0, -L cos th)."""
    d = trial["p"][0] - ANCHOR
    return math.atan2(-d[0], -d[2])


def positions(Q):
    return np.stack([ANCHOR[0] - L * np.sin(Q), np.full_like(Q, ANCHOR[1]), ANCHOR[2] - L * np.cos(Q)], axis=-1)


def period_fft(x, refine=False):
    """GAUGE PD: reciprocal of the dominant FFT frequency of the 300-frame trajectory (raw FFT, 0.1-Hz bins; this
    reproduces the paper's quantised values, e.g. 1.25/1.14 = 1.096 for the engines).  refine=True: zero-padded x64 with
    parabolic peak interpolation (reported as a finer mechanism metric only)."""
    x = np.asarray(x, float) - np.mean(x)
    if not refine:
        a = np.abs(np.fft.rfft(x)); f = np.fft.rfftfreq(len(x), 1.0 / FPS); return float(1.0 / f[int(np.argmax(a[1:])) + 1])
    n = len(x) * 64
    a = np.abs(np.fft.rfft(x, n)); f = np.fft.rfftfreq(n, 1.0 / FPS); k = int(np.argmax(a[1:])) + 1
    if 1 <= k < len(a) - 1:
        y0, y1, y2 = np.log(a[k - 1] + 1e-300), np.log(a[k] + 1e-300), np.log(a[k + 1] + 1e-300)
        d = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2) if (y0 - 2 * y1 + y2) != 0 else 0.0
        return 1.0 / (f[k] + d * (f[1] - f[0]))
    return 1.0 / f[k]


def energy_loss(z, z_eq):
    """GAUGE eq. 17 at peaks: E ~ potential at the release peak and at the last full swing peak."""
    pk = [i for i in range(2, len(z) - 2) if z[i] > z[i - 1] and z[i] >= z[i + 1] and z[i] >= z[i - 2] and z[i] >= z[i + 2]]
    if not pk:
        return None
    return float(1.0 - (z[pk[-1]] - z_eq) / (z[0] - z_eq))


def run_cell(cand, cell, trials, seed, R, simulate):
    t0 = time.perf_counter(); tr = [t for t in trials if t["id"] in trial_ids(cell["trials"])]
    rng = np.random.default_rng(seed)
    th0 = np.concatenate([release_angle(t) + rng.uniform(-3e-3, 3e-3, R) for t in tr])
    Q, fin = simulate(cand, cell, th0)
    z_eq = ANCHOR[2] - L; rows = []
    for j, t in enumerate(tr):
        pd_real = period_fft(t["p"][:PD_FRAMES, 0]); pdf_real = period_fft(t["p"][:PD_FRAMES, 0], refine=True)
        for r in range(R):
            w = j * R + r
            if not fin[w]:
                rows.append({"trial": t["id"], "finite": False}); continue
            p = positions(Q[:, w]); pd = period_fft(p[:PD_FRAMES, 0]); pdf = period_fft(p[:PD_FRAMES, 0], refine=True)
            el = energy_loss(p[:, 2], z_eq)
            n = min(PD_FRAMES, len(t["p"])); rmse = float(np.sqrt(np.mean(np.sum((p[:n] - t["p"][:n]) ** 2, axis=1))))
            rows.append({"trial": t["id"], "finite": True, "pd_s": pd, "pd_real_s": pd_real, "pd_ratio": pd / pd_real, "pd_fine_s": pdf, "pd_fine_real_s": pdf_real, "el": el,
                         "rmse_300_mm": rmse * 1e3, "loss": abs(pd / pd_real - 1.0) + abs(el if el is not None else 1.0)})
    ok = [x for x in rows if x["finite"]]
    mean = lambda k: float(np.mean([x[k] for x in ok])) if ok else None
    gate = len(ok) == len(rows) and all(x["el"] is not None for x in ok)
    return {"cell": cell, "candidate": cand, "R": R, "n": len(rows), "gate": gate, "loss": mean("loss") if gate else math.inf,
            "pd_s": mean("pd_s"), "pd_real_s": mean("pd_real_s"), "pd_ratio": mean("pd_ratio"), "pd_fine_s": mean("pd_fine_s"),
            "pd_fine_real_s": mean("pd_fine_real_s"), "el": mean("el"),
            "el_min": float(min(x["el"] for x in ok if x["el"] is not None)) if ok else None, "rmse_300_mm": mean("rmse_300_mm"),
            "wall_s": time.perf_counter() - t0, "rows": rows}


def execute(man, key, parse, simulate, root, log=print):
    R, s0 = man["R"], man["seed0"]; trials = load_trials(root)
    cands = [parse(t) for t in man["candidate_rule"][key]]
    F = lambda v: "None" if v is None else f"{v:.5f}"
    fmt = lambda r: (f'gate={r["gate"]} loss={F(r["loss"]) if math.isfinite(r["loss"]) else "inf"} PD={F(r["pd_s"])}s (real {F(r["pd_real_s"])}, '
                     f'ratio {F(r["pd_ratio"])}; fine {F(r["pd_fine_s"])} vs {F(r["pd_fine_real_s"])}) EL={F(r["el"])} (min {F(r["el_min"])}) RMSE300={F(r["rmse_300_mm"])}mm wall={r["wall_s"]:.0f}s')
    sel = []
    for i, c in enumerate(cands):
        r = run_cell(c, man["selection"], trials, s0, R, simulate); r["candidate_index"] = i + 1; sel.append(r); log(f"select {i + 1} {c['id']} {fmt(r)}")
    valid = [r for r in sel if math.isfinite(r["loss"])]
    win = min(valid, key=lambda r: (r["loss"], r["wall_s"])) if valid else None
    held = []
    if win:
        log(f"frozen {win['candidate']}")
        for j, cell in enumerate(man["held_out_conditions"]):
            r = run_cell(win["candidate"], cell, trials, s0 + 1000 + j, R, simulate); held.append(r); log(f"held {cell['id']} {fmt(r)}")
    return {"selection_candidates": sel, "frozen_index": win["candidate_index"] if win else None,
            "frozen_candidate": win["candidate"] if win else None, "heldout": held}
