#!/usr/bin/env python3
"""GAUGE pendulum v3.1 (real-data reference), Genesis RigidSolver.

Scene = GAUGE's released pendulum.xml plus the same edits as the MuJoCo Warp driver (anchor position, released ball
mesh, mass, friction 0.01, no collision) and no <option>.  Metrics, trial selection, release angles and perturbations
are the pendulum branch of run_gauge_real_oscillations_mjwarp.py with gauge_pendulum_v3_common (copied verbatim below
because that driver imports mujoco_warp at module level).
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import math
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

import genesis_rigid_common as G
import gauge_pendulum_v3_common as gp

TASK = "pendulum-gauge-real"
ASSETS = G.ROOT / "exp-results/gauge-rigid-reproduction/assets"


def model_xml(cell):
    tree = ET.parse(ASSETS / "source-gauge" / "pendulum.xml").getroot()
    ET.SubElement(tree, "compiler", meshdir=str(ASSETS))
    tree.find("asset/mesh").set("file", Path(gp.MESH).name)
    sps = round((1000 / gp.FPS) / cell["dt_ms"]); dt = 1 / (gp.FPS * sps)
    if abs(dt - cell["dt_ms"] * .001) > 1e-8:
        raise ValueError("Declared dt is not compatible with the 30-Hz sample clock")
    world = tree.find("worldbody")
    pivot = copy.deepcopy(world.find("body")); world.remove(world.find("body"))
    pivot.set("name", "pivot0"); pivot.set("pos", " ".join(map(str, gp.ANCHOR)))
    pivot.find("joint").set("name", "h0")
    ball = pivot.find("body"); ball.set("name", "ball0")
    geom = ball.find("geom"); geom.set("name", "ball_geom0")
    geom.set("mass", str(gp.MASS * cell.get("mass_scale", 1.0))); geom.set("friction", "0.01 0 0")
    geom.set("contype", "0"); geom.set("conaffinity", "0")
    world.append(pivot)
    return ET.tostring(tree, encoding="unicode"), sps, dt


def simulate(c, cell, angles):
    xml, sps, dt = model_xml(cell)
    W, S = len(angles), cell["frames"]
    b = G.build({"pendulum": xml}, dt=dt, cand=c, n_envs=W)
    e = b.ents["pendulum"]
    e.set_dofs_position(np.asarray(angles, np.float32)[:, None], zero_velocity=True)
    q = np.zeros((S, W, 1)); v = np.zeros((S, W, 1))
    for f in range(S):
        q[f] = G.np_(e.get_dofs_position()); v[f] = G.np_(e.get_dofs_velocity())
        if f < S - 1:
            for _ in range(sps):
                b.scene.step()
    finite = np.isfinite(q).all(axis=(0, 2)) & np.isfinite(v).all(axis=(0, 2))
    compiled = {**b.provenance(), "steps_per_frame": sps, "xml_sha256_full": hashlib.sha256(xml.encode()).hexdigest(), "xml": xml}
    b.destroy()
    return q, v, finite, compiled


def real_reference(trial, frames):
    p = trial["p"][:frames]
    return {"pd_s": gp.period_fft(p[:300, 0]), "pd_fine_s": gp.period_fft(p[:300, 0], refine=True),
            "el": gp.energy_loss(p[:, 2], gp.ANCHOR[2] - gp.L)}


def summarize(rows):
    keys = sorted({k for r in rows for k, v in r.items() if isinstance(v, (int, float)) and not isinstance(v, bool)
                   and k not in ("trial", "repeat")})
    out = {}
    for key in keys:
        vals = [r[key] for r in rows if r.get(key) is not None and math.isfinite(r[key])]
        out[key] = {"mean": float(np.mean(vals)) if vals else None, "sd": float(np.std(vals, ddof=1)) if len(vals) > 1 else None,
                    "n": len(vals)}
    return out


def run_cell(c, cell, trials, seed, R, raw_path):
    start = time.perf_counter()
    chosen = [t for t in trials if t["id"] in gp.trial_ids(cell["trials"])]
    rng = np.random.default_rng(seed)
    angles = np.concatenate([gp.release_angle(t) + rng.uniform(-.003, .003, R) for t in chosen])
    q, v, finite, compiled = simulate(c, cell, angles)
    raw_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(raw_path, qpos=q, qvel=v, initial_angles=angles,
                        trial_ids=np.repeat([t["id"] for t in chosen], R), repeat_ids=np.tile(np.arange(R), len(chosen)))
    rows, examples = [], []
    for j, trial in enumerate(chosen):
        ref = real_reference(trial, cell["frames"])
        for r in range(R):
            w = j * R + r
            row = {"trial": trial["id"], "repeat": r, "finite": bool(finite[w]), "capacity_ok": True, "valid": False}
            if finite[w]:
                p = gp.positions(q[:, w, 0]); pd = gp.period_fft(p[:300, 0]); pdf = gp.period_fft(p[:300, 0], refine=True)
                el = gp.energy_loss(p[:, 2], gp.ANCHOR[2] - gp.L)
                valid = el is not None and ref["el"] is not None
                n = min(300, len(p), len(trial["p"]))
                row.update(pd_s=pd, pd_real_s=ref["pd_s"], pd_ratio=pd / ref["pd_s"], pd_fine_s=pdf, pd_fine_real_s=ref["pd_fine_s"],
                           el=el, el_real=ref["el"], el_abs_error=abs(el - ref["el"]) if valid else None,
                           rmse_300_mm=float(np.sqrt(np.mean(np.sum((p[:n] - trial["p"][:n]) ** 2, axis=1))) * 1000),
                           valid=valid, loss=abs(pd / ref["pd_s"] - 1) + abs(el - ref["el"]) if valid else None)
                if r == 0:
                    examples.append({"trial": trial["id"], "sim": p.tolist()})
            rows.append(row)
    valid = [r for r in rows if r["valid"]]
    return {"cell": cell, "candidate": c, "seed": seed, "R": R, "n": len(rows), "valid_count": len(valid),
            "gate": len(valid) == len(rows), "loss": float(np.mean([r["loss"] for r in valid])) if len(valid) == len(rows) else None,
            "metrics": summarize(rows), "rows": rows, "examples": examples, "compiled": compiled,
            "raw_npz": raw_path.name, "raw_sha256": hashlib.sha256(raw_path.read_bytes()).hexdigest(),
            "reference_scope": cell["reference_scope"], "wall_s": time.perf_counter() - start}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = G.registry_task(TASK, a.manifest)
    man, _ = G.load_manifest(a.manifest)
    for name, expected in man["data"]["file_sha256"].items():
        if hashlib.sha256((G.ROOT / name).read_bytes()).hexdigest() != expected:
            raise ValueError("GAUGE source changed: " + name)
    trials = gp.load_trials(G.ROOT)
    cands = [G.parse_candidate(t, spec) for t in spec["candidates"]]
    path = a.out / "genesis_rigid_gauge_pendulum_real.json"
    data = {**G.header(man, sha, spec), "complete": False,
            "real_reference": [{"trial": t["id"], **real_reference(t, man["selection"]["frames"])} for t in trials],
            "selection_candidates": [], "frozen_candidate": None, "heldout": []}
    for i, c in enumerate(cands):
        r = run_cell(c, man["selection"], trials, man["seed0"], man["R"], a.out / f"pendulum_selection_{i + 1}.npz")
        r["candidate_index"] = i + 1; data["selection_candidates"].append(r); G.save_json(path, data)
        print("selection", i + 1, c["id"], "valid", r["valid_count"], "/", r["n"], "loss", r["loss"], flush=True)
    valid = [r for r in data["selection_candidates"] if r["gate"] and r["loss"] is not None]
    if not valid:
        data.update(stop_reason="No valid selection candidate; held-out evaluation not started", selection_complete=True,
                    heldout_status="not_run_no_valid_frozen_candidate", complete=True)
        G.save_json(path, data); print(data["stop_reason"], flush=True); return
    win = min(valid, key=lambda r: (r["loss"], r["wall_s"]))
    data.update(frozen_candidate=win["candidate"], frozen_index=win["candidate_index"]); G.save_json(path, data)
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(win["candidate"], cell, trials, man["seed0"] + 1000 + j, man["R"], a.out / f"pendulum_heldout_{j + 1}.npz")
        data["heldout"].append(r); G.save_json(path, data)
        print("heldout", cell["id"], "valid", r["valid_count"], "/", r["n"], "loss", r["loss"], flush=True)
    data["complete"] = True; G.save_json(path, data)


if __name__ == "__main__":
    main()
