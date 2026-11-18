#!/usr/bin/env python3
"""Three-plate strict-v3.1 B=1/B=5 selection + frozen held-out matrix, Genesis RigidSolver.

Scene, perturbations, gates, seeds and summary are the manifest and strict_v3_common (shared with SAP / MuJoCo Warp).
Ground and every plate are separate entities with sliding mu=0.8 (Genesis max rule -> 0.8 on every pair).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import genesis_rigid_common as G
from strict_v3_common import cell_seed, plate_choose, plate_perturbations, plate_summary

GRAV = 9.81
SAMPLE_S = 0.01
TASK = "three-plate"
MU = 0.8


def entities(h: float, mass: float, radius: float, offset: float) -> dict[str, str]:
    ents = {"ground": G.mjcf(f'<geom name="ground" type="plane" size="2 2 .1" friction="{MU} 0 0"/>')}
    for i in range(3):
        ents[f"plate_{i}"] = G.mjcf(
            f'<body name="plate_{i}" pos="{i * offset} 0 {(i + .5) * h}"><freejoint/>'
            f'<geom name="plate_{i}_geom" type="cylinder" size="{radius} {h / 2}" mass="{mass}" friction="{MU} 0 0"/></body>')
    return ents


def run_cell(c, cell, seed, R, duration):
    h, mass = cell["thickness_mm"] * 1e-3, cell["mass_per_plate_kg"]
    radius, offset, dt = cell["radius_mm"] * 1e-3, cell["interplate_x_offset_mm"] * 1e-3, cell["dt_ms"] * 1e-3
    ents = entities(h, mass, radius, offset)
    vec = G.sol_vector(c)
    b = G.build(ents, dt=dt, gravity=(0, 0, -GRAV), cand=c, n_envs=R, sol={k: vec for k in ents})
    plates = [b.ents[f"plate_{i}"] for i in range(3)]
    xy, yaw = plate_perturbations(seed, R)
    for i, e in enumerate(plates):
        q = G.np_(e.get_qpos()).astype(np.float64)
        q[:, 0:2] += xy[:, i]
        q[:, 3] = np.cos(yaw[:, i] / 2); q[:, 4:6] = 0; q[:, 6] = np.sin(yaw[:, i] / 2)
        e.set_qpos(q.astype(np.float32), zero_velocity=True)
    steps, every = round(duration / dt), max(1, round(SAMPLE_S / dt))
    pairs = [("ground", "plate_0"), ("plate_0", "plate_1"), ("plate_1", "plate_2")]
    Z, V, W, F, C = [], [], [], [], []
    with G.Timer() as tm:
        for step in range(steps + 1):
            if step % every == 0:
                qp = [G.np_(e.get_qpos()) for e in plates]; qv = [G.np_(e.get_dofs_velocity()) for e in plates]
                Z.append(np.stack([q[:, 2] for q in qp], 1))
                V.append(np.stack([v[:, 0:3] for v in qv], 1)); W.append(np.stack([v[:, 3:6] for v in qv], 1))
                con = b.contacts()
                loads = [b.pair_normal_load(a, bb, con) for a, bb in pairs]
                F.append(np.stack([x[0] for x in loads], 1)); C.append(np.stack([x[1] for x in loads], 1))
            if step < steps:
                b.scene.step()
    res = plate_summary(np.asarray(Z, float), np.asarray(V, float), np.asarray(W, float), np.asarray(F, float),
                        np.asarray(C, float), h=h, mass=mass, g=GRAV, sample_dt=every * dt, truncated=0)
    res.update({"cell": cell, "seed": seed, "candidate": c, "compiled": b.provenance(), "wall_s": tm.s})
    b.destroy()
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = G.registry_task(TASK, a.manifest)
    man, _ = G.load_manifest(a.manifest)
    R, seed0, dur = man["R"], man["seed0"], man["duration_s"]
    cands = [G.parse_candidate(t, spec) for t in spec["candidates"]]
    out = {**G.header(man, sha, spec), "complete": False, "selection_candidates": [], "heldout": []}
    path = a.out / "genesis_rigid_three_plate_strict_v3.json"
    sel = []
    for i, c in enumerate(cands):
        r = run_cell(c, man["selection"], cell_seed(seed0, "selection", i), R, dur); r["candidate_index"] = i + 1
        sel.append(r); out["selection_candidates"] = sel; G.save_json(path, out)
        print("select", i + 1, c["id"], r["pass_count"], r["failed_gate_counts"], f'{r["max_overlap_mm_mean"]:.4f}mm',
              f'{r["load_error_max_mean"]:.4%}', f'{r["wall_s"]:.0f}s', flush=True)
    win = plate_choose(sel); frozen = cands[win]
    out.update(B1=sel[0], frozen_index=win + 1, frozen_candidate=frozen)
    print("frozen", frozen["id"], flush=True)
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(frozen, cell, cell_seed(seed0, "heldout", j), R, dur); held.append(r)
        out["heldout"] = held; G.save_json(path, out)
        print("held", cell["id"], r["pass_count"], r["failed_gate_counts"], f'{r["max_overlap_mm_mean"]:.4f}mm',
              f'{r["load_error_max_mean"]:.4%}', flush=True)
    out.update(complete=True, heldout_pass_total=[sum(r["pass_count"] for r in held), R * len(held)])
    G.save_json(path, out)


if __name__ == "__main__":
    main()
