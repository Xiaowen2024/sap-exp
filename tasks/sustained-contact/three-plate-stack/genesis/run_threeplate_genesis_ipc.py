#!/usr/bin/env python3
"""Three-plate strict-v3.1, Genesis + IPC (preliminary).  Scene as the RigidSolver runner; IPC routes in the registry.

Plates are IPC-only ABD bodies, each interface starting g0 = d_hat apart.  Interface load = IPC's own barrier force on
the lower face of the upper body (ContactSystemFeature, direct readout); the contact count is the number of those
vertices carrying barrier force.  The momentum-balance chain is kept as a diagnostic (readout_check): its difference from
the direct load is the Newton residual.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

import genesis_ipc_common as I
from run_threeplate_genesis_rigid import MU, GRAV, SAMPLE_S, entities
from strict_v3_common import PLATE_TAIL_S, cell_seed, plate_choose, plate_perturbations, plate_summary

TASK = "three-plate"
CHUNK = 5
BOTTOM = lambda p: p[:, 2] < 0          # noqa: E731  body-frame lower face


def sim_cell(c, cell, seed, R, duration, rows=None):
    """rows: the rollouts of the R-rollout cell simulated in this scene (perturbations are drawn for all R, then sliced,
    so a rollout's start is the same whatever the chunking)."""
    h, mass = cell["thickness_mm"] * 1e-3, cell["mass_per_plate_kg"]
    radius, offset, dt = cell["radius_mm"] * 1e-3, cell["interplate_x_offset_mm"] * 1e-3, cell["dt_ms"] * 1e-3
    xml = entities(h, mass, radius, offset)
    g0, dh = I.start_gap(c), I.d_hat(c)
    bodies = {"ground": I.Body(xml["ground"], mu=MU)}
    xy, yaw = plate_perturbations(seed, R)
    if rows is not None:
        xy, yaw = xy[rows], yaw[rows]; R = len(rows)
    init = {}
    for i in range(3):
        bodies[f"plate_{i}"] = I.Body(xml[f"plate_{i}"], mu=MU, mass=mass)
        pos = np.zeros((R, 3)); pos[:, 0] = i * offset + xy[:, i, 0]; pos[:, 1] = xy[:, i, 1]; pos[:, 2] = (i + .5) * h + g0 * (i + 1)
        quat = np.zeros((R, 4)); quat[:, 0] = np.cos(yaw[:, i] / 2); quat[:, 3] = np.sin(yaw[:, i] / 2)
        init[f"plate_{i}"] = (I.T_from(pos, quat), None)
    b = I.build(bodies, dt=dt, gravity=(0, 0, -GRAV), cand=c, n_envs=R, init=init)
    names = [f"plate_{i}" for i in range(3)]
    steps, every = round(duration / dt), max(1, round(SAMPLE_S / dt))
    Z, V, W, F, C, FM, G_ = [], [], [], [], [], [], []
    with I.Timer() as tm:
        for step in range(steps + 1):
            if step % every == 0:
                pz = np.stack([b.pose(n)[0][:, 2] for n in names], 1)
                vel = [b.vel(n) for n in names]
                Z.append(pz); V.append(np.stack([v[0] for v in vel], 1)); W.append(np.stack([v[1] for v in vel], 1))
                # interface load = IPC barrier force on the lower face of the upper body (direct readout)
                vf = b.vertex_forces()
                low = [b.direct_force(n, BOTTOM, vf) for n in names]
                F.append(np.stack([d["N"][:, 2] for d in low], 1))
                C.append(np.stack([d["nv_active"] for d in low], 1).astype(float))
                fz = np.stack([b.contact_force(n)[:, 2] for n in names], 1)      # momentum balance (diagnostic)
                f2 = fz[:, 2]; f1 = fz[:, 1] + f2; f0 = fz[:, 0] + f1
                FM.append(np.stack([f0, f1, f2], 1))
                gaps = np.stack([pz[:, 0] - h / 2, pz[:, 1] - pz[:, 0] - h, pz[:, 2] - pz[:, 1] - h], 1)
                G_.append(gaps)
                if step % (every * 20) == 0:
                    b.rigid_contacts()
            if step < steps:
                b.step()
    raw = {k: np.asarray(v, float) for k, v in dict(Z=Z, V=V, W=W, F=F, C=C, FM=FM, GAP=G_).items()}
    raw.update(compiled=b.provenance(), wall_s=tm.s)
    b.destroy()
    return raw


def run_cell(c, cell, seed, R, duration):
    h, mass, dt = cell["thickness_mm"] * 1e-3, cell["mass_per_plate_kg"], cell["dt_ms"] * 1e-3
    every = max(1, round(SAMPLE_S / dt)); nT = round(duration / dt) // every + 1
    # a3: rollouts run in chunks of CHUNK scenes (R=20 in one IPC scene needs > 6 GB); a crashed chunk's rollouts are NaN
    parts, comp, why, fails, wall = [], None, None, [], 0.0
    for k in range(0, R, CHUNK):
        rows = list(range(k, min(R, k + CHUNK)))
        r, w = I.ISO.call("run_threeplate_genesis_ipc", "sim_cell", tag=f"{c['id']}|{cell['id']}|rollouts {rows[0]}-{rows[-1]}", c=c,
                          cell=cell, seed=seed, R=R, duration=duration, rows=rows)
        if r is None:
            n = len(rows); nan = np.full((nT, n, 3), np.nan)
            r = dict(Z=nan, V=np.full((nT, n, 3, 3), np.nan), W=np.full((nT, n, 3, 3), np.nan), F=nan, C=np.zeros((nT, n, 3)),
                     FM=nan, GAP=nan, wall_s=0.0)
            fails.append({"rollouts": [rows[0], rows[-1]], "reason": w}); why = why or w
        else:
            comp = comp or r["compiled"]
        wall += r["wall_s"]; parts.append(r)
    raw = {k: np.concatenate([p[k] for p in parts], axis=1) for k in ("Z", "V", "W", "F", "C", "FM", "GAP")}
    raw.update(compiled={**(comp or {"failed": why}), "chunk_size": CHUNK, "chunk_failures": fails}, wall_s=wall)
    res = plate_summary(raw["Z"], raw["V"], raw["W"], raw["F"], raw["C"], h=h, mass=mass, g=GRAV, sample_dt=every * dt, truncated=0, standoff=True)   # a3: mirror of the overlap gate (rest gap <= 5% h)
    res.update({"cell": cell, "seed": seed, "candidate": c, "compiled": raw["compiled"], "wall_s": raw["wall_s"], "failed": why,
                "rest_gap_mm_median": (np.median(raw["Z"][-1] - np.array([.5, 1.5, 2.5]) * h, axis=0) * 1e3).tolist()})
    if np.isfinite(raw["FM"]).any():
        res["readout_check"] = readout_check(raw["F"], raw["FM"], raw["GAP"], mass * GRAV, every * dt)
    return res


def readout_check(F, FM, GAP, mg, sample_dt):
    """Direct (barrier) vs momentum-balance interface loads over the gate window; their gap is the Newton residual."""
    tail = np.arange(len(F)) >= len(F) - round(PLATE_TAIL_S / sample_dt)
    d = np.abs(F[tail] - FM[tail]) / mg
    with np.errstate(all="ignore"):
        return {"load_readout": "direct: IPC barrier force on the lower face of each upper body (ContactSystemFeature)",
                "direct_minus_momentum_over_mg_max": float(np.nanmax(d)) if d.size else None,
                "direct_minus_momentum_over_mg_mean": float(np.nanmean(d)) if d.size else None,
                "rest_gap_mm_by_interface_mean": (np.nanmean(GAP[tail], axis=(0, 1)) * 1e3).tolist() if d.size else None}


def _fmt(r):
    return f'{r["pass_count"]} {r["failed_gate_counts"]} {r["load_error_max_mean"]:.4%} {r["rest_gap_mm_median"]} {r["wall_s"]:.0f}s'


def sharded(a, man, spec, sha, cands) -> None:
    """Cell-parallel execution with the same seeds and selection rule as the sequential path:
    --select-index K runs selection candidate K alone; --heldout-cells ids --frozen-index K runs those held-out cells;
    --merge assembles the standard artifact (frozen index re-derived from the selection shards and checked)."""
    R, seed0, dur = man["R"], man["seed0"], man["duration_s"]
    shard = a.out / "shards"; shard.mkdir(parents=True, exist_ok=True)
    if a.select_index:
        i = a.select_index - 1
        r = run_cell(cands[i], man["selection"], cell_seed(seed0, "selection", i), R, dur); r["candidate_index"] = i + 1
        r["device"] = I.versions().get("device")
        I.save_json(shard / f"select_{i + 1}.json", r); print("select", i + 1, cands[i]["id"], _fmt(r), flush=True)
        return
    if a.heldout_cells:
        frozen = cands[a.frozen_index - 1]
        ids = a.heldout_cells.split(",")
        for j, cell in enumerate(man["held_out_conditions"]):
            if cell["id"] in ids:
                r = run_cell(frozen, cell, cell_seed(seed0, "heldout", j), R, dur); r["device"] = I.versions().get("device")
                r["frozen_index_used"] = a.frozen_index
                I.save_json(shard / f"held_{j:02d}_{cell['id']}.json", r); print("held", cell["id"], _fmt(r), flush=True)
        return
    # merge
    sel = [json.loads((shard / f"select_{i + 1}.json").read_text()) for i in range(len(cands))]
    win = plate_choose(sel); frozen = cands[win]
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = json.loads((shard / f"held_{j:02d}_{cell['id']}.json").read_text())
        if r["frozen_index_used"] != win + 1:
            raise ValueError(f"held-out shard {cell['id']} used candidate {r['frozen_index_used']}, selection froze {win + 1}")
        held.append(r)
    out = {**I.header(man, sha, spec), "complete": True, "execution": "cell-parallel shards (same seeds and rule)",
           "selection_candidates": sel, "B1": sel[0], "frozen_index": win + 1, "frozen_candidate": frozen, "heldout": held,
           "heldout_pass_total": [sum(r["pass_count"] for r in held), R * len(held)]}
    I.save_json(a.out / "genesis_ipc_three_plate_strict_v3.json", out)
    print("merged frozen", frozen["id"], out["heldout_pass_total"], flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--select-index", type=int); ap.add_argument("--heldout-cells"); ap.add_argument("--frozen-index", type=int)
    ap.add_argument("--merge", action="store_true")
    a = ap.parse_args()
    _, spec, sha = I.registry_task(TASK, a.manifest)
    man, _ = I.load_manifest(a.manifest)
    R, seed0, dur = man["R"], man["seed0"], man["duration_s"]
    cands = [I.parse_candidate(t) for t in I.candidates_for(spec)]
    if a.select_index or a.heldout_cells or a.merge:
        return sharded(a, man, spec, sha, cands)
    out = {**I.header(man, sha, spec), "complete": False, "selection_candidates": [], "heldout": []}
    path = a.out / "genesis_ipc_three_plate_strict_v3.json"
    sel = []
    for i, c in enumerate(cands):
        r = run_cell(c, man["selection"], cell_seed(seed0, "selection", i), R, dur); r["candidate_index"] = i + 1
        sel.append(r); out["selection_candidates"] = sel; I.save_json(path, out)
        print("select", i + 1, c["id"], r["pass_count"], r["failed_gate_counts"], f'{r["load_error_max_mean"]:.4%}',
              r["rest_gap_mm_median"], f'{r["wall_s"]:.0f}s', flush=True)
    win = plate_choose(sel); frozen = cands[win]
    out.update(B1=sel[0], frozen_index=win + 1, frozen_candidate=frozen)
    print("frozen", frozen["id"], flush=True)
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(frozen, cell, cell_seed(seed0, "heldout", j), R, dur); held.append(r)
        out["heldout"] = held; I.save_json(path, out)
        print("held", cell["id"], r["pass_count"], r["failed_gate_counts"], f'{r["load_error_max_mean"]:.4%}',
              r["rest_gap_mm_median"], f'{r["wall_s"]:.0f}s', flush=True)
    out.update(complete=True, heldout_pass_total=[sum(r["pass_count"] for r in held), R * len(held)])
    I.save_json(path, out)


if __name__ == "__main__":
    main()
