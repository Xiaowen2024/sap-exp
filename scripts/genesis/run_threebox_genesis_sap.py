#!/usr/bin/env python3
"""Three-box push strict-v4: two-cell B=1/B=5 selection, frozen held-out matrix, +-10% stiffness sensitivity.
Genesis SAP coupler.  Labels, perturbations and summary come from strict_v3_common (shared with SAP / MuJoCo Warp).

Ground and three boxes are separate entities; box and ground friction = cell mu (Genesis max rule -> mu on every pair).
The pushed box receives +x force * scale * ramp at its COM every step, as MuJoCo's xfrc_applied.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

import genesis_sap_common as G
import strict_v3_common as common

GRAV = 9.81
SAMPLE_S = 0.01
TASK = "three-box"
D_IMP = 0.9


def entities(mu: float, mass: float, half: float) -> dict[str, str]:
    ents = {"ground": G.mjcf(f'<geom name="ground" type="plane" size="2 2 0.1" friction="{mu} 0 0"/>')}
    for i in range(3):
        ents[f"box_{i}"] = G.mjcf(
            f'<body name="box_{i}" pos="0 0 {half + 2 * half * i}"><freejoint/>'
            f'<geom name="box_{i}_geom" type="box" size="{half} {half} {half}" mass="{mass}" friction="{mu} 0 0"/></body>')
    return ents


def sol_for(c, mass: float) -> dict[str, list[float] | None]:
    """Pair targets of the MuJoCo k_pair candidate: ground s=(1-d)k/m, box-box 2s, critically damped; Genesis mixes
    geom params by their mean, so ground geom = 2*ground_target - box value."""
    return {}



def sim_cell(c, cell, seed, R):
    dt, mass, mu, half = cell["dt_ms"] * 1e-3, cell["mass_kg"], cell["mu"], cell["box_halfwidth_mm"] * 1e-3
    pushed = 1 if cell["pushed_body"] == "middle" else 2
    expected, _ = common.box_v4_expected(cell)
    if expected != cell["expected"]:
        raise ValueError(f"manifest label disagrees with analytic rule for {cell['id']}")
    force = common.box_v4_force_mg(cell) * mass * GRAV
    xy, yaw, scale = common.box_perturbations(seed, R)
    b = G.build(entities(mu, mass, half), dt=dt, cand=c, n_envs=R, sol=sol_for(c, mass))
    boxes = [b.ents[f"box_{i}"] for i in range(3)]
    for i, e in enumerate(boxes):
        q = G.np_(e.get_qpos()).astype(np.float64)
        q[:, 0:2] += xy[:, i]
        q[:, 3] = np.cos(yaw[:, i] / 2); q[:, 4:6] = 0; q[:, 6] = np.sin(yaw[:, i] / 2)
        e.set_qpos(q.astype(np.float32), zero_velocity=True)
    gs = G.gs_init()
    total = common.BOX_SETTLE_S + common.BOX_RAMP_S + common.BOX_HOLD_S
    steps, every = round(total / dt), max(1, round(SAMPLE_S / dt))
    a, bb = f"box_{pushed - 1}", f"box_{pushed}"
    T, P, Q, D, F = [], [], [], [], []
    wrench = np.zeros((R, 1, 3), np.float32)
    with G.Timer() as tm:
        for step in range(steps):
            fac = float(np.clip((step * dt - common.BOX_SETTLE_S) / common.BOX_RAMP_S, 0, 1))
            wrench[:, 0, 0] = force * scale * fac
            boxes[pushed].apply_links_external_wrench(force=wrench, links_idx_local=[0], ref=gs.link_ref_frame.link_COM)
            b.scene.step()
            tn = (step + 1) * dt
            if (step + 1) % every == 0 and tn >= common.BOX_SETTLE_S - 1e-12:
                qp = [G.np_(e.get_qpos()).astype(np.float64) for e in boxes]
                P.append(np.stack([q[:, 0:3] for q in qp], 1)); Q.append(np.stack([q[:, 3:7] for q in qp], 1))
                fn, cnt = b.pair_normal_load(a, bb)
                T.append(tn - common.BOX_SETTLE_S); D.append(cnt > 0); F.append(fn)
    P, Q = np.asarray(P), np.asarray(Q)
    Q = Q / np.linalg.norm(Q, axis=-1, keepdims=True)
    finite = np.isfinite(P).all(axis=(0, 2, 3)) & np.isfinite(Q).all(axis=(0, 2, 3))
    res = common.box_v4_summary(np.asarray(T), P, Q, np.asarray(D, float), np.asarray(F), half=half, pushed=pushed,
                                expected=expected, finite=finite, truncated=np.zeros(R, int))
    res.update({"cell": cell, "force_N": force, "seed": seed, "candidate": c, "wall_s": tm.s, "compiled": b.provenance()})
    b.destroy()
    return res


def run_cell(c, cell, seed, R):
    r, why = G.ISO.call('run_threebox_genesis_sap','sim_cell',tag=f"{c['id']}|{cell['id']}",c=c,cell=cell,seed=seed,R=R)
    if r is not None:return r
    dt=cell['dt_ms']*1e-3;half=cell['box_halfwidth_mm']*1e-3
    pushed=1 if cell['pushed_body']=='middle' else 2
    total=common.BOX_SETTLE_S+common.BOX_RAMP_S+common.BOX_HOLD_S
    steps,every=round(total/dt),max(1,round(SAMPLE_S/dt))
    T=np.array([(st+1)*dt-common.BOX_SETTLE_S for st in range(steps) if (st+1)%every==0 and (st+1)*dt>=common.BOX_SETTLE_S-1e-12]);n=len(T)
    with np.errstate(all='ignore'):
        r=common.box_v4_summary(T,np.full((n,R,3,3),np.nan),np.full((n,R,3,4),np.nan),np.zeros((n,R)),np.full((n,R),np.nan),half=half,pushed=pushed,expected=cell['expected'],finite=np.zeros(R,bool),truncated=np.zeros(R,int))
    r.update(cell=cell,seed=seed,candidate=c,wall_s=0.,compiled={'failed':why},failed=why)
    return r


def fmt(r):
    return (f'{r["agree_and_valid_count"]:2d}/20 obs={r["observed_counts"]} face={r["face_disp_final_mm_median"]:.3f}mm '
            f'com={r["com_rel_disp_final_mm_median"]:.3f}mm tilt={r["tilt_max_deg_median"]:.2f}deg creep={r["hold_face_creep_um_s_mean"]:.1f}um/s')


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = G.registry_task(TASK, a.manifest)
    man, _ = G.load_manifest(a.manifest)
    R, seed0 = man["R"], man["seed0"]
    cands = [G.parse_candidate(t, spec) for t in spec["candidates"]]
    out = {**G.header(man, sha, spec), "complete": False}
    path = a.out / "genesis_sap_three_box_strict_v4.json"
    selection, walls = [], []
    for i, c in enumerate(cands):
        rows = [run_cell(c, s, seed0 + k, R) for k, s in enumerate(man["selection"])]
        selection.append(rows); walls.append(sum(r["wall_s"] for r in rows))
        out["selection_candidates"] = selection; G.save_json(path, out)
        for r in rows:
            print("select", i + 1, c["id"], r["cell"]["id"], r["expected"], fmt(r), flush=True)
    win = common.box_v4_choose(selection, walls); frozen = cands[win]
    out.update(B1=selection[0], frozen_index=win + 1, frozen_candidate=frozen)
    print("frozen", frozen["id"], flush=True)
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(frozen, cell, seed0 + 1000 + j, R); held.append(r)
        out["heldout"] = held; G.save_json(path, out)
        print("held", cell["id"], r["expected"], fmt(r), flush=True)
    sens = []
    byid = {c["id"]: c for c in man["held_out_conditions"]}
    if frozen.get("native"):
        sens.append({"skipped": "frozen setting is the native default; no declared stiffness parameter to scale"})
    else:
        k = 0
        for cid in man["parameter_sensitivity"]["cells"]:
            for s in man["parameter_sensitivity"]["scales"]:
                c = dict(frozen); c["hydroelastic_stiffness"] = frozen["hydroelastic_stiffness"] * s; c["id"] = f'{frozen["id"]} | hydroelastic_stiffness x{s}'
                r = run_cell(c, byid[cid], seed0 + 2000 + k, R); r["scale"] = s; sens.append(r); k += 1
                print("sens", cid, s, fmt(r), flush=True)
    out.update(parameter_sensitivity=sens, complete=True,
               heldout_accuracy_total=[sum(r["agree_and_valid_count"] for r in held), R * len(held)])
    G.save_json(path, out)


if __name__ == "__main__":
    main()
