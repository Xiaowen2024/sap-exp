#!/usr/bin/env python3
"""Three-box push strict-v4, Genesis + IPC (preliminary).  Scene/labels/perturbations as the RigidSolver runner.

Boxes are IPC-only ABD bodies stacked with g0 = d_hat gaps; the push is an IPC external body force at the COM, ramped
exactly as the manifest.  Pushed-interface load = IPC's own barrier force on the pushed box's lower face
(ContactSystemFeature, direct readout); contact = that face carries barrier force.  The momentum-balance load of the boxes
above is kept as a diagnostic.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
from pathlib import Path

import numpy as np

import genesis_ipc_common as I
import strict_v3_common as common
from run_threebox_genesis_rigid import GRAV, SAMPLE_S, entities

TASK = "three-box"
BOTTOM = lambda p: p[:, 2] < 0          # noqa: E731  body-frame lower face


def sim_cell(c, cell, seed, R):
    dt, mass, mu, half = cell["dt_ms"] * 1e-3, cell["mass_kg"], cell["mu"], cell["box_halfwidth_mm"] * 1e-3
    pushed = 1 if cell["pushed_body"] == "middle" else 2
    expected, _ = common.box_v4_expected(cell)
    if expected != cell["expected"]:
        raise ValueError(f"manifest label disagrees with analytic rule for {cell['id']}")
    force = common.box_v4_force_mg(cell) * mass * GRAV
    xy, yaw, scale = common.box_perturbations(seed, R)
    xml = entities(mu, mass, half)
    g0, dh = I.start_gap(c), I.d_hat(c)
    names = [f"box_{i}" for i in range(3)]
    bodies = {"ground": I.Body(xml["ground"], mu=mu)}
    init = {}
    for i, n in enumerate(names):
        bodies[n] = I.Body(xml[n], mu=mu, mass=mass, force=(i == pushed))
        pos = np.zeros((R, 3)); pos[:, 0:2] = xy[:, i]; pos[:, 2] = half + 2 * half * i + g0 * (i + 1)
        quat = np.zeros((R, 4)); quat[:, 0] = np.cos(yaw[:, i] / 2); quat[:, 3] = np.sin(yaw[:, i] / 2)
        init[n] = (I.T_from(pos, quat), None)
    b = I.build(bodies, dt=dt, cand=c, n_envs=R, init=init)
    total = common.BOX_SETTLE_S + common.BOX_RAMP_S + common.BOX_HOLD_S
    steps, every = round(total / dt), max(1, round(SAMPLE_S / dt))
    lo, up = names[pushed - 1], names[pushed]
    T, P, Q, D, F, FM = [], [], [], [], [], []
    with I.Timer() as tm:
        for step in range(steps):
            fac = float(np.clip((step * dt - common.BOX_SETTLE_S) / common.BOX_RAMP_S, 0, 1))
            b.forces[up][:, 0] = force * scale * fac
            b.step()
            tn = (step + 1) * dt
            if (step + 1) % every == 0 and tn >= common.BOX_SETTLE_S - 1e-12:
                poses = [b.pose(n) for n in names]
                P.append(np.stack([p[0] for p in poses], 1)); Q.append(np.stack([p[1] for p in poses], 1))
                d = b.direct_force(up, BOTTOM)          # IPC barrier force on the pushed box's lower face (direct)
                load_m = sum(b.contact_force(names[j])[:, 2] for j in range(pushed, 3))   # momentum balance (diagnostic)
                T.append(tn - common.BOX_SETTLE_S); D.append(d["nv_active"] > 0); F.append(d["N"][:, 2]); FM.append(load_m)
                if len(T) % 20 == 1:
                    b.rigid_contacts()
    raw = {"T": np.asarray(T), "P": np.asarray(P), "Q": np.asarray(Q), "D": np.asarray(D, float), "F": np.asarray(F),
           "FM": np.asarray(FM), "compiled": b.provenance(), "wall_s": tm.s}
    b.destroy()
    return raw


def run_cell(c, cell, seed, R):
    dt, half = cell["dt_ms"] * 1e-3, cell["box_halfwidth_mm"] * 1e-3
    pushed = 1 if cell["pushed_body"] == "middle" else 2
    expected, _ = common.box_v4_expected(cell)
    force = common.box_v4_force_mg(cell) * cell["mass_kg"] * GRAV
    raw, why = I.ISO.call("run_threebox_genesis_ipc", "sim_cell", tag=f"{c['id']}|{cell['id']}", c=c, cell=cell, seed=seed, R=R)
    if raw is None:
        total = common.BOX_SETTLE_S + common.BOX_RAMP_S + common.BOX_HOLD_S
        steps, every = round(total / dt), max(1, round(SAMPLE_S / dt))
        T = np.array([(s + 1) * dt - common.BOX_SETTLE_S for s in range(steps)
                      if (s + 1) % every == 0 and (s + 1) * dt >= common.BOX_SETTLE_S - 1e-12])
        n = len(T)
        raw = {"T": T, "P": np.full((n, R, 3, 3), np.nan), "Q": np.full((n, R, 3, 4), np.nan), "D": np.zeros((n, R)),
               "F": np.full((n, R), np.nan), "compiled": {"failed": why}, "wall_s": float("nan")}
    P, Q = raw["P"], raw["Q"]
    finite = np.isfinite(P).all(axis=(0, 2, 3)) & np.isfinite(Q).all(axis=(0, 2, 3))
    with np.errstate(all="ignore"):
        res = common.box_v4_summary(raw["T"], P, Q, raw["D"], raw["F"], half=half, pushed=pushed,
                                    expected=expected, finite=finite, truncated=np.zeros(R, int))
    res.update({"cell": cell, "force_N": force, "seed": seed, "candidate": c, "wall_s": raw["wall_s"], "compiled": raw["compiled"],
                "failed": why})
    if "FM" in raw and len(raw["T"]):
        mg = cell["mass_kg"] * GRAV
        # compare only while the pushed box still sits on the one below (relative x displacement < 1 mm); after a slip it
        # leaves the stack and the momentum sum also contains its later impacts
        rel = P[:, :, pushed, 0] - P[:, :, pushed - 1, 0]
        on = np.abs(rel - rel[0]) < 1e-3
        with np.errstate(all="ignore"):
            d = np.where(on, np.abs(raw["F"] - raw["FM"]), np.nan) / mg
            res["readout_check"] = {"load_readout": "direct: IPC barrier force on the pushed box's lower face (ContactSystemFeature)",
                                    "window": "samples with the pushed box within 1 mm of its start on the box below",
                                    "direct_minus_momentum_over_mg_max": float(np.nanmax(d)) if np.isfinite(d).any() else None,
                                    "direct_minus_momentum_over_mg_mean": float(np.nanmean(d)) if np.isfinite(d).any() else None}
    return res


def fmt(r):
    return (f'{r["agree_and_valid_count"]:2d}/20 obs={r["observed_counts"]} face={r["face_disp_final_mm_median"]:.3f}mm '
            f'com={r["com_rel_disp_final_mm_median"]:.3f}mm tilt={r["tilt_max_deg_median"]:.2f}deg creep={r["hold_face_creep_um_s_mean"]:.1f}um/s '
            f'{r["wall_s"]:.0f}s {r.get("failed") or ""}')


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = I.registry_task(TASK, a.manifest)
    man, _ = I.load_manifest(a.manifest)
    R, seed0 = man["R"], man["seed0"]
    cands = [I.parse_candidate(t) for t in I.candidates_for(spec)]
    out = {**I.header(man, sha, spec), "complete": False}
    path = a.out / "genesis_ipc_three_box_strict_v4.json"
    selection, walls = [], []
    for i, c in enumerate(cands):
        rows = [run_cell(c, s, seed0 + k, R) for k, s in enumerate(man["selection"])]
        selection.append(rows); walls.append(sum(r["wall_s"] for r in rows if np.isfinite(r["wall_s"])))
        out["selection_candidates"] = selection; I.save_json(path, out)
        for r in rows:
            print("select", i + 1, c["id"], r["cell"]["id"], r["expected"], fmt(r), flush=True)
    win = common.box_v4_choose(selection, walls); frozen = cands[win]
    out.update(B1=selection[0], frozen_index=win + 1, frozen_candidate=frozen)
    print("frozen", frozen["id"], flush=True)
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(frozen, cell, seed0 + 1000 + j, R); held.append(r)
        out["heldout"] = held; I.save_json(path, out)
        print("held", cell["id"], r["expected"], fmt(r), flush=True)
    sens = []
    byid = {c["id"]: c for c in man["held_out_conditions"]}
    if frozen.get("native"):
        sens.append({"skipped": "frozen setting is the native default; no declared stiffness parameter to scale"})
    else:
        k = 0
        for cid in man["parameter_sensitivity"]["cells"]:
            for s in man["parameter_sensitivity"]["scales"]:
                c = dict(frozen); c["resistance"] = frozen.get("resistance", I.RESISTANCE_DEFAULT) * s
                c["id"] = f'{frozen["id"]} | resistance x{s}'
                r = run_cell(c, byid[cid], seed0 + 2000 + k, R); r["scale"] = s; sens.append(r); k += 1
                print("sens", cid, s, fmt(r), flush=True)
    out.update(parameter_sensitivity=sens, complete=True,
               heldout_accuracy_total=[sum(r["agree_and_valid_count"] for r in held), R * len(held)])
    I.save_json(path, out)


if __name__ == "__main__":
    main()
