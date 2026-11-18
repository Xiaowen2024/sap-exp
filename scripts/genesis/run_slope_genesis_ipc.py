#!/usr/bin/env python3
"""GAUGE slope slider dynamic-friction-v3, Genesis + IPC (preliminary).  Shared logic in slope_v3_common.

Static board: fixed IPC ABD.  Kinematic board (task-3): two_way_soft_constraint free body whose Genesis pose/velocity
are written every step (as the RigidSolver/MuJoCo drivers), IPC following through the soft constraint; its ground pair
is disabled (the driven board's ground contact has no effect in the reference drivers).  Cube: IPC-only ABD, start
lifted g0 = d_hat in z.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import genesis_ipc_common as I
import slope_v3_common as sc
from run_slope_genesis_rigid import ASSETS, entities

TASK = "gauge-slope"


def sim_case(c, case, init_pos, n_frames, spf, dt):
    if True:
        xml = entities(case); R = len(init_pos); mu = case["mu"]; g0 = I.start_gap(c)
        kin = case["kinematic"]
        bodies = {"ground": I.Body(xml["ground"], mu=mu),
                  "board": I.Body(xml["board"], mu=mu, mass=case["mass_board_kg"] if kin else None,
                                  coup="two_way_soft_constraint" if kin else "ipc_only", ground_contact=not kin),
                  "cube": I.Body(xml["cube"], mu=mu, mass=case["mass_kg"])}
        # start-gap rule: cube surface g0 above the board's top face along the board normal (lift only if closer)
        Rf = I.quat_to_R(np.asarray(case["q_f"], float)); n = Rf[:, 2]
        top = I.obj_vertices(ASSETS / "board.obj")[:, 2].max()
        cv = I.obj_vertices(ASSETS / "cube.obj") @ I.quat_to_R(np.asarray(case["q_m"], float)).T
        p0 = np.asarray(init_pos, float).copy()
        clear = ((p0[:, None, :] + cv[None] - np.asarray(case["p_f"], float)) @ n).min(axis=1) - top
        lift = np.maximum(0.0, g0 - clear); p0 += lift[:, None] * n
        T0 = I.T_from(p0, np.tile(case["q_m"], (R, 1)))
        b = I.build(bodies, dt=dt, cand=c, n_envs=R, init={"cube": (T0, None)})
        board = b.ents["board"]
        pos = np.zeros((R, n_frames, 3)); total = (n_frames - 1) * spf
        track = []
        for step in range(total + 1):
            if kin:
                st = sc.board_state(case, step * dt)
                board.set_qpos(np.tile(np.r_[st["p"], st["q"]], (R, 1)).astype(np.float32))
                board.set_dofs_velocity(np.tile(np.r_[st["v"], st["w_local"]], (R, 1)).astype(np.float32))
                if step % spf == 0 and "board" in b.T:
                    track.append(float(np.abs(b.T["board"][:, :3, 3] - st["p"]).max()))
            if step % spf == 0:
                pos[:, step // spf] = b.pose("cube")[0]
                if (step // spf) % 20 == 0:
                    b.rigid_contacts()
            if step < total:
                b.step()
        prov = b.provenance(); b.destroy()
        out = {"compiled": prov, "pos": pos, "finite": np.isfinite(pos).all(axis=(1, 2)), "truncated": 0,
               "start_gap": {"clearance_before_mm": (clear * 1e3).tolist(), "lift_mm": (lift * 1e3).tolist()}}
        if kin:
            out["tracking"] = {"ipc_board_vs_target_max_abs_m": max(track) if track else None,
                               "method": "Genesis pose+velocity written every step; IPC board via soft transform constraint"}
        return out


def make_run_case(provenance: list):
    def run_case(c, case, init_pos, n_frames, spf, dt):
        R = len(init_pos)
        r, why = I.ISO.call("run_slope_genesis_ipc", "sim_case", tag=f"{c['id']}|{case.get('id', '')}", c=c, case=case,
                            init_pos=np.asarray(init_pos), n_frames=n_frames, spf=spf, dt=dt)
        if r is None:
            provenance.append({"failed": why})
            out = {"pos": np.zeros((R, n_frames, 3)), "finite": np.zeros(R, bool), "truncated": 0}
            if case["kinematic"]:
                out["tracking"] = {"failed": why}
            return out
        provenance.append({**r.pop("compiled"), "start_gap": r.pop("start_gap")})
        return r
    return run_case


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = I.registry_task(TASK, a.manifest)
    man0, _ = I.load_manifest(a.manifest)
    man = I.with_candidates(man0, spec)
    prov: list = []
    res = sc.execute(man, "genesis_ipc_candidates", I.parse_candidate, make_run_case(prov), I.ROOT, log=lambda s: print(s, flush=True))
    cells = [cell for s in res["selection_candidates"] for cell in s["cells"]] + res["heldout"]
    for r in cells:
        r["compiled"] = prov.pop(0)
    I.save_json(a.out / "genesis_ipc_gauge_slope_v3.json", {**I.header(man0, sha, spec), "complete": True, **res})


if __name__ == "__main__":
    main()
