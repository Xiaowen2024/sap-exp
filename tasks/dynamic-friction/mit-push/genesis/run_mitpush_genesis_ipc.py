#!/usr/bin/env python3
"""MIT Push dynamic-friction-v3, Genesis + IPC (preliminary).  Shared logic in mitpush_v3_common.

Table: IPC ground.  Object: IPC-only ABD of the released mesh, start lifted g0 = d_hat (ABD inertia is uniform-density;
the manifest's explicit inertia cannot be imposed, ratio recorded).  Pusher: Genesis x/y slide joints written every
step, IPC following with two_way_soft_constraint; pusher-table pair disabled (the manifest pusher clears the table).
Pair friction by geometric mean: table/object 0.22, pusher coup_friction 0.25^2/0.22 -> pusher-object 0.25.
Wrench (a3) = IPC's own contact force on the pusher vertices (force ON the pusher; the pusher-table pair is off, so all of
it comes from the object); the bridge coupling force is kept as a diagnostic.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import genesis_ipc_common as I
import mitpush_v3_common as mc
from run_mitpush_genesis_rigid import ASSETS, entities

TASK = "mit-push"


def sim_trial(c, trial, cell, init_xy, sps, dt, S, fps_):
    if True:
        xml = entities(trial, cell); R = len(init_xy); g0 = I.start_gap(c)
        ms = float(cell.get("mass_scale", 1.0))
        bodies = {"table": I.Body(xml["table"], mu=mc.MU_TABLE),
                  "object": I.Body(xml["object"], mu=mc.MU_TABLE, mass=mc.MASS * ms),
                  "pusher": I.Body(xml["pusher"], mu=mc.MU_PUSH ** 2 / mc.MU_TABLE, mass=0.05,
                                   coup="two_way_soft_constraint", ground_contact=False)}
        # start-gap rule: object lifted g0 off the table and, if the recorded tip start is closer than g0 to (or inside)
        # the object outline, the object is moved away from the tip in the plane until the clearance is g0
        ov = I.obj_vertices(ASSETS / f"{trial['shape']}.obj") @ I.quat_to_R(np.array([0.70710678, 0.70710678, 0, 0])).T
        poly = I.hull2d(ov[:, 0:2]); tip = np.asarray(trial["tip"][0], float)
        pos = np.zeros((R, 3)); pos[:, 0:2] = init_xy; pos[:, 2] = mc.HALF_Z + g0
        shift = np.zeros(R)
        for r in range(R):
            d, u = I.point_polygon(tip, poly + pos[r, 0:2])
            need = g0 - (d - mc.TIP_R)
            if need > 0:
                pos[r, 0:2] += need * u; shift[r] = need
        b = I.build(bodies, dt=dt, cand=c, n_envs=R, init={"object": (I.T_from(pos, np.tile([1.0, 0, 0, 0], (R, 1))), None)})
        pusher = b.ents["pusher"]
        x0, y0 = trial["tip"][0]
        P = np.zeros((R, S, 3)); TW = np.zeros((R, S, 3)); W = np.zeros((R, S, 2)); D = np.zeros((R, S)); err = []
        WB = np.zeros((R, S, 2))

        def drive(t):
            x, y, vx, vy = mc.tip_target(trial, t)
            pusher.set_dofs_position(np.tile([x - x0, y - y0], (R, 1)).astype(np.float32), zero_velocity=False)
            pusher.set_dofs_velocity(np.tile([vx, vy], (R, 1)).astype(np.float32))
            return x, y

        step = 0
        for s in range(S):
            if s > 0:      # a3: direct IPC barrier+friction force on the pusher (its table pair is off: all from the object)
                d = b.direct_force("pusher"); f = d["total"]; fb = b.coupling_force("pusher")
                W[:, s] = f[:, 0:2]; D[:, s] = d["nv_active"] > 0; WB[:, s] = fb[:, 0:2]
            tx, ty = drive(step * dt)
            if "pusher" in b.T and s > 0:
                err.append(float(np.abs(b.T["pusher"][:, 0:2, 3] - [tx, ty]).max()))
            p, q = b.pose("object"); v, w, _ = b.vel("object")
            qw, qx, qy, qz = (q[:, k] for k in range(4))
            P[:, s, 0:2] = p[:, 0:2]; P[:, s, 2] = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            TW[:, s, 0:2] = v[:, 0:2]; TW[:, s, 2] = w[:, 2]
            if s % 20 == 0:
                b.rigid_contacts()
            if s < S - 1:
                for _ in range(sps):
                    drive(step * dt); b.step(); step += 1
        P[:, :, 2] = np.unwrap(P[:, :, 2], axis=1)
        prov = b.provenance()
        prov["note_inertia"] = "ABD uses uniform-density inertia of the mesh; manifest inertia " + str(mc.INERTIA[trial["shape"]])
        prov["start_object_shift_mm"] = (shift * 1e3).tolist()
        with np.errstate(all="ignore"):
            on = D > 0
            prov["wrench_readout_check"] = {
                "readout": "direct: IPC contact force on the pusher vertices (ContactSystemFeature); bridge spring kept as diagnostic",
                "direct_minus_bridge_rms_N": float(np.sqrt(np.mean(np.sum((W - WB) ** 2, axis=2)[on]))) if on.any() else None,
                "direct_rms_N": float(np.sqrt(np.mean(np.sum(W ** 2, axis=2)[on]))) if on.any() else None}
        b.destroy()
        e = max(err) if err else 0.0
        return {"compiled": prov, "pose": P, "twist": TW, "wrench": W, "duty": D, "finite": np.isfinite(P).all(axis=(1, 2)), "truncated": 0,
                "tracking_ok": True, "tracking": {"ipc_pusher_vs_target_max_abs_m": e,
                                                   "method": "Genesis slide joints written every step; IPC pusher via soft transform constraint"}}


def make_run_trial(provenance: list):
    def run_trial(c, trial, cell, init_xy, sps, dt, S, fps_):
        R = len(init_xy)
        r, why = I.ISO.call("run_mitpush_genesis_ipc", "sim_trial", tag=f"{c['id']}|{trial.get('file', '')}", c=c, trial=trial,
                            cell=cell, init_xy=np.asarray(init_xy), sps=sps, dt=dt, S=S, fps_=fps_)
        if r is None:
            provenance.append({"failed": why})
            return {"pose": np.zeros((R, S, 3)), "twist": np.zeros((R, S, 3)), "wrench": np.zeros((R, S, 2)),
                    "duty": np.zeros((R, S)), "finite": np.zeros(R, bool), "truncated": 0, "tracking_ok": False,
                    "tracking": {"failed": why}}
        provenance.append(r.pop("compiled"))
        return r
    return run_trial


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = I.registry_task(TASK, a.manifest)
    man0, _ = I.load_manifest(a.manifest)
    man = I.with_candidates(man0, spec)
    prov: list = []
    res = mc.execute(man, "genesis_ipc_candidates", I.parse_candidate, make_run_trial(prov), I.ROOT, log=lambda s: print(s, flush=True))
    for r in res["selection_candidates"] + res["heldout"]:
        r["compiled"] = prov.pop(0)
    I.save_json(a.out / "genesis_ipc_mit_push_v3.json", {**I.header(man0, sha, spec), "complete": True, **res})


if __name__ == "__main__":
    main()
