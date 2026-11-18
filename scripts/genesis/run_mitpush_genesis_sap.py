#!/usr/bin/env python3
"""MIT Push dynamic-friction-v3, Genesis SAP coupler (shared logic in mitpush_v3_common).

Object: released convex mesh (thickness axis y rotated onto z), explicit mass/inertia, free joint.  Pusher: cylinder on
x/y slide joints whose position/velocity are written every step from the recorded tip path (as MuJoCo Warp).
Genesis has no <pair>: friction is per geom with the max rule, so table 0.22, object 0.22, pusher 0.25 give exactly
table-object 0.22 and pusher-object 0.25.  The pusher's lowest point is 1.35 mm above the table (no table contact).
Wrench = contact force ON the pusher from the object (exact sign from Genesis' geom_b convention).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import genesis_sap_common as G
import mitpush_v3_common as mc

TASK = "mit-push"
ASSETS = G.ROOT / "exp-results/mit-push-v2/assets"


def entities(trial, cell):
    shape = trial["shape"]; ms = float(cell.get("mass_scale", 1.0)); I = tuple(x * ms for x in mc.INERTIA[shape])
    x0, y0 = trial["tip"][0]
    return {"table": G.mjcf(f'<geom name="table" type="plane" size="5 5 .1" friction="{mc.MU_TABLE} 0 0"/>'),
            "object": G.mjcf(f'<body name="object" pos="0 0 {mc.HALF_Z}"><freejoint/><inertial pos="0 0 0" mass="{mc.MASS * ms}" '
                             f'diaginertia="{I[0]} {I[1]} {I[2]}"/><geom name="obj_geom" type="mesh" mesh="obj" '
                             f'quat="0.70710678 0.70710678 0 0" friction="{mc.MU_TABLE} 0 0"/></body>',
                             meshdir=ASSETS, assets=f'<mesh name="obj" file="{shape}.obj"/>'),
            "pusher": G.mjcf(f'<body name="pusher" pos="{x0} {y0} {mc.TIP_Z}"><joint name="px" type="slide" axis="1 0 0"/>'
                             f'<joint name="py" type="slide" axis="0 1 0"/><geom name="tip" type="cylinder" '
                             f'size="{mc.TIP_R} {mc.TIP_HH}" mass="0.05" friction="{mc.MU_PUSH} 0 0"/></body>')}


def sim_chunk(c, trial, cell, init_xy, sps, dt, S, fps_):
    ents = entities(trial, cell); vec = G.sol_vector(c); R = len(init_xy)
    b = G.build(ents, dt=dt, cand=c, n_envs=R, sol={k: vec for k in ents})
    obj, pusher = b.ents["object"], b.ents["pusher"]
    q0 = G.np_(obj.get_qpos()).astype(np.float64); q0[:, 0:2] += init_xy
    obj.set_qpos(q0.astype(np.float32), zero_velocity=True)
    x0, y0 = trial["tip"][0]
    P = np.zeros((R, S, 3)); TW = np.zeros((R, S, 3)); W = np.zeros((R, S, 2)); D = np.zeros((R, S)); err = []

    def drive(t):
        x, y, vx, vy = mc.tip_target(trial, t)
        pusher.set_dofs_position(np.tile([x - x0, y - y0], (R, 1)).astype(np.float32), zero_velocity=False)
        pusher.set_dofs_velocity(np.tile([vx, vy], (R, 1)).astype(np.float32))
        return x - x0, y - y0

    step = 0
    for s in range(S):
        # Contacts must be read before the pusher state is written: Genesis clears contact data on a state write.
        # At s=0 no step has run yet, so the first sample has no contact force (MuJoCo's forward() supplies one).
        f, n = b.force_on("pusher", "object") if s > 0 else (np.zeros((R, 3)), np.zeros(R))
        W[:, s] = f[:, 0:2]; D[:, s] = n > 0
        tx, ty = drive(step * dt)
        pq = G.np_(pusher.get_dofs_position()); err.append(float(np.abs(pq - [tx, ty]).max()))
        q, v = G.np_(obj.get_qpos()), G.np_(obj.get_dofs_velocity())
        qw, qx, qy, qz = (q[:, 3 + k] for k in range(4))
        P[:, s, 0:2] = q[:, 0:2]; P[:, s, 2] = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        TW[:, s, 0:2] = v[:, 0:2]; TW[:, s, 2] = v[:, 5]          # body-frame wz == world wz for a planar object
        if s < S - 1:
            for _ in range(sps):
                drive(step * dt); b.scene.step(); step += 1
    P[:, :, 2] = np.unwrap(P[:, :, 2], axis=1)
    compiled=b.provenance(); b.destroy()
    return {"pose": P, "twist": TW, "wrench": W, "duty": D, "finite": np.isfinite(P).all(axis=(1, 2)), "truncated": 0, "compiled": compiled,
            "tracking_ok": max(err) <= 5e-4, "tracking": {"pusher_written_pose_max_abs_m": max(err),
                                                         "method": "slide-joint position and velocity written every step"}}


def make_run_trial(provenance: list):
    def run_trial(c,trial,cell,init_xy,sps,dt,S,fps_):
        parts=[];comp=[];failures=[];track=[]
        for k in range(0,len(init_xy),5):
            sl=slice(k,min(len(init_xy),k+5));n=sl.stop-sl.start
            r,why=G.ISO.call('run_mitpush_genesis_sap','sim_chunk',tag=f"{c['id']}|{cell['id']}|{sl.start}-{sl.stop}",c=c,trial=trial,cell=cell,init_xy=init_xy[sl],sps=sps,dt=dt,S=S,fps_=fps_)
            if r is None:
                r={'pose':np.zeros((n,S,3)),'twist':np.zeros((n,S,3)),'wrench':np.zeros((n,S,2)),'duty':np.zeros((n,S)),'finite':np.zeros(n,bool),'truncated':0,'tracking_ok':False}
                failures.append({'rollouts':[sl.start,sl.stop],'reason':why})
            else:comp.append(r['compiled']);track.append(r['tracking']['pusher_written_pose_max_abs_m'])
            parts.append(r)
        provenance.append({'chunks':comp,'chunk_size':5,'chunk_failures':failures})
        out={key:np.concatenate([p[key] for p in parts],axis=0) for key in ('pose','twist','wrench','duty','finite')}
        out.update(truncated=0,tracking_ok=all(p['tracking_ok'] for p in parts),tracking={'pusher_written_pose_max_abs_m':max(track) if track else float('inf'),'method':'slide-joint position and velocity written every step'})
        return out
    return run_trial


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = G.registry_task(TASK, a.manifest)
    man0, _ = G.load_manifest(a.manifest)
    man = G.with_candidates(man0, spec)
    prov: list = []
    res = mc.execute(man, "genesis_sap_candidates", lambda t: G.parse_candidate(t, spec), make_run_trial(prov), G.ROOT,
                     log=lambda s: print(s, flush=True))
    for r in res["selection_candidates"] + res["heldout"]:
        r["compiled"] = prov.pop(0)
    G.save_json(a.out / "genesis_sap_mit_push_v3.json", {**G.header(man0, sha, spec), "complete": True, **res})


if __name__ == "__main__":
    main()
