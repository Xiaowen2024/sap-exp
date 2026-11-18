#!/usr/bin/env python3
"""MIT Push dynamic-friction-v3, Genesis RigidSolver (shared logic in mitpush_v3_common).

Object: released convex mesh (thickness axis y rotated onto z), explicit mass/inertia, free joint.  Pusher: cylinder on
x/y slide joints whose position/velocity are written every step from the recorded tip path (as MuJoCo Warp).
Genesis has no <pair>: friction is per geom with the max rule, so table 0.22, object 0.22, pusher 0.25 give exactly
table-object 0.22 and pusher-object 0.25.  The pusher's lowest point is 1.35 mm above the table (no table contact).
Wrench = contact force ON the pusher from the object (exact sign from Genesis' geom_b convention).
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
from pathlib import Path

import numpy as np

import genesis_rigid_common as G
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


def make_run_trial(provenance: list):
    def run_trial(c, trial, cell, init_xy, sps, dt, S, fps_):
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
        provenance.append(b.provenance()); b.destroy()
        return {"pose": P, "twist": TW, "wrench": W, "duty": D, "finite": np.isfinite(P).all(axis=(1, 2)), "truncated": 0,
                "tracking_ok": max(err) <= 5e-4, "tracking": {"pusher_written_pose_max_abs_m": max(err),
                                                             "method": "slide-joint position and velocity written every step"}}
    return run_trial


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = G.registry_task(TASK, a.manifest)
    man0, _ = G.load_manifest(a.manifest)
    man = G.with_candidates(man0, spec)
    prov: list = []
    res = mc.execute(man, "genesis_rigid_candidates", lambda t: G.parse_candidate(t, spec), make_run_trial(prov), G.ROOT,
                     log=lambda s: print(s, flush=True))
    for r in res["selection_candidates"] + res["heldout"]:
        r["compiled"] = prov.pop(0)
    G.save_json(a.out / "genesis_rigid_mit_push_v3.json", {**G.header(man0, sha, spec), "complete": True, **res})


if __name__ == "__main__":
    main()
