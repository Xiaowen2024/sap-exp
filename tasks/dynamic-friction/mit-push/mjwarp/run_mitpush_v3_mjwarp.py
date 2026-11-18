#!/usr/bin/env python3
"""MIT Push dynamic-friction-v3, MuJoCo Warp (shared logic in mitpush_v3_common).

Object: released convex mesh (thickness axis y rotated onto z), explicit mass/inertia, freejoint.  Pusher: cylinder on
x/y slide joints whose qpos/qvel are overwritten every step from the recorded tip path.  Contacts only through explicit
<pair>s: table-object (mu 0.22) and pusher-object (mu 0.25).
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import math
from pathlib import Path

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from native_defaults import native_default_mjcf
from run_slope_v3_mjwarp import parse_candidate, ROOT
import mitpush_v3_common as mc


def xml(c, trial, cell, dt):
    shape = trial["shape"]; I = mc.INERTIA[shape]; m = mc.MASS * float(cell.get("mass_scale", 1.0))
    I = tuple(x * float(cell.get("mass_scale", 1.0)) for x in I)
    sol = "" if c["native"] else f' solref="{-c["k"]} {-c["b"]}" solimp="0.9 0.95 0.001 0.5 2"'
    opt = "" if c["native"] else f' integrator="{c["integrator"]}" solver="Newton" cone="{c["cone"]}" impratio="{c["impratio"]}"'
    x0, y0 = trial["tip"][0]
    s = f'''<mujoco model="mit_push_v3"><compiler meshdir="{ROOT / 'exp-results/mit-push-v2/assets'}"/>
 <option timestep="{dt}" gravity="0 0 -9.81"{opt}/><asset><mesh name="obj" file="{shape}.obj"/></asset>
 <worldbody><geom name="table" type="plane" size="5 5 .1" contype="0" conaffinity="0"/>
  <body name="object" pos="0 0 {mc.HALF_Z}"><freejoint name="obj_free"/><inertial pos="0 0 0" mass="{m}" diaginertia="{I[0]} {I[1]} {I[2]}"/>
   <geom name="obj_geom" type="mesh" mesh="obj" quat="0.70710678 0.70710678 0 0" contype="0" conaffinity="0"/></body>
  <body name="pusher" pos="{x0} {y0} {mc.TIP_Z}"><joint name="px" type="slide" axis="1 0 0"/><joint name="py" type="slide" axis="0 1 0"/>
   <geom name="tip" type="cylinder" size="{mc.TIP_R} {mc.TIP_HH}" mass="0.05" contype="0" conaffinity="0"/></body>
 </worldbody>
 <contact><pair geom1="table" geom2="obj_geom" condim="3" friction="{mc.MU_TABLE} {mc.MU_TABLE} 0.005 0.0001 0.0001"{sol}/>
  <pair geom1="tip" geom2="obj_geom" condim="3" friction="{mc.MU_PUSH} {mc.MU_PUSH} 0.005 0.0001 0.0001"{sol}/></contact></mujoco>'''
    return native_default_mjcf(s) if c["native"] else s


def run_trial(c, trial, cell, init_xy, sps, dt, S, fps_):
    m = mujoco.MjModel.from_xml_string(xml(c, trial, cell, dt)); gm = mjw.put_model(m); R = len(init_xy)
    oq = int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "obj_free")]); ov = int(m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "obj_free")])
    pq = [int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in ("px", "py")]
    pv = [int(m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in ("px", "py")]
    tip_g, obj_g = (mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ("tip", "obj_geom"))
    q0 = np.tile(m.qpos0, (R, 1)); q0[:, oq:oq + 2] += init_xy
    d = mjw.make_data(m, nworld=R, nconmax=64, njmax=256); d.qpos.assign(q0.astype(np.float32)); mjw.forward(gm, d)
    buf = wp.zeros(64 * R, dtype=wp.spatial_vectorf)
    x0, y0 = trial["tip"][0]
    P = np.zeros((R, S, 3)); TW = np.zeros((R, S, 3)); W = np.zeros((R, S, 2)); D = np.zeros((R, S)); maxn = 0

    def drive(t):
        x, y, vx, vy = mc.tip_target(trial, t)
        qp, qv = d.qpos.numpy(), d.qvel.numpy()
        qp[:, pq[0]] = x - x0; qp[:, pq[1]] = y - y0; qv[:, pv[0]] = vx; qv[:, pv[1]] = vy
        d.qpos.assign(qp); d.qvel.assign(qv); mjw.forward(gm, d)

    step = 0
    for s in range(S):
        drive(step * dt)
        qp, qv = d.qpos.numpy(), d.qvel.numpy()
        qw, qx, qy, qz = (qp[:, oq + 3 + k] for k in range(4))
        P[:, s, 0:2] = qp[:, oq:oq + 2]; P[:, s, 2] = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        TW[:, s, 0:2] = qv[:, ov:ov + 2]; TW[:, s, 2] = qv[:, ov + 5]  # body-frame wz == world wz for a planar object
        n = int(d.nacon.numpy()[0])
        if n:
            mjw.contact_force(gm, d, wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32), True, buf)
            f = buf.numpy()[:n]; wid = d.contact.worldid.numpy()[:n]; geom = d.contact.geom.numpy()[:n]
            for i in range(n):
                if {int(geom[i, 0]), int(geom[i, 1])} == {tip_g, obj_g}:
                    w = wid[i]; F = f[i, :2].astype(float)
                    rel = np.array([qp[w, pq[0]] + x0 - qp[w, oq], qp[w, pq[1]] + y0 - qp[w, oq + 1]])
                    W[w, s] += F if np.dot(F, rel) >= 0 else -F; D[w, s] = 1
        if s < S - 1:
            for _ in range(sps):
                drive(step * dt); mjw.step(gm, d); step += 1; maxn = max(maxn, int(d.nacon.numpy()[0]))
    P[:, :, 2] = np.unwrap(P[:, :, 2], axis=1)
    return {"pose": P, "twist": TW, "wrench": W, "duty": D, "finite": np.isfinite(P).all(axis=(1, 2)), "truncated": int(maxn >= 64 * R)}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(); man, sha = mc.load_manifest(a.manifest)
    res = mc.execute(man, "mujoco_warp_candidates", parse_candidate, run_trial, ROOT, log=lambda s: print(s, flush=True))
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha, "engine": "MuJoCo Warp",
           "mujoco_version": mujoco.__version__, "warp_version": wp.config.version, "device": str(wp.get_device()), **res}
    a.out.mkdir(parents=True, exist_ok=True); (a.out / "mjwarp_mit_push_v3.json").write_text(json.dumps(mc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
