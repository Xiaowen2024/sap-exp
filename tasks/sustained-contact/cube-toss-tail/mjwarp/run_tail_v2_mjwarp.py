#!/usr/bin/env python3
"""Cube-toss resting tail (sustained-contact-v2), MuJoCo Warp.  Shared logic in tail_v2_common."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
from pathlib import Path

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

import tail_v2_common as tc
from run_impact_v2_mjwarp import parse_candidate, ROOT
import impact_v2_common as ic


def xml(c, dt, mass_scale, mu, scale):
    attrs = "" if c["kind"] == "default" else f' solref="{-c["k"]} {-c["b"]}" solimp="0.9 0.9 0.001 0.5 2"'
    opt = "" if c["kind"] == "default" else f' integrator="{c["integrator"]}" solver="Newton"'
    h = tc.HALF * scale; I = 0.2950002913583124 * h ** 2 * mass_scale
    return f'''<mujoco model="cube_tail_v2"><option timestep="{dt}" gravity="0 0 -9.81"{opt}/>
 <worldbody><geom name="floor" type="plane" size="10 10 .1" friction="{mu} 0 0"{attrs}/>
 <body name="cube"><freejoint/><inertial pos="0 0 0" mass="{tc.MASS * mass_scale}" diaginertia="{I} {I} {I}"/>
 <geom name="cube_geom" type="box" size="{h} {h} {h}" friction="{mu} 0 0"{attrs}/></body></worldbody></mujoco>'''


def simulate(c, *, dt, mass_scale, mu, scale, init, n_samples, steps_per_sample):
    m = mujoco.MjModel.from_xml_string(xml(c, dt, mass_scale, mu, scale))
    gm = mjw.put_model(m); W = len(init); ncon = 16
    d = mjw.make_data(m, nworld=W, nconmax=ncon, njmax=96)
    q = np.zeros((W, 7)); q[:, :3] = init[:, :3]; q[:, 3:7] = init[:, 3:7]
    v = np.zeros((W, 6)); v[:, :3] = init[:, 7:10]; v[:, 3:6] = init[:, 10:13]
    d.qpos.assign(q.astype(np.float32)); d.qvel.assign(v.astype(np.float32)); mjw.forward(gm, d)
    buf = wp.zeros(ncon * W, dtype=wp.spatial_vectorf)
    P = np.zeros((W, n_samples, 3)); Q = np.zeros((W, n_samples, 4)); SP = np.zeros((W, n_samples)); OM = np.zeros((W, n_samples))
    FN = np.zeros((W, n_samples)); NC = np.zeros((W, n_samples)); maxn = 0
    for s in range(n_samples):
        qp, qv = d.qpos.numpy(), d.qvel.numpy()
        P[:, s] = qp[:, :3]; Q[:, s] = qp[:, 3:7]; SP[:, s] = np.linalg.norm(qv[:, :3], axis=1); OM[:, s] = np.linalg.norm(qv[:, 3:6], axis=1)
        n = int(d.nacon.numpy()[0]); maxn = max(maxn, n)
        if n:
            mjw.contact_force(gm, d, wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32), False, buf)
            f = buf.numpy()[:n, 0]; wid = d.contact.worldid.numpy()[:n]
            np.add.at(FN[:, s], wid, np.maximum(f, 0.0)); np.add.at(NC[:, s], wid, 1)
        if s < n_samples - 1:
            for _ in range(steps_per_sample):
                mjw.step(gm, d)
    finite = np.isfinite(P).all(axis=(1, 2)) & np.isfinite(Q).all(axis=(1, 2))
    Q = Q / np.maximum(np.linalg.norm(Q, axis=2, keepdims=True), 1e-12)
    return {"pos": P, "quat": Q, "speed": SP, "omega": OM, "fn": FN, "ncon": NC, "finite": finite, "truncated": int(maxn >= ncon * W)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cube-npz", type=Path, default=ROOT / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si_v2.npz")
    a = ap.parse_args()
    man, sha = tc.load_manifest(a.manifest)
    res = tc.execute(man, "mujoco_warp_candidates", parse_candidate, simulate, ic.cube_data(a.cube_npz), log=lambda s: print(s, flush=True))
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha, "engine": "MuJoCo Warp",
           "mujoco_version": mujoco.__version__, "warp_version": wp.config.version, "device": str(wp.get_device()), **res}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "mjwarp_cube_tail_v2.json").write_text(json.dumps(tc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
