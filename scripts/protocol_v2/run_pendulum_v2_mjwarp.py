#!/usr/bin/env python3
"""Pendulum energy-conservation-v2, MuJoCo Warp (shared logic in pendulum_v2_common)."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import mujoco, mujoco_warp as mjw, numpy as np, warp as wp
import pendulum_v2_common as pc


def parse_candidate(t):
    return {"id": "native_default", "integrator": None} if t.strip() == "native default" else {"id": t, "integrator": t.split("=")[1]}


def simulate(c, cell, th0):
    m, L, dt = cell["mass_kg"], cell["length_m"], cell["dt_ms"] * 1e-3
    integ = "" if c["integrator"] is None else f' integrator="{c["integrator"]}"'
    xml = f'''<mujoco model="pendulum_v2"><compiler angle="radian"/><option timestep="{dt}" gravity="0 0 -9.81"{integ}/>
 <worldbody><body name="bob"><joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
 <geom type="sphere" pos="0 0 -{L}" size="{pc.R_BOB}" mass="{m}" contype="0" conaffinity="0"/></body></worldbody></mujoco>'''
    mm = mujoco.MjModel.from_xml_string(xml); gm = mjw.put_model(mm); R = len(th0)
    d = mjw.make_data(mm, nworld=R, nconmax=1, njmax=1)
    q = d.qpos.numpy(); q[:, 0] = th0; d.qpos.assign(q); mjw.forward(gm, d)
    every = int(round(pc.SAMPLE_S / dt)); S = int(round(pc.DURATION_S / pc.SAMPLE_S)) + 1; Q, V = [], []
    for s in range(S):
        Q.append(d.qpos.numpy()[:, 0].astype(float)); V.append(d.qvel.numpy()[:, 0].astype(float))
        if s < S - 1:
            for _ in range(every):
                mjw.step(gm, d)
    return np.asarray(Q), np.asarray(V)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(); man, sha = pc.load_manifest(a.manifest)
    res = pc.execute(man, "mujoco_warp_candidates", parse_candidate, simulate, log=lambda s: print(s, flush=True))
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha, "engine": "MuJoCo Warp",
           "mujoco_version": mujoco.__version__, "warp_version": wp.config.version, "device": str(wp.get_device()), **res}
    a.out.mkdir(parents=True, exist_ok=True); (a.out / "mjwarp_pendulum_v2.json").write_text(json.dumps(pc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
