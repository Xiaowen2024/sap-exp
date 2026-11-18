#!/usr/bin/env python3
"""simple-pinch force-controlled-v3, MuJoCo Warp (manifest-driven, shared helpers in pinch_v3_common).

Pads: x slide joints get a constant inward force F + damping (no x stiffness); z slide joints track the lift by PD.
Gravity is off; the cube load m g (x scale) is an external -z force during lift/hold.  Normal loads are read with
mujoco_warp.contact_force (elliptic cone).
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from native_defaults import native_default_mjcf
import pinch_v3_common as pc


def parse_candidate(text: str) -> dict[str, Any]:
    if text.strip() == "native default":
        return {"id": "native_default", "native": True, "tc": 0.02, "impratio": 1.0, "noslip": 0}
    kv = dict(p.split("=") for p in text.replace(" ", "").split(","))
    return {"id": text, "tc": float(kv["tc"].removesuffix("ms")) * 1e-3, "impratio": float(kv["impratio"]), "noslip": int(kv["noslip"])}


def xml(c: dict[str, Any], cell: dict[str, Any]) -> str:
    mu, dt, hy = cell["mu"], cell["dt_ms"] * 1e-3, cell["pad_halfwidth_y_mm"] * 1e-3
    ns = f' noslip_iterations="{c["noslip"]}"' if c["noslip"] > 0 else ""
    s = f'''<mujoco model="simple_pinch_v3"><option timestep="{dt}" gravity="0 0 0" integrator="implicitfast" solver="Newton"
 cone="elliptic" impratio="{c["impratio"]}" iterations="100" tolerance="1e-12"{ns}/>
 <worldbody>
  <body name="left" pos="-0.03 0 0"><joint name="left_x" type="slide" axis="1 0 0"/><joint name="left_z" type="slide" axis="0 0 1"/>
   <geom name="left_pad" type="box" size="0.01 {hy} 0.04" mass="1" friction="{mu} 0.005 0.0001"/></body>
  <body name="right" pos="0.03 0 0"><joint name="right_x" type="slide" axis="1 0 0"/><joint name="right_z" type="slide" axis="0 0 1"/>
   <geom name="right_pad" type="box" size="0.01 {hy} 0.04" mass="1" friction="{mu} 0.005 0.0001"/></body>
  <body name="cube" pos="0 0 0"><freejoint name="cube_free"/><geom name="cube_geom" type="box" size="0.02 0.02 0.02" mass="{cell["mass_kg"]}" friction="{mu} 0.005 0.0001"/></body>
 </worldbody>
 <contact>
  <pair geom1="left_pad" geom2="cube_geom" condim="3" margin="0" gap="0" friction="{mu} {mu} 0.005 0.0001 0.0001" solref="{c["tc"]} 1" solimp="0.95 0.99 0.001 0.5 2"/>
  <pair geom1="right_pad" geom2="cube_geom" condim="3" margin="0" gap="0" friction="{mu} {mu} 0.005 0.0001 0.0001" solref="{c["tc"]} 1" solimp="0.95 0.99 0.001 0.5 2"/>
 </contact></mujoco>'''
    return native_default_mjcf(s) if c.get("native") else s


def run_cell(c: dict[str, Any], cell: dict[str, Any], seed: int, R: int) -> dict[str, Any]:
    m = mujoco.MjModel.from_xml_string(xml(c, cell))
    gid = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ("left_pad", "right_pad", "cube_geom")}
    cube_b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cq = int(m.jnt_qposadr[m.body_jntadr[cube_b]])
    jq = {n: int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in ("left_x", "left_z", "right_x", "right_z")}
    jv = {n: int(m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in jq}
    p = pc.perturbations(seed, R)
    q0 = np.tile(m.qpos0, (R, 1))
    q0[:, cq + 1:cq + 3] += p["yz"]; q0[:, cq + 3] = np.cos(p["pitch"] / 2); q0[:, cq + 4] = 0; q0[:, cq + 5] = np.sin(p["pitch"] / 2); q0[:, cq + 6] = 0
    gm = mjw.put_model(m); d = mjw.make_data(m, nworld=R, nconmax=32, njmax=128)
    d.qpos.assign(q0.astype(np.float32)); mjw.forward(gm, d)
    buf = wp.zeros(32 * R, dtype=wp.spatial_vectorf)
    dt = cell["dt_ms"] * 1e-3; F = cell["per_pad_force_N"]; load = cell["mass_kg"] * pc.G * p["load_scale"]
    every = max(1, round(pc.SAMPLE_S / dt)); max_nacon = 0
    qf = np.zeros((R, m.nv), np.float32); xf = np.zeros((R, m.nbody, 6), np.float32)

    def control(z, vz, with_load):
        qp, qv = d.qpos.numpy(), d.qvel.numpy()
        qf[:] = 0
        qf[:, jv["left_x"]] = F - pc.PAD_KD * qv[:, jv["left_x"]]
        qf[:, jv["right_x"]] = -F - pc.PAD_KD * qv[:, jv["right_x"]]
        for n in ("left_z", "right_z"):
            qf[:, jv[n]] = pc.PAD_Z_KE * (z - qp[:, jq[n]]) + pc.PAD_Z_KD * (vz - qv[:, jv[n]])
        xf[:] = 0
        if with_load:
            xf[:, cube_b, 2] = -load
        d.qfrc_applied.assign(qf); d.xfrc_applied.assign(xf)

    def pad_forces():
        n = int(d.nacon.numpy()[0]); fn = np.zeros((R, 2)); cnt = np.zeros((R, 2), bool)
        if n:
            mjw.contact_force(gm, d, wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32), False, buf)
            f = buf.numpy()[:n]; wid = d.contact.worldid.numpy()[:n]; geom = d.contact.geom.numpy()[:n]
            for i in range(n):
                pair = {int(geom[i, 0]), int(geom[i, 1])}
                if gid["cube_geom"] in pair:
                    side = 0 if gid["left_pad"] in pair else (1 if gid["right_pad"] in pair else None)
                    if side is not None:
                        fn[wid[i], side] += max(0.0, float(f[i, 0])); cnt[wid[i], side] = True
        return fn, cnt

    gate_steps = round(pc.GATE_S / dt); GF, GD = [], []
    for s in range(gate_steps):
        control(0.0, 0.0, False); mjw.step(gm, d); max_nacon = max(max_nacon, int(d.nacon.numpy()[0]))
        if s >= gate_steps - round(0.2 / dt) and s % every == 0:
            fn, cnt = pad_forces(); GF.append(fn); GD.append(cnt)
    qp = d.qpos.numpy(); z0 = qp[:, cq + 2].copy(); pz0 = 0.5 * (qp[:, jq["left_z"]] + qp[:, jq["right_z"]])
    steps = round((pc.LIFT_S + pc.HOLD_S) / dt); T, RISE, REL = [], [], []
    for s in range(steps + 1):
        t = s * dt; z, vz = pc.lift_target(t)
        if s % every == 0 or s == steps:
            qp = d.qpos.numpy(); cz = qp[:, cq + 2] - z0; pz = 0.5 * (qp[:, jq["left_z"]] + qp[:, jq["right_z"]]) - pz0
            T.append(t); RISE.append(cz); REL.append(cz - pz)
        if s < steps:
            control(z, vz, True); mjw.step(gm, d); max_nacon = max(max_nacon, int(d.nacon.numpy()[0]))
    RISE = np.asarray(RISE, float)
    res = pc.summarize(cell, gate_fn=np.asarray(GF), gate_duty=np.asarray(GD), t=np.asarray(T), cube_rise=RISE,
                       rel_z=np.asarray(REL, float), finite=np.isfinite(RISE).all(axis=0), truncated=int(max_nacon >= 32 * R))
    res["compiled"] = {"opt_cone": int(m.opt.cone), "opt_impratio": float(m.opt.impratio), "pair_solref": m.pair_solref.tolist(),
                       "opt_noslip_iterations": int(m.opt.noslip_iterations), "timestep": float(m.opt.timestep)}
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    man, sha = pc.load_manifest(a.manifest)
    res = pc.execute(man, "mujoco_warp_candidates", parse_candidate, run_cell, log=lambda s: print(s, flush=True))
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
           "engine": "MuJoCo Warp", "mujoco_version": mujoco.__version__, "warp_version": wp.config.version,
           "device": str(wp.get_device()), **res}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "mjwarp_simple_pinch_v3.json").write_text(json.dumps(pc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
