#!/usr/bin/env python3
"""GAUGE slope slider dynamic-friction-v3, MuJoCo Warp (shared logic in slope_v3_common).

Kinematic board (task-3): a free body whose pose and velocity are overwritten every step from the mean recorded
board trajectory (as in the historical MuJoCo runner).  Cube position = body origin (mesh centred at origin).
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
import slope_v3_common as sc

ROOT = REPO_ROOT if (REPO_ROOT / "exp-results").exists() else Path("/root/SIM-EXP")


def parse_candidate(text: str) -> dict[str, Any]:
    if text.strip() == "native default":
        return {"id": "native_default", "native": True}
    c = {"id": text, "native": False, "cone": "pyramidal", "integrator": "Euler", "impratio": 1.0}
    for p in text.replace(" ", "").split(","):
        if p in ("elliptic", "pyramidal"):
            c["cone"] = p
        elif p in ("implicitfast", "Euler", "implicit"):
            c["integrator"] = p
        else:
            k, v = p.split("="); c[{"k": "k", "b": "b", "impratio": "impratio"}[k]] = float(v)
    return c


def xml(c, case, dt):
    mu = case["mu"]
    attrs = "" if c["native"] else f' solref="{-c["k"]} {-c["b"]}" solimp="0.9 0.95 0.001 0.5 2"'
    opt = "" if c["native"] else f' integrator="{c["integrator"]}" solver="Newton" cone="{c["cone"]}" impratio="{c["impratio"]}"'
    joint = '<freejoint name="board_joint"/>' if case["kinematic"] else ""
    j = lambda a: " ".join(map(str, a))
    s = f'''<mujoco model="gauge_slope_v3"><compiler meshdir="{ROOT / 'exp-results/gauge-rigid-reproduction/assets'}"/>
 <option timestep="{dt}" gravity="0 0 -9.81"{opt}/>
 <asset><mesh name="cube_mesh" file="cube.obj"/><mesh name="board_mesh" file="board.obj"/></asset>
 <worldbody><geom name="ground" type="plane" size="5 5 .1" friction="{mu} 0 0"{attrs}/>
  <body name="board" pos="{j(case['p_f'])}" quat="{j(case['q_f'])}">{joint}<geom name="board_geom" type="mesh" mesh="board_mesh" mass="{case['mass_board_kg']}" friction="{mu} 0 0"{attrs}/></body>
  <body name="cube" pos="{j(case['p_m'])}" quat="{j(case['q_m'])}"><freejoint name="cube_free"/><geom name="cube_geom" type="mesh" mesh="cube_mesh" mass="{case['mass_kg']}" friction="{mu} 0 0"{attrs}/></body>
 </worldbody></mujoco>'''
    return native_default_mjcf(s) if c["native"] else s


def run_case(c, case, init_pos, n_frames, spf, dt):
    m = mujoco.MjModel.from_xml_string(xml(c, case, dt))
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cube"); cq = int(m.jnt_qposadr[m.body_jntadr[cid]])
    R = len(init_pos)
    q0 = np.tile(m.qpos0, (R, 1)); q0[:, cq:cq + 3] = init_pos
    gm = mjw.put_model(m); d = mjw.make_data(m, nworld=R, nconmax=64, njmax=256)
    d.qpos.assign(q0.astype(np.float32)); mjw.forward(gm, d)
    pos = np.zeros((R, n_frames, 3)); maxn = 0
    total = (n_frames - 1) * spf
    for step in range(total + 1):
        if case["kinematic"]:
            b = sc.board_state(case, step * dt)
            qp, qv = d.qpos.numpy(), d.qvel.numpy()
            qp[:, 0:3] = b["p"]; qp[:, 3:7] = b["q"]; qv[:, 0:3] = b["v"]; qv[:, 3:6] = b["w_local"]
            d.qpos.assign(qp); d.qvel.assign(qv); mjw.forward(gm, d)
        if step % spf == 0:
            pos[:, step // spf] = d.qpos.numpy()[:, cq:cq + 3]
        if step < total:
            mjw.step(gm, d); maxn = max(maxn, int(d.nacon.numpy()[0]))
    return {"pos": pos, "finite": np.isfinite(pos).all(axis=(1, 2)), "truncated": int(maxn >= 64 * R)}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    man, sha = sc.load_manifest(a.manifest)
    res = sc.execute(man, "mujoco_warp_candidates", parse_candidate, run_case, ROOT, log=lambda s: print(s, flush=True))
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha, "engine": "MuJoCo Warp",
           "mujoco_version": mujoco.__version__, "warp_version": wp.config.version, "device": str(wp.get_device()), **res}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "mjwarp_gauge_slope_v3.json").write_text(json.dumps(sc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
