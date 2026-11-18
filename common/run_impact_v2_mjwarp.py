#!/usr/bin/env python3
"""dynamic-impact-v2 (GAUGE ball bounce / ContactNets cube toss), MuJoCo Warp, manifest-driven.

All trial selection, start states, perturbations, cell transforms and metrics live in impact_v2_common
(shared with the SAP driver).  This file only builds the MJCF and steps MuJoCo Warp.
MuJoCo free-joint qvel = [world linear velocity, body angular velocity]; the common init uses the same.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

import impact_v2_common as ic

ROOT = REPO_ROOT if (REPO_ROOT / "exp-results").exists() else Path("/root/SIM-EXP")
BALL = {"mass": 0.121, "mu": 0.808698, "mesh": ROOT / "exp-results/gauge-rigid-reproduction/assets/ball-6.obj"}
CUBE = {"mass": 0.37, "mu": 0.18, "half": ic.CUBE_HALF, "inertia": 0.2950002913583124 * ic.CUBE_HALF ** 2}


def parse_candidate(text: str) -> dict[str, Any]:
    if text.strip() == "native default":
        return {"id": "native_default", "kind": "default"}
    kv = dict(p.split("=") if "=" in p else (p, True) for p in text.replace(" ", "").split(","))
    return {"id": text, "kind": "direct", "k": float(kv["k"]), "b": float(kv["b"]),
            "integrator": "implicitfast" if "implicitfast" in kv else "Euler"}


def xml(task: str, c: dict[str, Any], dt: float, mass_scale: float) -> str:
    attrs = "" if c["kind"] == "default" else f' solref="{-c["k"]} {-c["b"]}" solimp="0.9 0.9 0.001 0.5 2"'
    opt = "" if c["kind"] == "default" else f' integrator="{c["integrator"]}" solver="Newton"'
    if task == "ball":
        return f'''<mujoco model="gauge_ball_v2"><compiler meshdir="{BALL["mesh"].parent}"/>
 <option timestep="{dt}" gravity="0 0 -9.81"{opt}/><asset><mesh name="ball" file="{BALL["mesh"].name}"/></asset>
 <worldbody><geom name="ground" type="plane" pos="0 0 {ic.BALL_GROUND}" size="5 5 .1" friction="{BALL["mu"]} 0 0"{attrs}/>
 <body name="ball"><freejoint/><geom type="mesh" mesh="ball" mass="{BALL["mass"] * mass_scale}" friction="{BALL["mu"]} 0 0"{attrs}/></body>
 </worldbody></mujoco>'''
    h, I = CUBE["half"], CUBE["inertia"] * mass_scale
    return f'''<mujoco model="contactnets_cube_v2"><option timestep="{dt}" gravity="0 0 -9.81"{opt}/>
 <worldbody><geom type="plane" size="5 5 .1" friction="{CUBE["mu"]} 0 0"{attrs}/>
 <body name="cube"><freejoint/><inertial pos="0 0 0" mass="{CUBE["mass"] * mass_scale}" diaginertia="{I} {I} {I}"/>
 <geom type="box" size="{h} {h} {h}" friction="{CUBE["mu"]} 0 0"{attrs}/></body></worldbody></mujoco>'''


def simulate(c, *, task, dt, mass_scale, init, n_samples, steps_per_sample):
    m = mujoco.MjModel.from_xml_string(xml(task, c, dt, mass_scale))
    gm = mjw.put_model(m)
    W = len(init)
    ncon = 16
    d = mjw.make_data(m, nworld=W, nconmax=ncon, njmax=96)
    q = np.zeros((W, 7)); q[:, :3] = init[:, :3]; q[:, 3:7] = init[:, 3:7]
    v = np.zeros((W, 6)); v[:, :3] = init[:, 7:10]; v[:, 3:6] = init[:, 10:13]
    d.qpos.assign(q.astype(np.float32)); d.qvel.assign(v.astype(np.float32)); mjw.forward(gm, d)
    pos = np.zeros((W, n_samples, 3)); quat = np.zeros((W, n_samples, 4)); max_nacon = 0
    for s in range(n_samples):
        qp = d.qpos.numpy(); pos[:, s] = qp[:, :3]; quat[:, s] = qp[:, 3:7]
        if s < n_samples - 1:
            for _ in range(steps_per_sample):
                mjw.step(gm, d)
            max_nacon = max(max_nacon, int(d.nacon.numpy()[0]))
    finite = np.isfinite(pos).all(axis=(1, 2)) & np.isfinite(quat).all(axis=(1, 2))
    quat = quat / np.maximum(np.linalg.norm(quat, axis=2, keepdims=True), 1e-12)
    return {"pos": pos, "quat": quat, "finite": finite, "truncated": int(max_nacon >= ncon * W),
            "max_active_contacts": max_nacon}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", choices=("ball", "cube"), required=True)
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cube-npz", type=Path, default=ROOT / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si_v2.npz")
    a = ap.parse_args()
    man, sha = ic.load_manifest(a.manifest)
    sym = None
    if a.task == "ball":
        data = ic.ball_trials(ROOT)
    else:
        data = ic.cube_data(a.cube_npz)
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from cube_orientation_metrics import cube_symmetric_qangle as sym
    res = ic.execute(a.task, man, "mujoco_warp_candidates", parse_candidate, simulate, data, sym,
                     log=lambda s: print(s, flush=True))
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
           "engine": "MuJoCo Warp", "mujoco_version": mujoco.__version__, "warp_version": wp.config.version,
           "device": str(wp.get_device()), **res}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / f"mjwarp_{a.task}_dynamic_impact_v2.json").write_text(json.dumps(ic.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
