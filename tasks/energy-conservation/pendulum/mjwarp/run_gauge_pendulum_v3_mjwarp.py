#!/usr/bin/env python3
"""GAUGE pendulum strict-v3, MuJoCo Warp (shared logic in gauge_pendulum_v3_common).

Pivot body at the GAUGE anchor with an undamped hinge about world y; ball body L below with explicit mass / solid-sphere
inertia (non-colliding sphere geom).  Candidates: integrator only (contact-free task).
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import argparse, json
from pathlib import Path
import mujoco, mujoco_warp as mjw, numpy as np, warp as wp
import gauge_pendulum_v3_common as gp
ROOT = REPO_ROOT if (REPO_ROOT / "exp-results").exists() else Path("/root/SIM-EXP")


def parse_candidate(t):
    return {"id": "native_default", "integrator": None} if t.strip() == "native default" else {"id": t, "integrator": t.split("=")[1]}


def simulate(c, cell, th0):
    dt, ms = cell["dt_ms"] * 1e-3, float(cell.get("mass_scale", 1.0)); m = gp.MASS * ms; I = gp.ball_inertia(ms); a = gp.ANCHOR
    integ = "" if c["integrator"] is None else f' integrator="{c["integrator"]}"'
    s = (f'<mujoco model="gauge_pendulum_v3"><option timestep="{dt}" gravity="0 0 -9.81"{integ}/><worldbody>'
         f'<body name="pivot" pos="{a[0]} {a[1]} {a[2]}"><joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>'
         f'<body name="ball" pos="0 0 -{gp.L}"><inertial pos="0 0 0" mass="{m}" diaginertia="{I} {I} {I}"/>'
         f'<geom type="sphere" size="{gp.RAD}" contype="0" conaffinity="0"/></body></body></worldbody></mujoco>')
    mm = mujoco.MjModel.from_xml_string(s); gm = mjw.put_model(mm); W = len(th0)
    d = mjw.make_data(mm, nworld=W, nconmax=1, njmax=1)
    q = np.zeros((W, 1)); q[:, 0] = th0; d.qpos.assign(q.astype(np.float32)); mjw.forward(gm, d)
    sps = int(round((1.0 / gp.FPS) / dt)); S = cell["frames"]; Q = []
    for k in range(S):
        Q.append(d.qpos.numpy()[:, 0].astype(float).copy())
        if k < S - 1:
            for _ in range(sps):
                mjw.step(gm, d)
    Q = np.asarray(Q)
    return Q, np.isfinite(Q).all(axis=0)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(); man, sha = gp.load_manifest(a.manifest)
    res = gp.execute(man, "mujoco_warp_candidates", parse_candidate, simulate, ROOT, log=lambda s: print(s, flush=True))
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha, "engine": "MuJoCo Warp",
           "mujoco_version": mujoco.__version__, "warp_version": wp.config.version, "device": str(wp.get_device()), **res}
    a.out.mkdir(parents=True, exist_ok=True); (a.out / "mjwarp_gauge_pendulum_v3.json").write_text(json.dumps(gp.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
