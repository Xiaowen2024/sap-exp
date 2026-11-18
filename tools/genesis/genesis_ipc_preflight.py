#!/usr/bin/env python3
"""Genesis + IPC preflight: input routing, telemetry, friction pairing, contact ownership, rest gap.  Not task results.

Every check uses the same adapter (genesis_ipc_common) the task runners use.
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

import numpy as np

import genesis_ipc_common as I

M_BOX, H = 0.064, 0.02          # 4 cm box


def box_xml(name, z, mu, extra=""):
    return I.mjcf(f'<body name="{name}" pos="0 0 {z}"><freejoint/><geom type="box" size="{H} {H} {H}" mass="{M_BOX}" friction="{mu} 0 0"/></body>{extra}')


def plane_xml(mu):
    return I.mjcf(f'<geom name="ground" type="plane" size="5 5 .1" friction="{mu} 0 0"/>')


def check(name, ok, **kw):
    print(("PASS " if ok else "FAIL ") + name, json.dumps(I.json_safe(kw)), flush=True)
    return {"name": name, "pass": bool(ok), **kw}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args()
    I.gs_init()
    res = []
    dt, B = 0.002, 2
    nat = I.parse_candidate("native default")
    # 1 free-space force (animator route) and mass from rho*V
    b = I.build({"box": I.Body(box_xml("box", 0.5, 0.3), mu=0.3, mass=M_BOX, force=True)}, dt=dt, gravity=(0, 0, 0), cand=nat, n_envs=B)
    b.forces["box"][:, 0] = M_BOX * 1.0
    for _ in range(100):
        b.step()
    x = b.pose("box")[0][:, 0]; v = b.vel("box")[0][:, 0]
    res.append(check("force_route_F_over_m", np.allclose(v, 0.2, rtol=0.02), vx=v.tolist(), expected=0.2, x=x.tolist(),
                     ipc_mass=b.info["bodies"]["box"]["ipc_mass_kg"]))
    res.append(check("contact_force_estimate_zero_in_free_space", np.abs(b.contact_force("box")).max() < 1e-3 * M_BOX * 9.81,
                     f=b.contact_force("box").tolist()))
    b.destroy()
    # 2 initial linear + angular velocity (body-frame angular, MuJoCo qvel convention), tilted start pose
    q0 = np.array([math.cos(0.3), math.sin(0.3), 0, 0])        # 0.6 rad about x
    T = I.T_from(np.tile([0, 0, .5], (B, 1)), np.tile(q0, (B, 1)))
    wb = np.array([0.0, 0.0, 2.0])                              # body z
    V = I.V_from(T, np.tile([0.1, 0, 0], (B, 1)), np.tile(wb, (B, 1)))
    b = I.build({"box": I.Body(box_xml("box", 0.5, 0.3), mu=0.3, mass=M_BOX)}, dt=dt, gravity=(0, 0, 0), cand=nat, n_envs=B,
                init={"box": (T, V)})
    lin0, w0, wb0 = b.vel("box")
    for _ in range(50):
        b.step()
    lin, w, wbody = b.vel("box"); p, q = b.pose("box")
    res.append(check("initial_velocity_route", np.allclose(p[:, 0], 0.01, atol=2e-4) and np.allclose(wbody, wb, atol=0.02),
                     x=p[:, 0].tolist(), w_body=wbody.tolist(), w_body0=wb0.tolist()))
    b.destroy()
    # 3 rest on ground: gap vs d_hat, load = m g, no RigidSolver contact
    for cand in (nat, I.parse_candidate("d_hat=1mm")):
        g0 = I.start_gap(cand)
        b = I.build({"ground": I.Body(plane_xml(0.3), mu=0.3), "box": I.Body(box_xml("box", H + g0, 0.3), mu=0.3, mass=M_BOX)},
                    dt=dt, cand=cand, n_envs=B)
        fz = []
        for s in range(500):
            b.step()
            if s >= 400:
                fz.append(b.contact_force("box")[:, 2]); b.rigid_contacts()
        gap = b.pose("box")[0][:, 2] - H
        fz = np.mean(fz, 0)
        res.append(check(f"rest_load_{cand['id']}", np.allclose(fz, M_BOX * 9.81, rtol=0.01) and b.max_rigid_contacts == 0,
                         rest_gap_m=gap.tolist(), d_hat=I.d_hat(cand), load_over_mg=(fz / (M_BOX * 9.81)).tolist(),
                         rigid_contacts=b.max_rigid_contacts))
        b.destroy()
    # 4 friction pair: sliding box decelerates at mu*g (pair mu = sqrt(mu_a mu_b), all set to 0.3)
    cand = I.parse_candidate("d_hat=1mm")
    g0 = I.start_gap(cand)
    T = I.T_from(np.tile([0, 0, H + g0], (B, 1)), np.tile([1.0, 0, 0, 0], (B, 1)))
    b = I.build({"ground": I.Body(plane_xml(0.3), mu=0.3), "box": I.Body(box_xml("box", H + g0, 0.3), mu=0.3, mass=M_BOX)},
                dt=dt, cand=cand, n_envs=B)
    for _ in range(150):
        b.step()                                               # settle
    b2 = None
    b.destroy()
    V = I.V_from(T, np.tile([1.0, 0, 0], (B, 1)), np.zeros((B, 3)))
    T[:, 2, 3] = H + 0.86 * g0
    b = I.build({"ground": I.Body(plane_xml(0.3), mu=0.3), "box": I.Body(box_xml("box", H + g0, 0.3), mu=0.3, mass=M_BOX)},
                dt=dt, cand=cand, n_envs=B, init={"box": (T, V)})
    vs = []
    for s in range(100):
        b.step(); vs.append(b.vel("box")[0][:, 0])
    vs = np.asarray(vs); decel = -(vs[80] - vs[30]) / (50 * dt)
    res.append(check("sliding_friction_mu_0.3", np.allclose(decel, 0.3 * 9.81, rtol=0.05), decel=decel.tolist(), expected=0.3 * 9.81))
    b.destroy()
    out = {"engine": I.ENGINE, "versions": I.versions(), "checks": res, "all_pass": all(r["pass"] for r in res)}
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(I.json_safe(out), indent=1) + "\n")
    print("ALL PASS" if out["all_pass"] else "SOME FAILED", flush=True)


if __name__ == "__main__":
    main()
