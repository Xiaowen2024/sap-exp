#!/usr/bin/env python3
"""Genesis + IPC: direct contact-force readout (ContactSystemFeature) versus the momentum-balance readout.

Not a task result.  Four checks, each on a 4 cm, 64 g box (preflight geometry):
  1 rest on the ground at two dt     -> scale of the direct readout (f = -dE/dx / dt^2) and agreement at rest
  2 three-box stack                  -> per-interface loads from the vertices on one face; Newton's third law
  3 horizontal push r = 0.7 / 1.25   -> friction: direct vs momentum, at the default Newton tolerance and at 1e-4
  4 drop and impact                  -> direct vs momentum step by step through an impact

The momentum readout m(dv/dt - g) - F_ext equals whatever net force makes the observed motion, so it agrees with the
weight whenever a body does not accelerate.  The direct readout is the force of IPC's barrier/friction potentials at the
accepted positions; the two differ by the Newton residual, so their gap measures how well each step was solved.
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
import genesis_ipc_preflight as P

MG = P.M_BOX * 9.81
BOTTOM = lambda p: p[:, 2] < 0          # noqa: E731  vertices on the body-frame lower face
TOP = lambda p: p[:, 2] > 0             # noqa: E731


def ground():
    return I.Body(P.plane_xml(0.3), mu=0.3)


def rest(dt, cand, B=2, steps=400):
    g0 = I.start_gap(cand)
    b = I.build({"ground": ground(), "box": I.Body(P.box_xml("box", P.H + g0, 0.3), mu=0.3, mass=P.M_BOX)},
                dt=dt, cand=cand, n_envs=B)
    D, M = [], []
    for s in range(steps):
        b.step()
        if s >= steps - 100:
            d = b.direct_force("box"); D.append(np.c_[d["N"], d["F"], d["nv_active"]]); M.append(b.contact_force("box"))
    D, M = np.mean(D, 0), np.mean(M, 0)
    out = {"dt": dt, "candidate": cand["text"], "direct_N_over_mg": (D[:, 0:3] / MG).tolist(), "direct_F_over_mg": (D[:, 3:6] / MG).tolist(),
           "active_vertices": D[:, 6].tolist(), "momentum_over_mg": (M / MG).tolist(),
           "gap_mm": ((b.pose("box")[0][:, 2] - P.H) * 1e3).tolist()}
    b.destroy()
    out["pass"] = bool(np.allclose(D[:, 2] / MG, 1, rtol=1e-3) and np.abs(D[:, 0:2]).max() < 1e-6 and np.allclose(D[:, 6], 4))
    return out


def stack(dt, cand, B=2, steps=800):
    g0 = I.start_gap(cand)
    bodies = {"ground": ground()}
    for i in range(3):
        z = (2 * i + 1) * P.H + g0 * (i + 1)
        bodies[f"b{i}"] = I.Body(P.box_xml(f"b{i}", z, 0.3), mu=0.3, mass=P.M_BOX)
    init = {f"b{i}": (I.T_from(np.tile([0, 0, (2 * i + 1) * P.H + g0 * (i + 1)], (B, 1)), np.tile([1.0, 0, 0, 0], (B, 1))), None)
            for i in range(3)}
    b = I.build(bodies, dt=dt, cand=cand, n_envs=B, init=init)
    below, above, mom = [], [], []
    for s in range(steps):
        b.step()
        if s >= steps - 100:
            vf = b.vertex_forces()
            below.append(np.stack([b.direct_force(f"b{i}", BOTTOM, vf)["N"][:, 2] for i in range(3)], 1))
            above.append(np.stack([b.direct_force(f"b{i}", TOP, vf)["N"][:, 2] for i in range(3)], 1))
            fz = np.stack([b.contact_force(f"b{i}")[:, 2] for i in range(3)], 1)
            f2 = fz[:, 2]; f1 = fz[:, 1] + f2; f0 = fz[:, 0] + f1
            mom.append(np.stack([f0, f1, f2], 1))
    below, above, mom = (np.mean(x, 0) for x in (below, above, mom))
    expected = np.array([3, 2, 1]) * MG
    third = below[:, 1:] + above[:, :-1]           # force on upper box from below + force on lower box from above
    out = {"dt": dt, "candidate": cand["text"],
           "direct_interface_load_over_expected": (below / expected).tolist(),
           "momentum_interface_load_over_expected": (mom / expected).tolist(),
           "third_law_residual_over_mg": (third / MG).tolist(),
           "top_face_of_top_box_over_mg": (above[:, 2] / MG).tolist(),
           "gaps_mm": [((b.pose("b0")[0][:, 2] - P.H) * 1e3).tolist(),
                       ((b.pose("b1")[0][:, 2] - b.pose("b0")[0][:, 2] - 2 * P.H) * 1e3).tolist(),
                       ((b.pose("b2")[0][:, 2] - b.pose("b1")[0][:, 2] - 2 * P.H) * 1e3).tolist()]}
    b.destroy()
    out["pass"] = bool(np.allclose(below / expected, 1, rtol=1e-2) and np.abs(third).max() < 1e-2 * MG)
    return out


def push(dt, cand, r, mu=0.3, settle=200, steps=500):
    g0 = I.start_gap(cand)
    b = I.build({"ground": I.Body(P.plane_xml(mu), mu=mu), "box": I.Body(P.box_xml("box", P.H + g0, mu), mu=mu, mass=P.M_BOX, force=True)},
                dt=dt, cand=cand, n_envs=1)
    for _ in range(settle):
        b.step()
    b.forces["box"][:, 0] = r * mu * MG
    vx, mx, dF, dN = [], [], [], []
    for s in range(steps):
        b.step()
        vx.append(b.vel("box")[0][0, 0])
        if s >= steps - 100:
            d = b.direct_force("box"); mx.append(b.contact_force("box")[0]); dF.append(d["F"][0]); dN.append(d["N"][0])
    vx = np.asarray(vx); mx, dF, dN = (np.mean(x, 0) for x in (mx, dF, dN))
    acc = (vx[-1] - vx[-101]) / (100 * dt)
    b.destroy()
    eps = cand.get("eps_v", 0.01)
    # converged IPC friction (smoothed f1, |v| < eps_v): 2u - u^2 = r  ->  u = 1 - sqrt(1 - r); above r = 1: slides at (r-1) mu g
    theory = {"creep_mm_s": (1 - math.sqrt(1 - r)) * eps * 1e3} if r < 1 else {"accel_m_s2": (r - 1) * mu * 9.81}
    return {"dt": dt, "candidate": cand["text"], "r": r, "v_end_mm_s": vx[-1] * 1e3, "accel_m_s2": acc, "theory_converged": theory,
            "momentum_Fx_over_muN": mx[0] / (mu * MG), "direct_friction_Fx_over_muN": dF[0] / (mu * MG),
            "direct_normal_over_mg": dN[2] / MG, "residual_over_muN": (mx[0] - dF[0] - dN[0]) / (mu * MG)}


def drop(dt, cand, h0=0.03, steps=300):
    q = np.array([math.cos(0.15), math.sin(0.15), 0, 0])
    T = I.T_from(np.array([[0, 0, P.H * 1.3 + h0]]), q[None])
    b = I.build({"ground": ground(), "box": I.Body(P.box_xml("box", 0.2, 0.3), mu=0.3, mass=P.M_BOX)}, dt=dt, cand=cand, n_envs=1,
                init={"box": (T, None)})
    M, D = [], []
    for _ in range(steps):
        b.step(); M.append(b.contact_force("box")[0]); D.append(b.direct_force("box")["total"][0])
    b.destroy()
    M, D = np.asarray(M), np.asarray(D)
    peak = np.abs(M).max()
    k = int(np.argmax(np.linalg.norm(M, axis=1)))
    return {"dt": dt, "candidate": cand["text"], "peak_momentum_N": peak, "peak_direct_N": float(np.abs(D).max()),
            "peak_step": k, "max_abs_diff_over_peak": float(np.abs(M - D).max() / peak),
            "rms_diff_over_peak": float(np.sqrt(((M - D) ** 2).mean()) / peak)}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args()
    I.gs_init()
    res = {"engine": I.ENGINE, "versions": I.versions()}
    c1 = I.parse_candidate("d_hat=1mm")
    res["rest"] = [rest(0.002, c1), rest(0.001, c1), rest(0.002, I.parse_candidate("native default"))]
    for r_ in res["rest"]:
        print("rest", json.dumps(I.json_safe(r_)), flush=True)
    res["stack"] = [stack(0.002, I.parse_candidate("d_hat=1mm,newton_tol=1e-4"))]
    print("stack", json.dumps(I.json_safe(res["stack"][0])), flush=True)
    res["push"] = []
    for tol in ("", ",newton_tol=1e-4"):
        cand = I.parse_candidate("d_hat=1mm,eps_v=1e-3" + tol)
        for r in (0.7, 1.25):
            x = push(0.002, cand, r); res["push"].append(x); print("push", json.dumps(I.json_safe(x)), flush=True)
    res["drop"] = []
    for tol in ("", ",newton_tol=1e-4"):
        x = drop(0.001, I.parse_candidate("d_hat=1mm" + tol)); res["drop"].append(x); print("drop", json.dumps(I.json_safe(x)), flush=True)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(I.json_safe(res), indent=1) + "\n")


if __name__ == "__main__":
    main()
