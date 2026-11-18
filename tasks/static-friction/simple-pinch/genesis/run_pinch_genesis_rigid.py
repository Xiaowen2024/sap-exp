#!/usr/bin/env python3
"""simple-pinch force-controlled-v3, Genesis RigidSolver (shared helpers in pinch_v3_common).

Each pad is its own entity on x and z slide joints (armature 0); the x joint gets constant inward force F plus damping
(no stiffness), the z joint tracks the lift by PD, exactly the MuJoCo Warp control law applied as joint forces.
Gravity is off; the cube load m g (x scale) is an external -z force at the cube COM during lift/hold.
Pad-cube friction = cell mu on both geoms (Genesis max rule -> mu).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import genesis_rigid_common as G
import pinch_v3_common as pc

TASK = "simple-pinch"


def entities(cell):
    mu, hy = cell["mu"], cell["pad_halfwidth_y_mm"] * 1e-3
    pad = lambda n, x: G.mjcf(
        f'<body name="{n}" pos="{x} 0 0"><joint name="{n}_x" type="slide" axis="1 0 0"/><joint name="{n}_z" type="slide" axis="0 0 1"/>'
        f'<geom name="{n}_pad" type="box" size="0.01 {hy} 0.04" mass="1" friction="{mu} 0 0"/></body>')
    return {"left": pad("left", -0.03), "right": pad("right", 0.03),
            "cube": G.mjcf(f'<body name="cube" pos="0 0 0"><freejoint/><geom name="cube_geom" type="box" size="0.02 0.02 0.02" '
                           f'mass="{cell["mass_kg"]}" friction="{mu} 0 0"/></body>')}


def run_cell(c, cell, seed, R):
    dt = cell["dt_ms"] * 1e-3
    ents = entities(cell); vec = G.sol_vector(c)
    b = G.build(ents, dt=dt, gravity=(0, 0, 0), cand=c, n_envs=R, sol={k: vec for k in ents})
    L, Rt, cube = b.ents["left"], b.ents["right"], b.ents["cube"]
    gs = G.gs_init()
    p = pc.perturbations(seed, R)
    q0 = G.np_(cube.get_qpos()).astype(np.float64)
    q0[:, 1:3] += p["yz"]; q0[:, 3] = np.cos(p["pitch"] / 2); q0[:, 4] = 0; q0[:, 5] = np.sin(p["pitch"] / 2); q0[:, 6] = 0
    cube.set_qpos(q0.astype(np.float32), zero_velocity=True)
    F = cell["per_pad_force_N"]; load = cell["mass_kg"] * pc.G * p["load_scale"]
    every = max(1, round(pc.SAMPLE_S / dt))
    xf = np.zeros((R, 1, 3), np.float32)

    def control(z, vz, with_load):
        for e, sgn in ((L, 1.0), (Rt, -1.0)):
            q, v = G.np_(e.get_dofs_position()), G.np_(e.get_dofs_velocity())
            f = np.empty((R, 2), np.float32)
            f[:, 0] = sgn * F - pc.PAD_KD * v[:, 0]
            f[:, 1] = pc.PAD_Z_KE * (z - q[:, 1]) + pc.PAD_Z_KD * (vz - v[:, 1])
            e.control_dofs_force(f)
        if with_load:
            xf[:, 0, 2] = -load
            cube.apply_links_external_wrench(force=xf, links_idx_local=[0], ref=gs.link_ref_frame.link_COM)

    def pad_forces():
        con = b.contacts(); fl, nl = b.pair_normal_load("left", "cube", con); fr, nr = b.pair_normal_load("right", "cube", con)
        return np.stack([fl, fr], 1), np.stack([nl > 0, nr > 0], 1)

    with G.Timer() as tm:
        gate_steps = round(pc.GATE_S / dt); GF, GD = [], []
        for s in range(gate_steps):
            control(0.0, 0.0, False); b.scene.step()
            if s >= gate_steps - round(0.2 / dt) and s % every == 0:
                fn, cnt = pad_forces(); GF.append(fn); GD.append(cnt)
        pz = lambda: 0.5 * (G.np_(L.get_dofs_position())[:, 1] + G.np_(Rt.get_dofs_position())[:, 1])
        z0 = G.np_(cube.get_qpos())[:, 2].astype(np.float64).copy(); pz0 = pz().astype(np.float64)
        steps = round((pc.LIFT_S + pc.HOLD_S) / dt); T, RISE, REL = [], [], []
        for s in range(steps + 1):
            t = s * dt; z, vz = pc.lift_target(t)
            if s % every == 0 or s == steps:
                cz = G.np_(cube.get_qpos())[:, 2] - z0; T.append(t); RISE.append(cz); REL.append(cz - (pz() - pz0))
            if s < steps:
                control(z, vz, True); b.scene.step()
    RISE = np.asarray(RISE, float)
    res = pc.summarize(cell, gate_fn=np.asarray(GF), gate_duty=np.asarray(GD), t=np.asarray(T), cube_rise=RISE,
                       rel_z=np.asarray(REL, float), finite=np.isfinite(RISE).all(axis=0), truncated=0)
    res["compiled"] = b.provenance(); res["sim_wall_s"] = tm.s
    b.destroy()
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = G.registry_task(TASK, a.manifest)
    man0, _ = G.load_manifest(a.manifest)
    man = G.with_candidates(man0, spec)
    res = pc.execute(man, "genesis_rigid_candidates", lambda t: G.parse_candidate(t, spec), run_cell, log=lambda s: print(s, flush=True))
    G.save_json(a.out / "genesis_rigid_simple_pinch_v3.json", {**G.header(man0, sha, spec), "complete": True, **res})


if __name__ == "__main__":
    main()
