#!/usr/bin/env python3
"""simple-pinch force-controlled-v3, Genesis + IPC (preliminary).  Shared helpers in pinch_v3_common.

Pads keep the manifest's x/z slide joints in Genesis, which applies the manifest control law (constant inward force +
x damping, z PD lift); IPC follows them with the bridge's two_way_soft_constraint.  Per-pad preload = IPC's own barrier
force on the cube face that pad touches (ContactSystemFeature, direct readout); the bridge spring force is kept as a
diagnostic.  Pads start g0 = d_hat outside the cube faces; the gate phase closes the gap.
The cube is an IPC-only ABD; the load m g (x scale) is an IPC external body force during lift/hold.  Gravity off.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import genesis_ipc_common as I
import pinch_v3_common as pc

TASK = "simple-pinch"
LEFT = lambda p: p[:, 0] < 0            # noqa: E731  cube body-frame -x face (touches the left pad)
RIGHT = lambda p: p[:, 0] > 0           # noqa: E731


def xml_for(cell, g0):
    mu, hy = cell["mu"], cell["pad_halfwidth_y_mm"] * 1e-3
    pad = lambda n, x: I.mjcf(
        f'<body name="{n}" pos="{x} 0 0"><joint name="{n}_x" type="slide" axis="1 0 0"/><joint name="{n}_z" type="slide" axis="0 0 1"/>'
        f'<geom name="{n}_pad" type="box" size="0.01 {hy} 0.04" mass="1" friction="{mu} 0 0"/></body>')
    return {"left": pad("left", -0.03 - g0), "right": pad("right", 0.03 + g0),
            "cube": I.mjcf(f'<body name="cube" pos="0 0 0"><freejoint/><geom name="cube_geom" type="box" size="0.02 0.02 0.02" '
                           f'mass="{cell["mass_kg"]}" friction="{mu} 0 0"/></body>')}


def sim_cell(c, cell, seed, R):
    dt = cell["dt_ms"] * 1e-3; g0 = I.start_gap(c); mu = cell["mu"]
    xml = xml_for(cell, g0)
    p = pc.perturbations(seed, R)
    pos = np.zeros((R, 3)); pos[:, 1:3] = p["yz"]
    quat = np.zeros((R, 4)); quat[:, 0] = np.cos(p["pitch"] / 2); quat[:, 2] = np.sin(p["pitch"] / 2)
    bodies = {"left": I.Body(xml["left"], mu=mu, mass=1.0, coup="two_way_soft_constraint"),
              "right": I.Body(xml["right"], mu=mu, mass=1.0, coup="two_way_soft_constraint"),
              "cube": I.Body(xml["cube"], mu=mu, mass=cell["mass_kg"], force=True)}
    b = I.build(bodies, dt=dt, gravity=(0, 0, 0), cand=c, n_envs=R, init={"cube": (I.T_from(pos, quat), None)})
    L, Rt = b.ents["left"], b.ents["right"]
    F = cell["per_pad_force_N"]; load = cell["mass_kg"] * pc.G * p["load_scale"]
    every = max(1, round(pc.SAMPLE_S / dt))

    def control(z, vz, with_load):
        for e, sgn in ((L, 1.0), (Rt, -1.0)):
            q, v = I.np_(e.get_dofs_position()), I.np_(e.get_dofs_velocity())
            f = np.empty((R, 2), np.float32)
            f[:, 0] = sgn * F - pc.PAD_KD * v[:, 0]
            f[:, 1] = pc.PAD_Z_KE * (z - q[:, 1]) + pc.PAD_Z_KD * (vz - v[:, 1])
            e.control_dofs_force(f)
        b.forces["cube"][:, 2] = -load if with_load else 0.0

    def pad_forces():
        # direct: IPC barrier force on the cube's -x / +x faces (= force from the left / right pad)
        vf = b.vertex_forces()
        dl, dr = b.direct_force("cube", LEFT, vf), b.direct_force("cube", RIGHT, vf)
        fn = np.stack([np.abs(dl["N"][:, 0]), np.abs(dr["N"][:, 0])], 1)
        fc = np.stack([np.abs(b.coupling_force("left")[:, 0]), np.abs(b.coupling_force("right")[:, 0])], 1)   # bridge spring
        return fn, fn > 1e-3 * F, fc

    with I.Timer() as tm:
        gate_steps = round(pc.GATE_S / dt); GF, GD, GC = [], [], []
        for s in range(gate_steps):
            control(0.0, 0.0, False); b.step()
            if s >= gate_steps - round(0.2 / dt) and s % every == 0:
                fn, cnt, fc = pad_forces(); GF.append(fn); GD.append(cnt); GC.append(fc)
        pz = lambda: 0.5 * (I.np_(L.get_dofs_position())[:, 1] + I.np_(Rt.get_dofs_position())[:, 1])
        z0 = b.pose("cube")[0][:, 2].copy(); pz0 = pz().astype(np.float64)
        steps = round((pc.LIFT_S + pc.HOLD_S) / dt); T, RISE, REL = [], [], []
        for s in range(steps + 1):
            t = s * dt; z, vz = pc.lift_target(t)
            if s % every == 0 or s == steps:
                cz = b.pose("cube")[0][:, 2] - z0; T.append(t); RISE.append(cz); REL.append(cz - (pz() - pz0))
                if s % (every * 20) == 0:
                    b.rigid_contacts()
            if s < steps:
                control(z, vz, True); b.step()
    raw = {"GF": np.asarray(GF, float), "GD": np.asarray(GD), "GC": np.asarray(GC, float), "T": np.asarray(T), "RISE": np.asarray(RISE, float),
           "REL": np.asarray(REL, float), "compiled": b.provenance(), "sim_wall_s": tm.s}
    b.destroy()
    return raw


def run_cell(c, cell, seed, R):
    dt = cell["dt_ms"] * 1e-3; every = max(1, round(pc.SAMPLE_S / dt))
    raw, why = I.ISO.call("run_pinch_genesis_ipc", "sim_cell", tag=f"{c['id']}|{cell['id']}", c=c, cell=cell, seed=seed, R=R)
    if raw is None:
        gate_steps = round(pc.GATE_S / dt); ng = len([s for s in range(gate_steps) if s >= gate_steps - round(0.2 / dt) and s % every == 0])
        steps = round((pc.LIFT_S + pc.HOLD_S) / dt); T = np.array([s * dt for s in range(steps + 1) if s % every == 0 or s == steps])
        raw = {"GF": np.full((ng, R, 2), np.nan), "GD": np.zeros((ng, R, 2), bool), "T": T, "RISE": np.full((len(T), R), np.nan),
               "REL": np.full((len(T), R), np.nan), "compiled": {"failed": why}, "sim_wall_s": float("nan")}
    with np.errstate(all="ignore"):
        res = pc.summarize(cell, gate_fn=raw["GF"], gate_duty=raw["GD"], t=raw["T"], cube_rise=raw["RISE"],
                           rel_z=raw["REL"], finite=np.isfinite(raw["RISE"]).all(axis=0), truncated=0)
    res["compiled"] = raw["compiled"]; res["sim_wall_s"] = raw["sim_wall_s"]; res["failed"] = why
    if "GC" in raw and raw["GC"].size:
        with np.errstate(all="ignore"):
            res["readout_check"] = {
                "preload_readout": "direct: IPC barrier force on the cube's -x/+x faces (ContactSystemFeature)",
                "direct_preload_mean_N_left_right": np.nanmean(raw["GF"], axis=(0, 1)).tolist(),
                "bridge_spring_preload_mean_N_left_right": np.nanmean(raw["GC"], axis=(0, 1)).tolist(),
                "direct_minus_bridge_over_F_max": float(np.nanmax(np.abs(raw["GF"] - raw["GC"])) / cell["per_pad_force_N"])}
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = I.registry_task(TASK, a.manifest)
    man0, _ = I.load_manifest(a.manifest)
    man = I.with_candidates(man0, spec)
    res = pc.execute(man, "genesis_ipc_candidates", I.parse_candidate, run_cell, log=lambda s: print(s, flush=True))
    I.save_json(a.out / "genesis_ipc_simple_pinch_v3.json", {**I.header(man0, sha, spec), "complete": True, **res})


if __name__ == "__main__":
    main()
