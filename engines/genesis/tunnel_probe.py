#!/usr/bin/env python3
"""Tunnelling probe (2026-11-17): a 10 mm, 10 g box shot at a fixed 2 mm wall, no gravity, dt = 1 ms, 0.1 s.
Speeds 1 / 5 / 10 / 20 m/s (1 / 5 / 10 / 20 mm per step against a 2 mm wall).  Outcome: final centre x < 0 = blocked,
> 0 = passed through.  One declared parameter set per engine (not tuned):
  IPC  d_hat=1mm,newton_tol=1e-4 (a3 candidate 2) | SAP ke=1e5, tau=3 ms (its preflight shape) | MuJoCo Warp defaults.
Second scene, heavy stack: light 2 cm box (10 g) on the ground, heavy 2 cm box on top (10 g / 1 kg / 10 kg: mass ratio
1 / 100 / 1000), gravity, dt = 1 ms, 1 s.  Each engine uses its frozen three-plate (stacking) setting:
  IPC d_hat=0.1mm,eps_v=1e-3,newton_tol=1e-4 | SAP ke=1e6, tau=3 ms | MuJoCo solref 0.012 2, solimp 0.99 0.99 0.001.
Reported: final penetration (> 0) or standoff (< 0) of the ground-light and light-heavy interfaces, tilt, finiteness.
usage: <python> tunnel_probe.py ipc|sap|mjwarp out.json
"""
from __future__ import annotations

import json
import sys

import numpy as np

SPEEDS = [1.0, 5.0, 10.0, 20.0]
DT, STEPS, X0 = 1e-3, 100, -0.05
WALL_HALF, BOX_HALF, MASS = 0.001, 0.005, 0.01


def summarize(engine, xs, params):
    xs = np.asarray(xs, float)                       # [speeds, steps+1]
    rows = []
    for v, x in zip(SPEEDS, xs):
        pen = float(max(0.0, (x + BOX_HALF + WALL_HALF).max())) if np.isfinite(x).all() else None   # front face past wall back? no:
        front_overlap = np.maximum(0.0, x + BOX_HALF - (-WALL_HALF))           # box front past wall front face
        rows.append({"speed_m_s": v, "mm_per_step": v * DT * 1e3, "final_x_mm": float(x[-1] * 1e3),
                     "outcome": "blocked" if x[-1] < 0 else "PASSED THROUGH",
                     "max_front_overlap_mm": float(np.nanmax(front_overlap) * 1e3) if np.isfinite(x).any() else None,
                     "finite": bool(np.isfinite(x).all())})
    return {"engine": engine, "params": params, "dt_s": DT, "steps": STEPS, "wall_thickness_mm": 2 * WALL_HALF * 1e3,
            "box_size_mm": 2 * BOX_HALF * 1e3, "rows": rows}


def run_ipc():
    import genesis_ipc_common as I
    wall = I.mjcf(f'<body name="wall" pos="0 0 0"><geom type="box" size="{WALL_HALF} 0.05 0.05" friction="0.3 0 0"/></body>')
    box = I.mjcf(f'<body name="box" pos="{X0} 0 0"><freejoint/><geom type="box" size="{BOX_HALF} {BOX_HALF} {BOX_HALF}" mass="{MASS}" friction="0.3 0 0"/></body>')
    c = I.parse_candidate("d_hat=1mm,newton_tol=1e-4"); R = len(SPEEDS)
    T = I.T_from(np.tile([X0, 0, 0], (R, 1)), np.tile([1.0, 0, 0, 0], (R, 1)))
    V = I.V_from(T, np.array([[v, 0, 0] for v in SPEEDS]), np.zeros((R, 3)))
    b = I.build({"wall": I.Body(wall, mu=0.3), "box": I.Body(box, mu=0.3, mass=MASS)}, dt=DT, gravity=(0, 0, 0), cand=c,
                n_envs=R, init={"box": (T, V)})
    xs = [b.pose("box")[0][:, 0].copy()]
    for _ in range(STEPS):
        b.step(); xs.append(b.pose("box")[0][:, 0].copy())
    b.destroy()
    return summarize("Genesis IPC", np.asarray(xs).T, c["text"])


def run_sap():
    import tempfile
    from pathlib import Path
    import warp as wp
    import yaml
    from sap_warp.collision import SapCollisionPipeline
    from sap_warp.loader import load_sap_scene
    from sap_warp.sap_kinematics import sap_eval_fk
    from sap_warp.solver_sap import SolverSAP
    wp.init(); W = len(SPEEDS)
    shape = {"density": MASS / (2 * BOX_HALF) ** 3, "mu": 0.3, "ke": 1e5, "tau": 0.003}
    box = lambda n, p, h: {"id": n, "transform": {"p": list(p), "q": "identity"}, "shapes": [{"type": "box", "hx": h[0], "hy": h[1], "hz": h[2]}]}
    sc = {"schema_version": 1, "name": "tunnel", "simulation": {"dt": DT, "num_worlds": W, "nconmax": 256},
          "builder": {"gravity": 0.0, "defaults": {"shape": shape}}, "ground": {"enabled": False},
          "bodies": [box("wall", (0, 0, 0), (WALL_HALF, .05, .05)), box("box", (X0, 0, 0), (BOX_HALF,) * 3)],
          "joints": [{"id": "wf", "type": "fixed", "parent": "world", "child": "wall"},
                     {"id": "bf", "type": "free", "parent": "world", "child": "box"}],
          "articulations": [{"id": "A", "joints": ["wf"]}, {"id": "B", "joints": ["bf"]}]}
    path = Path(tempfile.mkdtemp()) / "scene.yaml"; path.write_text(yaml.safe_dump(sc, sort_keys=False))
    ld = load_sap_scene(path, device=wp.get_device("cuda:0"), naconmax=256, strict=True, num_worlds=W)
    m, s, c = ld.sap_model, ld.sap_state, ld.sap_control; o = m.state()
    pipe = SapCollisionPipeline(ld.collision_model, naconmax=256, bounds_filter="sphere_obb"); con = pipe.contacts()
    sol = SolverSAP(m, nconmax=256, enable_sleep=False)
    qd = s.joint_qd.numpy().reshape(W, -1); qd[:] = 0; qd[:, 0] = SPEEDS; s.joint_qd.assign(qd.reshape(-1))
    sap_eval_fk(m, s.joint_q, s.joint_qd, s)
    nq = s.joint_q.numpy().size // W
    xs = [s.joint_q.numpy().reshape(W, nq)[:, 0].copy()]
    for _ in range(STEPS):
        s.clear_forces(); sol.collide(pipe, con, s, c, collision_state=s); sol.step(s, o, c, con, DT); s, o = o, s
        xs.append(s.joint_q.numpy().reshape(W, nq)[:, 0].copy())
    return summarize("SAP (fixed build)", np.asarray(xs).T, "shape ke=1e5, tau=3 ms, mu 0.3")


def run_mjwarp():
    import mujoco
    import mujoco_warp as mjw
    import warp as wp
    xml = f"""<mujoco><option timestep="{DT}" gravity="0 0 0"/><worldbody>
      <body name="wall"><geom type="box" size="{WALL_HALF} 0.05 0.05" friction="0.3 0 0"/></body>
      <body name="box" pos="{X0} 0 0"><freejoint/><geom type="box" size="{BOX_HALF} {BOX_HALF} {BOX_HALF}" mass="{MASS}" friction="0.3 0 0"/></body>
    </worldbody></mujoco>"""
    mjm = mujoco.MjModel.from_xml_string(xml); xs_all = []
    for v in SPEEDS:
        mjd = mujoco.MjData(mjm); mjd.qvel[0] = v; mujoco.mj_forward(mjm, mjd)
        m = mjw.put_model(mjm); d = mjw.put_data(mjm, mjd)
        xs = [float(d.qpos.numpy()[0, 0])]
        for _ in range(STEPS):
            mjw.step(m, d); xs.append(float(d.qpos.numpy()[0, 0]))
        xs_all.append(xs)
    return summarize(f"MuJoCo Warp {mujoco.__version__}", np.asarray(xs_all), "MJCF defaults")


HB, M_LIGHT, M_HEAVY, ST_STEPS = 0.01, 0.01, [0.01, 1.0, 10.0], 1000


def stack_rows(engine, params, zs, finite):
    rows = []
    for M, (z1, z2), f in zip(M_HEAVY, zs, finite):
        rows.append({"heavy_kg": M, "mass_ratio": M / M_LIGHT, "finite": bool(f),
                     "ground_light_penetration_mm": float((HB - z1) * 1e3), "light_heavy_penetration_mm": float((z1 + 2 * HB - z2) * 1e3)})
    return {"engine": engine, "params": params, "rows": rows}


def stack_ipc():
    import genesis_ipc_common as I
    c = I.parse_candidate("d_hat=0.1mm,eps_v=1e-3,newton_tol=1e-4"); g0 = I.start_gap(c); out, fin = [], []
    plane = I.mjcf('<geom name="ground" type="plane" size="1 1 .1" friction="0.5 0 0"/>')
    for M in M_HEAVY:
        bx = lambda n, z, m: I.mjcf(f'<body name="{n}" pos="0 0 {z}"><freejoint/><geom type="box" size="{HB} {HB} {HB}" mass="{m}" friction="0.5 0 0"/></body>')
        z1, z2 = HB + g0, 3 * HB + 2 * g0
        b = I.build({"ground": I.Body(plane, mu=0.5), "a": I.Body(bx("a", z1, M_LIGHT), mu=0.5, mass=M_LIGHT), "b": I.Body(bx("b", z2, M), mu=0.5, mass=M)},
                    dt=DT, cand=c, n_envs=1, init={"a": (I.T_from(np.array([[0, 0, z1]]), np.array([[1.0, 0, 0, 0]])), None),
                                                   "b": (I.T_from(np.array([[0, 0, z2]]), np.array([[1.0, 0, 0, 0]])), None)})
        for _ in range(ST_STEPS):
            b.step()
        za, zb = b.pose("a")[0][0, 2], b.pose("b")[0][0, 2]; b.destroy(); out.append((za, zb)); fin.append(np.isfinite([za, zb]).all())
    return stack_rows("Genesis IPC", c["text"], out, fin)


def stack_sap():
    import tempfile
    from pathlib import Path
    import warp as wp
    import yaml
    from sap_warp.collision import SapCollisionPipeline
    from sap_warp.loader import load_sap_scene
    from sap_warp.sap_kinematics import sap_eval_fk
    from sap_warp.solver_sap import SolverSAP
    wp.init(); out, fin = [], []
    for M in M_HEAVY:
        box = lambda n, z, m: {"id": n, "transform": {"p": [0, 0, z], "q": "identity"}, "mass": m, "shapes": [{"type": "box", "hx": HB, "hy": HB, "hz": HB}]}
        sc = {"schema_version": 1, "name": "stack", "simulation": {"dt": DT, "num_worlds": 1, "nconmax": 256},
              "builder": {"gravity": -9.81, "defaults": {"shape": {"density": 1000.0, "mu": 0.5, "ke": 1e6, "tau": 0.003}}}, "ground": {"enabled": True},
              "bodies": [box("a", HB, M_LIGHT), box("b", 3 * HB, M)],
              "joints": [{"id": "fa", "type": "free", "parent": "world", "child": "a"}, {"id": "fb", "type": "free", "parent": "world", "child": "b"}],
              "articulations": [{"id": "A", "joints": ["fa"]}, {"id": "B", "joints": ["fb"]}]}
        path = Path(tempfile.mkdtemp()) / "scene.yaml"; path.write_text(yaml.safe_dump(sc, sort_keys=False))
        ld = load_sap_scene(path, device=wp.get_device("cuda:0"), naconmax=256, strict=True, num_worlds=1)
        m, s, cc = ld.sap_model, ld.sap_state, ld.sap_control; o = m.state()
        pipe = SapCollisionPipeline(ld.collision_model, naconmax=256, bounds_filter="sphere_obb"); con = pipe.contacts()
        sol = SolverSAP(m, nconmax=256, enable_sleep=False); sap_eval_fk(m, s.joint_q, s.joint_qd, s)
        for _ in range(ST_STEPS):
            s.clear_forces(); sol.collide(pipe, con, s, cc, collision_state=s); sol.step(s, o, cc, con, DT); s, o = o, s
        q = s.joint_q.numpy(); za, zb = q[2], q[9]; out.append((za, zb)); fin.append(np.isfinite(q).all())
    return stack_rows("SAP (fixed build)", "ke=1e6, tau=3 ms (three-plate frozen)", out, fin)


def stack_mjwarp():
    import mujoco
    import mujoco_warp as mjw
    out, fin = [], []
    for M in M_HEAVY:
        xml = f"""<mujoco><option timestep="{DT}"/><default><geom solref="0.012 2" solimp="0.99 0.99 0.001" friction="0.5 0 0"/></default><worldbody>
          <geom type="plane" size="1 1 .1"/>
          <body name="a" pos="0 0 {HB}"><freejoint/><geom type="box" size="{HB} {HB} {HB}" mass="{M_LIGHT}"/></body>
          <body name="b" pos="0 0 {3 * HB}"><freejoint/><geom type="box" size="{HB} {HB} {HB}" mass="{M}"/></body></worldbody></mujoco>"""
        mjm = mujoco.MjModel.from_xml_string(xml); mjd = mujoco.MjData(mjm); mujoco.mj_forward(mjm, mjd)
        m = mjw.put_model(mjm); d = mjw.put_data(mjm, mjd)
        for _ in range(ST_STEPS):
            mjw.step(m, d)
        q = d.qpos.numpy()[0]; out.append((q[2], q[9])); fin.append(np.isfinite(q).all())
    return stack_rows(f"MuJoCo Warp {mujoco.__version__}", "solref 0.012 2, solimp 0.99 0.99 0.001 (three-plate frozen)", out, fin)


if __name__ == "__main__":
    eng, out = sys.argv[1], sys.argv[2]
    res = {}
    for name, fn in (("stack", {"ipc": stack_ipc, "sap": stack_sap, "mjwarp": stack_mjwarp}[eng]),
                     ("tunnel", {"ipc": run_ipc, "sap": run_sap, "mjwarp": run_mjwarp}[eng])):
        try:
            res[name] = fn()
            for row in res[name]["rows"]:
                print(eng, name, row, flush=True)
        except Exception as e:  # noqa: BLE001
            import traceback; res[name] = {"error": traceback.format_exc()[-1500:]}; print(eng, name, "ERROR", repr(e), flush=True)
        open(out, "w").write(json.dumps(res, indent=1, default=float) + "\n")
