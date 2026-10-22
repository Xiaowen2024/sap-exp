"""Comprehensive GAUGE Slope Slider & Turntable Benchmark (MuJoCo vs SAP/Warp).

Evaluates all 18 Table 5 conditions (3 subtasks x 3 materials for Slope Slider
and Turntable) using the exact GAUGE paper Eq. (2) evaluation metrics:
  - E_RMSE(g_sim, mean_g_real) and Normalized RMSE = E_RMSE / Baseline_RMSE
  - E_DTW(g_sim, mean_g_real)  and Normalized DTW  = E_DTW  / Baseline_DTW
and also performs parameter sweeps to quantify:
  1. Out-of-the-box Default accuracy vs Paper Table 3 / Table 5 numbers
  2. Engine Best (both per-task and frozen cross-condition best)
  3. Accurate parameter range & robustness across contact stiffness / time-constant / damping
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent
SAP_ROOT = ROOT.parents[1] / "sap-sim"
DT, FPS = 1.0 / 180.0, 30

# Exact GAUGE Paper Table 5 / Table 3 specifications:
# (family, subtask, material, frames, base_rmse, base_rmse_std, base_dtw, base_dtw_std,
#  isaac_nrmse, genesis_nrmse, newton_nrmse, isaac_ndtw, genesis_ndtw, newton_ndtw)
TABLE5_SPECS = [
    ("slope slider", "task-1", "wood",    9,  0.59,  0.28,  0.59,  0.28, 0.019, 0.074, 24.68, 0.017, 0.055, 14.12),
    ("slope slider", "task-1", "plastic", 9,  1.90,  2.17,  1.90,  2.17, 0.013, 0.016,  6.24, 0.011, 0.014,  4.31),
    ("slope slider", "task-1", "metal",   9,  0.86,  0.69,  0.86,  0.69, 0.078, 0.062, 15.08, 0.067, 0.052, 10.63),
    ("slope slider", "task-2", "wood",   19, 18.00, 15.89,  7.70,  3.69, 1.230, 0.530,  1.71, 1.110, 1.050,  1.79),
    ("slope slider", "task-2", "plastic",17, 25.01, 25.45,  9.13,  5.00, 0.480, 0.820,  0.51, 0.870, 1.000,  1.06),
    ("slope slider", "task-2", "metal",  19, 32.06, 21.93, 10.87,  4.04, 1.000, 0.830,  1.15, 0.450, 0.540,  0.90),
    ("slope slider", "task-3", "wood",   46, 24.27, 16.85,  6.73,  2.56, 2.620, 2.620,  1.35, 2.210, 2.240,  2.16),
    ("slope slider", "task-3", "plastic",46, 49.96, 20.37, 12.97,  4.94, 1.580, 1.460,  0.67, 2.200, 1.830,  1.55),
    ("slope slider", "task-3", "metal",  60, 38.01, 28.53,  9.54,  8.52, 0.610, 0.580,  1.95, 0.710, 0.690,  2.93),
    ("turntable",    "task-1", "wood",   95,  2.01,  1.45,  1.40,  0.64, 0.320, 0.830, 34.37, 0.430, 1.140, 36.33),
    ("turntable",    "task-1", "plastic",95,  2.00,  2.48,  0.97,  0.61, 1.220, 1.120, 36.04, 2.190, 2.220, 53.08),
    ("turntable",    "task-1", "metal",  95,  3.17,  1.74,  1.53,  0.50, 0.420, 0.510, 17.19, 0.820, 0.990, 28.43),
    ("turntable",    "task-2", "wood",   18,  6.76,  4.65,  4.70,  2.68, 4.820, 3.690,  2.88, 4.640, 3.560,  3.26),
    ("turntable",    "task-2", "plastic",17,  7.86,  5.21,  3.92,  1.58, 3.610, 1.570,  1.92, 4.750, 1.960,  2.20),
    ("turntable",    "task-2", "metal",  17,  5.76,  4.17,  3.60,  1.66, 5.700, 3.090,  1.97, 6.060, 3.160,  2.84),
    ("turntable",    "task-3", "wood",   95, 27.15, 27.08, 14.36, 12.90, 1.550, 1.750,  9.50, 0.850, 1.170, 15.09),
    ("turntable",    "task-3", "plastic",95, 12.50,  6.49,  3.48,  0.84, 2.370, 2.470, 21.76, 1.780, 2.040, 66.49),
    ("turntable",    "task-3", "metal",  95, 13.26,  7.43,  3.33,  0.81, 0.170, 0.570, 20.04, 0.610, 2.010, 66.72),
]


def quat_zyx_xyzw(euler_xyz_degrees: list[float]) -> tuple[float, float, float, float]:
    x, y, z = [math.radians(v) / 2.0 for v in euler_xyz_degrees]
    cx, sx, cy, sy, cz, sz = math.cos(x), math.sin(x), math.cos(y), math.sin(y), math.cos(z), math.sin(z)
    return (
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz,
        cx * cy * cz + sx * sy * sz,
    )


def quat_zyx_wxyz(euler_xyz_degrees: list[float]) -> np.ndarray:
    x, y, z, w = quat_zyx_xyzw(euler_xyz_degrees)
    return np.array([w, x, y, z], dtype=float)


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=float)


def dtw_dist(a: np.ndarray, b: np.ndarray) -> float:
    n, m = len(a), len(b)
    cost = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=-1)
    dp = np.full((n + 1, m + 1), np.inf)
    steps = np.zeros((n + 1, m + 1), dtype=int)
    dp[0, 0] = 0.0
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            prevs = (dp[i - 1, j - 1], dp[i - 1, j], dp[i, j - 1])
            ls = (steps[i - 1, j - 1], steps[i - 1, j], steps[i, j - 1])
            k = int(np.argmin(prevs))
            dp[i, j] = cost[i - 1, j - 1] + prevs[k]
            steps[i, j] = ls[k] + 1
    return float(dp[n, m] / steps[n, m])


def load_case_data(spec: tuple) -> dict:
    fam, sub, mat, frms, b_rmse, b_rstd, b_dtw, b_dstd, isaac_r, gen_r, newt_r, isaac_d, gen_d, newt_d = spec
    meta = json.loads((ROOT / "metadata" / f"{fam}.json").read_text())
    task = meta["tasks"][sub]
    fixed = "board" if fam == "slope slider" else "plate"
    d = ROOT / "data" / "full-table5" / fam.replace(" ", "-") / mat / sub
    files = sorted(d.glob("*.json"), key=lambda p: int(p.stem))
    cubes = []
    f_pos_list = []
    f_quat_list = []
    f_ang_list = []
    for f in files:
        raw = json.loads(f.read_text())
        cubes.append(np.column_stack([raw["cube"][k] for k in "xyz"])[:frms] / 1000.0)
        f_pos_list.append(np.column_stack([raw[fixed][k] for k in "xyz"])[:frms] / 1000.0)
        q = np.column_stack([raw[fixed][k] for k in ("qw", "qx", "qy", "qz")])[:frms]
        for i in range(1, len(q)):
            if np.dot(q[i], q[i - 1]) < 0:
                q[i:] *= -1
        f_quat_list.append(q)
        axis_idx = 3 if fam == "turntable" else 2
        ang = np.unwrap(2.0 * np.arctan2(q[:, axis_idx], q[:, 0]))
        ang -= ang[0]
        f_ang_list.append(ang)

    cubes_arr = np.stack(cubes)
    mean_cube = cubes_arr.mean(axis=0)
    mean_f_pos = np.mean(f_pos_list, axis=0)
    mean_f_quat = np.mean(f_quat_list, axis=0)
    mean_f_quat /= np.linalg.norm(mean_f_quat, axis=-1, keepdims=True)
    mean_f_ang = np.mean(f_ang_list, axis=0)

    p_m = np.array(task["translation"]["cube"][mat], dtype=float)
    p_f = np.array(task["translation"][fixed][mat], dtype=float)
    q_m_wxyz = quat_zyx_wxyz(task["orientation"]["cube"][mat])
    q_f_wxyz = quat_zyx_wxyz(task["orientation"][fixed][mat])
    q_m_xyzw = quat_zyx_xyzw(task["orientation"]["cube"][mat])
    q_f_xyzw = quat_zyx_xyzw(task["orientation"][fixed][mat])

    mu_c = float(meta["assets"]["cube"]["material"][mat]["friction"])
    mu_f = float(meta["assets"][fixed]["material"]["wood"]["friction"])
    mass_c = float(meta["assets"]["cube"]["material"][mat]["mass"])
    mass_f = float(meta["assets"][fixed]["material"]["wood"]["mass"])
    dens_c = float(meta["assets"]["cube"]["material"][mat]["density"])
    dens_f = float(meta["assets"][fixed]["material"]["wood"]["density"])
    mu_pair = 2.0 * mu_c * mu_f / (mu_c + mu_f)

    return {
        "family": fam,
        "subtask": sub,
        "material": mat,
        "fixed": fixed,
        "frames": frms,
        "paper_base_rmse_mm": b_rmse,
        "paper_base_rmse_std_mm": b_rstd,
        "paper_base_dtw_mm": b_dtw,
        "paper_base_dtw_std_mm": b_dstd,
        "paper_isaac_nrmse": isaac_r,
        "paper_genesis_nrmse": gen_r,
        "paper_newton_nrmse": newt_r,
        "paper_isaac_ndtw": isaac_d,
        "paper_genesis_ndtw": gen_d,
        "paper_newton_ndtw": newt_d,
        "kinematic": bool(task["kinematic"][fixed]),
        "p_m": p_m,
        "p_f": p_f,
        "q_m_wxyz": q_m_wxyz,
        "q_f_wxyz": q_f_wxyz,
        "q_m_xyzw": q_m_xyzw,
        "q_f_xyzw": q_f_xyzw,
        "mu_c": mu_c,
        "mu_f": mu_f,
        "mu_pair": mu_pair,
        "mass_c": mass_c,
        "mass_f": mass_f,
        "dens_c": dens_c,
        "dens_f": dens_f,
        "mean_cube": mean_cube,
        "cubes_arr": cubes_arr,
        "mean_f_pos": mean_f_pos,
        "mean_f_quat": mean_f_quat,
        "mean_f_ang": mean_f_ang,
    }


def interp_hinge(ang_arr: np.ndarray, step: int) -> tuple[float, float]:
    tf = min(step / 6.0, len(ang_arr) - 1 - 1e-9)
    i = int(tf)
    a = tf - i
    angle = (1.0 - a) * ang_arr[i] + a * ang_arr[i + 1]
    omega = (ang_arr[i + 1] - ang_arr[i]) * FPS
    return float(angle), float(omega)


def interp_6dof(info: dict, step: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    pos = info["mean_f_pos"]
    quat = info["mean_f_quat"]
    tf = min(step / 6.0, len(pos) - 1 - 1e-9)
    i = int(tf)
    a = tf - i
    p = info["p_f"] + (1.0 - a) * pos[i] + a * pos[i + 1] - pos[0]
    v = (pos[i + 1] - pos[i]) * FPS
    q0, q1 = quat[i], quat[i + 1]
    if np.dot(q0, q1) < 0:
        q1 = -q1
    q = (1.0 - a) * q0 + a * q1
    q /= np.linalg.norm(q)
    q_start = quat[0] * np.array([1.0, -1.0, -1.0, -1.0])
    mesh_q = quat_mul(quat_mul(q, q_start), info["q_f_wxyz"])
    delta = quat_mul(q1, q0 * np.array([1.0, -1.0, -1.0, -1.0]))
    if delta[0] < 0:
        delta = -delta
    theta = 2.0 * np.arctan2(np.linalg.norm(delta[1:]), delta[0])
    w = delta[1:] * theta / np.linalg.norm(delta[1:]) * FPS if theta > 1e-10 else np.zeros(3)
    return p, mesh_q, v, w


def eval_pred(info: dict, pred: np.ndarray) -> dict:
    mean_cube = info["mean_cube"]
    rmse_mm = 1000.0 * float(np.sqrt(np.mean(np.sum((pred - mean_cube) ** 2, axis=-1))))
    dtw_mm = 1000.0 * dtw_dist(pred, mean_cube)
    return {
        "rmse_mm": rmse_mm,
        "nrmse": rmse_mm / info["paper_base_rmse_mm"],
        "dtw_mm": dtw_mm,
        "ndtw": dtw_mm / info["paper_base_dtw_mm"],
        "pred_traj_mm": (1000.0 * pred).tolist(),
    }


def sim_mujoco(
    info: dict,
    solref: tuple[float, float] = (0.02, 1.0),
    impratio: float = 1.0,
    cone: str = "pyramidal",
    solver_name: str = "Newton",
    slope3_driver: str = "6dof",
    mu_override: float | None = None,
) -> dict:
    import mujoco

    fixed = info["fixed"]
    kin = info["kinematic"]
    frms = info["frames"]
    mu = info["mu_pair"] if mu_override is None else mu_override
    use_6dof = kin and (info["family"] == "slope slider") and (slope3_driver == "6dof")

    if kin and not use_6dof:
        axis = "0 0 1" if info["family"] == "turntable" else "0 1 0"
        joint_xml = f'<joint name="driver_joint" type="hinge" axis="{axis}"/>'
    elif use_6dof:
        joint_xml = '<freejoint name="driver_joint"/>'
    else:
        joint_xml = ""

    sr_str = f"{solref[0]} {solref[1]}"
    xml = f"""
    <mujoco>
      <compiler meshdir="{ROOT / 'assets'}"/>
      <option timestep="{DT}" gravity="0 0 -9.81" integrator="Euler"
              solver="{solver_name}" impratio="{impratio}" cone="{cone}"/>
      <default>
        <geom solref="{sr_str}"/>
      </default>
      <asset>
        <mesh name="cube_mesh" file="cube.obj"/>
        <mesh name="fixed_mesh" file="{fixed}.obj"/>
      </asset>
      <worldbody>
        <geom name="ground" type="plane" pos="0 0 0" size="2 2 .1" friction="{mu} 0 0"/>
        <body name="{fixed}" pos="{' '.join(map(str, info['p_f']))}" quat="{' '.join(map(str, info['q_f_wxyz']))}">
          {joint_xml}
          <geom name="fixed_geom" type="mesh" mesh="fixed_mesh" mass="{info['mass_f']}" friction="{mu} 0 0"/>
        </body>
        <body name="cube" pos="{' '.join(map(str, info['p_m']))}" quat="{' '.join(map(str, info['q_m_wxyz']))}">
          <freejoint name="cube_free"/>
          <geom name="cube_geom" type="mesh" mesh="cube_mesh" mass="{info['mass_c']}" friction="{mu} 0 0"/>
        </body>
      </worldbody>
    </mujoco>"""
    t0 = time.perf_counter()
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    mujoco.mj_forward(model, data)
    pred = []
    for step in range((frms - 1) * 6 + 1):
        if kin:
            if use_6dof:
                p, mesh_q, v, w = interp_6dof(info, step)
                qinv = mesh_q * np.array([1.0, -1.0, -1.0, -1.0])
                w_local = quat_mul(quat_mul(qinv, np.array([0.0, *w])), mesh_q)[1:]
                data.qpos[:3] = p
                data.qpos[3:7] = mesh_q
                data.qvel[:3] = v
                data.qvel[3:6] = w_local
            else:
                ang, omg = interp_hinge(info["mean_f_ang"], step)
                data.qpos[0] = ang
                data.qvel[0] = omg
            mujoco.mj_forward(model, data)
        if step % 6 == 0:
            pred.append(data.xpos[cid].copy())
        if step < (frms - 1) * 6:
            mujoco.mj_step(model, data)
    elapsed = time.perf_counter() - t0
    res = eval_pred(info, np.asarray(pred))
    res.update({
        "engine": "mujoco",
        "solref": list(solref),
        "impratio": impratio,
        "cone": cone,
        "solver": solver_name,
        "slope3_driver": slope3_driver if info["family"] == "slope slider" and kin else "hinge_or_static",
        "wall_s": elapsed,
    })
    return res


def sim_sap(
    info: dict,
    ke: float = 2500.0,
    tau: float = 0.01,
    preset: str = "approx32",
    slope3_driver: str = "6dof",
    mu_override: float | None = None,
) -> dict:
    import warp as wp
    import yaml

    if str(SAP_ROOT) not in sys.path:
        sys.path.insert(0, str(SAP_ROOT))
    from sim.collision.pipeline import SapCollisionPipeline
    from sim.loader.scene import load_sap_scene
    from sim.resources.collision_model import sap_collision_state_from_state
    from sim.solver_sap import SolverSAP
    from sim.sap_kinematics import sap_eval_fk

    fixed = info["fixed"]
    kin = info["kinematic"]
    frms = info["frames"]
    mu = info["mu_pair"] if mu_override is None else mu_override
    use_6dof = kin and (info["family"] == "slope slider") and (slope3_driver == "6dof")
    line_search = "monotone_decay" if preset == "approx32" else "armijo_decay"
    cap = 256

    if kin and not use_6dof:
        axis = [0, 0, 1] if info["family"] == "turntable" else [0, 1, 0]
        fixed_joint = {
            "id": "driver_joint",
            "type": "revolute",
            "parent": "world",
            "child": fixed,
            "axis": axis,
            "parent_xform": {"p": info["p_f"].tolist(), "q": list(info["q_f_xyzw"])},
            "child_xform": "identity",
        }
    elif use_6dof:
        fixed_joint = {"id": "driver_joint", "type": "free", "parent": "world", "child": fixed}
    else:
        fixed_joint = {
            "id": "driver_joint",
            "type": "fixed",
            "parent": "world",
            "child": fixed,
            "parent_xform": {"p": info["p_f"].tolist(), "q": list(info["q_f_xyzw"])},
            "child_xform": "identity",
        }

    scene = {
        "schema_version": 1,
        "name": f"gauge_{info['family'].replace(' ', '_')}_{info['subtask']}_{info['material']}",
        "simulation": {
            "dt": DT,
            "num_worlds": 1,
            "max_rigid_contact": cap,
            "solver": {"contact_preset_variant": preset, "line_search_variant": line_search},
        },
        "builder": {
            "gravity": -9.81,
            "rigid_gap": 0.0,
            "defaults": {"shape": {"density": 1000.0, "ke": float(ke), "tau": float(tau), "mu": float(mu)}},
        },
        "ground": {"enabled": True, "height": 0.0},
        "bodies": [
            {
                "id": fixed,
                "transform": {"p": info["p_f"].tolist(), "q": list(info["q_f_xyzw"])},
                "shapes": [{
                    "type": "convex_mesh",
                    "path": str(ROOT / "assets" / f"{fixed}.obj"),
                    "is_solid": True,
                    "cfg": {"density": info["dens_f"], "ke": float(ke), "tau": float(tau), "mu": float(mu)},
                }],
            },
            {
                "id": "cube",
                "transform": {"p": info["p_m"].tolist(), "q": list(info["q_m_xyzw"])},
                "shapes": [{
                    "type": "convex_mesh",
                    "path": str(ROOT / "assets" / "cube.obj"),
                    "is_solid": True,
                    "cfg": {"density": info["dens_c"], "ke": float(ke), "tau": float(tau), "mu": float(mu)},
                }],
            },
        ],
        "joints": [
            fixed_joint,
            {"id": "cube_free", "type": "free", "parent": "world", "child": "cube"},
        ],
        "articulations": [
            {"id": "fixed_art", "joints": ["driver_joint"]},
            {"id": "cube_art", "joints": ["cube_free"]},
        ],
    }
    scene_path = ROOT / f"_tmp_sap_{os.getpid()}_{time.time_ns()}.yaml"
    scene_path.write_text(yaml.safe_dump(scene, sort_keys=False))
    t0 = time.perf_counter()
    try:
        loaded = load_sap_scene(scene_path, device="cpu", rigid_contact_max=cap, strict=True)
    finally:
        if scene_path.exists():
            scene_path.unlink()

    model, control = loaded.sap_model, loaded.sap_control
    solver = SolverSAP(model, max_rigid_contact=cap, contact_preset_variant=preset, line_search_variant=line_search)
    collision = SapCollisionPipeline(loaded.collision_model, rigid_contact_max=cap)
    contacts = collision.contacts()
    cid = model.body_label.index("cube")
    cube_jid = model.joint_label.index("cube_free")
    q_cube_start = int(model.joint_q_start.numpy()[cube_jid])

    if kin:
        drv_jid = model.joint_label.index("driver_joint")
        q_drv_start = int(model.joint_q_start.numpy()[drv_jid])
        qd_drv_start = int(model.joint_qd_start.numpy()[drv_jid])

    state, next_state = model.state(), model.state()
    q = state.joint_q.numpy().copy()
    q[q_cube_start : q_cube_start + 3] = info["p_m"]
    q[q_cube_start + 3 : q_cube_start + 7] = info["q_m_xyzw"]
    wp.copy(state.joint_q, wp.array(q, dtype=state.joint_q.dtype, device="cpu"))
    sap_eval_fk(model, state.joint_q, state.joint_qd, state)

    pred = []
    for step in range((frms - 1) * 6 + 1):
        if kin:
            q = state.joint_q.numpy().copy()
            qd = state.joint_qd.numpy().copy()
            if use_6dof:
                p, mesh_q_wxyz, v, w = interp_6dof(info, step)
                q[q_drv_start : q_drv_start + 3] = p
                q[q_drv_start + 3 : q_drv_start + 7] = mesh_q_wxyz[[1, 2, 3, 0]]
                qd[qd_drv_start : qd_drv_start + 3] = v
                qd[qd_drv_start + 3 : qd_drv_start + 6] = w
            else:
                ang, omg = interp_hinge(info["mean_f_ang"], step)
                q[q_drv_start] = ang
                qd[qd_drv_start] = omg
            wp.copy(state.joint_q, wp.array(q, dtype=state.joint_q.dtype, device="cpu"))
            wp.copy(state.joint_qd, wp.array(qd, dtype=state.joint_qd.dtype, device="cpu"))
            sap_eval_fk(model, state.joint_q, state.joint_qd, state)
        if step % 6 == 0:
            pred.append(state.body_q.numpy()[cid, :3].copy())
        if step == (frms - 1) * 6:
            break
        state.clear_forces()
        collision.collide(sap_collision_state_from_state(state), contacts)
        solver.step(state, next_state, control, contacts, DT)
        state, next_state = next_state, state

    elapsed = time.perf_counter() - t0
    res = eval_pred(info, np.asarray(pred))
    res.update({
        "engine": "sap",
        "ke": ke,
        "tau": tau,
        "preset": preset,
        "line_search": line_search,
        "slope3_driver": slope3_driver if info["family"] == "slope slider" and kin else "hinge_or_static",
        "wall_s": elapsed,
    })
    return res


def run_table5(engine: str) -> None:
    rows = []
    for spec in TABLE5_SPECS:
        info = load_case_data(spec)
        if engine == "mujoco":
            r_def_6dof = sim_mujoco(info, solref=(0.02, 1.0), slope3_driver="6dof")
            r_def_hinge = sim_mujoco(info, solref=(0.02, 1.0), slope3_driver="hinge") if (info["family"] == "slope slider" and info["kinematic"]) else r_def_6dof
            row = {
                "family": info["family"],
                "subtask": info["subtask"],
                "material": info["material"],
                "frames": info["frames"],
                "base_rmse_mm": info["paper_base_rmse_mm"],
                "base_dtw_mm": info["paper_base_dtw_mm"],
                "paper_isaac_nrmse": info["paper_isaac_nrmse"],
                "paper_genesis_nrmse": info["paper_genesis_nrmse"],
                "paper_newton_nrmse": info["paper_newton_nrmse"],
                "mj_default_6dof": {k: v for k, v in r_def_6dof.items() if k != "pred_traj_mm"},
                "mj_default_hinge": {k: v for k, v in r_def_hinge.items() if k != "pred_traj_mm"},
            }
            print(f"[MJ] {info['family']:12s} {info['subtask']} {info['material']:7s} | 6dof nRMSE={r_def_6dof['nrmse']:6.2f} ({r_def_6dof['rmse_mm']:6.2f} mm) | hinge nRMSE={r_def_hinge['nrmse']:6.2f} | Paper Newton={info['paper_newton_nrmse']:6.2f}")
        else:
            r_lit_6dof = sim_sap(info, ke=2500.0, tau=0.01, preset="approx32", slope3_driver="6dof")
            r_drake_6dof = sim_sap(info, ke=100000.0, tau=0.003, preset="drake", slope3_driver="6dof")
            r_drake_hinge = sim_sap(info, ke=100000.0, tau=0.003, preset="drake", slope3_driver="hinge") if (info["family"] == "slope slider" and info["kinematic"]) else r_drake_6dof
            row = {
                "family": info["family"],
                "subtask": info["subtask"],
                "material": info["material"],
                "frames": info["frames"],
                "base_rmse_mm": info["paper_base_rmse_mm"],
                "base_dtw_mm": info["paper_base_dtw_mm"],
                "sap_literal_default_6dof": {k: v for k, v in r_lit_6dof.items() if k != "pred_traj_mm"},
                "sap_drake_1e5_6dof": {k: v for k, v in r_drake_6dof.items() if k != "pred_traj_mm"},
                "sap_drake_1e5_hinge": {k: v for k, v in r_drake_hinge.items() if k != "pred_traj_mm"},
            }
            print(f"[SAP] {info['family']:12s} {info['subtask']} {info['material']:7s} | literal(2.5e3,approx32) nRMSE={r_lit_6dof['nrmse']:6.2f} ({r_lit_6dof['rmse_mm']:6.2f} mm) | drake(1e5) nRMSE={r_drake_6dof['nrmse']:6.2f} ({r_drake_6dof['rmse_mm']:6.2f} mm)")
        rows.append(row)
    out = ROOT / f"table5_full_{engine}_results.json"
    out.write_text(json.dumps(rows, indent=2) + "\n")
    print("Saved:", out)


def _sap_sweep_worker(args: tuple) -> dict:
    preset, ke, tau, target_keys = args
    cases = [load_case_data(s) for s in TABLE5_SPECS if (s[0], s[1], s[2]) in target_keys]
    cfg_rows = []
    for info in cases:
        r = sim_sap(info, ke=ke, tau=tau, preset=preset, slope3_driver="6dof")
        cfg_rows.append({
            "family": info["family"],
            "subtask": info["subtask"],
            "material": info["material"],
            "rmse_mm": r["rmse_mm"],
            "nrmse": r["nrmse"],
            "dtw_mm": r["dtw_mm"],
            "ndtw": r["ndtw"],
        })
    mean_nrmse = float(np.mean([x["nrmse"] for x in cfg_rows]))
    max_nrmse = float(np.max([x["nrmse"] for x in cfg_rows]))
    return {
        "engine": "sap",
        "preset": preset,
        "ke": ke,
        "tau": tau,
        "mean_nrmse": mean_nrmse,
        "max_nrmse": max_nrmse,
        "cases": cfg_rows,
    }


def run_sweep(engine: str) -> None:
    target_keys = [
        ("slope slider", "task-2", "wood"),
        ("slope slider", "task-3", "metal"),
        ("turntable", "task-1", "wood"),
        ("turntable", "task-2", "metal"),
        ("turntable", "task-3", "metal"),
    ]
    cases = [load_case_data(s) for s in TABLE5_SPECS if (s[0], s[1], s[2]) in target_keys]

    results = []
    if engine == "mujoco":
        trefs = [0.002, 0.004, 0.006, 0.010, 0.015, 0.020, 0.030, 0.050]
        drefs = [0.5, 0.8, 1.0, 1.5, 2.0]
        cones = ["pyramidal", "elliptic"]
        for cone in cones:
            for tc in trefs:
                for dr in drefs:
                    cfg_rows = []
                    for info in cases:
                        r = sim_mujoco(info, solref=(tc, dr), impratio=1.0, cone=cone, slope3_driver="6dof")
                        cfg_rows.append({
                            "family": info["family"],
                            "subtask": info["subtask"],
                            "material": info["material"],
                            "rmse_mm": r["rmse_mm"],
                            "nrmse": r["nrmse"],
                            "dtw_mm": r["dtw_mm"],
                            "ndtw": r["ndtw"],
                        })
                    mean_nrmse = float(np.mean([x["nrmse"] for x in cfg_rows]))
                    max_nrmse = float(np.max([x["nrmse"] for x in cfg_rows]))
                    rec = {
                        "engine": "mujoco",
                        "tref_s": tc,
                        "dampratio": dr,
                        "cone": cone,
                        "mean_nrmse": mean_nrmse,
                        "max_nrmse": max_nrmse,
                        "cases": cfg_rows,
                    }
                    results.append(rec)
                    print(f"[MJ sweep] cone={cone:9s} tc={tc:.3f} dr={dr:.1f} | mean_nRMSE={mean_nrmse:.3f} max_nRMSE={max_nrmse:.3f} | T3_metal={cfg_rows[4]['rmse_mm']:.2f}mm S3_metal={cfg_rows[1]['rmse_mm']:.2f}mm")
    else:
        from concurrent.futures import ProcessPoolExecutor
        kes = [1000.0, 2500.0, 5000.0, 10000.0, 25000.0, 50000.0, 100000.0, 500000.0]
        taus = [0.001, 0.002, 0.003, 0.005, 0.010, 0.020]
        presets = ["approx32", "drake"]
        tasks = [(preset, ke, tau, target_keys) for preset in presets for ke in kes for tau in taus]
        with ProcessPoolExecutor(max_workers=8) as pool:
            for rec in pool.map(_sap_sweep_worker, tasks):
                results.append(rec)
                cfg_rows = rec["cases"]
                print(f"[SAP sweep] preset={rec['preset']:8s} ke={rec['ke']:8.0f} tau={rec['tau']:.3f} | mean_nRMSE={rec['mean_nrmse']:.3f} max_nRMSE={rec['max_nrmse']:.3f} | T3_metal={cfg_rows[4]['rmse_mm']:.2f}mm S3_metal={cfg_rows[1]['rmse_mm']:.2f}mm", flush=True)

    out = ROOT / f"sweep_5cases_{engine}_results.json"
    out.write_text(json.dumps(results, indent=2) + "\n")
    print("Saved:", out)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("engine", choices=["mujoco", "sap"])
    parser.add_argument("--mode", choices=["table5", "sweep"], default="table5")
    args = parser.parse_args()
    if args.mode == "table5":
        run_table5(args.engine)
    else:
        run_sweep(args.engine)


if __name__ == "__main__":
    main()
