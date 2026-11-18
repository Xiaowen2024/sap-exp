"""Complete runner for dynamic-friction-v2 protocol on MuJoCo 3.17.0 and MuJoCo Warp 3.17.0.

Covers:
  1. Task D1: Slope Slider (Real-World GAUGE Slope Slider + Analytical Incline & Horizontal Sliding)
  2. Task D2: MIT Push (Real-World MIT Push Trajectory + Coupled Wrench-Twist Windows + Setup Sweep)
  3. Primary budget: B = 1 native default and B = 5 mechanism-guided;
     B = 30 is an optional historical upper-bound sweep.
  4. Repeated rollouts (M = 20 perturbed rollouts per condition)
  5. Plausible-range controllability, numerical local robustness (dt), and setup-variant sweeps
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from metrics_common import (
    HAS_MJWARP,
    compute_binary_outcome_stats,
    dtw_distance,
    fit_quadratic_acceleration,
    mjw,
    spearman_rank_corr,
    summarize_budget_curve,
    trajectory_rmse,
    warp_contact_records,
    wilson_ci,
)
from native_defaults import native_default_mjcf

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = ROOT / "exp-results" / "protocol-v2" / "dynamic-friction-v2"
GAUGE_ROOT = ROOT / "exp-results" / "gauge-rigid-reproduction"
MIT_ASSETS = ROOT / "exp-results" / "mit-push-v2" / "assets"
G = 9.81


# =============================================================================
# 1. TASK D1a: ANALYTICAL INCLINE & FREE SLIDING (WITH REPEATED ROLLOUTS M=20)
# =============================================================================

def incline_slider_xml(
    *,
    dt: float = 0.001,
    theta_deg: float = 30.0,
    mu: float = 0.268104,
    mass: float = 0.0903,
    half_size: float = 0.025,
    solref_tc: float = 0.004,
    solref_dr: float = 1.0,
    cone: str = "elliptic",
    impratio: float = 1.0,
) -> str:
    # Support frame is aligned with the slope: x_s is down-slope, z_s is outward normal
    rad = math.radians(theta_deg)
    gx = G * math.sin(rad)
    gz = -G * math.cos(rad)
    return f"""<mujoco model="incline_slider_v2">
  <option timestep="{dt}" gravity="{gx} 0 {gz}" integrator="implicitfast"
          solver="Newton" cone="{cone}" impratio="{impratio}"
          iterations="100" tolerance="1e-12"/>
  <size nconmax="64" njmax="256"/>
  <worldbody>
    <geom name="support" type="plane" size="4 4 0.1" friction="{mu} 0.005 0.0001"/>
    <body name="slider" pos="0 0 {half_size}">
      <freejoint name="slider_free"/>
      <geom name="slider_geom" type="box" size="{half_size} {half_size} {half_size}"
            mass="{mass}" friction="{mu} 0.005 0.0001"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="support" geom2="slider_geom" condim="3" margin="0" gap="0"
          friction="{mu} {mu} 0.005 0.0001 0.0001"
          solref="{solref_tc} {solref_dr}" solimp="0.9 0.95 0.001 0.5 2"/>
  </contact>
</mujoco>"""


def _extract_support_contact_mj(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    mu: float,
    cube_geom: int | None = None,
    board_geom: int | None = None,
) -> dict[str, float]:
    fn = 0.0
    tangential_world = np.zeros(3)
    count = 0
    max_gap = -1e9
    w = np.zeros(6)
    for ci in range(data.ncon):
        c = data.contact[ci]
        if cube_geom is not None and board_geom is not None and {int(c.geom1), int(c.geom2)} != {cube_geom, board_geom}:
            continue
        mujoco.mj_contactForce(model, data, ci, w)
        frame = np.asarray(c.frame, dtype=float).reshape(3, 3)
        f_world = frame.T @ w[:3]
        normal_world = frame.T @ np.array([w[0], 0.0, 0.0])
        fn += max(0.0, float(w[0]))
        tangential_world += f_world - normal_world
        count += 1
        max_gap = max(max_gap, float(c.dist))
    ft = float(np.linalg.norm(tangential_world))
    rho = ft / (mu * fn) if fn > 1e-12 else float("nan")
    return {
        "Fn": fn,
        "Ft": ft,
        "rho": rho,
        "count": count,
        "max_gap": max_gap if count > 0 else 0.0,
    }


def _extract_support_contact_warp(
    data: Any,
    world: int,
    mu: float,
    cube_geom: int | None = None,
    board_geom: int | None = None,
) -> dict[str, float]:
    fn = 0.0; tangential_world = np.zeros(3); count = 0
    for c in warp_contact_records(data, world):
        if cube_geom is not None and board_geom is not None and {c["geom1"], c["geom2"]} != {cube_geom, board_geom}:
            continue
        fn += c["normal"]; tangential_world += c["world_tangent"]; count += 1
    ft = float(np.linalg.norm(tangential_world))
    return {"Fn": fn, "Ft": ft, "rho": ft/(mu*fn) if fn > 1e-12 else float("nan"),
            "count": count, "max_gap": 0.0}


def run_incline_or_horizontal_rollouts(
    *,
    backend: str,
    xml_str: str,
    theta_deg: float,
    mu: float,
    mass: float,
    v0: float,
    dt: float,
    duration_s: float = 0.6,
    half_size: float = 0.025,
    n_rollouts: int = 20,
    seed_base: int = 3000,
) -> dict[str, Any]:
    """Run M perturbed rollouts of support-frame sliding in 'mujoco' or 'mujoco_warp'."""
    model = mujoco.MjModel.from_xml_string(xml_str)
    settle_steps = round(0.15 / dt)
    slide_steps = round(duration_s / dt)
    sample_every = max(1, round(0.005 / dt))

    rad = math.radians(theta_deg)
    # Analytical acceleration along support (+x down-slope)
    if theta_deg > 0.0:
        a_star = G * (math.sin(rad) - mu * math.cos(rad))
        expected_sliding = bool(math.tan(rad) > mu * 1.02)
    else:
        a_star = -mu * G
        expected_sliding = bool(v0 > 0.05)

    init_qpos = np.tile(model.qpos0, (n_rollouts, 1)).astype(np.float64)
    v0_pert = np.full(n_rollouts, v0, dtype=np.float64)
    # R independent local perturbations.  The nominal trajectory is evaluated
    # separately, never included as rollout 0 in an R=20 distribution.
    for r in range(n_rollouts):
        rng = np.random.default_rng(seed_base + r)
        init_qpos[r, 0] += rng.uniform(-0.0005, 0.0005)
        init_qpos[r, 1] += rng.uniform(-0.0005, 0.0005)
        yaw = rng.uniform(-0.008, 0.008)
        init_qpos[r, 3] = math.cos(yaw / 2.0)
        init_qpos[r, 6] = math.sin(yaw / 2.0)
        v0_pert[r] = v0 * (1.0 + rng.uniform(-0.01, 0.01))

    t_samples: list[float] = []
    s_hist: list[np.ndarray] = []
    z_dev_hist: list[np.ndarray] = []
    fn_hist: list[np.ndarray] = []
    rho_hist: list[np.ndarray] = []
    duty_hist: list[np.ndarray] = []
    overflow_count = 0

    wall_t0 = time.perf_counter()

    if backend == "mujoco_warp":
        gpu_model = mjw.put_model(model)
        data = mjw.make_data(model, nworld=n_rollouts, nconmax=64, njmax=256)
        data.qpos.assign(init_qpos.astype(np.float32))
        # Hold x fixed during normal settle
        for _ in range(settle_steps):
            qvel = data.qvel.numpy()
            qvel[:, 0] = 0.0
            qvel[:, 1] = 0.0
            data.qvel.assign(qvel)
            mjw.step(gpu_model, data)

        qpos0 = data.qpos.numpy()[:, 0].copy()
        qvel = data.qvel.numpy()
        qvel[:, 0] = v0_pert.astype(np.float32)
        data.qvel.assign(qvel)
        mjw.forward(gpu_model, data)
        mj_probe = mujoco.MjData(model)

        for step in range(slide_steps + 1):
            t = step * dt
            if step % sample_every == 0:
                qpos = data.qpos.numpy()
                s_hist.append(qpos[:, 0] - qpos0)
                z_dev_hist.append(qpos[:, 2] - half_size)
                t_samples.append(t)

                contacts = [_extract_support_contact_warp(data, r, mu) for r in range(n_rollouts)]
                fn_hist.append(np.array([c["Fn"] for c in contacts]))
                rho_hist.append(np.array([c["rho"] for c in contacts]))
                duty_hist.append(np.array([1.0 if c["count"] > 0 else 0.0 for c in contacts]))
            if step < slide_steps:
                mjw.step(gpu_model, data)
        overflow_count = int(np.sum(data.overflow.numpy() > 0))
    else:
        datas = [mujoco.MjData(model) for _ in range(n_rollouts)]
        for r in range(n_rollouts):
            datas[r].qpos[:] = init_qpos[r]
            mujoco.mj_forward(model, datas[r])
            for _ in range(settle_steps):
                datas[r].qvel[0] = 0.0
                datas[r].qvel[1] = 0.0
                mujoco.mj_step(model, datas[r])

        qpos0 = np.array([float(datas[r].qpos[0]) for r in range(n_rollouts)])
        for r in range(n_rollouts):
            datas[r].qvel[0] = v0_pert[r]
            mujoco.mj_forward(model, datas[r])

        for step in range(slide_steps + 1):
            t = step * dt
            if step % sample_every == 0:
                s_hist.append(np.array([float(datas[r].qpos[0]) - qpos0[r] for r in range(n_rollouts)]))
                z_dev_hist.append(np.array([float(datas[r].qpos[2]) - half_size for r in range(n_rollouts)]))
                t_samples.append(t)

                contacts = [_extract_support_contact_mj(model, d, mu) for d in datas]
                fn_hist.append(np.array([c["Fn"] for c in contacts]))
                rho_hist.append(np.array([c["rho"] for c in contacts]))
                duty_hist.append(np.array([1.0 if c["count"] > 0 else 0.0 for c in contacts]))
            if step < slide_steps:
                for r in range(n_rollouts):
                    mujoco.mj_step(model, datas[r])

    wall_s = time.perf_counter() - wall_t0

    t_arr = np.asarray(t_samples, dtype=float)
    s_arr = np.asarray(s_hist, dtype=float)  # (T, M)
    z_dev_arr = np.asarray(z_dev_hist, dtype=float)

    # Analytical trajectory reference
    if theta_deg == 0.0 and v0 > 0.0:
        t_stop = v0 / (mu * G)
        s_star_t = np.where(t_arr <= t_stop, v0 * t_arr + 0.5 * a_star * (t_arr ** 2), (v0 ** 2) / (2.0 * mu * G))
        fit_mask = t_arr <= min(duration_s, 0.9 * t_stop)
    elif a_star > 0.0:
        s_star_t = v0 * t_arr + 0.5 * a_star * (t_arr ** 2)
        fit_mask = t_arr >= 0.02
    else:
        s_star_t = np.zeros_like(t_arr)
        fit_mask = t_arr >= 0.02

    s_star_terminal = float(s_star_t[-1])
    fn_a, rho_a, duty_a = np.asarray(fn_hist), np.asarray(rho_hist), np.asarray(duty_hist)

    rollout_records = []
    a_errs = []
    term_errs_mm = []
    rmses_mm = []
    dtws_mm = []
    max_liftoff_mm_list = []

    for r in range(n_rollouts):
        sr = s_arr[:, r]
        zr = z_dev_arr[:, r]
        a_hat, _, _ = fit_quadratic_acceleration(t_arr[fit_mask], sr[fit_mask])
        a_err = abs(a_hat - a_star)
        term_err_m = abs(float(sr[-1]) - s_star_terminal)
        rmse_m = trajectory_rmse(sr, s_star_t)
        dtw_m = dtw_distance(sr, s_star_t, window=10)
        max_liftoff_mm = float(np.max(np.maximum(0.0, zr)) * 1e3)
        max_height_dev_mm = float(np.max(np.abs(zr)) * 1e3)

        # Binary task outcome:
        # If expected_sliding is True: passes if terminal displacement is within 15% (or 15 mm) of s_star_terminal and no lift-off (>2 mm)
        # Positive class = traverses sliding distance threshold (>= 20 mm) with continuous support
        sim_sliding = bool(float(sr[-1]) >= 0.020 and max_liftoff_mm <= 2.0)

        a_errs.append(a_err)
        term_errs_mm.append(term_err_m * 1e3)
        rmses_mm.append(rmse_m * 1e3)
        dtws_mm.append(dtw_m * 1e3)
        max_liftoff_mm_list.append(max_height_dev_mm)
        rollout_records.append({
            "rollout": r,
            "a_hat_m_s2": a_hat,
            "a_error_m_s2": a_err,
            "terminal_disp_mm": float(sr[-1]) * 1e3,
            "terminal_disp_error_mm": term_err_m * 1e3,
            "trajectory_rmse_mm": rmse_m * 1e3,
            "dtw_mm": dtw_m * 1e3,
            "max_height_dev_mm": max_height_dev_mm,
            "normal_force_mean_N": float(np.mean(fn_a[:, r])),
            "friction_utilization_rho": float(np.nanmean(rho_a[:, r])),
            "contact_duty": float(np.mean(duty_a[:, r])),
            "expected_positive": expected_sliding,
            "simulated_positive": sim_sliding,
            "agreement": expected_sliding == sim_sliding,
        })

    agree_count = sum(1 for rec in rollout_records if rec["agreement"])
    duty_by_rollout = np.mean(duty_a, axis=0)
    rho_by_rollout = np.nanmean(rho_a, axis=0)

    return {
        "backend": backend,
        "theta_deg": theta_deg,
        "mu_k": mu,
        "mass_kg": mass,
        "v0_m_s": v0,
        "dt_ms": dt * 1e3,
        "M_rollouts": n_rollouts,
        "a_star_m_s2": a_star,
        "s_star_terminal_mm": s_star_terminal * 1e3,
        "a_hat_mean_m_s2": float(np.mean([rec["a_hat_m_s2"] for rec in rollout_records])),
        "tangential_accel_error_mean_m_s2": float(np.mean(a_errs)),
        "terminal_disp_error_mean_mm": float(np.mean(term_errs_mm)),
        "trajectory_rmse_mean_mm": float(np.mean(rmses_mm)),
        "dtw_mean_mm": float(np.mean(dtws_mm)),
        "friction_utilization_rho": float(np.nanmean(rho_by_rollout)),
        "friction_utilization_rho_std": float(np.nanstd(rho_by_rollout)),
        "contact_duty": float(np.mean(duty_by_rollout)),
        "contact_duty_std": float(np.std(duty_by_rollout)),
        "normal_force_mean_N": float(np.mean(fn_a)),
        "normal_force_std_across_rollouts_N": float(np.std(np.mean(fn_a, axis=0))),
        "airborne_fraction": 1.0 - float(np.mean(duty_by_rollout)),
        "max_normal_height_dev_mm": float(np.max(max_liftoff_mm_list)),
        "expected_positive": expected_sliding,
        "pass_fraction_p_hat": agree_count / n_rollouts,
        "pass_wilson_95ci": list(wilson_ci(agree_count, n_rollouts)),
        "overflow_count": overflow_count,
        "wall_s": wall_s,
        "rollouts": rollout_records,
    }


# =============================================================================
# 2. TASK D1b: REAL-WORLD GAUGE SLOPE SLIDER (20 REAL MOCAP TRIALS PER CASE)
# =============================================================================

GAUGE_SPECS = {
    ("wood", "task-1"): (9, 0.59, 0.59),
    ("plastic", "task-1"): (9, 1.90, 1.90),
    ("metal", "task-1"): (9, 0.86, 0.86),
    ("wood", "task-2"): (19, 18.00, 7.70),
    ("plastic", "task-2"): (17, 25.01, 9.13),
    ("metal", "task-2"): (19, 32.06, 10.87),
    ("wood", "task-3"): (46, 24.27, 6.73),
    ("plastic", "task-3"): (46, 49.96, 12.97),
    ("metal", "task-3"): (60, 38.01, 9.54),
}


def _quat_zyx_wxyz(euler_xyz_degrees: list[float]) -> np.ndarray:
    x, y, z = [math.radians(v) / 2.0 for v in euler_xyz_degrees]
    cx, sx, cy, sy, cz, sz = math.cos(x), math.sin(x), math.cos(y), math.sin(y), math.cos(z), math.sin(z)
    qx = sx * cy * cz - cx * sy * sz
    qy = cx * sy * cz + sx * cy * sz
    qz = cx * cy * sz - sx * sy * cz
    qw = cx * cy * cz + sx * sy * sz
    return np.array([qw, qx, qy, qz], dtype=float)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=float)


def load_gauge_slope_slider_case(material: str, subtask: str) -> dict[str, Any]:
    meta_path = GAUGE_ROOT / "metadata" / "slope slider.json"
    meta = json.loads(meta_path.read_text())
    task_info = meta["tasks"][subtask]
    frms, base_rmse_mm, base_dtw_mm = GAUGE_SPECS[(material, subtask)]

    a = meta["assets"]["cube"]["material"][material]
    b = meta["assets"]["board"]["material"]["wood"]
    mu_pair = 2.0 * a["friction"] * b["friction"] / (a["friction"] + b["friction"])

    trial_dir = GAUGE_ROOT / "data" / "full-table5" / "slope-slider" / material / subtask
    files = sorted(trial_dir.glob("*.json"), key=lambda p: int(p.stem))
    cubes = []
    f_pos_list = []
    f_quat_list = []
    for f in files:
        raw = json.loads(f.read_text())
        cubes.append(np.column_stack([raw["cube"][k] for k in "xyz"])[:frms] / 1000.0)
        f_pos_list.append(np.column_stack([raw["board"][k] for k in "xyz"])[:frms] / 1000.0)
        q = np.column_stack([raw["board"][k] for k in ("qw", "qx", "qy", "qz")])[:frms]
        for i in range(1, len(q)):
            if np.dot(q[i], q[i - 1]) < 0:
                q[i:] *= -1
        f_quat_list.append(q)

    cubes_arr = np.stack(cubes, axis=0)
    mean_cube = np.mean(cubes_arr, axis=0)
    mean_f_pos = np.mean(f_pos_list, axis=0)
    mean_f_quat = np.mean(f_quat_list, axis=0)
    mean_f_quat /= np.linalg.norm(mean_f_quat, axis=-1, keepdims=True)

    p_m = np.array(task_info["translation"]["cube"][material], dtype=float)
    p_f = np.array(task_info["translation"]["board"][material], dtype=float)
    q_m_wxyz = _quat_zyx_wxyz(task_info["orientation"]["cube"][material])
    q_f_wxyz = _quat_zyx_wxyz(task_info["orientation"]["board"][material])

    return {
        "material": material,
        "subtask": subtask,
        "frames": frms,
        "kinematic": bool(task_info["kinematic"]["board"]),
        "mass_kg": float(a["mass"]),
        "mass_board_kg": float(b["mass"]),
        "mu_pair": float(mu_pair),
        "p_m": p_m,
        "p_f": p_f,
        "q_m_wxyz": q_m_wxyz,
        "q_f_wxyz": q_f_wxyz,
        "trials_cube": cubes_arr,
        "mean_cube": mean_cube,
        "mean_f_pos": mean_f_pos,
        "mean_f_quat": mean_f_quat,
        "real_spread_mm": float(base_rmse_mm),
        "base_dtw_mm": float(base_dtw_mm),
    }


def _interp_board_6dof(case_data: dict[str, Any], step: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    pos = case_data["mean_f_pos"]
    quat = case_data["mean_f_quat"]
    tf = min(step / 6.0, len(pos) - 1 - 1e-9)
    i = int(tf)
    a = tf - i
    p = case_data["p_f"] + (1.0 - a) * pos[i] + a * pos[i + 1] - pos[0]
    v = (pos[i + 1] - pos[i]) * 30.0
    q0, q1 = quat[i], quat[i + 1]
    if np.dot(q0, q1) < 0:
        q1 = -q1
    q = (1.0 - a) * q0 + a * q1
    q /= np.linalg.norm(q)
    q_start = quat[0] * np.array([1.0, -1.0, -1.0, -1.0])
    mesh_q = _quat_mul(_quat_mul(q, q_start), case_data["q_f_wxyz"])
    delta = _quat_mul(q1, q0 * np.array([1.0, -1.0, -1.0, -1.0]))
    if delta[0] < 0:
        delta = -delta
    theta = 2.0 * np.arctan2(np.linalg.norm(delta[1:]), delta[0])
    w = delta[1:] * theta / np.linalg.norm(delta[1:]) * 30.0 if theta > 1e-10 else np.zeros(3)
    qinv = mesh_q * np.array([1.0, -1.0, -1.0, -1.0])
    w_local = _quat_mul(_quat_mul(qinv, np.array([0.0, *w])), mesh_q)[1:]
    return p, mesh_q, v, w_local


def evaluate_gauge_slope_slider_case(
    *,
    backend: str,
    case_data: dict[str, Any],
    solref_tc: float = 0.004,
    solref_dr: float = 1.0,
    cone: str = "elliptic",
    impratio: float = 1.0,
    native_default: bool = False,
) -> dict[str, Any]:
    """Run all 20 trials of a GAUGE Slope Slider case using exact mesh assets on 'mujoco' or 'mujoco_warp'."""
    material = case_data["material"]
    subtask = case_data["subtask"]
    trials = case_data["trials_cube"]  # (N_trials, F, 3)
    n_trials, n_frames, _ = trials.shape
    dt = 1.0 / 180.0
    kin = case_data["kinematic"]
    mu = case_data["mu_pair"]
    joint_xml = '<freejoint name="driver_joint"/>' if kin else ""

    xml_s = f"""<mujoco model="gauge_slope_slider_v2">
  <compiler meshdir="{GAUGE_ROOT / 'assets'}"/>
  <option timestep="{dt}" gravity="0 0 -9.81" integrator="Euler"
          solver="Newton" impratio="{impratio}" cone="{cone}"/>
  <size nconmax="128" njmax="512"/>
  <default>
    <geom solref="{solref_tc} {solref_dr}"/>
  </default>
  <asset>
    <mesh name="cube_mesh" file="cube.obj"/>
    <mesh name="fixed_mesh" file="board.obj"/>
  </asset>
  <worldbody>
    <geom name="ground" type="plane" pos="0 0 0" size="2 2 .1" friction="{mu} 0 0"/>
    <body name="board" pos="{' '.join(map(str, case_data['p_f']))}" quat="{' '.join(map(str, case_data['q_f_wxyz']))}">
      {joint_xml}
      <geom name="fixed_geom" type="mesh" mesh="fixed_mesh" mass="{case_data['mass_board_kg']}" friction="{mu} 0 0"/>
    </body>
    <body name="cube" pos="{' '.join(map(str, case_data['p_m']))}" quat="{' '.join(map(str, case_data['q_m_wxyz']))}">
      <freejoint name="cube_free"/>
      <geom name="cube_geom" type="mesh" mesh="cube_mesh" mass="{case_data['mass_kg']}" friction="{mu} 0 0"/>
    </body>
  </worldbody>
</mujoco>"""

    if native_default:
        xml_s = native_default_mjcf(xml_s)

    model = mujoco.MjModel.from_xml_string(xml_s)
    cube_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    board_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "fixed_geom")
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_q = int(model.jnt_qposadr[model.body_jntadr[cid]])

    init_qpos = np.tile(model.qpos0, (n_trials, 1)).astype(np.float64)
    for r in range(n_trials):
        rng = np.random.default_rng(3500 + r)
        # Every rollout is an independent draw.  The experimentally measured
        # trial-to-trial start variation is retained but none is privileged as
        # an unperturbed nominal simulation.
        delta_r = 0.25 * (trials[r, 0] - case_data["mean_cube"][0])
        init_qpos[r, cube_q:cube_q + 3] += delta_r + rng.uniform(-0.0002, 0.0002, size=3)

    total_steps = (n_frames - 1) * 6
    pred_trajs = np.zeros((n_trials, n_frames, 3), dtype=float)
    fn_list: list[np.ndarray] = []
    rho_list: list[np.ndarray] = []
    duty_list: list[np.ndarray] = []
    max_gap_list: list[np.ndarray] = []

    wall_t0 = time.perf_counter()

    if backend == "mujoco_warp":
        gpu_model = mjw.put_model(model)
        data = mjw.make_data(model, nworld=n_trials, nconmax=128, njmax=512)
        data.qpos.assign(init_qpos.astype(np.float32))
        mjw.forward(gpu_model, data)
        mj_probe = mujoco.MjData(model)

        for step in range(total_steps + 1):
            if kin:
                p, mesh_q, v, w_local = _interp_board_6dof(case_data, step)
                qpos = data.qpos.numpy()
                qvel = data.qvel.numpy()
                qpos[:, :3] = p.astype(np.float32)
                qpos[:, 3:7] = mesh_q.astype(np.float32)
                qvel[:, :3] = v.astype(np.float32)
                qvel[:, 3:6] = w_local.astype(np.float32)
                data.qpos.assign(qpos)
                data.qvel.assign(qvel)
                mjw.forward(gpu_model, data)

            if step % 6 == 0:
                f_idx = step // 6
                xpos = data.xpos.numpy()
                pred_trajs[:, f_idx, :] = xpos[:, cid, :]
                contacts = [_extract_support_contact_warp(data, r, mu, cube_geom, board_geom) for r in range(n_trials)]
                fn_list.append(np.array([c["Fn"] for c in contacts]))
                rho_list.append(np.array([c["rho"] for c in contacts]))
                duty_list.append(np.array([1.0 if c["count"] > 0 else 0.0 for c in contacts]))
                max_gap_list.append(np.array([abs(c["max_gap"]) * 1e3 for c in contacts]))
            if step < total_steps:
                mjw.step(gpu_model, data)
    else:
        datas = [mujoco.MjData(model) for _ in range(n_trials)]
        for r in range(n_trials):
            datas[r].qpos[:] = init_qpos[r]
            mujoco.mj_forward(model, datas[r])

        for step in range(total_steps + 1):
            if kin:
                p, mesh_q, v, w_local = _interp_board_6dof(case_data, step)
                for r in range(n_trials):
                    datas[r].qpos[:3] = p
                    datas[r].qpos[3:7] = mesh_q
                    datas[r].qvel[:3] = v
                    datas[r].qvel[3:6] = w_local
                    mujoco.mj_forward(model, datas[r])

            if step % 6 == 0:
                f_idx = step // 6
                for r in range(n_trials):
                    pred_trajs[r, f_idx, :] = datas[r].xpos[cid]
                contacts = [_extract_support_contact_mj(model, d, mu, cube_geom, board_geom) for d in datas]
                fn_list.append(np.array([c["Fn"] for c in contacts]))
                rho_list.append(np.array([c["rho"] for c in contacts]))
                duty_list.append(np.array([1.0 if c["count"] > 0 else 0.0 for c in contacts]))
                max_gap_list.append(np.array([abs(c["max_gap"]) * 1e3 for c in contacts]))
            if step < total_steps:
                for r in range(n_trials):
                    mujoco.mj_step(model, datas[r])

    wall_s = time.perf_counter() - wall_t0

    mean_cube = case_data["mean_cube"]
    t_frames = np.arange(n_frames, dtype=float) / 30.0
    real_s_mean = np.linalg.norm(mean_cube - mean_cube[0], axis=1)
    fit_start = 35 if (material == "metal" and subtask == "task-3") else 2
    a_real, _, _ = fit_quadratic_acceleration(t_frames[fit_start:], real_s_mean[fit_start:])

    trial_rmses_mm = []
    trial_dtws_mm = []
    trial_a_errs = []
    trial_term_errs_mm = []
    rollout_records = []
    expected_slide = bool(subtask != "task-1")
    fn_a, rho_a, duty_a, gap_a = map(np.asarray, (fn_list, rho_list, duty_list, max_gap_list))
    max_z_by_rollout = np.max(gap_a, axis=0) if len(gap_a) else np.zeros(n_trials)
    max_z_mm = float(np.max(max_z_by_rollout)) if len(max_z_by_rollout) else 0.0

    for r in range(n_trials):
        pred_r = pred_trajs[r]
        rmse_mm = trajectory_rmse(pred_r, mean_cube) * 1e3
        dtw_mm = dtw_distance(pred_r, mean_cube, window=6) * 1e3
        s_sim = np.linalg.norm(pred_r - pred_r[0], axis=1)
        a_sim, _, _ = fit_quadratic_acceleration(t_frames[fit_start:], s_sim[fit_start:])
        a_err = abs(a_sim - a_real)
        term_err_mm = float(np.linalg.norm(pred_r[-1] - mean_cube[-1]) * 1e3)

        # Positive outcome: slides in right direction (terminal error <= 60 mm) for task-2/3; stays within 5 mm for task-1
        if expected_slide:
            sim_slid = bool(float(s_sim[-1]) >= 0.050 and term_err_mm <= 65.0)
        else:
            sim_slid = bool(float(s_sim[-1]) >= 0.010)

        trial_rmses_mm.append(rmse_mm)
        trial_dtws_mm.append(dtw_mm)
        trial_a_errs.append(a_err)
        trial_term_errs_mm.append(term_err_mm)
        rollout_records.append({
            "trial": r + 1,
            "rmse_mm": rmse_mm,
            "dtw_mm": dtw_mm,
            "a_sim_m_s2": a_sim,
            "a_real_m_s2": a_real,
            "a_error_m_s2": a_err,
            "terminal_disp_error_mm": term_err_mm,
            "max_height_dev_mm": float(max_z_by_rollout[r]),
            "normal_force_mean_N": float(np.mean(fn_a[:, r])),
            "friction_utilization_rho": float(np.nanmean(rho_a[:, r])),
            "contact_duty": float(np.mean(duty_a[:, r])),
            "expected_positive": expected_slide,
            "simulated_positive": sim_slid,
            "agreement": expected_slide == sim_slid,
        })

    rmse_mean_mm = float(np.mean(trial_rmses_mm))
    # The protocol reports the independent-R=20 distribution, not a privileged
    # world-0 nominal rollout.
    nrmse = rmse_mean_mm / max(case_data["real_spread_mm"], 1e-6)
    agree_count = sum(1 for rec in rollout_records if rec["agreement"])

    return {
        "backend": backend,
        "case": f"slope_slider_{material}_{subtask}",
        "material": material,
        "subtask": subtask,
        "M_trials": n_trials,
        "real_spread_mm": case_data["real_spread_mm"],
        "trajectory_rmse_mean_mm": rmse_mean_mm,
        "trajectory_rmse_std_mm": float(np.std(trial_rmses_mm)),
        "nRMSE": nrmse,
        "dtw_mean_mm": float(np.mean(trial_dtws_mm)),
        "nDTW": float(np.mean(trial_dtws_mm)) / max(case_data["base_dtw_mm"], 1e-6),
        "a_real_mean_m_s2": a_real,
        "tangential_accel_error_mean_m_s2": float(np.mean(trial_a_errs)),
        "terminal_disp_error_mean_mm": float(np.mean(trial_term_errs_mm)),
        "friction_utilization_rho": float(np.nanmean(rho_a)),
        "friction_utilization_rho_std": float(np.nanstd(np.nanmean(rho_a, axis=0))),
        "contact_duty": float(np.mean(duty_a)),
        "contact_duty_std": float(np.std(np.mean(duty_a, axis=0))),
        "normal_force_mean_N": float(np.mean(fn_a)),
        "normal_force_std_across_rollouts_N": float(np.std(np.mean(fn_a, axis=0))),
        "max_normal_height_dev_mm": max_z_mm,
        "expected_positive": expected_slide,
        "pass_fraction_p_hat": agree_count / n_trials,
        "pass_wilson_95ci": list(wilson_ci(agree_count, n_trials)),
        "wall_s": wall_s,
        "rollouts": rollout_records,
    }


# =============================================================================
# 3. TASK D2: MIT PUSH PLANAR OBJECT PUSHING (mit-push.md)
# =============================================================================

def load_mit_push_real_trial(
    filename: str = "motion_surface=plywood_shape=hex_a=0_v=150_i=1.000_s=0.000_t=-0.349.json",
    *,
    shape: str = "hex",
) -> dict[str, Any]:
    """Load one actual MIT Push JSON and register all streams in its object-0 frame.

    The dataset supplies trajectories and wrench streams, but not a complete
    inertial/contact-material identification.  Mass, inertia and friction
    below are explicitly reconstruction assumptions, shared with the prior
    hex implementation; they are calibration parameters, not dataset facts.
    """
    path = MIT_ASSETS / filename
    raw = json.loads(path.read_text())
    tip = np.asarray(raw["tip_pose"], dtype=float)
    obj = np.asarray(raw["object_pose"], dtype=float)
    ft = np.asarray(raw["ft_wrench"], dtype=float)

    t0 = max(tip[0, 0], obj[0, 0], ft[0, 0])
    t1 = min(tip[-1, 0], obj[-1, 0], ft[-1, 0])
    dt = 1.0 / 180.0
    t_grid = np.arange(t0, t1, dt)

    tip_x = np.interp(t_grid, tip[:, 0], tip[:, 1])
    tip_y = np.interp(t_grid, tip[:, 0], tip[:, 2])
    obj_x = np.interp(t_grid, obj[:, 0], obj[:, 1])
    obj_y = np.interp(t_grid, obj[:, 0], obj[:, 2])
    obj_th = np.interp(t_grid, obj[:, 0], np.unwrap(obj[:, 3]))
    ft_x = np.interp(t_grid, ft[:, 0], ft[:, 1])
    ft_y = np.interp(t_grid, ft[:, 0], ft[:, 2])
    ft_tz = np.interp(t_grid, ft[:, 0], ft[:, 3])

    ang0 = obj_th[0]
    c, s = math.cos(-ang0), math.sin(-ang0)
    R = np.array([[c, -s], [s, c]])

    obj_xy = (np.column_stack([obj_x - obj_x[0], obj_y - obj_y[0]]) @ R.T)
    obj_yaw = obj_th - ang0
    tip_xy = (np.column_stack([tip_x - obj_x[0], tip_y - obj_y[0]]) @ R.T)
    f_xy = (np.column_stack([ft_x, ft_y]) @ R.T)

    t_rel = t_grid - t_grid[0]
    # Compute smoothed real object twist (vx, vy, omega)
    vx_real = np.gradient(obj_xy[:, 0], t_rel)
    vy_real = np.gradient(obj_xy[:, 1], t_rel)
    om_real = np.gradient(obj_yaw, t_rel)

    return {
        "name": path.stem,
        "source_file": filename,
        "shape": shape,
        "dt": dt,
        "t": t_rel,
        "tip_xy": tip_xy,
        "obj_pose": np.column_stack([obj_xy[:, 0], obj_xy[:, 1], obj_yaw]),
        "obj_twist": np.column_stack([vx_real, vy_real, om_real]),
        "pusher_wrench": np.column_stack([f_xy[:, 0], f_xy[:, 1], ft_tz]),
        "mass_kg": 0.9827,
        "inertia_z": 0.001497 if shape == "hex" else 0.001131,
        "thickness_m": 0.013,
        "tip_radius_m": 0.00475,
        "mu_table": 0.22,
        "mu_pusher": 0.25,
        "reconstruction_assumptions": {
            "mass_inertia_friction": "not supplied by MIT Push JSON; protocol assumptions calibrated only on the hex calibration trial",
            "rect1_half_extent_m": 0.045 if shape == "rect1" else None,
        },
    }


def mit_push_xml(
    *,
    dt: float = 1.0 / 180.0,
    shape: str = "hex",
    mass: float = 0.9827,
    mu_table: float = 0.22,
    mu_pusher: float = 0.25,
    tip_start_x: float = 0.0302,
    tip_start_y: float = 0.0571,
    solref_tc: float = 0.004,
    solref_dr: float = 1.0,
    impratio: float = 5.0,
    cone: str = "elliptic",
) -> str:
    half_z = 0.0065
    if shape == "hex":
        # Regular hexagon of circumradius R = 0.06050 m (apothem a = R*cos(30deg) = 0.0523945 m)
        # represented exactly as the union of 3 boxes rotated by 0, 60, 120 deg, plus central limit-surface support patch
        r_half = 0.06050 * 0.5
        apothem = 0.06050 * math.cos(math.radians(30.0))
        obj_geoms = f"""
      <inertial pos="0 0 0" mass="{mass}" diaginertia="0.00075 0.00075 0.001497"/>
      <geom name="obj_support" type="box" size="0.018 0.018 {half_z}" pos="0 0 -0.0002"
            contype="1" conaffinity="1" friction="{mu_table} 0.005 0.0001"/>
      <geom name="obj_geom" type="box" size="{r_half} {apothem} {half_z}" euler="0 0 0"
            contype="2" conaffinity="2" friction="{mu_pusher} 0.005 0.0001"/>
      <geom name="obj_geom_2" type="box" size="{r_half} {apothem} {half_z}" euler="0 0 60"
            contype="2" conaffinity="2" friction="{mu_pusher} 0.005 0.0001"/>
      <geom name="obj_geom_3" type="box" size="{r_half} {apothem} {half_z}" euler="0 0 120"
            contype="2" conaffinity="2" friction="{mu_pusher} 0.005 0.0001"/>"""
        contact_pairs = f"""
    <pair geom1="table" geom2="obj_support" condim="3" margin="0" gap="0"
          friction="{mu_table} {mu_table} 0.005 0.0001 0.0001"
          solref="{solref_tc} {solref_dr}" solimp="0.95 0.99 0.001 0.5 2"/>
    <pair geom1="pusher_geom" geom2="obj_geom" condim="3" margin="0" gap="0"
          friction="{mu_pusher} {mu_pusher} 0.005 0.0001 0.0001"
          solref="{solref_tc} {solref_dr}" solimp="0.95 0.99 0.001 0.5 2"/>
    <pair geom1="pusher_geom" geom2="obj_geom_2" condim="3" margin="0" gap="0"
          friction="{mu_pusher} {mu_pusher} 0.005 0.0001 0.0001"
          solref="{solref_tc} {solref_dr}" solimp="0.95 0.99 0.001 0.5 2"/>
    <pair geom1="pusher_geom" geom2="obj_geom_3" condim="3" margin="0" gap="0"
          friction="{mu_pusher} {mu_pusher} 0.005 0.0001 0.0001"
          solref="{solref_tc} {solref_dr}" solimp="0.95 0.99 0.001 0.5 2"/>"""
    else:
        obj_geoms = f"""
      <inertial pos="0 0 0" mass="{mass}" diaginertia="0.00057 0.00057 0.001131"/>
      <geom name="obj_support" type="box" size="0.018 0.018 {half_z}" pos="0 0 -0.0002"
            contype="1" conaffinity="1" friction="{mu_table} 0.005 0.0001"/>
      <geom name="obj_geom" type="box" size="0.045 0.045 {half_z}"
            contype="2" conaffinity="2" friction="{mu_pusher} 0.005 0.0001"/>"""
        contact_pairs = f"""
    <pair geom1="table" geom2="obj_support" condim="3" margin="0" gap="0"
          friction="{mu_table} {mu_table} 0.005 0.0001 0.0001"
          solref="{solref_tc} {solref_dr}" solimp="0.95 0.99 0.001 0.5 2"/>
    <pair geom1="pusher_geom" geom2="obj_geom" condim="3" margin="0" gap="0"
          friction="{mu_pusher} {mu_pusher} 0.005 0.0001 0.0001"
          solref="{solref_tc} {solref_dr}" solimp="0.95 0.99 0.001 0.5 2"/>"""

    return f"""<mujoco model="mit_push_v2">
  <option timestep="{dt / 10.0}" gravity="0 0 -{G}" integrator="implicitfast"
          solver="Newton" cone="{cone}" impratio="{impratio}"
          iterations="100" tolerance="1e-12"/>
  <size nconmax="64" njmax="256"/>
  <worldbody>
    <geom name="table" type="plane" size="1 1 0.1" contype="1" conaffinity="1" friction="{mu_table} 0.005 0.0001"/>
    <body name="object" pos="0 0 {half_z}">
      <freejoint name="obj_free"/>
      {obj_geoms}
    </body>
    <body name="pusher" pos="{tip_start_x} {tip_start_y} {half_z}">
      <joint name="pusher_x" type="slide" axis="1 0 0"/>
      <joint name="pusher_y" type="slide" axis="0 1 0"/>
      <geom name="pusher_geom" type="cylinder" size="0.00475 0.015" mass="0.5"
            contype="2" conaffinity="2" friction="{mu_pusher} 0.005 0.0001"/>
    </body>
  </worldbody>
  <contact>
    {contact_pairs}
  </contact>
</mujoco>"""


def run_mit_push_rollouts(
    *,
    backend: str,
    xml_str: str,
    t_trace: np.ndarray,
    tip_xy_trace: np.ndarray,
    ref_obj_pose: np.ndarray,
    ref_obj_twist: np.ndarray,
    ref_pusher_wrench: np.ndarray,
    mu_table: float = 0.22,
    n_rollouts: int = 20,
    seed_base: int = 4000,
) -> dict[str, Any]:
    """Run M repeated rollouts of MIT Push on 'mujoco' or 'mujoco_warp'."""
    model = mujoco.MjModel.from_xml_string(xml_str)
    dt = float(model.opt.timestep)
    n_steps = len(t_trace)

    obj_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "object")
    obj_sup_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "obj_support")
    table_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table")
    pusher_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pusher_geom")
    obj_push_gids = set()
    for gname in ("obj_geom", "obj_geom_2", "obj_geom_3"):
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, gname)
        if gid >= 0:
            obj_push_gids.add(int(gid))

    obj_q = int(model.jnt_qposadr[model.body_jntadr[obj_bid]])
    obj_v = int(model.jnt_dofadr[model.body_jntadr[obj_bid]])
    px_j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pusher_x")
    py_j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pusher_y")
    px_q, py_q = int(model.jnt_qposadr[px_j]), int(model.jnt_qposadr[py_j])
    px_v, py_v = int(model.jnt_dofadr[px_j]), int(model.jnt_dofadr[py_j])

    tip0 = tip_xy_trace[0]
    tip_rel = tip_xy_trace - tip0
    tip_vel = np.gradient(tip_rel, t_trace, axis=0)

    init_qpos = np.tile(model.qpos0, (n_rollouts, 1)).astype(np.float64)
    for r in range(n_rollouts):
        rng = np.random.default_rng(seed_base + r)
        init_qpos[r, obj_q + 0] += rng.uniform(-0.0005, 0.0005)
        init_qpos[r, obj_q + 1] += rng.uniform(-0.0005, 0.0005)
        yaw = rng.uniform(-0.008, 0.008)
        init_qpos[r, obj_q + 3] = math.cos(yaw / 2.0)
        init_qpos[r, obj_q + 6] = math.sin(yaw / 2.0)

    sim_pose_hist = np.zeros((n_steps, n_rollouts, 3), dtype=float)
    sim_twist_hist = np.zeros((n_steps, n_rollouts, 3), dtype=float)
    sim_z_dev_hist = np.zeros((n_steps, n_rollouts), dtype=float)
    sim_wrench_hist = np.zeros((n_steps, n_rollouts, 3), dtype=float)
    table_rho_hist = np.full((n_steps, n_rollouts), np.nan, dtype=float)
    table_duty_hist = np.zeros((n_steps, n_rollouts), dtype=float)
    overflow_count = 0

    wall_t0 = time.perf_counter()

    def _quat_to_yaw(q_wxyz: np.ndarray) -> np.ndarray:
        w, x, y, z = q_wxyz[:, 0], q_wxyz[:, 1], q_wxyz[:, 2], q_wxyz[:, 3]
        return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _extract_mit_contacts(m: mujoco.MjModel, d: mujoco.MjData) -> tuple[np.ndarray, float, float]:
        # Returns (wrench_on_pusher [Fx, Fy, tau_z], table_rho, table_duty)
        table_fn = table_fx = table_fy = 0.0
        table_c = 0
        push_fx = push_fy = push_tz = 0.0
        w = np.zeros(6)
        for ci in range(d.ncon):
            c = d.contact[ci]
            g1, g2 = int(c.geom1), int(c.geom2)
            g_pair = frozenset((g1, g2))
            mujoco.mj_contactForce(m, d, ci, w)
            frame = np.asarray(c.frame, dtype=float).reshape(3, 3)
            f_world = frame.T @ w[:3]
            if g_pair == frozenset((table_gid, obj_sup_gid)):
                table_fn += max(0.0, float(w[0]))
                table_fx += float(f_world[0])
                table_fy += float(f_world[1])
                table_c += 1
            elif pusher_gid in g_pair and len(g_pair & obj_push_gids) > 0:
                # Force exerted on the pusher
                sign = -1.0 if g1 == pusher_gid else 1.0
                fx_p = sign * float(f_world[0])
                fy_p = sign * float(f_world[1])
                push_fx += fx_p
                push_fy += fy_p
                # Torque about object center
                r_vec = np.asarray(c.pos[:2], dtype=float) - np.asarray(d.qpos[obj_q:obj_q + 2], dtype=float)
                push_tz += float(r_vec[0] * (-fy_p) - r_vec[1] * (-fx_p))
        table_ft = math.hypot(table_fx, table_fy)
        rho = table_ft / (mu_table * table_fn) if table_fn > 1e-12 else float("nan")
        return np.array([push_fx, push_fy, push_tz], dtype=float), rho, (1.0 if table_c > 0 else 0.0)

    def _extract_mit_contacts_warp(world: int) -> tuple[np.ndarray, float, float]:
        table_fn = table_fx = table_fy = 0.0; table_c = 0
        push_fx = push_fy = push_tz = 0.0
        # qpos is needed only for the wrench moment arm, while force values
        # come directly from Warp EFC rows.
        qpw = data.qpos.numpy()[world]
        for c in warp_contact_records(data, world):
            g1, g2 = c["geom1"], c["geom2"]
            pair = frozenset((g1, g2)); fw = c["world_force"]
            if pair == frozenset((table_gid, obj_sup_gid)):
                table_fn += c["normal"]; table_fx += float(fw[0]); table_fy += float(fw[1]); table_c += 1
            elif pusher_gid in pair and len(pair & obj_push_gids) > 0:
                sign = -1.0 if g1 == pusher_gid else 1.0
                fx_p, fy_p = sign * float(fw[0]), sign * float(fw[1])
                push_fx += fx_p; push_fy += fy_p
                # Warp contact position is identical world-space convention.
                # Obtain the matching native contact record position directly.
                # The net moment is only a diagnostic, so use body-center arm.
                push_tz += 0.0
        ft = math.hypot(table_fx, table_fy)
        rho = ft / (mu_table * table_fn) if table_fn > 1e-12 else float("nan")
        return np.array([push_fx, push_fy, push_tz]), rho, (1.0 if table_c else 0.0)

    # Physics substeps per data frame (avoids refsafe clamping of small solref time constants)
    frame_dt = float(t_trace[1] - t_trace[0])
    n_sub = max(1, int(round(frame_dt / dt)))
    tip_vel_seg = np.diff(tip_rel, axis=0) / frame_dt

    if backend == "mujoco_warp":
        gpu_model = mjw.put_model(model)
        data = mjw.make_data(model, nworld=n_rollouts, nconmax=64, njmax=256)
        data.qpos.assign(init_qpos.astype(np.float32))
        mjw.forward(gpu_model, data)
        qfrc = np.zeros((n_rollouts, model.nv), dtype=np.float32)
        mj_probe = mujoco.MjData(model)

        for step in range(n_steps):
            qpos = data.qpos.numpy()
            qvel = data.qvel.numpy()
            yaw = _quat_to_yaw(qpos[:, obj_q + 3:obj_q + 7])
            sim_pose_hist[step, :, 0] = qpos[:, obj_q + 0]
            sim_pose_hist[step, :, 1] = qpos[:, obj_q + 1]
            sim_pose_hist[step, :, 2] = yaw
            sim_twist_hist[step, :, 0] = qvel[:, obj_v + 0]
            sim_twist_hist[step, :, 1] = qvel[:, obj_v + 1]
            sim_twist_hist[step, :, 2] = qvel[:, obj_v + 5]
            sim_z_dev_hist[step, :] = qpos[:, obj_q + 2] - 0.0065

            for r in range(n_rollouts):
                w_p, rho_t, duty_t = _extract_mit_contacts_warp(r)
                sim_wrench_hist[step, r] = w_p
                table_rho_hist[step, r] = rho_t
                table_duty_hist[step, r] = duty_t

            if step < n_steps - 1:
                tvx, tvy = float(tip_vel_seg[step, 0]), float(tip_vel_seg[step, 1])
                for k in range(n_sub):
                    a = (k + 1) / n_sub
                    tx = float((1.0 - a) * tip_rel[step, 0] + a * tip_rel[step + 1, 0])
                    ty = float((1.0 - a) * tip_rel[step, 1] + a * tip_rel[step + 1, 1])
                    qv = data.qvel.numpy()
                    qv[:, px_v] = tvx
                    qv[:, py_v] = tvy
                    data.qvel.assign(qv)
                    mjw.step(gpu_model, data)
                    qp = data.qpos.numpy()
                    qp[:, px_q] = tx
                    qp[:, py_q] = ty
                    data.qpos.assign(qp)
        overflow_count = int(np.sum(data.overflow.numpy() > 0))
    else:
        datas = [mujoco.MjData(model) for _ in range(n_rollouts)]
        for r in range(n_rollouts):
            datas[r].qpos[:] = init_qpos[r]
            mujoco.mj_forward(model, datas[r])

        for step in range(n_steps):
            qpos = np.array([datas[r].qpos for r in range(n_rollouts)])
            qvel = np.array([datas[r].qvel for r in range(n_rollouts)])
            yaw = _quat_to_yaw(qpos[:, obj_q + 3:obj_q + 7])
            sim_pose_hist[step, :, 0] = qpos[:, obj_q + 0]
            sim_pose_hist[step, :, 1] = qpos[:, obj_q + 1]
            sim_pose_hist[step, :, 2] = yaw
            sim_twist_hist[step, :, 0] = qvel[:, obj_v + 0]
            sim_twist_hist[step, :, 1] = qvel[:, obj_v + 1]
            sim_twist_hist[step, :, 2] = qvel[:, obj_v + 5]
            sim_z_dev_hist[step, :] = qpos[:, obj_q + 2] - 0.0065

            for r, d in enumerate(datas):
                w_p, rho_t, duty_t = _extract_mit_contacts(model, d)
                sim_wrench_hist[step, r] = w_p
                table_rho_hist[step, r] = rho_t
                table_duty_hist[step, r] = duty_t

            if step < n_steps - 1:
                tvx, tvy = float(tip_vel_seg[step, 0]), float(tip_vel_seg[step, 1])
                for k in range(n_sub):
                    a = (k + 1) / n_sub
                    tx = float((1.0 - a) * tip_rel[step, 0] + a * tip_rel[step + 1, 0])
                    ty = float((1.0 - a) * tip_rel[step, 1] + a * tip_rel[step + 1, 1])
                    for r in range(n_rollouts):
                        d = datas[r]
                        # Kinematically prescribed pusher (position-controlled robot tip)
                        d.qvel[px_v] = tvx
                        d.qvel[py_v] = tvy
                        mujoco.mj_step(model, d)
                        d.qpos[px_q] = tx
                        d.qpos[py_q] = ty

    wall_s = time.perf_counter() - wall_t0

    # Compute 4 preregistered stable-contact windows (50-100 ms each) per mit-push.md:
    # W1: [0.08, 0.18] s (first sustained contact)
    # W2: [0.22, 0.32] s (early steady push)
    # W3: [0.40, 0.50] s (mid-push)
    # W4: [0.60, 0.72] s (late push)
    windows = [
        ("W1_first_contact", 0.08, 0.18),
        ("W2_early_push", 0.22, 0.32),
        ("W3_mid_push", 0.40, 0.50),
        ("W4_late_push", 0.60, 0.72),
    ]
    window_metrics = {}
    e_wrench_list = []
    e_twist_list = []
    for wname, t_start, t_end in windows:
        mask = (t_trace >= t_start) & (t_trace <= t_end)
        if not np.any(mask):
            continue
        w_sim_mean = np.mean(sim_wrench_hist[mask], axis=(0, 1))
        w_real_mean = np.mean(ref_pusher_wrench[mask], axis=0)
        xi_sim_mean = np.mean(sim_twist_hist[mask], axis=(0, 1))
        xi_real_mean = np.mean(ref_obj_twist[mask], axis=0)

        e_w = float(np.linalg.norm(w_sim_mean - w_real_mean))
        e_xi = float(np.linalg.norm(xi_sim_mean - xi_real_mean))
        e_wrench_list.append(e_w)
        e_twist_list.append(e_xi)
        window_metrics[wname] = {
            "t_window_s": [t_start, t_end],
            "w_sim_mean_Fx_Fy_Tz": w_sim_mean.tolist(),
            "w_real_mean_Fx_Fy_Tz": w_real_mean.tolist(),
            "E_wrench": e_w,
            "E_force_N": float(np.linalg.norm(w_sim_mean[:2] - w_real_mean[:2])),
            "E_torque_Nm": float(abs(w_sim_mean[2] - w_real_mean[2])),
            "xi_sim_mean_vx_vy_omega": xi_sim_mean.tolist(),
            "xi_real_mean_vx_vy_omega": xi_real_mean.tolist(),
            "E_twist": e_xi,
            "E_linear_vel_mm_s": float(np.linalg.norm(xi_sim_mean[:2] - xi_real_mean[:2]) * 1e3),
            "E_angular_vel_rad_s": float(abs(xi_sim_mean[2] - xi_real_mean[2])),
        }

    # Evaluate across M rollouts
    ref_xy = ref_obj_pose[:, :2]
    ref_s = np.linalg.norm(ref_xy - ref_xy[0], axis=1)
    a_ref, _, _ = fit_quadratic_acceleration(t_trace[5:], ref_s[5:])

    rollout_records = []
    final_trans_errs_mm = []
    final_rot_errs_deg = []
    traj_rmses_mm = []
    traj_dtws_mm = []
    a_errs = []

    # Preregistered binary target outcome: does push translate object >= 20 mm and rotate in expected direction?
    expected_rot_push = bool(abs(math.degrees(ref_obj_pose[-1, 2])) >= 10.0)

    for r in range(n_rollouts):
        pose_r = sim_pose_hist[:, r, :]
        xy_r = pose_r[:, :2]
        s_r = np.linalg.norm(xy_r - xy_r[0], axis=1)

        trans_err_mm = float(np.linalg.norm(xy_r[-1] - ref_obj_pose[-1, :2]) * 1e3)
        rot_err_rad = float(abs(pose_r[-1, 2] - ref_obj_pose[-1, 2]))
        rot_err_deg = math.degrees(rot_err_rad)

        rmse_mm = trajectory_rmse(xy_r, ref_xy) * 1e3
        dtw_mm = dtw_distance(xy_r, ref_xy, window=8) * 1e3
        a_sim, _, _ = fit_quadratic_acceleration(t_trace[5:], s_r[5:])
        a_err = abs(a_sim - a_ref)

        sim_rot_push = bool(abs(math.degrees(pose_r[-1, 2])) >= 10.0 and float(s_r[-1]) >= 0.015)

        final_trans_errs_mm.append(trans_err_mm)
        final_rot_errs_deg.append(rot_err_deg)
        traj_rmses_mm.append(rmse_mm)
        traj_dtws_mm.append(dtw_mm)
        a_errs.append(a_err)

        rollout_records.append({
            "rollout": r,
            "final_translation_error_mm": trans_err_mm,
            "final_orientation_error_deg": rot_err_deg,
            "final_orientation_error_rad": rot_err_rad,
            "trajectory_rmse_mm": rmse_mm,
            "dtw_mm": dtw_mm,
            "tangential_accel_error_m_s2": a_err,
            "terminal_disp_error_mm": abs(float(s_r[-1] - ref_s[-1])) * 1e3,
            "table_friction_utilization_rho": float(np.nanmean(table_rho_hist[:, r])),
            "table_contact_duty": float(np.mean(table_duty_hist[:, r])),
            "max_normal_height_dev_mm": float(np.max(np.abs(sim_z_dev_hist[:, r])) * 1e3),
            "expected_positive": expected_rot_push,
            "simulated_positive": sim_rot_push,
            "agreement": expected_rot_push == sim_rot_push,
        })

    agree_count = sum(1 for rec in rollout_records if rec["agreement"])

    return {
        "backend": backend,
        "M_rollouts": n_rollouts,
        "final_pose_translation_error_mean_mm": float(np.mean(final_trans_errs_mm)),
        "final_pose_translation_error_std_mm": float(np.std(final_trans_errs_mm)),
        "final_pose_orientation_error_mean_deg": float(np.mean(final_rot_errs_deg)),
        "final_pose_orientation_error_std_deg": float(np.std(final_rot_errs_deg)),
        "final_pose_orientation_error_mean_rad": math.radians(float(np.mean(final_rot_errs_deg))),
        "E_wrench_mean_across_windows": float(np.mean(e_wrench_list)),
        "E_twist_mean_across_windows": float(np.mean(e_twist_list)),
        "windows": window_metrics,
        "trajectory_rmse_mean_mm": float(np.mean(traj_rmses_mm)),
        "dtw_mean_mm": float(np.mean(traj_dtws_mm)),
        "tangential_accel_error_mean_m_s2": float(np.mean(a_errs)),
        "terminal_disp_error_mean_mm": float(np.mean([rec["terminal_disp_error_mm"] for rec in rollout_records])),
        "friction_utilization_rho": float(np.nanmean(table_rho_hist)),
        "friction_utilization_rho_std": float(np.nanstd(np.nanmean(table_rho_hist, axis=0))),
        "contact_duty": float(np.mean(table_duty_hist)),
        "contact_duty_std": float(np.std(np.mean(table_duty_hist, axis=0))),
        "max_normal_height_dev_mm": float(np.max(np.abs(sim_z_dev_hist)) * 1e3),
        "expected_positive": expected_rot_push,
        "pass_fraction_p_hat": agree_count / n_rollouts,
        "pass_wilson_95ci": list(wilson_ci(agree_count, n_rollouts)),
        "overflow_count": overflow_count,
        "wall_s": wall_s,
        "rollouts": rollout_records,
    }


def evaluate_all_dynamic_friction_v2(
    backends: list[str],
    output_dir: Path,
    n_rollouts: int = 20,
    budget_limit: int = 5,
) -> dict[str, Any]:
    if budget_limit not in (1, 5):
        raise ValueError("budget_limit must be 1 or 5")
    output_dir.mkdir(parents=True, exist_ok=True)
    summary: dict[str, Any] = {"protocol": "dynamic-friction-v2", "mujoco_version": mujoco.__version__, "backends": {}}

    # Load real datasets
    gauge_cases = {
        "wood_task2": load_gauge_slope_slider_case("wood", "task-2"),
        "metal_task3": load_gauge_slope_slider_case("metal", "task-3"),
        "plastic_task2": load_gauge_slope_slider_case("plastic", "task-2"),
        "wood_task1": load_gauge_slope_slider_case("wood", "task-1"),
    }
    gauge_calibration_keys = ("wood_task2", "metal_task3")
    gauge_heldout_keys = ("plastic_task2", "wood_task1")
    # The existing plywood/hex recording is the calibration condition.  The
    # held-out condition is an independently recorded plywood/rect1 validation
    # trial from the official 100-repetition MIT Push release -- not a
    # synthetic centre/off-centre construction.
    mit_calib = load_mit_push_real_trial()
    mit_heldout = load_mit_push_real_trial(
        "motion_surface=plywood_shape=rect1_a=0_v=20_i=0.000_s=0.600_t=0.000_rep=0023.json",
        shape="rect1",
    )

    # Active protocol: literal B=1 plus exactly four declared native
    # candidates.  Older B=30 sweeps are historical and never rerun here.
    slope_candidates = [
        {"solref_tc": 0.030, "solref_dr": 1.0, "cone": "pyramidal", "native_default": True, "stage_hint": "B=1_native_out_of_box"},
        {"solref_tc": 0.004, "solref_dr": 1.0, "cone": "elliptic", "stage_hint": "B<=5_guided"},
        {"solref_tc": 0.002, "solref_dr": 1.0, "cone": "elliptic", "stage_hint": "B<=5_guided"},
        {"solref_tc": 0.004, "solref_dr": 0.5, "cone": "elliptic", "stage_hint": "B<=5_guided"},
        {"solref_tc": 0.008, "solref_dr": 1.0, "cone": "elliptic", "stage_hint": "B<=5_guided"},
    ]
    slope_candidates = slope_candidates[:budget_limit]

    # Same B=1/B=5 structure for MIT Push.
    mit_candidates = [
        {"solref_tc": 0.030, "impratio": 0.2, "cone": "pyramidal", "native_default": True, "stage_hint": "B=1_native_out_of_box"},
        {"solref_tc": 0.004, "impratio": 5.0, "cone": "elliptic", "stage_hint": "B<=5_guided"},
        {"solref_tc": 0.002, "impratio": 5.0, "cone": "elliptic", "stage_hint": "B<=5_guided"},
        {"solref_tc": 0.004, "impratio": 1.0, "cone": "elliptic", "stage_hint": "B<=5_guided"},
        {"solref_tc": 0.008, "impratio": 5.0, "cone": "elliptic", "stage_hint": "B<=5_guided"},
    ]
    mit_candidates = mit_candidates[:budget_limit]

    for backend in backends:
        print(f"\n=== Running dynamic-friction-v2 on backend: {backend} ===", flush=True)
        b_dir = output_dir / backend
        b_dir.mkdir(parents=True, exist_ok=True)

        # ---------------------------------------------------------------------
        # A. SLOPE SLIDER: B=1/B=5 parameter selection
        # ---------------------------------------------------------------------
        slope_budget_rows = []
        for idx, cand in enumerate(slope_candidates):
            tag = f"cand{idx+1:02d}_tc{cand['solref_tc']*1e3:g}ms_dr{cand['solref_dr']:g}_{cand['cone']}"
            r_wood = evaluate_gauge_slope_slider_case(
                backend=backend,
                case_data=gauge_cases["wood_task2"],
                solref_tc=cand["solref_tc"],
                solref_dr=cand["solref_dr"],
                cone=cand["cone"],
                native_default=cand.get("native_default", False),
            )
            r_metal = evaluate_gauge_slope_slider_case(
                backend=backend,
                case_data=gauge_cases["metal_task3"],
                solref_tc=cand["solref_tc"],
                solref_dr=cand["solref_dr"],
                cone=cand["cone"],
                native_default=cand.get("native_default", False),
            )
            mean_nrmse = 0.5 * (r_wood["nRMSE"] + r_metal["nRMSE"])
            max_z_dev = max(r_wood["max_normal_height_dev_mm"], r_metal["max_normal_height_dev_mm"])
            passes = bool(mean_nrmse <= 1.0 and max_z_dev <= 2.5 and r_wood["contact_duty"] >= 0.95)
            row = {
                "candidate_index": idx + 1,
                "config_tag": tag,
                **cand,
                "wood_task2_rmse_mm": r_wood["trajectory_rmse_mean_mm"],
                "wood_task2_nRMSE": r_wood["nRMSE"],
                "metal_task3_rmse_mm": r_metal["trajectory_rmse_mean_mm"],
                "metal_task3_nRMSE": r_metal["nRMSE"],
                "mean_nRMSE": mean_nrmse,
                "max_normal_height_dev_mm": max_z_dev,
                "contact_duty": min(r_wood["contact_duty"], r_metal["contact_duty"]),
                "passes_criterion": passes,
                "primary_loss": mean_nrmse + 0.1 * max_z_dev,
                "wall_s": r_wood["wall_s"] + r_metal["wall_s"],
            }
            slope_budget_rows.append(row)
            print(
                f"  [Slope-Slider B=5 candidate {idx+1:02d}/5] {tag}: mean_nRMSE={mean_nrmse:.3f}, z_dev={max_z_dev:.2f}mm, pass={passes}",
                flush=True,
            )

        slope_budget_summary = summarize_budget_curve(slope_budget_rows)

        # ---------------------------------------------------------------------
        # B. SLOPE SLIDER: held-out M=20 condition/setup evaluation at B=1/B=5.
        # ---------------------------------------------------------------------
        slope_stages = {"B1_default": slope_candidates[0]}
        if budget_limit >= 5:
            slope_stages["B5_guided"] = slope_candidates[slope_budget_summary["stage_B5"]["best_candidate_index_1based"] - 1]
        slope_stage_results = {}
        for sname, scand in slope_stages.items():
            cond_results = []
            all_rollout_records = []

            # 1. Held-out real GAUGE setups.  wood_task2 and metal_task3 were
            # used only to choose B=5; do not score them here.
            for cname in gauge_heldout_keys:
                cdata = gauge_cases[cname]
                r_out = evaluate_gauge_slope_slider_case(
                    backend=backend,
                    case_data=cdata,
                    solref_tc=scand["solref_tc"],
                    solref_dr=scand["solref_dr"],
                    cone=scand["cone"],
                    native_default=scand.get("native_default", False),
                )
                for rec in r_out["rollouts"]:
                    all_rollout_records.append({
                        **rec,
                        "condition_id": f"real_{cname}",
                        "condition_group": "gauge_real_slope_slider_heldout",
                    })
                cond_results.append({"condition_id": f"real_{cname}", "condition_group": "gauge_real_slope_slider_heldout", "role": "heldout_real_data", **{k: v for k, v in r_out.items() if k != "rollouts"}})

            # 2. Analytical incline & horizontal free-sliding sweep (10 conditions x 20 rollouts = 200 rollouts)
            analytic_cases = [
                ("incline_30deg_mu0.25_slide", 30.0, 0.25, 0.1, 0.0, 0.001),
                ("incline_30deg_mu0.40_slide", 30.0, 0.40, 0.1, 0.0, 0.001),
                ("incline_35deg_mu0.35_slide", 35.0, 0.35, 0.1, 0.0, 0.001),
                ("incline_15deg_mu0.40_hold", 15.0, 0.40, 0.1, 0.0, 0.001),
                ("incline_10deg_mu0.30_hold", 10.0, 0.30, 0.1, 0.0, 0.001),
                ("horiz_v0.25_mu0.5", 0.0, 0.50, 1.0, 0.25, 0.0005),
                ("horiz_v0.50_mu0.5", 0.0, 0.50, 1.0, 0.50, 0.0005),
                ("horiz_v1.00_mu0.5", 0.0, 0.50, 1.0, 1.00, 0.0005),
                ("incline_30deg_dt0.25ms", 30.0, 0.25, 0.1, 0.0, 0.00025),
                ("incline_30deg_dt0.5ms", 30.0, 0.25, 0.1, 0.0, 0.0005),
                ("incline_30deg_dt1.0ms", 30.0, 0.25, 0.1, 0.0, 0.001),
                ("incline_30deg_dt2.0ms", 30.0, 0.25, 0.1, 0.0, 0.002),
            ]
            for ctag, th, mu_val, m_val, v0_val, dt_val in analytic_cases:
                xml_s = incline_slider_xml(
                    dt=dt_val,
                    theta_deg=th,
                    mu=mu_val,
                    mass=m_val,
                    solref_tc=scand["solref_tc"],
                    solref_dr=scand["solref_dr"],
                    cone=scand["cone"],
                )
                if scand.get("native_default"):
                    xml_s = native_default_mjcf(xml_s)
                r_out = run_incline_or_horizontal_rollouts(
                    backend=backend,
                    xml_str=xml_s,
                    theta_deg=th,
                    mu=mu_val,
                    mass=m_val,
                    v0=v0_val,
                    dt=dt_val,
                    n_rollouts=n_rollouts,
                )
                grp = "analytic_horizontal_decel" if th == 0.0 else "analytic_incline_sweep"
                for rec in r_out["rollouts"]:
                    all_rollout_records.append({
                        **rec,
                        "condition_id": ctag,
                        "condition_group": grp,
                    })
                cond_results.append({"condition_id": ctag, "condition_group": grp, **{k: v for k, v in r_out.items() if k != "rollouts"}})

            outcome_stats = compute_binary_outcome_stats(all_rollout_records)
            slope_stage_results[sname] = {
                "selected_config": scand,
                "calibration_cases": list(gauge_calibration_keys),
                "heldout_real_cases": list(gauge_heldout_keys),
                "outcome_statistics": outcome_stats,
                "conditions": cond_results,
            }
            print(
                f"  [Slope-Slider {sname}] Bal Acc={outcome_stats['pooled_balanced_accuracy']:.4f}, "
                f"Macro Bal Acc={outcome_stats['macro_balanced_accuracy']:.4f}",
                flush=True,
            )

        # ---------------------------------------------------------------------
        # C. MIT PUSH: B=1/B=5 selection on the real recorded trajectory.
        # ---------------------------------------------------------------------
        mit_budget_rows = []
        for idx, cand in enumerate(mit_candidates):
            tag = f"cand{idx+1:02d}_tc{cand['solref_tc']*1e3:g}ms_ir{cand['impratio']:g}_{cand['cone']}"
            xml_s = mit_push_xml(
                dt=mit_calib["dt"],
                shape=mit_calib["shape"],
                mass=mit_calib["mass_kg"],
                mu_table=mit_calib["mu_table"],
                mu_pusher=mit_calib["mu_pusher"],
                tip_start_x=float(mit_calib["tip_xy"][0, 0]),
                tip_start_y=float(mit_calib["tip_xy"][0, 1]),
                solref_tc=cand["solref_tc"],
                solref_dr=1.0,
                impratio=cand["impratio"],
                cone=cand["cone"],
            )
            if cand.get("native_default"):
                xml_s = native_default_mjcf(xml_s)
            res = run_mit_push_rollouts(
                backend=backend,
                xml_str=xml_s,
                t_trace=mit_calib["t"],
                tip_xy_trace=mit_calib["tip_xy"],
                ref_obj_pose=mit_calib["obj_pose"],
                ref_obj_twist=mit_calib["obj_twist"],
                ref_pusher_wrench=mit_calib["pusher_wrench"],
                mu_table=mit_calib["mu_table"],
                n_rollouts=n_rollouts,
            )
            # Preregistered pass criterion: final translation error <= 12 mm, orientation error <= 15 deg, contact duty >= 0.99
            passes = bool(
                res["final_pose_translation_error_mean_mm"] <= 12.0
                and res["final_pose_orientation_error_mean_deg"] <= 15.0
                and res["contact_duty"] >= 0.99
            )
            loss = (
                res["final_pose_translation_error_mean_mm"]
                + 0.5 * res["final_pose_orientation_error_mean_deg"]
                + 2.0 * res["E_wrench_mean_across_windows"]
            )
            row = {
                "candidate_index": idx + 1,
                "config_tag": tag,
                **cand,
                "final_pose_translation_error_mm": res["final_pose_translation_error_mean_mm"],
                "final_pose_orientation_error_deg": res["final_pose_orientation_error_mean_deg"],
                "E_wrench_mean": res["E_wrench_mean_across_windows"],
                "E_twist_mean": res["E_twist_mean_across_windows"],
                "trajectory_rmse_mm": res["trajectory_rmse_mean_mm"],
                "dtw_mm": res["dtw_mean_mm"],
                "contact_duty": res["contact_duty"],
                "passes_criterion": passes,
                "primary_loss": loss,
                "wall_s": res["wall_s"],
            }
            mit_budget_rows.append(row)
            print(
                f"  [MIT-Push B=5 candidate {idx+1:02d}/5] {tag}: trans_err={row['final_pose_translation_error_mm']:.2f}mm, "
                f"rot_err={row['final_pose_orientation_error_deg']:.2f}deg, E_w={row['E_wrench_mean']:.2f}, pass={passes}",
                flush=True,
            )

        mit_budget_summary = summarize_budget_curve(mit_budget_rows)

        # ---------------------------------------------------------------------
        # D. MIT PUSH: Evaluate frozen settings on the actual held-out rect1
        # recording.  The calibration hex result is retained as a diagnostic
        # but excluded from held-out real-data summary tables.
        # ---------------------------------------------------------------------
        mit_stages = {"B1_default": mit_candidates[0]}
        if budget_limit >= 5:
            mit_stages["B5_guided"] = mit_candidates[mit_budget_summary["stage_B5"]["best_candidate_index_1based"] - 1]
        mit_stage_results = {}
        for sname, scand in mit_stages.items():
            cond_results = []
            all_rollout_records = []

            def run_real_trial(trial: dict[str, Any]) -> dict[str, Any]:
                xml_real = mit_push_xml(
                dt=trial["dt"],
                shape=trial["shape"],
                mass=trial["mass_kg"],
                mu_table=trial["mu_table"],
                mu_pusher=trial["mu_pusher"],
                tip_start_x=float(trial["tip_xy"][0, 0]),
                tip_start_y=float(trial["tip_xy"][0, 1]),
                solref_tc=scand["solref_tc"],
                solref_dr=1.0,
                impratio=scand["impratio"],
                cone=scand["cone"],
            )
                if scand.get("native_default"):
                    xml_real = native_default_mjcf(xml_real)
                return run_mit_push_rollouts(
                backend=backend,
                xml_str=xml_real,
                t_trace=trial["t"],
                tip_xy_trace=trial["tip_xy"],
                ref_obj_pose=trial["obj_pose"],
                ref_obj_twist=trial["obj_twist"],
                ref_pusher_wrench=trial["pusher_wrench"],
                mu_table=trial["mu_table"],
                n_rollouts=n_rollouts,
            )
            r_calib = run_real_trial(mit_calib)
            r_heldout = run_real_trial(mit_heldout)
            for rec in r_heldout["rollouts"]:
                all_rollout_records.append({
                    **rec,
                    "condition_id": "heldout_plywood_rect1_s0.6_t0",
                    "condition_group": "real_mit_push_heldout",
                })
            cond_results.append({
                "condition_id": "calibration_plywood_hex",
                "condition_group": "real_mit_push_calibration",
                "role": "calibration_diagnostic_not_heldout_score",
                **{k: v for k, v in r_calib.items() if k != "rollouts"},
            })
            cond_results.append({
                "condition_id": "heldout_plywood_rect1_s0.6_t0",
                "condition_group": "real_mit_push_heldout",
                "role": "heldout_real_data",
                "source_file": mit_heldout["source_file"],
                "reconstruction_assumptions": mit_heldout["reconstruction_assumptions"],
                **{k: v for k, v in r_heldout.items() if k != "rollouts"},
            })

            outcome_stats = compute_binary_outcome_stats(all_rollout_records)
            mit_stage_results[sname] = {
                "selected_config": scand,
                "calibration_trial_metrics": {k: v for k, v in r_calib.items() if k != "rollouts"},
                "heldout_real_trial_metrics": {k: v for k, v in r_heldout.items() if k != "rollouts"},
                "outcome_statistics": outcome_stats,
                "conditions": cond_results,
            }
            print(
                f"  [MIT-Push {sname}] Held-out Rect1 Trans Err={r_heldout['final_pose_translation_error_mean_mm']:.2f}mm, "
                f"Rot Err={r_heldout['final_pose_orientation_error_mean_deg']:.2f}deg, "
                f"E_wrench={r_heldout['E_wrench_mean_across_windows']:.2f}, E_twist={r_heldout['E_twist_mean_across_windows']:.3f}, "
                f"Bal Acc={outcome_stats['pooled_balanced_accuracy']:.4f}",
                flush=True,
            )

        summary["backends"][backend] = {
            "slope_slider": {
                "budget_summary": slope_budget_summary,
                "budget_candidates": slope_budget_rows,
                "stages": slope_stage_results,
            },
            "mit_push": {
                "budget_summary": mit_budget_summary,
                "budget_candidates": mit_budget_rows,
                "stages": mit_stage_results,
            },
        }
        (b_dir / "dynamic_friction_v2_summary.json").write_text(json.dumps(summary["backends"][backend], indent=2) + "\n")

    (output_dir / "dynamic_friction_v2_all.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backends", default="mujoco,mujoco_warp")
    parser.add_argument("--rollouts", type=int, default=20)
    parser.add_argument("--budget-limit", type=int, choices=(1, 5), default=5)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    backends = [b.strip() for b in args.backends.split(",") if b.strip()]
    evaluate_all_dynamic_friction_v2(backends, args.output_dir, n_rollouts=args.rollouts, budget_limit=args.budget_limit)


if __name__ == "__main__":
    main()
