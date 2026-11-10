"""Complete runner for static-friction-v2 protocol on MuJoCo 3.17.0 and MuJoCo Warp 3.17.0.

Covers:
  1. Task S1: Three-Box Push (sub-Coulomb hold + middle-box retain/eject)
  2. Task S2: Two-Finger Simple Pinch (minimum hold grip force F*_grip, E_grip, E_z(T), retain/release)
  3. Primary budget: B = 1 native default and B = 5 mechanism-guided;
     B = 30 is an optional historical upper-bound sweep.
  4. Repeated rollouts (M = 20 perturbed rollouts per physical condition)
  5. Plausible-range controllability, numerical local robustness (dt), and setup-variant sweeps
"""
from __future__ import annotations

import argparse
import csv
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
    fit_linear_slope,
    mjw,
    spearman_rank_corr,
    summarize_budget_curve,
    warp_contact_records,
    wilson_ci,
)
from native_defaults import native_default_mjcf

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = ROOT / "exp-results" / "protocol-v2" / "static-friction-v2"
G = 9.81


# =============================================================================
# 1. THREE-BOX PUSH & SUB-COULOMB HOLD (Task S1)
# =============================================================================

def three_box_xml(
    *,
    dt: float = 0.0005,
    mass: float = 1.0,
    mu: float = 0.5,
    k_pair: float = 1e4,
    impratio: float = 1.0,
    solreffriction_tc: float = 0.0,
    noslip_iterations: int = 0,
    box_half: float = 0.02,
) -> str:
    d = 0.9
    s_ground = (1.0 - d) * k_pair / mass
    s_internal = 2.0 * s_ground
    b_ground = 2.0 * math.sqrt(s_ground)
    b_internal = 2.0 * math.sqrt(s_internal)
    srf_attr = f'solreffriction="{solreffriction_tc} 1"' if solreffriction_tc > 0 else ""
    noslip_attr = f'noslip_iterations="{noslip_iterations}"' if noslip_iterations > 0 else ""
    bodies = "".join(
        f'<body name="box_{i}" pos="0 0 {box_half + 2 * box_half * i}">'
        f'<freejoint name="j_box_{i}"/><geom name="box_{i}_geom" type="box" '
        f'size="{box_half} {box_half} {box_half}" mass="{mass}"/></body>'
        for i in range(3)
    )
    pair_common = (
        f'condim="3" margin="0" gap="0" friction="{mu} {mu} 0.005 0.0001 0.0001" '
        f'solimp="{d} {d} 0.001 0.5 2" {srf_attr}'
    )
    return f"""<mujoco model="three_box_push_v2">
  <option timestep="{dt}" gravity="0 0 -{G}" integrator="implicitfast"
          solver="Newton" cone="elliptic" impratio="{impratio}"
          iterations="100" tolerance="1e-12" {noslip_attr}/>
  <size nconmax="128" njmax="512"/>
  <default><geom friction="{mu} 0.005 0.0001" margin="0" gap="0"/></default>
  <worldbody>
    <geom name="ground" type="plane" size="2 2 0.1"/>
    {bodies}
  </worldbody>
  <contact>
    <pair geom1="ground" geom2="box_0_geom" {pair_common} solref="{-s_ground} {-b_ground}"/>
    <pair geom1="box_0_geom" geom2="box_1_geom" {pair_common} solref="{-s_internal} {-b_internal}"/>
    <pair geom1="box_1_geom" geom2="box_2_geom" {pair_common} solref="{-s_internal} {-b_internal}"/>
  </contact>
</mujoco>"""


def _extract_three_box_contacts_mj(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    geom_ids: list[int],
    ground_id: int,
    mu: float,
) -> dict[str, dict[str, float]]:
    pair_map = {
        frozenset((ground_id, geom_ids[0])): "ground_bottom",
        frozenset((geom_ids[0], geom_ids[1])): "bottom_middle",
        frozenset((geom_ids[1], geom_ids[2])): "middle_top",
    }
    res = {k: {"Fn": 0.0, "Fx": 0.0, "Fy": 0.0, "count": 0} for k in pair_map.values()}
    wrench = np.zeros(6)
    for ci in range(data.ncon):
        c = data.contact[ci]
        name = pair_map.get(frozenset((int(c.geom1), int(c.geom2))))
        if name is None:
            continue
        mujoco.mj_contactForce(model, data, ci, wrench)
        frame = np.asarray(c.frame, dtype=float).reshape(3, 3)
        f_world = frame.T @ wrench[:3]
        fn = max(0.0, float(wrench[0]))
        res[name]["Fn"] += fn
        res[name]["Fx"] += float(f_world[0])
        res[name]["Fy"] += float(f_world[1])
        res[name]["count"] += 1
    for name, v in res.items():
        ft = math.hypot(v["Fx"], v["Fy"])
        v["Ft"] = ft
        v["rho"] = ft / (mu * v["Fn"]) if v["Fn"] > 1e-12 else float("nan")
    return res


def _extract_three_box_contacts_warp(data: Any, world: int, geom_ids: list[int], ground_id: int, mu: float) -> dict[str, dict[str, float]]:
    pair_map = {frozenset((ground_id, geom_ids[0])): "ground_bottom",
                frozenset((geom_ids[0], geom_ids[1])): "bottom_middle",
                frozenset((geom_ids[1], geom_ids[2])): "middle_top"}
    res = {k: {"Fn": 0.0, "Fx": 0.0, "Fy": 0.0, "count": 0} for k in pair_map.values()}
    for c in warp_contact_records(data, world):
        name = pair_map.get(frozenset((c["geom1"], c["geom2"])))
        if name is None:
            continue
        res[name]["Fn"] += c["normal"]
        res[name]["Fx"] += float(c["world_force"][0])
        res[name]["Fy"] += float(c["world_force"][1])
        res[name]["count"] += 1
    for v in res.values():
        v["Ft"] = math.hypot(v["Fx"], v["Fy"])
        v["rho"] = v["Ft"] / (mu * v["Fn"]) if v["Fn"] > 1e-12 else float("nan")
    return res


def run_three_box_rollouts(
    *,
    backend: str,
    xml_str: str,
    pushed_body_index: int,
    force_target_N: float,
    mu: float,
    mass: float,
    dt: float,
    n_rollouts: int = 20,
    settle_s: float = 0.6,
    ramp_s: float = 0.4,
    hold_s: float = 2.0,
    tail_s: float = 0.8,
    ejection_threshold_m: float = 0.005,
    seed_base: int = 1000,
) -> dict[str, Any]:
    """Run M repeated rollouts of Three-Box Push on either 'mujoco' or 'mujoco_warp'."""
    model = mujoco.MjModel.from_xml_string(xml_str)
    bodies = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"box_{i}") for i in range(3)]
    geoms = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"box_{i}_geom") for i in range(3)]
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    qaddrs = [int(model.jnt_qposadr[model.body_jntadr[b]]) for b in bodies]
    vaddrs = [int(model.jnt_dofadr[model.body_jntadr[b]]) for b in bodies]

    total_s = settle_s + ramp_s + hold_s
    total_steps = round(total_s / dt)
    sample_every = max(1, round(0.01 / dt))

    # Generate R independent initial perturbations. A nominal run, when useful,
    # is saved separately and is never folded into the R=20 distribution.
    init_qpos = np.tile(model.qpos0, (n_rollouts, 1)).astype(np.float64)
    force_scales = np.ones(n_rollouts, dtype=np.float64)
    for r in range(n_rollouts):
        rng = np.random.default_rng(seed_base + r)
        for b_idx in range(3):
            qa = qaddrs[b_idx]
            init_qpos[r, qa + 0] += rng.uniform(-0.0004, 0.0004)
            init_qpos[r, qa + 1] += rng.uniform(-0.0004, 0.0004)
            yaw = rng.uniform(-0.005, 0.005)
            init_qpos[r, qa + 3] = math.cos(yaw / 2.0)
            init_qpos[r, qa + 6] = math.sin(yaw / 2.0)
        force_scales[r] = 1.0 + rng.uniform(-0.005, 0.005)

    # Ideal breakaway threshold for the pushed box:
    # - If top box (box_2) is pushed: middle_top normal load is m*g, threshold = mu * m * g.
    # - If middle box (box_1) is pushed while top box is restrained or co-moving:
    #   bottom_middle normal load is 2*m*g; if box_2 moves with box_1, breakaway at bottom_middle is 2*mu*m*g.
    if pushed_body_index == 2:
        ideal_breakaway_N = mu * mass * G
        active_interface = "middle_top"
        expected_fn = mass * G
    else:
        ideal_breakaway_N = 2.0 * mu * mass * G
        active_interface = "bottom_middle"
        expected_fn = 2.0 * mass * G

    expected_eject = bool(force_target_N > ideal_breakaway_N)

    t_samples: list[float] = []
    # Shape: (n_samples, n_rollouts)
    rel_x_hist: list[np.ndarray] = []
    rel_vt_hist: list[np.ndarray] = []
    vz_hist: list[np.ndarray] = []
    gap_hist: list[np.ndarray] = []
    fn_hist: list[np.ndarray] = []
    ft_hist: list[np.ndarray] = []
    rho_hist: list[np.ndarray] = []
    duty_hist: list[np.ndarray] = []
    overflow_count = 0

    wall_t0 = time.perf_counter()

    if backend == "mujoco_warp":
        if not HAS_MJWARP:
            raise RuntimeError("mujoco_warp is not installed in this Python environment")
        gpu_model = mjw.put_model(model)
        data = mjw.make_data(model, nworld=n_rollouts, nconmax=128, njmax=512)
        data.qpos.assign(init_qpos.astype(np.float32))
        mjw.forward(gpu_model, data)
        applied = np.zeros((n_rollouts, model.nbody, 6), dtype=np.float32)
        mj_probe = mujoco.MjData(model)

        baseline_rel_x: np.ndarray | None = None
        for step in range(total_steps):
            t0 = step * dt
            factor = float(np.clip((t0 - settle_s) / ramp_s, 0.0, 1.0))
            applied[:, :, :] = 0.0
            applied[:, bodies[pushed_body_index], 0] = (force_target_N * force_scales * factor).astype(np.float32)
            data.xfrc_applied.assign(applied)
            mjw.step(gpu_model, data)

            t = (step + 1) * dt
            if (step + 1) % sample_every == 0:
                qpos = data.qpos.numpy()
                qvel = data.qvel.numpy()
                if pushed_body_index == 2:
                    rx = qpos[:, qaddrs[2]] - qpos[:, qaddrs[1]]
                    rv = qvel[:, vaddrs[2]] - qvel[:, vaddrs[1]]
                    gap = qpos[:, qaddrs[2] + 2] - qpos[:, qaddrs[1] + 2] - 0.04
                    vz = qvel[:, vaddrs[2] + 2]
                else:
                    rx = qpos[:, qaddrs[1]] - qpos[:, qaddrs[0]]
                    rv = qvel[:, vaddrs[1]] - qvel[:, vaddrs[0]]
                    gap = qpos[:, qaddrs[1] + 2] - qpos[:, qaddrs[0] + 2] - 0.04
                    vz = qvel[:, vaddrs[1] + 2]

                if baseline_rel_x is None and t >= settle_s:
                    baseline_rel_x = rx.copy()
                if t >= settle_s and baseline_rel_x is not None:
                    t_samples.append(t)
                    rel_x_hist.append(rx - baseline_rel_x)
                    rel_vt_hist.append(rv)
                    vz_hist.append(vz)
                    gap_hist.append(gap)

                    contacts = [_extract_three_box_contacts_warp(data, r, geoms, ground, mu)[active_interface]
                                for r in range(n_rollouts)]
                    fn_hist.append(np.array([c["Fn"] for c in contacts]))
                    ft_hist.append(np.array([c["Ft"] for c in contacts]))
                    rho_hist.append(np.array([c["rho"] for c in contacts]))
                    duty_hist.append(np.array([1.0 if c["count"] > 0 else 0.0 for c in contacts]))
        overflow_arr = data.overflow.numpy()
        overflow_count = int(np.sum(overflow_arr > 0))
    else:
        # Standalone CPU MuJoCo 3.17.0
        datas = [mujoco.MjData(model) for _ in range(n_rollouts)]
        for r in range(n_rollouts):
            datas[r].qpos[:] = init_qpos[r]
            mujoco.mj_forward(model, datas[r])

        baseline_rel_x = None
        for step in range(total_steps):
            t0 = step * dt
            factor = float(np.clip((t0 - settle_s) / ramp_s, 0.0, 1.0))
            for r in range(n_rollouts):
                d = datas[r]
                d.xfrc_applied[:, :] = 0.0
                d.xfrc_applied[bodies[pushed_body_index], 0] = force_target_N * force_scales[r] * factor
                mujoco.mj_step(model, d)

            t = (step + 1) * dt
            if (step + 1) % sample_every == 0:
                rx = np.zeros(n_rollouts)
                rv = np.zeros(n_rollouts)
                vz = np.zeros(n_rollouts)
                gap = np.zeros(n_rollouts)
                fns = np.zeros(n_rollouts)
                fts = np.zeros(n_rollouts)
                rhos = np.zeros(n_rollouts)
                duties = np.zeros(n_rollouts)

                for r in range(n_rollouts):
                    d = datas[r]
                    if pushed_body_index == 2:
                        rx[r] = float(d.qpos[qaddrs[2]] - d.qpos[qaddrs[1]])
                        rv[r] = float(d.qvel[vaddrs[2]] - d.qvel[vaddrs[1]])
                        gap[r] = float(d.qpos[qaddrs[2] + 2] - d.qpos[qaddrs[1] + 2] - 0.04)
                        vz[r] = float(d.qvel[vaddrs[2] + 2])
                    else:
                        rx[r] = float(d.qpos[qaddrs[1]] - d.qpos[qaddrs[0]])
                        rv[r] = float(d.qvel[vaddrs[1]] - d.qvel[vaddrs[0]])
                        gap[r] = float(d.qpos[qaddrs[1] + 2] - d.qpos[qaddrs[0] + 2] - 0.04)
                        vz[r] = float(d.qvel[vaddrs[1] + 2])
                    cinfo = _extract_three_box_contacts_mj(model, d, geoms, ground, mu)[active_interface]
                    fns[r] = cinfo["Fn"]
                    fts[r] = cinfo["Ft"]
                    rhos[r] = cinfo["rho"]
                    duties[r] = 1.0 if cinfo["count"] > 0 else 0.0

                if baseline_rel_x is None and t >= settle_s:
                    baseline_rel_x = rx.copy()
                if t >= settle_s and baseline_rel_x is not None:
                    t_samples.append(t)
                    rel_x_hist.append(rx - baseline_rel_x)
                    rel_vt_hist.append(rv)
                    vz_hist.append(vz)
                    gap_hist.append(gap)
                    fn_hist.append(fns)
                    ft_hist.append(fts)
                    rho_hist.append(rhos)
                    duty_hist.append(duties)

    wall_s = time.perf_counter() - wall_t0

    t_arr = np.asarray(t_samples, dtype=float)
    rel_x_arr = np.asarray(rel_x_hist, dtype=float)  # (T, M)
    rel_vt_arr = np.asarray(rel_vt_hist, dtype=float)
    vz_arr = np.asarray(vz_hist, dtype=float)
    gap_arr = np.asarray(gap_hist, dtype=float)
    fn_arr = np.asarray(fn_hist, dtype=float)
    rho_arr = np.asarray(rho_hist, dtype=float)
    duty_arr = np.asarray(duty_hist, dtype=float)

    tail_mask = t_arr >= (total_s - tail_s)
    t_tail = t_arr[tail_mask]

    rollout_records = []
    creeps_um_s = []
    tail_disps_mm = []
    tail_vt_rms_mm_s = []

    for r in range(n_rollouts):
        x_tail = rel_x_arr[tail_mask, r]
        vt_tail = rel_vt_arr[tail_mask, r]
        final_disp_m = float(abs(x_tail[-1]))
        slope_m_s = abs(fit_linear_slope(t_tail, x_tail))
        vt_rms = float(np.sqrt(np.mean(vt_tail ** 2)))
        sim_eject = bool(final_disp_m >= ejection_threshold_m)

        creeps_um_s.append(slope_m_s * 1e6)
        tail_disps_mm.append(final_disp_m * 1e3)
        tail_vt_rms_mm_s.append(vt_rms * 1e3)
        rollout_records.append({
            "rollout": r,
            "final_rel_disp_mm": final_disp_m * 1e3,
            "tail_creep_um_s": slope_m_s * 1e6,
            "tail_vt_rms_mm_s": vt_rms * 1e3,
            "Fn_tail_mean_N": float(np.mean(fn_arr[tail_mask, r])),
            "rho_tail_mean": float(np.nanmean(rho_arr[tail_mask, r])),
            "contact_duty_tail": float(np.mean(duty_arr[tail_mask, r])),
            "gap_tail_std_mm": float(np.std(gap_arr[tail_mask, r]) * 1e3),
            "vz_tail_rms_mm_s": float(np.sqrt(np.mean(vz_arr[tail_mask, r] ** 2)) * 1e3),
            "expected_positive": expected_eject,
            "simulated_positive": sim_eject,
            "agreement": expected_eject == sim_eject,
        })

    fn_tail = fn_arr[tail_mask]
    fn_by_rollout = np.mean(fn_tail, axis=0)
    fn_mean = float(np.mean(fn_by_rollout))
    fn_cv = float(np.std(fn_by_rollout) / fn_mean) if fn_mean > 1e-12 else float("inf")
    fn_rel_err = abs(fn_mean - expected_fn) / expected_fn
    rho_tail = rho_arr[tail_mask]
    duty_tail = float(np.mean(duty_arr[tail_mask]))
    gap_std_mm = float(np.mean(np.std(gap_arr[tail_mask], axis=0)) * 1e3)
    vz_rms_mm_s = float(np.mean(np.sqrt(np.mean(vz_arr[tail_mask] ** 2, axis=0))) * 1e3)

    agree_count = sum(1 for rec in rollout_records if rec["agreement"])
    return {
        "backend": backend,
        "pushed_body": f"box_{pushed_body_index}",
        "active_interface": active_interface,
        "force_target_N": force_target_N,
        "ideal_breakaway_N": ideal_breakaway_N,
        "mu": mu,
        "mass_kg": mass,
        "dt_ms": dt * 1e3,
        "M_rollouts": n_rollouts,
        "expected_outcome": "eject" if expected_eject else "retain",
        "expected_positive": expected_eject,
        "pass_fraction_p_hat": agree_count / n_rollouts,
        "pass_wilson_95ci": list(wilson_ci(agree_count, n_rollouts)),
        "tail_rel_disp_mean_mm": float(np.mean(tail_disps_mm)),
        "tail_rel_disp_std_mm": float(np.std(tail_disps_mm)),
        "tail_creep_mean_um_s": float(np.mean(creeps_um_s)),
        "tail_creep_std_um_s": float(np.std(creeps_um_s)),
        "tail_vt_rms_mean_mm_s": float(np.mean(tail_vt_rms_mm_s)),
        "friction_utilization_rho": float(np.nanmean(rho_tail)),
        "friction_utilization_rho_std": float(np.nanstd(np.nanmean(rho_tail, axis=0))),
        "contact_duty": duty_tail,
        "contact_duty_std": float(np.std(np.mean(duty_arr[tail_mask], axis=0))),
        "Fn_mean_N": fn_mean,
        "Fn_rel_error": fn_rel_err,
        "Fn_cv": fn_cv,
        "gap_std_mm": gap_std_mm,
        "vz_rms_mm_s": vz_rms_mm_s,
        "overflow_count": overflow_count,
        "wall_s": wall_s,
        "rollouts": rollout_records,
    }


# =============================================================================
# 2. TWO-FINGER SIMPLE PINCH (Task S2)
# =============================================================================

def simple_pinch_xml(
    *,
    dt: float = 0.0005,
    mu: float = 0.4,
    mass: float = 0.25,
    compression_m: float = 0.0002,
    solref_tc: float = 0.004,
    impratio: float = 1.0,
    noslip_iterations: int = 0,
) -> str:
    x = 0.030 - compression_m
    noslip_attr = f'noslip_iterations="{noslip_iterations}"' if noslip_iterations > 0 else ""
    return f"""<mujoco model="simple_pinch_v2_protocol">
  <option timestep="{dt}" gravity="0 0 0" integrator="implicitfast"
          solver="Newton" cone="elliptic" impratio="{impratio}"
          iterations="100" tolerance="1e-12" {noslip_attr}/>
  <size nconmax="64" njmax="256"/>
  <worldbody>
    <body name="left" pos="{-x} 0 0">
      <joint name="left_x" type="slide" axis="1 0 0"/>
      <joint name="left_z" type="slide" axis="0 0 1"/>
      <geom name="left_pad" type="box" size="0.01 0.04 0.04" mass="1" friction="{mu} 0.005 0.0001"/>
    </body>
    <body name="right" pos="{x} 0 0">
      <joint name="right_x" type="slide" axis="1 0 0"/>
      <joint name="right_z" type="slide" axis="0 0 1"/>
      <geom name="right_pad" type="box" size="0.01 0.04 0.04" mass="1" friction="{mu} 0.005 0.0001"/>
    </body>
    <body name="cube" pos="0 0 0">
      <freejoint name="cube_free"/>
      <geom name="cube_geom" type="box" size="0.02 0.02 0.02" mass="{mass}" friction="{mu} 0.005 0.0001"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="left_pad" geom2="cube_geom" condim="3" margin="0" gap="0"
          friction="{mu} {mu} 0.005 0.0001 0.0001" solref="{solref_tc} 1" solimp="0.95 0.99 0.001 0.5 2"/>
    <pair geom1="right_pad" geom2="cube_geom" condim="3" margin="0" gap="0"
          friction="{mu} {mu} 0.005 0.0001 0.0001" solref="{solref_tc} 1" solimp="0.95 0.99 0.001 0.5 2"/>
  </contact>
</mujoco>"""


def _extract_pinch_contacts_mj(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    left_id: int,
    right_id: int,
    cube_id: int,
    mu: float,
) -> dict[str, dict[str, float]]:
    out = {s: {"Fn": 0.0, "Ft": 0.0, "count": 0} for s in ("left", "right")}
    w = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        other = c.geom2 if c.geom1 == cube_id else (c.geom1 if c.geom2 == cube_id else -1)
        side = "left" if other == left_id else ("right" if other == right_id else None)
        if side is None:
            continue
        mujoco.mj_contactForce(model, data, i, w)
        out[side]["Fn"] += max(0.0, float(w[0]))
        out[side]["Ft"] += float(math.hypot(w[1], w[2]))
        out[side]["count"] += 1
    for s in ("left", "right"):
        fn = out[s]["Fn"]
        out[s]["rho"] = out[s]["Ft"] / (mu * fn) if fn > 1e-12 else float("nan")
    return out


def _extract_pinch_contacts_warp(data: Any, world: int, left_id: int, right_id: int, cube_id: int, mu: float) -> dict[str, dict[str, float]]:
    out = {s: {"Fn": 0.0, "Ft": 0.0, "count": 0} for s in ("left", "right")}
    for c in warp_contact_records(data, world):
        other = c["geom2"] if c["geom1"] == cube_id else (c["geom1"] if c["geom2"] == cube_id else -1)
        side = "left" if other == left_id else ("right" if other == right_id else None)
        if side:
            out[side]["Fn"] += c["normal"]
            out[side]["Ft"] += float(np.linalg.norm(c["tangent"]))
            out[side]["count"] += 1
    for v in out.values():
        v["rho"] = v["Ft"] / (mu * v["Fn"]) if v["Fn"] > 1e-12 else float("nan")
    return out


def run_simple_pinch_rollouts(
    *,
    backend: str,
    xml_str: str,
    mu: float,
    mass: float,
    compression_m: float,
    dt: float = 0.0005,
    n_rollouts: int = 20,
    gate_s: float = 0.8,
    lift_s: float = 1.0,
    hold_s: float = 0.25,
    lift_height_m: float = 0.060,
    seed_base: int = 2000,
) -> dict[str, Any]:
    """Run M repeated rollouts of Simple Pinch on 'mujoco' or 'mujoco_warp'."""
    model = mujoco.MjModel.from_xml_string(xml_str)
    left_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_pad")
    right_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_pad")
    cube_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    cube_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_q = int(model.jnt_qposadr[model.body_jntadr[cube_bid]])
    cube_v = int(model.jnt_dofadr[model.body_jntadr[cube_bid]])

    j_names = ("left_x", "left_z", "right_x", "right_z")
    j_q = {n: int(model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in j_names}
    j_v = {n: int(model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in j_names}

    init_qpos = np.tile(model.qpos0, (n_rollouts, 1)).astype(np.float64)
    load_scales = np.ones(n_rollouts, dtype=np.float64)
    for r in range(n_rollouts):
        rng = np.random.default_rng(seed_base + r)
        init_qpos[r, cube_q + 1] += rng.uniform(-0.0004, 0.0004)
        init_qpos[r, cube_q + 2] += rng.uniform(-0.0004, 0.0004)
        pitch = rng.uniform(-0.004, 0.004)
        init_qpos[r, cube_q + 3] = math.cos(pitch / 2.0)
        init_qpos[r, cube_q + 5] = math.sin(pitch / 2.0)
        load_scales[r] = 1.0 + rng.uniform(-0.005, 0.005)

    gate_steps = round(gate_s / dt)
    lift_hold_steps = round((lift_s + hold_s) / dt)
    lift_v = lift_height_m / lift_s
    sample_every = max(1, round(0.01 / dt))

    gate_fn_l: list[np.ndarray] = []
    gate_fn_r: list[np.ndarray] = []
    lift_fn_l: list[np.ndarray] = []
    lift_fn_r: list[np.ndarray] = []
    lift_rho_l: list[np.ndarray] = []
    lift_rho_r: list[np.ndarray] = []
    lift_duty_l: list[np.ndarray] = []
    lift_duty_r: list[np.ndarray] = []

    t_lift_samples: list[float] = []
    rel_z_hist: list[np.ndarray] = []
    rel_vt_hist: list[np.ndarray] = []
    cube_z_hist: list[np.ndarray] = []
    overflow_count = 0

    wall_t0 = time.perf_counter()

    if backend == "mujoco_warp":
        gpu_model = mjw.put_model(model)
        data = mjw.make_data(model, nworld=n_rollouts, nconmax=64, njmax=256)
        data.qpos.assign(init_qpos.astype(np.float32))
        mjw.forward(gpu_model, data)
        qfrc = np.zeros((n_rollouts, model.nv), dtype=np.float32)
        xfrc = np.zeros((n_rollouts, model.nbody, 6), dtype=np.float32)

        # Gate phase
        for step in range(gate_steps):
            qpos = data.qpos.numpy()
            qvel = data.qvel.numpy()
            qfrc[:, :] = 0.0
            xfrc[:, :, :] = 0.0
            for n in ("left_x", "right_x", "left_z", "right_z"):
                qfrc[:, j_v[n]] = 20000.0 * (0.0 - qpos[:, j_q[n]]) + 300.0 * (0.0 - qvel[:, j_v[n]])
            data.qfrc_applied.assign(qfrc)
            data.xfrc_applied.assign(xfrc)
            mjw.step(gpu_model, data)
            if step >= gate_steps - round(0.2 / dt) and (step % sample_every == 0):
                fs = [_extract_pinch_contacts_warp(data, r, left_gid, right_gid, cube_gid, mu)
                      for r in range(n_rollouts)]
                gate_fn_l.append(np.array([f["left"]["Fn"] for f in fs]))
                gate_fn_r.append(np.array([f["right"]["Fn"] for f in fs]))

        z0_cube = data.qpos.numpy()[:, cube_q + 2].copy()
        # Lift + hold phase
        for step in range(lift_hold_steps + 1):
            t = step * dt
            active = t <= lift_s
            target_z = lift_height_m * min(1.0, t / lift_s)
            target_vz = lift_v if active else 0.0
            qpos = data.qpos.numpy()
            qvel = data.qvel.numpy()
            qfrc[:, :] = 0.0
            xfrc[:, :, :] = 0.0
            for n in ("left_x", "right_x"):
                qfrc[:, j_v[n]] = 20000.0 * (0.0 - qpos[:, j_q[n]]) + 300.0 * (0.0 - qvel[:, j_v[n]])
            for n in ("left_z", "right_z"):
                qfrc[:, j_v[n]] = 20000.0 * (target_z - qpos[:, j_q[n]]) + 300.0 * (target_vz - qvel[:, j_v[n]])
            xfrc[:, cube_bid, 2] = (-mass * G * load_scales).astype(np.float32)
            data.qfrc_applied.assign(qfrc)
            data.xfrc_applied.assign(xfrc)
            if step < lift_hold_steps:
                mjw.step(gpu_model, data)

            if step % sample_every == 0:
                qpos = data.qpos.numpy()
                qvel = data.qvel.numpy()
                pad_z = 0.5 * (qpos[:, j_q["left_z"]] + qpos[:, j_q["right_z"]])
                pad_vz = 0.5 * (qvel[:, j_v["left_z"]] + qvel[:, j_v["right_z"]])
                cz = qpos[:, cube_q + 2] - z0_cube
                cvz = qvel[:, cube_v + 2]
                t_lift_samples.append(t)
                cube_z_hist.append(cz)
                rel_z_hist.append(cz - pad_z)
                rel_vt_hist.append(cvz - pad_vz)

                fs = [_extract_pinch_contacts_warp(data, r, left_gid, right_gid, cube_gid, mu)
                      for r in range(n_rollouts)]
                lift_fn_l.append(np.array([f["left"]["Fn"] for f in fs]))
                lift_fn_r.append(np.array([f["right"]["Fn"] for f in fs]))
                lift_rho_l.append(np.array([f["left"]["rho"] for f in fs]))
                lift_rho_r.append(np.array([f["right"]["rho"] for f in fs]))
                lift_duty_l.append(np.array([1.0 if f["left"]["count"] > 0 else 0.0 for f in fs]))
                lift_duty_r.append(np.array([1.0 if f["right"]["count"] > 0 else 0.0 for f in fs]))
        overflow_count = int(np.sum(data.overflow.numpy() > 0))
    else:
        datas = [mujoco.MjData(model) for _ in range(n_rollouts)]
        for r in range(n_rollouts):
            datas[r].qpos[:] = init_qpos[r]
            mujoco.mj_forward(model, datas[r])

        for step in range(gate_steps):
            for r in range(n_rollouts):
                d = datas[r]
                d.qfrc_applied[:] = 0.0
                d.xfrc_applied[:, :] = 0.0
                for n in ("left_x", "right_x", "left_z", "right_z"):
                    d.qfrc_applied[j_v[n]] = 20000.0 * (0.0 - d.qpos[j_q[n]]) + 300.0 * (0.0 - d.qvel[j_v[n]])
                mujoco.mj_step(model, d)
            if step >= gate_steps - round(0.2 / dt) and (step % sample_every == 0):
                fs = [_extract_pinch_contacts_mj(model, d, left_gid, right_gid, cube_gid, mu) for d in datas]
                gate_fn_l.append(np.array([f["left"]["Fn"] for f in fs]))
                gate_fn_r.append(np.array([f["right"]["Fn"] for f in fs]))

        z0_cube = np.array([float(datas[r].qpos[cube_q + 2]) for r in range(n_rollouts)])
        for step in range(lift_hold_steps + 1):
            t = step * dt
            active = t <= lift_s
            target_z = lift_height_m * min(1.0, t / lift_s)
            target_vz = lift_v if active else 0.0
            for r in range(n_rollouts):
                d = datas[r]
                d.qfrc_applied[:] = 0.0
                d.xfrc_applied[:, :] = 0.0
                for n in ("left_x", "right_x"):
                    d.qfrc_applied[j_v[n]] = 20000.0 * (0.0 - d.qpos[j_q[n]]) + 300.0 * (0.0 - d.qvel[j_v[n]])
                for n in ("left_z", "right_z"):
                    d.qfrc_applied[j_v[n]] = 20000.0 * (target_z - d.qpos[j_q[n]]) + 300.0 * (target_vz - d.qvel[j_v[n]])
                d.xfrc_applied[cube_bid, 2] = -mass * G * load_scales[r]
                if step < lift_hold_steps:
                    mujoco.mj_step(model, d)

            if step % sample_every == 0:
                cz = np.array([float(datas[r].qpos[cube_q + 2]) - z0_cube[r] for r in range(n_rollouts)])
                cvz = np.array([float(datas[r].qvel[cube_v + 2]) for r in range(n_rollouts)])
                pad_z = np.array([0.5 * float(datas[r].qpos[j_q["left_z"]] + datas[r].qpos[j_q["right_z"]]) for r in range(n_rollouts)])
                pad_vz = np.array([0.5 * float(datas[r].qvel[j_v["left_z"]] + datas[r].qvel[j_v["right_z"]]) for r in range(n_rollouts)])
                t_lift_samples.append(t)
                cube_z_hist.append(cz)
                rel_z_hist.append(cz - pad_z)
                rel_vt_hist.append(cvz - pad_vz)

                fs = [_extract_pinch_contacts_mj(model, d, left_gid, right_gid, cube_gid, mu) for d in datas]
                lift_fn_l.append(np.array([f["left"]["Fn"] for f in fs]))
                lift_fn_r.append(np.array([f["right"]["Fn"] for f in fs]))
                lift_rho_l.append(np.array([f["left"]["rho"] for f in fs]))
                lift_rho_r.append(np.array([f["right"]["rho"] for f in fs]))
                lift_duty_l.append(np.array([1.0 if f["left"]["count"] > 0 else 0.0 for f in fs]))
                lift_duty_r.append(np.array([1.0 if f["right"]["count"] > 0 else 0.0 for f in fs]))

    wall_s = time.perf_counter() - wall_t0

    gate_fn_l_a, gate_fn_r_a = np.asarray(gate_fn_l), np.asarray(gate_fn_r)
    lift_fn_l_a, lift_fn_r_a = np.asarray(lift_fn_l), np.asarray(lift_fn_r)
    lift_rho_l_a, lift_rho_r_a = np.asarray(lift_rho_l), np.asarray(lift_rho_r)
    lift_duty_l_a, lift_duty_r_a = np.asarray(lift_duty_l), np.asarray(lift_duty_r)
    nl_preload = float(np.mean(gate_fn_l_a))
    nr_preload = float(np.mean(gate_fn_r_a))
    n_preload = 0.5 * (nl_preload + nr_preload)
    r_margin = (2.0 * mu * n_preload) / (mass * G)
    f_grip_ideal = (mass * G) / (2.0 * mu)

    # Positive class in simple-pinch.md is release/slip (r < 1.0)
    expected_release = bool(r_margin < 1.0)

    t_arr = np.asarray(t_lift_samples, dtype=float)
    cz_arr = np.asarray(cube_z_hist, dtype=float)
    rel_z_arr = np.asarray(rel_z_hist, dtype=float)
    rel_vt_arr = np.asarray(rel_vt_hist, dtype=float)
    hold_mask = t_arr >= lift_s

    rollout_records = []
    ez_mm_list = []
    creep_um_s_list = []
    for r in range(n_rollouts):
        final_z_m = float(cz_arr[-1, r])
        ez_m = abs(final_z_m - lift_height_m)
        hold_creep_m_s = abs(fit_linear_slope(t_arr[hold_mask], rel_z_arr[hold_mask, r])) if np.sum(hold_mask) >= 2 else 0.0
        # Task gate: retains cube if final lift follow >= 57 mm (E_z <= 3 mm)
        sim_release = bool(final_z_m < 0.057)
        ez_mm_list.append(ez_m * 1e3)
        creep_um_s_list.append(hold_creep_m_s * 1e6)
        rollout_records.append({
            "rollout": r,
            "lift_follow_mm": final_z_m * 1e3,
            "E_z_mm": ez_m * 1e3,
            "E_z_normalized": ez_m / lift_height_m,
            "hold_creep_um_s": hold_creep_m_s * 1e6,
            "N_left_preload_N": float(np.mean(gate_fn_l_a[:, r])),
            "N_right_preload_N": float(np.mean(gate_fn_r_a[:, r])),
            "left_rho_lift": float(np.nanmean(lift_rho_l_a[:, r])),
            "right_rho_lift": float(np.nanmean(lift_rho_r_a[:, r])),
            "left_contact_duty": float(np.mean(lift_duty_l_a[:, r])),
            "right_contact_duty": float(np.mean(lift_duty_r_a[:, r])),
            "expected_positive": expected_release,
            "simulated_positive": sim_release,
            "agreement": expected_release == sim_release,
        })

    agree_count = sum(1 for rec in rollout_records if rec["agreement"])
    retain_count = sum(1 for rec in rollout_records if not rec["simulated_positive"])

    return {
        "backend": backend,
        "mu": mu,
        "mass_kg": mass,
        "compression_mm": compression_m * 1e3,
        "dt_ms": dt * 1e3,
        "M_rollouts": n_rollouts,
        "N_left_preload_N": nl_preload,
        "N_right_preload_N": nr_preload,
        "N_mean_preload_N": n_preload,
        "F_grip_ideal_N": f_grip_ideal,
        "grasp_margin_r": r_margin,
        "expected_outcome": "release" if expected_release else "retain",
        "expected_positive": expected_release,
        "retain_fraction": retain_count / n_rollouts,
        "pass_fraction_p_hat": agree_count / n_rollouts,
        "pass_wilson_95ci": list(wilson_ci(agree_count, n_rollouts)),
        "E_z_mean_mm": float(np.mean(ez_mm_list)),
        "E_z_std_mm": float(np.std(ez_mm_list)),
        "E_z_normalized_mean": float(np.mean(ez_mm_list) / (lift_height_m * 1e3)),
        "hold_creep_mean_um_s": float(np.mean(creep_um_s_list)),
        "N_mean_preload_std_across_rollouts_N": float(np.std(0.5 * (np.mean(gate_fn_l_a, axis=0) + np.mean(gate_fn_r_a, axis=0)))),
        "left_Fn_lift_mean_N": float(np.mean(lift_fn_l_a)),
        "right_Fn_lift_mean_N": float(np.mean(lift_fn_r_a)),
        "left_rho_mean": float(np.nanmean(lift_rho_l_a)),
        "right_rho_mean": float(np.nanmean(lift_rho_r_a)),
        "left_duty": float(np.mean(lift_duty_l_a)),
        "right_duty": float(np.mean(lift_duty_r_a)),
        "left_duty_std": float(np.std(np.mean(lift_duty_l_a, axis=0))),
        "right_duty_std": float(np.std(np.mean(lift_duty_r_a, axis=0))),
        "overflow_count": overflow_count,
        "wall_s": wall_s,
        "rollouts": rollout_records,
    }


def evaluate_all_static_friction_v2(
    backends: list[str],
    output_dir: Path,
    n_rollouts: int = 20,
    budget_limit: int = 5,
    reuse_b1_summary: Path | None = None,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    summary: dict[str, Any] = {"protocol": "static-friction-v2", "mujoco_version": mujoco.__version__, "backends": {}}

    # The active protocol has exactly five candidates: literal B=1 plus four
    # declared mechanism-guided candidates.  It intentionally has no B>5 arm.
    grid_candidates = [
        {"impratio": 1.0, "solreffriction_tc": 0.0, "noslip": 0, "native_default": True, "stage_hint": "B=1_native_out_of_box"},
        {"impratio": 100.0, "solreffriction_tc": 0.0, "noslip": 0, "stage_hint": "B=5_guided"},
        {"impratio": 100.0, "solreffriction_tc": 0.0, "noslip": 2, "stage_hint": "B<=5_guided"},
        {"impratio": 10.0, "solreffriction_tc": 0.004, "noslip": 0, "stage_hint": "B<=5_guided"},
        {"impratio": 10.0, "solreffriction_tc": 0.004, "noslip": 2, "stage_hint": "B<=5_guided"},
    ]
    if budget_limit not in (1, 5):
        raise ValueError("budget_limit must be 1 or 5")
    grid_candidates = grid_candidates[:budget_limit]

    for backend in backends:
        print(f"\n=== Running static-friction-v2 on backend: {backend} ===", flush=True)
        b_dir = output_dir / backend
        b_dir.mkdir(parents=True, exist_ok=True)
        prior = None
        if reuse_b1_summary is not None:
            prior = json.loads(reuse_b1_summary.read_text())
            for task in ("three_box_push", "simple_pinch"):
                old_stage = prior[task]["stages"]["B1_default"]
                if not old_stage["selected_config"].get("native_default"):
                    raise ValueError(f"Cannot reuse non-native B1 for {task}")
                if any(c.get("M_rollouts") != n_rollouts for c in old_stage["conditions"]):
                    raise ValueError(f"Cannot reuse B1 with different rollout count for {task}")

        # ---------------------------------------------------------------------
        # A. THREE-BOX PUSH: B=1/B=5 selection on a calibration-only
        # 3.5-N top-box sub-Coulomb hold.  This exact force is excluded from
        # every later evaluation condition.
        # ---------------------------------------------------------------------
        three_box_budget_rows = []
        for idx, cand in enumerate(grid_candidates):
            if idx == 0 and prior is not None:
                three_box_budget_rows.append(prior["three_box_push"]["budget_candidates"][0])
                continue
            tag = f"cand{idx+1:02d}_ir{cand['impratio']:g}_srf{cand['solreffriction_tc']*1e3:g}ms_ns{cand['noslip']}"
            xml_s = three_box_xml(
                dt=0.0005,
                mass=1.0,
                mu=0.5,
                k_pair=1e4,
                impratio=cand["impratio"],
                solreffriction_tc=cand["solreffriction_tc"],
                noslip_iterations=cand["noslip"],
            )
            if cand.get("native_default"):
                xml_s = native_default_mjcf(xml_s)
            res = run_three_box_rollouts(
                backend=backend,
                xml_str=xml_s,
                pushed_body_index=2,
                force_target_N=3.5,
                mu=0.5,
                mass=1.0,
                dt=0.0005,
                n_rollouts=n_rollouts,
            )
            # Preregistered sticking pass criterion: contact_duty >= 0.99 and tail creep <= 10 um/s
            passes = bool(res["contact_duty"] >= 0.99 and res["tail_creep_mean_um_s"] <= 10.0)
            row = {
                "candidate_index": idx + 1,
                "config_tag": tag,
                **cand,
                "tail_creep_mean_um_s": res["tail_creep_mean_um_s"],
                "tail_rel_disp_mean_mm": res["tail_rel_disp_mean_mm"],
                "contact_duty": res["contact_duty"],
                "friction_utilization_rho": res["friction_utilization_rho"],
                "passes_criterion": passes,
                "primary_loss": res["tail_creep_mean_um_s"] + (1e6 if res["contact_duty"] < 0.99 else 0.0),
                "wall_s": res["wall_s"],
            }
            three_box_budget_rows.append(row)
            print(
                f"  [Three-Box B=5 candidate {idx+1:02d}/5] {tag}: creep={row['tail_creep_mean_um_s']:.3f} um/s, pass={passes}",
                flush=True,
            )

        tb_budget_summary = summarize_budget_curve(three_box_budget_rows)
        # Monotonicity along impratio (with srf=0, ns=0)
        ir_sub = [r for r in three_box_budget_rows if r["solreffriction_tc"] == 0.0 and r["noslip"] == 0]
        ir_sub.sort(key=lambda x: x["impratio"])
        tb_budget_summary["impratio_monotonicity_spearman"] = abs(
            spearman_rank_corr([r["impratio"] for r in ir_sub], [r["tail_creep_mean_um_s"] for r in ir_sub])
        )

        # ---------------------------------------------------------------------
        # B. THREE-BOX PUSH: held-out M=20 condition/setup evaluation at B=1/B=5.
        # ---------------------------------------------------------------------
        tb_stages = {"B1_default": grid_candidates[0]}
        if budget_limit >= 5:
            tb_stages["B5_guided"] = grid_candidates[tb_budget_summary["stage_B5"]["best_candidate_index_1based"] - 1]
        tb_stage_results = {}
        for sname, scand in tb_stages.items():
            if sname == "B1_default" and prior is not None:
                tb_stage_results[sname] = prior["three_box_push"]["stages"][sname]
                continue
            cond_results = []
            all_rollout_records = []

            # Group 1: Middle-box push force sweep (retain vs eject, ideal breakaway = 2*mu*m*g = 9.81 N)
            for f_val in (2.0, 4.0, 6.0, 8.0, 11.0, 13.0, 15.0, 18.0):
                xml_s = three_box_xml(
                    dt=0.0005,
                    mass=1.0,
                    mu=0.5,
                    impratio=scand["impratio"],
                    solreffriction_tc=scand["solreffriction_tc"],
                    noslip_iterations=scand["noslip"],
                )
                if scand.get("native_default"):
                    xml_s = native_default_mjcf(xml_s)
                r_out = run_three_box_rollouts(
                    backend=backend,
                    xml_str=xml_s,
                    pushed_body_index=1,
                    force_target_N=f_val,
                    mu=0.5,
                    mass=1.0,
                    dt=0.0005,
                    n_rollouts=n_rollouts,
                )
                cid = f"middle_push_F{f_val:g}N_mu0.5_m1.0"
                for rec in r_out["rollouts"]:
                    all_rollout_records.append({
                        **rec,
                        "condition_id": cid,
                        "condition_group": "middle_box_force_sweep",
                    })
                cond_results.append({"condition_id": cid, "condition_group": "middle_box_force_sweep", **{k: v for k, v in r_out.items() if k != "rollouts"}})

            # Group 2: Top-box sub-Coulomb & supra-Coulomb force sweep (ideal breakaway = mu*m*g = 4.905 N)
            for f_val in (1.0, 2.0, 3.0, 4.0, 6.0, 8.0):
                xml_s = three_box_xml(
                    dt=0.0005,
                    mass=1.0,
                    mu=0.5,
                    impratio=scand["impratio"],
                    solreffriction_tc=scand["solreffriction_tc"],
                    noslip_iterations=scand["noslip"],
                )
                if scand.get("native_default"):
                    xml_s = native_default_mjcf(xml_s)
                r_out = run_three_box_rollouts(
                    backend=backend,
                    xml_str=xml_s,
                    pushed_body_index=2,
                    force_target_N=f_val,
                    mu=0.5,
                    mass=1.0,
                    dt=0.0005,
                    n_rollouts=n_rollouts,
                )
                cid = f"top_push_F{f_val:g}N_mu0.5_m1.0"
                for rec in r_out["rollouts"]:
                    all_rollout_records.append({
                        **rec,
                        "condition_id": cid,
                        "condition_group": "top_box_force_sweep",
                    })
                cond_results.append({"condition_id": cid, "condition_group": "top_box_force_sweep", **{k: v for k, v in r_out.items() if k != "rollouts"}})

            # Group 3: Setup & Plausible-Range Sweep (varying mu, mass, and dt)
            setup_variants = [
                ("mu0.3_m1.0_retain", 0.3, 1.0, 0.0005, 1, 4.0),   # breakaway = 5.886 N -> retain
                ("mu0.3_m1.0_eject", 0.3, 1.0, 0.0005, 1, 8.0),    # breakaway = 5.886 N -> eject
                ("mu0.7_m1.0_retain", 0.7, 1.0, 0.0005, 1, 10.0),  # breakaway = 13.734 N -> retain
                ("mu0.7_m1.0_eject", 0.7, 1.0, 0.0005, 1, 16.0),   # breakaway = 13.734 N -> eject
                ("mu0.5_m0.5_retain", 0.5, 0.5, 0.0005, 1, 3.5),   # breakaway = 4.905 N -> retain
                ("mu0.5_m0.5_eject", 0.5, 0.5, 0.0005, 1, 6.5),    # breakaway = 4.905 N -> eject
                ("dt0.25ms_hold", 0.5, 1.0, 0.00025, 1, 7.5),      # numerical dt=0.25ms
                ("dt1.0ms_hold", 0.5, 1.0, 0.001, 1, 7.5),         # numerical dt=1.0ms
            ]
            for vtag, v_mu, v_m, v_dt, p_idx, f_val in setup_variants:
                xml_s = three_box_xml(
                    dt=v_dt,
                    mass=v_m,
                    mu=v_mu,
                    impratio=scand["impratio"],
                    solreffriction_tc=scand["solreffriction_tc"],
                    noslip_iterations=scand["noslip"],
                )
                if scand.get("native_default"):
                    xml_s = native_default_mjcf(xml_s)
                r_out = run_three_box_rollouts(
                    backend=backend,
                    xml_str=xml_s,
                    pushed_body_index=p_idx,
                    force_target_N=f_val,
                    mu=v_mu,
                    mass=v_m,
                    dt=v_dt,
                    n_rollouts=n_rollouts,
                )
                for rec in r_out["rollouts"]:
                    all_rollout_records.append({
                        **rec,
                        "condition_id": vtag,
                        "condition_group": "setup_and_dt_variants",
                    })
                cond_results.append({"condition_id": vtag, "condition_group": "setup_and_dt_variants", **{k: v for k, v in r_out.items() if k != "rollouts"}})

            outcome_stats = compute_binary_outcome_stats(all_rollout_records)
            tb_stage_results[sname] = {
                "selected_config": scand,
                "outcome_statistics": outcome_stats,
                "conditions": cond_results,
            }
            print(
                f"  [Three-Box {sname}] Balanced Acc={outcome_stats['pooled_balanced_accuracy']:.4f}, "
                f"Macro Bal Acc={outcome_stats['macro_balanced_accuracy']:.4f}, "
                f"FPR={outcome_stats['pooled_FPR']:.4f}, FNR={outcome_stats['pooled_FNR']:.4f}",
                flush=True,
            )

        # ---------------------------------------------------------------------
        # C. SIMPLE PINCH: B=1/B=5 selection on a near-boundary grasp.
        # ---------------------------------------------------------------------
        # At compression=0.2mm, N ≈ 3.92 N, mu=0.4 -> 2*mu*N = 3.136 N.
        # For mass = 0.295 kg, mg = 2.894 N -> ideal r = 1.084 (should retain).
        pinch_candidates = [
            {"solref_tc": 0.004, "impratio": 1.0, "noslip": 0, "native_default": True, "stage_hint": "B=1_native_out_of_box"},
            {"solref_tc": 0.002, "impratio": 50.0, "noslip": 0, "stage_hint": "B=5_guided"},
            {"solref_tc": 0.001, "impratio": 50.0, "noslip": 0, "stage_hint": "B<=5_guided"},
            {"solref_tc": 0.002, "impratio": 100.0, "noslip": 2, "stage_hint": "B<=5_guided"},
            {"solref_tc": 0.004, "impratio": 100.0, "noslip": 2, "stage_hint": "B<=5_guided"},
        ]
        pinch_candidates = pinch_candidates[:budget_limit]

        pinch_budget_rows = []
        for idx, cand in enumerate(pinch_candidates):
            if idx == 0 and prior is not None:
                pinch_budget_rows.append(prior["simple_pinch"]["budget_candidates"][0])
                continue
            tag = f"cand{idx+1:02d}_tc{cand['solref_tc']*1e3:g}ms_ir{cand['impratio']:g}_ns{cand['noslip']}"
            xml_s = simple_pinch_xml(
                dt=0.0005,
                mu=0.4,
                mass=0.295,
                compression_m=0.0002,
                solref_tc=cand["solref_tc"],
                impratio=cand["impratio"],
                noslip_iterations=cand["noslip"],
            )
            if cand.get("native_default"):
                xml_s = native_default_mjcf(xml_s)
            res = run_simple_pinch_rollouts(
                backend=backend,
                xml_str=xml_s,
                mu=0.4,
                mass=0.295,
                compression_m=0.0002,
                dt=0.0005,
                n_rollouts=n_rollouts,
            )
            passes = bool(res["retain_fraction"] >= 0.99 and res["E_z_mean_mm"] <= 3.0)
            row = {
                "candidate_index": idx + 1,
                "config_tag": tag,
                **cand,
                "grasp_margin_r": res["grasp_margin_r"],
                "retain_fraction": res["retain_fraction"],
                "E_z_mean_mm": res["E_z_mean_mm"],
                "hold_creep_mean_um_s": res["hold_creep_mean_um_s"],
                "passes_criterion": passes,
                "primary_loss": res["E_z_mean_mm"],
                "wall_s": res["wall_s"],
            }
            pinch_budget_rows.append(row)
            print(
                f"  [Simple-Pinch B=5 candidate {idx+1:02d}/5] {tag}: E_z={row['E_z_mean_mm']:.2f} mm, pass={passes}",
                flush=True,
            )

        pinch_budget_summary = summarize_budget_curve(pinch_budget_rows)

        # ---------------------------------------------------------------------
        # D. SIMPLE PINCH: Bracketed Grip-Force Threshold F*_grip,sim & M=20 Sweeps
        # ---------------------------------------------------------------------
        pinch_stages = {"B1_default": pinch_candidates[0]}
        if budget_limit >= 5:
            pinch_stages["B5_guided"] = pinch_candidates[pinch_budget_summary["stage_B5"]["best_candidate_index_1based"] - 1]
        pinch_stage_results = {}
        for sname, scand in pinch_stages.items():
            if sname == "B1_default" and prior is not None:
                pinch_stage_results[sname] = prior["simple_pinch"]["stages"][sname]
                continue
            cond_results = []
            all_rollout_records = []

            # Setup 1: N4 preload (compression=0.2mm, N ≈ 3.92 N, mu=0.4) across 8 mass conditions spanning r in [0.78, 1.60]
            n4_masses = [0.200, 0.240, 0.270, 0.290, 0.305, 0.335, 0.370, 0.420]
            for m_val in n4_masses:
                xml_s = simple_pinch_xml(
                    dt=0.0005,
                    mu=0.4,
                    mass=m_val,
                    compression_m=0.0002,
                    solref_tc=scand["solref_tc"],
                    impratio=scand["impratio"],
                    noslip_iterations=scand["noslip"],
                )
                if scand.get("native_default"):
                    xml_s = native_default_mjcf(xml_s)
                r_out = run_simple_pinch_rollouts(
                    backend=backend,
                    xml_str=xml_s,
                    mu=0.4,
                    mass=m_val,
                    compression_m=0.0002,
                    dt=0.0005,
                    n_rollouts=n_rollouts,
                )
                cid = f"N4_mu0.4_m{m_val:.3f}kg"
                for rec in r_out["rollouts"]:
                    all_rollout_records.append({
                        **rec,
                        "condition_id": cid,
                        "condition_group": "N4_preload_mass_sweep",
                    })
                cond_results.append({"condition_id": cid, "condition_group": "N4_preload_mass_sweep", **{k: v for k, v in r_out.items() if k != "rollouts"}})

            # Estimate minimum hold grip force F*_grip,sim for reference mass m_ref = 0.250 kg (F*_grip,ideal = 0.25*9.81/(2*0.4) = 3.0656 N)
            # by bracketed preload compression sweep
            m_ref = 0.250
            mu_ref = 0.4
            f_grip_ideal_ref = (m_ref * G) / (2.0 * mu_ref)
            comp_grid = np.linspace(0.00012, 0.00024, 9)
            passing_preloads = []
            for c_val in comp_grid:
                xml_s = simple_pinch_xml(
                    dt=0.0005,
                    mu=mu_ref,
                    mass=m_ref,
                    compression_m=float(c_val),
                    solref_tc=scand["solref_tc"],
                    impratio=scand["impratio"],
                    noslip_iterations=scand["noslip"],
                )
                if scand.get("native_default"):
                    xml_s = native_default_mjcf(xml_s)
                probe = run_simple_pinch_rollouts(
                    backend=backend,
                    xml_str=xml_s,
                    mu=mu_ref,
                    mass=m_ref,
                    compression_m=float(c_val),
                    dt=0.0005,
                    n_rollouts=n_rollouts,
                )
                if probe["retain_fraction"] >= 0.99:
                    passing_preloads.append(probe["N_mean_preload_N"])
            f_grip_sim_star = min(passing_preloads) if passing_preloads else float("nan")
            e_grip = abs(f_grip_sim_star - f_grip_ideal_ref) / f_grip_ideal_ref if math.isfinite(f_grip_sim_star) else float("nan")

            # Setup 2: N6 preload (compression=0.3mm, N ≈ 5.88 N) + friction variants + dt variants
            setup2_cases = [
                ("N6_mu0.4_m0.360_retain", 0.4, 0.360, 0.0003, 0.0005),
                ("N6_mu0.4_m0.430_retain", 0.4, 0.430, 0.0003, 0.0005),
                ("N6_mu0.4_m0.520_release", 0.4, 0.520, 0.0003, 0.0005),
                ("N4_mu0.3_m0.200_retain", 0.3, 0.200, 0.0002, 0.0005),
                ("N4_mu0.3_m0.280_release", 0.3, 0.280, 0.0002, 0.0005),
                ("N4_mu0.5_m0.340_retain", 0.5, 0.340, 0.0002, 0.0005),
                ("N4_mu0.5_m0.450_release", 0.5, 0.450, 0.0002, 0.0005),
                ("N4_dt0.25ms_m0.270_retain", 0.4, 0.270, 0.0002, 0.00025),
                ("N4_dt1.0ms_m0.270_retain", 0.4, 0.270, 0.0002, 0.001),
            ]
            for vtag, v_mu, v_m, v_comp, v_dt in setup2_cases:
                xml_s = simple_pinch_xml(
                    dt=v_dt,
                    mu=v_mu,
                    mass=v_m,
                    compression_m=v_comp,
                    solref_tc=scand["solref_tc"],
                    impratio=scand["impratio"],
                    noslip_iterations=scand["noslip"],
                )
                if scand.get("native_default"):
                    xml_s = native_default_mjcf(xml_s)
                r_out = run_simple_pinch_rollouts(
                    backend=backend,
                    xml_str=xml_s,
                    mu=v_mu,
                    mass=v_m,
                    compression_m=v_comp,
                    dt=v_dt,
                    n_rollouts=n_rollouts,
                )
                for rec in r_out["rollouts"]:
                    all_rollout_records.append({
                        **rec,
                        "condition_id": vtag,
                        "condition_group": "N6_and_setup_variants",
                    })
                cond_results.append({"condition_id": vtag, "condition_group": "N6_and_setup_variants", **{k: v for k, v in r_out.items() if k != "rollouts"}})

            outcome_stats = compute_binary_outcome_stats(all_rollout_records)
            pinch_stage_results[sname] = {
                "selected_config": scand,
                "selection_condition": "mu=0.4, mass=0.295 kg, compression=0.2 mm; excluded from evaluation grid",
                "F_grip_ideal_N": f_grip_ideal_ref,
                "F_grip_sim_star_N": f_grip_sim_star,
                "E_grip_relative_error": e_grip,
                "outcome_statistics": outcome_stats,
                "conditions": cond_results,
            }
            print(
                f"  [Simple-Pinch {sname}] F*_grip={f_grip_sim_star:.3f} N (ideal {f_grip_ideal_ref:.3f} N, E_grip={e_grip*100:.2f}%), "
                f"Bal Acc={outcome_stats['pooled_balanced_accuracy']:.4f}, Macro Bal Acc={outcome_stats['macro_balanced_accuracy']:.4f}",
                flush=True,
            )

        summary["backends"][backend] = {
            "three_box_push": {
                "selection_condition": "top-box force=3.5 N, mu=0.5, mass=1 kg; excluded from evaluation grid",
                "budget_summary": tb_budget_summary,
                "budget_candidates": three_box_budget_rows,
                "stages": tb_stage_results,
            },
            "simple_pinch": {
                "budget_summary": pinch_budget_summary,
                "budget_candidates": pinch_budget_rows,
                "stages": pinch_stage_results,
            },
        }

        (b_dir / "static_friction_v2_summary.json").write_text(json.dumps(summary["backends"][backend], indent=2) + "\n")

    (output_dir / "static_friction_v2_all.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backends", default="mujoco,mujoco_warp")
    parser.add_argument("--rollouts", type=int, default=20)
    parser.add_argument("--budget-limit", type=int, choices=(1, 5), default=5)
    parser.add_argument("--reuse-b1-summary", type=Path)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    backends = [b.strip() for b in args.backends.split(",") if b.strip()]
    evaluate_all_static_friction_v2(backends, args.output_dir, n_rollouts=args.rollouts,
                                    budget_limit=args.budget_limit, reuse_b1_summary=args.reuse_b1_summary)


if __name__ == "__main__":
    main()
