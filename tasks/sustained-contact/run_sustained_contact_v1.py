#!/usr/bin/env python3
"""Sustained-contact-v1: three free analytic plates, MuJoCo 3.17 / MuJoCo Warp.

This runner deliberately uses the same evaluation conventions as protocol-v2:
selection and evaluation are separated, primary B={1,5}, and every reported
evaluation condition contains R independent perturbed worlds.  It implements
the three-plate task in tasks/sustained-contact; Cube Toss is kept
separate because its public ContactNets recordings do not include a one-second
post-settling physical reference window.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from metrics_common import HAS_MJWARP, mjw, warp_contact_records, wilson_ci
from native_defaults import native_default_mjcf

ROOT = REPO_ROOT
OUT = ROOT / "exp-results" / "protocol-v2-revised" / "sustained-contact-v1"
G = 9.81
RADIUS = 0.1
MASS = 1.0 / 3.0
DEFAULT_DT = 0.0005
RUN_S = 2.0
TAIL_S = 1.0


def candidates() -> list[dict[str, float | str]]:
    """Ordered, fixed 30-configuration native MuJoCo contact budget.

    Index 0 uses the compiled out-of-box contact/solver defaults. Index 1 is the
    mechanism-guided near-rigid/damped point. The other 28 give the B=30
    ceiling a bounded time-constant/damping/impedance search.
    """
    # The first five are the *predeclared B=5* set.  They deliberately span
    # compliant through near-rigid contact; listing all 1-ms variants first
    # would turn B=5 into an accidental local sweep rather than the intended
    # mechanism-aware budget.
    ans: list[dict[str, float | str]] = [
        {"id": "native_default", "native_default": True},
        {"id": "tc1_dr2_d99", "timeconst_s": 0.001, "dampratio": 2.0, "dmax": 0.99},
        {"id": "tc2_dr2_d99", "timeconst_s": 0.002, "dampratio": 2.0, "dmax": 0.99},
        {"id": "tc4_dr2_d99", "timeconst_s": 0.004, "dampratio": 2.0, "dmax": 0.99},
        {"id": "tc8_dr2_d99", "timeconst_s": 0.008, "dampratio": 2.0, "dmax": 0.99},
    ]
    for tc in (0.001, 0.002, 0.004, 0.008, 0.016):
        for dr in (0.5, 1.0, 2.0):
            for dmax in (0.90, 0.99):
                c = {"id": f"tc{tc*1e3:g}_dr{dr:g}_d{dmax:g}", "timeconst_s": tc,
                     "dampratio": dr, "dmax": dmax}
                if c not in ans:
                    ans.append(c)
    return ans[:30]


def xml_for(h: float, c: dict[str, float | str], dt: float) -> str:
    bodies = "".join(
        f'<body name="plate_{i}" pos="0 0 {(i + .5) * h}">'
        f'<freejoint/><geom name="plate_{i}_geom" type="cylinder" size="{RADIUS} {h / 2}" mass="{MASS}" '
        f'friction="0.8 .005 .0001"/></body>'
        for i in range(3)
    )
    tc, dr, dmax = (float(c.get("timeconst_s", 0.02)),
                    float(c.get("dampratio", 1.0)), float(c.get("dmax", 0.90)))
    xml = f'''<mujoco model="sustained_three_plate">
 <option timestep="{dt}" gravity="0 0 -{G}" integrator="implicitfast" solver="Newton"
         cone="elliptic" iterations="100" tolerance="1e-10"/>
 <size nconmax="256" njmax="1024"/>
 <default><geom margin="0" gap="0" solref="{tc} {dr}" solimp="{dmax} {dmax} .001 .5 2"/></default>
 <worldbody><geom name="ground" type="plane" size="2 2 .1"/>{bodies}</worldbody>
</mujoco>'''
    return native_default_mjcf(xml) if c.get("native_default") else xml


def ids(model: mujoco.MjModel) -> tuple[list[int], list[int], list[int]]:
    bids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"plate_{i}") for i in range(3)]
    gids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"plate_{i}_geom") for i in range(3)]
    qaddrs = [int(model.jnt_qposadr[model.body_jntadr[b]]) for b in bids]
    return bids, gids, qaddrs


def contact_info(model: mujoco.MjModel, data: mujoco.MjData, gids: list[int], ground: int) -> tuple[np.ndarray, np.ndarray]:
    """Per interface normal load and active-contact indicator, bottom to top."""
    pairs = {
        frozenset((ground, gids[0])): 0,
        frozenset((gids[0], gids[1])): 1,
        frozenset((gids[1], gids[2])): 2,
    }
    fn = np.zeros(3)
    count = np.zeros(3)
    wrench = np.zeros(6)
    for ci in range(data.ncon):
        con = data.contact[ci]
        which = pairs.get(frozenset((int(con.geom1), int(con.geom2))))
        if which is None:
            continue
        mujoco.mj_contactForce(model, data, ci, wrench)
        fn[which] += max(0.0, float(wrench[0]))
        count[which] += 1
    return fn, count


def contact_info_warp(data: Any, world: int, gids: list[int], ground: int) -> tuple[np.ndarray, np.ndarray]:
    pairs = {frozenset((ground, gids[0])): 0, frozenset((gids[0], gids[1])): 1,
             frozenset((gids[1], gids[2])): 2}
    fn, count = np.zeros(3), np.zeros(3)
    for con in warp_contact_records(data, world):
        which = pairs.get(frozenset((con["geom1"], con["geom2"])))
        if which is not None:
            fn[which] += con["normal"]; count[which] += 1
    return fn, count


def _summarize(
    z: np.ndarray, vlin: np.ndarray, vang: np.ndarray, fns: np.ndarray, counts: np.ndarray,
    h: float, dt: float, wall_s: float, overflow: int, backend: str, c: dict[str, Any],
) -> dict[str, Any]:
    # Shapes: [sample, rollout, plate/interface].  g proxies follow the declared
    # analytic, axis-aligned interface convention for this task.
    gaps = np.empty((len(z), z.shape[1], 3))
    gaps[:, :, 0] = z[:, :, 0] - h / 2
    gaps[:, :, 1] = z[:, :, 1] - z[:, :, 0] - h
    gaps[:, :, 2] = z[:, :, 2] - z[:, :, 1] - h
    overlap = np.maximum(0.0, -gaps)
    tail = np.arange(len(z)) >= max(0, len(z) - round(TAIL_S / (0.01)))
    # Sustained classification: final 0.2s remains under the protocol velocity limits.
    speed = np.linalg.norm(vlin, axis=2)
    omega = np.linalg.norm(vang, axis=2)
    last = np.arange(len(z)) >= max(0, len(z) - round(0.2 / .01))
    settled = np.all((speed[last] < .01) & (omega[last] < .05), axis=(0, 2))
    # After selecting an individual plate the array is [sample, rollout].
    order = np.all((z[:, :, 2] > z[:, :, 1]) & (z[:, :, 1] > z[:, :, 0]) & (z[:, :, 0] > 0), axis=0)
    peak_overlap = np.max(overlap, axis=(0, 2))
    fn_tail = fns[tail]
    expected = np.array([3, 2, 1], dtype=float) * MASS * G
    fn_mean = np.mean(fn_tail, axis=0)  # rollout, interface
    force_err = np.abs(fn_mean - expected[None, :]) / expected[None, :]
    duty = np.mean(counts[tail] > 0, axis=(0, 2))
    fn_cv = np.std(fn_tail, axis=0) / np.maximum(np.mean(fn_tail, axis=0), 1e-12)
    fn_cv_rollout = np.mean(fn_cv, axis=1)
    zref = np.array([.5, 1.5, 2.5]) * h
    height_bias = z[-1] - zref[None, :]
    # success has the predeclared 5%-of-thickness overlap gate.
    passed = settled & order & (peak_overlap <= .05 * h) & np.all(force_err <= .01, axis=1)
    records = []
    for r in range(z.shape[1]):
        records.append({
            "rollout": r, "settled": bool(settled[r]), "order_preserved": bool(order[r]),
            "max_overlap_mm": float(peak_overlap[r] * 1e3),
            "max_overlap_over_h": float(peak_overlap[r] / h),
            "force_balance_error_max": float(np.max(force_err[r])),
            "force_balance_error_by_interface": force_err[r].tolist(),
            "contact_duty_by_interface": np.mean(counts[tail, r] > 0, axis=0).tolist(),
            "normal_force_cv_by_interface": fn_cv[r].tolist(),
            "height_bias_mm": (height_bias[r] * 1e3).tolist(),
            "vz_tail_rms_mm_s": float(np.sqrt(np.mean(vlin[tail, r, :, 2] ** 2)) * 1e3),
            "passes": bool(passed[r]),
        })
    n = len(records); k = int(np.sum(passed))
    return {
        "backend": backend, "mujoco_version": mujoco.__version__, "candidate": c,
        "dt_ms": dt * 1e3, "thickness_mm": h * 1e3, "radius_mm": RADIUS * 1e3,
        "plate_mass_kg": MASS, "duration_s": RUN_S, "tail_window_s": TAIL_S, "M_rollouts": n,
        "pass_fraction": k / n, "pass_wilson_95ci": list(wilson_ci(k, n)),
        "settled_fraction": float(np.mean(settled)), "order_preserved_fraction": float(np.mean(order)),
        "max_overlap_mm_mean": float(np.mean(peak_overlap) * 1e3),
        "max_overlap_mm_std": float(np.std(peak_overlap) * 1e3),
        "max_overlap_over_h_mean": float(np.mean(peak_overlap / h)),
        "normal_load_error_mean": float(np.mean(force_err)),
        "normal_load_error_max_mean": float(np.mean(np.max(force_err, axis=1))),
        "contact_duty_mean": float(np.mean(duty)), "normal_force_cv_mean": float(np.mean(fn_cv_rollout)),
        "height_bias_abs_mm_mean": float(np.mean(np.abs(height_bias)) * 1e3),
        "vz_tail_rms_mm_s_mean": float(np.mean(np.sqrt(np.mean(vlin[tail, :, :, 2] ** 2, axis=0))) * 1e3),
        "overflow_worlds": overflow, "wall_s": wall_s, "rollouts": records,
    }


def run_case(backend: str, c: dict[str, Any], h: float, dt: float, rollouts: int, seed: int) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_string(xml_for(h, c, dt))
    bids, gids, qaddrs = ids(model)
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    q0 = np.tile(model.qpos0, (rollouts, 1)).astype(np.float64)
    rng = np.random.default_rng(seed)
    # Independent micro perturbations; there is deliberately no nominal member.
    for r in range(rollouts):
        for i, qa in enumerate(qaddrs):
            q0[r, qa:qa+2] += rng.uniform(-1e-4, 1e-4, 2)
            yaw = rng.uniform(-.003, .003)
            q0[r, qa+3] = math.cos(yaw / 2)
            q0[r, qa+6] = math.sin(yaw / 2)
    steps, sample = round(RUN_S / dt), max(1, round(.01 / dt))
    z, vlin, vang, fns, counts = [], [], [], [], []
    started = time.perf_counter(); overflow = 0
    if backend == "mujoco_warp":
        if not HAS_MJWARP:
            raise RuntimeError("MuJoCo Warp not installed")
        gm = mjw.put_model(model); data = mjw.make_data(model, nworld=rollouts, nconmax=256, njmax=1024)
        data.qpos.assign(q0.astype(np.float32)); mjw.forward(gm, data)
        for step in range(steps + 1):
            if step % sample == 0:
                qp, qv = data.qpos.numpy(), data.qvel.numpy()
                z.append(np.stack([qp[:, qa+2] for qa in qaddrs], axis=1))
                vlin.append(np.stack([qv[:, int(model.jnt_dofadr[model.body_jntadr[b]]):int(model.jnt_dofadr[model.body_jntadr[b]])+3] for b in bids], axis=1))
                vang.append(np.stack([qv[:, int(model.jnt_dofadr[model.body_jntadr[b]])+3:int(model.jnt_dofadr[model.body_jntadr[b]])+6] for b in bids], axis=1))
                ff, cc = [], []
                for r in range(rollouts):
                    f, n = contact_info_warp(data, r, gids, ground); ff.append(f); cc.append(n)
                fns.append(np.asarray(ff)); counts.append(np.asarray(cc))
            if step < steps: mjw.step(gm, data)
        overflow = int(np.sum(data.overflow.numpy() > 0))
    else:
        datas = [mujoco.MjData(model) for _ in range(rollouts)]
        for r, d in enumerate(datas): d.qpos[:] = q0[r]; mujoco.mj_forward(model, d)
        dofs = [int(model.jnt_dofadr[model.body_jntadr[b]]) for b in bids]
        for step in range(steps + 1):
            if step % sample == 0:
                z.append(np.asarray([[d.qpos[qa+2] for qa in qaddrs] for d in datas]))
                vlin.append(np.asarray([[d.qvel[va:va+3] for va in dofs] for d in datas]))
                vang.append(np.asarray([[d.qvel[va+3:va+6] for va in dofs] for d in datas]))
                fc = [contact_info(model, d, gids, ground) for d in datas]
                fns.append(np.asarray([x[0] for x in fc])); counts.append(np.asarray([x[1] for x in fc]))
            if step < steps:
                for d in datas: mujoco.mj_step(model, d)
    return _summarize(np.asarray(z), np.asarray(vlin), np.asarray(vang), np.asarray(fns), np.asarray(counts), h, dt, time.perf_counter()-started, overflow, backend, c)


def select(backend: str, candidates_: list[dict[str, Any]], budget: int, seed: int) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    rows = [run_case(backend, c, .004, DEFAULT_DT, 20, seed + i * 100) for i, c in enumerate(candidates_[:budget])]
    # Pre-registered lexicographic selection: pass fraction, overlap, balance, wall time.
    best = min(rows, key=lambda x: (-x["pass_fraction"], x["max_overlap_over_h_mean"], x["normal_load_error_mean"], x["wall_s"]))
    return dict(best["candidate"]), rows


def execute(backend: str, budgets: list[int], output: Path) -> dict[str, Any]:
    cs = candidates(); stages = {}
    for b in budgets:
        chosen, selection = select(backend, cs, b, 10000 + b)
        # Held-out thicknesses plus one numerical local-robustness dt slice.
        evaluation = [run_case(backend, chosen, h, DEFAULT_DT, 20, 20000 + b * 100 + j)
                      for j, h in enumerate((.020, .010, .005, .002, .001, .0005))]
        evaluation += [run_case(backend, chosen, .005, dt, 20, 30000 + b * 100 + j)
                       for j, dt in enumerate((.00025, .001))]
        stages[f"B{b}"] = {"budget": b, "calibration_condition": {"thickness_mm": 4.0, "dt_ms": .5, "M": 20},
                             "selection_candidates": selection, "frozen_candidate": chosen,
                             "heldout_evaluation": evaluation}
    result = {"protocol": "sustained-contact-v1", "task": "three-plate-stack", "engine": backend,
              "engine_version": mujoco.__version__, "execution": "MuJoCo Warp batched worlds" if backend == "mujoco_warp" else "MuJoCo CPU",
              "physics": {"radius_m": RADIUS, "mass_per_plate_kg": MASS, "gravity_m_s2": G,
                          "initial_state": "concentric, rest; R independent xy/yaw micro-perturbations; no drop"},
              "shared_protocol": {"B": budgets, "formal_R": 20, "selection_R": 20,
                                  "overlap_gate": "max_i delta_eq,i <= 0.05 h", "force_gate": "all interface load errors <= 1%",
                                  "sustained_rule": "all bodies |v|<.01 m/s and |omega|<.05 rad/s for final .2 s; 1 s tail"},
              "stages": stages}
    output.mkdir(parents=True, exist_ok=True)
    (output / f"three_plate_{backend}.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--backends", nargs="+", default=["mujoco", "mujoco_warp"], choices=["mujoco", "mujoco_warp"])
    p.add_argument("--budgets", nargs="+", type=int, default=[1, 5])
    p.add_argument("--output", type=Path, default=OUT)
    args = p.parse_args()
    for backend in args.backends:
        r = execute(backend, args.budgets, args.output)
        print(json.dumps({"backend": backend, "version": r["engine_version"],
                          "chosen": {k: v["frozen_candidate"] for k, v in r["stages"].items()}}, indent=2), flush=True)


if __name__ == "__main__":
    main()
