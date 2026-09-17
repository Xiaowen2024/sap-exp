"""Factorial MuJoCo sweep for the SimBenchmark rolling test."""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import mujoco
import numpy as np

from run_mujoco_rolling_benchmark import BALL_MASS, BOX_MASS, DT, DURATION, FORCE, G, N_BALLS, model_xml

WORKSPACE = Path(__file__).resolve().parents[3]
OUT = WORKSPACE / "exp-results" / "rolling" / "mujoco-sweep"
SOLVERS = {"pgs": mujoco.mjtSolver.mjSOL_PGS, "cg": mujoco.mjtSolver.mjSOL_CG, "newton": mujoco.mjtSolver.mjSOL_NEWTON}
INTEGRATORS = {"euler": mujoco.mjtIntegrator.mjINT_EULER, "rk4": mujoco.mjtIntegrator.mjINT_RK4, "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT, "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST}
CONES = {"pyramidal": mujoco.mjtCone.mjCONE_PYRAMIDAL, "elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC}
SOFTNESS = {"default": None, "soft_40ms": (0.040, 1.0), "stiff_4ms": (0.004, 1.0)}


def reference(t: float) -> tuple[np.ndarray, np.ndarray]:
    f_ground = .4 * (BOX_MASS + N_BALLS * BALL_MASS) * G
    f_ball = (FORCE - f_ground) / BOX_MASS / (3.5 / BALL_MASS + N_BALLS / BOX_MASS)
    box_acc = (FORCE - f_ground - N_BALLS * f_ball) / BOX_MASS
    ball_acc = f_ball / BALL_MASS
    direction = np.array([.5, math.sqrt(3.) / 2., 0.])
    return box_acc * t * direction, ball_acc * t * direction


def run_case(solver: str, integrator: str, cone: str, iterations: int, timestep: float, softness: str) -> dict:
    m = mujoco.MjModel.from_xml_string(model_xml()); m.opt.solver = SOLVERS[solver]; m.opt.integrator = INTEGRATORS[integrator]
    # Match the published SimBenchmark CSV.  ``iterations`` remains a maximum;
    # MuJoCo may still stop an iterative solver once its residual is below this
    # deliberately tiny tolerance.
    m.opt.cone = CONES[cone]; m.opt.iterations = iterations; m.opt.timestep = timestep; m.opt.tolerance = 1e-30
    if SOFTNESS[softness] is not None: m.geom_solref[:] = SOFTNESS[softness]
    d = mujoco.MjData(m); box = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "box"); ball = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ball_-4_-4")
    box_dof, ball_dof = m.body_dofadr[box], m.body_dofadr[ball]; force = np.array([FORCE * .5, FORCE * math.sqrt(3.) / 2., 0.])
    steps, error_sum, max_contacts, finite = int(DURATION / timestep), 0., 0, True; t0 = time.perf_counter()
    for _ in range(steps):
        box_ref, ball_ref = reference(d.time); box_v, ball_v = d.qvel[box_dof:box_dof+3], d.qvel[ball_dof:ball_dof+3]
        error_sum += float(np.dot(box_v-box_ref, box_v-box_ref) + np.dot(ball_v-ball_ref, ball_v-ball_ref))
        d.xfrc_applied[box, :3] = force; mujoco.mj_step(m, d); max_contacts = max(max_contacts, d.ncon)
        finite &= bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
        if not finite: break
    elapsed = time.perf_counter() - t0; box_ref, ball_ref = reference(d.time)
    box_v, ball_v = d.qvel[box_dof:box_dof+3].copy(), d.qvel[ball_dof:ball_dof+3].copy()
    return {"solver": solver, "integrator": integrator, "cone": cone, "iterations": iterations, "dt_s": timestep, "softness": softness,
            "steps_planned": steps, "steps_completed": int(round(d.time/timestep)), "stable": finite and d.time >= DURATION - timestep,
            "mse_velocity_error": error_sum / max(1, int(round(d.time/timestep))), "final_box_speed_error": float(np.linalg.norm(box_v-box_ref)),
            "final_ball_speed_error": float(np.linalg.norm(ball_v-ball_ref)), "max_contacts": max_contacts,
            "step_rate_hz": int(round(d.time/timestep)) / elapsed, "wall_s": elapsed}


def main() -> None:
    p = argparse.ArgumentParser(); p.add_argument("--limit", type=int); p.add_argument("--jobs", type=int, default=1); args = p.parse_args(); OUT.mkdir(parents=True, exist_ok=True)
    cases = list(itertools.product(SOLVERS, INTEGRATORS, CONES, (20, 100, 500), (.0005, .001, .002), SOFTNESS))
    if args.limit: cases = cases[:args.limit]
    rows = []
    if args.jobs == 1:
        for index, case in enumerate(cases, 1):
            row = run_case(*case); rows.append(row); print(index, json.dumps(row), flush=True)
    else:
        with ProcessPoolExecutor(max_workers=args.jobs) as pool:
            futures = {pool.submit(run_case, *case): index for index, case in enumerate(cases, 1)}
            for future in as_completed(futures):
                index, row = futures[future], future.result(); rows.append(row); print(index, json.dumps(row), flush=True)
    rows.sort(key=lambda row: (list(SOLVERS).index(row["solver"]), list(INTEGRATORS).index(row["integrator"]), list(CONES).index(row["cone"]), row["iterations"], row["dt_s"], list(SOFTNESS).index(row["softness"])))
    fields = list(rows[0]) if rows else []
    with (OUT / "results.csv").open("w", newline="") as f: writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    (OUT / "metadata.json").write_text(json.dumps({"scenario": "SimBenchmark rolling; 10 kg 20x20x1 box, 25 1 kg radius-.5 balls, 150 N at 60 degrees", "mujoco_tolerance": 1e-30, "default_contact": {"solref": [0.02, 1.0], "solimp": [0.9, 0.95, 0.001, 0.5, 2.0]}, "softness_presets": SOFTNESS, "rows": len(rows)}, indent=2) + "\n")

if __name__ == "__main__": main()
