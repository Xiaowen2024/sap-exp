"""MuJoCo A/B/C contact sweep matched in structure to the SAP Drake sweep.

There is no MuJoCo equivalent of SAP's scalar ``ke``.  MuJoCo's contact
impedance is jointly controlled by ``solref`` and ``solimp``; section C varies
the latter while keeping the solref time constant fixed.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import sys
import time

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_mujoco_rolling_benchmark import FORCE, model_xml, reference_velocity

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "mujoco-matched-contact-sweep"


def evaluate(section: str, dt: float, solref_timeconst: float, solimp_min: float, solimp_max: float) -> dict:
    model = mujoco.MjModel.from_xml_string(model_xml())
    model.opt.solver = mujoco.mjtSolver.mjSOL_NEWTON
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_EULER
    model.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    model.opt.timestep, model.opt.iterations, model.opt.tolerance = dt, 1000, 1e-30
    model.geom_solref[:] = (solref_timeconst, 1.0)
    model.geom_solimp[:] = (solimp_min, solimp_max, .001, .5, 2.0)
    data = mujoco.MjData(model)
    box = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box")
    ball = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball_-4_-4")
    box_dof, ball_dof = model.body_dofadr[box], model.body_dofadr[ball]
    steps, error_sum = int(4.0 / dt), 0.0
    force = np.array([FORCE * .5, FORCE * math.sqrt(3.) / 2., 0.])
    t0 = time.perf_counter()
    for _ in range(steps):
        box_ref, ball_ref = reference_velocity(np.array([data.time]))
        box_v, ball_v = data.qvel[box_dof:box_dof + 3], data.qvel[ball_dof:ball_dof + 3]
        error_sum += float(np.dot(box_v - box_ref[0], box_v - box_ref[0]) + np.dot(ball_v - ball_ref[0], ball_v - ball_ref[0]))
        data.xfrc_applied[box, :3] = force
        mujoco.mj_step(model, data)
    elapsed = time.perf_counter() - t0
    return {"section": section, "solver": "Newton", "integrator": "Euler", "cone": "elliptic", "dt_s": dt,
            "solref_timeconst_s": solref_timeconst, "solimp_min": solimp_min, "solimp_max": solimp_max,
            "steps": steps, "mse_velocity_error": error_sum / steps, "elapsed_s": elapsed, "step_rate_hz": steps / elapsed}


def main() -> None:
    cases = []
    # A: same timesteps as SAP's 25-ball A sweep.
    cases += [("A_dt", dt, .020, .90, .95) for dt in (.0005, .001, .002, .005)]
    # B: MuJoCo contact time constant is the closest counterpart to SAP tau.
    cases += [("B_solref_timeconst", .001, tc, .90, .95) for tc in (.004, .010, .030, .060)]
    # C: contact impedance, not an exact SAP-ke mapping.
    cases += [("C_solimp_impedance", .001, .020, lo, hi) for lo, hi in ((.70, .75), (.90, .95), (.98, .99))]
    OUT.mkdir(parents=True, exist_ok=True)
    result_path = OUT / "results.csv"
    rows = []
    for i, case in enumerate(cases, 1):
        row = evaluate(*case); rows.append(row)
        with result_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(row)); writer.writeheader(); writer.writerows(rows)
        print(f"{i}/{len(cases)} {json.dumps(row)}", flush=True)
    (OUT / "metadata.json").write_text(json.dumps({"purpose": "MuJoCo A/B/C sweep structurally matched to SAP Drake experiment", "fixed": {"solver": "Newton", "integrator": "Euler", "cone": "elliptic", "iterations": 1000, "tolerance": 1e-30}, "sections": {"A": "dt", "B": "solref time constant", "C": "solimp impedance; no one-to-one SAP ke equivalence"}}, indent=2) + "\n")


if __name__ == "__main__":
    main()
