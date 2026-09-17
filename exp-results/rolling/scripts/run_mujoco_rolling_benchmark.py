"""Modern MuJoCo reproduction of SimBenchmark's rolling test.

The model geometry, masses, initial 5x5 ball lattice, 150 N force at 60 deg,
dt and the reference error metric are copied from leggedrobotics/SimBenchmark.
"""
from __future__ import annotations

import csv
import json
import time
from pathlib import Path

import mujoco
import numpy as np

DT, DURATION, FORCE = 0.001, 4.0, 150.0
N_BALLS, BOX_MASS, BALL_MASS, G = 25, 10.0, 1.0, 9.81
WORKSPACE = Path(__file__).resolve().parents[3]
OUT = WORKSPACE / "exp-results" / "rolling" / "mujoco"


def reference_velocity(t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    # This deliberately uses the original benchmark's analytic model, including
    # its RaiSim-calibrated effective coefficients mu_ground_box=.4, mu_box_ball=.8.
    f_ground = 0.4 * (BOX_MASS + N_BALLS * BALL_MASS) * G
    f_ball = (FORCE - f_ground) / BOX_MASS / (3.5 / BALL_MASS + N_BALLS / BOX_MASS)
    a_box = (FORCE - f_ground - N_BALLS * f_ball) / BOX_MASS
    a_ball = f_ball / BALL_MASS
    direction = np.array([0.5, np.sqrt(3.0) / 2.0, 0.0])
    return t[:, None] * a_box * direction, t[:, None] * a_ball * direction


def model_xml() -> str:
    parts = ['<mujoco><option timestep="0.001" cone="elliptic" solver="Newton" iterations="1000" tolerance="1e-30"/>',
             '<worldbody><geom name="ground" type="plane" size="100 100 5" friction="0.4 0 0"/>',
             '<body name="box" pos="0 0 0.499998"><inertial pos="0 0 0" mass="10" diaginertia="334.1666666667 334.1666666667 666.6666666667"/><freejoint/><geom type="box" size="10 10 .5" friction=".4 0 0"/></body>']
    for x in (-4, -2, 0, 2, 4):
        for y in (-4, -2, 0, 2, 4):
            parts.append(f'<body name="ball_{x}_{y}" pos="{x} {y} 1.499998"><inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/><freejoint/><geom type="sphere" size=".5" friction=".8 0 0"/></body>')
    return ''.join(parts) + '</worldbody></mujoco>'


def run(record: bool) -> tuple[np.ndarray, float]:
    model = mujoco.MjModel.from_xml_string(model_xml())
    data = mujoco.MjData(model)
    box = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box")
    ball = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball_-4_-4")
    trace = np.zeros((int(DURATION / DT), 7))
    force = np.array([FORCE * .5, FORCE * np.sqrt(3) / 2, 0.0])
    t0 = time.perf_counter()
    for step in range(len(trace)):
        if record:
            # The original wrapper's ``getLinearVelocity`` corresponds to the
            # translational part of each free joint's generalized velocity.
            # Do not use ``mj_objectVelocity(..., flg_local=1)`` here: that
            # rotates a spinning ball's velocity into its changing local frame.
            box_dof = model.body_dofadr[box]
            ball_dof = model.body_dofadr[ball]
            trace[step] = (data.time, *data.qvel[box_dof:box_dof + 3], *data.qvel[ball_dof:ball_dof + 3])
        data.xfrc_applied[box, :3] = force
        mujoco.mj_step(model, data)
    return trace, time.perf_counter() - t0


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    trace, _ = run(record=True)
    _, elapsed = run(record=False)
    box_ref, ball_ref = reference_velocity(trace[:, 0])
    sq_error = np.sum((trace[:, 1:4] - box_ref) ** 2 + (trace[:, 4:7] - ball_ref) ** 2, axis=1)
    np.savetxt(OUT / "trace.csv", trace, delimiter=",", header="time_s,box_vx,box_vy,box_vz,ball_vx,ball_vy,ball_vz", comments="")
    with (OUT / "metrics.json").open("w") as f:
        json.dump({"engine": "MuJoCo " + mujoco.__version__, "dt_s": DT, "duration_s": DURATION, "force_N": FORCE,
                   "force_direction": "xy at 60 deg", "mean_squared_velocity_error": float(sq_error.mean()),
                   "step_rate_hz": len(trace) / elapsed, "final_box_velocity_m_s": trace[-1, 1:4].tolist(),
                   "final_ball_velocity_m_s": trace[-1, 4:7].tolist()}, f, indent=2)
    print((OUT / "metrics.json").read_text())


if __name__ == "__main__":
    main()
