"""Modern MuJoCo reproduction of SimBenchmark's 49-ball elastic bounce test.

The original task is copied from leggedrobotics/SimBenchmark: 7x7 independent
10 kg, radius-0.1 m balls drop from 5 m onto a frictionless plane for 20 s.
Its analytic invariant is total mechanical energy E0.  This script uses the
current MuJoCo *direct* solref format [-stiffness, -damping]; damping=0 is the
modern mechanism intended for a perfectly elastic contact response.
"""
from __future__ import annotations

import csv
import json
import time
from pathlib import Path

import mujoco
import numpy as np

WORKSPACE = Path(__file__).resolve().parents[3]
OUT = WORKSPACE / "exp-results" / "bouncing" / "mujoco"
MASS, RADIUS, HEIGHT, G, N_SIDE, DURATION = 10.0, 0.1, 5.0, 9.81, 7, 20.0
N_BALLS = N_SIDE * N_SIDE
E0 = N_BALLS * MASS * G * HEIGHT


def xml(dt: float, stiffness: float, damping: float, solver: str = "Newton") -> str:
    bodies = []
    # The 2 m lattice from the original XML prevents ball--ball contacts.
    for ix in range(N_SIDE):
        for iy in range(N_SIDE):
            bodies.append(
                f'<body name="ball_{ix}_{iy}" pos="{2*ix} {2*iy} {HEIGHT}">'
                f'<inertial pos="0 0 0" mass="{MASS}" diaginertia="0.04 0.04 0.04"/>'
                '<freejoint/><geom type="sphere" size="0.1" friction="0 0 0"/>'
                '</body>'
            )
    return (
        '<mujoco model="elastic-bouncing">'
        f'<option timestep="{dt}" gravity="0 0 -{G}" solver="{solver}" '
        'cone="elliptic" iterations="1000" tolerance="1e-12"/>'
        '<default><geom solimp="0.999 0.999 0.001 0.5 2" '
        f'solref="-{stiffness} -{damping}"/></default>'
        '<worldbody><geom name="ground" type="plane" size="200 200 5" friction="0 0 0"/>'
        + ''.join(bodies) + '</worldbody></mujoco>'
    )


def mechanical_energy(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    # For frictionless spheres, rotational kinetic energy stays zero. Include it
    # nevertheless so this remains the system mechanical-energy metric.
    total = 0.0
    for body_id in range(1, model.nbody):
        dof = model.body_dofadr[body_id]
        v = data.qvel[dof:dof + 3]
        omega = data.qvel[dof + 3:dof + 6]
        inertia = model.body_inertia[body_id]
        total += 0.5 * MASS * float(v @ v)
        total += 0.5 * float(np.sum(inertia * omega * omega))
        total += MASS * G * data.xpos[body_id, 2]
    return total


def run_case(dt: float, stiffness: float, damping: float) -> tuple[dict[str, float], np.ndarray]:
    model = mujoco.MjModel.from_xml_string(xml(dt, stiffness, damping))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)  # populate xpos before sampling t=0 energy
    steps = round(DURATION / dt)
    trace = np.empty((steps, 4))
    start = time.perf_counter()
    for i in range(steps):
        energy = mechanical_energy(model, data)
        trace[i] = (data.time, energy, energy - E0, data.xpos[1, 2])
        mujoco.mj_step(model, data)
    elapsed = time.perf_counter() - start
    err = trace[:, 2]
    # Local apexes after impact are an intuitive restitution check. The energy
    # MSE remains the exact original benchmark metric.
    apexes = trace[1:-1, 3][(trace[1:-1, 3] > trace[:-2, 3]) &
                              (trace[1:-1, 3] >= trace[2:, 3])]
    rebound_height = float(apexes[0]) if len(apexes) else float("nan")
    metrics = {
        "dt_s": dt, "stiffness_N_m": stiffness, "damping_N_s_m": damping,
        "energy_initial_J": E0, "energy_mse_J2": float(np.mean(err**2)),
        "energy_rmse_J": float(np.sqrt(np.mean(err**2))),
        "max_abs_energy_error_J": float(np.max(np.abs(err))),
        "final_energy_ratio": float(trace[-1, 1] / E0),
        "first_rebound_height_m": rebound_height,
        "first_rebound_height_ratio": rebound_height / HEIGHT,
        "elapsed_s": elapsed, "step_rate_hz": steps / elapsed,
    }
    return metrics, trace


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    # First row preserves the original page's dt. Remaining rows test the
    # stiffness--dt coupling inherent in any compliant-contact discretization.
    cases = [
        # Coarse points, including the original benchmark's 10 ms timestep.
        (0.010, 1e4, 0.0), (0.010, 1e5, 0.0),
        (0.002, 1e4, 0.0), (0.002, 1e5, 0.0),
        (0.0005, 1e5, 0.0), (0.0005, 1e6, 0.0),
        # Local stiffness--timestep sweep around the currently stable region.
        (0.0002, 1e4, 0.0), (0.0002, 3e4, 0.0),
        (0.0002, 1e5, 0.0), (0.0002, 3e5, 0.0),
        (0.0002, 1e6, 0.0),
        (0.0001, 1e5, 0.0), (0.0001, 3e5, 0.0),
        (0.0001, 1e6, 0.0),
    ]
    rows = []
    for index, case in enumerate(cases):
        metrics, trace = run_case(*case)
        rows.append(metrics)
        np.savetxt(OUT / f"energy-trace-case-{index}.csv", trace, delimiter=",",
                   header="time_s,mechanical_energy_J,energy_error_J,ball_0_height_m", comments="")
        print(json.dumps(metrics, indent=2))
    with (OUT / "results.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    (OUT / "metadata.json").write_text(json.dumps({
        "source_task": "leggedrobotics/SimBenchmark bouncing", "balls": N_BALLS,
        "mass_kg": MASS, "radius_m": RADIUS, "height_m": HEIGHT,
        "duration_s": DURATION, "reference": "E(t) = E0; reported metric mean((E-E0)^2)",
        "contact": "MuJoCo direct solref=[-stiffness,-damping], solimp=[.999,.999,.001,.5,2]",
    }, indent=2))


if __name__ == "__main__":
    main()
