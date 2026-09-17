"""SAP Warp reproduction of SimBenchmark's 49-ball elastic-bounce test.

Task and metric match ``run_mujoco_bouncing_benchmark.py`` exactly.  SAP uses
its native compliant contact material (ke, tau); no claim is made that either
parameter is numerically equivalent to MuJoCo's solref.
"""
from __future__ import annotations

import csv
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
import warp as wp
import yaml

WORKSPACE = Path(__file__).resolve().parents[3]
SAP_ROOT = WORKSPACE / "sap-sim"
if str(SAP_ROOT) not in sys.path:
    sys.path.insert(0, str(SAP_ROOT))
from sim.collision.pipeline import SapCollisionPipeline
from sim.loader.scene import load_sap_scene
from sim.resources.collision_model import sap_collision_state_from_state
from sim.solver_sap import SolverSAP

MASS, RADIUS, HEIGHT, G, N_SIDE, DURATION = 10.0, .1, 5.0, 9.81, 7, 20.0
N_BALLS, E0, INERTIA = N_SIDE * N_SIDE, N_SIDE * N_SIDE * MASS * G * HEIGHT, .04
DEVICE = os.environ.get("SAP_BOUNCING_DEVICE", "cpu")
OUT = WORKSPACE / "exp-results" / "bouncing" / os.environ.get("SAP_BOUNCING_OUTPUT", "sap-cpu")


def scene_config(dt: float, ke: float, tau: float, preset: str) -> dict:
    bodies, joints, articulations = [], [], []
    density = MASS / ((4.0 / 3.0) * np.pi * RADIUS**3)
    for ix in range(N_SIDE):
        for iy in range(N_SIDE):
            name = f"ball_{ix}_{iy}"
            bodies.append({"id": name, "transform": {"p": [2. * ix, 2. * iy, HEIGHT], "q": [0., 0., 0., 1.]},
                           "shapes": [{"type": "sphere", "radius": RADIUS,
                                       "cfg": {"density": density, "mu": 0.}}]})
            joints.append({"id": name + "_free", "type": "free", "parent": "world", "child": name})
            articulations.append({"id": name + "_articulation", "joints": [name + "_free"]})
    return {"schema_version": 1, "name": "simbenchmark_bouncing_sap",
            "simulation": {"dt": dt, "num_worlds": 1, "max_rigid_contact": 64,
                           "solver": {"contact_preset_variant": preset, "line_search_variant": "armijo_decay"}},
            "builder": {"gravity": -G, "rigid_gap": 0.,
                        "defaults": {"shape": {"density": 1000., "ke": ke, "tau": tau, "mu": 0.}}},
            "ground": {"enabled": True, "cfg": {"has_shape_collision": True, "mu": 0.}},
            "bodies": bodies, "joints": joints, "articulations": articulations}


def run_case(dt: float, ke: float, tau: float, preset: str, duration: float) -> tuple[dict[str, float], np.ndarray]:
    OUT.mkdir(parents=True, exist_ok=True)
    scene_path = OUT / "scene.yaml"
    scene_path.write_text(yaml.safe_dump(scene_config(dt, ke, tau, preset), sort_keys=False))
    loaded = load_sap_scene(scene_path, device=DEVICE, rigid_contact_max=64, strict=True)
    model, state, control = loaded.sap_model, loaded.sap_state, loaded.sap_control
    nxt = model.state()
    solver = SolverSAP(model, max_rigid_contact=64, contact_preset_variant=preset, line_search_variant="armijo_decay")
    pipe = SapCollisionPipeline(loaded.collision_model, rigid_contact_max=64)
    contacts = pipe.contacts()
    starts_q, starts_qd = model.joint_q_start.numpy(), model.joint_qd_start.numpy()
    joint_ids = [model.joint_label.index(f"ball_{i}_{j}_free") for i in range(N_SIDE) for j in range(N_SIDE)]
    steps = round(duration / dt)
    trace, max_contacts = np.empty((steps, 4)), 0
    start = time.perf_counter()
    for step in range(steps):
        q, qd = state.joint_q.numpy(), state.joint_qd.numpy()
        # All free-joint segments are contiguous and identically sized here.
        # Vectorization preserves the original metric while avoiding 49 Python
        # slices/dot products at every timestep.
        q_balls = q.reshape(N_BALLS, 7)
        qd_balls = qd.reshape(N_BALLS, 6)
        energy = (.5 * MASS * float(np.sum(qd_balls[:, :3] ** 2)) +
                  .5 * INERTIA * float(np.sum(qd_balls[:, 3:] ** 2)) +
                  MASS * G * float(np.sum(q_balls[:, 2])))
        first_q = int(starts_q[joint_ids[0]])
        trace[step] = (step * dt, energy, energy - E0, q[first_q + 2])
        state.clear_forces()
        pipe.collide(sap_collision_state_from_state(state), contacts)
        max_contacts = max(max_contacts, int(contacts.rigid_contact_count.numpy()[0]))
        solver.step(state, nxt, control, contacts, dt)
        state, nxt = nxt, state
    elapsed = time.perf_counter() - start
    err = trace[:, 2]
    apexes = trace[1:-1, 3][(trace[1:-1, 3] > trace[:-2, 3]) & (trace[1:-1, 3] >= trace[2:, 3])]
    rebound = float(apexes[0]) if len(apexes) else float("nan")
    return {"preset": preset, "duration_s": duration, "dt_s": dt, "ke_N_m": ke, "tau_s": tau, "energy_initial_J": E0,
            "energy_mse_J2": float(np.mean(err**2)), "energy_rmse_J": float(np.sqrt(np.mean(err**2))),
            "max_abs_energy_error_J": float(np.max(np.abs(err))), "final_energy_ratio": float(trace[-1, 1] / E0),
            "first_rebound_height_m": rebound, "first_rebound_height_ratio": rebound / HEIGHT,
            "max_raw_collision_contacts": max_contacts, "elapsed_s": elapsed, "step_rate_hz": steps / elapsed}, trace


def main() -> None:
    # Syntax: preset:dt:ke:tau. An environment override makes coarse screening
    # and subsequent full-horizon validation reproducible without edits.
    duration = float(os.environ.get("SAP_BOUNCING_DURATION", DURATION))
    raw = os.environ.get("SAP_BOUNCING_CASES", "drake:.001:1e4:0")
    cases = []
    for item in raw.split(","):
        preset, dt, ke, tau = item.split(":")
        cases.append((preset, float(dt), float(ke), float(tau)))
    rows = []
    for i, (preset, dt, ke, tau) in enumerate(cases):
        metrics, trace = run_case(dt, ke, tau, preset, duration); rows.append(metrics)
        np.savetxt(OUT / f"energy-trace-case-{i}.csv", trace, delimiter=",",
                   header="time_s,mechanical_energy_J,energy_error_J,ball_0_height_m", comments="")
        print(json.dumps(metrics, indent=2), flush=True)
    with (OUT / "results.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    (OUT / "metadata.json").write_text(json.dumps({"engine": "SAP Warp " + wp.__version__, "device": DEVICE,
        "source_task": "leggedrobotics/SimBenchmark bouncing", "balls": N_BALLS, "mass_kg": MASS,
        "radius_m": RADIUS, "height_m": HEIGHT, "duration_s": duration,
        "reference": "E(t) = E0; reported metric mean((E-E0)^2)",
        "contact": "SAP native compliant contact; Drake preset; per-shape ke/tau sweep"}, indent=2))


if __name__ == "__main__":
    main()
