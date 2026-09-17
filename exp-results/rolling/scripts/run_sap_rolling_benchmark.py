"""SAP Warp counterpart of SimBenchmark's 25-ball rolling test.

The MuJoCo source combines geom friction using ``max``.  SAP combines two
material coefficients harmonically, so the per-shape values below (ground=.25,
box=1, ball=2/3) produce the same *pair* coefficients: box-ground=.4 and
ball-box=.8.  SAP stiffness/tau remain explicit compliant-contact parameters
and are reported rather than treated as a MuJoCo-equivalent setting.
"""
from __future__ import annotations

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

DT, DURATION, FORCE = float(os.environ.get("SAP_ROLLING_DT", ".001")), float(os.environ.get("SAP_ROLLING_DURATION", "4.0")), 150.
DEVICE = os.environ.get("SAP_ROLLING_DEVICE", "cpu")
OUT = WORKSPACE / "exp-results" / "rolling" / os.environ.get("SAP_ROLLING_OUTPUT", "sap")
DEFAULT_KE, DEFAULT_TAU = 10000., .03


@wp.kernel
def apply_box_force(body_f: wp.array(dtype=wp.spatial_vector), body: int, fx: float, fy: float):
    if wp.tid() == 0:
        # Public SAP convention: top three entries are COM force.
        body_f[body] = wp.spatial_vector(fx, fy, 0., 0., 0., 0.)


def reference_velocity(t: np.ndarray, n_balls: int = 25) -> tuple[np.ndarray, np.ndarray]:
    """Original analytic model, generalized only by the number of balls."""
    f_ground = .4 * (10. + n_balls) * 9.81
    f_ball = (FORCE - f_ground) / 10. / (3.5 + n_balls / 10.)
    a_box, a_ball = (FORCE - f_ground - n_balls * f_ball) / 10., f_ball
    direction = np.array([.5, np.sqrt(3.) / 2., 0.])
    return t[:, None] * a_box * direction, t[:, None] * a_ball * direction


def ball_positions(ball_grid: int) -> tuple[float, ...]:
    if ball_grid == 1:
        return (0.,)  # centered: preserve the translation-only analytic model
    if ball_grid == 5:
        return (-4., -2., 0., 2., 4.)
    raise ValueError("ball_grid must be 1 or 5")


def scene_config(dt: float = DT, ke: float = DEFAULT_KE, tau: float = DEFAULT_TAU, ball_grid: int = 5) -> dict:
    bodies, joints, articulations = [], [], []
    bodies.append({"id": "box", "transform": {"p": [0., 0., .499998], "q": [0., 0., 0., 1.]},
                   "shapes": [{"type": "box", "hx": 10., "hy": 10., "hz": .5,
                               "cfg": {"density": .025, "mu": 1.}}]})
    joints.append({"id": "box_free", "type": "free", "parent": "world", "child": "box"})
    articulations.append({"id": "box_articulation", "joints": ["box_free"]})
    for x in ball_positions(ball_grid):
        for y in ball_positions(ball_grid):
            name = f"ball_{x:g}_{y:g}".replace("-", "m")
            bodies.append({"id": name, "transform": {"p": [x, y, 1.499998], "q": [0., 0., 0., 1.]},
                           "shapes": [{"type": "sphere", "radius": .5,
                                       "cfg": {"density": 3. / (4. * np.pi * .5 ** 3), "mu": 2. / 3.}}]})
            joints.append({"id": name + "_free", "type": "free", "parent": "world", "child": name})
            articulations.append({"id": name + "_articulation", "joints": [name + "_free"]})
    return {"schema_version": 1, "name": "simbenchmark_rolling_sap", "simulation": {"dt": dt, "num_worlds": 1, "max_rigid_contact": 64,
            "solver": {"contact_preset_variant": "drake", "line_search_variant": "armijo_decay"}},
            "builder": {"gravity": -9.81, "rigid_gap": 0., "defaults": {"shape": {"density": 1000., "ke": ke, "tau": tau, "mu": .5}}},
            "ground": {"enabled": True, "cfg": {"has_shape_collision": True, "mu": .25}}, "bodies": bodies, "joints": joints, "articulations": articulations}


def run(record: bool, *, dt: float = DT, duration: float = DURATION, ke: float = DEFAULT_KE, tau: float = DEFAULT_TAU, ball_grid: int = 5) -> tuple[np.ndarray, float, int]:
    OUT.mkdir(parents=True, exist_ok=True)
    scene = OUT / "scene.yaml"
    scene.write_text(yaml.safe_dump(scene_config(dt, ke, tau, ball_grid), sort_keys=False))
    loaded = load_sap_scene(scene, device=DEVICE, rigid_contact_max=64, strict=True)
    model, state, control = loaded.sap_model, loaded.sap_state, loaded.sap_control
    nxt = model.state(); solver = SolverSAP(model, max_rigid_contact=64, contact_preset_variant="drake", line_search_variant="armijo_decay")
    pipe = SapCollisionPipeline(loaded.collision_model, rigid_contact_max=64); contacts = pipe.contacts()
    box_body = model.body_label.index("box"); starts = model.joint_qd_start.numpy(); box_joint = model.joint_label.index("box_free")
    selected_ball = "ball_0_0_free" if ball_grid == 1 else "ball_m4_m4_free"
    ball_joint = model.joint_label.index(selected_ball)
    trace, max_contacts = np.zeros((int(duration / dt), 7)), 0
    t0 = time.perf_counter()
    for step in range(len(trace)):
        qd = state.joint_qd.numpy()
        if record: trace[step] = (step * dt, *qd[int(starts[box_joint]):int(starts[box_joint]) + 3], *qd[int(starts[ball_joint]):int(starts[ball_joint]) + 3])
        state.clear_forces(); wp.launch(apply_box_force, dim=1, inputs=[state.body_f, box_body, FORCE * .5, FORCE * np.sqrt(3.) / 2.], device=DEVICE)
        pipe.collide(sap_collision_state_from_state(state), contacts)
        # ``SolverSAP.last_contact_count`` currently reports the allocated
        # per-world matrix size.  The collision counter is the actual count.
        max_contacts = max(max_contacts, int(contacts.rigid_contact_count.numpy()[0]))
        solver.step(state, nxt, control, contacts, dt); state, nxt = nxt, state
    return trace, time.perf_counter() - t0, max_contacts


def main() -> None:
    # Record and time the same trajectory; a second timing-only pass is
    # prohibitively expensive on a CPU-only Warp build and adds no information.
    ball_grid = int(os.environ.get("SAP_ROLLING_BALL_GRID", "5"))
    ke, tau = float(os.environ.get("SAP_ROLLING_KE", DEFAULT_KE)), float(os.environ.get("SAP_ROLLING_TAU", DEFAULT_TAU))
    trace, elapsed, max_contacts = run(True, ke=ke, tau=tau, ball_grid=ball_grid)
    box_ref, ball_ref = reference_velocity(trace[:, 0], ball_grid * ball_grid); error = np.sum((trace[:, 1:4] - box_ref) ** 2 + (trace[:, 4:7] - ball_ref) ** 2, axis=1)
    np.savetxt(OUT / "trace.csv", trace, delimiter=",", header="time_s,box_vx,box_vy,box_vz,ball_vx,ball_vy,ball_vz", comments="")
    report = {"engine": "SAP Warp " + wp.__version__, "device": DEVICE, "dt_s": DT, "duration_s": DURATION, "ball_count": ball_grid * ball_grid, "force_N": FORCE, "force_direction": "xy at 60 deg", "effective_pair_mu": {"box_ground": .4, "ball_box": .8}, "contact_parameters": {"ke": ke, "tau_s": tau, "preset": "drake"}, "mean_squared_velocity_error": float(error.mean()), "step_rate_hz": len(trace) / elapsed, "max_raw_collision_contacts": max_contacts, "contact_capacity": 64, "final_box_velocity_m_s": trace[-1, 1:4].tolist(), "final_ball_velocity_m_s": trace[-1, 4:7].tolist()}
    (OUT / "metrics.json").write_text(json.dumps(report, indent=2) + "\n"); print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
