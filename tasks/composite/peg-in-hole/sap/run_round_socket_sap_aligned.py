#!/usr/bin/env python3
"""Batched frozen-B=5 generalizability evaluation for the round socket task.

This runner evaluates one declared held-out condition in 20 replicated SAP
worlds.  Socket geometry, duration, perturbations, and credibility gates match
the paired MuJoCo Warp evaluation; only SAP-native contact settings differ.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import warp as wp
import yaml


ROOT = Path(__file__).resolve().parents[2]
SAP_ROOT = Path("/home/xw/sim-exp-gpu/sap-sim-round")
if SAP_ROOT.exists():
    sys.path.insert(0, str(SAP_ROOT))
else:
    sys.path.insert(0, str(ROOT / "sap-sim"))

from sim.collision.pipeline import SapCollisionPipeline
from sim.loader.scene import load_sap_scene
from sim.resources.collision_model import sap_collision_state_from_state
from sim.solver_sap import SolverSAP


R, DURATION_S, CONTACT_CAP = 20, 2.0, 2048
FROZEN = {
    "id": "pair_k1e5_tau3ms_gap0p1",
    "pair_ke_N_m": 1e5,
    "pair_tau_ms": 3.0,
    "rigid_gap_mm": 0.1,
    "preset": "drake",
}


def copy_values(target: wp.array, values: np.ndarray) -> None:
    wp.copy(target, wp.array(np.asarray(values), dtype=target.dtype, device=target.device))


def build_scene(asset_dir: Path, clearance_mm: float, offset_mm: float, tilt_deg: float,
                release_height_mm: float, dt_ms: float) -> dict:
    # Import the existing shared-geometry scene builder after its asset location
    # has been set explicitly for the execution host.
    import importlib.util

    matrix = SAP_ROOT / "scripts" / "run_round_peg_socket_matrix.py"
    spec = importlib.util.spec_from_file_location("round_matrix", matrix)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    module.PIECES = asset_dir
    module.TILT_DEG = tilt_deg
    scene = module.material_scene(.012 - clearance_mm / 1000.0, offset_mm / 1000.0,
                                  "coacd_shared", Path("unused.obj"))
    scene["simulation"]["dt"] = dt_ms / 1000.0
    scene["simulation"]["solver"]["contact_preset_variant"] = FROZEN["preset"]
    scene["builder"]["rigid_gap"] = FROZEN["rigid_gap_mm"] / 1000.0
    shape = scene["builder"]["defaults"]["shape"]
    shape["ke"] = 2.0 * FROZEN["pair_ke_N_m"]
    shape["tau"] = 0.5 * FROZEN["pair_tau_ms"] / 1000.0
    scene["bodies"][1]["transform"]["p"][2] = release_height_mm / 1000.0
    return scene


def run(condition: dict, asset_dir: Path, output_dir: Path, device: str,
        repeat_index: int | None = None) -> dict:
    worlds = 1 if repeat_index is not None else R
    scene = build_scene(asset_dir, **{k: condition[k] for k in
                                      ("clearance_mm", "offset_mm", "tilt_deg",
                                       "release_height_mm", "dt_ms")})
    scene_path = output_dir / f"{condition['id']}.yaml"
    scene_path.write_text(yaml.safe_dump(scene, sort_keys=False))
    loaded = load_sap_scene(scene_path, device=device, rigid_contact_max=CONTACT_CAP,
                            strict=True, num_worlds=worlds, spacing=(.25, .25, .25))
    model, state, control = loaded.sap_model, loaded.sap_state, loaded.sap_control
    next_state = model.state()
    solver = SolverSAP(model, max_rigid_contact=CONTACT_CAP,
                       contact_preset_variant=FROZEN["preset"], line_search_variant="armijo_decay")
    pipeline = SapCollisionPipeline(loaded.collision_model, rigid_contact_max=CONTACT_CAP)
    contacts = pipeline.contacts()
    local_joint_count = len(model.joint_label) // worlds
    peg_joint = model.joint_label[:local_joint_count].index("peg_free")
    starts = model.joint_q_start.numpy()
    q_start = int(starts[peg_joint])
    q_stride = int(model.joint_q.shape[0] // worlds)
    local_body_count = len(model.body_label) // worlds
    peg_body = model.body_label[:local_body_count].index("peg")
    initial = state.joint_q.numpy().copy()
    seed0 = int(condition["seed0"])
    rng = np.random.default_rng(seed0)
    perturbed = initial.copy()
    schedule = rng.uniform(-.05, .05, (R, 2))
    perturb_mm = schedule if repeat_index is None else schedule[repeat_index:repeat_index + 1]
    for world in range(worlds):
        base = world * q_stride + q_start
        perturbed[base:base + 2] += perturb_mm[world] / 1000.0
    copy_values(state.joint_q, perturbed)
    # The contact pipeline starts from body transforms, whereas the free-joint
    # state is the solver's generalized coordinate.  Keep those two initial
    # representations synchronized when applying the declared perturbation.
    body_q = state.body_q.numpy().copy()
    for world in range(worlds):
        body = world * local_body_count + peg_body
        body_q[body, :2] += perturb_mm[world] / 1000.0
    copy_values(state.body_q, body_q)

    min_phi = np.full(worlds, np.inf)
    final_phi = np.full(worlds, np.inf)
    max_contacts = np.zeros(worlds, dtype=int)
    truncated = 0
    for _ in range(round(DURATION_S / (condition["dt_ms"] / 1000.0))):
        state.clear_forces()
        pipeline.collide(sap_collision_state_from_state(state), contacts)
        max_contacts = np.maximum(max_contacts, contacts.rigid_contact_count.numpy().astype(int))
        solver.step(state, next_state, control, contacts, condition["dt_ms"] / 1000.0)
        counts = solver.contact_jacobian.contact_env_count_wp.numpy().astype(int)
        phi = solver.contact_jacobian.contact_env_phi0_wp.numpy()
        for world, count in enumerate(counts):
            if count:
                value = float(phi[world, :count].min())
                min_phi[world] = min(min_phi[world], value)
                final_phi[world] = value
        truncated = max(truncated, int(solver.last_truncated_contact_count))
        state, next_state = next_state, state

    final_q = state.joint_q.numpy()
    clearance = condition["clearance_mm"]
    rows = []
    for world in range(worlds):
        base = world * q_stride + q_start
        # Replication translates every world.  Subtract the translation inferred
        # from the unperturbed starting pose to recover socket-relative position.
        origin_xy = initial[base:base + 2] - np.array([condition["offset_mm"] / 1000.0, 0.0])
        x, y, z = final_q[base:base + 3] - np.array([origin_xy[0], origin_xy[1], 0.0])
        radial_mm = float(np.hypot(x, y) * 1000.0)
        pmax_mm = float(max(0.0, -min_phi[world]) * 1000.0) if np.isfinite(min_phi[world]) else 0.0
        pfinal_mm = float(max(0.0, -final_phi[world]) * 1000.0) if np.isfinite(final_phi[world]) else 0.0
        entered = bool(0.0 <= z <= .045 and radial_mm <= 3.0)
        escaped = bool(radial_mm > 50.0 or z < -.01 or z > .16)
        credible = bool(entered and radial_mm <= clearance and not escaped and truncated == 0
                        and pmax_mm <= .1 * clearance and pfinal_mm <= .1 * clearance)
        row_seed = seed0 + (world if repeat_index is None else repeat_index)
        rows.append({"seed": row_seed, "perturb_xy_mm": perturb_mm[world].tolist(),
                     "final_xyz_mm": [float(x * 1000), float(y * 1000), float(z * 1000)],
                     "final_radial_mm": radial_mm, "entered": entered, "escaped": escaped,
                     "pmax_mm": pmax_mm, "pfinal_mm": pfinal_mm,
                     "pmax_over_clearance": pmax_mm / clearance,
                     "truncated_contacts": truncated, "credible": credible})
    return {"protocol": "aligned-frozen-generalizability-v1", "task": "round-socket",
            "engine": "SAP Warp", "B": 5, "frozen_candidate": FROZEN,
            "condition": condition, "R": worlds, "repeat_index": repeat_index,
            "duration_s": DURATION_S,
            "credible_count": sum(row["credible"] for row in rows),
            "credible_rate": float(np.mean([row["credible"] for row in rows])),
            "pmax_over_clearance_mean": float(np.mean([row["pmax_over_clearance"] for row in rows])),
            "final_radial_mm_mean": float(np.mean([row["final_radial_mm"] for row in rows])),
            "max_contacts_per_world": max_contacts.tolist(), "rollouts": rows}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--condition", required=True, type=Path)
    parser.add_argument("--assets", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--repeat-index", type=int, choices=range(R))
    args = parser.parse_args()
    condition = json.loads(args.condition.read_text())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(run(condition, args.assets, args.output.parent, args.device,
                                          args.repeat_index), indent=2) + "\n")


if __name__ == "__main__":
    main()
