#!/usr/bin/env python3
"""Private-SAP-runtime round-socket frozen-setting evaluation.

This is intentionally separate from the public ``sim`` implementation: the
published SAP round-socket artifact was made with the internal ``sap_warp``
runtime.  It uses that same API family and performs a selection-point
preflight before any held-out result is accepted.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import warp as wp
import yaml


ROOT = Path("/home/xw/sim-exp-gpu")
PRIVATE = ROOT / "SAP-WARP-INTERNAL"
sys.path.insert(0, str(PRIVATE / "src"))

from sap_warp.collision import SapCollisionPipeline
from sap_warp.loader import load_sap_scene
from sap_warp.sap_kinematics import sap_eval_fk
from sap_warp.solver_sap import SolverSAP


R, DURATION_S, NCONMAX = 20, 2.0, 2048
FROZEN = {"id": "pair_k1e5_tau3ms_gap0p1", "ke": 200000.0, "tau": 0.0015,
          "dt": 0.00025, "gap": 0.0001}


def scene(pieces: Path, condition: dict) -> dict:
    clearance = float(condition["clearance_mm"]) / 1000.0
    offset = float(condition["offset_mm"]) / 1000.0
    return {
        "schema_version": 1, "name": "round_socket_internal_aligned",
        "simulation": {"dt": float(condition["dt_ms"]) / 1000.0,
                       "num_worlds": R, "nconmax": NCONMAX},
        "builder": {"gravity": -9.81, "rigid_gap": FROZEN["gap"],
                    "defaults": {"shape": {"density": 1000.0, "mu": 0.8,
                                               "ke": FROZEN["ke"], "tau": FROZEN["tau"]}}},
        "ground": {"enabled": False},
        "bodies": [
            {"id": "socket", "shapes": [
                {"type": "convex_mesh", "path": str(p), "is_solid": True}
                for p in sorted(pieces.glob("socket_piece_*.obj"))]},
            {"id": "peg", "transform": {"p": [offset, 0.0, float(condition["release_height_mm"]) / 1000.0],
                                              "q": "identity"},
             "mass": .08, "shapes": [{"type": "cylinder", "radius": .012 - clearance, "half_height": .025}]},
        ],
        "joints": [{"id": "peg_free", "type": "free", "parent": "world", "child": "peg"}],
        "articulations": [{"id": "peg_art", "joints": ["peg_free"]}],
    }


def run(condition: dict, pieces: Path, output: Path, device: str) -> dict:
    output.parent.mkdir(parents=True, exist_ok=True)
    yaml_path = output.with_suffix(".yaml")
    yaml_path.write_text(yaml.safe_dump(scene(pieces, condition), sort_keys=False))
    loaded = load_sap_scene(yaml_path, device=wp.get_device(device), naconmax=NCONMAX,
                            strict=True, num_worlds=R)
    model, state, control = loaded.sap_model, loaded.sap_state, loaded.sap_control
    other = model.state()
    pipeline = SapCollisionPipeline(loaded.collision_model, naconmax=NCONMAX, bounds_filter="sphere_obb")
    contacts = pipeline.contacts()
    solver = SolverSAP(model, nconmax=NCONMAX, enable_sleep=False)
    q = state.joint_q.numpy().reshape(R, 7)
    rng = np.random.default_rng(int(condition["seed0"]))
    perturb = rng.uniform(-.00005, .00005, size=(R, 2))
    q[:, :2] += perturb
    angle = math.radians(float(condition["tilt_deg"])) / 2.0
    q[:, 3:7] = [math.cos(angle), 0.0, math.sin(angle), 0.0]
    state.joint_q.assign(q.reshape(-1))
    sap_eval_fk(model, state.joint_q, state.joint_qd, state)

    min_phi = np.full(R, np.inf)
    final_phi = np.full(R, np.inf)
    max_candidates = np.zeros(R, dtype=int)
    for _ in range(round(DURATION_S / (float(condition["dt_ms"]) / 1000.0))):
        state.clear_forces()
        solver.collide(pipeline, contacts, state, control, collision_state=state)
        solver.step(state, other, control, contacts, float(condition["dt_ms"]) / 1000.0)
        jac = solver.last_contact_jacobian_result
        if jac is not None:
            counts = jac.contact_env_count.numpy().astype(int)
            phi = jac.contact_env_phi0.numpy()
            max_candidates = np.maximum(max_candidates, counts)
            for world, count in enumerate(counts):
                if count:
                    value = float(phi[world, :count].min())
                    min_phi[world] = min(min_phi[world], value)
                    final_phi[world] = value
        state, other = other, state
    pipeline.check_buffers(contacts)
    q = state.joint_q.numpy().reshape(R, 7)
    clearance_mm = float(condition["clearance_mm"])
    rows = []
    for world in range(R):
        x, y, z = (float(v) for v in q[world, :3])
        radial = math.hypot(x, y) * 1000.0
        pmax = max(0.0, -min_phi[world]) * 1000.0 if np.isfinite(min_phi[world]) else 0.0
        pfinal = max(0.0, -final_phi[world]) * 1000.0 if np.isfinite(final_phi[world]) else 0.0
        entered = bool(0.0 <= z <= .045 and radial <= 3.0)
        escaped = bool(radial > 50.0 or z < -.01 or z > .16)
        trunc = int(solver.last_truncated_contact_count or 0)
        credible = bool(entered and radial <= clearance_mm and not escaped and trunc == 0
                        and pmax <= .1 * clearance_mm and pfinal <= .1 * clearance_mm)
        rows.append({"seed": int(condition["seed0"]) + world,
                     "perturb_xy_mm": (perturb[world] * 1000.0).tolist(),
                     "final_xyz_mm": [x * 1000.0, y * 1000.0, z * 1000.0],
                     "final_radial_mm": radial, "entered": entered, "escaped": escaped,
                     "pmax_phi0_mm": pmax, "pfinal_phi0_mm": pfinal,
                     "pmax_over_clearance": pmax / clearance_mm,
                     "max_candidates": int(max_candidates[world]), "truncated_contacts": trunc,
                     "credible": credible})
    payload = {"protocol": "aligned-frozen-generalizability-v1", "task": "round-socket",
               "engine": "SAP Warp internal", "B": 5, "frozen_candidate": FROZEN,
               "condition": condition, "R": R, "duration_s": DURATION_S,
               "credible_count": sum(row["credible"] for row in rows),
               "credible_rate": float(np.mean([row["credible"] for row in rows])),
               "pmax_over_clearance_mean": float(np.mean([row["pmax_over_clearance"] for row in rows])),
               "final_radial_mm_mean": float(np.mean([row["final_radial_mm"] for row in rows])),
               "truncated_contacts": int(solver.last_truncated_contact_count or 0), "rollouts": rows}
    output.write_text(json.dumps(payload, indent=2) + "\n")
    print(json.dumps({key: payload[key] for key in ("credible_count", "credible_rate",
                                                     "pmax_over_clearance_mean", "final_radial_mm_mean")}), flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--condition", type=Path, required=True)
    parser.add_argument("--pieces", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    wp.init()
    run(json.loads(args.condition.read_text()), args.pieces, args.output, args.device)


if __name__ == "__main__":
    main()
