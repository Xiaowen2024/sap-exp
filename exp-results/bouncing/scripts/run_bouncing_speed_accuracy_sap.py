"""Speed--accuracy points for bouncing with fixed SAP material and preset."""
from __future__ import annotations

import csv
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).parent))
import run_sap_bouncing_benchmark as bounce
from sim.collision.pipeline import SapCollisionPipeline
from sim.loader.scene import load_sap_scene
from sim.resources.collision_model import sap_collision_state_from_state
from sim.solver_sap import SolverSAP
import yaml

OUT = Path(__file__).resolve().parents[1] / "sap-speed-accuracy"
CASES = (.0005, .001, .002)
PRESET, KE, TAU, DURATION = "approx32", 1e4, 0.0, bounce.DURATION
TIMING_DURATION = 4.0


def pure_timing(dt: float) -> float:
    scene = OUT / "timing-scene.yaml"
    scene.write_text(yaml.safe_dump(bounce.scene_config(dt, KE, TAU, PRESET), sort_keys=False))
    loaded = load_sap_scene(scene, device=bounce.DEVICE, rigid_contact_max=64, strict=True)
    model, state, control = loaded.sap_model, loaded.sap_state, loaded.sap_control
    nxt = model.state(); solver = SolverSAP(model, max_rigid_contact=64, contact_preset_variant=PRESET, line_search_variant="armijo_decay")
    pipe = SapCollisionPipeline(loaded.collision_model, rigid_contact_max=64); contacts = pipe.contacts()
    start = time.perf_counter()
    for _ in range(round(TIMING_DURATION / dt)):
        state.clear_forces(); pipe.collide(sap_collision_state_from_state(state), contacts)
        solver.step(state, nxt, control, contacts, dt); state, nxt = nxt, state
    return time.perf_counter() - start


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True); rows = []
    for dt in CASES:
        accuracy, _ = bounce.run_case(dt, KE, TAU, PRESET, DURATION)
        wall = pure_timing(dt)
        row = {"engine": "SAP Warp", "preset": PRESET, "ke_N_m": KE, "tau_s": TAU,
               "dt_s": dt, "accuracy_duration_s": DURATION, "timing_duration_s": TIMING_DURATION, "accuracy_wall_s": accuracy["elapsed_s"],
               "timing_wall_s": wall, "simulated_seconds_per_wall_second": TIMING_DURATION / wall,
               "energy_mse_J2": accuracy["energy_mse_J2"],
               "first_rebound_height_m": accuracy["first_rebound_height_m"],
               "final_energy_ratio": accuracy["final_energy_ratio"],
               "max_raw_collision_contacts": accuracy["max_raw_collision_contacts"]}
        rows.append(row)
        with (OUT / "results.partial.csv").open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(row)); writer.writeheader(); writer.writerows(rows)
        print(json.dumps(row), flush=True)
    (OUT / "results.csv").write_text((OUT / "results.partial.csv").read_text())


if __name__ == "__main__": main()
