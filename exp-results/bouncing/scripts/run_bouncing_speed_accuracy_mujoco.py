"""Speed--accuracy points for bouncing with a fixed MuJoCo contact material."""
from __future__ import annotations

import csv
import json
from pathlib import Path
import sys
import time

import mujoco

sys.path.insert(0, str(Path(__file__).parent))
from run_mujoco_bouncing_benchmark import DURATION, run_case, xml

OUT = Path(__file__).resolve().parents[1] / "mujoco-speed-accuracy"
CASES = (.0005, .001, .002)
STIFFNESS, DAMPING = 1e4, 0.0
TIMING_DURATION = 4.0


def pure_timing(dt: float) -> float:
    model = mujoco.MjModel.from_xml_string(xml(dt, STIFFNESS, DAMPING))
    data = mujoco.MjData(model); mujoco.mj_forward(model, data)
    start = time.perf_counter()
    for _ in range(round(TIMING_DURATION / dt)):
        mujoco.mj_step(model, data)
    return time.perf_counter() - start


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True); rows = []
    for dt in CASES:
        accuracy, _ = run_case(dt, STIFFNESS, DAMPING)
        wall = pure_timing(dt)
        row = {"engine": "MuJoCo", "contact": "solref=[-1e4,0]", "dt_s": dt,
               "accuracy_duration_s": DURATION, "timing_duration_s": TIMING_DURATION, "accuracy_wall_s": accuracy["elapsed_s"],
               "timing_wall_s": wall, "simulated_seconds_per_wall_second": TIMING_DURATION / wall,
               "energy_mse_J2": accuracy["energy_mse_J2"],
               "first_rebound_height_m": accuracy["first_rebound_height_m"],
               "final_energy_ratio": accuracy["final_energy_ratio"]}
        rows.append(row); print(json.dumps(row), flush=True)
    with (OUT / "results.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


if __name__ == "__main__": main()
