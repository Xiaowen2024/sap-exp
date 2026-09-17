"""CPU A/B/C sweep for SAP's Drake-preset rolling contact model.

The default is a centred one-ball smoke benchmark.  Its analytic reference is
recomputed with n=1, so its MSE is intentionally not comparable to the
published 25-ball SimBenchmark score.  Use ``--ball-grid 5`` for that task.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from run_sap_rolling_benchmark import DEFAULT_KE, DEFAULT_TAU, DEVICE, DURATION, reference_velocity, run

ROOT = Path(__file__).resolve().parents[1]


def evaluate(section: str, dt: float, ke: float, tau: float, ball_grid: int) -> dict:
    trace, elapsed, max_contacts = run(True, dt=dt, ke=ke, tau=tau, ball_grid=ball_grid)
    box_ref, ball_ref = reference_velocity(trace[:, 0], ball_grid * ball_grid)
    mse = np.sum((trace[:, 1:4] - box_ref) ** 2 + (trace[:, 4:7] - ball_ref) ** 2, axis=1).mean()
    return {"section": section, "ball_count": ball_grid * ball_grid, "preset": "drake", "device": DEVICE,
            "dt_s": dt, "ke": ke, "tau_s": tau, "duration_s": DURATION, "steps": len(trace),
            "mse_velocity_error": float(mse), "elapsed_s": elapsed, "step_rate_hz": len(trace) / elapsed,
            "max_raw_collision_contacts": max_contacts}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ball-grid", type=int, choices=(1, 5), default=1)
    parser.add_argument("--section", choices=("a", "b", "c"))
    args = parser.parse_args()
    if DEVICE != "cpu":
        raise RuntimeError(f"This prescribed sweep is CPU-only; got SAP_ROLLING_DEVICE={DEVICE!r}")
    cases = []
    if args.section in (None, "a"):
        cases += [("A_dt", dt, DEFAULT_KE, DEFAULT_TAU) for dt in (.0005, .001, .002, .005)]
    if args.section in (None, "b"):
        cases += [("B_tau", .001, DEFAULT_KE, tau) for tau in (.004, .010, .030, .060)]
    if args.section in (None, "c"):
        cases += [("C_ke", .001, ke, DEFAULT_TAU) for ke in (1e3, 1e4, 1e5)]
    out = ROOT / ("sap-cpu-one-ball-drake-sweep" if args.ball_grid == 1 else "sap-cpu-25ball-drake-sweep")
    out.mkdir(parents=True, exist_ok=True)
    result_path = out / "results.csv"
    previous = []
    if result_path.exists():
        with result_path.open(newline="") as f:
            previous = list(csv.DictReader(f))
    done = {(row["section"], float(row["dt_s"]), float(row["ke"]), float(row["tau_s"])) for row in previous}
    rows = []
    pending = [case for case in cases if case not in done]
    for index, (section, dt, ke, tau) in enumerate(pending, 1):
        row = evaluate(section, dt, ke, tau, args.ball_grid)
        rows.append(row)
        # Persist each finished case: an interrupted multi-hour CPU sweep can
        # resume without discarding completed work.
        with result_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(row)); writer.writeheader(); writer.writerows(previous + rows)
        print(f"{index}/{len(pending)} {json.dumps(row)}", flush=True)
    metadata = {"purpose": "SAP Drake preset CPU A/B/C sweep", "ball_grid": args.ball_grid,
                "sections": {"A": "dt", "B": "tau at dt=1ms, ke=1e4", "C": "ke at dt=1ms, tau=30ms"}}
    if args.ball_grid == 1:
        metadata["warning"] = "one-ball values use a separately recomputed n=1 analytic reference and are not comparable to 25-ball MSE"
    (out / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    main()
