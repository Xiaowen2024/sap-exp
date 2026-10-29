"""Run SAP's frozen simple-pinch B=5 setting on MuJoCo's 17-cell matrix.

The original public SAP runner is not the runner that generated the published
SAP B=5 artifact.  This adapter calls the recovered internal runner without
changing its model, control primitive, or contact parameter semantics.  It
first reproduces the excluded selection point, then evaluates the frozen
winner on the same mass/preload/friction/timestep cells used by MuJoCo Warp.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INTERNAL = Path("/Users/xiaowenyuan/Documents/SAP-WARP-INTERNAL")
DEFAULT_OUT = ROOT / "exp-results" / "protocol-v2-native-default" / "sap-internal-simple-pinch-aligned-r20-20261029-r2"

# Disjoint from selection (mu=.4, mass=.25 kg, compression=.2 mm, dt=.5 ms).
# This is exactly the existing MuJoCo Warp B=5 frozen matrix.  It aligns the
# core physical axes; neither historical engine artifact yet tests geometry or
# surface transfer, so this runner must not be presented as that claim.
CONDITIONS = [
    ("N4_mu0.4_m0.200kg", 0.4, 0.200, 0.0002, 0.0005),
    ("N4_mu0.4_m0.240kg", 0.4, 0.240, 0.0002, 0.0005),
    ("N4_mu0.4_m0.270kg", 0.4, 0.270, 0.0002, 0.0005),
    ("N4_mu0.4_m0.290kg", 0.4, 0.290, 0.0002, 0.0005),
    ("N4_mu0.4_m0.305kg", 0.4, 0.305, 0.0002, 0.0005),
    ("N4_mu0.4_m0.335kg", 0.4, 0.335, 0.0002, 0.0005),
    ("N4_mu0.4_m0.370kg", 0.4, 0.370, 0.0002, 0.0005),
    ("N4_mu0.4_m0.420kg", 0.4, 0.420, 0.0002, 0.0005),
    ("N6_mu0.4_m0.360_retain", 0.4, 0.360, 0.0003, 0.0005),
    ("N6_mu0.4_m0.430_retain", 0.4, 0.430, 0.0003, 0.0005),
    ("N6_mu0.4_m0.520_release", 0.4, 0.520, 0.0003, 0.0005),
    ("N4_mu0.3_m0.200_retain", 0.3, 0.200, 0.0002, 0.0005),
    ("N4_mu0.3_m0.280_release", 0.3, 0.280, 0.0002, 0.0005),
    ("N4_mu0.5_m0.340_retain", 0.5, 0.340, 0.0002, 0.0005),
    ("N4_mu0.5_m0.450_release", 0.5, 0.450, 0.0002, 0.0005),
    ("N4_dt0.25ms_m0.270_retain", 0.4, 0.270, 0.0002, 0.00025),
    ("N4_dt1.0ms_m0.270_retain", 0.4, 0.270, 0.0002, 0.0010),
]


def git_revision(path: Path) -> str | None:
    try:
        return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def write_summary(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n")


def outcome_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    expected_retain = sum(row["expected_retain_count"] for row in rows)
    expected_release = sum(row["expected_release_count"] for row in rows)
    observed_retain = sum(row["observed_retain_count"] for row in rows)
    observed_release = sum(row["observed_release_count"] for row in rows)
    agree = sum(row["agreement_count"] for row in rows)
    return {
        "total_rollouts": expected_retain + expected_release,
        "expected_retain": expected_retain,
        "expected_release": expected_release,
        "observed_retain": observed_retain,
        "observed_release": observed_release,
        "agreement_count": agree,
        "agreement_rate": agree / (expected_retain + expected_release),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sap-internal-root", type=Path, default=DEFAULT_INTERNAL)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    internal = args.sap_internal_root.resolve()
    source = internal / "local_experiments" / "run_simple_pinch_b1_b5.py"
    if not source.is_file():
        raise FileNotFoundError(f"Recovered internal runner is missing: {source}")

    sys.path.insert(0, str(internal / "src"))
    sys.path.insert(0, str(source.parent))
    import warp as wp
    import run_simple_pinch_b1_b5 as pinch

    # The original runner was written for CUDA.  This adapter changes only its
    # device lookup so it can execute the identical solver/model on an
    # available CPU device when CUDA is unavailable.
    original_get_device = pinch.wp.get_device
    pinch.wp.get_device = lambda name: wp.get_device("cpu") if name == "cuda:0" else original_get_device(name)
    wp.init()

    frozen = next(candidate for candidate in pinch.CANDIDATES if candidate["id"] == "ke1e5_tau3ms_c02")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    payload: dict[str, Any] = {
        "protocol": "static-friction-v2/simple-pinch/aligned-core-matrix",
        "complete": False,
        "engine": "SAP Warp internal",
        "engine_warp_version": wp.__version__,
        "internal_runner": str(source),
        "internal_runner_git_revision": git_revision(internal),
        "device": "cpu (CUDA lookup mapped for an otherwise unchanged runner)",
        "frozen_b5": frozen,
        "selection_condition_excluded": {"mu": 0.4, "mass_kg": 0.25, "compression_mm": 0.2, "dt_ms": 0.5, "R": 20},
        "matrix_scope": "Aligned with the existing MuJoCo Warp 17-cell core matrix: mass, compression/preload, friction, and timestep. Geometry/surface transfer is not included for either engine.",
        "conditions": [],
    }

    # Preflight is deliberately re-run in the same environment as the sweep.
    pinch.MU, pinch.DT = 0.4, 0.0005
    preflight_dir = args.output_dir / "selection_preflight"
    preflight_dir.mkdir(parents=True, exist_ok=True)
    preflight = pinch.run(frozen, 0.25, 20, preflight_dir)
    payload["selection_preflight"] = preflight
    if abs(preflight["N_mean_preload_N"] - 3.6363458953564987) > 0.05 or preflight["retain_fraction"] != 1.0 or preflight["truncated_contacts"] != 0:
        write_summary(args.output_dir / "summary.json", payload)
        raise RuntimeError("Selection preflight does not reproduce the published SAP B=5 point")
    write_summary(args.output_dir / "summary.json", payload)

    for condition_id, mu, mass, compression, dt in CONDITIONS:
        pinch.MU, pinch.DT = mu, dt
        case_dir = args.output_dir / condition_id
        case_dir.mkdir(parents=True, exist_ok=True)
        # B=5 freezes SAP's contact parameters, not the task command.  The
        # command compression is an explicit physical axis in this matrix.
        condition_config = {**frozen, "compression": compression}
        result = pinch.run(condition_config, mass, 20, case_dir)
        rollouts = result["rollouts"]
        expected_retain = [rollout["grasp_margin_r"] >= 1.0 for rollout in rollouts]
        observed_retain = [rollout["retain"] for rollout in rollouts]
        row = {
            "condition_id": condition_id,
            "mu": mu,
            "mass_kg": mass,
            "compression_mm": compression * 1e3,
            "dt_ms": dt * 1e3,
            "R": 20,
            "N_mean_preload_N": result["N_mean_preload_N"],
            "grasp_margin_r": result["grasp_margin_r"],
            "hold_creep_mean_um_s": result["hold_creep_mean_um_s"],
            "truncated_contacts": result["truncated_contacts"],
            "finite": result["finite"],
            "expected_retain_count": sum(expected_retain),
            "expected_release_count": 20 - sum(expected_retain),
            "observed_retain_count": sum(observed_retain),
            "observed_release_count": 20 - sum(observed_retain),
            "agreement_count": sum(expected == observed for expected, observed in zip(expected_retain, observed_retain)),
            "rollouts": rollouts,
        }
        payload["conditions"].append(row)
        payload["outcome_summary"] = outcome_summary(payload["conditions"])
        write_summary(args.output_dir / "summary.json", payload)
        print(json.dumps({key: row[key] for key in (
            "condition_id", "N_mean_preload_N", "grasp_margin_r", "expected_retain_count",
            "expected_release_count", "observed_retain_count", "observed_release_count", "agreement_count",
        )}), flush=True)

    payload["complete"] = True
    write_summary(args.output_dir / "summary.json", payload)


if __name__ == "__main__":
    main()
