#!/usr/bin/env python3
"""SOP-v1 SAP Warp force-matched simple-pinch rerun.

This runner intentionally does not reuse the historical position-compression
labels.  For every candidate and frozen condition it calibrates the finite-gain
pad command to the manifest's *measured* per-pad normal-force target, then
uses the manifest's engine-independent Coulomb label.  The calibration command
is task control, not a B candidate; candidates differ only in SAP parameters.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

ROOT = REPO_ROOT
DEFAULT_MANIFEST = ROOT / "tasks/simulation-only-atomic/simple-pinch/manifests/simple_pinch_force_matched_v1.json"


def load_module(path: Path) -> Any:
    spec = importlib.util.spec_from_file_location("sap_pinch_private", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def candidate_table() -> list[dict[str, Any]]:
    return [
        {"id": "native_default", "ke": None, "tau": None},
        {"id": "ke3e4_tau3ms", "ke": 3e4, "tau": .003},
        {"id": "ke1e5_tau3ms", "ke": 1e5, "tau": .003},
        {"id": "ke1e5_tau1ms", "ke": 1e5, "tau": .001},
        {"id": "ke3e5_tau3ms", "ke": 3e5, "tau": .003},
    ]


def install_pad_surface_variant(pinch: Any) -> None:
    """Permit the manifest's held-out pad-width surface/setup transfer."""
    original = pinch.scene

    def scene(candidate: dict[str, Any], worlds: int, mass: float) -> dict[str, Any]:
        payload = original(candidate, worlds, mass)
        halfwidth = float(candidate.get("pad_halfwidth_y_m", .04))
        for body in payload["bodies"]:
            if body["id"] in ("left", "right"):
                body["shapes"][0]["hy"] = halfwidth
        return payload

    pinch.scene = scene


def calibrate(
    pinch: Any, candidate: dict[str, Any], mass: float, target_n: float,
    output: Path, *, tolerance: float = .05,
) -> dict[str, Any]:
    """Find the position command whose deterministic preload is nearest target."""
    probes: list[dict[str, Any]] = []
    output.mkdir(parents=True, exist_ok=True)
    # This deliberately broad, fixed grid is declared implementation detail;
    # it is identical for candidates and does not use the lift result.
    for index, compression in enumerate(np.linspace(.00001, .00150, 13)):
        config = {**candidate, "compression": float(compression)}
        trial = pinch.run(config, mass, 1, output / f"calibration_{index:02d}")
        probes.append({"compression_m": float(compression), "preload_N": trial["N_mean_preload_N"]})
    chosen = min(probes, key=lambda row: abs(row["preload_N"] - target_n))
    rel_error = abs(chosen["preload_N"] - target_n) / target_n
    return {"commanded_compression_m": chosen["compression_m"], "measured_preload_N": chosen["preload_N"],
            "relative_target_error": rel_error, "within_5pct": rel_error <= tolerance, "probes": probes}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--private-runner", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    if manifest["status"] != "declared; not yet run" or manifest["R"] != 20:
        raise ValueError("this runner requires the unmodified declared R=20 manifest")
    pinch = load_module(args.private_runner)
    install_pad_surface_variant(pinch)
    pinch.wp.init()
    args.output.mkdir(parents=True, exist_ok=True)

    def execute(candidate: dict[str, Any], condition: dict[str, Any], label: str) -> dict[str, Any]:
        pinch.MU = float(condition["mu"])
        pinch.DT = float(condition["dt_ms"]) / 1000.0
        surface = condition["pad_surface"]
        pad_halfwidth = .03 if surface == "narrow-pad" else .04
        base = {**candidate, "pad_halfwidth_y_m": pad_halfwidth}
        out = args.output / label / candidate["id"] / condition["id"]
        out.mkdir(parents=True, exist_ok=True)
        calibration = calibrate(pinch, base, float(condition["mass_kg"]),
                                float(condition["target_per_pad_preload_N"]), out)
        config = {**base, "compression": calibration["commanded_compression_m"]}
        result = pinch.run(config, float(condition["mass_kg"]), 20, out / "R20")
        observed_retain = sum(bool(row["retain"]) for row in result["rollouts"])
        expected_retain = condition.get("expected", "retain") == "retain"
        return {
            "condition": condition, "candidate": candidate, "calibration": calibration,
            "observed_retain_count": observed_retain, "observed_release_count": 20 - observed_retain,
            "expected_retain_count": 20 if expected_retain else 0,
            "expected_release_count": 0 if expected_retain else 20,
            "agreement_count": observed_retain if expected_retain else 20 - observed_retain,
            "result": result,
        }

    selection = manifest["selection"]
    selection_rows = [execute(candidate, selection, "selection") for candidate in candidate_table()]
    viable = [row for row in selection_rows if row["calibration"]["within_5pct"]
              and row["result"]["finite"] and row["result"]["truncated_contacts"] == 0
              and row["result"]["retain_fraction"] >= .95]
    if not viable:
        payload = {"protocol": manifest["protocol"], "task": manifest["task"], "engine": "SAP Warp internal",
                   "manifest": str(args.manifest), "selection": selection_rows, "complete": False,
                   "reason": "no B=5 candidate passed the declared selection gate"}
        (args.output / "summary.json").write_text(json.dumps(payload, indent=2) + "\n")
        raise RuntimeError(payload["reason"])
    winner = min(viable, key=lambda row: (abs(60.0 - row["result"]["lift_follow_mean_mm"]),
                                           row["result"]["hold_creep_mean_um_s"]))
    held = [execute(winner["candidate"], condition, "held_out") for condition in manifest["held_out_conditions"]]
    payload = {"protocol": manifest["protocol"], "task": manifest["task"], "engine": "SAP Warp internal",
               "manifest": str(args.manifest), "R": 20, "selection": selection_rows,
               "frozen_B5": winner["candidate"], "held_out": held, "complete": True}
    (args.output / "summary.json").write_text(json.dumps(payload, indent=2) + "\n")


if __name__ == "__main__":
    main()
