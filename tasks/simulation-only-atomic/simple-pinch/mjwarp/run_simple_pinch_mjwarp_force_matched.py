#!/usr/bin/env python3
"""SOP-v1 MuJoCo Warp force-matched simple-pinch rerun.

Uses the same manifest and force-control calibration rule as the SAP runner.
The command compression may differ between engines; target measured normal load,
geometry, material, mass, lift command, labels, R, and seeds are shared.
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
DEFAULT_MANIFEST = ROOT / "protocol/manifests/simple_pinch_force_matched_v1.json"
DEFAULT_SOURCE = ROOT / "tasks/simulation-only-atomic/shared/run_static_friction_v2.py"


def source_module(path: Path) -> Any:
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location("static_friction_source", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def candidates() -> list[dict[str, Any]]:
    return [
        {"id": "native_default", "tc": .004, "impratio": 1., "noslip": 0, "native": True},
        {"id": "tc2ms_ir50_ns0", "tc": .002, "impratio": 50., "noslip": 0},
        {"id": "tc1ms_ir50_ns0", "tc": .001, "impratio": 50., "noslip": 0},
        {"id": "tc2ms_ir100_ns0", "tc": .002, "impratio": 100., "noslip": 0},
        {"id": "tc1ms_ir50_ns2", "tc": .001, "impratio": 50., "noslip": 2},
    ]


def json_default(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(type(value).__name__)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    if manifest["status"] != "declared; not yet run" or manifest["R"] != 20:
        raise ValueError("this runner requires the unmodified declared R=20 manifest")
    sim = source_module(args.source)
    args.output.mkdir(parents=True, exist_ok=True)

    def execute(candidate: dict[str, Any], condition: dict[str, Any]) -> dict[str, Any]:
        pad_halfwidth = .03 if condition["pad_surface"] == "narrow-pad" else .04

        def run(compression: float, repeats: int) -> dict[str, Any]:
            xml = sim.simple_pinch_xml(dt=float(condition["dt_ms"]) / 1000., mu=float(condition["mu"]),
                                       mass=float(condition["mass_kg"]), compression_m=compression,
                                       solref_tc=candidate["tc"], impratio=candidate["impratio"],
                                       noslip_iterations=candidate["noslip"])
            xml = xml.replace('size="0.01 0.04 0.04"', f'size="0.01 {pad_halfwidth:g} 0.04"')
            if candidate.get("native"):
                xml = sim.native_default_mjcf(xml)
            return sim.run_simple_pinch_rollouts(backend="mujoco_warp", xml_str=xml, mu=float(condition["mu"]),
                                                 mass=float(condition["mass_kg"]), compression_m=compression,
                                                 dt=float(condition["dt_ms"]) / 1000., n_rollouts=repeats,
                                                 seed_base=int(manifest["seed0"]))

        probes = []
        for compression in np.linspace(.00001, .00150, 13):
            probe = run(float(compression), 1)
            probes.append({"compression_m": float(compression), "preload_N": probe["N_mean_preload_N"]})
        target = float(condition["target_per_pad_preload_N"])
        # Copy the selected probe before retaining the complete probe table;
        # otherwise ``calibration[\"probes\"]`` would contain calibration itself.
        calibration = dict(min(probes, key=lambda row: abs(row["preload_N"] - target)))
        calibration.update({"relative_target_error": abs(calibration["preload_N"] - target) / target,
                            "within_5pct": abs(calibration["preload_N"] - target) / target <= .05,
                            "probes": probes})
        result = run(calibration["compression_m"], 20)
        observed_retain = sum(not bool(row["simulated_positive"]) for row in result["rollouts"])
        expected_retain = condition.get("expected", "retain") == "retain"
        return {"condition": condition, "candidate": candidate, "calibration": calibration,
                "expected_retain_count": 20 if expected_retain else 0,
                "expected_release_count": 0 if expected_retain else 20,
                "observed_retain_count": observed_retain, "observed_release_count": 20 - observed_retain,
                "agreement_count": observed_retain if expected_retain else 20 - observed_retain,
                "result": result}

    selection = manifest["selection"]
    selection_rows = [execute(candidate, selection) for candidate in candidates()]
    viable = [row for row in selection_rows if row["calibration"]["within_5pct"]
              and np.isfinite(row["result"]["E_z_mean_mm"]) and row["result"]["overflow_count"] == 0
              and row["observed_retain_count"] >= 19]
    if not viable:
        payload = {"protocol": manifest["protocol"], "task": manifest["task"], "engine": "MuJoCo Warp",
                   "manifest": str(args.manifest), "selection": selection_rows, "complete": False,
                   "reason": "no B=5 candidate passed the declared selection gate"}
        (args.output / "summary.json").write_text(json.dumps(payload, indent=2, default=json_default) + "\n")
        raise RuntimeError(payload["reason"])
    winner = min(viable, key=lambda row: (row["result"]["E_z_mean_mm"], row["result"]["hold_creep_mean_um_s"]))
    held = [execute(winner["candidate"], condition) for condition in manifest["held_out_conditions"]]
    payload = {"protocol": manifest["protocol"], "task": manifest["task"], "engine": "MuJoCo Warp",
               "manifest": str(args.manifest), "R": 20, "selection": selection_rows,
               "frozen_B5": winner["candidate"], "held_out": held, "complete": True}
    (args.output / "summary.json").write_text(json.dumps(payload, indent=2, default=json_default) + "\n")


if __name__ == "__main__":
    main()
