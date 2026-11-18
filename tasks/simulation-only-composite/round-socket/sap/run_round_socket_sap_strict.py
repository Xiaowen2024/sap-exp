#!/usr/bin/env python3
"""Strict-v2 SAP Warp round-socket rerun using the recovered private runner."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import importlib.util
import json
import math
import sys
from pathlib import Path
from typing import Any

ROOT = REPO_ROOT


def load(path: Path) -> Any:
    spec = importlib.util.spec_from_file_location("round_sap_source", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def candidates() -> list[dict[str, Any]]:
    return [
        {"id": "native_default", "ke": None, "tau": None, "dt": None, "gap": None},
        {"id": "ke2e4_tau1p5ms_gap0p1", "ke": 2e4, "tau": .0015, "dt": None, "gap": .0001},
        {"id": "ke6e4_tau1p5ms_gap0p1", "ke": 6e4, "tau": .0015, "dt": None, "gap": .0001},
        {"id": "ke2e5_tau1p5ms_gap0p1", "ke": 2e5, "tau": .0015, "dt": None, "gap": .0001},
        {"id": "ke2e4_tau5ms_gap0p1", "ke": 2e4, "tau": .005, "dt": None, "gap": .0001},
    ]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--private-runner", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    if manifest["status"] != "declared; not yet run" or manifest["R"] != 20:
        raise ValueError("requires the unmodified strict-v2 manifest")
    mod = load(args.private_runner)
    mod.R = 20
    mod.OUT = args.output / "raw"
    args.output.mkdir(parents=True, exist_ok=True)
    active: dict[str, Any] = {}
    original_scene = mod.scene

    def scene(candidate: dict[str, Any], clearance: float, offset: float, worlds: int) -> dict[str, Any]:
        condition = active
        payload = original_scene(candidate, clearance, offset, worlds)
        payload["simulation"]["dt"] = float(condition["dt_ms"]) / 1000.
        payload["builder"]["defaults"]["shape"]["mu"] = float(condition["mu"])
        peg = next(body for body in payload["bodies"] if body["id"] == "peg")
        tilt = math.radians(float(condition["tilt_deg"])) / 2.
        peg["transform"] = {"p": [float(condition["offset_mm"]) / 1000., 0.,
                                   float(condition["release_height_mm"]) / 1000.],
                            "q": [0., math.sin(tilt), 0., math.cos(tilt)]}
        peg["shapes"][0]["radius"] = (12. - float(condition["clearance_mm"])) / 1000.
        return payload

    mod.scene = scene
    mod.wp.init()

    def run(candidate: dict[str, Any], condition: dict[str, Any]) -> dict[str, Any]:
        nonlocal active
        active = condition
        row = mod.run(candidate, float(condition["clearance_mm"]), float(condition["offset_mm"]),
                      int(manifest["seed0"]) + len(condition["id"]) * 100)
        row["condition"] = condition
        return row

    selection = manifest["selection"]
    screen = [run(candidate, selection) for candidate in candidates()]
    eligible = [row for row in screen if row["truncated_contacts"] == 0]
    winner = min(eligible, key=lambda row: (-row["credible_rate"], row["pmax_over_clearance_mean"],
                                             row["final_radial_mm_mean"]))
    frozen = next(candidate for candidate in candidates() if candidate["id"] == winner["candidate"]["id"])
    held = [run(frozen, condition) for condition in manifest["held_out_conditions"]]
    payload = {"protocol": manifest["protocol"], "task": "round-socket", "engine": "SAP Warp internal",
               "manifest": str(args.manifest), "R": 20, "selection": screen,
               "frozen_B5": frozen, "held_out": held, "complete": True}
    (args.output / "summary.json").write_text(json.dumps(payload, indent=2) + "\n")


if __name__ == "__main__":
    main()
