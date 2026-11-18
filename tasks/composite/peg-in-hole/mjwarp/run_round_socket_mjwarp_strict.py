#!/usr/bin/env python3
"""Strict-v2 MuJoCo Warp round-socket rerun from a common manifest."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import importlib.util
import json
import re
import sys
from pathlib import Path
from typing import Any

ROOT = REPO_ROOT


def load(path: Path) -> Any:
    spec = importlib.util.spec_from_file_location("round_mjwarp_source", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def candidates() -> list[dict[str, Any]]:
    return [
        {"id": "native_default", "native": True},
        {"id": "tc0p75ms_dr1_d95", "tc": .00075, "dr": 1., "d": .95},
        {"id": "tc1ms_dr1_d95", "tc": .001, "dr": 1., "d": .95},
        {"id": "tc2ms_dr2_d95", "tc": .002, "dr": 2., "d": .95},
        {"id": "tc4ms_dr2_d99", "tc": .004, "dr": 2., "d": .99},
    ]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text())
    if manifest["status"] != "declared; not yet run" or manifest["R"] != 20:
        raise ValueError("requires the unmodified strict-v2 manifest")
    mod = load(args.source)
    mod.R = 20
    args.output.mkdir(parents=True, exist_ok=True)
    active: dict[str, Any] = {}
    original_xml = mod.xml

    def xml(candidate: dict[str, Any], clearance: float, offset: float) -> str:
        condition = active
        actual = dict(candidate)
        if not candidate.get("native"):
            actual["dt"] = float(condition["dt_ms"]) / 1000.
        text = original_xml(actual, clearance, offset)
        mu = float(condition["mu"])
        text = text.replace('friction=".8 .005 .0001"', f'friction="{mu:g} .005 .0001"')
        x = float(condition["offset_mm"]) / 1000.
        z = float(condition["release_height_mm"]) / 1000.
        tilt = float(condition["tilt_deg"])
        text = re.sub(r'<body name="peg" pos="[^"]+" euler="[^"]+"',
                      f'<body name="peg" pos="{x:g} 0 {z:g}" euler="0 {tilt:g} 0"', text)
        return text

    mod.xml = xml

    def run(candidate: dict[str, Any], condition: dict[str, Any]) -> dict[str, Any]:
        nonlocal active
        active = condition
        row = mod.run(candidate, float(condition["clearance_mm"]), float(condition["offset_mm"]),
                      int(manifest["seed0"]) + len(condition["id"]) * 100)
        row["condition"] = condition
        return row

    selection = manifest["selection"]
    screen = [run(candidate, selection) for candidate in candidates()]
    eligible = [row for row in screen if row["overflow_worlds"] == 0]
    winner = min(eligible, key=lambda row: (-row["credible_rate"], row["pmax_over_clearance_mean"],
                                             row["final_radial_mm_mean"]))
    frozen = next(candidate for candidate in candidates() if candidate["id"] == winner["candidate"]["id"])
    held = [run(frozen, condition) for condition in manifest["held_out_conditions"]]
    payload = {"protocol": manifest["protocol"], "task": "round-socket", "engine": "MuJoCo Warp",
               "manifest": str(args.manifest), "R": 20, "selection": screen,
               "frozen_B5": frozen, "held_out": held, "complete": True}
    (args.output / "summary.json").write_text(json.dumps(payload, indent=2) + "\n")


if __name__ == "__main__":
    main()
