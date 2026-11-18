#!/usr/bin/env python3
"""Run the declared common frozen B=5 matrix with the private SAP runner.

The private runner is imported at runtime and is not copied into this public
repository.  This wrapper changes only physical evaluation variables; it
never re-runs selection or changes the already frozen SAP contact parameters.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Any


# Each non-selection cell isolates one declared axis about a held-out 5-mm
# stack.  Values and seeds are shared verbatim with the MuJoCo Warp wrapper.
CASES: list[dict[str, Any]] = [
    {"id": "thickness_0p5mm", "h_m": .0005, "mass_kg": 1/3, "radius_m": .1, "offset_m": .0, "dt_s": .0005},
    {"id": "thickness_5mm",   "h_m": .005,  "mass_kg": 1/3, "radius_m": .1, "offset_m": .0, "dt_s": .0005},
    {"id": "thickness_20mm",  "h_m": .020,  "mass_kg": 1/3, "radius_m": .1, "offset_m": .0, "dt_s": .0005},
    {"id": "mass_0p167kg",    "h_m": .005,  "mass_kg": 1/6, "radius_m": .1, "offset_m": .0, "dt_s": .0005},
    {"id": "mass_0p667kg",    "h_m": .005,  "mass_kg": 2/3, "radius_m": .1, "offset_m": .0, "dt_s": .0005},
    {"id": "radius_75mm",     "h_m": .005,  "mass_kg": 1/3, "radius_m": .075,"offset_m": .0, "dt_s": .0005},
    {"id": "radius_125mm",    "h_m": .005,  "mass_kg": 1/3, "radius_m": .125,"offset_m": .0, "dt_s": .0005},
    {"id": "offset_0p5mm",    "h_m": .005,  "mass_kg": 1/3, "radius_m": .1, "offset_m": .0005, "dt_s": .0005},
    {"id": "offset_1mm",      "h_m": .005,  "mass_kg": 1/3, "radius_m": .1, "offset_m": .001, "dt_s": .0005},
    {"id": "dt_0p25ms",       "h_m": .005,  "mass_kg": 1/3, "radius_m": .1, "offset_m": .0, "dt_s": .00025},
    {"id": "dt_1ms",          "h_m": .005,  "mass_kg": 1/3, "radius_m": .1, "offset_m": .0, "dt_s": .001},
]


def load(path: Path):
    spec = importlib.util.spec_from_file_location("private_threeplate", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--repeats", type=int, default=20)
    args = ap.parse_args()
    if args.repeats != 20:
        raise ValueError("aligned protocol requires R=20")
    m = load(args.source)
    m.wp.init()
    frozen = {"id": "shape_ke1e6_tau3ms", "shape_ke_N_m": 1e6, "shape_tau_s": .003}
    original_scene = m.scene
    args.out.mkdir(parents=True, exist_ok=True)
    rows = []
    for i, case in enumerate(CASES):
        m.MASS, m.RADIUS = case["mass_kg"], case["radius_m"]
        offset = case["offset_m"]
        def shifted_scene(c, h, dt, worlds, *, _offset=offset):
            scene = original_scene(c, h, dt, worlds)
            # A deterministic held-out placement perturbation: bottom stays
            # centered; middle/top are shifted by one/two declared offsets.
            for body, scale in zip(scene["bodies"], (0., 1., 2.)):
                body["transform"]["p"][0] += scale * _offset
            return scene
        m.scene = shifted_scene
        result = m.run_case(frozen, case["h_m"], case["dt_s"], args.repeats,
                            41000 + i, args.out, args.device)
        result["condition"] = case
        rows.append(result)
        print(case["id"], result["pass_fraction"], result["max_overlap_mm_mean"],
              result["normal_load_error_max_mean"], flush=True)
    payload = {
        "protocol": "aligned-frozen-generalizability-v1",
        "task": "three-plate-stack", "engine": "SAP Warp internal",
        "B": 5, "R_per_condition": args.repeats, "frozen_candidate": frozen,
        "matrix_design": "axis-isolated held-out cells; nominal selection h=4mm excluded",
        "axes": {"thickness_mm": [.5, 5., 20.], "mass_per_plate_kg": [1/6, 1/3, 2/3],
                 "radius_mm": [75., 100., 125.], "interplate_x_offset_mm": [0., .5, 1.],
                 "dt_ms": [.25, .5, 1.]},
        "seed_formula": "41000 + condition index; same condition order in both engines",
        "conditions": rows,
    }
    (args.out / "sap_threeplate_aligned_frozen_r20.json").write_text(json.dumps(payload, indent=2) + "\n")


if __name__ == "__main__":
    main()
