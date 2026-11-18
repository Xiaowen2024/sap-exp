#!/usr/bin/env python3
"""Run the same declared frozen three-plate matrix with MuJoCo Warp."""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
from typing import Any

from run_threeplate_sap_frozen_generalization import CASES


def load(path: Path):
    spec = importlib.util.spec_from_file_location("mj_threeplate", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def json_safe(value: Any) -> Any:
    """JSON has no NaN.  Preserve numerical invalidity explicitly as null."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_safe(item) for item in value]
    return value


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--repeats", type=int, default=20)
    args = ap.parse_args()
    if args.repeats != 20:
        raise ValueError("aligned protocol requires R=20")
    m = load(args.source)
    frozen = {"id": "tc8_dr2_d99", "timeconst_s": .008, "dampratio": 2., "dmax": .99}
    original_xml = m.xml_for
    rows = []
    for i, case in enumerate(CASES):
        m.MASS, m.RADIUS = case["mass_kg"], case["radius_m"]
        offset, h = case["offset_m"], case["h_m"]
        def shifted_xml(h_arg, c, dt, *, _offset=offset, _h=h):
            xml = original_xml(h_arg, c, dt)
            for plate, scale in ((1, 1.), (2, 2.)):
                old = f'name="plate_{plate}" pos="0 0 {(plate + .5) * _h}"'
                new = f'name="plate_{plate}" pos="{scale * _offset} 0 {(plate + .5) * _h}"'
                if old not in xml:
                    raise RuntimeError(f"expected plate pose not found: {old}")
                xml = xml.replace(old, new)
            return xml
        m.xml_for = shifted_xml
        result = m.run_case("mujoco_warp", frozen, h, case["dt_s"], args.repeats, 41000 + i)
        result["condition"] = case
        rows.append(result)
        print(case["id"], result["pass_fraction"], result["max_overlap_mm_mean"],
              result["normal_load_error_max_mean"], flush=True)
    payload = {
        "protocol": "aligned-frozen-generalizability-v1",
        "task": "three-plate-stack", "engine": "MuJoCo Warp", "B": 5,
        "R_per_condition": args.repeats, "frozen_candidate": frozen,
        "matrix_design": "axis-isolated held-out cells; nominal selection h=4mm excluded",
        "axes": {"thickness_mm": [.5, 5., 20.], "mass_per_plate_kg": [1/6, 1/3, 2/3],
                 "radius_mm": [75., 100., 125.], "interplate_x_offset_mm": [0., .5, 1.],
                 "dt_ms": [.25, .5, 1.]},
        "seed_formula": "41000 + condition index; same condition order in both engines",
        "conditions": rows,
    }
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "mjwarp_threeplate_aligned_frozen_r20.json").write_text(json.dumps(json_safe(payload), indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
