#!/usr/bin/env python3
"""Add the missing 0.25-ms cell to the frozen round-socket Warp matrix."""

import importlib.util
import json
import os
from pathlib import Path


ROOT = Path(os.environ.get("SIM_EXP_GPU_ROOT", "/home/xw/sim-exp-gpu"))
SOURCE = ROOT / "mujoco-protocol/protocol_v2/run_round_socket_b1_b5_mjwarp.py"
ASSETS = ROOT / "sap-sim-round/collision"
OUT = ROOT / "results/round-mjwarp-aligned-r20"

spec = importlib.util.spec_from_file_location("round_socket", SOURCE)
round_socket = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(round_socket)
round_socket.ASSET = ASSETS

frozen = {"id": "tc2ms_dr2_d95", "dt": 0.00025, "tc": 0.002, "dr": 2.0, "d": 0.95}
result = round_socket.run(frozen, clearance=2.0, offset=1.0, seed=64000)
result["condition"] = {
    "id": "dt_0p25ms",
    "clearance_mm": 2.0,
    "offset_mm": 1.0,
    "tilt_deg": 1.0,
    "release_height_mm": 120.0,
    "dt_ms": 0.25,
}
OUT.mkdir(parents=True, exist_ok=True)
(OUT / "mjwarp_round_dt025_r20.json").write_text(json.dumps(result, indent=2) + "\n")
print(result["condition"]["id"], result["credible_count"])
