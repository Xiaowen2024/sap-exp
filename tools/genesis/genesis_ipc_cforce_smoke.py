#!/usr/bin/env python3
"""Smoke test of the direct contact readout inside the four affected runners (R=4, selection cells only).

Not a task result.  Each line: the gate outcome with the direct readout plus readout_check (direct vs momentum or
bridge spring), at the frozen preliminary candidate and at the same candidate with newton_tol=1e-4.
  python-genesis genesis_ipc_cforce_smoke.py --lane plates|boxes --out <dir>
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT, MANIFESTS  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
from pathlib import Path

import numpy as np

import genesis_ipc_common as I

MAN = MANIFESTS
R = 4


def emit(out, tag, d):
    line = json.dumps(I.json_safe({"tag": tag, **d}))
    print(line, flush=True)
    with open(out, "a") as f:
        f.write(line + "\n")


def plates(out):
    import run_threeplate_genesis_ipc as tp
    man = json.loads((MAN / "three_plate_stack_strict_v3p1.json").read_text())
    for text in ("native default", "d_hat=1mm,newton_tol=1e-4"):
        r = tp.run_cell(I.parse_candidate(text), man["selection"], 1234, R, man["duration_s"])
        emit(out, f"three-plate|{text}", {"pass": r["pass_count"], "failed_gates": r["failed_gate_counts"],
                                          "load_error_max_mean": r["load_error_max_mean"], "contact_duty_mean": r["contact_duty_mean"],
                                          "readout_check": r.get("readout_check"), "failed": r["failed"], "wall_s": r["wall_s"]})
    import run_tail_genesis_ipc as rt
    import tail_v2_common as tc
    tman = json.loads((MAN / "contactnets_cube_tail_sustained_v3.json").read_text())
    cell = tman["selection"]; dt = cell["dt_ms"] * 1e-3
    for text in ("native default", "d_hat=1mm,newton_tol=1e-4"):
        c = I.parse_candidate(text); h = tc.HALF * cell["scale"]
        init = np.zeros((R, 13)); init[:, 2] = h + I.start_gap(c); init[:, 3] = 1.0
        spp = max(1, round(1 / 30 / dt)); n = 60
        r, why = I.ISO.call("run_tail_genesis_ipc", "sim_chunk", tag=text, c=c, dt=dt, mass_scale=cell["mass_scale"], mu=cell["mu"],
                            scale=cell["scale"], init=init, n_samples=n, steps_per_sample=spp)
        if r is None:
            emit(out, f"cube-tail|{text}", {"failed": why}); continue
        mg = tc.MASS * cell["mass_scale"] * tc.G; sl = slice(n // 2, None)
        emit(out, f"cube-tail|{text}", {"direct_load_over_mg": float(r["fn"][:, sl].mean() / mg),
                                        "momentum_load_over_mg": float(r["fn_momentum"][:, sl].mean() / mg),
                                        "direct_minus_momentum_over_mg_max": float(np.abs(r["fn"][:, sl] - r["fn_momentum"][:, sl]).max() / mg),
                                        "active_vertices_mean": float(r["ncon"][:, sl].mean()),
                                        "rest_gap_mm": float(np.median(r["min_corner_gap"][:, -1]) * 1e3)})


def boxes(out):
    import run_pinch_genesis_ipc as rp
    pman = json.loads((MAN / "simple_pinch_force_controlled_v3.json").read_text())
    for text in ("d_hat=0.3mm,eps_v=1e-3,strength=1", "d_hat=0.3mm,eps_v=1e-3,strength=1,newton_tol=1e-4"):
        for k, cell in enumerate(pman["selection"]):
            r = rp.run_cell(I.parse_candidate(text), cell, pman["seed0"] + k, R)
            emit(out, f"simple-pinch|{text}|{cell['id']}", {k_: r.get(k_) for k_ in (
                "expected", "agree_and_valid_count", "observed_retain", "valid_count", "gate_preload_mean_N", "preload_rel_error_max",
                "hold_slip_creep_mm_s_mean", "readout_check", "failed")})
    import run_threebox_genesis_ipc as rb
    bman = json.loads((MAN / "three_box_push_strict_v4.json").read_text())
    for text in ("d_hat=0.1mm,eps_v=1e-3", "d_hat=0.1mm,eps_v=1e-3,newton_tol=1e-4"):
        for k, cell in enumerate(bman["selection"]):
            r = rb.run_cell(I.parse_candidate(text), cell, bman["seed0"] + k, R)
            emit(out, f"three-box|{text}|{cell['id']}", {"expected": cell["expected"], "summary": rb.fmt(r).replace("/20", f"/{R}"),
                                                          "readout_check": r.get("readout_check"), "failed": r.get("failed")})


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--lane", required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
    {"plates": plates, "boxes": boxes}[a.lane](a.out / f"smoke_{a.lane}.jsonl")
    print("SMOKE DONE", a.lane, flush=True)


if __name__ == "__main__":
    main()
