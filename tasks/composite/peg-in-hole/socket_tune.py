#!/usr/bin/env python3
"""Round-socket v9 engine tuning iterations (user decision 2026-11-18): no declared candidate list, up to five iterations
per engine, each one changing only engine-numerical settings to fix the problems seen in the previous iteration.

The evaluated world is fixed by the manifest and the SOP and is never changed here: geometry assets, clearance, mass,
inertia, mu, gravity, dt, duration, initial pose and perturbations (seeds), R, labels and thresholds.  Every condition is
simulated by the engine's existing socket driver and scored by socket_v8_common.run_condition (unchanged).  No held-out:
each iteration runs the 49 labelled conditions (3 selection + 46 labelled frozen); the 15 unlabelled report-only
conditions are skipped.

Engine settings accepted in the candidate JSON
  mj : native (bool) | tc, dr, solimp[5] | integrator, cone, solver | impratio, iterations, ls_iterations, tolerance,
       noslip_iterations, noslip_tolerance, ccd_iterations | margin, gap (geom) | friction_torsional, friction_rolling
  sap: ke, tau, gap (shape material) | solver: dict of SolverSAP keyword arguments (e.g. contact_sigma, max_iterations)

Usage: socket_tune.py --engine mj|sap --cand cand.json --manifest M --out DIR [--shard i/n] [--merge]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import sys
import time
from pathlib import Path

import numpy as np


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path); mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod; spec.loader.exec_module(mod); return mod


def mj_engine(runner_dir: Path, sc):
    R = load(runner_dir / "run_socket_v8_mjwarp.py", "run_socket_v8_mjwarp"); R.sc = sc
    import mujoco  # noqa: F401
    orig_xml = R.xml

    def xml(c, cell):
        if c.get("native"):
            return orig_xml({"native": True}, cell)
        dt, mu, r = cell["dt_ms"] * 1e-3, cell["mu"], sc.peg_radius_mm(cell) * 1e-3
        assets = "".join(f'<mesh name="m{i}" file="{f}"/>' for i, f in enumerate(sc.FILES))
        pieces = "".join(f'<geom name="piece{i}" type="mesh" mesh="m{i}"/>' for i in range(len(sc.FILES)))
        o = {"integrator": c.get("integrator", "implicitfast"), "cone": c.get("cone", "elliptic"), "solver": c.get("solver", "Newton")}
        for k in ("impratio", "iterations", "ls_iterations", "tolerance", "noslip_iterations", "noslip_tolerance", "ccd_iterations"):
            if k in c:
                o[k] = c[k]
        opts = "".join(f' {k}="{v}"' for k, v in o.items())
        opt = f'<option timestep="{dt}" gravity="0 0 {-sc.gravity(cell)}"{opts}/>'
        g = f' friction="{mu} {c.get("friction_torsional", .005)} {c.get("friction_rolling", .0001)}"'
        if "tc" in c:
            g += f' solref="{c["tc"]} {c.get("dr", 1.0)}"'
        if "solimp" in c:
            g += f' solimp="{" ".join(map(str, c["solimp"]))}"'
        for k in ("margin", "gap"):
            if k in c:
                g += f' {k}="{c[k]}"'
        I = sc.peg_inertia(cell); p0, q0 = sc.initial_pose(cell)
        return (f'<mujoco model="round_socket_tune">{opt}<compiler meshdir="{R.ROOT / sc.ASSET_DIR}"/><default><geom{g}/></default>'
                f'<asset>{assets}</asset><worldbody>{pieces}'
                '<body name="peg" pos="{} {} {}" quat="{} {} {} {}"><freejoint/>'.format(*p0, *q0) +
                f'<inertial pos="0 0 0" mass="{sc.peg_mass(cell)}" diaginertia="{I[0]} {I[1]} {I[2]}"/>'
                f'<geom name="peg_geom" type="cylinder" size="{r} {sc.PEG_HALF * 1e-3}"/></body></worldbody></mujoco>')

    R.xml = xml

    def runnable(c, dt):
        if not c.get("native") and "tc" in c and c["tc"] < 2 * dt - 1e-12:
            return False, f"timeconst {c['tc'] * 1e3:.4g} ms < 2 dt (MuJoCo limit)"
        return True, ""
    import mujoco_warp
    return R.simulate, runnable, {"engine": "MuJoCo Warp", "version": getattr(mujoco_warp, "__version__", "?")}


def sap_engine(runner_dir: Path, sc):
    R = load(runner_dir / "run_socket_v8_sap.py", "run_socket_v8_sap"); R.sc = sc
    import sap_warp
    orig_solver = R.SolverSAP

    def make_sim(c):
        kw = dict(c.get("solver") or {})
        R.SolverSAP = (lambda m, **k: orig_solver(m, **{**k, **kw})) if kw else orig_solver
        base = {"ke": c.get("ke"), "tau": c.get("tau"), "gap": c.get("gap"), "id": c["id"]}
        return lambda cc, cell, xy, dump_contacts=False: R.simulate(base, cell, xy, dump_contacts)
    return make_sim, (lambda c, dt: (True, "")), {"engine": "SAP Warp internal", "sap_source": str(Path(sap_warp.__file__).parent)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--engine", choices=["mj", "sap"], required=True); ap.add_argument("--cand", type=Path, required=True)
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--simexp-scripts", type=Path, default=Path(__file__).resolve().parent)
    ap.add_argument("--runner-dir", type=Path, required=True); ap.add_argument("--shard", default="0/1"); ap.add_argument("--merge", action="store_true")
    ap.add_argument("--rollouts", type=int, default=0, help="tuning screen: rollouts per condition (0 = manifest R)")
    a = ap.parse_args()
    sys.path.insert(0, str(a.simexp_scripts))
    import socket_v8_common as sc
    man, sha = sc.load_manifest(a.manifest)
    if a.rollouts:
        man = dict(man, R=a.rollouts)   # tuning screen only: fewer rollouts per condition; the final run uses the manifest R
    cand = json.loads(a.cand.read_text())
    conds = [c for c in man["conditions"] if c.get("expected") is not None]
    idx = {c["id"]: j for j, c in enumerate(man["conditions"])}
    a.out.mkdir(parents=True, exist_ok=True)
    if a.merge:
        rows = []
        for f in sorted(a.out.glob("shard_*.json")):
            rows += json.loads(f.read_text())["conditions"]
        by = {r["id"]: r for r in rows}; rows = [by[c["id"]] for c in conds if c["id"] in by]
        fro = [r for r in rows if man["conditions"][idx[r["id"]]]["role"] == "frozen"]
        sel = [r for r in rows if man["conditions"][idx[r["id"]]]["role"] == "selection"]
        allc = {"n_conditions": len(rows), "correct": sum(r["verdict"] == "correct" for r in rows),
                "unstable": sum(r["verdict"] == "unstable" for r in rows), "wrong": sum(r["verdict"] == "wrong" for r in rows),
                "correct_rollouts": sum(r["correct_rollouts"] for r in rows), "rollouts": sum(r["R"] for r in rows),
                "inadmissible_conditions": sum(1 for r in rows if r.get("inadmissible_rests")),
                "peak_base_pen_mm_mean": float(np.mean([r["pen_total_peak_mm"]["mean"] for r in rows]))}
        out = {"manifest_sha256": sha, "manifest_revision": man["revision"], "candidate": cand, "summary_49": allc, "rollouts_per_condition": man["R"],
               "selection": {"conditions": sel, "tally": sc.tally(sel) if sel else None},
               "frozen_matrix": {"B5": {"candidate_index": cand.get("iteration"), "conditions": fro, "tally": sc.tally(fro)}},
               "missing": [c["id"] for c in conds if c["id"] not in by]}
        (a.out / "result.json").write_text(json.dumps(sc.json_safe(out), indent=1) + "\n")
        print(json.dumps(allc), flush=True); return
    i, n = map(int, a.shard.split("/")); mine = conds[i::n]
    import warp as wp
    wp.init()
    if a.engine == "mj":
        simulate, runnable, info = mj_engine(a.runner_dir, sc); sim = simulate
    else:
        make_sim, runnable, info = sap_engine(a.runner_dir, sc); sim = make_sim(cand)
    ok, why = runnable(cand, mine[0]["dt_ms"] * 1e-3)
    path = a.out / f"shard_{i}.json"
    prev = {r["id"]: r for r in json.loads(path.read_text())["conditions"]} if path.exists() else {}
    rows = []
    for c in mine:
        if c["id"] in prev:
            rows.append(prev[c["id"]]); continue
        ok, why = runnable(cand, c["dt_ms"] * 1e-3)
        if not ok:
            rows.append({"id": c["id"], "verdict": "not_runnable", "reason": why, "correct_rollouts": 0, "R": man["R"]}); continue
        t0 = time.time()
        r = sc.run_condition(cand, c, man["seed0"] + idx[c["id"]], man, sim, a.out / "traj", "")
        r["wall_s"] = time.time() - t0; rows.append(r)
        print(f"[iter {cand.get('iteration')}] {sc.fmt(r)}", flush=True)
        path.write_text(json.dumps(sc.json_safe({"engine": info, "candidate": cand, "conditions": rows}), indent=1) + "\n")
    path.write_text(json.dumps(sc.json_safe({"engine": info, "candidate": cand, "conditions": rows, "complete": True}), indent=1) + "\n")


if __name__ == "__main__":
    main()
