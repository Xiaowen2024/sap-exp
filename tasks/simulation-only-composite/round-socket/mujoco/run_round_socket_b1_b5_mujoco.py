#!/usr/bin/env python3
"""Strict B=1/B=5, R=20 MuJoCo CPU round peg--socket evaluation.

The shared 32-piece CoACD socket is fixed.  Candidate 0 is a literal MuJoCo
numerical default (no option/default solver parameters in the generated MJCF).
Candidates 1--4 are declared before running; they are not a hidden sweep.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import json
import re
import time
from pathlib import Path

import mujoco
import numpy as np

ROOT = REPO_ROOT
HERE = ROOT / "mujoco-sim" / "learn_mujoco" / "peg_in_hole_experiments"
BASE = HERE / "peg_convex_decomposition.xml"
OUT = ROOT / "exp-results" / "protocol-v2-native-default" / "composite-v1" / "round-socket" / "mujoco"
ASSETS = {"blind_socket.obj": (HERE / "assets" / "blind_socket.obj").read_bytes()}
ASSETS.update({p.name: p.read_bytes() for p in (HERE / "assets" / "collision").glob("socket_piece_*.obj")})
R, T = 20, 2.0
SEED = 20261029


def candidates():
    # B=1 + four predeclared, native MuJoCo candidates.  Physical geometry,
    # friction, gravity, and the shared collision decomposition never change.
    return [
        {"id": "native_default", "native": True},
        {"id": "tc0p75ms_dr1_d95", "dt": .0005, "tc": .00075, "dr": 1.0, "d": .95},
        {"id": "tc1ms_dr1_d95", "dt": .0005, "tc": .001, "dr": 1.0, "d": .95},
        {"id": "tc2ms_dr2_d95", "dt": .0005, "tc": .002, "dr": 2.0, "d": .95},
        {"id": "tc4ms_dr2_d99", "dt": .001, "tc": .004, "dr": 2.0, "d": .99},
    ]


def source(c, clearance_mm: float, offset_mm: float) -> str:
    text = BASE.read_text()
    radius = 12.0 - clearance_mm
    text = text.replace('size="0.010 0.025"', f'size="{radius / 1000:g} 0.025"')
    text = text.replace('pos="0.001 0 0.120"', f'pos="{offset_mm / 1000:g} 0 0.120"')
    if c.get("native"):
        # Preserve the task clock and gravity in B=1; remove only numerical
        # solver/contact overrides.  Out-of-box is not a license to change dt.
        text = re.sub(r'<option\s+[^>]*/>', '<option timestep="0.0005" gravity="0 0 -9.81"/>', text, count=1)
        text = re.sub(r'<default><geom friction="([^"]+)" solref="[^"]+" solimp="[^"]+"/></default>',
                      r'<default><geom friction="\1"/></default>', text, count=1)
    else:
        option = (f'<option timestep="{c["dt"]:g}" gravity="0 0 -9.81" '
                  'integrator="implicitfast" cone="elliptic" solver="Newton"/>')
        text = re.sub(r'<option\s+[^>]*/>', option, text, count=1)
        text = re.sub(r'<default><geom friction="([^"]+)" solref="[^"]+" solimp="[^"]+"/></default>',
                      f'<default><geom friction="\\1" solref="{c["tc"]:g} {c["dr"]:g}" '
                      f'solimp="{c["d"]:g} {c["d"]:g} .001 .5 2"/></default>', text, count=1)
    return text


def one(c, clearance_mm: float, offset_mm: float, seed: int):
    m = mujoco.MjModel.from_xml_string(source(c, clearance_mm, offset_mm), assets=ASSETS)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(seed)
    # Frozen, genuinely small local perturbation: ±0.05 mm lateral placement.
    d.qpos[0] += rng.uniform(-.00005, .00005)
    d.qpos[1] += rng.uniform(-.00005, .00005)
    mujoco.mj_forward(m, d)
    peg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "peg_geom")
    table = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "table")
    pmax, max_contacts = 0.0, 0
    start = time.perf_counter()
    for _ in range(round(T / m.opt.timestep)):
        mujoco.mj_step(m, d)
        max_contacts = max(max_contacts, int(d.ncon))
        for ci in range(d.ncon):
            con = d.contact[ci]
            if peg in (con.geom1, con.geom2) and (con.geom2 if con.geom1 == peg else con.geom1) != table:
                pmax = max(pmax, -float(con.dist) * 1e3)
    pfinal = 0.0
    for ci in range(d.ncon):
        con = d.contact[ci]
        if peg in (con.geom1, con.geom2) and (con.geom2 if con.geom1 == peg else con.geom1) != table:
            pfinal = max(pfinal, -float(con.dist) * 1e3)
    x, y, z = (float(v) * 1e3 for v in d.qpos[:3])
    radial = float(np.hypot(x, y))
    entered = z <= 45.0 and radial <= 3.0
    escaped = radial > 50.0 or z < -10.0 or z > 160.0
    full = int(d.warning[mujoco.mjtWarning.mjWARN_CONTACTFULL].number)
    credible = bool(entered and radial <= clearance_mm and not escaped and full == 0
                    and pmax <= .1 * clearance_mm and pfinal <= .1 * clearance_mm)
    return {
        "seed": seed, "dt_ms": float(m.opt.timestep * 1e3), "solver": int(m.opt.solver),
        "integrator": int(m.opt.integrator), "clearance_mm": clearance_mm,
        "initial_offset_mm": offset_mm, "final_xyz_mm": [x, y, z], "final_radial_mm": radial,
        "entered": entered, "escaped": escaped, "contactfull_warning": full,
        "max_contacts": max_contacts, "pmax_mm": pmax, "pfinal_mm": pfinal,
        "pmax_over_clearance": pmax / clearance_mm, "credible": credible,
        "wall_s": time.perf_counter() - start,
    }


def summarize(c, rows):
    return {
        "candidate": c, "R": len(rows), "credible_count": int(sum(r["credible"] for r in rows)),
        "runtime": {key: rows[0][key] for key in ("dt_ms", "solver", "integrator")},
        "credible_rate": float(np.mean([r["credible"] for r in rows])),
        "pmax_over_clearance_mean": float(np.mean([r["pmax_over_clearance"] for r in rows])),
        "final_radial_mm_mean": float(np.mean([r["final_radial_mm"] for r in rows])),
        "mean_wall_s": float(np.mean([r["wall_s"] for r in rows])), "rollouts": rows,
    }


def rank(x):
    return (-x["credible_rate"], x["pmax_over_clearance_mean"], x["final_radial_mm_mean"], x["mean_wall_s"])


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    selection = []
    for j, c in enumerate(candidates()):
        rows = [one(c, 2.0, 1.0, SEED + 1000 * j + r) for r in range(R)]
        selection.append(summarize(c, rows))
        print(json.dumps({k: selection[-1][k] for k in selection[-1] if k != "rollouts"}), flush=True)
    chosen = min(selection, key=rank)
    heldout = []
    # Selection geometry is excluded; centered and remaining clearance/offset
    # cases assess generalization of the frozen B=5 setting.
    for clearance in (2.0, 1.5, 1.0, .5, .25):
        for offset in (0.0, 1.0):
            if clearance == 2.0 and offset == 1.0:
                continue
            rows = [one(chosen["candidate"], clearance, offset, SEED + 100000 + int(clearance * 1000) * 100 + int(offset * 10) + r)
                    for r in range(R)]
            heldout.append(summarize(chosen["candidate"], rows) | {"condition": {"clearance_mm": clearance, "offset_mm": offset}})
            print(json.dumps({k: heldout[-1][k] for k in heldout[-1] if k not in ("rollouts", "candidate")}), flush=True)
    payload = {
        "protocol": "composite-v1 round peg--socket strict B=1/B=5", "engine": "MuJoCo CPU",
        "engine_version": mujoco.__version__, "representation": "shared 32-piece CoACD",
        "selection_condition": "2-mm clearance, 1-mm offset, 1-degree tilt, 2 s",
        "perturbation": "independent uniform lateral x/y ±0.05 mm, R=20",
        "selection_rule": "max credible rate -> min mean pmax/clearance -> min final radial -> wall time",
        "B1": selection[0], "B5_candidates": selection, "frozen_B5": chosen, "heldout": heldout,
    }
    (OUT / "round_socket_b1_b5_r20.json").write_text(json.dumps(payload, indent=2) + "\n")


if __name__ == "__main__":
    main()
