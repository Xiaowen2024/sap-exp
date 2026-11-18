#!/usr/bin/env python3
"""Diagnostic (not a protocol run): MuJoCo Warp contact points on the round-socket v6 rim-rest cells.

Re-runs the frozen v6 MuJoCo Warp candidate (from a result json) on the given held-out rim cells, rollout 0 of the
held-out seed (R=1), and records every SAMPLE_S the peg pose and every peg contact: world point, normal (frame row 0),
dist and the efc force of the contact's first row (normal force for the elliptic cone), all read from the GPU arrays.
Output: one npz per cell, same layout as the SAP diagnostic.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
from pathlib import Path

import mujoco
import mujoco_warp as mjw
import numpy as np

import socket_v6_common as sc
import run_socket_v4_mjwarp as driver
from run_socket_v4_mjwarp import parse_candidate, xml

driver.sc = sc                     # the driver's xml() reads its module-level sc (v6 pose path)

SAMPLE_S = 0.005


def run(cand, cell, xy, out):
    m = mujoco.MjModel.from_xml_string(xml(cand, cell)); gm = mjw.put_model(m)
    d = mjw.make_data(m, nworld=1, nconmax=512, njmax=2048)
    q = d.qpos.numpy(); q[:, 0:2] += xy; d.qpos.assign(q.astype(np.float32)); mjw.forward(gm, d)
    peg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "peg_geom"); dt = m.opt.timestep
    every = max(1, int(round(SAMPLE_S / dt))); T, P, Q, C = [], [], [], []
    for k in range(round(cell["duration_s"] / dt) + 1):
        if k % every == 0:
            qp = d.qpos.numpy()[0].astype(float)
            T.append(k * dt); P.append(qp[0:3]); Q.append(qp[3:7])
            n = int(d.nacon.numpy()[0]); g = d.contact.geom.numpy()[:n]; ps = d.contact.pos.numpy()[:n]; fr = d.contact.frame.numpy()[:n]
            ds = d.contact.dist.numpy()[:n]; ea = d.contact.efc_address.numpy()[:n]; ef = d.efc.force.numpy()
            ef = ef[0] if ef.ndim == 2 else ef
            rows = []
            for i in range(n):
                if peg not in (g[i, 0], g[i, 1]):
                    continue
                sgn = 1.0 if g[i, 1] == peg else -1.0                       # frame row 0 points geom1 -> geom2
                adr = int(ea[i, 0]) if ea.ndim == 2 else int(ea[i])
                rows.append([*ps[i], *(sgn * fr[i, 0]), ds[i], float(ef[adr]) if adr >= 0 else 0.0])
            C.append(np.array(rows).reshape(-1, 8))
        mjw.step(gm, d)
    off = np.cumsum([0] + [len(c) for c in C])
    np.savez(out, t=np.array(T), pos=np.array(P), quat=np.array(Q), contacts=np.concatenate(C) if off[-1] else np.zeros((0, 8)), offsets=off,
             meta=json.dumps({"engine": f"MuJoCo Warp {mujoco.__version__}", "candidate": cand["id"], "cell": cell, "xy": xy.tolist(),
                              "columns": "px py pz nx ny nz dist fn"}))
    print("wrote", out, "samples", len(T), "contacts/sample max", max(len(c) for c in C), "final z", P[-1][2], flush=True)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--result", type=Path, required=True)
    ap.add_argument("--cells", nargs="+", required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(); man, _ = sc.load_manifest(a.manifest); res = json.loads(a.result.read_text())
    fc = res["frozen_candidate"]; cand = parse_candidate("native default" if fc.get("native") else fc["id"]); a.out.mkdir(parents=True, exist_ok=True)
    for j, cell in enumerate(man["held_out_conditions"]):
        if cell["id"] in a.cells:
            xy = sc.perturbation(cell, man["seed0"] + 1000 + j, man["R"])[0:1]
            run(cand, cell, xy, a.out / f"mj_{cell['id']}.npz")


if __name__ == "__main__":
    main()
