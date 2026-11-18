#!/usr/bin/env python3
"""Rollout coupling check: IPC solves all environments of a scene in one Newton system.  A box pushed at r = 0.7 alone,
and the same box with a neighbouring environment sliding at r = 1.25, at newton_tol 1e-4 and the default.  Not a task result."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
from pathlib import Path

import numpy as np

import genesis_ipc_common as I
import genesis_ipc_preflight as P

MU = 0.3; MG = P.M_BOX * 9.81


def creep(cand, rs, dt=0.002, settle=200, steps=500):
    g0 = I.start_gap(cand)
    b = I.build({"ground": I.Body(P.plane_xml(MU), mu=MU), "box": I.Body(P.box_xml("box", P.H + g0, MU), mu=MU, mass=P.M_BOX, force=True)},
                dt=dt, cand=cand, n_envs=len(rs))
    for _ in range(settle):
        b.step()
    b.forces["box"][:, 0] = np.asarray(rs) * MU * MG
    for _ in range(steps):
        b.step()
    v = b.vel("box")[0][:, 0]; b.destroy()
    return v


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args()
    I.gs_init(); res = []
    for text in ("d_hat=1mm,eps_v=1e-3,newton_tol=1e-4", "d_hat=1mm,eps_v=1e-3"):
        c = I.parse_candidate(text)
        solo = creep(c, [0.7])[0]; pair = creep(c, [0.7, 1.25])
        row = {"candidate": text, "r0.7_alone_mm_s": solo * 1e3, "r0.7_next_to_sliding_mm_s": pair[0] * 1e3,
               "neighbour_r1.25_mm_s": pair[1] * 1e3, "relative_change": (pair[0] - solo) / solo}
        print("batch", json.dumps(row), flush=True); res.append(row)
    a.out.write_text(json.dumps(res, indent=1) + "\n")


if __name__ == "__main__":
    main()
