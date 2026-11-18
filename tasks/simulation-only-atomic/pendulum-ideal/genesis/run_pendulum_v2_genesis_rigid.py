#!/usr/bin/env python3
"""Pendulum energy-conservation-v2 (idealised, analytic reference), Genesis RigidSolver (pendulum_v2_common).

Hinge with zero damping and zero armature; solid-sphere bob, no collision.  Candidates are Genesis' integrators.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
from pathlib import Path

import numpy as np

import genesis_rigid_common as G
import pendulum_v2_common as pc

TASK = "pendulum-v2"


def make_simulate(provenance: list):
    def simulate(c, cell, th0):
        m, L, dt = cell["mass_kg"], cell["length_m"], cell["dt_ms"] * 1e-3
        ents = {"pendulum": G.mjcf(f'<body name="bob"><joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>'
                                   f'<geom type="sphere" pos="0 0 -{L}" size="{pc.R_BOB}" mass="{m}" contype="0" conaffinity="0"/></body>')}
        R = len(th0)
        b = G.build(ents, dt=dt, cand=c, n_envs=R)
        e = b.ents["pendulum"]
        e.set_dofs_position(np.asarray(th0, np.float32)[:, None], zero_velocity=True)
        every = int(round(pc.SAMPLE_S / dt)); S = int(round(pc.DURATION_S / pc.SAMPLE_S)) + 1; Q, V = [], []
        for s in range(S):
            Q.append(G.np_(e.get_dofs_position())[:, 0].astype(float)); V.append(G.np_(e.get_dofs_velocity())[:, 0].astype(float))
            if s < S - 1:
                for _ in range(every):
                    b.scene.step()
        provenance.append(b.provenance()); b.destroy()
        return np.asarray(Q), np.asarray(V)
    return simulate


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = G.registry_task(TASK, a.manifest)
    man0, _ = G.load_manifest(a.manifest)
    man = G.with_candidates(man0, spec)
    prov: list = []
    res = pc.execute(man, "genesis_rigid_candidates", lambda t: G.parse_candidate(t, spec), make_simulate(prov),
                     log=lambda s: print(s, flush=True))
    for r in res["selection_candidates"] + res["heldout"]:
        r["compiled"] = prov.pop(0)
    G.save_json(a.out / "genesis_rigid_pendulum_v2.json", {**G.header(man0, sha, spec), "complete": True, **res})


if __name__ == "__main__":
    main()
