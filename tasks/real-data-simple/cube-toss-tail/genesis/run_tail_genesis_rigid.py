#!/usr/bin/env python3
"""Cube-toss resting tail (sustained-contact v3), Genesis RigidSolver.  Shared logic in tail_v2_common.

The single candidate is inherited from the Genesis cube-toss frozen winner (manifest frozen_from rule); the artifact
path and sha256 are recorded.  Support load = sum of |f.n| over cube-floor contacts.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT, MANIFESTS  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

import genesis_rigid_common as G
import impact_v2_common as ic
import tail_v2_common as tc

TASK = "cube-tail"


def entities(mass_scale, mu, scale):
    h = tc.HALF * scale; I = 0.2950002913583124 * h ** 2 * mass_scale
    return {"floor": G.mjcf(f'<geom name="floor" type="plane" size="10 10 .1" friction="{mu} 0 0"/>'),
            "cube": G.mjcf(f'<body name="cube"><freejoint/><inertial pos="0 0 0" mass="{tc.MASS * mass_scale}" '
                           f'diaginertia="{I} {I} {I}"/><geom name="cube_geom" type="box" size="{h} {h} {h}" friction="{mu} 0 0"/></body>')}


def make_simulate(provenance: list):
    def simulate(c, *, dt, mass_scale, mu, scale, init, n_samples, steps_per_sample):
        ents = entities(mass_scale, mu, scale); vec = G.sol_vector(c); W = len(init)
        b = G.build(ents, dt=dt, cand=c, n_envs=W, sol={k: vec for k in ents})
        cube = b.ents["cube"]
        cube.set_qpos(init[:, 0:7].astype(np.float32), zero_velocity=True)
        cube.set_dofs_velocity(init[:, 7:13].astype(np.float32))
        P = np.zeros((W, n_samples, 3)); Q = np.zeros((W, n_samples, 4)); SP = np.zeros((W, n_samples))
        OM = np.zeros((W, n_samples)); FN = np.zeros((W, n_samples)); NC = np.zeros((W, n_samples))
        for s in range(n_samples):
            q, v = G.np_(cube.get_qpos()), G.np_(cube.get_dofs_velocity())
            P[:, s] = q[:, :3]; Q[:, s] = q[:, 3:7]
            SP[:, s] = np.linalg.norm(v[:, :3], axis=1); OM[:, s] = np.linalg.norm(v[:, 3:6], axis=1)
            FN[:, s], NC[:, s] = b.pair_normal_load("floor", "cube")
            if s < n_samples - 1:
                for _ in range(steps_per_sample):
                    b.scene.step()
        finite = np.isfinite(P).all(axis=(1, 2)) & np.isfinite(Q).all(axis=(1, 2))
        Q = Q / np.maximum(np.linalg.norm(Q, axis=2, keepdims=True), 1e-12)
        provenance.append(b.provenance()); b.destroy()
        return {"pos": P, "quat": Q, "speed": SP, "omega": OM, "fn": FN, "ncon": NC, "finite": finite, "truncated": 0}
    return simulate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cube-toss-artifact", type=Path, required=True,
                    help="Genesis cube-toss genesis_rigid_cube_dynamic_impact_v2.json supplying the frozen candidate")
    ap.add_argument("--cube-npz", type=Path, default=G.ROOT / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si_v2.npz")
    a = ap.parse_args()
    _, spec, sha = G.registry_task(TASK, a.manifest)
    man0, _ = G.load_manifest(a.manifest)
    toss = json.loads(a.cube_toss_artifact.read_text())
    if not toss.get("complete") or toss.get("engine") != G.ENGINE or toss.get("frozen_candidate") is None:
        raise ValueError("cube-toss artifact is not a complete Genesis RigidSolver run with a frozen candidate")
    if toss["manifest_sha256"] != man0["frozen_from"]["manifest_sha256"]:
        raise ValueError("cube-toss artifact used a different manifest than this task's frozen_from")
    _, toss_spec, _ = G.registry_task("cube-toss", MANIFESTS / man0["frozen_from"]["manifest"])
    text = toss["frozen_candidate"]["text"]
    inherited = {**spec, "candidates": [text], "nonnative": toss_spec.get("nonnative", {})}
    man = G.with_candidates(man0, inherited)
    prov: list = []
    res = tc.execute(man, "genesis_rigid_candidates", lambda t: G.parse_candidate(t, inherited), make_simulate(prov),
                     ic.cube_data(a.cube_npz), log=lambda s: print(s, flush=True))
    for r in res["selection_candidates"] + res["heldout"]:
        r["compiled"] = prov.pop(0)
    out = {**G.header(man0, sha, inherited), "complete": True,
           "inherited_from": {"artifact": str(a.cube_toss_artifact), "sha256": hashlib.sha256(a.cube_toss_artifact.read_bytes()).hexdigest(),
                              "candidate": text}, **res}
    G.save_json(a.out / "genesis_rigid_cube_tail_v3.json", out)


if __name__ == "__main__":
    main()
