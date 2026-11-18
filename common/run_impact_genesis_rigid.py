#!/usr/bin/env python3
"""dynamic-impact-v2/v3 (GAUGE ball bounce, ContactNets cube toss), Genesis RigidSolver.

Trial selection, start states, R=20 perturbations, cell transforms and metrics are impact_v2_common (shared with SAP and
MuJoCo Warp).  Genesis free-joint state matches MuJoCo's: qpos = [pos, quat wxyz], dofs velocity = [world linear,
body angular] (verified in the preflight), so init[:, 7:13] is passed unchanged.
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
import impact_v2_common as ic

BALL = {"mass": 0.121, "mu": 0.808698, "mesh": G.ROOT / "exp-results/gauge-rigid-reproduction/assets/ball-6.obj"}
CUBE = {"mass": 0.37, "mu": 0.18, "half": ic.CUBE_HALF, "inertia": 0.2950002913583124 * ic.CUBE_HALF ** 2}


def entities(task: str, mass_scale: float) -> dict[str, str]:
    if task == "ball":
        return {"ground": G.mjcf(f'<geom name="ground" type="plane" pos="0 0 {ic.BALL_GROUND}" size="5 5 .1" friction="{BALL["mu"]} 0 0"/>'),
                "ball": G.mjcf(f'<body name="ball"><freejoint/><geom type="mesh" mesh="ball" mass="{BALL["mass"] * mass_scale}" '
                               f'friction="{BALL["mu"]} 0 0"/></body>', meshdir=BALL["mesh"].parent,
                               assets=f'<mesh name="ball" file="{BALL["mesh"].name}"/>')}
    h, I = CUBE["half"], CUBE["inertia"] * mass_scale
    return {"ground": G.mjcf(f'<geom name="ground" type="plane" size="5 5 .1" friction="{CUBE["mu"]} 0 0"/>'),
            "cube": G.mjcf(f'<body name="cube"><freejoint/><inertial pos="0 0 0" mass="{CUBE["mass"] * mass_scale}" '
                           f'diaginertia="{I} {I} {I}"/><geom type="box" size="{h} {h} {h}" friction="{CUBE["mu"]} 0 0"/></body>')}


def make_simulate(provenance: list):
    def simulate(c, *, task, dt, mass_scale, init, n_samples, steps_per_sample):
        ents = entities(task, mass_scale)
        vec = G.sol_vector(c)
        W = len(init)
        b = G.build(ents, dt=dt, cand=c, n_envs=W, sol={k: vec for k in ents})
        body = b.ents[task]
        body.set_qpos(init[:, 0:7].astype(np.float32), zero_velocity=True)
        body.set_dofs_velocity(init[:, 7:13].astype(np.float32))
        v0 = G.np_(body.get_dofs_velocity())
        if not np.allclose(v0, init[:, 7:13], atol=1e-5):
            raise RuntimeError("initial velocity did not reach the Genesis state")
        pos = np.zeros((W, n_samples, 3)); quat = np.zeros((W, n_samples, 4))
        for s in range(n_samples):
            q = G.np_(body.get_qpos()); pos[:, s] = q[:, :3]; quat[:, s] = q[:, 3:7]
            if s < n_samples - 1:
                for _ in range(steps_per_sample):
                    b.scene.step()
                b.contacts()
        finite = np.isfinite(pos).all(axis=(1, 2)) & np.isfinite(quat).all(axis=(1, 2))
        quat = quat / np.maximum(np.linalg.norm(quat, axis=2, keepdims=True), 1e-12)
        provenance.append(b.provenance())
        b.destroy()
        return {"pos": pos, "quat": quat, "finite": finite, "truncated": 0}
    return simulate


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", choices=("ball", "cube"), required=True)
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cube-npz", type=Path, default=G.ROOT / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si_v2.npz")
    a = ap.parse_args()
    man0, _ = G.load_manifest(a.manifest)
    task_id = G.registry_id_for(a.manifest)
    if (task_id == "cube-toss") != (a.task == "cube"):
        raise ValueError(f"--task {a.task} does not match registry task {task_id}")
    _, spec, sha = G.registry_task(task_id, a.manifest)
    man = G.with_candidates(man0, spec)
    sym = None
    if a.task == "ball":
        data = ic.ball_trials(G.ROOT)
    else:
        data = ic.cube_data(a.cube_npz)
        from cube_orientation_metrics import cube_symmetric_qangle as sym
    prov: list = []
    res = ic.execute(a.task, man, "genesis_rigid_candidates", lambda t: G.parse_candidate(t, spec), make_simulate(prov),
                     data, sym, log=lambda s: print(s, flush=True))
    for r in res["selection_candidates"] + res["heldout"]:
        r["compiled"] = prov.pop(0)
    out = {**G.header(man0, sha, spec), "registry_task": task_id, "complete": True, **res}
    name = f"genesis_rigid_{a.task}_{'dynamic_impact_v3' if task_id == 'ball-bounce-v3' else 'dynamic_impact_v2'}.json"
    G.save_json(a.out / name, out)


if __name__ == "__main__":
    main()
