#!/usr/bin/env python3
"""GAUGE slope slider dynamic-friction-v3, Genesis RigidSolver (shared logic in slope_v3_common).

Static board: a body with no joint (fixed).  Kinematic board (task-3): a free body whose pose and velocity are written
every step from the mean recorded board trajectory, as the MuJoCo Warp driver does (Genesis free-joint angular velocity
is body-frame, so board_state's w_local is passed unchanged).  Ground, board and cube share the GAUGE harmonic-mean mu.
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
import slope_v3_common as sc

TASK = "gauge-slope"
ASSETS = G.ROOT / "exp-results/gauge-rigid-reproduction/assets"


def entities(case):
    mu = case["mu"]; j = lambda a: " ".join(map(str, a))
    joint = "<freejoint/>" if case["kinematic"] else ""
    return {"ground": G.mjcf(f'<geom name="ground" type="plane" size="5 5 .1" friction="{mu} 0 0"/>'),
            "board": G.mjcf(f'<body name="board" pos="{j(case["p_f"])}" quat="{j(case["q_f"])}">{joint}<geom name="board_geom" '
                            f'type="mesh" mesh="board_mesh" mass="{case["mass_board_kg"]}" friction="{mu} 0 0"/></body>',
                            meshdir=ASSETS, assets='<mesh name="board_mesh" file="board.obj"/>'),
            "cube": G.mjcf(f'<body name="cube" pos="{j(case["p_m"])}" quat="{j(case["q_m"])}"><freejoint/><geom name="cube_geom" '
                           f'type="mesh" mesh="cube_mesh" mass="{case["mass_kg"]}" friction="{mu} 0 0"/></body>',
                           meshdir=ASSETS, assets='<mesh name="cube_mesh" file="cube.obj"/>')}


def make_run_case(provenance: list):
    def run_case(c, case, init_pos, n_frames, spf, dt):
        ents = entities(case); vec = G.sol_vector(c); R = len(init_pos)
        b = G.build(ents, dt=dt, cand=c, n_envs=R, sol={k: vec for k in ents})
        cube, board = b.ents["cube"], b.ents["board"]
        q0 = G.np_(cube.get_qpos()).astype(np.float64); q0[:, 0:3] = init_pos
        cube.set_qpos(q0.astype(np.float32), zero_velocity=True)
        pos = np.zeros((R, n_frames, 3)); total = (n_frames - 1) * spf
        track = []
        for step in range(total + 1):
            if case["kinematic"]:
                st = sc.board_state(case, step * dt)
                board.set_qpos(np.tile(np.r_[st["p"], st["q"]], (R, 1)).astype(np.float32))
                board.set_dofs_velocity(np.tile(np.r_[st["v"], st["w_local"]], (R, 1)).astype(np.float32))
                if step % spf == 0:
                    track.append(float(np.abs(G.np_(board.get_qpos())[:, 0:3] - st["p"]).max()))
            if step % spf == 0:
                pos[:, step // spf] = G.np_(cube.get_qpos())[:, 0:3]
                b.contacts()
            if step < total:
                b.scene.step()
        provenance.append(b.provenance()); b.destroy()
        out = {"pos": pos, "finite": np.isfinite(pos).all(axis=(1, 2)), "truncated": 0}
        if case["kinematic"]:
            out["tracking"] = {"board_written_pose_max_abs_m": max(track), "method": "pose and velocity written every step"}
        return out
    return run_case


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    _, spec, sha = G.registry_task(TASK, a.manifest)
    man0, _ = G.load_manifest(a.manifest)
    man = G.with_candidates(man0, spec)
    prov: list = []
    res = sc.execute(man, "genesis_rigid_candidates", lambda t: G.parse_candidate(t, spec), make_run_case(prov), G.ROOT,
                     log=lambda s: print(s, flush=True))
    cells = [cell for s in res["selection_candidates"] for cell in s["cells"]] + res["heldout"]
    for r in cells:
        r["compiled"] = prov.pop(0)
    G.save_json(a.out / "genesis_rigid_gauge_slope_v3.json", {**G.header(man0, sha, spec), "complete": True, **res})


if __name__ == "__main__":
    main()
