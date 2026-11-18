#!/usr/bin/env python3
"""dynamic-impact-v2/v3 (GAUGE ball bounce, ContactNets cube toss), Genesis + IPC (preliminary).

Trial selection, start states, perturbations and metrics are impact_v2_common.  The body is an IPC-only ABD; its
initial pose and velocity (MuJoCo qvel convention: world linear, body angular) are written into IPC before world init.
Starts are in the air and unchanged.  The cube's ABD inertia is uniform-density (I/m = (2h)^2/6), not the ContactNets
value; the ratio is recorded.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import genesis_ipc_common as I
import impact_v2_common as ic
from run_impact_genesis_rigid import BALL, CUBE, entities


# The GAUGE ball mesh hull has 16128 faces (8066 vertices); in IPC that costs ~20 s per step for 8 rollouts (500-vertex hull: ~2.5 s per step for 64).
# Declared representation change (IPC track only; Genesis' MJCF importer ignores morph.decimate): the collision surface
# is the convex hull of BALL_HULL_N vertices of the original mesh chosen by deterministic farthest-point sampling.
# Hull vertices lie on the original surface; the radius spread of both meshes is recorded.
BALL_MESH_KW = None
BALL_HULL_N = 162


def ball_hull_file() -> Path:
    import tempfile
    from scipy.spatial import ConvexHull
    out = Path(tempfile.gettempdir()) / f"ball-6-fps{BALL_HULL_N}-hull.obj"
    if out.exists():
        return out
    v = I.obj_vertices(BALL["mesh"])
    sel = [int(np.argmax(np.linalg.norm(v - v.mean(0), axis=1)))]
    d = np.linalg.norm(v - v[sel[0]], axis=1)
    for _ in range(BALL_HULL_N - 1):
        k = int(np.argmax(d)); sel.append(k); d = np.minimum(d, np.linalg.norm(v - v[k], axis=1))
    pts = v[sel]; h = ConvexHull(pts)
    faces = []
    for f in h.simplices:                                   # orient outward
        a, b, c = pts[f]
        faces.append(f if np.dot(np.cross(b - a, c - a), a - pts.mean(0)) > 0 else f[[0, 2, 1]])
    tmp = out.with_suffix(".tmp")
    tmp.write_text("".join(f"v {x:.9f} {y:.9f} {z:.9f}\n" for x, y, z in pts) + "".join(f"f {a+1} {b+1} {c+1}\n" for a, b, c in faces))
    tmp.replace(out)
    return out


def ball_xml(xml: str) -> str:
    f = ball_hull_file()
    return xml.replace(f'meshdir="{BALL["mesh"].parent}"', f'meshdir="{f.parent}"').replace(f'file="{BALL["mesh"].name}"', f'file="{f.name}"')
CHUNK = 64          # rollouts per IPC scene (one diverging rollout aborts only its chunk)


def sim_chunk(c, task, dt, mass_scale, init, n_samples, steps_per_sample):
    if True:
        xml = entities(task, mass_scale)
        mu = BALL["mu"] if task == "ball" else CUBE["mu"]
        mass = (BALL["mass"] if task == "ball" else CUBE["mass"]) * mass_scale
        W = len(init)
        T = I.T_from(init[:, 0:3], init[:, 3:7]); V = I.V_from(T, init[:, 7:10], init[:, 10:13])
        body_xml = ball_xml(xml["ball"]) if task == "ball" else xml[task]
        b = I.build({"ground": I.Body(xml["ground"], mu=mu), task: I.Body(body_xml, mu=mu, mass=mass)},
                    dt=dt, cand=c, n_envs=W, init={task: (T, V)})
        pos = np.zeros((W, n_samples, 3)); quat = np.zeros((W, n_samples, 4))
        for s in range(n_samples):
            p, q = b.pose(task); pos[:, s] = p; quat[:, s] = q
            if s < n_samples - 1:
                for _ in range(steps_per_sample):
                    b.step()
                if s % 20 == 0:
                    b.rigid_contacts()
        finite = np.isfinite(pos).all(axis=(1, 2)) & np.isfinite(quat).all(axis=(1, 2))
        prov = b.provenance()
        if task == "ball":
            g = [g for g in b.ents["ball"].geoms if g.n_verts][0]
            rad = np.linalg.norm(np.asarray(g.init_verts) - np.asarray(g.init_verts).mean(0), axis=1)
            v0 = I.obj_vertices(BALL["mesh"]); r0 = np.linalg.norm(v0 - v0.mean(0), axis=1)
            prov["ball_mesh"] = {"ipc_hull_vertices": int(g.n_verts), "ipc_hull_faces": int(len(g.init_faces)),
                                 "ipc_vertex_radius_m": [float(rad.min()), float(rad.max())],
                                 "original_vertex_radius_m": [float(r0.min()), float(r0.max())], "original_vertices": int(len(v0)),
                                 "rule": f"convex hull of {BALL_HULL_N} farthest-point-sampled original vertices"}
        if task == "cube":
            prov["inertia_ratio_ipc_over_manifest"] = ((2 * CUBE["half"]) ** 2 / 6) / (CUBE["inertia"] / CUBE["mass"])
        b.destroy()
        return {"pos": pos, "quat": quat, "finite": finite, "compiled": prov}


def make_simulate(provenance: list):
    def simulate(c, *, task, dt, mass_scale, init, n_samples, steps_per_sample):
        W = len(init); pos = np.full((W, n_samples, 3), np.nan); quat = np.full((W, n_samples, 4), np.nan)
        finite = np.zeros(W, bool); comp, fails = None, []
        for k in range(0, W, CHUNK):
            sl = slice(k, min(W, k + CHUNK))
            r, why = I.ISO.call("run_impact_genesis_ipc", "sim_chunk", tag=f"{c['id']}|rollouts {sl.start}-{sl.stop}", c=c,
                                task=task, dt=dt, mass_scale=mass_scale, init=init[sl], n_samples=n_samples,
                                steps_per_sample=steps_per_sample)
            if r is None:
                fails.append({"rollouts": [sl.start, sl.stop], "reason": why}); continue
            pos[sl], quat[sl], finite[sl] = r["pos"], r["quat"], r["finite"]; comp = comp or r["compiled"]
        quat = np.where(np.isfinite(quat), quat, 0.0); quat[~finite] = [1.0, 0, 0, 0]
        pos[~finite] = np.where(np.isfinite(pos[~finite]), pos[~finite], 0.0)
        provenance.append({**(comp or {}), "chunk_failures": fails, "chunk_size": CHUNK})
        return {"pos": pos, "quat": quat, "finite": finite, "truncated": 0}
    return simulate


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", choices=("ball", "cube"), required=True)
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cube-npz", type=Path, default=I.ROOT / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si_v2.npz")
    a = ap.parse_args()
    man0, _ = I.load_manifest(a.manifest)
    task_id = I.registry_id_for(a.manifest)
    if (task_id == "cube-toss") != (a.task == "cube"):
        raise ValueError(f"--task {a.task} does not match registry task {task_id}")
    _, spec, sha = I.registry_task(task_id, a.manifest)
    man = I.with_candidates(man0, spec)
    sym = None
    if a.task == "ball":
        data = ic.ball_trials(I.ROOT)
    else:
        data = ic.cube_data(a.cube_npz)
        from cube_orientation_metrics import cube_symmetric_qangle as sym
    prov: list = []
    res = ic.execute(a.task, man, "genesis_ipc_candidates", I.parse_candidate, make_simulate(prov), data, sym,
                     log=lambda s: print(s, flush=True))
    for r in res["selection_candidates"] + res["heldout"]:
        r["compiled"] = prov.pop(0)
    out = {**I.header(man0, sha, spec), "registry_task": task_id, "complete": True, **res}
    name = f"genesis_ipc_{a.task}_{'dynamic_impact_v3' if task_id == 'ball-bounce-v3' else 'dynamic_impact_v2'}.json"
    I.save_json(a.out / name, out)


if __name__ == "__main__":
    main()
