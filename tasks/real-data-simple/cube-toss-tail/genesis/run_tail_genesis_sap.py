#!/usr/bin/env python3
"""Cube-toss resting tail (sustained-contact v3), Genesis SAP coupler.  Shared logic in tail_v2_common.

The single candidate is inherited from the Genesis cube-toss frozen winner (manifest frozen_from rule); the artifact
path and sha256 are recorded.  Support load = sum of |f.n| over cube-floor contacts.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

import genesis_sap_common as G
import impact_v2_common as ic
import tail_v2_common as tc

TASK = "cube-tail"


def entities(mass_scale, mu, scale):
    h = tc.HALF * scale; I = 0.2950002913583124 * h ** 2 * mass_scale
    return {"floor": G.mjcf(f'<geom name="floor" type="plane" size="10 10 .1" friction="{mu} 0 0"/>'),
            "cube": G.mjcf(f'<body name="cube"><freejoint/><inertial pos="0 0 0" mass="{tc.MASS * mass_scale}" '
                           f'diaginertia="{I} {I} {I}"/><geom name="cube_geom" type="box" size="{h} {h} {h}" friction="{mu} 0 0"/></body>')}


def sim_chunk(c, *, dt, mass_scale, mu, scale, init, n_samples, steps_per_sample):
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
    compiled=b.provenance(); b.destroy()
    return {"pos": P, "quat": Q, "speed": SP, "omega": OM, "fn": FN, "ncon": NC, "finite": finite, "truncated": 0, "compiled": compiled}


def make_simulate(provenance: list):
    def simulate(c, *, dt, mass_scale, mu, scale, init, n_samples, steps_per_sample):
        parts=[];comp=[];failures=[]
        for k in range(0,len(init),64):
            sl=slice(k,min(len(init),k+64));n=sl.stop-sl.start
            r,why=G.ISO.call('run_tail_genesis_sap','sim_chunk',tag=f"{c['id']}|rollouts {sl.start}-{sl.stop}",c=c,dt=dt,mass_scale=mass_scale,mu=mu,scale=scale,init=init[sl],n_samples=n_samples,steps_per_sample=steps_per_sample)
            if r is None:
                r=dict(pos=np.zeros((n,n_samples,3)),quat=np.tile([1.,0,0,0],(n,n_samples,1)),finite=np.zeros(n,bool))
                r.update(speed=np.zeros((n,n_samples)),omega=np.zeros((n,n_samples)),fn=np.zeros((n,n_samples)),ncon=np.zeros((n,n_samples)))
                failures.append({'rollouts':[sl.start,sl.stop],'reason':why})
            else:comp.append(r['compiled'])
            parts.append(r)
        provenance.append({'chunks':comp,'chunk_size':64,'chunk_failures':failures})
        out={key:np.concatenate([r[key] for r in parts],axis=0) for key in ('pos','quat','speed','omega','fn','ncon','finite')};out['truncated']=0
        return out
    return simulate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cube-toss-artifact", type=Path, required=True,
                    help="Genesis cube-toss genesis_sap_cube_dynamic_impact_v2.json supplying the frozen candidate")
    ap.add_argument("--cube-npz", type=Path, default=G.ROOT / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si_v2.npz")
    a = ap.parse_args()
    _, spec, sha = G.registry_task(TASK, a.manifest)
    man0, _ = G.load_manifest(a.manifest)
    toss = json.loads(a.cube_toss_artifact.read_text())
    if not toss.get("complete") or toss.get("engine") != G.ENGINE or toss.get("frozen_candidate") is None:
        raise ValueError("cube-toss artifact is not a complete Genesis SAP coupler run with a frozen candidate")
    if toss["manifest_sha256"] != man0["frozen_from"]["manifest_sha256"]:
        raise ValueError("cube-toss artifact used a different manifest than this task's frozen_from")
    _, toss_spec, _ = G.registry_task("cube-toss", G.ROOT / "protocol/manifests" / man0["frozen_from"]["manifest"])
    text = toss["frozen_candidate"]["text"]
    inherited = {**spec, "candidates": [text], "nonnative": toss_spec.get("nonnative", {})}
    man = G.with_candidates(man0, inherited)
    prov: list = []
    res = tc.execute(man, "genesis_sap_candidates", lambda t: G.parse_candidate(t, inherited), make_simulate(prov),
                     ic.cube_data(a.cube_npz), log=lambda s: print(s, flush=True))
    for r in res["selection_candidates"] + res["heldout"]:
        r["compiled"] = prov.pop(0)
    out = {**G.header(man0, sha, inherited), "complete": True,
           "inherited_from": {"artifact": str(a.cube_toss_artifact), "sha256": hashlib.sha256(a.cube_toss_artifact.read_bytes()).hexdigest(),
                              "candidate": text}, **res}
    G.save_json(a.out / "genesis_sap_cube_tail_v3.json", out)


if __name__ == "__main__":
    main()
