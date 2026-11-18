#!/usr/bin/env python3
"""Cube-toss resting tail (sustained-contact v3), Genesis + IPC (preliminary).  Shared logic in tail_v2_common.

Single candidate inherited from the Genesis-IPC cube-toss frozen winner.  Speed/omega are IPC's own velocities (the
Genesis-side velocity of an IPC-only body is zeroed by the bridge); support load = IPC's own barrier force from the
floor (ContactSystemFeature, direct readout); contact count = cube vertices carrying barrier force.  The momentum-balance
load is kept as a diagnostic (its difference from the direct load is the Newton residual).
"""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np

import genesis_ipc_common as I
import impact_v2_common as ic
import tail_v2_common as tc
from run_tail_genesis_rigid import entities

TASK = "cube-tail"
CORNERS = np.array(list(itertools.product((-1, 1), repeat=3)), float)


CHUNK = 64


def sim_chunk(c, dt, mass_scale, mu, scale, init, n_samples, steps_per_sample):
    if True:
        xml = entities(mass_scale, mu, scale); W = len(init); h = tc.HALF * scale
        T = I.T_from(init[:, 0:3], init[:, 3:7]); V = I.V_from(T, init[:, 7:10], init[:, 10:13])
        b = I.build({"floor": I.Body(xml["floor"], mu=mu), "cube": I.Body(xml["cube"], mu=mu, mass=tc.MASS * mass_scale)},
                    dt=dt, cand=c, n_envs=W, init={"cube": (T, V)})
        dh = I.d_hat(c)
        P = np.zeros((W, n_samples, 3)); Q = np.zeros((W, n_samples, 4)); SP = np.zeros((W, n_samples))
        OM = np.zeros((W, n_samples)); FN = np.zeros((W, n_samples)); NC = np.zeros((W, n_samples))
        FM = np.zeros((W, n_samples)); GAP = np.zeros((W, n_samples))
        for s in range(n_samples):
            p, q = b.pose("cube"); v, w, _ = b.vel("cube")
            P[:, s] = p; Q[:, s] = q; SP[:, s] = np.linalg.norm(v, axis=1); OM[:, s] = np.linalg.norm(w, axis=1)
            d = b.direct_force("cube")                                   # IPC barrier force from the floor (direct)
            FN[:, s] = np.maximum(d["N"][:, 2], 0.0); NC[:, s] = d["nv_active"]
            FM[:, s] = b.contact_force("cube")[:, 2]                     # momentum balance (diagnostic)
            R = b.T["cube"][:, :3, :3]
            zc = p[:, 2:3] + np.einsum("bij,kj->bki", R, CORNERS * h)[:, :, 2]
            GAP[:, s] = zc.min(axis=1)
            if s < n_samples - 1:
                for _ in range(steps_per_sample):
                    b.step()
                if s % 50 == 0:
                    b.rigid_contacts()
        finite = np.isfinite(P).all(axis=(1, 2)) & np.isfinite(Q).all(axis=(1, 2))
        prov = b.provenance(); b.destroy()
        return {"pos": P, "quat": Q, "speed": SP, "omega": OM, "fn": FN, "ncon": NC, "fn_momentum": FM, "min_corner_gap": GAP,
                "finite": finite, "compiled": prov}


KEYS = ("pos", "quat", "speed", "omega", "fn", "ncon")
DIAG = ("fn_momentum", "min_corner_gap")


def make_simulate(provenance: list):
    def simulate(c, *, dt, mass_scale, mu, scale, init, n_samples, steps_per_sample):
        W = len(init)
        out = {"pos": np.zeros((W, n_samples, 3)), "quat": np.tile([1.0, 0, 0, 0], (W, n_samples, 1)),
               "speed": np.full((W, n_samples), np.inf), "omega": np.full((W, n_samples), np.inf),
               "fn": np.zeros((W, n_samples)), "ncon": np.zeros((W, n_samples))}
        finite = np.zeros(W, bool); comp, fails = None, []
        diag = {k: np.full((W, n_samples), np.nan) for k in DIAG}
        for k in range(0, W, CHUNK):
            sl = slice(k, min(W, k + CHUNK))
            r, why = I.ISO.call("run_tail_genesis_ipc", "sim_chunk", tag=f"{c['id']}|rollouts {sl.start}-{sl.stop}", c=c, dt=dt,
                                mass_scale=mass_scale, mu=mu, scale=scale, init=init[sl], n_samples=n_samples,
                                steps_per_sample=steps_per_sample)
            if r is None:
                fails.append({"rollouts": [sl.start, sl.stop], "reason": why}); continue
            for key in KEYS:
                out[key][sl] = r[key]
            for key in DIAG:
                diag[key][sl] = r[key]
            finite[sl] = r["finite"]; comp = comp or r["compiled"]
        # last-half window: direct vs momentum support load (Newton residual) and the IPC rest gap (no overlap exists)
        tail = slice(n_samples // 2, None); mg = tc.MASS * mass_scale * tc.G
        with np.errstate(all="ignore"):
            chk = {"load_readout": "direct: IPC barrier force from the floor (ContactSystemFeature)",
                   "direct_minus_momentum_over_mg_max": float(np.nanmax(np.abs(out["fn"][:, tail] - diag["fn_momentum"][:, tail])) / mg),
                   "rest_gap_mm_median": float(np.nanmedian(diag["min_corner_gap"][:, -1]) * 1e3),
                   "overlap_gate_note": "IPC never interpenetrates; the overlap gate cannot fail and is reported as not applicable"}
        provenance.append({**(comp or {}), "chunk_failures": fails, "chunk_size": CHUNK, "readout_check": chk})
        return {**out, "finite": finite, "truncated": 0}
    return simulate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cube-toss-artifact", type=Path, required=True)
    ap.add_argument("--cube-npz", type=Path, default=I.ROOT / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si_v2.npz")
    a = ap.parse_args()
    _, spec, sha = I.registry_task(TASK, a.manifest)
    man0, _ = I.load_manifest(a.manifest)
    toss = json.loads(a.cube_toss_artifact.read_text())
    if not toss.get("complete") or toss.get("engine") != I.ENGINE or toss.get("frozen_candidate") is None:
        raise ValueError("cube-toss artifact is not a complete Genesis IPC run with a frozen candidate")
    if toss["manifest_sha256"] != man0["frozen_from"]["manifest_sha256"]:
        raise ValueError("cube-toss artifact used a different manifest than this task's frozen_from")
    text = toss["frozen_candidate"]["text"]
    inherited = {**spec, "candidates": [text]}
    man = I.with_candidates(man0, inherited)
    prov: list = []
    res = tc.execute(man, "genesis_ipc_candidates", I.parse_candidate, make_simulate(prov), ic.cube_data(a.cube_npz),
                     log=lambda s: print(s, flush=True), standoff=True)      # a3: mirror of the overlap gate
    for r in res["selection_candidates"] + res["heldout"]:
        r["compiled"] = prov.pop(0)
    out = {**I.header(man0, sha, inherited), "complete": True,
           "inherited_from": {"artifact": str(a.cube_toss_artifact), "sha256": hashlib.sha256(a.cube_toss_artifact.read_bytes()).hexdigest(),
                              "candidate": text}, **res}
    I.save_json(a.out / "genesis_ipc_cube_tail_v3.json", out)


if __name__ == "__main__":
    main()
