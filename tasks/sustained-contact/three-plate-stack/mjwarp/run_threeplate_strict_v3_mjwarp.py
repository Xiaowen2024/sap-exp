#!/usr/bin/env python3
"""Three-plate strict-v3 (manifest-driven) B=1/B=5 selection + frozen held-out matrix, MuJoCo Warp.

Physical scene, perturbations, gates and seeds come from the manifest and
strict_v3_common, shared verbatim with the SAP driver.  Contact normal loads are
read with mujoco_warp.contact_force, which is correct for both pyramidal (the
native default cone) and elliptic cones.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import time
from pathlib import Path
from typing import Any

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from native_defaults import native_default_mjcf
from mujoco_capacity_audit import CapacityAudit
from strict_v3_common import (cell_seed, json_safe, load_manifest, plate_choose, plate_perturbations,
                              plate_summary)

G = 9.81
SAMPLE_S = 0.01


def parse_candidate(text: str) -> dict[str, Any]:
    if text.strip() == "native default":
        return {"id": "native_default", "native_default": True}
    kv = dict(part.split("=") for part in text.replace(" ", "").split(","))
    return {"id": text, "timeconst_s": float(re.sub("ms$", "", kv["tc"])) * 1e-3,
            "dampratio": float(kv["dr"]), "dmax": float(kv["dmax"])}


def xml_for(c: dict[str, Any], *, h: float, mass: float, radius: float, offset: float, dt: float) -> str:
    bodies = "".join(
        f'<body name="plate_{i}" pos="{i * offset} 0 {(i + .5) * h}"><freejoint/>'
        f'<geom name="plate_{i}_geom" type="cylinder" size="{radius} {h / 2}" mass="{mass}" '
        f'friction="0.8 .005 .0001"/></body>' for i in range(3))
    tc, dr, dmax = (float(c.get("timeconst_s", .02)), float(c.get("dampratio", 1.)), float(c.get("dmax", .9)))
    xml = f'''<mujoco model="three_plate_strict_v3">
 <option timestep="{dt}" gravity="0 0 -{G}" integrator="implicitfast" solver="Newton"
         cone="elliptic" iterations="100" tolerance="1e-10"/>
 <default><geom margin="0" gap="0" solref="{tc} {dr}" solimp="{dmax} {dmax} .001 .5 2"/></default>
 <worldbody><geom name="ground" type="plane" size="2 2 .1" friction="0.8 .005 .0001"/>{bodies}</worldbody>
</mujoco>'''
    return native_default_mjcf(xml) if c.get("native_default") else xml


def compiled_contact(model: mujoco.MjModel) -> dict[str, Any]:
    return {"opt_cone": int(model.opt.cone), "opt_integrator": int(model.opt.integrator),
            "opt_solver": int(model.opt.solver), "opt_iterations": int(model.opt.iterations),
            "opt_tolerance": float(model.opt.tolerance), "opt_impratio": float(model.opt.impratio),
            "geom_solref": model.geom_solref[1].tolist(), "geom_solimp": model.geom_solimp[1].tolist(),
            "geom_friction": model.geom_friction.tolist(),
            "timestep": float(model.opt.timestep)}


def interface_loads(m, d, gm, nworld: int, gids: list[int], ground: int, buf) -> tuple[np.ndarray, np.ndarray]:
    fn = np.zeros((nworld, 3)); cnt = np.zeros((nworld, 3))
    n = int(d.nacon.numpy()[0])
    if n == 0:
        return fn, cnt
    ids = wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32)
    mjw.contact_force(gm, d, ids, False, buf)
    f = buf.numpy()[:n]
    wid = d.contact.worldid.numpy()[:n]
    geom = d.contact.geom.numpy()[:n]
    pairs = {frozenset((ground, gids[0])): 0, frozenset((gids[0], gids[1])): 1, frozenset((gids[1], gids[2])): 2}
    for i in range(n):
        which = pairs.get(frozenset((int(geom[i, 0]), int(geom[i, 1]))))
        if which is not None:
            fn[wid[i], which] += max(0.0, float(f[i, 0])); cnt[wid[i], which] += 1
    return fn, cnt


def run_cell(c: dict[str, Any], cell: dict[str, Any], seed: int, R: int, duration: float) -> dict[str, Any]:
    h, mass = cell["thickness_mm"] * 1e-3, cell["mass_per_plate_kg"]
    radius, offset, dt = cell["radius_mm"] * 1e-3, cell["interplate_x_offset_mm"] * 1e-3, cell["dt_ms"] * 1e-3
    model = mujoco.MjModel.from_xml_string(xml_for(c, h=h, mass=mass, radius=radius, offset=offset, dt=dt))
    bids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"plate_{i}") for i in range(3)]
    gids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"plate_{i}_geom") for i in range(3)]
    ground = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    qa = [int(model.jnt_qposadr[model.body_jntadr[b]]) for b in bids]
    va = [int(model.jnt_dofadr[model.body_jntadr[b]]) for b in bids]
    xy, yaw = plate_perturbations(seed, R)
    q0 = np.tile(model.qpos0, (R, 1))
    for i in range(3):
        q0[:, qa[i]:qa[i] + 2] += xy[:, i]
        q0[:, qa[i] + 3] = np.cos(yaw[:, i] / 2); q0[:, qa[i] + 4:qa[i] + 6] = 0; q0[:, qa[i] + 6] = np.sin(yaw[:, i] / 2)
    gm = mjw.put_model(model)
    d = mjw.make_data(model, nworld=R, nconmax=64 * R, njmax=256 * R)
    d.qpos.assign(q0.astype(np.float32)); mjw.forward(gm, d)
    capacity = CapacityAudit(d, R); capacity.record(d)
    buf = wp.zeros(64 * R, dtype=wp.spatial_vectorf)
    steps, every = round(duration / dt), max(1, round(SAMPLE_S / dt))
    Z, V, W, F, C = [], [], [], [], []
    t0 = time.perf_counter()
    for step in range(steps + 1):
        if step % every == 0:
            qp, qv = d.qpos.numpy(), d.qvel.numpy()
            Z.append(np.stack([qp[:, a + 2] for a in qa], 1))
            V.append(np.stack([qv[:, a:a + 3] for a in va], 1)); W.append(np.stack([qv[:, a + 3:a + 6] for a in va], 1))
            f, n = interface_loads(model, d, gm, R, gids, ground, buf); F.append(f); C.append(n)
        if step < steps:
            mjw.step(gm, d)
            capacity.record(d)
    bad_capacity, capacity_details = capacity.result()
    overflow = int(bad_capacity.sum())
    res = plate_summary(np.asarray(Z), np.asarray(V), np.asarray(W), np.asarray(F), np.asarray(C),
                        h=h, mass=mass, g=G, sample_dt=every * dt, truncated=overflow)
    res.update({"cell": cell, "seed": seed, "candidate": c, "compiled": compiled_contact(model),
                "capacity_audit": capacity_details,
                "wall_s": time.perf_counter() - t0})
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    man, sha = load_manifest(a.manifest)
    R, seed0, dur = man["R"], man["seed0"], man["duration_s"]
    cands = [parse_candidate(t) for t in man["candidate_rule"]["mujoco_warp_candidates"]]
    sel = man["selection"]
    selection = []
    for i, c in enumerate(cands):
        r = run_cell(c, sel, cell_seed(seed0, "selection", i), R, dur); r["candidate_index"] = i + 1
        selection.append(r)
        print("select", i + 1, c["id"], r["pass_count"], r["failed_gate_counts"], f'{r["max_overlap_mm_mean"]:.4f}mm',
              f'{r["load_error_max_mean"]:.4%}', flush=True)
    win = plate_choose(selection); frozen = cands[win]
    print("frozen", frozen, flush=True)
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(frozen, cell, cell_seed(seed0, "heldout", j), R, dur); held.append(r)
        print("held", cell["id"], r["pass_count"], r["failed_gate_counts"], f'{r["max_overlap_mm_mean"]:.4f}mm',
              f'{r["load_error_max_mean"]:.4%}', flush=True)
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
           "engine": "MuJoCo Warp", "mujoco_version": mujoco.__version__, "warp_version": wp.config.version,
           "device": str(wp.get_device()), "load_readout": "mujoco_warp.contact_force (cone-correct)",
           "B1": selection[0], "selection_candidates": selection, "frozen_index": win + 1, "frozen_candidate": frozen,
           "heldout": held,
           "heldout_pass_total": [sum(r["pass_count"] for r in held), R * len(held)]}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "mjwarp_three_plate_strict_v3.json").write_text(json.dumps(json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
