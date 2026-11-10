#!/usr/bin/env python3
"""Three-box push strict-v4 (manifest-driven): two-cell B=1/B=5 selection, frozen held-out matrix with
retain/slip/tip labels, and a +-10% normal-stiffness sensitivity check.  MuJoCo Warp.

Classification and perturbations come from strict_v3_common (shared verbatim with the SAP driver):
tip = any box tilt > 10 deg, slip = contact-face-centre displacement >= 1 mm, first event wins.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from native_defaults import native_default_mjcf
from mujoco_capacity_audit import CapacityAudit
from run_static_friction_v2 import three_box_xml
import strict_v3_common as common

G = 9.81
SAMPLE_S = 0.01


def parse_candidate(text: str) -> dict[str, Any]:
    c = {"id": text, "k_pair": 1e4, "impratio": 1.0, "solreffriction_tc": 0.0, "noslip": 0}
    if text.strip() == "native default":
        c["native_default"] = True
        return c
    for part in text.replace(" ", "").split(","):
        k, v = part.split("=")
        if k == "k":
            c["k_pair"] = float(v)
        elif k == "tc":
            c["solreffriction_tc"] = float(v.removesuffix("ms")) * 1e-3
        elif k == "impratio":
            c["impratio"] = float(v)
        elif k == "noslip":
            c["noslip"] = int(v)
        else:
            raise ValueError(text)
    return c


def run_cell(c: dict[str, Any], cell: dict[str, Any], seed: int, R: int) -> dict[str, Any]:
    dt, mass, mu, half = cell["dt_ms"] * 1e-3, cell["mass_kg"], cell["mu"], cell["box_halfwidth_mm"] * 1e-3
    xml = three_box_xml(dt=dt, mass=mass, mu=mu, k_pair=c["k_pair"], impratio=c["impratio"],
                        solreffriction_tc=c["solreffriction_tc"], noslip_iterations=c["noslip"], box_half=half)
    if c.get("native_default"):
        xml = native_default_mjcf(xml)
    model = mujoco.MjModel.from_xml_string(xml)
    bids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"box_{i}") for i in range(3)]
    gids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"box_{i}_geom") for i in range(3)]
    qa = [int(model.jnt_qposadr[model.body_jntadr[b]]) for b in bids]
    pushed = 1 if cell["pushed_body"] == "middle" else 2
    iface = frozenset((gids[pushed - 1], gids[pushed]))
    expected, _ = common.box_v4_expected(cell)
    if expected != cell["expected"]:
        raise ValueError(f"manifest label disagrees with analytic rule for {cell['id']}")
    force = common.box_v4_force_mg(cell) * mass * G
    xy, yaw, scale = common.box_perturbations(seed, R)
    q0 = np.tile(model.qpos0, (R, 1))
    for i in range(3):
        q0[:, qa[i]:qa[i] + 2] += xy[:, i]
        q0[:, qa[i] + 3] = np.cos(yaw[:, i] / 2); q0[:, qa[i] + 4:qa[i] + 6] = 0; q0[:, qa[i] + 6] = np.sin(yaw[:, i] / 2)
    gm = mjw.put_model(model)
    d = mjw.make_data(model, nworld=R, nconmax=64 * R, njmax=256 * R)
    d.qpos.assign(q0.astype(np.float32)); mjw.forward(gm, d)
    capacity = CapacityAudit(d, R); capacity.record(d)
    buf = wp.zeros(64 * R, dtype=wp.spatial_vectorf)
    applied = np.zeros((R, model.nbody, 6), np.float32)
    total = common.BOX_SETTLE_S + common.BOX_RAMP_S + common.BOX_HOLD_S
    steps, every = round(total / dt), max(1, round(SAMPLE_S / dt))
    T, P, Q, D, F = [], [], [], [], []
    t0 = time.perf_counter()
    for step in range(steps):
        t = step * dt
        fac = float(np.clip((t - common.BOX_SETTLE_S) / common.BOX_RAMP_S, 0, 1))
        applied[:] = 0; applied[:, bids[pushed], 0] = force * scale * fac
        d.xfrc_applied.assign(applied)
        mjw.step(gm, d)
        capacity.record(d)
        tn = (step + 1) * dt
        if (step + 1) % every == 0 and tn >= common.BOX_SETTLE_S - 1e-12:
            qp = d.qpos.numpy().astype(np.float64)
            P.append(np.stack([qp[:, a:a + 3] for a in qa], 1)); Q.append(np.stack([qp[:, a + 3:a + 7] for a in qa], 1))
            fn = np.zeros(R); cnt = np.zeros(R)
            n = int(d.nacon.numpy()[0])
            if n:
                mjw.contact_force(gm, d, wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32), False, buf)
                f = buf.numpy()[:n]; wid = d.contact.worldid.numpy()[:n]; geom = d.contact.geom.numpy()[:n]
                for i in range(n):
                    if frozenset((int(geom[i, 0]), int(geom[i, 1]))) == iface:
                        fn[wid[i]] += max(0.0, float(f[i, 0])); cnt[wid[i]] += 1
            T.append(tn - common.BOX_SETTLE_S); D.append(cnt > 0); F.append(fn)
    P, Q = np.asarray(P), np.asarray(Q)
    Q = Q / np.linalg.norm(Q, axis=-1, keepdims=True)
    bad_capacity, capacity_details = capacity.result()
    res = common.box_v4_summary(np.asarray(T), P, Q, np.asarray(D, float), np.asarray(F), half=half, pushed=pushed,
                                expected=expected, finite=np.asarray(capacity_details["finite_by_world"]), truncated=bad_capacity)
    res.update({"cell": cell, "force_N": force, "seed": seed, "candidate": c, "wall_s": time.perf_counter() - t0,
                "capacity_audit": capacity_details,
                "compiled": {"opt_cone": int(model.opt.cone), "opt_impratio": float(model.opt.impratio),
                             "opt_noslip_iterations": int(model.opt.noslip_iterations),
                             "pair_solref": model.pair_solref.tolist(), "pair_solreffriction": model.pair_solreffriction.tolist(),
                             "timestep": float(model.opt.timestep)}})
    return res


def fmt(r: dict[str, Any]) -> str:
    return (f'{r["agree_and_valid_count"]:2d}/20 obs={r["observed_counts"]} face={r["face_disp_final_mm_median"]:.3f}mm '
            f'com={r["com_rel_disp_final_mm_median"]:.3f}mm tilt={r["tilt_max_deg_median"]:.2f}deg creep={r["hold_face_creep_um_s_mean"]:.1f}um/s')


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    man, sha = common.load_manifest(a.manifest)
    R, seed0 = man["R"], man["seed0"]
    cands = [parse_candidate(t) for t in man["candidate_rule"]["mujoco_warp_candidates"]]
    sels = man["selection"]
    selection, walls = [], []
    for i, c in enumerate(cands):
        rows = [run_cell(c, s, seed0 + k, R) for k, s in enumerate(sels)]
        selection.append(rows); walls.append(sum(r["wall_s"] for r in rows))
        for r in rows:
            print("select", i + 1, c["id"], r["cell"]["id"], r["expected"], fmt(r), flush=True)
    win = common.box_v4_choose(selection, walls); frozen = cands[win]
    print("frozen", frozen, flush=True)
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(frozen, cell, seed0 + 1000 + j, R); held.append(r)
        print("held", cell["id"], r["expected"], fmt(r), flush=True)
    sens = []
    byid = {c["id"]: c for c in man["held_out_conditions"]}
    if frozen.get("native_default"):
        sens.append({"skipped": "frozen setting is the native default; no declared stiffness parameter to scale"})
    else:
        k = 0
        for cid in man["parameter_sensitivity"]["cells"]:
            for s in man["parameter_sensitivity"]["scales"]:
                c = dict(frozen); c["k_pair"] = frozen["k_pair"] * s; c["id"] = f'{frozen["id"]} | k x{s}'
                r = run_cell(c, byid[cid], seed0 + 2000 + k, R); r["scale"] = s; sens.append(r); k += 1
                print("sens", cid, s, fmt(r), flush=True)
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
           "audit_revision": "capacity-and-nonfinite-v1-20261109",
           "engine": "MuJoCo Warp", "mujoco_version": mujoco.__version__, "warp_version": wp.config.version,
           "device": str(wp.get_device()), "B1": selection[0], "selection_candidates": selection,
           "frozen_index": win + 1, "frozen_candidate": frozen, "heldout": held, "parameter_sensitivity": sens,
           "heldout_accuracy_total": [sum(r["agree_and_valid_count"] for r in held), R * len(held)]}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "mjwarp_three_box_strict_v4.json").write_text(json.dumps(common.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
