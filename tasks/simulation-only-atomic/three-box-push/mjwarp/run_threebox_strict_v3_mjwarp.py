#!/usr/bin/env python3
"""Three-box push strict-v3 (manifest-driven) B=1/B=5 selection + frozen held-out matrix, MuJoCo Warp.

Force profile, slip rule, perturbations and seeds come from the manifest and
strict_v3_common, shared verbatim with the SAP driver.  The scene XML is the
historical static-friction-v2 three_box_xml (pair stiffness k_pair=1e4, d=0.9).
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from native_defaults import native_default_mjcf
from run_static_friction_v2 import three_box_xml
import strict_v3_common as common

G = 9.81
SAMPLE_S = 0.01


def parse_candidate(text: str) -> dict[str, Any]:
    c = {"id": text, "impratio": 1.0, "solreffriction_tc": 0.0, "noslip": 0}
    if text.strip() == "native default":
        c["native_default"] = True
        return c
    for part in text.replace(" ", "").split(","):
        k, v = part.split("=")
        if k == "tc":
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
    xml = three_box_xml(dt=dt, mass=mass, mu=mu, k_pair=1e4, impratio=c["impratio"],
                        solreffriction_tc=c["solreffriction_tc"], noslip_iterations=c["noslip"], box_half=half)
    if c.get("native_default"):
        xml = native_default_mjcf(xml)
    model = mujoco.MjModel.from_xml_string(xml)
    bids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"box_{i}") for i in range(3)]
    gids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"box_{i}_geom") for i in range(3)]
    qa = [int(model.jnt_qposadr[model.body_jntadr[b]]) for b in bids]
    pushed = 1 if cell["pushed_body"] == "middle" else 2
    support = pushed - 1
    iface = frozenset((gids[support], gids[pushed]))
    force = common.box_force_N(cell, G)
    expected = "release" if force > common.box_threshold_N(cell, G) else "retain"
    if "expected" in cell and cell["expected"] != expected:
        raise ValueError(f"manifest label disagrees with analytic threshold for {cell['id']}")
    xy, yaw, scale = common.box_perturbations(seed, R)
    q0 = np.tile(model.qpos0, (R, 1))
    for i in range(3):
        q0[:, qa[i]:qa[i] + 2] += xy[:, i]
        q0[:, qa[i] + 3] = np.cos(yaw[:, i] / 2); q0[:, qa[i] + 4:qa[i] + 6] = 0; q0[:, qa[i] + 6] = np.sin(yaw[:, i] / 2)
    gm = mjw.put_model(model)
    d = mjw.make_data(model, nworld=R, nconmax=64 * R, njmax=256 * R)
    d.qpos.assign(q0.astype(np.float32)); mjw.forward(gm, d)
    buf = wp.zeros(64 * R, dtype=wp.spatial_vectorf)
    applied = np.zeros((R, model.nbody, 6), np.float32)
    total = common.BOX_SETTLE_S + common.BOX_RAMP_S + common.BOX_HOLD_S
    steps, every = round(total / dt), max(1, round(SAMPLE_S / dt))
    T, X, D, F = [], [], [], []
    base = None
    t0 = time.perf_counter()
    for step in range(steps):
        t = step * dt
        fac = float(np.clip((t - common.BOX_SETTLE_S) / common.BOX_RAMP_S, 0, 1))
        applied[:] = 0; applied[:, bids[pushed], 0] = force * scale * fac
        d.xfrc_applied.assign(applied)
        mjw.step(gm, d)
        tn = (step + 1) * dt
        if (step + 1) % every == 0 and tn >= common.BOX_SETTLE_S - 1e-12:
            qp = d.qpos.numpy()
            rx = qp[:, qa[pushed]] - qp[:, qa[support]]
            if base is None:
                base = rx.copy()
            fn = np.zeros(R); cnt = np.zeros(R)
            n = int(d.nacon.numpy()[0])
            if n:
                mjw.contact_force(gm, d, wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32), False, buf)
                f = buf.numpy()[:n]; wid = d.contact.worldid.numpy()[:n]; geom = d.contact.geom.numpy()[:n]
                for i in range(n):
                    if frozenset((int(geom[i, 0]), int(geom[i, 1]))) == iface:
                        fn[wid[i]] += max(0.0, float(f[i, 0])); cnt[wid[i]] += 1
            T.append(tn - common.BOX_SETTLE_S); X.append(rx - base); D.append(cnt > 0); F.append(fn)
    X, D, F = np.asarray(X), np.asarray(D, float), np.asarray(F)
    finite = np.isfinite(X).all(axis=0)
    # Duty gate is evaluated while the outcome is being decided: up to slip detection (|rel_x| < 1 mm).
    pre = np.abs(X) < common.BOX_SLIP_M  # samples after slip detection do not count against duty
    res =common.box_summary(np.asarray(T), X, np.where(pre, D, 1.0), F, expected=expected, finite=finite, truncated=0)
    res["duty_window"] = "force window, up to slip detection"
    res.update({"cell": cell, "force_N": force, "seed": seed, "candidate": c, "wall_s": time.perf_counter() - t0,
                "compiled": {"opt_cone": int(model.opt.cone), "opt_impratio": float(model.opt.impratio),
                             "opt_noslip_iterations": int(model.opt.noslip_iterations),
                             "pair_solref": model.pair_solref.tolist(), "pair_solreffriction": model.pair_solreffriction.tolist(),
                             "timestep": float(model.opt.timestep)}})
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    man, sha = common.load_manifest(a.manifest)
    R, seed0 = man["R"], man["seed0"]
    cands = [parse_candidate(t) for t in man["candidate_rule"]["mujoco_warp_candidates"]]
    selection = []
    for i, c in enumerate(cands):
        r = run_cell(c, man["selection"], common.cell_seed(seed0, "selection", i), R); r["candidate_index"] = i + 1
        selection.append(r)
        print("select", i + 1, c["id"], r["agree_and_valid_count"], f'creep={r["tail_creep_um_s_mean"]:.3f}um/s',
              f'disp={r["final_rel_disp_mm_mean"]:.4f}mm duty={r["contact_duty_min"]:.3f}', flush=True)
    win = common.box_choose(selection); frozen = cands[win]
    print("frozen", frozen, flush=True)
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(frozen, cell, common.cell_seed(seed0, "heldout", j), R); held.append(r)
        print("held", cell["id"], r["expected"], r["agree_and_valid_count"], f'disp={r["final_rel_disp_mm_mean"]:.3f}mm',
              f'creep={r["tail_creep_um_s_mean"]:.2f} duty={r["contact_duty_min"]:.3f}', flush=True)
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
           "engine": "MuJoCo Warp", "mujoco_version": mujoco.__version__, "warp_version": wp.config.version,
           "device": str(wp.get_device()), "B1": selection[0], "selection_candidates": selection,
           "frozen_index": win + 1, "frozen_candidate": frozen, "heldout": held,
           "heldout_accuracy_total": [sum(r["agree_and_valid_count"] for r in held), R * len(held)]}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "mjwarp_three_box_strict_v3.json").write_text(json.dumps(common.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
