#!/usr/bin/env python3
"""Round socket strict-v8 (DRAFT), MuJoCo Warp runner.  All scoring is in socket_v8_common; this file only builds the model and
logs the per-step peg pose and the deepest peg contact (both kept on the GPU during the run, copied once at the end).

Candidates: native default (literal MuJoCo defaults), and ladder rungs = solref (timeconst, dampratio 1), solimp 0.95 0.95 0.001 0.5 2,
implicitfast / elliptic / Newton; timeconst is bisected per rung on the calibration cell (pose-based peak base penetration).
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
from pathlib import Path

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

import socket_v8_common as sc
from native_defaults import native_default_mjcf

ROOT = REPO_ROOT if (REPO_ROOT / "exp-results").exists() else Path("/root/SIM-EXP")
NCON = 4096
SOLIMP = (0.95, 0.95, 0.001, 0.5, 2.0)


@wp.kernel
def _log_step(qpos: wp.array2d(dtype=float), k: int, buf: wp.array3d(dtype=float)):
    w = wp.tid()
    for i in range(7):
        buf[k, w, i] = qpos[w, i]


@wp.kernel
def _peg_depth(nacon: wp.array(dtype=int), worldid: wp.array(dtype=int), geom: wp.array(dtype=wp.vec2i), dist: wp.array(dtype=float),
               peg: int, k: int, out: wp.array2d(dtype=float)):
    i = wp.tid()
    if i < nacon[0]:
        g = geom[i]
        if g[0] == peg or g[1] == peg:
            wp.atomic_max(out, k, worldid[i], -dist[i])


def rung(tc, extra=None):
    return {"id": f"tc={tc * 1e3:.4g}ms,dampratio=1,solimp=.95/.95/.001/.5/2", "native": False, "tc": float(tc), "dr": 1.0, "solimp": list(SOLIMP),
            "options": "implicitfast/elliptic/Newton"}


def attribution(c):
    return dict(c, id=c["id"] + ",native-solver-options", options="native")


NATIVE = {"id": "native_default", "native": True}


def runnable(c, dt):
    if c.get("native"):
        return True, ""
    if c["tc"] < 2 * dt - 1e-12:
        return False, f"timeconst {c['tc'] * 1e3:.4g} ms < 2 dt = {2 * dt * 1e3:g} ms (MuJoCo limit)"
    return True, ""


def xml(c, cell):
    dt, mu, r = cell["dt_ms"] * 1e-3, cell["mu"], sc.peg_radius_mm(cell) * 1e-3
    assets = "".join(f'<mesh name="m{i}" file="{f}"/>' for i, f in enumerate(sc.FILES))
    pieces = "".join(f'<geom name="piece{i}" type="mesh" mesh="m{i}"/>' for i in range(len(sc.FILES)))
    opts = "" if (c.get("native") or c.get("options") == "native") else ' integrator="implicitfast" cone="elliptic" solver="Newton"'
    opt = f'<option timestep="{dt}" gravity="0 0 {-sc.gravity(cell)}"{opts}/>'
    contact = "" if c.get("native") else f' solref="{c["tc"]} {c["dr"]}" solimp="{" ".join(map(str, c["solimp"]))}"'
    dflt = f'<default><geom friction="{mu} .005 .0001"{contact}/></default>'
    I = sc.peg_inertia(cell); p0, q0 = sc.initial_pose(cell)
    s = (f'<mujoco model="round_socket_v8">{opt}<compiler meshdir="{ROOT / sc.ASSET_DIR}"/>{dflt}<asset>{assets}</asset><worldbody>{pieces}'
         '<body name="peg" pos="{} {} {}" quat="{} {} {} {}"><freejoint/>'.format(*p0, *q0) +
         f'<inertial pos="0 0 0" mass="{sc.peg_mass(cell)}" diaginertia="{I[0]} {I[1]} {I[2]}"/>'
         f'<geom name="peg_geom" type="cylinder" size="{r} {sc.PEG_HALF * 1e-3}"/></body></worldbody></mujoco>')
    return native_default_mjcf(s) if c.get("native") else s


def simulate(c, cell, init_xy, dump_contacts=False):
    m = mujoco.MjModel.from_xml_string(xml(c, cell)); assert abs(m.opt.timestep - cell["dt_ms"] * 1e-3) < 1e-12
    gm = mjw.put_model(m); R = len(init_xy)
    d = mjw.make_data(m, nworld=R, nconmax=NCON // R + 64, njmax=2048)
    q = d.qpos.numpy(); q[:, 0:2] += init_xy; d.qpos.assign(q.astype(np.float32)); mjw.forward(gm, d)
    peg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "peg_geom")
    n = round(float(cell["duration_s"]) / m.opt.timestep); S = n + 1
    buf = wp.zeros((S, R, 7), dtype=float); dep = wp.zeros((S, R), dtype=float); cap = d.contact.dist.shape[0]; maxn = 0; contacts = None
    wp.launch(_log_step, dim=R, inputs=[d.qpos, 0, buf])
    for k in range(n):
        mjw.step(gm, d)
        wp.launch(_peg_depth, dim=cap, inputs=[d.nacon, d.contact.worldid, d.contact.geom, d.contact.dist, peg, k, dep])   # contacts of pose k
        wp.launch(_log_step, dim=R, inputs=[d.qpos, k + 1, buf])
        if k % 200 == 0:
            maxn = max(maxn, int(d.nacon.numpy()[0]))
        if dump_contacts and k == 1:
            contacts = _dump(d, peg, R)
    qp = buf.numpy().astype(float); dp = dep.numpy().astype(float); dp[-1] = np.nan
    return {"pos": qp[:, :, 0:3].transpose(1, 0, 2), "quat": qp[:, :, 3:7].transpose(1, 0, 2), "engine_depth": np.maximum(dp, 0.0).T,
            "truncated": int(maxn >= cap), "contacts": contacts}


def _dump(d, peg, R):
    n = int(d.nacon.numpy()[0]); g = d.contact.geom.numpy()[:n]; ps = d.contact.pos.numpy()[:n]; fr = d.contact.frame.numpy()[:n]
    ds = d.contact.dist.numpy()[:n]; wid = d.contact.worldid.numpy()[:n]; ea = d.contact.efc_address.numpy()[:n]; ef = d.efc.force.numpy()
    out = [[] for _ in range(R)]
    for i in range(n):
        if peg not in (g[i, 0], g[i, 1]):
            continue
        sgn = 1.0 if g[i, 1] == peg else -1.0
        adr = int(ea[i, 0]) if ea.ndim == 2 else int(ea[i]); w = int(wid[i])
        f = (float(ef[w, adr]) if ef.ndim == 2 else float(ef[adr])) if adr >= 0 else 0.0
        out[w].append([*ps[i], *(sgn * fr[i, 0]), max(0.0, -float(ds[i])), f])
    return [np.array(o).reshape(-1, 8) for o in out]


LADDER = {"lo": lambda dt: 2 * dt, "hi": 0.2, "increasing": True, "log": True, "make": lambda p, extra: rung(p), "extras": [None],
          "damping_selects_extra": False, "native": NATIVE, "attribution": attribution}


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--phases", default="calibration,selection,frozen,robustness,attribution")
    ap.add_argument("--frozen-configs", default="B5,B1", help="which frozen configurations execute() runs (strict-v9: B1 may run in its own process)")
    ap.add_argument("--native-matrix", action="store_true", help="only the native default (B = 1) on the frozen matrix, in its own output dir")
    ap.add_argument("--hardness-only", action="store_true", help="strict-v9 supplementary phase 5 only (calibration cell, every dt)")
    ap.add_argument("--smoke", action="store_true", help="selection conditions only, native default only (code check)")
    a = ap.parse_args(); man, sha = sc.load_manifest(a.manifest)
    engine = {"engine": "MuJoCo Warp", "mujoco_version": mujoco.__version__, "mujoco_warp_version": getattr(mjw, "__version__", None),
              "warp_version": wp.config.version, "device": str(wp.get_device()), "manifest_sha256": sha, "manifest_revision": man["revision"]}
    log = lambda s: print(s, flush=True)
    if a.smoke:
        a.out.mkdir(parents=True, exist_ok=True); idx = {c["id"]: j for j, c in enumerate(man["conditions"])}
        for cid in man["run_plan"]["1_selection"]["conditions"]:
            r = sc.run_condition(NATIVE, man["conditions"][idx[cid]], man["seed0"] + idx[cid], man, simulate, a.out / "traj")
            log(sc.fmt(r)); (a.out / f"smoke_{cid}.json").write_text(json.dumps(sc.json_safe(r), indent=1))
        return
    if a.hardness_only:
        a.out.mkdir(parents=True, exist_ok=True)
        res = {"engine": engine, "interpretation": sc.INTERPRETATION, "protocol": man["protocol"], "task": man["task"], "revision": man["revision"],
               "hardness_vs_time_step": sc.hardness_vs_dt(man, LADDER, simulate, runnable, log)}
        (a.out / "result.json").write_text(json.dumps(sc.json_safe(res), indent=1) + "\n")
        return
    if a.native_matrix:
        res = sc.run_native_matrix(man, LADDER, simulate, a.out, log=log)
        res.update(engine=engine, protocol=man["protocol"], task=man["task"], revision=man["revision"])
        (a.out / "result.json").write_text(json.dumps(sc.json_safe(res), indent=1) + "\n")
        return
    res = sc.execute(man, engine, simulate, runnable, LADDER, a.out, log=log, phases=tuple(a.phases.split(",")),
                     frozen_configs=tuple(a.frozen_configs.split(",")))
    res.update(protocol=man["protocol"], task=man["task"], revision=man["revision"])
    (a.out / "result.json").write_text(json.dumps(sc.json_safe(res), indent=1) + "\n")


if __name__ == "__main__":
    main()
