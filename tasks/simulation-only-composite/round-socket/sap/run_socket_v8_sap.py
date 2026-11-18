"""Private SAP Warp round-socket strict-v8 (DRAFT) driver.  All scoring is in SIM-EXP socket_v8_common; this file only builds the
scene and logs, every step, the peg pose, the deepest peg contact (max(0, -contact_env_phi0)) and, for rim cells, the contact set of
the second step (point, normal = R_WC column 2, depth, normal force = contact_gamma normal / dt).

Candidates: native default (SAP literal material defaults) and ladder rungs (ke bisected on the calibration cell, tau = smallest of
{1.5, 3, 6, 12, 24} ms that passes the damping check, gap = 0.1 mm).
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import warp as wp
import yaml

from sap_warp.collision import SapCollisionPipeline
from sap_warp.loader import load_sap_scene
from sap_warp.sap_kinematics import sap_eval_fk
from sap_warp.solver_sap import SolverSAP

SIMEXP = REPO_ROOT
DEVICE = "cuda:0"
sc = None


def rung(ke, tau):
    return {"id": f"ke={ke:.5g},tau={tau * 1e3:g}ms,gap=.1mm", "ke": float(ke), "tau": float(tau), "gap": 1e-4}


NATIVE = {"id": "native_default", "ke": None, "tau": None, "gap": None}


def runnable(c, dt):
    return True, ""


def simulate(c, cell, init_xy, dump_contacts=False):
    R = len(init_xy); ncon = 512 * R; dt = cell["dt_ms"] * 1e-3
    mat = {"density": 0.0, "mu": cell["mu"]}
    if c["ke"] is not None:
        mat.update(ke=c["ke"], tau=c["tau"])
    if c.get("gap") is not None:
        mat["gap"] = c["gap"]
    I = sc.peg_inertia(cell); pos0, q0 = sc.initial_pose(cell)
    peg_xf = {"p": [float(v) for v in pos0], "q": [float(q0[1]), float(q0[2]), float(q0[3]), float(q0[0])]}     # wxyz -> xyzw
    world = [{"type": "convex_mesh", "path": str(SIMEXP / sc.ASSET_DIR / f), "is_solid": True} for f in sc.FILES]
    scene = {"schema_version": 1, "name": "round_socket_v8", "simulation": {"dt": dt, "num_worlds": R, "nconmax": ncon},
             "builder": {"gravity": -sc.gravity(cell), "rigid_gap": 0.0, "defaults": {"shape": mat}}, "ground": {"enabled": False},
             "world_shapes": world,
             "bodies": [{"id": "peg", "transform": peg_xf, "mass": sc.peg_mass(cell), "inertia": [I[0], 0, 0, 0, I[1], 0, 0, 0, I[2]], "lock_inertia": True,
                         "shapes": [{"type": "cylinder", "radius": sc.peg_radius_mm(cell) * 1e-3, "half_height": sc.PEG_HALF * 1e-3}]}],
             "joints": [{"id": "peg_free", "type": "free", "parent": "world", "child": "peg"}],
             "articulations": [{"id": "a", "joints": ["peg_free"]}]}
    path = Path(tempfile.mkdtemp()) / "socket.yaml"; path.write_text(yaml.safe_dump(scene, sort_keys=False))
    l = load_sap_scene(path, device=wp.get_device(DEVICE), naconmax=ncon, strict=True, num_worlds=R)
    m, s, ctrl = l.sap_model, l.sap_state, l.sap_control; o = m.state()
    pipe = SapCollisionPipeline(l.collision_model, naconmax=ncon, bounds_filter="sphere_obb"); cts = pipe.contacts()
    sol = SolverSAP(m, nconmax=ncon, enable_sleep=False)
    q = s.joint_q.numpy().reshape(R, 7); q[:, 0:2] += init_xy; s.joint_q.assign(q.reshape(-1)); sap_eval_fk(m, s.joint_q, s.joint_qd, s)
    n = round(float(cell["duration_s"]) / dt); S = n + 1
    Q = np.zeros((S, R, 7)); D = np.full((S, R), np.nan); trunc = 0; contacts = None
    Q[0] = s.joint_q.numpy().reshape(R, 7)
    for k in range(n):
        s.clear_forces(); sol.collide(pipe, cts, s, ctrl, collision_state=s); sol.step(s, o, ctrl, cts, dt); s, o = o, s
        Q[k + 1] = s.joint_q.numpy().reshape(R, 7)
        trunc = max(trunc, int(sol.last_truncated_contact_count or 0))
        jac = sol.last_contact_jacobian_result
        if jac is not None:
            cnt = jac.contact_env_count.numpy(); phi = jac.contact_env_phi0.numpy()
            if phi.shape[1]:
                mask = np.arange(phi.shape[1])[None, :] < cnt[:, None]
                D[k] = np.where(mask, np.maximum(0.0, -phi), 0.0).max(axis=1)
            else:
                D[k] = 0.0
            if dump_contacts and k == 1:
                contacts = _dump(sol, jac, cnt, R, dt)
        else:
            D[k] = 0.0
    try:
        pipe.check_buffers(cts)
    except Exception:
        trunc = max(trunc, 1)
    Q = Q.transpose(1, 0, 2)
    return {"pos": Q[:, :, 0:3], "quat": Q[:, :, [6, 3, 4, 5]], "engine_depth": D.T, "truncated": trunc, "contacts": contacts}     # xyzw -> wxyz


def _dump(sol, jac, cnt, R, dt):
    res = sol.last_contact_solve_result; pt = jac.contact_env_point.numpy(); Rwc = jac.contact_env_R_WC.numpy(); phi = jac.contact_env_phi0.numpy()
    g = res.contact_gamma.numpy() if res is not None else None
    if g is not None:
        g = g.reshape(R, -1, 3)
    out = []
    for w in range(R):
        nw = int(cnt[w]); rows = np.zeros((nw, 8))
        if nw:
            rows[:, 0:3] = pt[w, :nw]; rows[:, 3:6] = Rwc[w, :nw, :, 2]; rows[:, 6] = np.maximum(0.0, -phi[w, :nw])
            rows[:, 7] = g[w, :nw, 2] / dt if g is not None else 0.0
        out.append(rows)
    return out


LADDER = {"lo": lambda dt: 1e3, "hi": 1e9, "increasing": False, "log": True, "make": lambda p, tau: rung(p, tau),
          "extras": [t * 1e-3 for t in (1.5, 3.0, 6.0, 12.0, 24.0)], "damping_selects_extra": True, "native": NATIVE, "attribution": None}


def main():
    global sc
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--preflight-json", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--phases", default="calibration,selection,frozen,robustness")
    ap.add_argument("--smoke", action="store_true", help="selection conditions only, native default only (code check)")
    a = ap.parse_args()
    pf = json.loads(a.preflight_json.read_text()); dev = pf.get("devices", {}).get(DEVICE)
    if not pf.get("all_pass") or not dev or not all(v["pass"] for v in dev.values()) or (pf.get("cpu_gpu") or {}).get("pass") is not True:
        raise SystemExit("SAP preflight must pass on cuda:0 with CPU/GPU agreement")
    sys.path.insert(0, str(a.simexp_scripts)); import socket_v8_common as sc_mod
    sc = sc_mod
    wp.init(); man, sha = sc.load_manifest(a.manifest)
    import sap_warp
    engine = {"engine": "SAP Warp internal", "sap_source": str(Path(sap_warp.__file__).parent), "warp_version": wp.config.version, "device": DEVICE,
              "manifest_sha256": sha, "manifest_revision": man["revision"], "preflight": a.preflight_json.name}
    log = lambda s: print(s, flush=True)
    if a.smoke:
        a.out.mkdir(parents=True, exist_ok=True); idx = {c["id"]: j for j, c in enumerate(man["conditions"])}
        for cid in man["run_plan"]["1_selection"]["conditions"]:
            r = sc.run_condition(NATIVE, man["conditions"][idx[cid]], man["seed0"] + idx[cid], man, simulate, a.out / "traj")
            log(sc.fmt(r)); (a.out / f"smoke_{cid}.json").write_text(json.dumps(sc.json_safe(r), indent=1))
        return
    res = sc.execute(man, engine, simulate, runnable, LADDER, a.out, log=log, phases=tuple(a.phases.split(",")))
    res.update(protocol=man["protocol"], task=man["task"], revision=man["revision"])
    (a.out / "result.json").write_text(json.dumps(sc.json_safe(res), indent=1) + "\n")


if __name__ == "__main__":
    main()
