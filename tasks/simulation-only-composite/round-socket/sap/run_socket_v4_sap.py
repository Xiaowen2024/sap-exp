"""Private SAP Warp round-socket strict-v4 driver (exact-bore socket; public logic in SIM-EXP socket_v4_common).

Socket = 33 static world_shapes (convex meshes from assets/round_socket_exact); no ground.  Peg = free body with a
cylinder shape, explicit mass and solid-cylinder inertia (density 0), released tilted about y.  Peak / final
penetration = max(0, -min contact_env_phi0) over the peg's contacts (every step).
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import math
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


def parse_candidate(t):
    if t.strip() == "native default":
        return {"id": "native_default", "ke": None, "tau": None, "gap": None}
    kv = dict(p.split("=") for p in t.replace(" ", "").split(","))
    tau, gap = kv["tau"], kv["gap"]
    return {"id": t, "ke": float(kv["ke"]), "tau": float(tau[:-2]) * 1e-3 if tau.endswith("ms") else float(tau),
            "gap": float(gap[:-2]) * 1e-3 if gap.endswith("mm") else float(gap)}


def make_simulate(sc):
    def simulate(c, cell, init_xy):
        R = len(init_xy); ncon = 512 * R; dt = cell["dt_ms"] * 1e-3
        mat = {"density": 0.0, "mu": cell["mu"]}
        if c["ke"] is not None:
            mat.update(ke=c["ke"], tau=c["tau"])
        if c.get("gap") is not None:
            mat["gap"] = c["gap"]
        I = sc.peg_inertia(cell)
        if "pose" in cell:                                              # v6: analytic tilt-plane pose (wxyz -> xyzw)
            pos0, q0 = sc.initial_pose(cell); peg_xf = {"p": [float(v) for v in pos0], "q": [float(q0[1]), float(q0[2]), float(q0[3]), float(q0[0])]}
        else:
            t = math.radians(cell["tilt_deg"])
            peg_xf = {"p": [cell["offset_mm"] / 1000, 0, cell["release_height_mm"] / 1000], "q": [0.0, math.sin(t / 2), 0.0, math.cos(t / 2)]}
        world = [{"type": "convex_mesh", "path": str(SIMEXP / sc.ASSET_DIR / f), "is_solid": True} for f in sc.FILES]
        scene = {"schema_version": 1, "name": "round_socket_v4", "simulation": {"dt": dt, "num_worlds": R, "nconmax": ncon},
                 "builder": {"gravity": -9.81, "rigid_gap": 0.0, "defaults": {"shape": mat}}, "ground": {"enabled": False},
                 "world_shapes": world,
                 "bodies": [{"id": "peg", "transform": peg_xf,
                             "mass": sc.PEG_MASS, "inertia": [I[0], 0, 0, 0, I[1], 0, 0, 0, I[2]], "lock_inertia": True,
                             "shapes": [{"type": "cylinder", "radius": sc.peg_radius(cell), "half_height": sc.PEG_HALF}]}],
                 "joints": [{"id": "peg_free", "type": "free", "parent": "world", "child": "peg"}],
                 "articulations": [{"id": "a", "joints": ["peg_free"]}]}
        path = Path(tempfile.mkdtemp()) / "socket.yaml"; path.write_text(yaml.safe_dump(scene, sort_keys=False))
        l = load_sap_scene(path, device=wp.get_device(DEVICE), naconmax=ncon, strict=True, num_worlds=R)
        m, s, ctrl = l.sap_model, l.sap_state, l.sap_control; o = m.state()
        pipe = SapCollisionPipeline(l.collision_model, naconmax=ncon, bounds_filter="sphere_obb"); cts = pipe.contacts()
        sol = SolverSAP(m, nconmax=ncon, enable_sleep=False)
        q = s.joint_q.numpy().reshape(R, 7); q[:, 0:2] += init_xy; s.joint_q.assign(q.reshape(-1)); sap_eval_fk(m, s.joint_q, s.joint_qd, s)
        peak = np.zeros(R); trunc = 0; last = np.zeros(R); pv = np.zeros(R); pl = np.zeros(R)
        dur = float(cell.get("duration_s", 2.0)); every = max(1, int(round(0.01 / dt))); TP, TQ = [], []
        for k in range(round(dur / dt)):
            if k % every == 0:
                qq0 = s.joint_q.numpy().reshape(R, 7).astype(float); TP.append(qq0[:, 0:3].copy()); TQ.append(qq0[:, [6, 3, 4, 5]].copy())  # xyzw -> wxyz
            s.clear_forces(); sol.collide(pipe, cts, s, ctrl, collision_state=s); sol.step(s, o, ctrl, cts, dt); s, o = o, s
            trunc = max(trunc, int(sol.last_truncated_contact_count or 0))
            jac = sol.last_contact_jacobian_result
            if jac is not None:
                n = jac.contact_env_count.numpy(); phi = jac.contact_env_phi0.numpy()
                mask = np.arange(phi.shape[1])[None, :] < n[:, None]
                p_all = np.where(mask, np.maximum(0.0, -phi), 0.0)
                pen = p_all.max(axis=1) if phi.shape[1] else np.zeros(R)
                peak = np.maximum(peak, pen); last = pen
                if phi.shape[1]:
                    vert = np.abs(jac.contact_env_R_WC.numpy()[:, :, 2, 2]) > 0.5      # normal = R_WC column 2
                    pv = np.maximum(pv, np.where(vert, p_all, 0.0).max(axis=1)); pl = np.maximum(pl, np.where(vert, 0.0, p_all).max(axis=1))
        try:
            pipe.check_buffers(cts)
        except Exception:
            trunc = max(trunc, 1)
        qq = s.joint_q.numpy().reshape(R, 7).astype(float); TP.append(qq[:, 0:3].copy()); TQ.append(qq[:, [6, 3, 4, 5]].copy())
        return {"traj_pos": np.stack(TP, axis=1), "traj_quat": np.stack(TQ, axis=1), "final_xyz": qq[:, 0:3], "peak_pen": peak, "final_pen": last, "peak_pen_vertical": pv, "peak_pen_lateral": pl, "finite": np.isfinite(qq).all(axis=1), "truncated": trunc}
    return simulate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--preflight-json", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    pf = json.loads(a.preflight_json.read_text()); dev = pf.get("devices", {}).get(DEVICE)
    if not pf.get("all_pass") or not dev or not all(v["pass"] for v in dev.values()) or (pf.get("cpu_gpu") or {}).get("pass") is not True:
        raise SystemExit("SAP preflight must pass on cuda:0 with CPU/GPU agreement")
    sys.path.insert(0, str(a.simexp_scripts)); import socket_v4_common as sc
    wp.init(); man, sha = sc.load_manifest(a.manifest)
    v6 = man["revision"].startswith("strict-v6")
    if v6:
        import socket_v6_common as sc
    res = sc.execute(man, "sap_candidates", parse_candidate, make_simulate(sc), log=lambda s: print(s, flush=True))
    import sap_warp
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha, "engine": "SAP Warp internal",
           "sap_source": str(Path(sap_warp.__file__).parent), "warp_version": wp.config.version, "device": DEVICE, "preflight": pf, **res}
    a.out.mkdir(parents=True, exist_ok=True); (a.out / ("sap_round_socket_v6.json" if v6 else "sap_round_socket_v4.json")).write_text(json.dumps(sc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
