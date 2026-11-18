"""Private SAP Warp MIT Push dynamic-friction-v3 driver (public logic in SIM-EXP mitpush_v3_common).

Object: released convex mesh (axis_order [0,2,1] maps the thickness axis onto z), explicit mass/inertia, free joint.
Pusher: cylinder on prismatic x / prismatic y joints with stiff implicit PD tracking the recorded tip path (targets and
target velocities).  Per-shape mu chosen so SAP's harmonic pair combination gives table-object 0.22 and
pusher-object 0.25.  Force on the pusher = R_WC gamma / dt, signed to point from the object towards the pusher.
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
MU_OBJ, MU_GROUND, MU_TIP = 1.0, 0.22 / (2 - 0.22), 0.25 / (2 - 0.25)
KE, KD = 1e6, 2e3
from run_slope_v3_sap import parse_candidate  # noqa: E402


def make_run_trial(mc):
    def scene(c, trial, cell, dt, R, ncon):
        mat = {"density": 0.0, "mu": MU_OBJ}
        if c["ke"] is not None:
            mat.update(ke=c["ke"], tau=c["tau"])
        ms = float(cell.get("mass_scale", 1.0)); I = [x * ms for x in mc.INERTIA[trial["shape"]]]
        x0, y0 = [float(v) for v in trial["tip"][0]]
        pd = {"target_ke": KE, "target_kd": KD, "actuator_mode": 1}
        mesh = SIMEXP / "exp-results/mit-push-v2/assets" / f"{trial['shape']}.obj"
        return {"schema_version": 1, "name": "mit_push_v3", "simulation": {"dt": dt, "num_worlds": R, "nconmax": ncon},
                "builder": {"gravity": -9.81, "rigid_gap": 0.0, "defaults": {"shape": mat}},
                "ground": {"enabled": True, "cfg": {"mu": MU_GROUND}},
                "bodies": [{"id": "object", "transform": {"p": [0, 0, mc.HALF_Z], "q": "identity"}, "mass": mc.MASS * ms,
                            "inertia": [I[0], 0, 0, 0, I[1], 0, 0, 0, I[2]], "lock_inertia": True,
                            "shapes": [{"type": "convex_mesh", "path": str(mesh), "axis_order": [0, 2, 1], "face_order": [0, 2, 1]}]},
                           {"id": "sx", "transform": {"p": [x0, y0, mc.TIP_Z], "q": "identity"}, "mass": 0.01,
                            "inertia": [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6], "lock_inertia": True, "shapes": []},
                           {"id": "pusher", "transform": {"p": [x0, y0, mc.TIP_Z], "q": "identity"}, "mass": 0.05,
                            "inertia": [1e-5, 0, 0, 0, 1e-5, 0, 0, 0, 1e-5], "lock_inertia": True,
                            "shapes": [{"type": "cylinder", "radius": mc.TIP_R, "half_height": mc.TIP_HH, "cfg": {"mu": MU_TIP}}]}],
                "joints": [{"id": "object_free", "type": "free", "parent": "world", "child": "object"},
                           {"id": "px", "type": "prismatic", "parent": "world", "child": "sx", "axis": [1, 0, 0],
                            "parent_xform": {"p": [x0, y0, mc.TIP_Z], "q": "identity"}, "child_xform": "identity", **pd},
                           {"id": "py", "type": "prismatic", "parent": "sx", "child": "pusher", "axis": [0, 1, 0], **pd}],
                "articulations": [{"id": "obj", "joints": ["object_free"]}, {"id": "push", "joints": ["px", "py"]}]}

    def run_trial(c, trial, cell, init_xy, sps, dt, S, fps_):
        R = len(init_xy); ncon = 64 * R
        path = Path(tempfile.mkdtemp()) / "mitpush.yaml"; path.write_text(yaml.safe_dump(scene(c, trial, cell, dt, R, ncon), sort_keys=False))
        l = load_sap_scene(path, device=wp.get_device(DEVICE), naconmax=ncon, strict=True, num_worlds=R)
        m, s, ctrl = l.sap_model, l.sap_state, l.sap_control; o = m.state()
        pipe = SapCollisionPipeline(l.collision_model, naconmax=ncon, bounds_filter="sphere_obb"); cts = pipe.contacts()
        kw = {} if c.get("sigma") is None else {"contact_sigma": c["sigma"]}
        sol = SolverSAP(m, nconmax=ncon, enable_sleep=False, **kw)
        q = s.joint_q.numpy().reshape(R, 9); q[:, 0:2] += init_xy; s.joint_q.assign(q.reshape(-1)); sap_eval_fk(m, s.joint_q, s.joint_qd, s)
        x0, y0 = trial["tip"][0]
        P = np.zeros((R, S, 3)); TW = np.zeros((R, S, 3)); W = np.zeros((R, S, 2)); D = np.zeros((R, S)); trunc = 0; track = 0.0
        step = 0
        for k in range(S):
            qq = s.joint_q.numpy().reshape(R, 9); vv = s.joint_qd.numpy().reshape(R, 8)
            x, y, z, w = (qq[:, 3 + i] for i in range(4))
            P[:, k, 0:2] = qq[:, 0:2]; P[:, k, 2] = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            TW[:, k, 0:2] = vv[:, 0:2]; TW[:, k, 2] = vv[:, 5]
            jac, so = sol.last_contact_jacobian_result, sol.last_contact_solve_result
            if jac is not None and so is not None and k > 0:
                n = jac.contact_env_count.numpy(); g = so.contact_gamma.numpy(); Rw = jac.contact_env_R_WC.numpy()
                b0, b1 = jac.contact_env_body0.numpy(), jac.contact_env_body1.numpy()
                for wi in range(R):
                    for j in range(int(n[wi])):
                        pair = {int(b0[wi, j]) - 3 * wi, int(b1[wi, j]) - 3 * wi}
                        if pair == {0, 2}:
                            F = (np.asarray(Rw[wi, j]) @ np.asarray(g[wi, j]))[:2] / dt
                            rel = np.array([qq[wi, 7] + x0 - qq[wi, 0], qq[wi, 8] + y0 - qq[wi, 1]])
                            W[wi, k] += F if np.dot(F, rel) >= 0 else -F; D[wi, k] = 1
            if k == S - 1:
                break
            for _ in range(sps):
                xt, yt, vx, vy = mc.tip_target(trial, (step + 1) * dt)
                tp = np.zeros((R, 8), np.float32); tv = np.zeros((R, 8), np.float32)
                tp[:, 6], tp[:, 7] = xt - x0, yt - y0; tv[:, 6], tv[:, 7] = vx, vy
                ctrl.joint_target_pos.assign(tp.reshape(-1)); ctrl.joint_target_vel.assign(tv.reshape(-1))
                s.clear_forces(); sol.collide(pipe, cts, s, ctrl, collision_state=s); sol.step(s, o, ctrl, cts, dt); s, o = o, s
                trunc = max(trunc, int(sol.last_truncated_contact_count or 0)); step += 1
                qq2 = s.joint_q.numpy().reshape(R, 9)
                track = max(track, float(np.abs(qq2[:, 7] - (xt - x0)).max()), float(np.abs(qq2[:, 8] - (yt - y0)).max()))
        try:
            pipe.check_buffers(cts)
        except Exception:
            trunc = max(trunc, 1)
        P[:, :, 2] = np.unwrap(P[:, :, 2], axis=1)
        return {"pose": P, "twist": TW, "wrench": W, "duty": D, "finite": np.isfinite(P).all(axis=(1, 2)), "truncated": trunc,
                "tracking_ok": track <= 5e-4, "tracking": {"max_err_mm": track * 1e3}}
    return run_trial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--preflight-json", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    pf = json.loads(a.preflight_json.read_text()); dev = pf.get("devices", {}).get(DEVICE)
    if not pf.get("all_pass") or not dev or not all(v["pass"] for v in dev.values()) or (pf.get("cpu_gpu") or {}).get("pass") is not True:
        raise SystemExit("SAP preflight must pass on cuda:0 with CPU/GPU agreement")
    sys.path.insert(0, str(a.simexp_scripts)); import mitpush_v3_common as mc
    wp.init(); man, sha = mc.load_manifest(a.manifest)
    res = mc.execute(man, "sap_candidates", parse_candidate, make_run_trial(mc), SIMEXP, log=lambda s: print(s, flush=True))
    import sap_warp
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha, "engine": "SAP Warp internal",
           "sap_source": str(Path(sap_warp.__file__).parent), "warp_version": wp.config.version, "device": DEVICE, "preflight": pf, **res}
    a.out.mkdir(parents=True, exist_ok=True); (a.out / "sap_mit_push_v3.json").write_text(json.dumps(mc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
