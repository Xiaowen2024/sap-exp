"""Private SAP Warp GAUGE slope-slider dynamic-friction-v3 driver (public logic in SIM-EXP slope_v3_common).

task-1/2: board on a fixed joint at its recorded pose.  task-3: board driven by a prismatic-x / prismatic-z /
revolute-about-world-y chain whose stiff implicit PD tracks the mean recorded board trajectory (targets and target
velocities); tracking error is recorded and gated.  Cube and board get explicit mass and box inertia (density 0).
Quaternions in SAP YAML/joint_q are xyzw.
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
ASSETS = SIMEXP / "exp-results/gauge-rigid-reproduction/assets"
DEVICE = "cuda:0"
KE_LIN, KD_LIN, KE_ROT, KD_ROT = 1e7, 2e5, 1e5, 2e3


def parse_candidate(text):
    if text.strip() == "native default":
        return {"id": "native_default", "ke": None, "tau": None, "sigma": None}
    kv = dict(p.split("=") for p in text.replace(" ", "").split(","))
    tau = kv["tau"]
    return {"id": text, "ke": float(kv["ke"]), "tau": float(tau[:-2]) * 1e-3 if tau.endswith("ms") else float(tau),
            "sigma": float(kv["sigma"]) if "sigma" in kv else None}


xyzw = lambda q: [float(q[1]), float(q[2]), float(q[3]), float(q[0])]
box_I = lambda m, a, b, c: [m * (b * b + c * c) / 12, 0, 0, 0, m * (a * a + c * c) / 12, 0, 0, 0, m * (a * a + b * b) / 12]


def scene(c, case, dt, R, ncon):
    mat = {"density": 0.0, "mu": case["mu"]}
    if c["ke"] is not None:
        mat.update(ke=c["ke"], tau=c["tau"])
    pf, qf = [float(x) for x in case["p_f"]], xyzw(case["q_f"])
    board = {"id": "board", "transform": {"p": pf, "q": qf}, "mass": case["mass_board_kg"],
             "inertia": box_I(case["mass_board_kg"], .5, .15, .01), "lock_inertia": True,
             "shapes": [{"type": "convex_mesh", "path": str(ASSETS / "board.obj"), "is_solid": True}]}
    cube = {"id": "cube", "transform": {"p": [float(x) for x in case["p_m"]], "q": xyzw(case["q_m"])}, "mass": case["mass_kg"],
            "inertia": box_I(case["mass_kg"], .05, .05, .05), "lock_inertia": True,
            "shapes": [{"type": "convex_mesh", "path": str(ASSETS / "cube.obj"), "is_solid": True}]}
    cubefree = {"id": "cubefree", "type": "free", "parent": "world", "child": "cube"}
    if not case["kinematic"]:
        bodies = [board, cube]
        joints = [{"id": "boardfix", "type": "fixed", "parent": "world", "child": "board", "parent_xform": {"p": pf, "q": qf}, "child_xform": "identity"}, cubefree]
        arts = [{"id": "a", "joints": ["boardfix"]}, {"id": "b", "joints": ["cubefree"]}]
    else:
        small = {"mass": 0.01, "inertia": [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6], "lock_inertia": True, "shapes": []}
        bodies = [{"id": "sx", "transform": {"p": pf, "q": "identity"}, **small}, {"id": "sz", "transform": {"p": pf, "q": "identity"}, **small}, board, cube]
        qfc = [-qf[0], -qf[1], -qf[2], qf[3]]
        joints = [{"id": "bx", "type": "prismatic", "parent": "world", "child": "sx", "axis": [1, 0, 0], "parent_xform": {"p": pf, "q": "identity"}, "child_xform": "identity",
                   "target_ke": KE_LIN, "target_kd": KD_LIN, "actuator_mode": 1},
                  {"id": "bz", "type": "prismatic", "parent": "sx", "child": "sz", "axis": [0, 0, 1], "target_ke": KE_LIN, "target_kd": KD_LIN, "actuator_mode": 1},
                  {"id": "by", "type": "revolute", "parent": "sz", "child": "board", "axis": [0, 1, 0], "child_xform": {"p": [0, 0, 0], "q": qfc},
                   "target_ke": KE_ROT, "target_kd": KD_ROT, "actuator_mode": 1}, cubefree]
        arts = [{"id": "a", "joints": ["bx", "bz", "by"]}, {"id": "b", "joints": ["cubefree"]}]
    return {"schema_version": 1, "name": "gauge_slope_v3", "simulation": {"dt": dt, "num_worlds": R, "nconmax": ncon},
            "builder": {"gravity": -9.81, "rigid_gap": 0.0, "defaults": {"shape": mat}},
            "ground": {"enabled": True, "height": 0.0, "cfg": {"mu": case["mu"]}}, "bodies": bodies, "joints": joints, "articulations": arts}


def make_run_case(sc):
    def run_case(c, case, init_pos, n_frames, spf, dt):
        R = len(init_pos); ncon = 64 * R
        path = Path(tempfile.mkdtemp()) / "slope.yaml"
        path.write_text(yaml.safe_dump(scene(c, case, dt, R, ncon), sort_keys=False))
        l = load_sap_scene(path, device=wp.get_device(DEVICE), naconmax=ncon, strict=True, num_worlds=R)
        m, s, ctrl = l.sap_model, l.sap_state, l.sap_control; o = m.state()
        pipe = SapCollisionPipeline(l.collision_model, naconmax=ncon, bounds_filter="sphere_obb"); cts = pipe.contacts()
        kw = {} if c.get("sigma") is None else {"contact_sigma": c["sigma"]}
        sol = SolverSAP(m, nconmax=ncon, enable_sleep=False, **kw)
        kin = case["kinematic"]; nq = 10 if kin else 7; ndof = 9 if kin else 6; qoff = 3 if kin else 0
        nb = 4 if kin else 2; cube_b = nb - 1
        q = s.joint_q.numpy().reshape(R, nq); q[:, qoff:qoff + 3] = init_pos; s.joint_q.assign(q.reshape(-1))
        sap_eval_fk(m, s.joint_q, s.joint_qd, s)
        pos = np.zeros((R, n_frames, 3)); trunc = 0; track = [0.0, 0.0]
        total = (n_frames - 1) * spf
        for step in range(total + 1):
            if step % spf == 0:
                pos[:, step // spf] = s.body_q.numpy().reshape(R, nb, 7)[:, cube_b, 0:3]
            if step == total:
                break
            if kin:
                b = sc.board_state(case, (step + 1) * dt)
                tp = np.zeros((R, ndof), np.float32); tv = np.zeros((R, ndof), np.float32)
                tp[:, 0], tp[:, 1], tp[:, 2] = b["dx"], b["dz"], b["theta"]; tv[:, 0], tv[:, 1], tv[:, 2] = b["vdx"], b["vdz"], b["wy"]
                ctrl.joint_target_pos.assign(tp.reshape(-1)); ctrl.joint_target_vel.assign(tv.reshape(-1))
            s.clear_forces(); sol.collide(pipe, cts, s, ctrl, collision_state=s); sol.step(s, o, ctrl, cts, dt); s, o = o, s
            trunc = max(trunc, int(sol.last_truncated_contact_count or 0))
            if kin:
                qq = s.joint_q.numpy().reshape(R, nq)
                track[0] = max(track[0], float(np.abs(qq[:, 0] - b["dx"]).max()), float(np.abs(qq[:, 1] - b["dz"]).max()))
                track[1] = max(track[1], float(np.degrees(np.abs(qq[:, 2] - b["theta"]).max())))
        try:
            pipe.check_buffers(cts)
        except Exception:
            trunc = max(trunc, 1)
        ok = (track[0] <= 1e-3 and track[1] <= 0.5) if kin else True
        return {"pos": pos, "finite": np.isfinite(pos).all(axis=(1, 2)), "truncated": trunc, "tracking_ok": ok,
                "tracking": {"max_lin_err_mm": track[0] * 1e3, "max_rot_err_deg": track[1]} if kin else None}
    return run_case


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--preflight-json", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    pf = json.loads(a.preflight_json.read_text()); dev = pf.get("devices", {}).get(DEVICE)
    if not pf.get("all_pass") or not dev or not all(v["pass"] for v in dev.values()) or (pf.get("cpu_gpu") or {}).get("pass") is not True:
        raise SystemExit(f"SAP preflight must pass on {DEVICE} with CPU/GPU agreement: {a.preflight_json}")
    sys.path.insert(0, str(a.simexp_scripts))
    import slope_v3_common as sc
    wp.init()
    man, sha = sc.load_manifest(a.manifest)
    res = sc.execute(man, "sap_candidates", parse_candidate, make_run_case(sc), SIMEXP, log=lambda s: print(s, flush=True))
    import sap_warp
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
           "engine": "SAP Warp internal", "sap_source": str(Path(sap_warp.__file__).parent), "warp_version": wp.config.version,
           "device": DEVICE, "preflight": pf, **res}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "sap_gauge_slope_v3.json").write_text(json.dumps(sc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
