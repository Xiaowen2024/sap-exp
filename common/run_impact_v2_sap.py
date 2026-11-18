"""Private SAP Warp dynamic-impact-v2 driver (GAUGE ball bounce / ContactNets cube toss).

Uses the public impact_v2_common (trials, starts, perturbations, transforms, metrics) so SAP and MuJoCo Warp
see identical inputs.  SAP specifics handled here: joint_q quaternion is xyzw; free-joint angular velocity is
world-frame, so omega_world = R(q) omega_body is written; outputs are converted back to wxyz.
The shape 'restitution' field is not set (SAP ignores it).
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
BALL = {"density": 1069.8749, "mu": 0.808698, "mesh": SIMEXP / "exp-results/gauge-rigid-reproduction/assets/ball-6.obj"}
CUBE = {"mass": 0.37, "mu": 0.18, "half": 0.0524, "inertia": 0.2950002913583124 * 0.0524 ** 2}
DEVICE = "cuda:0"


def parse_candidate(text: str) -> dict:
    if text.strip() == "native default":
        return {"id": "native_default", "ke": None, "tau": None}
    kv = dict(p.split("=") for p in text.replace(" ", "").split(","))
    tau = kv["tau"]
    return {"id": text, "ke": float(kv["ke"]), "tau": float(tau[:-2]) * 1e-3 if tau.endswith("ms") else float(tau)}


def scene(task: str, c: dict, dt: float, mass_scale: float, W: int, ncon: int) -> dict:
    if task == "ball":
        shape = {"density": BALL["density"] * mass_scale, "mu": BALL["mu"]}
        if c["ke"] is not None:
            shape.update(ke=c["ke"], tau=c["tau"])
        return {"schema_version": 1, "name": "gauge_ball_v2", "simulation": {"dt": dt, "num_worlds": W, "nconmax": ncon},
                "builder": {"gravity": -9.81, "rigid_gap": 0.0, "defaults": {"shape": shape}},
                "ground": {"enabled": True, "height": 0.013297, "cfg": {"mu": BALL["mu"]}},
                "bodies": [{"id": "ball", "transform": {"p": [0, 0, .5], "q": "identity"},
                            "shapes": [{"type": "convex_mesh", "path": str(BALL["mesh"]), "is_solid": True,
                                        "cfg": {"density": BALL["density"] * mass_scale, "mu": BALL["mu"]}}]}],
                "joints": [{"id": "ball_free", "type": "free", "parent": "world", "child": "ball"}],
                "articulations": [{"id": "ball_a", "joints": ["ball_free"]}]}
    mat = {"density": 0.0, "mu": CUBE["mu"]}
    if c["ke"] is not None:
        mat.update(ke=c["ke"], tau=c["tau"])
    I = CUBE["inertia"] * mass_scale; h = CUBE["half"]
    return {"schema_version": 1, "name": "contactnets_cube_v2", "simulation": {"dt": dt, "num_worlds": W, "nconmax": ncon},
            "builder": {"gravity": -9.81, "defaults": {"shape": mat}}, "ground": {"enabled": True},
            "bodies": [{"id": "cube", "transform": {"p": [0, 0, 0], "q": "identity"}, "mass": CUBE["mass"] * mass_scale,
                        "inertia": [I, 0, 0, 0, I, 0, 0, 0, I], "lock_inertia": True,
                        "shapes": [{"type": "box", "hx": h, "hy": h, "hz": h}]}],
            "joints": [{"id": "cube_free", "type": "free", "parent": "world", "child": "cube"}],
            "articulations": [{"id": "cube_art", "joints": ["cube_free"]}]}


CHUNK = 160  # worlds per SAP batch (640 cube worlds in one batch ran out of GPU memory); worlds are independent


def simulate(c, *, task, dt, mass_scale, init, n_samples, steps_per_sample, ic=None):
    if len(init) > CHUNK:
        parts = [simulate(c, task=task, dt=dt, mass_scale=mass_scale, init=init[i:i + CHUNK], n_samples=n_samples,
                          steps_per_sample=steps_per_sample, ic=ic) for i in range(0, len(init), CHUNK)]
        return {"pos": np.concatenate([p["pos"] for p in parts]), "quat": np.concatenate([p["quat"] for p in parts]),
                "finite": np.concatenate([p["finite"] for p in parts]), "truncated": max(p["truncated"] for p in parts)}
    W = len(init); ncon = 16 * W
    path = Path(tempfile.mkdtemp()) / "scene.yaml"
    path.write_text(yaml.safe_dump(scene(task, c, dt, mass_scale, W, ncon), sort_keys=False))
    l = load_sap_scene(path, device=wp.get_device(DEVICE), naconmax=ncon, strict=True, num_worlds=W)
    m, s, ctrl = l.sap_model, l.sap_state, l.sap_control
    o = m.state()
    pipe = SapCollisionPipeline(l.collision_model, naconmax=ncon, bounds_filter="sphere_obb")
    cts = pipe.contacts(); sol = SolverSAP(m, nconmax=ncon, enable_sleep=False)
    q = s.joint_q.numpy().reshape(W, 7); qd = s.joint_qd.numpy().reshape(W, 6)
    q[:, 0:3] = init[:, 0:3]; q[:, 3:7] = init[:, [4, 5, 6, 3]]                      # wxyz -> xyzw
    Rm = ic.quat_to_mat(init[:, 3:7])
    qd[:, 0:3] = init[:, 7:10]; qd[:, 3:6] = np.einsum("wij,wj->wi", Rm, init[:, 10:13])  # body -> world omega
    s.joint_q.assign(q.reshape(-1)); s.joint_qd.assign(qd.reshape(-1)); sap_eval_fk(m, s.joint_q, s.joint_qd, s)
    pos = np.zeros((W, n_samples, 3)); quat = np.zeros((W, n_samples, 4)); trunc = 0
    for k in range(n_samples):
        qq = s.joint_q.numpy().reshape(W, 7)
        pos[:, k] = qq[:, 0:3]; quat[:, k] = qq[:, [6, 3, 4, 5]]                       # xyzw -> wxyz
        if k < n_samples - 1:
            for _ in range(steps_per_sample):
                s.clear_forces(); sol.collide(pipe, cts, s, ctrl, collision_state=s); sol.step(s, o, ctrl, cts, dt); s, o = o, s
                trunc = max(trunc, int(sol.last_truncated_contact_count or 0))
    try:
        pipe.check_buffers(cts)
    except Exception:
        trunc = max(trunc, 1)
    finite = np.isfinite(pos).all(axis=(1, 2)) & np.isfinite(quat).all(axis=(1, 2))
    quat = quat / np.maximum(np.linalg.norm(quat, axis=2, keepdims=True), 1e-12)
    return {"pos": pos, "quat": quat, "finite": finite, "truncated": trunc}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", choices=("ball", "cube"), required=True)
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--preflight-json", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cube-npz", type=Path, default=SIMEXP / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si_v2.npz")
    a = ap.parse_args()
    pf = json.loads(a.preflight_json.read_text())
    dev = pf.get("devices", {}).get(DEVICE)
    if not pf.get("all_pass") or not dev or not all(v["pass"] for v in dev.values()) or (pf.get("cpu_gpu") or {}).get("pass") is not True:
        raise SystemExit(f"SAP preflight must pass on {DEVICE} with CPU/GPU agreement: {a.preflight_json}")
    sys.path.insert(0, str(a.simexp_scripts))
    import impact_v2_common as ic
    wp.init()
    man, sha = ic.load_manifest(a.manifest)
    sym = None
    if a.task == "ball":
        data = ic.ball_trials(SIMEXP)
    else:
        data = ic.cube_data(a.cube_npz)
        from cube_orientation_metrics import cube_symmetric_qangle as sym
    sim = lambda c, **kw: simulate(c, ic=ic, **kw)
    res = ic.execute(a.task, man, "sap_candidates", parse_candidate, sim, data, sym, log=lambda s: print(s, flush=True))
    import sap_warp
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
           "engine": "SAP Warp internal", "sap_source": str(Path(sap_warp.__file__).parent), "warp_version": wp.config.version,
           "device": DEVICE, "preflight": pf, **res}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / f"sap_{a.task}_dynamic_impact_v2.json").write_text(json.dumps(ic.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
