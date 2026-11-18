"""Private SAP Warp cube-toss resting tail (sustained-contact-v2) driver; public logic in SIM-EXP tail_v2_common."""
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
CHUNK = 160


def make_simulate(tc, ic, parse):
    def scene(c, dt, mass_scale, mu, scale, W, ncon):
        mat = {"density": 0.0, "mu": mu}
        if c["ke"] is not None:
            mat.update(ke=c["ke"], tau=c["tau"])
        h = tc.HALF * scale; I = 0.2950002913583124 * h ** 2 * mass_scale
        return {"schema_version": 1, "name": "cube_tail_v2", "simulation": {"dt": dt, "num_worlds": W, "nconmax": ncon},
                "builder": {"gravity": -9.81, "defaults": {"shape": mat}}, "ground": {"enabled": True, "cfg": {"mu": mu}},
                "bodies": [{"id": "cube", "transform": {"p": [0, 0, 0], "q": "identity"}, "mass": tc.MASS * mass_scale,
                            "inertia": [I, 0, 0, 0, I, 0, 0, 0, I], "lock_inertia": True,
                            "shapes": [{"type": "box", "hx": h, "hy": h, "hz": h}]}],
                "joints": [{"id": "cube_free", "type": "free", "parent": "world", "child": "cube"}],
                "articulations": [{"id": "cube_art", "joints": ["cube_free"]}]}

    def simulate(c, *, dt, mass_scale, mu, scale, init, n_samples, steps_per_sample):
        if len(init) > CHUNK:
            parts = [simulate(c, dt=dt, mass_scale=mass_scale, mu=mu, scale=scale, init=init[i:i + CHUNK], n_samples=n_samples,
                              steps_per_sample=steps_per_sample) for i in range(0, len(init), CHUNK)]
            out = {k: np.concatenate([p[k] for p in parts]) for k in ("pos", "quat", "speed", "omega", "fn", "ncon", "finite")}
            out["truncated"] = max(p["truncated"] for p in parts); return out
        W = len(init); ncon = 16 * W
        path = Path(tempfile.mkdtemp()) / "tail.yaml"
        path.write_text(yaml.safe_dump(scene(c, dt, mass_scale, mu, scale, W, ncon), sort_keys=False))
        l = load_sap_scene(path, device=wp.get_device(DEVICE), naconmax=ncon, strict=True, num_worlds=W)
        m, s, ctrl = l.sap_model, l.sap_state, l.sap_control; o = m.state()
        pipe = SapCollisionPipeline(l.collision_model, naconmax=ncon, bounds_filter="sphere_obb"); cts = pipe.contacts()
        sol = SolverSAP(m, nconmax=ncon, enable_sleep=False)
        q = s.joint_q.numpy().reshape(W, 7); qd = s.joint_qd.numpy().reshape(W, 6)
        q[:, 0:3] = init[:, 0:3]; q[:, 3:7] = init[:, [4, 5, 6, 3]]
        qd[:, 0:3] = init[:, 7:10]; qd[:, 3:6] = np.einsum("wij,wj->wi", ic.quat_to_mat(init[:, 3:7]), init[:, 10:13])
        s.joint_q.assign(q.reshape(-1)); s.joint_qd.assign(qd.reshape(-1)); sap_eval_fk(m, s.joint_q, s.joint_qd, s)
        P = np.zeros((W, n_samples, 3)); Q = np.zeros((W, n_samples, 4)); SP = np.zeros((W, n_samples)); OM = np.zeros((W, n_samples))
        FN = np.zeros((W, n_samples)); NC = np.zeros((W, n_samples)); trunc = 0
        for k in range(n_samples):
            qq = s.joint_q.numpy().reshape(W, 7); vv = s.joint_qd.numpy().reshape(W, 6)
            P[:, k] = qq[:, 0:3]; Q[:, k] = qq[:, [6, 3, 4, 5]]
            SP[:, k] = np.linalg.norm(vv[:, 0:3], axis=1); OM[:, k] = np.linalg.norm(vv[:, 3:6], axis=1)
            jac, so = sol.last_contact_jacobian_result, sol.last_contact_solve_result
            if jac is not None and so is not None:
                n = jac.contact_env_count.numpy(); g = so.contact_gamma.numpy()
                mask = np.arange(g.shape[1])[None, :] < n[:, None]
                NC[:, k] = n; FN[:, k] = (np.maximum(g[:, :, 2], 0) * mask).sum(axis=1) / dt
            if k < n_samples - 1:
                for _ in range(steps_per_sample):
                    s.clear_forces(); sol.collide(pipe, cts, s, ctrl, collision_state=s); sol.step(s, o, ctrl, cts, dt); s, o = o, s
                    trunc = max(trunc, int(sol.last_truncated_contact_count or 0))
        try:
            pipe.check_buffers(cts)
        except Exception:
            trunc = max(trunc, 1)
        finite = np.isfinite(P).all(axis=(1, 2)) & np.isfinite(Q).all(axis=(1, 2))
        Q = Q / np.maximum(np.linalg.norm(Q, axis=2, keepdims=True), 1e-12)
        return {"pos": P, "quat": Q, "speed": SP, "omega": OM, "fn": FN, "ncon": NC, "finite": finite, "truncated": trunc}
    return simulate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--preflight-json", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--cube-npz", type=Path, default=SIMEXP / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si_v2.npz")
    a = ap.parse_args()
    pf = json.loads(a.preflight_json.read_text()); dev = pf.get("devices", {}).get(DEVICE)
    if not pf.get("all_pass") or not dev or not all(v["pass"] for v in dev.values()) or (pf.get("cpu_gpu") or {}).get("pass") is not True:
        raise SystemExit(f"SAP preflight must pass on {DEVICE} with CPU/GPU agreement: {a.preflight_json}")
    sys.path.insert(0, str(a.simexp_scripts)); sys.path.insert(0, str(Path(__file__).resolve().parent))
    import tail_v2_common as tc, impact_v2_common as ic
    from run_impact_v2_sap import parse_candidate
    wp.init()
    man, sha = tc.load_manifest(a.manifest)
    res = tc.execute(man, "sap_candidates", parse_candidate, make_simulate(tc, ic, parse_candidate), ic.cube_data(a.cube_npz),
                     log=lambda s: print(s, flush=True))
    import sap_warp
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
           "engine": "SAP Warp internal", "sap_source": str(Path(sap_warp.__file__).parent), "warp_version": wp.config.version,
           "device": DEVICE, "preflight": pf, **res}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "sap_cube_tail_v2.json").write_text(json.dumps(tc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
