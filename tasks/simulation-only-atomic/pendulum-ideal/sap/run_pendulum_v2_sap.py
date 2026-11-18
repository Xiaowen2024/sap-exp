"""Private SAP Warp pendulum energy-conservation-v2 driver (public logic in SIM-EXP pendulum_v2_common).

Revolute joint about world y at the origin; bob body at (0,0,-L) with explicit mass and solid-sphere inertia
(density 0); no contact (ground disabled).  Candidates: solver precision only.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import argparse, json, sys, tempfile
from pathlib import Path
import numpy as np, warp as wp, yaml
from sap_warp.loader import load_sap_scene
from sap_warp.collision import SapCollisionPipeline
from sap_warp.sap_kinematics import sap_eval_fk
from sap_warp.solver_sap import SolverSAP
DEVICE = "cuda:0"


def parse_candidate(t):
    return {"id": "native_default", "dtype": "float32"} if t.strip() == "native default" else {"id": t, "dtype": t.split("=")[1]}


def make_simulate(pc):
    def simulate(c, cell, th0):
        m, L, dt, R = cell["mass_kg"], cell["length_m"], cell["dt_ms"] * 1e-3, len(th0)
        I = pc.i_com(m)
        sc = {"schema_version": 1, "name": "pendulum_v2", "simulation": {"dt": dt, "num_worlds": R, "nconmax": 8},
              "builder": {"gravity": -9.81, "defaults": {"shape": {"density": 0.0}}}, "ground": {"enabled": False},
              "bodies": [{"id": "bob", "transform": {"p": [0, 0, -L], "q": "identity"}, "mass": m, "inertia": [I, 0, 0, 0, I, 0, 0, 0, I], "lock_inertia": True,
                          "shapes": [{"type": "sphere", "radius": pc.R_BOB}]}],
              "joints": [{"id": "hinge", "type": "revolute", "parent": "world", "child": "bob", "axis": [0, 1, 0],
                          "parent_xform": "identity", "child_xform": {"p": [0, 0, L], "q": "identity"}}],
              "articulations": [{"id": "a", "joints": ["hinge"]}]}
        path = Path(tempfile.mkdtemp()) / "pendulum.yaml"; path.write_text(yaml.safe_dump(sc, sort_keys=False))
        l = load_sap_scene(path, device=wp.get_device(DEVICE), naconmax=8 * R, strict=True, num_worlds=R)
        mdl, s, ctrl = l.sap_model, l.sap_state, l.sap_control; o = mdl.state()
        pipe = SapCollisionPipeline(l.collision_model, naconmax=8 * R, bounds_filter="sphere_obb"); cts = pipe.contacts()
        sol = SolverSAP(mdl, nconmax=8 * R, enable_sleep=False, dtype=(wp.float64 if c["dtype"] == "float64" else wp.float32))
        q = s.joint_q.numpy().reshape(R, 1); q[:, 0] = th0; s.joint_q.assign(q.reshape(-1)); sap_eval_fk(mdl, s.joint_q, s.joint_qd, s)
        every = int(round(pc.SAMPLE_S / dt)); S = int(round(pc.DURATION_S / pc.SAMPLE_S)) + 1; Q, V = [], []
        for k in range(S):
            Q.append(s.joint_q.numpy().astype(float).copy()); V.append(s.joint_qd.numpy().astype(float).copy())
            if k < S - 1:
                for _ in range(every):
                    s.clear_forces(); sol.collide(pipe, cts, s, ctrl, collision_state=s); sol.step(s, o, ctrl, cts, dt); s, o = o, s
        return np.asarray(Q), np.asarray(V)
    return simulate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--preflight-json", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    pf = json.loads(a.preflight_json.read_text()); dev = pf.get("devices", {}).get(DEVICE)
    if not pf.get("all_pass") or not dev or not all(v["pass"] for v in dev.values()) or (pf.get("cpu_gpu") or {}).get("pass") is not True:
        raise SystemExit("SAP preflight must pass on cuda:0 with CPU/GPU agreement")
    sys.path.insert(0, str(a.simexp_scripts)); import pendulum_v2_common as pc
    wp.init(); man, sha = pc.load_manifest(a.manifest)
    res = pc.execute(man, "sap_candidates", parse_candidate, make_simulate(pc), log=lambda s: print(s, flush=True))
    import sap_warp
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha, "engine": "SAP Warp internal",
           "sap_source": str(Path(sap_warp.__file__).parent), "warp_version": wp.config.version, "device": DEVICE, "preflight": pf, **res}
    a.out.mkdir(parents=True, exist_ok=True); (a.out / "sap_pendulum_v2.json").write_text(json.dumps(pc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
