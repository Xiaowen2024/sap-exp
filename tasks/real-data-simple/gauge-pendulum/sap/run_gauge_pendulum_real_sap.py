"""Private SAP Warp driver for the GAUGE pendulum real-reference revision (gauge-v3.1, pendulum_gauge_real_v1.json).

Scoring is Codex's public v3.1 code unchanged: `run_cell`, `real_reference` and metrics come from SIM-EXP
run_gauge_real_oscillations_mjwarp.py; only its `simulate` is replaced by a SAP simulation.  Ball mass and inertia are
the values MuJoCo compiles from the released ball-6.obj (the v3.1 MuJoCo artifacts' `compiled.body_inertia`); the
1e-8 m mesh COM offset is ignored.  Every physics step checks finite joint_q and joint_qd (v3.1 gate).  Contact-free.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import argparse, hashlib, importlib.metadata as metadata, json, sys, tempfile
from pathlib import Path
import numpy as np, warp as wp, yaml
from sap_warp.collision import SapCollisionPipeline
from sap_warp.loader import load_sap_scene
from sap_warp.sap_kinematics import sap_eval_fk
from sap_warp.solver_sap import SolverSAP
from run_pendulum_v2_sap import parse_candidate
DEVICE = "cuda:0"
I_MESH_PER_KG = np.array([0.00011423132724318597, 0.00011423128235464011, 0.00011420859158445886]) / 0.3175


def make_simulate(gp):
    def simulate(task, candidate, cell, angles, graph=None):
        assert task == "pendulum"
        sps = round((1000 / gp.FPS) / cell["dt_ms"]); dt = 1.0 / (gp.FPS * sps)
        W, S = len(angles), cell["frames"]; m = gp.MASS * float(cell.get("mass_scale", 1.0)); I = (I_MESH_PER_KG * m).tolist()
        a = [float(x) for x in gp.ANCHOR]
        sc = {"schema_version": 1, "name": "gauge_pendulum_real", "simulation": {"dt": dt, "num_worlds": W, "nconmax": 8},
              "builder": {"gravity": -9.81, "defaults": {"shape": {"density": 0.0}}}, "ground": {"enabled": False},
              "bodies": [{"id": "ball", "transform": {"p": [a[0], a[1], a[2] - gp.L], "q": "identity"}, "mass": m,
                          "inertia": [I[0], 0, 0, 0, I[1], 0, 0, 0, I[2]], "lock_inertia": True, "shapes": [{"type": "sphere", "radius": gp.RAD}]}],
              "joints": [{"id": "hinge", "type": "revolute", "parent": "world", "child": "ball", "axis": [0, 1, 0],
                          "parent_xform": {"p": a, "q": "identity"}, "child_xform": {"p": [0, 0, gp.L], "q": "identity"}}],
              "articulations": [{"id": "a", "joints": ["hinge"]}]}
        path = Path(tempfile.mkdtemp()) / "pendulum.yaml"; path.write_text(yaml.safe_dump(sc, sort_keys=False))
        l = load_sap_scene(path, device=wp.get_device(DEVICE), naconmax=8 * W, strict=True, num_worlds=W)
        mdl, s, ctrl = l.sap_model, l.sap_state, l.sap_control; o = mdl.state()
        pipe = SapCollisionPipeline(l.collision_model, naconmax=8 * W, bounds_filter="sphere_obb"); cts = pipe.contacts()
        sol = SolverSAP(mdl, nconmax=8 * W, enable_sleep=False, dtype=(wp.float64 if candidate["dtype"] == "float64" else wp.float32))
        q0 = s.joint_q.numpy().reshape(W, 1); q0[:, 0] = angles; s.joint_q.assign(q0.reshape(-1)); sap_eval_fk(mdl, s.joint_q, s.joint_qd, s)
        Q = np.zeros((S, W, 1)); V = np.zeros((S, W, 1)); finite = np.ones(W, bool)
        for k in range(S):
            Q[k, :, 0] = s.joint_q.numpy(); V[k, :, 0] = s.joint_qd.numpy()
            if k < S - 1:
                for _ in range(sps):
                    s.clear_forces(); sol.collide(pipe, cts, s, ctrl, collision_state=s); sol.step(s, o, ctrl, cts, dt); s, o = o, s
                    finite &= np.isfinite(s.joint_q.numpy()) & np.isfinite(s.joint_qd.numpy())
        health = {"finite_by_world": finite.tolist(), "capacity_ok_by_world": [True] * W, "contacts": "none (contact-free task)"}
        compiled = {"dt_s": dt, "steps_per_frame": sps, "engine": "SAP Warp", "dtype": candidate["dtype"], "body_mass": [m],
                    "body_inertia": [I], "hinge_inertia": [m * gp.L ** 2 + I[1]], "dof_damping": [0.0],
                    "scene_yaml_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        return Q, V, finite, health, compiled
    return simulate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--preflight-json", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    pf = json.loads(a.preflight_json.read_text()); dev = pf.get("devices", {}).get(DEVICE)
    if not pf.get("all_pass") or not dev or not all(v["pass"] for v in dev.values()) or (pf.get("cpu_gpu") or {}).get("pass") is not True:
        raise SystemExit("SAP preflight must pass on cuda:0 with CPU/GPU agreement")
    sys.path.insert(0, str(a.simexp_scripts))
    import run_gauge_real_oscillations_mjwarp as G
    gp = G.gp; G.simulate = make_simulate(gp); ROOT = G.ROOT = REPO_ROOT
    wp.init(); man, sha = G.load_manifest(a.manifest)
    for name, expected in man["data"]["file_sha256"].items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != expected:
            raise ValueError("GAUGE source changed: " + name)
    trials = gp.load_trials(ROOT); cands = [parse_candidate(t) for t in man["candidate_rule"]["sap_candidates"]]
    a.out.mkdir(parents=True, exist_ok=True); path = a.out / "sap_gauge_pendulum_real.json"
    import sap_warp
    data = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha, "engine": "SAP Warp internal",
            "complete": False, "sap_source": str(Path(sap_warp.__file__).parent), "warp_version": metadata.version("warp-lang"), "device": DEVICE,
            "preflight": pf, "real_reference": [{"trial": t["id"], **G.real_reference("pendulum", t, man["selection"]["frames"])} for t in trials],
            "selection_candidates": [], "frozen_candidate": None, "heldout": []}
    for i, c in enumerate(cands):
        r = G.run_cell("pendulum", c, man["selection"], trials, man["seed0"], man["R"], a.out / f"selection_{i + 1}.npz")
        r["candidate_index"] = i + 1; data["selection_candidates"].append(r); G.save(path, data)
        print("selection", i + 1, c, "valid", r["valid_count"], "/", r["n"], "loss", r["loss"], flush=True)
    valid = [r for r in data["selection_candidates"] if r["gate"] and r["loss"] is not None]
    if not valid:
        data["stop_reason"] = "No valid selection candidate; held-out evaluation not started"; G.save(path, data); return
    win = min(valid, key=lambda r: (r["loss"], r["wall_s"]))
    data["frozen_candidate"] = win["candidate"]; data["frozen_index"] = win["candidate_index"]; G.save(path, data)
    for j, cell in enumerate(man["held_out_conditions"]):
        r = G.run_cell("pendulum", win["candidate"], cell, trials, man["seed0"] + 1000 + j, man["R"], a.out / f"heldout_{j + 1}.npz")
        data["heldout"].append(r); G.save(path, data)
        print("heldout", cell["id"], "valid", r["valid_count"], "/", r["n"], "loss", r["loss"], flush=True)
    data["complete"] = True; G.save(path, data)


if __name__ == "__main__":
    main()
