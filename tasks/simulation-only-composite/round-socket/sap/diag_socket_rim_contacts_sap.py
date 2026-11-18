"""Diagnostic (not a protocol run): SAP contact points on the round-socket v6 rim-rest cells.

Re-runs the frozen v6 SAP candidate on the given held-out rim cells (rollout 0 of the held-out seed, R=1) and records,
every SAMPLE_S, the peg pose plus every peg contact the solver used: world point, normal (R_WC column 2), phi0 and the
normal impulse (contact_gamma, normal component).  Output: one npz per cell.
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
SAMPLE_S = 0.005


def run(sc, cand, cell, xy, out):
    dt = cell["dt_ms"] * 1e-3; ncon = 512
    mat = {"density": 0.0, "mu": cell["mu"], "ke": cand["ke"], "tau": cand["tau"], "gap": cand["gap"]}
    I = sc.peg_inertia(cell); pos0, q0 = sc.initial_pose(cell)
    peg_xf = {"p": [float(v) for v in pos0], "q": [float(q0[1]), float(q0[2]), float(q0[3]), float(q0[0])]}
    world = [{"type": "convex_mesh", "path": str(SIMEXP / sc.ASSET_DIR / f), "is_solid": True} for f in sc.FILES]
    scene = {"schema_version": 1, "name": "round_socket_rim_diag", "simulation": {"dt": dt, "num_worlds": 1, "nconmax": ncon},
             "builder": {"gravity": -9.81, "rigid_gap": 0.0, "defaults": {"shape": mat}}, "ground": {"enabled": False},
             "world_shapes": world,
             "bodies": [{"id": "peg", "transform": peg_xf, "mass": sc.PEG_MASS, "inertia": [I[0], 0, 0, 0, I[1], 0, 0, 0, I[2]], "lock_inertia": True,
                         "shapes": [{"type": "cylinder", "radius": sc.peg_radius(cell), "half_height": sc.PEG_HALF}]}],
             "joints": [{"id": "peg_free", "type": "free", "parent": "world", "child": "peg"}],
             "articulations": [{"id": "a", "joints": ["peg_free"]}]}
    path = Path(tempfile.mkdtemp()) / "socket.yaml"; path.write_text(yaml.safe_dump(scene, sort_keys=False))
    l = load_sap_scene(path, device=wp.get_device(DEVICE), naconmax=ncon, strict=True, num_worlds=1)
    m, s, ctrl = l.sap_model, l.sap_state, l.sap_control; o = m.state()
    pipe = SapCollisionPipeline(l.collision_model, naconmax=ncon, bounds_filter="sphere_obb"); cts = pipe.contacts()
    sol = SolverSAP(m, nconmax=ncon, enable_sleep=False)
    q = s.joint_q.numpy().reshape(1, 7); q[:, 0:2] += xy; s.joint_q.assign(q.reshape(-1)); sap_eval_fk(m, s.joint_q, s.joint_qd, s)
    every = max(1, int(round(SAMPLE_S / dt))); T, P, Q, C = [], [], [], []
    for k in range(round(cell["duration_s"] / dt) + 1):
        qq = s.joint_q.numpy().reshape(7).astype(float)
        rec = k % every == 0
        s.clear_forces(); sol.collide(pipe, cts, s, ctrl, collision_state=s); sol.step(s, o, ctrl, cts, dt)
        if rec:
            jac, res = sol.last_contact_jacobian_result, sol.last_contact_solve_result
            n = int(jac.contact_env_count.numpy()[0]) if jac is not None else 0
            rows = np.zeros((n, 8))
            if n:
                pt = jac.contact_env_point.numpy()[0, :n]; Rwc = jac.contact_env_R_WC.numpy()[0, :n]; phi = jac.contact_env_phi0.numpy()[0, :n]
                g = res.contact_gamma.numpy() if res is not None else None
                gn = np.zeros(n)
                if g is not None:
                    g = g.reshape(1, -1, 3)[0, :n] if g.size % 3 == 0 else None
                    gn = g[:, 2] if g is not None else gn
                rows[:, 0:3] = pt; rows[:, 3:6] = Rwc[:, :, 2]; rows[:, 6] = phi; rows[:, 7] = gn / dt     # normal force (N)
            T.append(k * dt); P.append(qq[0:3]); Q.append(qq[[6, 3, 4, 5]]); C.append(rows)
        s, o = o, s
    off = np.cumsum([0] + [len(c) for c in C])
    np.savez(out, t=np.array(T), pos=np.array(P), quat=np.array(Q), contacts=np.concatenate(C) if off[-1] else np.zeros((0, 8)), offsets=off,
             meta=json.dumps({"engine": "SAP Warp", "candidate": cand["id"], "cell": cell, "xy": xy.tolist(), "columns": "px py pz nx ny nz phi0 fn"}))
    print("wrote", out, "samples", len(T), "contacts/sample max", max(len(c) for c in C), "final z", P[-1][2], flush=True)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--result", type=Path, required=True); ap.add_argument("--cells", nargs="+", required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--rollout", type=int, default=0)
    a = ap.parse_args(); sys.path.insert(0, str(a.simexp_scripts)); import socket_v6_common as sc
    from run_socket_v4_sap import parse_candidate
    wp.init(); man, _ = sc.load_manifest(a.manifest); res = json.loads(a.result.read_text())
    cand = parse_candidate(res["frozen_candidate"]["id"]); a.out.mkdir(parents=True, exist_ok=True)
    for j, cell in enumerate(man["held_out_conditions"]):
        if cell["id"] in a.cells:
            xy = sc.perturbation(cell, man["seed0"] + 1000 + j, man["R"])[a.rollout:a.rollout + 1]
            run(sc, cand, cell, xy, a.out / (f"sap_{cell['id']}.npz" if a.rollout == 0 else f"sap_{cell['id']}_r{a.rollout}.npz"))


if __name__ == "__main__":
    main()
