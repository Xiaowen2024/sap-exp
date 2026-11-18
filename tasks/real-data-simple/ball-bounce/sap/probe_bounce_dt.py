"""Diagnostic (not a protocol run): SAP ball-bounce restitution vs timestep and stiffness.

GAUGE trial-1 frame-0 release (zero velocity), single world, tau=0.  For each (dt, ke): first-bounce e = sqrt(apex1/h0)
(heights above the contact point), peak penetration = max(-phi0) over the run, contact duration, wall time.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import json
import math
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import warp as wp
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import impact_v2_common as ic  # noqa: E402
from run_impact_v2_sap import scene  # noqa: E402
from sap_warp.collision import SapCollisionPipeline  # noqa: E402
from sap_warp.loader import load_sap_scene  # noqa: E402
from sap_warp.sap_kinematics import sap_eval_fk  # noqa: E402
from sap_warp.solver_sap import SolverSAP  # noqa: E402

FRAME = 1 / 30
SPS = [6, 12, 30, 60, 150, 300]           # dt = 5.56, 2.78, 1.11, 0.556, 0.222, 0.111 ms
KES = [2.5e3, 1e4, 1e5, 1e6]
T_END = 1.2


def run(ke, dt, p0):
    ncon = 16
    path = Path(tempfile.mkdtemp()) / "s.yaml"
    path.write_text(yaml.safe_dump(scene("ball", {"ke": ke, "tau": 0.0}, dt, 1.0, 1, ncon), sort_keys=False))
    l = load_sap_scene(path, device=wp.get_device("cuda:0"), naconmax=ncon, strict=True, num_worlds=1)
    m, s, ctrl = l.sap_model, l.sap_state, l.sap_control; o = m.state()
    pipe = SapCollisionPipeline(l.collision_model, naconmax=ncon, bounds_filter="sphere_obb"); cts = pipe.contacts()
    sol = SolverSAP(m, nconmax=ncon, enable_sleep=False)
    q = s.joint_q.numpy(); q[0:3] = p0; q[3:7] = [0, 0, 0, 1]; s.joint_q.assign(q); sap_eval_fk(m, s.joint_q, s.joint_qd, s)
    g = ic.BALL_GROUND + ic.BALL_RADIUS; h0 = p0[2] - g
    z = []; pen = 0.0; in_contact = []; t0 = time.perf_counter()
    for k in range(round(T_END / dt)):
        s.clear_forces(); sol.collide(pipe, cts, s, ctrl, collision_state=s); sol.step(s, o, ctrl, cts, dt); s, o = o, s
        jac = sol.last_contact_jacobian_result; c = 0
        if jac is not None:
            n = int(jac.contact_env_count.numpy()[0])
            if n:
                phi = jac.contact_env_phi0.numpy()[0, :n]; pen = max(pen, float(np.max(-phi))); c = int(np.min(phi) < 0)
        in_contact.append(c); z.append(float(s.joint_q.numpy()[2]))
    wall = time.perf_counter() - t0
    z = np.array(z); ic_ = np.array(in_contact)
    first = int(np.argmax(ic_)) if ic_.any() else None
    if first is None:
        return {"ke": ke, "dt_ms": dt * 1e3, "e1": None}
    after = np.where(ic_[first:] == 0)[0]
    if not len(after):
        return {"ke": ke, "dt_ms": dt * 1e3, "e1": 0.0, "peak_pen_mm": pen * 1e3, "contact_ms": None, "wall_s": wall}
    leave = first + int(after[0])
    apex = float(z[leave:].max() - g)
    return {"ke": ke, "dt_ms": dt * 1e3, "e1": math.sqrt(max(apex, 0) / h0), "apex1_m": apex, "h0_m": h0,
            "peak_pen_mm": pen * 1e3, "contact_ms": (leave - first) * dt * 1e3, "contact_steps": leave - first,
            "wall_s": wall, "wall_per_sim_s": wall / T_END}


def main():
    wp.init()
    ids, xyz = ic.ball_trials(REPO_ROOT)
    p0 = xyz[0][0].copy()
    rows = []
    for sps in SPS:
        for ke in KES:
            r = run(ke, FRAME / sps, p0); rows.append(r); print(json.dumps(r), flush=True)
    out = Path("/root/bounce_dt_probe"); out.mkdir(exist_ok=True)
    (out / "sap_bounce_dt_probe.json").write_text(json.dumps({"trial": ids[0], "rows": rows}, indent=1) + "\n")


if __name__ == "__main__":
    main()
