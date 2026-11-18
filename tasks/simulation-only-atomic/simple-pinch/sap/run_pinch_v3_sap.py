"""Private SAP Warp simple-pinch force-controlled-v3 driver (public helpers in SIM-EXP pinch_v3_common).

Pads: x prismatic joints get a constant inward joint force F (control.joint_f) with damping-only PD
(target_ke 0, target_kd 300); z prismatic joints track the lift (ke 2e4, kd 300).  Gravity off; the cube load
m g (x scale) is a -z body force during lift/hold.  Shape density is 0 so body masses are exactly the declared
values (SAP adds shape density*volume to an explicit body mass); the cube therefore gets an explicit inertia.  Requires fixed SAP + preflight; CUDA only.
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

DEVICE = "cuda:0"
NCON = 1024


def parse_candidate(text: str) -> dict:
    if text.strip() == "native default":
        return {"id": "native_default", "ke": None, "tau": None, "sigma": None}
    kv = dict(p.split("=") for p in text.replace(" ", "").split(","))
    tau = kv["tau"]
    return {"id": text, "ke": float(kv["ke"]), "tau": float(tau[:-2]) * 1e-3 if tau.endswith("ms") else float(tau),
            "sigma": float(kv["sigma"]) if "sigma" in kv else None}


def scene(c: dict, cell: dict, W: int, pc) -> dict:
    shape = {"density": 0.0, "mu": cell["mu"]}
    if c["ke"] is not None:
        shape.update(ke=c["ke"], tau=c["tau"])
    hy = cell["pad_halfwidth_y_mm"] * 1e-3
    x = 0.03
    Ic = cell["mass_kg"] * (2 * 0.04 ** 2) / 12.0                     # solid 40-mm cube
    px = {"target_ke": 0.0, "target_kd": pc.PAD_KD, "actuator_mode": 1}
    pz = {"target_ke": pc.PAD_Z_KE, "target_kd": pc.PAD_Z_KD, "actuator_mode": 1}
    return {"schema_version": 1, "name": "simple_pinch_v3",
            "simulation": {"dt": cell["dt_ms"] * 1e-3, "num_worlds": W, "nconmax": NCON},
            "builder": {"gravity": 0.0, "rigid_gap": 0.0, "defaults": {"shape": shape}}, "ground": {"enabled": False},
            "bodies": [{"id": "left_stage", "transform": {"p": [-x, 0, 0], "q": "identity"}, "mass": 0.01, "shapes": []},
                       {"id": "left", "transform": {"p": [-x, 0, 0], "q": "identity"}, "mass": 1.0, "shapes": [{"type": "box", "hx": .01, "hy": hy, "hz": .04}]},
                       {"id": "right_stage", "transform": {"p": [x, 0, 0], "q": "identity"}, "mass": 0.01, "shapes": []},
                       {"id": "right", "transform": {"p": [x, 0, 0], "q": "identity"}, "mass": 1.0, "shapes": [{"type": "box", "hx": .01, "hy": hy, "hz": .04}]},
                       {"id": "cube", "transform": {"p": [0, 0, 0], "q": "identity"}, "mass": cell["mass_kg"],
                        # explicit solid-box inertia: with density 0 and no inertia SAP would clamp it to 1e-6 kg m^2
                        "inertia": [Ic, 0, 0, 0, Ic, 0, 0, 0, Ic], "lock_inertia": True,
                        "shapes": [{"type": "box", "hx": .02, "hy": .02, "hz": .02}]}],
            "joints": [{"id": "left_x", "type": "prismatic", "parent": "world", "child": "left_stage", "axis": [1, 0, 0], "parent_xform": {"p": [-x, 0, 0], "q": "identity"}, "child_xform": "identity", **px},
                       {"id": "left_z", "type": "prismatic", "parent": "left_stage", "child": "left", "axis": [0, 0, 1], **pz},
                       {"id": "right_x", "type": "prismatic", "parent": "world", "child": "right_stage", "axis": [1, 0, 0], "parent_xform": {"p": [x, 0, 0], "q": "identity"}, "child_xform": "identity", **px},
                       {"id": "right_z", "type": "prismatic", "parent": "right_stage", "child": "right", "axis": [0, 0, 1], **pz},
                       {"id": "cube_free", "type": "free", "parent": "world", "child": "cube"}],
            "articulations": [{"id": "la", "joints": ["left_x", "left_z"]}, {"id": "ra", "joints": ["right_x", "right_z"]}, {"id": "ca", "joints": ["cube_free"]}]}


def make_run_cell(pc):
    def run_cell(c: dict, cell: dict, seed: int, R: int) -> dict:
        path = Path(tempfile.mkdtemp()) / "pinch.yaml"
        path.write_text(yaml.safe_dump(scene(c, cell, R, pc), sort_keys=False))
        l = load_sap_scene(path, device=wp.get_device(DEVICE), naconmax=NCON, strict=True, num_worlds=R)
        m, s, ctrl = l.sap_model, l.sap_state, l.sap_control
        o = m.state()
        pipe = SapCollisionPipeline(l.collision_model, naconmax=NCON, bounds_filter="sphere_obb"); cts = pipe.contacts()
        kw = {} if c.get("sigma") is None else {"contact_sigma": c["sigma"]}
        sol = SolverSAP(m, nconmax=NCON, enable_sleep=False, **kw)
        p = pc.perturbations(seed, R)
        q = s.joint_q.numpy().reshape(R, 11)
        q[:, 5:7] += p["yz"]; q[:, 7:11] = 0; q[:, 8] = np.sin(p["pitch"] / 2); q[:, 10] = np.cos(p["pitch"] / 2)   # xyzw, pitch about y
        s.joint_q.assign(q.reshape(-1)); sap_eval_fk(m, s.joint_q, s.joint_qd, s)
        dt = cell["dt_ms"] * 1e-3; F = cell["per_pad_force_N"]; load = cell["mass_kg"] * pc.G * p["load_scale"]
        jf = np.zeros((R, 10), np.float32); jf[:, 0] = F; jf[:, 2] = -F            # inward x forces
        ctrl.joint_f.assign(jf.reshape(-1))
        every = max(1, round(pc.SAMPLE_S / dt)); trunc = 0
        state = {"s": s, "o": o}

        def step(z, vz, with_load):
            tgt = ctrl.joint_target_pos.numpy().reshape(R, 10); tgt[:, :4] = 0; tgt[:, 1] = z; tgt[:, 3] = z
            ctrl.joint_target_pos.assign(tgt.reshape(-1))
            if hasattr(ctrl, "joint_target_vel") and ctrl.joint_target_vel is not None:
                tv = ctrl.joint_target_vel.numpy().reshape(R, 10); tv[:, :4] = 0; tv[:, 1] = vz; tv[:, 3] = vz
                ctrl.joint_target_vel.assign(tv.reshape(-1))
            ss = state["s"]; ss.clear_forces()
            if with_load:
                bf = ss.body_f.numpy().reshape(R, 5, 6); bf[:, 4, 2] = -load; ss.body_f.assign(bf.reshape(-1, 6))
            sol.collide(pipe, cts, ss, ctrl, collision_state=ss); sol.step(ss, state["o"], ctrl, cts, dt)
            state["s"], state["o"] = state["o"], ss

        def pad_forces():
            jac, so = sol.last_contact_jacobian_result, sol.last_contact_solve_result
            fn = np.zeros((R, 2)); cnt = np.zeros((R, 2), bool)
            if jac is None or so is None:
                return fn, cnt
            n = jac.contact_env_count.numpy(); g = so.contact_gamma.numpy()
            b0, b1 = jac.contact_env_body0.numpy(), jac.contact_env_body1.numpy()
            for w in range(R):
                for j in range(int(n[w])):
                    pair = {int(b0[w, j]) - 5 * w, int(b1[w, j]) - 5 * w}
                    if 4 in pair:
                        side = 0 if 1 in pair else (1 if 3 in pair else None)
                        if side is not None:
                            fn[w, side] += max(0.0, float(g[w, j, 2]) / dt); cnt[w, side] = True
            return fn, cnt

        gate_steps = round(pc.GATE_S / dt); GF, GD = [], []
        for k in range(gate_steps):
            step(0.0, 0.0, False); trunc = max(trunc, int(sol.last_truncated_contact_count or 0))
            if k >= gate_steps - round(0.2 / dt) and k % every == 0:
                fn, cnt = pad_forces(); GF.append(fn); GD.append(cnt)
        qq = state["s"].joint_q.numpy().reshape(R, 11); z0 = qq[:, 6].copy(); pz0 = 0.5 * (qq[:, 1] + qq[:, 3])
        steps = round((pc.LIFT_S + pc.HOLD_S) / dt); T, RISE, REL = [], [], []
        for k in range(steps + 1):
            t = k * dt; z, vz = pc.lift_target(t)
            if k % every == 0 or k == steps:
                qq = state["s"].joint_q.numpy().reshape(R, 11); cz = qq[:, 6] - z0; pz = 0.5 * (qq[:, 1] + qq[:, 3]) - pz0
                T.append(t); RISE.append(cz.astype(float)); REL.append((cz - pz).astype(float))
            if k < steps:
                step(z, vz, True); trunc = max(trunc, int(sol.last_truncated_contact_count or 0))
        pipe.check_buffers(cts)
        RISE = np.asarray(RISE)
        return pc.summarize(cell, gate_fn=np.asarray(GF), gate_duty=np.asarray(GD), t=np.asarray(T), cube_rise=RISE,
                            rel_z=np.asarray(REL), finite=np.isfinite(RISE).all(axis=0), truncated=trunc)
    return run_cell


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--preflight-json", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args()
    pf = json.loads(a.preflight_json.read_text()); dev = pf.get("devices", {}).get(DEVICE)
    if not pf.get("all_pass") or not dev or not all(v["pass"] for v in dev.values()) or (pf.get("cpu_gpu") or {}).get("pass") is not True:
        raise SystemExit(f"SAP preflight must pass on {DEVICE} with CPU/GPU agreement: {a.preflight_json}")
    sys.path.insert(0, str(a.simexp_scripts))
    import pinch_v3_common as pc
    wp.init()
    man, sha = pc.load_manifest(a.manifest)
    res = pc.execute(man, "sap_candidates", parse_candidate, make_run_cell(pc), log=lambda s: print(s, flush=True))
    import sap_warp
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
           "engine": "SAP Warp internal", "sap_source": str(Path(sap_warp.__file__).parent), "warp_version": wp.config.version,
           "device": DEVICE, "preflight": pf, **res}
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / "sap_simple_pinch_v3.json").write_text(json.dumps(pc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
