"""Private SAP Warp three-box push strict-v4 run (two-cell selection, retain/slip/tip labels, +-10% ke check).

Same force profile, perturbations, seeds and classification as the MuJoCo Warp driver (strict_v3_common).
Requires the fixed SAP build + passing preflight; CUDA only (bug A on CPU in db8d236).
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import warp as wp
import yaml

from sap_warp.collision import SapCollisionPipeline
from sap_warp.loader import load_sap_scene
from sap_warp.solver_sap import SolverSAP

G = 9.81
NCONMAX = 2048
SAMPLE_S = 0.01


def parse_candidate(text: str) -> dict:
    if text.strip() == "native default":
        return {"id": "native_default", "shape_ke_N_m": None, "shape_tau_s": None, "sigma": None}
    kv = dict(p.split("=") for p in text.replace(" ", "").split(","))
    tau = kv["tau"]
    return {"id": text, "shape_ke_N_m": float(kv["ke"]),
            "shape_tau_s": float(tau[:-2]) * 1e-3 if tau.endswith("ms") else float(tau),
            "sigma": float(kv["sigma"]) if "sigma" in kv else None}


def scene(c: dict, *, mass: float, mu: float, half: float, dt: float, worlds: int) -> dict:
    material = {"density": mass / (2 * half) ** 3, "mu": mu}
    if c["shape_ke_N_m"] is not None:
        material["ke"] = c["shape_ke_N_m"]; material["tau"] = c["shape_tau_s"]
    return {"schema_version": 1, "name": "three_box_strict_v4",
            "simulation": {"dt": dt, "num_worlds": worlds, "nconmax": NCONMAX},
            "builder": {"gravity": -G, "defaults": {"shape": material}},
            "ground": {"enabled": True, "cfg": dict(material)},
            "bodies": [{"id": f"box{i}", "transform": {"p": [0, 0, half + 2 * half * i], "q": "identity"},
                        "shapes": [{"type": "box", "hx": half, "hy": half, "hz": half}]} for i in range(3)],
            "joints": [{"id": f"box{i}_free", "type": "free", "parent": "world", "child": f"box{i}"} for i in range(3)],
            "articulations": [{"id": f"box{i}_art", "joints": [f"box{i}_free"]} for i in range(3)]}


def run_cell(c: dict, cell: dict, seed: int, R: int, out: Path, device: str, common) -> dict:
    dt, mass, mu, half = cell["dt_ms"] * 1e-3, cell["mass_kg"], cell["mu"], cell["box_halfwidth_mm"] * 1e-3
    tag = c["id"].replace(",", "_").replace("=", "").replace(" ", "").replace("|", "_")
    path = out / "scenes" / f"{cell['id']}_{tag}.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(scene(c, mass=mass, mu=mu, half=half, dt=dt, worlds=R), sort_keys=False))
    loaded = load_sap_scene(path, device=wp.get_device(device), naconmax=NCONMAX, strict=True, num_worlds=R)
    model, state, control = loaded.sap_model, loaded.sap_state, loaded.sap_control
    other = model.state()
    pipeline = SapCollisionPipeline(loaded.collision_model, naconmax=NCONMAX, bounds_filter="sphere_obb")
    contacts = pipeline.contacts()
    kw = {} if c.get("sigma") is None else {"contact_sigma": c["sigma"]}
    solver = SolverSAP(model, nconmax=NCONMAX, **kw)
    pushed = 1 if cell["pushed_body"] == "middle" else 2
    support = pushed - 1
    expected, _ = common.box_v4_expected(cell)
    if expected != cell["expected"]:
        raise ValueError(f"manifest label disagrees with analytic rule for {cell['id']}")
    force = common.box_v4_force_mg(cell) * mass * G
    xy, yaw, scale = common.box_perturbations(seed, R)
    q = state.joint_q.numpy().reshape(R, 3, 7)
    q[:, :, :2] += xy
    q[:, :, 3:7] = 0.0; q[:, :, 5] = np.sin(yaw / 2); q[:, :, 6] = np.cos(yaw / 2)   # xyzw
    state.joint_q.assign(q.reshape(-1))

    def step(fx: np.ndarray) -> None:
        nonlocal state, other
        state.clear_forces()
        bf = state.body_f.numpy().reshape(R, 3, 6)
        bf[:, pushed, 0] = fx
        state.body_f.assign(bf.reshape(-1, 6))
        solver.collide(pipeline, contacts, state, control, collision_state=state)
        solver.step(state, other, control, contacts, dt)
        state, other = other, state

    total = common.BOX_SETTLE_S + common.BOX_RAMP_S + common.BOX_HOLD_S
    steps, every = round(total / dt), max(1, round(SAMPLE_S / dt))
    T, P, Q, D, F = [], [], [], [], []
    trunc = 0
    t0 = time.perf_counter()
    for k in range(steps):
        t = k * dt
        fac = float(np.clip((t - common.BOX_SETTLE_S) / common.BOX_RAMP_S, 0, 1))
        step(force * scale * fac)
        tn = (k + 1) * dt
        if (k + 1) % every == 0 and tn >= common.BOX_SETTLE_S - 1e-12:
            qq = state.joint_q.numpy().reshape(R, 3, 7).astype(np.float64)
            P.append(qq[:, :, 0:3].copy()); Q.append(qq[:, :, [6, 3, 4, 5]].copy())    # -> wxyz
            fn = np.zeros(R); cnt = np.zeros(R)
            jac, sol = solver.last_contact_jacobian_result, solver.last_contact_solve_result
            if jac is not None and sol is not None:
                counts = jac.contact_env_count.numpy(); gamma = sol.contact_gamma.numpy()
                b0, b1 = jac.contact_env_body0.numpy(), jac.contact_env_body1.numpy()
                for w in range(R):
                    for j in range(int(counts[w])):
                        if sorted((int(b0[w, j]) - 3 * w, int(b1[w, j]) - 3 * w)) == [support, pushed]:
                            fn[w] += max(0.0, float(gamma[w, j, 2]) / dt); cnt[w] += 1
            trunc = max(trunc, int(solver.last_truncated_contact_count or 0))
            T.append(tn - common.BOX_SETTLE_S); D.append(cnt > 0); F.append(fn)
    pipeline.check_buffers(contacts)
    P, Q = np.asarray(P), np.asarray(Q)
    Q = Q / np.linalg.norm(Q, axis=-1, keepdims=True)
    res = common.box_v4_summary(np.asarray(T), P, Q, np.asarray(D, float), np.asarray(F), half=half, pushed=pushed,
                                expected=expected, finite=np.isfinite(P).all(axis=(0, 2, 3)), truncated=trunc)
    res.update({"cell": cell, "force_N": force, "seed": seed, "candidate": c, "wall_s": time.perf_counter() - t0})
    return res


def fmt(r: dict) -> str:
    return (f'{r["agree_and_valid_count"]:2d}/20 obs={r["observed_counts"]} face={r["face_disp_final_mm_median"]:.3f}mm '
            f'com={r["com_rel_disp_final_mm_median"]:.3f}mm tilt={r["tilt_max_deg_median"]:.2f}deg creep={r["hold_face_creep_um_s_mean"]:.1f}um/s')


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--preflight-json", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--device", default="cuda:0")
    a = ap.parse_args()
    if not a.device.startswith("cuda"):
        raise SystemExit("multi-body SAP runs must use CUDA (bug A on CPU in db8d236)")
    pf = json.loads(a.preflight_json.read_text())
    dev_checks = pf.get("devices", {}).get(a.device)
    if not pf.get("all_pass", False) or not dev_checks or not all(v["pass"] for v in dev_checks.values()) \
            or (pf.get("cpu_gpu") or {}).get("pass") is not True:
        raise SystemExit(f"SAP preflight must pass on {a.device} with CPU/GPU agreement: {a.preflight_json}")
    sys.path.insert(0, str(a.simexp_scripts))
    import strict_v3_common as common
    man, sha = common.load_manifest(a.manifest)
    R, seed0 = man["R"], man["seed0"]
    wp.init(); a.out.mkdir(parents=True, exist_ok=True)
    cands = [parse_candidate(t) for t in man["candidate_rule"]["sap_candidates"]]
    sels = man["selection"]
    selection, walls = [], []
    for i, c in enumerate(cands):
        rows = [run_cell(c, s, seed0 + k, R, a.out, a.device, common) for k, s in enumerate(sels)]
        selection.append(rows); walls.append(sum(r["wall_s"] for r in rows))
        for r in rows:
            print("select", i + 1, c["id"], r["cell"]["id"], r["expected"], fmt(r), flush=True)
    win = common.box_v4_choose(selection, walls); frozen = cands[win]
    print("frozen", frozen, flush=True)
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(frozen, cell, seed0 + 1000 + j, R, a.out, a.device, common); held.append(r)
        print("held", cell["id"], r["expected"], fmt(r), flush=True)
    sens = []
    byid = {c["id"]: c for c in man["held_out_conditions"]}
    if frozen["shape_ke_N_m"] is None:
        sens.append({"skipped": "frozen setting is the native default; no declared stiffness parameter to scale"})
    else:
        k = 0
        for cid in man["parameter_sensitivity"]["cells"]:
            for s in man["parameter_sensitivity"]["scales"]:
                c = dict(frozen); c["shape_ke_N_m"] = frozen["shape_ke_N_m"] * s; c["id"] = f'{frozen["id"]} | ke x{s}'
                r = run_cell(c, byid[cid], seed0 + 2000 + k, R, a.out, a.device, common); r["scale"] = s; sens.append(r); k += 1
                print("sens", cid, s, fmt(r), flush=True)
    import sap_warp
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
           "engine": "SAP Warp internal", "sap_source": str(Path(sap_warp.__file__).parent), "warp_version": wp.config.version,
           "device": a.device, "preflight": pf, "B1": selection[0], "selection_candidates": selection,
           "frozen_index": win + 1, "frozen_candidate": frozen, "heldout": held, "parameter_sensitivity": sens,
           "heldout_accuracy_total": [sum(r["agree_and_valid_count"] for r in held), R * len(held)]}
    (a.out / "sap_three_box_strict_v4.json").write_text(json.dumps(common.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
