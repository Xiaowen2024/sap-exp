"""Private SAP Warp three-plate strict-v3 run (manifest-driven B=1/B=5 + frozen held-out matrix).

Imports the public strict_v3_common helpers (scene-independent gates, seeds and
perturbations) so SAP and MuJoCo Warp see identical perturbation arrays.
Fixes vs run_three_plate_b1_b5.py: quaternion written in SAP's xyzw order (the
old runner wrote cos(yaw/2) into x, i.e. a ~pi rotation about x = a tilt rather
than a yaw), full linear/angular velocity in the settling gate, mass/radius/
offset axes.  Requires the fixed SAP build (fix-gamma-readout) + passing preflight.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import math
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
        return {"id": "native_default", "shape_ke_N_m": None, "shape_tau_s": None}
    kv = dict(p.split("=") for p in text.replace(" ", "").split(","))
    tau = kv["tau"]
    tau_s = float(tau[:-2]) * 1e-3 if tau.endswith("ms") else float(tau)
    return {"id": text, "shape_ke_N_m": float(kv["ke"]), "shape_tau_s": tau_s}


def scene(c: dict, *, h: float, mass: float, radius: float, offset: float, dt: float, worlds: int) -> dict:
    material = {"density": mass / (math.pi * radius * radius * h), "mu": 0.8}
    if c["shape_ke_N_m"] is not None:
        material["ke"] = c["shape_ke_N_m"]; material["tau"] = c["shape_tau_s"]
    return {
        "schema_version": 1, "name": "three_plate_strict_v3",
        "simulation": {"dt": dt, "num_worlds": worlds, "nconmax": NCONMAX},
        "builder": {"gravity": -G, "defaults": {"shape": material}},
        "ground": {"enabled": True},
        "bodies": [{"id": f"plate{i}", "transform": {"p": [i * offset, 0, (i + .5) * h], "q": "identity"},
                    "shapes": [{"type": "cylinder", "radius": radius, "half_height": h / 2}]} for i in range(3)],
        "joints": [{"id": f"plate{i}_free", "type": "free", "parent": "world", "child": f"plate{i}"} for i in range(3)],
        "articulations": [{"id": "plate_stack", "joints": [f"plate{i}_free" for i in range(3)]}],
    }


def interface_loads(solver, worlds: int, dt: float) -> tuple[np.ndarray, np.ndarray]:
    fn = np.zeros((worlds, 3)); cnt = np.zeros((worlds, 3))
    jac, sol = solver.last_contact_jacobian_result, solver.last_contact_solve_result
    if jac is None or sol is None:
        return fn, cnt
    counts = jac.contact_env_count.numpy(); gamma = sol.contact_gamma.numpy()
    b0, b1 = jac.contact_env_body0.numpy(), jac.contact_env_body1.numpy()
    for w in range(worlds):
        for j in range(int(counts[w])):
            local = sorted(x - w * 3 for x in (int(b0[w, j]), int(b1[w, j])) if w * 3 <= x < (w + 1) * 3)
            idx = {(0,): 0, (0, 1): 1, (1, 2): 2}.get(tuple(local))
            if idx is not None:
                fn[w, idx] += max(0.0, float(gamma[w, j, 2]) / dt); cnt[w, idx] += 1
    return fn, cnt


def run_cell(c: dict, cell: dict, seed: int, R: int, duration: float, out: Path, device: str, common) -> dict:
    h, mass = cell["thickness_mm"] * 1e-3, cell["mass_per_plate_kg"]
    radius, offset, dt = cell["radius_mm"] * 1e-3, cell["interplate_x_offset_mm"] * 1e-3, cell["dt_ms"] * 1e-3
    path = out / "scenes" / f"{cell['id']}_{c['id'].replace(',', '_').replace('=', '')}.yaml"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(scene(c, h=h, mass=mass, radius=radius, offset=offset, dt=dt, worlds=R), sort_keys=False))
    loaded = load_sap_scene(path, device=wp.get_device(device), naconmax=NCONMAX, strict=True, num_worlds=R)
    model, state, control = loaded.sap_model, loaded.sap_state, loaded.sap_control
    other = model.state()
    pipeline = SapCollisionPipeline(loaded.collision_model, naconmax=NCONMAX, bounds_filter="sphere_obb")
    contacts = pipeline.contacts(); solver = SolverSAP(model, nconmax=NCONMAX)
    xy, yaw = common.plate_perturbations(seed, R)
    q = state.joint_q.numpy().reshape(R, 3, 7)
    q[:, :, :2] += xy
    q[:, :, 3:7] = 0.0
    q[:, :, 5] = np.sin(yaw / 2); q[:, :, 6] = np.cos(yaw / 2)          # xyzw: yaw about +z
    state.joint_q.assign(q.reshape(-1))
    steps, every = round(duration / dt), max(1, round(SAMPLE_S / dt))
    Z, V, W, F, C = [], [], [], [], []
    trunc = 0
    t0 = time.perf_counter()
    for step in range(steps + 1):
        if step % every == 0:
            qq = state.joint_q.numpy().reshape(R, 3, 7); qd = state.joint_qd.numpy().reshape(R, 3, 6)
            Z.append(qq[:, :, 2].copy()); V.append(qd[:, :, 0:3].copy()); W.append(qd[:, :, 3:6].copy())
            f, n = interface_loads(solver, R, dt); F.append(f); C.append(n)
            trunc = max(trunc, int(solver.last_truncated_contact_count or 0))
        if step < steps:
            state.clear_forces()
            solver.collide(pipeline, contacts, state, control, collision_state=state)
            solver.step(state, other, control, contacts, dt)
            state, other = other, state
    pipeline.check_buffers(contacts)
    res = common.plate_summary(np.asarray(Z), np.asarray(V), np.asarray(W), np.asarray(F), np.asarray(C),
                               h=h, mass=mass, g=G, sample_dt=every * dt, truncated=trunc)
    res.update({"cell": cell, "seed": seed, "candidate": c, "wall_s": time.perf_counter() - t0})
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--simexp-scripts", type=Path, required=True, help="dir containing strict_v3_common.py")
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
    R, seed0, dur = man["R"], man["seed0"], man["duration_s"]
    wp.init(); a.out.mkdir(parents=True, exist_ok=True)
    cands = [parse_candidate(t) for t in man["candidate_rule"]["sap_candidates"]]
    selection = []
    for i, c in enumerate(cands):
        r = run_cell(c, man["selection"], common.cell_seed(seed0, "selection", i), R, dur, a.out, a.device, common)
        r["candidate_index"] = i + 1; selection.append(r)
        print("select", i + 1, c["id"], r["pass_count"], r["failed_gate_counts"], f'{r["max_overlap_mm_mean"]:.4f}mm',
              f'{r["load_error_max_mean"]:.4%}', flush=True)
    win = common.plate_choose(selection); frozen = cands[win]
    print("frozen", frozen, flush=True)
    held = []
    for j, cell in enumerate(man["held_out_conditions"]):
        r = run_cell(frozen, cell, common.cell_seed(seed0, "heldout", j), R, dur, a.out, a.device, common); held.append(r)
        print("held", cell["id"], r["pass_count"], r["failed_gate_counts"], f'{r["max_overlap_mm_mean"]:.4f}mm',
              f'{r["load_error_max_mean"]:.4%}', flush=True)
    import sap_warp
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
           "engine": "SAP Warp internal", "sap_source": str(Path(sap_warp.__file__).parent), "warp_version": wp.config.version,
           "device": a.device, "preflight": pf, "load_readout": "sum(contact_gamma.z)/dt (actual dt), fixed SAP build",
           "B1": selection[0], "selection_candidates": selection, "frozen_index": win + 1, "frozen_candidate": frozen,
           "heldout": held, "heldout_pass_total": [sum(r["pass_count"] for r in held), R * len(held)]}
    (a.out / "sap_three_plate_strict_v3.json").write_text(json.dumps(common.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
