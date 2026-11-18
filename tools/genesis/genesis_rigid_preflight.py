#!/usr/bin/env python3
"""Genesis RigidSolver preflight: API/readout probes + one short native-default smoke cell per task runner.

Not a benchmark result and never used for candidate choice (candidates are fixed in genesis_rigid_rerun.json).
Fails loudly when a probe disagrees with the SOP assumptions.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT, MANIFESTS  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import sys
from pathlib import Path

import numpy as np

import genesis_rigid_common as G


def check(name, ok, detail, out):
    out["checks"].append({"name": name, "pass": bool(ok), "detail": detail})
    print(("PASS " if ok else "FAIL ") + name, json.dumps(G.json_safe(detail))[:300], flush=True)


def probes(out):
    gs = G.gs_init(); nat = {"id": "native_default", "native": True, "text": "native default"}
    I = 0.2950002913583124 * 0.0524 ** 2
    cube = G.mjcf(f'<body name="cube" pos="0 0 0.0524"><freejoint/><inertial pos="0 0 0" mass="0.37" diaginertia="{I} {I} {I}"/>'
                  f'<geom type="box" size=".0524 .0524 .0524" friction="0.18 0 0"/></body>')
    floor = G.mjcf('<geom type="plane" size="5 5 .1" friction="0.18 0 0"/>')
    b = G.build({"floor": floor, "cube": cube}, dt=1e-3, cand=nat, n_envs=2)
    p = b.provenance(); e = p["entities"]["cube"]
    check("explicit mass/inertia reach Genesis", abs(e["links_mass"][0] - .37) < 1e-6 and abs(e["links_inertia"][0][0][0] - I) < 1e-8, e, out)
    check("free-body armature is zero", max(map(abs, e["dofs_armature"])) == 0, e["dofs_armature"], out)
    sol = np.asarray(p["geoms_sol_params"]); check("B=1 geom sol_params = Genesis default", np.allclose(sol[:, :2], [[.01, 1]] * len(sol)), sol.tolist(), out)
    for _ in range(500):
        b.scene.step()
    fn, n = b.pair_normal_load("floor", "cube")
    check("resting support load = m g", np.allclose(fn, .37 * 9.81, rtol=2e-3), {"fn": fn.tolist(), "n": n.tolist()}, out)
    f_on, _ = b.force_on("cube", "floor")
    check("force_on sign: floor pushes cube up", np.all(f_on[:, 2] > 0), f_on.tolist(), out)
    b.destroy()
    # angular velocity frame
    free = G.mjcf(f'<body name="c" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="0.37" diaginertia="{I} {I} {I}"/>'
                  '<geom type="box" size=".05 .05 .05" contype="0" conaffinity="0"/></body>')
    b = G.build({"c": free}, dt=1e-3, gravity=(0, 0, 0), cand=nat, n_envs=2); c = b.ents["c"]
    q = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0, 0])
    c.set_qpos(np.tile(np.r_[0, 0, 1, q], (2, 1)).astype(np.float32), zero_velocity=True)
    c.set_dofs_velocity(np.tile([0.3, 0, 0, 0, 0, 1.0], (2, 1)).astype(np.float32))
    for _ in range(100):
        b.scene.step()
    ang = G.np_(c.get_ang())[0]
    check("dofs angular velocity is body-frame (as MuJoCo qvel)", abs(ang[1] + 1) < 1e-3 and abs(ang[2]) < 1e-3, ang.tolist(), out)
    check("dofs linear velocity is world-frame", abs(G.np_(c.get_qpos())[0, 0] - .03) < 1e-4, G.np_(c.get_qpos())[0].tolist(), out)
    for _ in range(100):
        c.apply_links_external_wrench(force=np.tile([[[0.37, 0, 0]]], (2, 1, 1)).astype(np.float32), links_idx_local=[0],
                                      ref=gs.link_ref_frame.link_COM)
        b.scene.step()
    b.destroy()
    # stacked bodies in separate entities do collide (neutral-collision filter only acts within one entity)
    box = lambda i: G.mjcf(f'<body pos="0 0 {.02 + .04 * i}"><freejoint/><geom type="box" size=".02 .02 .02" mass="1" friction="0.3 0 0"/></body>')
    b = G.build({"ground": G.mjcf('<geom type="plane" size="2 2 .1" friction="0.3 0 0"/>'), "b0": box(0), "b1": box(1), "b2": box(2)},
                dt=5e-4, cand=nat, n_envs=2)
    for _ in range(400):
        b.scene.step()
    loads = [b.pair_normal_load(x, y)[0].tolist() for x, y in (("ground", "b0"), ("b0", "b1"), ("b1", "b2"))]
    check("stacked separate entities carry 3 m g / 2 m g / 1 m g", np.allclose([l[0] for l in loads], [3 * 9.81, 2 * 9.81, 9.81], rtol=5e-3), loads, out)
    b.destroy()
    # slide joint: armature 0, force reaches joint, written position/velocity hold
    pad = G.mjcf('<body name="p"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 0 1"/>'
                 '<geom type="box" size=".01 .02 .04" mass="1" contype="0" conaffinity="0"/></body>')
    b = G.build({"p": pad}, dt=1e-3, gravity=(0, 0, 0), cand=nat, n_envs=2); e = b.ents["p"]
    check("slide-joint armature zero (default_armature=None)", np.allclose(G.np_(e.get_dofs_armature()), 0), G.np_(e.get_dofs_armature()).tolist(), out)
    for _ in range(100):
        e.control_dofs_force(np.tile([4.0, 0.0], (2, 1)).astype(np.float32)); b.scene.step()
    check("joint force F gives F/m acceleration", abs(G.np_(e.get_dofs_position())[0, 0] - 0.0202) < 2e-4, G.np_(e.get_dofs_position()).tolist(), out)
    b.destroy()


SMOKE = {
    "three-plate": ("run_threeplate_genesis_rigid", "three_plate_stack_strict_v3p1.json"),
    "three-box": ("run_threebox_genesis_rigid", "three_box_push_strict_v4.json"),
    "simple-pinch": ("run_pinch_genesis_rigid", "simple_pinch_force_controlled_v3.json"),
}


def smoke(out, R=4):
    import importlib
    nat = {"id": "native_default", "native": True, "text": "native default"}
    M = MANIFESTS
    T = importlib.import_module("run_threeplate_genesis_rigid"); man, _ = G.load_manifest(M / "three_plate_stack_strict_v3p1.json")
    r = T.run_cell(nat, man["selection"], 1, R, 0.3)
    check("smoke three-plate runs; order kept", r["failed_gate_counts"]["order"] == 0 and r["failed_gate_counts"]["finite"] == 0,
          {k: r[k] for k in ("failed_gate_counts", "max_overlap_mm_mean", "load_error_max_mean")}, out)
    T = importlib.import_module("run_threebox_genesis_rigid"); man, _ = G.load_manifest(M / "three_box_push_strict_v4.json")
    r = T.run_cell(nat, man["selection"][0], 1, R)
    check("smoke three-box runs; finite; pushed interface in contact", r["finite_count"] == R and r["duty_pre_event_min"] > .9,
          {k: r[k] for k in ("observed_counts", "duty_pre_event_min", "face_disp_final_mm_median")}, out)
    T = importlib.import_module("run_pinch_genesis_rigid"); man, _ = G.load_manifest(M / "simple_pinch_force_controlled_v3.json")
    r = T.run_cell(nat, man["selection"][0], 1, R)
    check("smoke pinch: both pads loaded during gate", r["gate_preload_mean_N"] > 0,
          {k: r[k] for k in ("gate_preload_mean_N", "preload_rel_error_max", "rise_mm_median", "valid_count")}, out)
    import impact_v2_common as ic
    T = importlib.import_module("run_impact_genesis_rigid")
    for task, mf in (("ball", "gauge_ball_bounce_dynamic_impact_v3.json"), ("cube", "contactnets_cube_toss_dynamic_impact_v2.json")):
        man, _ = G.load_manifest(M / mf); prov = []
        data = ic.ball_trials(G.ROOT) if task == "ball" else ic.cube_data(G.ROOT / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si_v2.npz")
        elig = data[2] if task == "cube" else None
        payload = data[1] if task == "ball" else (data[0], data[1])
        ids = ic.trial_ids(task, "selection", elig)[:1]
        sym = None
        if task == "cube":
            from cube_orientation_metrics import cube_symmetric_qangle as sym
        r = ic.run_cell(task, nat, man["selection"], ids, payload, [1], R, T.make_simulate(prov), sym)
        fin = all(x["finite"] for tr in r["trials"] for x in tr["rollouts"])
        check(f"smoke {task} impact runs and is finite (validity is a scored outcome, not checked here)", fin,
              {"loss": r["loss"], "all_valid": r["all_valid"], "pooled": r["pooled"]}, out)
    T = importlib.import_module("run_slope_genesis_rigid"); import slope_v3_common as sc
    man, _ = G.load_manifest(M / "gauge_slope_slider_dynamic_friction_v3.json")
    kin = next(c for c in man["held_out_conditions"] + man["selection"] if c["subtask"] == "task-3")
    r = sc.run_cell(nat, kin, 1, R, G.ROOT, T.make_run_case([]))
    check("smoke slope kinematic board runs; board pose written", r["n_finite"] == R and r["tracking"]["board_written_pose_max_abs_m"] < 1e-4,
          {k: r[k] for k in ("rmse_m", "slide_m", "ref_slide_m", "tracking")}, out)
    T = importlib.import_module("run_mitpush_genesis_rigid"); import mitpush_v3_common as mc
    man, _ = G.load_manifest(M / "mit_push_dynamic_friction_v3.json")
    r = mc.run_cell(nat, man["selection"], 1, R, G.ROOT, T.make_run_trial([]))
    check("smoke MIT Push: pusher contacts object, wrench readable", (r["push_contact_duty"] or 0) > 0 and r["gate"],
          {k: r[k] for k in ("loss_mean", "push_contact_duty", "wrench_error_mean_N", "tracking")}, out)
    T = importlib.import_module("run_pendulum_v2_genesis_rigid"); import pendulum_v2_common as pc
    man, _ = G.load_manifest(M / "pendulum_energy_v2.json")
    r = pc.run_cell(nat, man["selection"], 1, R, T.make_simulate([]))
    check("smoke pendulum v2 finite", r["n_finite"] == R, {k: r[k] for k in ("loss", "final_energy_ratio", "period_rel_error")}, out)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--skip-smoke", action="store_true")
    a = ap.parse_args()
    out = {"kind": "preflight_not_benchmark_result", "versions": None, "checks": []}
    G.gs_init(); out["versions"] = G.versions()
    probes(out)
    if not a.skip_smoke:
        smoke(out)
    out["all_pass"] = all(c["pass"] for c in out["checks"])
    G.save_json(a.out, out)
    print("ALL PASS" if out["all_pass"] else "PREFLIGHT FAILED", flush=True)
    sys.exit(0 if out["all_pass"] else 1)


if __name__ == "__main__":
    main()
