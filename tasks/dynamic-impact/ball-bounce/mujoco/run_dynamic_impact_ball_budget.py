#!/usr/bin/env python3
"""Budgeted GAUGE bouncing-ball evaluation with MuJoCo >=3.17.

The public GAUGE ball trajectories provide 30-Hz positions but no released
initial velocity/contact-force channel.  Each measured trial's frame-0 pose is
therefore replayed with zero release velocity (an explicit alignment limit),
and trial-to-trial variation supplies the repeated rollout population.

Stages are deliberately separate: B1 default, B5 mechanism-directed contact
settings, and B100 a disclosed 10x10 direct-solref grid.  Configuration
selection uses odd-numbered trials only; all reported metrics use even trials.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "exp-results/gauge-rigid-reproduction/data/bouncing-ball"
MESH = ROOT / "exp-results/gauge-rigid-reproduction/assets/ball-6.obj"
OUT_DEFAULT = ROOT / "exp-results/dynamic-impact-v1/ball-bounce/mujoco-3.17"
MASS, MU, RADIUS, GROUND_Z, G = 0.121, 0.808698, 0.03, 0.013297, 9.81
FPS, DT = 30.0, 1.0 / 180.0


def load_trials() -> tuple[list[str], list[np.ndarray]]:
    paths = sorted(DATA.glob("*.json"), key=lambda p: int(p.stem))
    names, traces = [], []
    for p in paths:
        raw = json.loads(p.read_text())
        xyz = np.column_stack([raw["ball"][axis] for axis in "xyz"]) / 1000.0
        names.append(p.stem)
        traces.append(xyz)
    if len(traces) != 22:
        raise RuntimeError(f"Expected 22 GAUGE trials, got {len(traces)}")
    return names, traces


def mjcf(cfg: dict) -> str:
    attrs = ""
    numerical_option = "" if cfg["kind"] == "default" else f' integrator="{cfg["integrator"]}" solver="Newton"'
    if cfg["kind"] != "default":
        attrs = f' solref="{-cfg["k"]} {-cfg["b"]}" solimp="0.9 0.9 0.001 0.5 2"'
    return f'''<mujoco model="gauge_ball_budgeted">
      <compiler meshdir="{MESH.parent}"/>
      <option timestep="{DT}" gravity="0 0 {-G}"{numerical_option}/>
      <asset><mesh name="ball" file="{MESH.name}"/></asset>
      <worldbody>
        <geom name="ground" type="plane" pos="0 0 {GROUND_Z}" size="2 2 .1" friction="{MU} 0 0"{attrs}/>
        <body name="ball"><freejoint/><geom type="mesh" mesh="ball" mass="{MASS}" friction="{MU} 0 0"{attrs}/></body>
      </worldbody>
    </mujoco>'''


def dtw(a: np.ndarray, b: np.ndarray) -> float:
    n, m = len(a), len(b)
    cost = np.full((n + 1, m + 1), np.inf)
    length = np.zeros((n + 1, m + 1), int); cost[0, 0] = 0
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            choices = [(cost[i-1,j], length[i-1,j]), (cost[i,j-1], length[i,j-1]),
                       (cost[i-1,j-1], length[i-1,j-1])]
            pc, pl = min(choices, key=lambda x: x[0])
            cost[i,j] = pc + np.linalg.norm(a[i-1] - b[j-1]); length[i,j] = pl + 1
    return float(cost[n, m] / max(length[n, m], 1))


def peaks(z: np.ndarray) -> list[float]:
    # A 3-point maximum is sufficient for the 30-Hz public measurement.
    ids = [i for i in range(1, len(z)-1) if z[i] >= z[i-1] and z[i] > z[i+1]]
    return [float(z[i] - (GROUND_Z + RADIUS)) for i in ids if z[i] > GROUND_Z + RADIUS]


def simulate(model: mujoco.MjModel, initial: np.ndarray, nframes: int) -> dict:
    d = mujoco.MjData(model); d.qpos[:3] = initial; d.qpos[3] = 1.0; mujoco.mj_forward(model, d)
    duration = (nframes - 1) / FPS
    nsteps = int(math.ceil(duration / DT))
    sample_steps = {int(round((i / FPS) / DT)): i for i in range(nframes)}
    sample = np.zeros((nframes, 3)); contact = np.zeros(nsteps + 1, dtype=bool); vz = np.zeros(nsteps + 1)
    normal_force = np.zeros(nsteps + 1)
    for s in range(nsteps + 1):
        if s in sample_steps: sample[sample_steps[s]] = d.xpos[1]
        # ncon includes near-contact candidates.  Duration is defined here by
        # an actual normal support force, not by candidate allocation.
        for ci in range(d.ncon):
            wrench = np.zeros(6)
            mujoco.mj_contactForce(model, d, ci, wrench)
            normal_force[s] += max(float(wrench[0]), 0.0)
        contact[s] = normal_force[s] > 1e-4
        vz[s] = d.qvel[2]
        if s < nsteps: mujoco.mj_step(model, d)
    # First continuous contact episode. e is defined only if separation occurs.
    starts = np.flatnonzero(np.diff(np.r_[False, contact, False].astype(int)) == 1)
    stops = np.flatnonzero(np.diff(np.r_[False, contact, False].astype(int)) == -1) - 1
    tc = e = float("nan")
    if len(starts) and len(stops):
        st = int(starts[0]); sp = int(stops[stops >= st][0])
        tc = (sp - st + 1) * DT
        if st > 0 and sp + 1 < len(vz) and vz[st - 1] < 0:
            e = float(-vz[sp + 1] / vz[st - 1])
    return {"xyz": sample, "e": e, "tc_s": tc, "ncontacts_any": int(contact.any()),
            "max_ncon": int(contact.max()), "peaks": peaks(sample[:,2])}


def ci(values: list[float]) -> dict:
    a = np.asarray(values, float); a = a[np.isfinite(a)]
    if not len(a): return {"mean": None, "std": None, "ci95": None, "n": 0}
    sd = float(a.std(ddof=1)) if len(a)>1 else 0.0
    return {"mean": float(a.mean()), "std": sd, "ci95": float(1.96 * sd / math.sqrt(len(a))), "n": int(len(a))}


def score(cfg: dict, ids: list[int], names: list[str], real: list[np.ndarray], keep: bool = False) -> dict:
    model = mujoco.MjModel.from_xml_string(mjcf(cfg)); runs = []; t0 = time.perf_counter()
    for i in ids:
        sim = simulate(model, real[i][0], len(real[i]))
        ref = real[i]
        rmse = float(np.sqrt(np.mean(np.sum((sim["xyz"] - ref)**2, axis=1))))
        p, q = sim["peaks"][:3], peaks(ref[:,2])[:3]
        apex = float(np.mean([abs(a-b)/max(abs(b),1e-6) for a,b in zip(p,q)])) if p and q else float("nan")
        runs.append({"trial": names[i], "rmse_m": rmse, "dtw_m": dtw(sim["xyz"], ref), "e": sim["e"],
                     "tc_s": sim["tc_s"], "apex_height_rel_error": apex, "sim_apex_heights_m": p,
                     "real_apex_heights_m": q, "max_ncon": sim["max_ncon"]})
    out = {"config": cfg, "rollouts": len(ids), "wall_s": time.perf_counter()-t0,
           "trajectory_rmse_m": ci([r["rmse_m"] for r in runs]), "dtw_m": ci([r["dtw_m"] for r in runs]),
           "effective_restitution": ci([r["e"] for r in runs]), "contact_duration_s": ci([r["tc_s"] for r in runs]),
           "apex_height_decay_error": ci([r["apex_height_rel_error"] for r in runs]),
           "max_active_contacts": ci([r["max_ncon"] for r in runs])}
    if keep: out["per_rollout"] = runs
    # Composite selection objective: equal relative contributions from trajectory and apex energy.
    out["selection_loss"] = out["trajectory_rmse_m"]["mean"] + 0.05 * (out["apex_height_decay_error"]["mean"] or 1.0)
    return out


def configs(stage: str) -> list[dict]:
    if stage == "b1": return [{"id":"default", "kind":"default", "integrator":"Euler"}]
    if stage == "b5":
        # Predeclared mechanisms: increasing normal stiffness shortens tc;
        # increasing direct damping removes rebound energy.
        return configs("b1") + [{"id":f"k{k:g}_b{b:g}", "kind":"direct", "k":k, "b":b, "integrator":"implicitfast"}
                for k,b in [(3e3, 0), (1e4, 5), (1e4, 15), (3e4, 30)]]
    if stage == "b100":
        return [{"id":f"k{k:g}_b{b:g}", "kind":"direct", "k":float(k), "b":float(b), "integrator":"implicitfast"}
                for k in np.geomspace(1e3, 1e5, 10) for b in np.linspace(0, 60, 10)]
    raise ValueError(stage)


def main() -> None:
    ap = argparse.ArgumentParser(); ap.add_argument("--stage", choices=("b1","b5","b100"), required=True)
    ap.add_argument("--output", type=Path, default=OUT_DEFAULT); args = ap.parse_args()
    names, real = load_trials(); train, test = list(range(0,22,2)), list(range(1,22,2))
    grid = configs(args.stage); train_rows = [score(c, train, names, real) for c in grid]
    selected = min(train_rows, key=lambda x: x["selection_loss"])
    frozen = selected["config"]; heldout = score(frozen, test, names, real, keep=True)
    report = {"protocol":"dynamic-impact-v1", "task":"GAUGE public bouncing ball", "engine":"MuJoCo", "engine_version":mujoco.__version__,
              "stage":args.stage, "budget_configurations":len(grid), "split":"odd file IDs train; even file IDs held-out",
              "initialization":"each real frame-0 position; zero release velocity (public-data limitation)",
              "physics":{"dt_s":DT,"mass_kg":MASS,"friction":MU,"ground_z_m":GROUND_Z,"mesh":str(MESH.relative_to(ROOT))},
              "selection_metric":"train mean trajectory RMSE + 0.05*train mean apex-height relative error",
              "train_candidates":train_rows, "frozen_config":frozen, "heldout_metrics":heldout,
              "limitations":{"real_tc":"unavailable at 30 Hz; simulation-only value", "real_e":"apex proxy only; no released high-rate velocity"}}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / f"{args.stage}.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({"stage":args.stage,"frozen":frozen,"heldout":heldout}, indent=2))


if __name__ == "__main__": main()
