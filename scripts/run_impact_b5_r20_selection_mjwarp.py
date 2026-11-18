"""Strict B=5, R=20-per-sample impact selection on MuJoCo Warp.

The physical models, candidate grids, perturbations, splits, and selection
losses are the same as run_impact_b5_r20_selection.py. Each sample's 20 small
perturbations are simulated as one GPU batch; candidate selection never sees
the held-out samples. This runner is deliberately separate from the older
one-trajectory-per-candidate B=5 artifacts.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np

from run_dynamic_impact_ball_budget import configs as ball_configs, dtw as ball_dtw, load_trials, peaks
from run_dynamic_impact_ball_budget_mjwarp import simulate_batch as ball_batch
from run_dynamic_impact_cube_budget import HALF, SRC as CUBE_SRC, configs as cube_configs, dtw as cube_dtw, qangle
from cube_orientation_metrics import cube_symmetric_qangle
from run_dynamic_impact_cube_budget_mjwarp import simulate_batch as cube_batch
from run_dynamic_impact_cassie_budget import ROOT, SRC as CASSIE_SRC, XML, configs as cassie_configs
from run_dynamic_impact_cassie_budget_mjwarp import simulate as cassie_batch
from repeat_dynamic_impact_cube import perturb as cube_perturb
from repeat_dynamic_impact_cassie import perturb as cassie_perturb
from run_impact_b5_r20_selection import summary


def finite_record(**values: object) -> dict[str, float]:
    return {key: float(value) for key, value in values.items()
            if isinstance(value, (int, float, np.number)) and math.isfinite(float(value))}


def ball_score(cfg: dict, ids: list[int], names: list[str], real: list[np.ndarray], repeats: int) -> dict:
    per_sample = []
    losses = []
    overflow_count = 0
    for i in ids:
        rng = np.random.default_rng(20261027 + i)
        initials = np.asarray([real[i][0] + rng.normal(0, 5e-4, 3) for _ in range(repeats)])
        sims = ball_batch(cfg, initials, [len(real[i])] * repeats)
        reference_peaks = peaks(real[i][:, 2])[:3]
        rows = []
        for sim in sims:
            simulated_peaks = sim["peaks"][:3]
            apex = (float(np.mean([abs(a - b) / max(abs(b), 1e-6)
                                   for a, b in zip(simulated_peaks, reference_peaks)]))
                    if simulated_peaks and reference_peaks else math.nan)
            rows.append(finite_record(
                rmse_m=np.sqrt(np.mean(np.sum((sim["xyz"] - real[i]) ** 2, axis=1))),
                dtw_m=ball_dtw(sim["xyz"], real[i]), e=sim["e"],
                tc_s=sim["tc_s"], apex=apex))
            overflow_count += int(sim["overflow"])
        metrics = summary(rows)
        per_sample.append({"trial": names[i], "metrics": metrics, "rollouts": rows})
        if "apex" in metrics:
            losses.append(metrics["rmse_m"]["mean"] + 0.05 * metrics["apex"]["mean"])
    loss = float(np.mean(losses)) if len(losses) == len(ids) and overflow_count == 0 else math.inf
    return {"config": cfg, "per_sample": per_sample, "overflow_worlds": overflow_count,
            "selection_loss": loss}


def cube_score(cfg: dict, ids: list[int], states: np.ndarray,
               lengths: np.ndarray, repeats: int, orientation_metric: str = "raw") -> dict:
    # raw = historical quaternion angle; sym = cube-symmetry-reduced angle (dynamic-impact-v1-sym).
    orient_key = "final_orientation_error_rad" if orientation_metric == "raw" else "final_orientation_sym_rad"
    per_sample = []
    losses = []
    overflow_count = 0
    for i in ids:
        real = states[i, :int(lengths[i])]
        rng = np.random.default_rng(20261027 + i)
        initials = np.asarray([cube_perturb(real[0], rng) for _ in range(repeats)])
        sims = cube_batch(cfg, initials, [len(real)] * repeats)
        rows = []
        for simulated, e, tc, overflow in sims:
            rows.append(finite_record(
                position_rmse_m=np.sqrt(np.mean(np.sum((simulated[:, :3] - real[:, :3]) ** 2, axis=1))),
                position_dtw_m=cube_dtw(simulated[:, :3], real[:, :3]),
                linear_velocity_rmse_m_s=np.sqrt(np.mean(np.sum((simulated[:, 7:10] - real[:, 7:10]) ** 2, axis=1))),
                final_position_error_m=np.linalg.norm(simulated[-1, :3] - real[-1, :3]),
                final_orientation_error_rad=qangle(simulated[-1, 3:7], real[-1, 3:7]),
                final_orientation_sym_rad=cube_symmetric_qangle(simulated[-1, 3:7], real[-1, 3:7]),
                final_quat_sim_w=simulated[-1, 3], final_quat_sim_x=simulated[-1, 4],
                final_quat_sim_y=simulated[-1, 5], final_quat_sim_z=simulated[-1, 6],
                e_com_proxy=e, contact_duration_s=tc,
                postimpact_linear_speed_error_m_s=abs(np.linalg.norm(simulated[-1, 7:10]) - np.linalg.norm(real[-1, 7:10])),
                postimpact_angular_speed_error_rad_s=abs(np.linalg.norm(simulated[-1, 10:13]) - np.linalg.norm(real[-1, 10:13]))))
            overflow_count += int(overflow)
        metrics = summary(rows)
        per_sample.append({"trial": int(i), "metrics": metrics, "rollouts": rows})
        if "position_rmse_m" in metrics and orient_key in metrics:
            losses.append(metrics["position_rmse_m"]["mean"]
                          + 0.05 * metrics[orient_key]["mean"])
    loss = float(np.mean(losses)) if len(losses) == len(ids) and overflow_count == 0 else math.inf
    return {"config": cfg, "per_sample": per_sample, "overflow_worlds": overflow_count,
            "selection_loss": loss}


def cassie_score(cfg: dict, ids: list[int], data: np.lib.npyio.NpzFile,
                 repeats: int) -> dict:
    per_sample = []
    losses = []
    overflow_count = 0
    weights = np.ones(45) * 10
    weights[23:26] = 5
    weights[26:29] = 100
    for i in ids:
        trial_id = int(data["ids"][i])
        real = np.load(ROOT / f"external/cassie-impact-dataset/jumping_data/data/x_{trial_id:02d}.npy")
        rng = np.random.default_rng(20261027 + trial_id)
        initial = [cassie_perturb(data["qpos"][i], data["qvel"][i], rng)
                   for _ in range(repeats)]
        batch = {"qpos": np.asarray([x[0] for x in initial]),
                 "qvel": np.asarray([x[1] for x in initial]),
                 "ctrl": np.repeat(data["ctrl"][i:i+1], repeats, axis=0)}
        simulated, touch, overflow = cassie_batch(cfg, batch, list(range(repeats)))
        rows = []
        for r in range(repeats):
            state = simulated[r].copy()
            if np.dot(state[0, :4], real[0, :4]) < 0:
                state[:, :4] *= -1
            error = state - real
            rows.append(finite_record(
                weighted_50ms_state_error=np.mean(np.sum(weights * error * error, axis=1)),
                joint_position_rmse_rad_m=np.sqrt(np.mean(np.sum(error[:, 4:23] ** 2, axis=1))),
                velocity_rmse=np.sqrt(np.mean(np.sum(error[:, 23:] ** 2, axis=1))),
                contact_duty=touch[r].mean(),
                foot_contact_switches=np.count_nonzero(np.diff(touch[r].astype(int))),
                final_base_position_error_m=np.linalg.norm(error[-1, 4:7])))
            overflow_count += int(overflow[r])
        metrics = summary(rows)
        per_sample.append({"trial": trial_id, "metrics": metrics, "rollouts": rows})
        if "weighted_50ms_state_error" in metrics:
            losses.append(metrics["weighted_50ms_state_error"]["mean"])
    loss = float(np.mean(losses)) if len(losses) == len(ids) and overflow_count == 0 else math.inf
    return {"config": cfg, "per_sample": per_sample, "overflow_worlds": overflow_count,
            "selection_loss": loss}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", choices=("ball", "cube", "cassie"), required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--R", type=int, default=20)
    parser.add_argument("--orientation-metric", choices=("raw", "sym"), default="raw",
                        help="cube only: selection-loss orientation term (sym = declared revision dynamic-impact-v1-sym)")
    args = parser.parse_args()
    if args.R != 20:
        raise ValueError("Unified B=5 requires R=20 per sample per candidate")

    if args.task == "ball":
        names, real = load_trials()
        train, held = list(range(0, len(names), 2)), list(range(1, len(names), 2))
        grid = ball_configs("b5")
        scorer = lambda cfg, ids: ball_score(cfg, ids, names, real, args.R)
    elif args.task == "cube":
        data = np.load(CUBE_SRC)
        states, lengths = data["state"], data["lengths"]
        eligible = [i for i, length in enumerate(lengths)
                    if states[i, 0, 2] > 2 * HALF and np.nanmin(states[i, :length, 2]) < .09]
        train, held = eligible[::2][:32], eligible[1::2][:32]
        grid = cube_configs("b5")
        scorer = lambda cfg, ids: cube_score(cfg, ids, states, lengths, args.R, args.orientation_metric)
    else:
        data = np.load(CASSIE_SRC)
        train = list(range(0, len(data["ids"]), 2))
        held = list(range(1, len(data["ids"]), 2))
        grid = cassie_configs("b5")
        scorer = lambda cfg, ids: cassie_score(cfg, ids, data, args.R)
    if len(grid) != 5 or grid[0]["kind"] != "default":
        raise RuntimeError("B=5 must include literal native default plus four guided candidates")

    started = time.perf_counter()
    selection = []
    for index, cfg in enumerate(grid, 1):
        result = scorer(cfg, train)
        selection.append({"candidate_index": index, **result})
        print(f"{args.task} MJWarp candidate {index}/5: loss={result['selection_loss']:.6g}", flush=True)
    valid = [row for row in selection if math.isfinite(row["selection_loss"])]
    winner = min(valid, key=lambda row: (row["selection_loss"], row["candidate_index"])) if valid else None
    heldout = scorer(winner["config"], held) if winner else None
    protocol = "dynamic-impact-v1-sym" if (args.task == "cube" and args.orientation_metric == "sym") else "dynamic-impact-v1"
    report = {"protocol": protocol, "task": args.task, "orientation_metric": args.orientation_metric if args.task == "cube" else None,
              "engine": "MuJoCo Warp", "version": mujoco.__version__,
              "B": 5, "R_per_sample_per_candidate": args.R,
              "selection_sample_indices": train, "heldout_sample_indices": held,
              "selection": selection,
              "selected_candidate_index": winner["candidate_index"] if winner else None,
              "frozen_config": winner["config"] if winner else None,
              "heldout": heldout, "wall_s": time.perf_counter() - started}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"task": args.task, "winner": report["frozen_config"],
                      "heldout_loss": heldout["selection_loss"] if heldout else None}, indent=2))


if __name__ == "__main__":
    main()
