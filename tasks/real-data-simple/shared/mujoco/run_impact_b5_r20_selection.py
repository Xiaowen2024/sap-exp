"""Five-config, R=20-per-training-sample impact selection (MuJoCo CPU).

This is the strict unified-B5 successor to the earlier B5 impact selection,
which compared one trajectory per real training sample.  It uses the same
small perturbation law and metrics as the existing held-out repeat scripts.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np

from run_dynamic_impact_ball_budget import configs as ball_configs, load_trials, mjcf
from run_dynamic_impact_cube_budget import configs as cube_configs, make_model, SRC as CUBE_SRC, HALF
from repeat_dynamic_impact_ball import one as ball_one
from repeat_dynamic_impact_cube import one as cube_one, perturb as cube_perturb
try:  # Cassie landing runners are not part of this repository yet; only --task cassie needs them.
    from run_dynamic_impact_cassie_budget import configs as cassie_configs, model as cassie_model, SRC as CASSIE_SRC
    from repeat_dynamic_impact_cassie import one as cassie_one, perturb as cassie_perturb
except ImportError:
    cassie_configs = None
ROOT = REPO_ROOT


def summary(records: list[dict]) -> dict:
    keys = sorted(set().union(*(r.keys() for r in records)))
    out: dict = {"n_rollouts": len(records)}
    for key in keys:
        vals = [float(r[key]) for r in records if isinstance(r.get(key), (int, float, np.number))]
        valid = np.asarray([v for v in vals if math.isfinite(v)], dtype=float)
        if len(valid):
            out[key] = {"mean": float(valid.mean()),
                        "sd": float(valid.std(ddof=1)) if len(valid) > 1 else 0.0,
                        "valid_n": len(valid)}
    return out


def metric_record(one_result: dict) -> dict:
    return {k: float(v) for k, v in one_result.items()
            if isinstance(v, (int, float, np.number))}


def score_ball(cfg: dict, ids: list[int], names: list[str], real: list[np.ndarray], R: int) -> dict:
    model = mujoco.MjModel.from_xml_string(mjcf(cfg))
    per_sample = []
    for i in ids:
        rng = np.random.default_rng(20261027 + i)
        rows = [metric_record(ball_one(model, real[i][0] + rng.normal(0, 5e-4, 3), real[i]))
                for _ in range(R)]
        per_sample.append({"trial": names[i], "metrics": summary(rows), "rollouts": rows})
    losses = [s["metrics"]["rmse_m"]["mean"] + 0.05 * s["metrics"]["apex"]["mean"]
              for s in per_sample if "apex" in s["metrics"]]
    return {"config": cfg, "per_sample": per_sample,
            "selection_loss": float(np.mean(losses)) if len(losses) == len(ids) else math.inf}


def score_cube(cfg: dict, ids: list[int], states: np.ndarray, lengths: np.ndarray, R: int) -> dict:
    model = make_model(cfg)
    per_sample = []
    for i in ids:
        rng = np.random.default_rng(20261027 + i)
        real = states[i, :int(lengths[i])]
        rows = [metric_record(cube_one(model, cube_perturb(real[0], rng), real))
                for _ in range(R)]
        per_sample.append({"trial": int(i), "metrics": summary(rows), "rollouts": rows})
    losses = [s["metrics"]["position_rmse_m"]["mean"]
              + 0.05 * s["metrics"]["final_orientation_error_rad"]["mean"]
              for s in per_sample]
    return {"config": cfg, "per_sample": per_sample,
            "selection_loss": float(np.mean(losses))}


def score_cassie(cfg: dict, ids: list[int], data: np.lib.npyio.NpzFile, R: int) -> dict:
    model = cassie_model(cfg)
    per_sample = []
    for i in ids:
        trial_id = int(data["ids"][i])
        real = np.load(ROOT / f"external/cassie-impact-dataset/jumping_data/data/x_{trial_id:02d}.npy")
        rng = np.random.default_rng(20261027 + trial_id)
        rows = []
        for _ in range(R):
            q, v = cassie_perturb(data["qpos"][i], data["qvel"][i], rng)
            rows.append(metric_record(cassie_one(model, q, v, data["ctrl"][i], data["times"][i], real)))
        per_sample.append({"trial": trial_id, "metrics": summary(rows), "rollouts": rows})
    losses = [s["metrics"]["weighted_50ms_state_error"]["mean"] for s in per_sample]
    return {"config": cfg, "per_sample": per_sample,
            "selection_loss": float(np.mean(losses))}


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--task", choices=("ball", "cube", "cassie"), required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--R", type=int, default=20)
    args = p.parse_args()
    if args.task == "cassie" and cassie_configs is None:
        raise SystemExit("--task cassie needs the Cassie landing runners (run_dynamic_impact_cassie_budget.py)")
    if args.R != 20:
        raise ValueError("Unified B=5 selection requires R=20")

    if args.task == "ball":
        names, real = load_trials()
        train, held = list(range(0, len(names), 2)), list(range(1, len(names), 2))
        grid = ball_configs("b5")
        scorer = lambda cfg, ids: score_ball(cfg, ids, names, real, args.R)
    elif args.task == "cube":
        data = np.load(CUBE_SRC)
        states, lengths = data["state"], data["lengths"]
        eligible = [i for i, L in enumerate(lengths)
                    if states[i, 0, 2] > 2 * HALF and np.nanmin(states[i, :L, 2]) < .09]
        train, held = eligible[::2][:32], eligible[1::2][:32]
        grid = cube_configs("b5")
        scorer = lambda cfg, ids: score_cube(cfg, ids, states, lengths, args.R)
    else:
        data = np.load(CASSIE_SRC)
        train = list(range(0, len(data["ids"]), 2))
        held = list(range(1, len(data["ids"]), 2))
        grid = cassie_configs("b5")
        scorer = lambda cfg, ids: score_cassie(cfg, ids, data, args.R)
    if len(grid) != 5 or grid[0]["kind"] != "default":
        raise RuntimeError("B=5 must include literal default followed by four guided candidates")

    t0 = time.perf_counter()
    selection = []
    for i, cfg in enumerate(grid, 1):
        result = scorer(cfg, train)
        selection.append({"candidate_index": i, **result})
        print(f"{args.task} candidate {i}/5: loss={result['selection_loss']:.6g}", flush=True)
    valid = [s for s in selection if math.isfinite(s["selection_loss"])]
    if not valid:
        winner = None
        heldout = None
    else:
        winner = min(valid, key=lambda s: (s["selection_loss"], s["candidate_index"]))
        heldout = scorer(winner["config"], held)
    report = {"protocol": "dynamic-impact-v1", "task": args.task,
              "engine": "MuJoCo CPU", "version": mujoco.__version__,
              "B": 5, "R_per_sample_per_candidate": args.R,
              "selection_sample_indices": train, "heldout_sample_indices": held,
              "selection": selection,
              "selected_candidate_index": winner["candidate_index"] if winner else None,
              "frozen_config": winner["config"] if winner else None,
              "heldout": heldout, "wall_s": time.perf_counter() - t0}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"task": args.task, "winner": report["frozen_config"],
                      "heldout_loss": heldout["selection_loss"] if heldout else None}, indent=2))


if __name__ == "__main__":
    main()
