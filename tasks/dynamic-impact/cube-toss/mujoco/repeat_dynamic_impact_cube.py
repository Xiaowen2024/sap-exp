#!/usr/bin/env python3
"""Repeat-certification and local-perturbation evaluation for cube toss.

This deliberately evaluates only an already-frozen B-stage configuration.
It never feeds perturbation results back into parameter selection.
"""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np

from run_dynamic_impact_cube_budget import (
    HALF, SRC, ci, dtw, make_model, qangle,qangle_sym, simulate, upward_face,
)

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / 'exp-results/dynamic-impact-v1/cube-toss/mujoco-3.17'

METRICS = {
    'trajectory_position_rmse_m': 'position_rmse_m',
    'trajectory_position_dtw_m': 'position_dtw_m',
    'linear_velocity_rmse_m_s': 'linear_velocity_rmse_m_s',
    'final_position_error_m': 'final_position_error_m',
    'final_orientation_error_rad': 'final_orientation_error_rad',
    'final_orientation_sym_rad': 'final_orientation_sym_rad',
    'final_up_face_agreement': 'final_up_face_agreement',
    'effective_restitution_com_proxy': 'e_com_proxy',
    'first_contact_duration_s': 'contact_duration_s',
    'postimpact_linear_speed_error_m_s': 'postimpact_linear_speed_error_m_s',
    'postimpact_angular_speed_error_rad_s': 'postimpact_angular_speed_error_rad_s',
}


def quaternion_from_rotvec(v):
    a = float(np.linalg.norm(v))
    if a < 1e-15:
        return np.array([1., 0., 0., 0.])
    return np.r_[math.cos(a / 2), math.sin(a / 2) * v / a]


def qmul(a, b):
    return np.array([
        a[0]*b[0] - np.dot(a[1:], b[1:]),
        a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
        a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
        a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
    ])


def perturb(x, rng):
    """Stress-v1 setup uncertainty; not claimed to be a sensor-noise model."""
    y = np.array(x, dtype=float, copy=True)
    y[:3] += rng.normal(0., 5e-4, 3)          # 0.5 mm position
    y[3:7] = qmul(quaternion_from_rotvec(rng.normal(0., np.deg2rad(.5), 3)), y[3:7])
    y[3:7] /= np.linalg.norm(y[3:7])
    y[7:10] += rng.normal(0., .01, 3)         # 1 cm/s linear velocity
    y[10:13] += rng.normal(0., .05, 3)        # 0.05 rad/s angular velocity
    return y


def one(model, initial, real):
    sim, e, tc = simulate(model, initial[None, :], len(real))
    return {
        'position_rmse_m': float(np.sqrt(np.mean(np.sum((sim[:, :3] - real[:, :3])**2, axis=1)))),
        'position_dtw_m': dtw(sim[:, :3], real[:, :3]),
        'linear_velocity_rmse_m_s': float(np.sqrt(np.mean(np.sum((sim[:, 7:10] - real[:, 7:10])**2, axis=1)))),
        'final_position_error_m': float(np.linalg.norm(sim[-1, :3] - real[-1, :3])),
        'final_orientation_error_rad': qangle(sim[-1, 3:7], real[-1, 3:7]),'final_orientation_sym_rad':qangle_sym(sim[-1, 3:7], real[-1,3:7]),
        'final_up_face_agreement': float(upward_face(sim[-1, 3:7]) == upward_face(real[-1, 3:7])),
        'e_com_proxy': e,
        'contact_duration_s': tc,
        'postimpact_linear_speed_error_m_s': float(abs(np.linalg.norm(sim[-1, 7:10])-np.linalg.norm(real[-1, 7:10]))),
        'postimpact_angular_speed_error_rad_s': float(abs(np.linalg.norm(sim[-1, 10:13])-np.linalg.norm(real[-1, 10:13]))),
        '_state': sim,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stage', choices=('b1', 'b5', 'b100'), required=True)
    ap.add_argument('--replicates', type=int, default=20)
    ap.add_argument('--seed', type=int, default=20261024)
    ap.add_argument('--results-dir', type=Path, default=RESULTS)
    args = ap.parse_args()
    source = json.loads((args.results_dir / f'{args.stage}.json').read_text())
    frozen = source['frozen_config']
    data = np.load(SRC); states, lengths = data['state'], data['lengths']
    ids = [x['id'] for x in source['heldout_metrics']['per_rollout']]
    model = make_model(frozen)
    per_sample = []
    max_exact_abs = 0.
    for sid in ids:
        real = states[sid, :int(lengths[sid])]
        baseline = one(model, real[0], real)
        # Required exact deterministic-repeat certificate, R=20 by default.
        for _ in range(args.replicates - 1):
            replay = one(model, real[0], real)
            max_exact_abs = max(max_exact_abs, float(np.max(np.abs(replay['_state'] - baseline['_state']))))
        rng = np.random.default_rng(args.seed + int(sid))
        rows = [one(model, perturb(real[0], rng), real) for _ in range(args.replicates)]
        summary = {'id': int(sid)}
        for output_key, row_key in METRICS.items():
            vals = [r[row_key] for r in rows]
            summary[output_key] = ci(vals)
        per_sample.append(summary)
    aggregate = {}
    for key in METRICS:
        means = [s[key]['mean'] for s in per_sample]
        within = [s[key]['std'] for s in per_sample]
        aggregate[key] = {
            'across_sample_mean_of_perturbed_means': ci(means),
            'pooled_within_sample_sd_mean': float(np.mean(within)),
            'pooled_within_sample_sd_max': float(np.max(within)),
        }
    out = {
        'protocol': 'dynamic-impact-v1 local-repeats v1',
        'task': 'ContactNets real cube toss',
        'engine': 'MuJoCo', 'stage': args.stage,
        'frozen_config': frozen, 'heldout_samples': len(ids),
        'exact_deterministic_repeats_per_sample': args.replicates,
        'exact_deterministic_replay_max_abs_state_difference': max_exact_abs,
        'local_perturbation_repeats_per_sample': args.replicates,
        'random_seed_rule': f'{args.seed} + released sample id',
        'perturbation': {
            'label': 'stress-v1; predeclared numerical-local-robustness perturbation, not measured sensor noise',
            'initial_position_sigma_m': 5e-4,
            'initial_orientation_rotvec_sigma_rad': float(np.deg2rad(.5)),
            'initial_linear_velocity_sigma_m_s': .01,
            'initial_angular_velocity_sigma_rad_s': .05,
        },
        'metrics': aggregate, 'per_sample': per_sample,
    }
    dst = args.results_dir / f'{args.stage}_repeats_r{args.replicates}.json'
    dst.write_text(json.dumps(out, indent=2) + '\n')
    print(dst)
    print(json.dumps(out['metrics'], indent=2))


if __name__ == '__main__':
    main()
