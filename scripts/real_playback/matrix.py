"""Frozen cells, identical perturbations, budget selection and safe verdicts."""
from __future__ import annotations

from collections import Counter
import copy
import numpy as np

from .io_utils import GateError, digest

ENGINES = ('sap', 'mujoco', 'ipc')
CELL_IDS = ('nominal', 'dt_small', 'dt_large', 'mass_low', 'mass_high',
            'mu_pad_low', 'mu_pad_high', 'mu_env_low', 'mu_env_high',
            'grip_low', 'grip_high', 'coupling_soft', 'coupling_stiff',
            'profile_flat_coupe', 'profile_raised_rim', 'plate_pose_minus',
            'plate_pose_plus', 'rack_pose_minus', 'rack_pose_plus')


def seed_for(manifest, split, episode_index, cell_index):
    if split == 'selection':
        return manifest['seed0'] + episode_index
    return manifest['seed0'] + 1000 * (episode_index + 1) + cell_index


def perturbations(seed, R=20):
    rng = np.random.default_rng(seed)
    return np.column_stack((rng.uniform(-.0005, .0005, (R, 2)),
                            rng.uniform(-np.deg2rad(.5), np.deg2rad(.5), R)))


def apply_cell(scene, cell_id):
    """Change one axis only. All SI values and direction vectors are frozen."""
    s = copy.deepcopy(scene)
    s['dt_s'] = .0005
    s['goal']['rack_position_m'] = s['rack']['position_m']
    s['goal']['rack_quaternion_xyzw'] = s['rack']['quaternion_xyzw']
    if cell_id == 'nominal':
        return s
    if cell_id in ('dt_small', 'dt_large'):
        s['dt_s'] = .00025 if cell_id == 'dt_small' else .001
    elif cell_id.startswith('mass_'):
        sign = -1 if cell_id.endswith('low') else 1
        s['plate']['mass_kg'] += sign * s['plate']['mass_half_range_kg']
    elif cell_id.startswith(('mu_pad_', 'mu_env_')):
        k = 'mu_pad_plate' if cell_id.startswith('mu_pad') else 'mu_plate_environment'
        s[k] = s[k + '_low_high'][0 if cell_id.endswith('low') else 1]
    elif cell_id.startswith('grip_'):
        sign = -1 if cell_id.endswith('low') else 1
        s['drive']['closing_force_N'] += sign * s['drive']['closing_force_half_range_N']
    elif cell_id.startswith('coupling_'):
        factor = 1 / 3 if cell_id.endswith('soft') else 3
        for key in ('K_t_N_m', 'K_r_Nm_rad'):
            s['drive'][key] *= factor
    elif cell_id.startswith('profile_'):
        s['plate']['profile'] = cell_id.removeprefix('profile_')
    elif cell_id.startswith(('plate_pose_', 'rack_pose_')):
        obj = s['plate'] if cell_id.startswith('plate') else s['rack']
        sign = -1 if cell_id.endswith('minus') else 1
        offset = sign * np.asarray(obj['pose_edge_xyz_yaw'])
        obj['position_m'] = (np.asarray(obj['position_m']) + offset[:3]).tolist()
        from scipy.spatial.transform import Rotation
        obj['quaternion_xyzw'] = (Rotation.from_rotvec([0, 0, offset[3]]) *
                                  Rotation.from_quat(obj['quaternion_xyzw'])).as_quat().tolist()
    else:
        raise GateError(f'undeclared cell: {cell_id}')
    s['goal']['rack_position_m'] = s['rack']['position_m']
    s['goal']['rack_quaternion_xyzw'] = s['rack']['quaternion_xyzw']
    return s


def jobs(manifest, split, budget, winners=None):
    if split not in ('selection', 'held_out') or budget not in (1, 5):
        raise GateError('invalid split or budget')
    rows = []
    for ei, ep in enumerate(manifest['episodes'][split]):
        cells = ('nominal',) if split == 'selection' else CELL_IDS
        for ci, cell in enumerate(cells):
            seed = seed_for(manifest, split, ei, ci)
            noise = perturbations(seed, manifest['R'])
            for engine in ENGINES:
                candidates = manifest['execution']['candidates'][engine]
                if split == 'selection':
                    candidates = candidates[:budget]
                else:
                    candidate = 0 if budget == 1 else winners[engine]
                    candidates = [candidates[candidate]]
                for candidate in candidates:
                    row = {'episode_id': ep['id'], 'split': split, 'budget': budget,
                           'engine': engine, 'candidate': candidate, 'cell_id': cell,
                           'seed': seed, 'perturbations_xy_yaw': noise.tolist(),
                           'perturbations_sha256': digest(noise.tolist()), 'R': manifest['R']}
                    row['job_id'] = digest(row)
                    rows.append(row)
    return rows


def cell_result(rows, expected_R=20):
    """Missing/duplicate/invalid rollouts never become a scientific failure."""
    ids = [r['rollout'] for r in rows]
    if len(rows) != expected_R or sorted(ids) != list(range(expected_R)):
        return {'status': 'not run', 'correct': None, 'denominator': expected_R}
    n = sum(bool(r['valid'] and r['outcome'] == r['real_label']) for r in rows)
    return {'status': 'pass' if n >= 18 else 'fail' if n <= 2 else 'mixed',
            'correct': n, 'denominator': expected_R,
            'invalid': sum(not r['valid'] for r in rows),
            'retained': sum(r['outcome'] == 'retained' for r in rows),
            'flags': dict(Counter(flag for r in rows for flag in r['flags']))}


def episode_result(cells):
    if set(cells) != set(CELL_IDS) or any(c['status'] == 'not run' for c in cells.values()):
        return 'not run'
    states = [c['status'] for c in cells.values()]
    return 'reproduces' if all(s == 'pass' for s in states) else (
        'fails' if all(s == 'fail' for s in states) else 'sensitive')


def permitted_claim(a, b, real_label):
    if 'not run' in (a, b):
        return 'not run; no engine conclusion'
    if a == b == 'fails':
        return 'completion suspect; review real-data completion as a new revision'
    if 'sensitive' in (a, b):
        return 'sensitive; no ranking'
    if a == b == 'reproduces':
        return 'neither engine is contradicted by this episode'
    if {a, b} == {'reproduces', 'fails'}:
        winner = 'A' if a == 'reproduces' else 'B'
        if real_label == 'retained':
            return f'{winner} reproduces real success; the other fails to reproduce it'
        return f'{winner} reproduces real task failure; the other fails to reproduce it'
    raise GateError('invalid verdict pair')


def select_candidate(candidate_rows, selection_episode_ids):
    """Each candidate must cover each selection episode, with R=20 and health.

    Return None when no candidate meets >=18 valid+correct in *each* episode.
    Diagnostic force/width/contact-time columns are intentionally ignored.
    """
    viable = []
    for c in candidate_rows:
        if set(c['episodes']) != set(selection_episode_ids) or not c['preflight_passed']:
            continue
        summaries = [cell_result(r) for r in c['episodes'].values()]
        if not all(r['status'] == 'pass' for r in summaries):
            continue
        flagged = sum(bool(r['flags']) for rs in c['episodes'].values() for r in rs)
        viable.append((-sum(s['correct'] for s in summaries), flagged, c['wall_s'], c['candidate_index']))
    return min(viable)[-1] if viable else None
