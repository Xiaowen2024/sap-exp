"""Fail-closed declaration contract. The design draft is never executable."""
from __future__ import annotations

from pathlib import Path
import copy
import re
import numpy as np

from .completion import admission
from .io_utils import GateError, digest, verified_file, write_json
from .matrix import CELL_IDS, ENGINES

SCHEMA = 'real-playback-execution-v1'


def _no_placeholders(value, where='manifest'):
    if isinstance(value, dict):
        for k, v in value.items():
            _no_placeholders(v, where+'.'+k)
    elif isinstance(value, list):
        for i, v in enumerate(value):
            _no_placeholders(v, where+f'[{i}]')
    elif isinstance(value, str) and (re.match(r'^P[0123](?:\b|:)', value) or value in ('TBD', 'TODO')):
        raise GateError(f'unresolved placeholder: {where}')


def seal_hash(manifest):
    data = copy.deepcopy(manifest)
    data.pop('declaration_sha256', None)
    return digest(data)


def validate(manifest, input_root=None, require_seal=True):
    if manifest.get('schema') != SCHEMA:
        raise GateError('design draft is not executable; supply a P2 execution manifest')
    if manifest.get('status') != 'declared' or manifest.get('R') != 20:
        raise GateError('requires a declared R=20 manifest')
    _no_placeholders(manifest)
    if require_seal and manifest.get('declaration_sha256') != seal_hash(manifest):
        raise GateError('declaration hash mismatch')
    if tuple(c['id'] for c in manifest['held_out_cells_per_episode']) != CELL_IDS:
        raise GateError('exact 19-cell order required by seed rule')
    execution = manifest['execution']
    if set(execution['candidates']) != set(ENGINES):
        raise GateError('declare SAP, MuJoCo and IPC candidates before any object run')
    native_keys = {'sap': {'ke', 'tau_s', 'sigma'},
                   'mujoco': {'tc_s', 'impratio', 'noslip_iterations'},
                   'ipc': {'d_hat_m', 'eps_velocity_m_s', 'newton_velocity_tol'}}
    for engine, candidates in execution['candidates'].items():
        if not 1 <= len(candidates) <= 5:
            raise GateError('B<=5 includes literal native default')
        if candidates[0] != {'id': 'native_default', 'native': True, 'parameters': {}}:
            raise GateError('B=1 must omit native numerical overrides')
        if len({c['id'] for c in candidates}) != len(candidates):
            raise GateError('duplicate candidate IDs')
        for candidate in candidates[1:]:
            if candidate['native'] or not candidate['parameters'] or not set(candidate['parameters']) <= native_keys[engine]:
                raise GateError(f'{engine}: non-native or undeclared candidate parameter')
            if not all(type(v) in (int, float) and np.isfinite(v) and v >= 0
                       for v in candidate['parameters'].values()):
                raise GateError('candidate parameters must be finite nonnegative numbers')
    if not manifest['audit']['gate_passed'] or not manifest['audit']['official_provenance_verified']:
        raise GateError('P0 official-source/geometric audit has not passed')
    required_audit = ('frame_geometric_check', 'calibration_residual_measured',
                      'native_time_base_confirmed', 'gripper_units_confirmed',
                      'depth_alignment_confirmed', 'tool_offset_confirmed')
    if not all(manifest['audit'].get(k) is True for k in required_audit):
        raise GateError('missing P0 geometric/unit/calibration evidence')
    sel, held = manifest['episodes']['selection'], manifest['episodes']['held_out']
    all_ep = sel+held
    ids = [e['id'] for e in all_ep]
    if len(set(ids)) != len(ids):
        raise GateError('selection and held-out IDs overlap or repeat')
    if sum(e['real_label'] == 'retained' for e in sel) < 1 or sum(e['real_label'] == 'retained' for e in held) < 3:
        raise GateError('need >=1 selection and >=3 held-out real successes')
    if not execution['owner_authorization']:
        raise GateError('P2 owner authorization must cite the user instruction')
    for ep in all_ep:
        if ep['real_label'] not in ('retained', 'not_retained'):
            raise GateError('invalid real label')
        if ep['real_label'] == 'not_retained' and ep.get('real_failure_stage') not in ('acquired', 'carried', 'placed', 'retained'):
            raise GateError('real failures require a video-audited failure stage')
        if not admission(ep)['admitted']:
            raise GateError(f'{ep["id"]}: P2 admission failed')
        if ep['completion_sha256'] != digest(ep['completion']):
            raise GateError('completion hash mismatch')
        scene = ep['scene']
        if scene['plate']['profile'] != 'shallow_rim' or set(scene['plate']['profiles_radius_lower_upper_m']) != {'shallow_rim', 'flat_coupe', 'raised_rim'}:
            raise GateError('declare nominal shallow_rim plus both profile variants')
        if scene['dt_s'] != .0005:
            raise GateError('selection/nominal dt must be 0.5 ms')
        if any(x <= 0 for x in (scene['plate']['mass_kg']-scene['plate']['mass_half_range_kg'],
                                scene['drive']['closing_force_N']-scene['drive']['closing_force_half_range_N'])):
            raise GateError('mass and grip low levels must be positive')
        if not all(0 <= lo <= scene[k] <= hi for k in ('mu_pad_plate', 'mu_plate_environment')
                   for lo, hi in [scene[k+'_low_high']]):
            raise GateError('friction nominal outside declared interval')
        if scene['drive']['base_mass_kg'] != 1.0:
            raise GateError('v1 base mass is declared 1.0 kg')
        events = ep['events']
        if not events['start_s'] < events['lift_off_s'] <= events['acquired_s'] <= events['final_descent_s'] < events['release_s'] < events['withdrawal_s'] < events['end_s']:
            # Acquisition can occur at lift-off; real episodes outside this
            # event order require an explicitly revised task specification.
            raise GateError('invalid frozen real-event order')
        if abs(events['end_s']-events['withdrawal_s']-2.) > 1e-8:
            raise GateError('requires two-second settle after withdrawal')
        if not ep['source_files']:
            raise GateError('no hashed real input files')
        if input_root is not None:
            for spec in ep['source_files']:
                verified_file(input_root, spec)
            verified_file(input_root, ep['playback_npz'])
    return True


def declare(manifest, output, input_root):
    result = copy.deepcopy(manifest)
    result['schema'], result['status'] = SCHEMA, 'declared'
    result['declaration_sha256'] = seal_hash(result)
    validate(result, input_root)
    output = Path(output)
    if output.exists():
        raise GateError('declarations are immutable; choose a new revision path')
    write_json(output, result)
    return result
