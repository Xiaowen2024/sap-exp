"""Engine-independent stage and health scoring from native-step telemetry.

All overlap arrays are positive penetration in meters, measured by the same
geometry observer. Missing health telemetry is an error, never a zero.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from .io_utils import GateError

INTERFACES = ('pad_plate', 'plate_rack', 'plate_table')
STAGES = ('acquired', 'carried', 'placed', 'retained')


def retained_predicate(state, goal):
    """The same predicate is used for P2 real-frame admission and simulation.

    Real-state contact and speed must be supported by consecutive real RGB-D
    frames; a successful rating alone is not evidence for those fields.
    """
    p = np.asarray(state['plate_position_m'])
    R = Rotation.from_quat(goal['rack_quaternion_xyzw']).as_matrix()
    local = R.T @ (p - np.asarray(goal['rack_position_m']))
    lo, hi = np.asarray(goal['footprint_min_xy_m']), np.asarray(goal['footprint_max_xy_m'])
    inside = bool(np.all(local[:2] >= lo) and np.all(local[:2] <= hi))
    normal = Rotation.from_quat(state['plate_quaternion_xyzw']).apply([0, 0, 1])
    axis = R @ np.asarray(goal['plate_normal_axis_rack'])
    angle = np.arccos(np.clip(abs(np.dot(normal, axis)), 0, 1))
    return bool(state['rack_contact'] and not any(state['pad_contacts']) and inside and
                angle <= goal['normal_cone_half_angle_rad'] and
                state['tail_max_linear_speed_m_s'] <= .01 and
                state['tail_max_angular_speed_rad_s'] <= .05)


def evaluate(trace, scene, episode):
    t = np.asarray(trace['t_s'], float)
    if len(t) < 2 or not np.all(np.diff(t) > 0):
        raise GateError('scoring requires monotone native-step telemetry')
    expected_step = float(scene['dt_s'])
    if np.max(np.diff(t)) > expected_step * 1.001:
        raise GateError('subsampled health trace; transient contacts/flags could be missed')
    p, q = np.asarray(trace['plate_pose_xyzw'], float)[:, :3], np.asarray(trace['plate_pose_xyzw'], float)[:, 3:]
    twist = np.asarray(trace['plate_twist_world'], float)
    pad = np.asarray(trace['pad_contacts'], bool)
    rack = np.asarray(trace['rack_contact'], bool)
    lowest = np.asarray(trace['plate_lowest_z_m'], float)
    finite = all(np.isfinite(np.asarray(trace[k], float)).all() for k in
                 ('plate_pose_xyzw', 'plate_twist_world', 'base_pose_xyzw', 'base_twist_world',
                  'width_m', 'coupling_wrench_world', 'plate_lowest_z_m'))
    if not finite:
        return {'valid': False, 'outcome': 'unknown', 'flags': ['invalid'],
                'invalid_reasons': ['non-finite native state'], 'first_failed_stage': None}
    if p.shape != (len(t), 3) or q.shape != (len(t), 4) or twist.shape != (len(t), 6) or pad.shape != (len(t), 2):
        raise GateError('inconsistent telemetry shapes')
    overlaps = {k: np.asarray(trace['overlap_m'][k], float) for k in INTERFACES}
    if any(v.shape != t.shape or np.any(v < 0) or not np.isfinite(v).all() for v in overlaps.values()):
        raise GateError('invalid or missing interface overlap telemetry')
    if np.any(np.abs(np.linalg.norm(q, axis=1) - 1) > .001):
        raise GateError('invalid plate quaternion')
    events = episode['events']
    end = events['end_s']
    complete = t[-1] >= end - expected_step * .51
    if not complete:
        raise GateError('incomplete rollout cannot be scored')
    def at(time):
        if time < t[0] or time > t[-1] + expected_step * .51:
            raise GateError('stage event outside telemetry')
        return int(np.argmin(abs(t - time)))
    acquired_i = at(events['acquired_s'])
    release_i = at(events['release_s'])
    contact_i = np.flatnonzero(rack & (t >= events['lift_off_s']))
    carried_i = int(contact_i[0]) if len(contact_i) else at(events['final_descent_s'])
    goal = scene['goal']
    Rrack = Rotation.from_quat(goal['rack_quaternion_xyzw']).as_matrix()
    local = (p - np.asarray(goal['rack_position_m'])) @ Rrack
    inside = np.all(local[:, :2] >= np.asarray(goal['footprint_min_xy_m']), axis=1)
    inside &= np.all(local[:, :2] <= np.asarray(goal['footprint_max_xy_m']), axis=1)
    tail = t >= end - .5 - 1e-10
    if not tail.any() or t[np.flatnonzero(tail)[0]] > end - .5 + expected_step:
        raise GateError('missing 0.5-second settling tail')
    state = {'plate_position_m': p[-1], 'plate_quaternion_xyzw': q[-1],
             'rack_contact': bool(rack[-1]), 'pad_contacts': pad[-1],
             'tail_max_linear_speed_m_s': float(np.linalg.norm(twist[tail, :3], axis=1).max()),
             'tail_max_angular_speed_rad_s': float(np.linalg.norm(twist[tail, 3:], axis=1).max())}
    stages = {'acquired': bool(lowest[acquired_i] - scene['table_z_m'] >= .01 and pad[acquired_i].all()),
              'carried': bool(pad[carried_i].all()),
              'placed': bool(rack[release_i] and inside[release_i]),
              'retained': retained_predicate(state, goal)}
    outcome = 'retained' if stages['retained'] else 'not_retained'
    invalid = []
    if np.any(np.asarray(trace['contact_truncated'], bool)):
        invalid.append('contact capacity truncation')
    if np.any(p[:, 2] < scene['table_z_m']):
        invalid.append('plate centre below table')
    if np.any(np.asarray(trace['tunnelling'], bool)):
        invalid.append('swept plate fully through pad/rack member')
    if not trace['health_observer_verified']:
        invalid.append('geometry/health observer not preflight-verified')
    if not trace['engine_valid']:
        invalid.append('engine rejected state')
    if np.any(np.asarray(trace.get('affine_deformation_max',[]))>scene['rigid_body_strain_tolerance']):
        invalid.append('IPC affine deformation exceeds declared rigid-body tolerance')
    thickness = {'pad_plate': scene['plate']['rim_thickness_m'],
                 'plate_rack': scene['rack']['post_diameter_m'],
                 'plate_table': scene['plate']['wall_thickness_m']}
    deep = {k: bool(v.max() > .5 * thickness[k]) for k, v in overlaps.items()}
    flags = ['invalid'] if invalid else []
    if any(deep.values()):
        flags.append('deep_penetration')
    if stages['retained'] and (overlaps['plate_rack'][-1] > .5 * thickness['plate_rack'] or
                               overlaps['pad_plate'][carried_i] > .5 * thickness['pad_plate']):
        flags.append('penetration_assisted')
    touch = rack | pad.any(axis=1) | np.asarray(trace['table_contact'], bool)
    recent = np.zeros(len(t), bool)
    last_contact = -np.inf
    for i, ti in enumerate(t):
        if touch[i]:
            last_contact = ti
        recent[i] = ti - last_contact <= .05 + 1e-10
    speed = np.linalg.norm(twist[:, :3], axis=1)
    outside = np.any(p < scene['workspace_min_m'], axis=1) | np.any(p > scene['workspace_max_m'], axis=1)
    if np.any(recent & (speed > 3 * episode['max_recorded_tcp_speed_m_s'])) or outside.any():
        flags.append('ejected')
    base = np.asarray(trace['base_pose_xyzw'], float)
    lift_i = at(events['lift_off_s'])
    rel_positions = []
    rel_rotations = []
    for i in (lift_i, carried_i):
        rb = Rotation.from_quat(base[i, 3:])
        rel_positions.append(rb.inv().apply(p[i] - base[i, :3]))
        rel_rotations.append(rb.inv() * Rotation.from_quat(q[i]))
    slip = float(np.linalg.norm(rel_positions[1] - rel_positions[0]))
    slip_angle = float((rel_rotations[1] * rel_rotations[0].inv()).magnitude())
    if slip > .01 or slip_angle > np.deg2rad(10):
        flags.append('large_slip')
    first_failed = next((s for s in STAGES if not stages[s]), None)
    scored_failed = next((s for s in STAGES if s not in episode['unscored_stages'] and not stages[s]), None)
    return {'valid': not invalid, 'invalid_reasons': invalid, 'outcome': outcome,
            'real_label': episode['real_label'], 'correct': not invalid and outcome == episode['real_label'],
            'stages': stages, 'first_failed_stage': first_failed,
            'first_failed_scored_stage': scored_failed, 'flags': flags,
            'real_failure_stage_matches': first_failed == episode.get('real_failure_stage') if episode['real_label'] == 'not_retained' else None,
            'overlap_mm': {k: {'peak': float(v.max() * 1000), 'final': float(v[-1] * 1000)} for k, v in overlaps.items()},
            'slip_m': slip, 'slip_rad': slip_angle, 'final_state': {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in state.items()}}
