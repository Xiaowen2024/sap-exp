"""RH20T task_0050 (dish on rack) — scene assembly from the frozen completion,
event times from the real robot data only, and the stage / flag evaluation.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path

import numpy as np

from .rp_sim import Observer, Playback, qmat, segments

FINGER_RULE = {'stall_window_s': 0.15, 'stall_tol_m': 0.0005, 'cmd_delta_m': 0.002, 'min_hold_width_m': 0.001, 'min_hold_s': 0.3}

THRESH = {
    'contact_tol_m': 0.003,
    'acquired_clear_m': 0.010, 'acquired_rise_m': 0.020, 'acquired_delay_s': 0.3,
    'settle_s': 2.0, 'rest_window_s': 0.5, 'rest_v_m_s': 0.010, 'rest_w_rad_s': 0.05,
    'cone_margin_deg': 10.0,
    'slip_m': 0.010, 'slip_deg': 10.0,
    'eject_factor': 3.0,
    'min_hold_s': 0.3,
}


def events(pbdata):
    """Replay window and stage instants from the recorded robot trace only."""
    pb = Playback(pbdata, FINGER_RULE)
    segs = [(a, b) for a, b in segments(pb.gt, pb.closed) if b - a >= THRESH['min_hold_s']]
    if not segs:
        raise ValueError('no holding segment')
    t_close = segs[0][0]
    t_release = segs[-1][1]
    t = pb.t
    z = np.array([pb.pos(x)[2] for x in t])
    p_rel = pb.pos(t_release)
    after = t[(t > t_release) & (np.linalg.norm(np.array([pb.pos(x) for x in t]) - p_rel, axis=1) >= .05)]
    t_withdraw = float(after[0]) if len(after) else float(t[-1])
    z_close = float(pb.pos(t_close)[2])
    risen = t[(t > t_close) & (z >= z_close + THRESH['acquired_rise_m'])]
    t_acq = float(risen[0]) + THRESH['acquired_delay_s'] if len(risen) else np.nan
    # start of the final descent: last maximum of TCP z before release
    m = (t >= t_close) & (t <= t_release)
    t_final_desc = float(t[m][np.argmax(z[m])])
    t_start = max(float(t[0]), t_close - 2.0)
    t_end = min(float(t[-1]), t_withdraw + THRESH['settle_s'])
    vmax = float(np.max(np.linalg.norm(np.array([pb.vel(x) for x in t[m]]), axis=1)))
    return {'t_start': t_start, 't_close': t_close, 't_acquired': t_acq, 't_final_descent': t_final_desc,
            't_release': t_release, 't_withdraw': t_withdraw, 't_end': t_end,
            'segments': segs, 'tcp_speed_max': vmax,
            'settle_covered': bool(t_withdraw + THRESH['settle_s'] <= float(t[-1]))}


def apply_cell(scene, cell):
    s = copy.deepcopy(scene)
    lvl = cell.get('level')
    ax = cell.get('axis', 'none')
    if ax == 'dt':
        s['dt'] = cell['dt']
    elif ax == 'mass':
        s['basket']['mass'] = s['ranges']['mass'][lvl]
    elif ax == 'mu_pad':
        s['mu']['pad_basket'] = s['ranges']['mu_pad'][lvl]
    elif ax == 'mu_env':
        s['mu']['basket_env'] = s['ranges']['mu_env'][lvl]
    elif ax == 'grip':
        s['drive']['F_close'] = s['ranges']['grip'][lvl]
    elif ax == 'coupling':
        f = 1 / 3 if lvl == 'low' else 3.
        s['drive']['K_t'] *= f; s['drive']['K_r'] *= f
    elif ax == 'profile':
        s['basket']['rim'] = 'rolled' if lvl == 'rolled' else 'none'
        s['basket']['wall_t'] = s['ranges']['wall_t'][lvl] if lvl in s['ranges']['wall_t'] else s['basket']['wall_t']
    elif ax == 'basket_pose':
        sgn = -1 if lvl == 'minus' else 1
        dxy = np.asarray(s['ranges']['basket_pose_dir']) * s['ranges']['basket_pose_half'] * sgn
        s['basket']['xy'] = (np.asarray(s['basket']['xy']) + dxy).tolist()
    elif ax == 'rack_pose':
        sgn = -1 if lvl == 'minus' else 1
        dxy = np.asarray(s['ranges']['rack_pose_dir']) * s['ranges']['rack_pose_half'] * sgn
        s['rack']['xy'] = (np.asarray(s['rack']['xy']) + dxy).tolist()
    elif ax != 'none':
        raise ValueError('unknown axis ' + ax)
    from .rp_engines import shape_mu
    s['mu']['pad_env'] = shape_mu(s['mu']['pad_basket'], s['mu']['basket_env'])['pad_env_derived']
    return s


def evaluate(trace, health, scene, meta, ev):
    """Stage ladder, outcome and flags from a 100-Hz trace (engine independent)."""
    obs = Observer(scene, meta)
    T = np.array([s['t'] for s in trace])
    out = {'valid': True, 'invalid_reasons': [], 'flags': []}
    if not health['finite']:
        out['valid'] = False; out['invalid_reasons'].append('non-finite state')
    if health['truncated']:
        out['valid'] = False; out['invalid_reasons'].append('contact capacity truncation')
    if T[-1] < ev['t_end'] - 0.05:
        out['valid'] = False; out['invalid_reasons'].append('trace ended early')
    tol = THRESH['contact_tol_m']
    D = []
    for s in trace:
        if not np.isfinite(s['basket']).all():
            D.append((np.inf, np.inf, [np.inf, np.inf])); continue
        D.append(obs.distances(s['basket'][:3], s['basket'][3:], [(p[:3], p[3:]) for p in s['pads']]))
    dr = np.array([x[0] for x in D]); dt = np.array([x[1] for x in D]); dp = np.array([x[2] for x in D])
    pen = {'pad_basket': float(max(0, -np.nanmin(dp))), 'basket_rack': float(max(0, -np.nanmin(dr))),
           'basket_table': float(max(0, -np.nanmin(dt)))}
    out['observer_peak_overlap_mm'] = {k: v * 1e3 for k, v in pen.items()}
    out['engine_peak_overlap_mm'] = {k: v * 1e3 for k, v in health.get('engine_peak_overlap_m', {}).items()}
    out['final_overlap_mm'] = {'pad_basket': float(max(0, -dp[-1].min()) * 1e3), 'basket_rack': float(max(0, -dr[-1]) * 1e3),
                               'basket_table': float(max(0, -dt[-1]) * 1e3)}
    # tunnelling / through-table: basket centre below the table plane
    if np.nanmin(dt) < -0.5 * scene['basket']['H']:
        out['valid'] = False; out['invalid_reasons'].append('basket through table')

    def at(t):
        return int(np.clip(np.searchsorted(T, t), 0, len(T) - 1))
    both_pads = (dp < tol).all(1)
    # acquired (reported, not scored: wall thickness is E3)
    ia = at(ev['t_acquired']) if np.isfinite(ev['t_acquired']) else len(T) - 1
    acquired = bool(dt[ia] >= THRESH['acquired_clear_m'] and both_pads[ia])
    # placement onset: first basket-rack contact after acquisition, else start of final descent
    rc = np.nonzero((dr < tol) & (T > ev['t_close']))[0]
    t_onset = T[rc[0]] if len(rc) and T[rc[0]] <= ev['t_release'] else ev['t_final_descent']
    io = at(t_onset)
    carried = bool(both_pads[max(0, io - 1)])
    ir = at(ev['t_release'])
    placed = bool(dr[ir] < tol and obs.in_footprint(trace[ir]['basket'][:3]))
    ie = len(T) - 1
    r = scene['rack']
    lean_max = np.deg2rad(scene['lean_max_deg'])
    up = obs.upright_angle(trace[ie]['basket'][3:])
    ax = qmat(trace[ie]['basket'][3:])[:, 2]
    xr = obs.rack_pose[1][:, 0]
    hproj = ax - (ax @ obs.table_R[:, 2]) * obs.table_R[:, 2]
    align = float(np.degrees(np.arccos(min(1., abs(hproj @ xr) / max(np.linalg.norm(hproj), 1e-9)))))
    # geometric centre (mid-height on the axis) for the footprint test
    centre_end = trace[ie]['basket'][:3]
    w0 = at(T[ie] - THRESH['rest_window_s'])
    V = np.array([s['basket_v'] for s in trace[w0:]])
    rest = bool(np.all(np.linalg.norm(V[:, :3], axis=1) <= THRESH['rest_v_m_s']) and
                np.all(np.linalg.norm(V[:, 3:], axis=1) <= THRESH['rest_w_rad_s']))
    goal = scene.get('goal_type', 'upright_in_slot')
    orient_ok = (up <= lean_max and align <= 45.) if goal == 'upright_in_slot' else True
    retained = bool(dr[ie] < tol and (dp[ie] > tol).all() and obs.in_footprint(centre_end) and orient_ok and rest)
    out.update(stage={'acquired': acquired, 'carried': carried, 'placed': placed, 'retained': retained},
               retained_detail={'touch_rack': bool(dr[ie] < tol), 'free_of_pads': bool((dp[ie] > tol).all()),
                                'in_footprint': obs.in_footprint(trace[ie]['basket'][:3]),
                                'lean_deg': float(np.degrees(up)), 'lean_max_deg': float(scene['lean_max_deg']),
                                'axis_vs_rails_deg': align,
                                'at_rest': rest, 'final_basket_pose': trace[ie]['basket'].tolist()},
               t_onset=float(t_onset))
    out['outcome'] = 'retained' if retained else 'not_retained'
    first = None
    for k in ('carried', 'placed', 'retained'):
        if not out['stage'][k]:
            first = k; break
    out['first_failed_stage'] = first
    # flags
    h_rim = scene['basket']['wall_t']
    if pen['pad_basket'] > .5 * h_rim:
        out['flags'].append('deep_penetration:pad_basket')
    if pen['basket_rack'] > .5 * r['dp']:
        out['flags'].append('deep_penetration:basket_rack')
    if pen['basket_table'] > .5 * scene['basket']['bottom_t']:
        out['flags'].append('deep_penetration:basket_table')
    if retained and (out['final_overlap_mm']['basket_rack'] / 1e3 > .5 * r['dp']):
        out['flags'].append('penetration_assisted')
    vmax = np.max(np.linalg.norm(np.array([s['basket_v'][:3] for s in trace]), axis=1))
    if vmax > THRESH['eject_factor'] * max(ev['tcp_speed_max'], 0.05):
        out['flags'].append('ejected')
    # large slip between lift (t_acquired) and onset, in the base frame
    ia2, io2 = ia, io
    if io2 > ia2:
        def rel(s):
            Rb = qmat(s['base'][3:]); return Rb.T @ (s['basket'][:3] - s['base'][:3]), Rb.T @ qmat(s['basket'][3:])
        p0, R0 = rel(trace[ia2]); p1, R1 = rel(trace[io2])
        from scipy.spatial.transform import Rotation as Rr
        ang = np.degrees(Rr.from_matrix(R1 @ R0.T).magnitude())
        out['slip_mm'] = float(np.linalg.norm(p1 - p0) * 1e3); out['slip_deg'] = float(ang)
        if np.linalg.norm(p1 - p0) > THRESH['slip_m'] or ang > THRESH['slip_deg']:
            out['flags'].append('large_slip')
    # pads fully closed on a recorded hold (stalled width >= 1.5 mm) while the object
    # was within 5 mm of both pads at the start of the hold: tunnelling or a missed grasp
    seg_flags = []
    for (a, b) in ev.get('segments', []):
        i0, i1 = at(a), at(b)
        if i1 <= i0:
            continue
        near = bool((dp[i0] < 0.005).all())
        sep = np.array([sum(s['slider']) for s in trace[i0:i1]])
        if near and (sep < 5e-4).mean() > 0.5:
            seg_flags.append([float(a), float(b)])
    if seg_flags:
        out['flags'].append('full_closure_on_object')
        out['full_closure_segments'] = seg_flags
    out['correct'] = bool(out['valid'] and out['outcome'] == scene['real_label'])
    return out
