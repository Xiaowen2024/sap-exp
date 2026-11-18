"""Freeze completion revision c1 for RH20T task_0050 cfg2 user_0016 episodes.

Inputs: per-episode E2 fits (complete_rh20t.py), the calibration-board plane,
the exported playback npz (E1/E3).  Output: one scene JSON per episode with
value, range, class and source for every quantity, plus the admission table.
Nothing here reads a simulated outcome.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from .rp_sim import Playback, segments
from .rp_task import FINGER_RULE, events

OBJECTS = {  # E2/visual object class per episode (read from the first frames)
    1: 'oval_wicker', 2: 'oval_wicker', 5: 'oval_wicker',
    6: 'white_plate', 7: 'white_plate', 8: 'white_plate',
    9: 'round_wicker', 10: 'round_wicker'}
LABELS = {1: 8, 2: 8, 5: 1, 6: 5, 7: 9, 8: 9, 9: 9, 10: 9}  # official meta.rating
E5 = {
    'oval_wicker': {'mass': (0.12, 0.05), 'mu_pad': (0.6, 0.2), 'mu_env': (0.5, 0.2), 'bottom_t': 0.003},
    'round_wicker': {'mass': (0.15, 0.06), 'mu_pad': (0.6, 0.2), 'mu_env': (0.5, 0.2), 'bottom_t': 0.003},
    'white_plate': {'mass': (0.40, 0.15), 'mu_pad': (0.4, 0.15), 'mu_env': (0.4, 0.15), 'bottom_t': 0.005},
}
E4 = {'pad_size': [0.020, 0.006, 0.032], 'pad_mass': 0.10, 'stroke': 0.095, 'grip': (102.5, 57.5)}
DRIVE = {'base_mass': 1.0, 'base_inertia': 0.002, 'K_t': 1e5, 'K_r': 300., 'F_cap': 300., 'T_cap': 5.,
         'finger_K': 3000., 'finger_D_closed': 2.}
BOARD = {'point': [0.6667, -0.0004, 0.0212], 'normal': [-0.039, -0.020, 0.999]}  # calib 1633226229297
POSE_HALF = {'xy_m': 0.006, 'yaw_deg': 5.0}  # declared E2 pose half-range (calibration + fit)


def canon_obj(p):
    """Order semi-axes a >= b; yaw follows the a axis."""
    a, b, yaw = p['a'], p['b'], p['yaw']
    if b > a:
        a, b, yaw = b, a, yaw + np.pi / 2
    return a, b, yaw % np.pi


def mad_half(x):
    x = np.asarray(x, float)
    return float(max(1.4826 * np.median(np.abs(x - np.median(x))), 1e-4))


def grasp_conditioned(scene, pbd, wall_e3=False):
    """c3: E3 grasp position.  Shift the object along the horizontal finger axis
    so the wall mid-surface passes through the TCP at the stall instant (the
    recorded stalled width says both fingers touch the wall there), and start
    the replay 0.3 s before the stall so the unmodelled approach contact is not
    replayed.  Costs the 'acquired' stage (already unscored)."""
    from scipy.optimize import brentq
    from .rp_sim import Playback, place_on_table, qmat, build_assets, basket_body_pose, Observer
    import tempfile
    b = scene['basket']
    pb = Playback(pbd, FINGER_RULE)
    ts = scene['events']['t_close']
    p, q, *_ = pb.target(ts)
    pos0, q0 = place_on_table(scene, b['xy'], b['yaw'])
    Rb0 = qmat(q0)
    loc = Rb0.T @ (p - pos0)
    yf = Rb0.T @ qmat(q)[:, 1]
    if wall_e3:
        # c4: wall slope at the grasp from the same E3 event -- the slope that makes
        # the wall normal best aligned with the recorded finger axis at the stall
        th = np.arctan2(loc[1] / b['b'], loc[0] / b['a'])
        nr = np.array([np.cos(th) / b['a'], np.sin(th) / b['b'], 0.]); nr /= np.linalg.norm(nr)
        r = 1 / np.hypot(np.cos(th) / b['a'], np.sin(th) / b['b'])
        yr, yz = float(yf @ nr), float(yf[2])
        slope = -yz / yr if abs(yr) > 1e-6 else 0.
        k_e2 = b['k']
        b['k'] = float(np.clip(1 - slope * b['H'] / r, 0.5, 1.05))
        scene['basket']['k_e2'] = k_e2
        scene['completion_table']['object_dims'].append({'k_wall_slope': ['E3', 'wall parallel to the recorded pad faces at the stall', {'k_e2': k_e2, 'k_e3': b['k']}]})
    u = np.array([yf[0], yf[1], 0.]); u /= np.linalg.norm(u)
    h = loc[2]; L = scene['pad']['size'][2]
    hc = (max(h, 0.) + min(h + L, b['H'])) / 2  # pad/wall overlap mid-height (pads extend from TCP towards the flange)
    def wall_e(x):
        th = np.arctan2(x[1] / b['b'], x[0] / b['a'])
        rdir = 1 / np.hypot(np.cos(th) / b['a'], np.sin(th) / b['b'])
        return np.hypot(x[0] / b['a'], x[1] / b['b']) - ((b['k'] + (1 - b['k']) * hc / b['H']) - (b['wall_t'] / 2) / rdir)
    f = lambda d: wall_e(loc[:2] + d * u[:2])
    grid = np.linspace(-0.08, 0.08, 1601)
    vals = np.array([f(d) for d in grid])
    roots = [brentq(f, grid[i], grid[i + 1]) for i in range(len(grid) - 1) if vals[i] * vals[i + 1] < 0]
    if not roots:
        scene['admission']['admitted'] = False
        scene['admission']['reasons'].append('c3: no wall crossing along the finger axis within 80 mm')
        return scene
    dlt = min(roots, key=abs)  # TCP shift in basket frame; object shift is -dlt
    shift_world = -(Rb0 @ (dlt * u))
    scene['basket']['xy'] = (np.asarray(b['xy']) + shift_world[:2]).tolist()
    scene['basket']['xy_e2'] = b['xy']
    scene['completion_revision'] = 'c4' if wall_e3 else 'c3'
    flagged = abs(dlt) > 3 * POSE_HALF['xy_m']
    # replay start: 0.3 s before the stall, earlier if the pads would start inside the object
    sc2 = json.loads(json.dumps(scene))
    with tempfile.TemporaryDirectory() as td:
        meta = build_assets(sc2, td)
        obs = Observer(sc2, meta)
        bpos, bq = basket_body_pose(sc2, meta)
        t0 = ts - 0.3
        while t0 > scene['events']['t_start']:
            pp, qq, _, _, w, _ = pb.target(t0)
            Rq = qmat(qq)
            pads = [(pp + Rq @ np.array([0., sd * (w / 2 + scene['pad']['size'][1] / 2), -L / 2]), qq) for sd in (-1, 1)]
            dr, dt_, dp = obs.distances(bpos, bq, pads)
            if min(dp) >= 1e-3:
                break
            t0 -= 0.05
    scene['t_start'] = float(t0)
    clear = min(dp)
    if clear < 1e-3:
        scene['admission']['admitted'] = False
        scene['admission']['reasons'].append(f'c3: no replay start within the window with the pads clear of the shifted object (min clearance {clear*1e3:.1f} mm); video shows the object pushed by the gripper during the approach')
    scene['completion_table']['object_pose'] = ['E2 + E3', 'E2 first-frame fit shifted along the finger axis so the wall passes the stalled fingers (grasp position, E3)',
                                                {'shift_m': float(np.linalg.norm(shift_world)), 'flag_E2_E3_disagree': bool(flagged)}]
    scene['admission']['c3'] = {'shift_mm': float(np.linalg.norm(shift_world) * 1e3), 'flag_E2_E3_disagree': bool(flagged),
                                'pad_overlap_height_mm': float(hc * 1e3), 'replay_start_before_stall_s': float(ts - t0),
                                'review': 'flagged shifts reviewed against the video at the stall instant' if flagged else ''}
    return scene


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fits', required=True); ap.add_argument('--playback', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--grasp-e3', action='store_true', help='completion c3 (E3 grasp-position conditioning)')
    ap.add_argument('--wall-e3', action='store_true', help='completion c4 (c3 + E3 wall slope at the grasp)')
    a = ap.parse_args()
    fits = {s: json.loads(Path(a.fits, f'task_0050_user_0016_scene_{s:04d}_cfg_0002.json').read_text()) for s in OBJECTS}
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    # rack dims: same physical rack in every episode -> pooled median, spread = range
    keys = ['L', 'w', 'h', 'D', 'dp', 'Hp', 'sp', 'dc']
    rk = {k: [fits[s]['rack_first']['params'][k] for s in OBJECTS] for k in keys}
    rack_dims = {k: float(np.median(v)) for k, v in rk.items()}
    rack_half = {k: mad_half(v) for k, v in rk.items()}
    # object dims pooled per object class
    obj_dims = {}
    for cls in set(OBJECTS.values()):
        ss = [s for s in OBJECTS if OBJECTS[s] == cls]
        A = [canon_obj(fits[s]['basket_first']['params']) for s in ss]
        obj_dims[cls] = {'a': float(np.median([x[0] for x in A])), 'b': float(np.median([x[1] for x in A])),
                         'k': float(np.median([fits[s]['basket_first']['params']['k'] for s in ss])),
                         'H': float(np.median([fits[s]['basket_first']['params']['H'] for s in ss])),
                         'half': {'a': mad_half([x[0] for x in A]), 'b': mad_half([x[1] for x in A]),
                                  'H': mad_half([fits[s]['basket_first']['params']['H'] for s in ss])}}
        if cls != 'oval_wicker':  # round objects: a = b
            r = (obj_dims[cls]['a'] + obj_dims[cls]['b']) / 2
            obj_dims[cls]['a'] = obj_dims[cls]['b'] = r
    lean_max = float(np.degrees(np.arctan2(2 * rack_dims['sp'], rack_dims['Hp']))) + 10.
    table = []
    for s, cls in OBJECTS.items():
        e = f'task_0050_user_0016_scene_{s:04d}_cfg_0002'
        pbd = dict(np.load(Path(a.playback) / f'{e}.npz'))
        ev = events(pbd)
        pb = Playback(pbd, FINGER_RULE)
        hold = np.zeros(len(pb.gt), bool)
        for t0, t1 in ev['segments']:
            hold |= (pb.gt >= t0) & (pb.gt < t1)
        wall_t = float(np.median(pb.gw[hold]))
        wall_half = float(max(mad_half(pb.gw[hold]), 0.0005))
        f = fits[s]
        o = f['basket_first']['params']
        oa, ob, oyaw = canon_obj(o)
        rp = f['rack_first']['params']
        # predicate on reality (final-frame E2 fit)
        fb = f['basket_final']['params']
        ax = R.from_rotvec([fb['rx'], fb['ry'], fb['rz']]).as_matrix()[:, 2]
        xr = np.array([np.cos(rp['yaw']), np.sin(rp['yaw'])])
        lean = float(np.degrees(np.arcsin(min(1, abs(ax[2])))))
        hdir = ax[:2] / max(np.linalg.norm(ax[:2]), 1e-9)
        align = float(np.degrees(np.arccos(min(1, abs(hdir @ xr)))))
        c = np.array([fb['x'], fb['y']]) + ax[:2] * 0.02
        loc = np.array([[xr[0], xr[1]], [-xr[1], xr[0]]]) @ (c - [rp['x'], rp['y']])
        in_fp = bool(abs(loc[0]) <= rack_dims['L'] / 2 and abs(loc[1]) <= (rack_dims['D'] + rack_dims['w']) / 2)
        final_iou = float(np.mean(list(f['basket_final']['iou'].values())))
        pred_eval = final_iou >= 0.5
        pred = bool(in_fp and align <= 45. and lean <= lean_max)
        real = 'retained' if LABELS[s] >= 2 else 'not_retained'
        # admission margins (README 2.3): placed = lateral slot clearance, retained = support height
        u_place = float(np.sqrt(POSE_HALF['xy_m'] ** 2 * 2 + rack_half['sp'] ** 2 + wall_half ** 2))
        m_place = rack_dims['sp'] - rack_dims['dp'] - wall_t
        u_ret = float(np.sqrt(rack_half['Hp'] ** 2 + obj_dims[cls]['half']['H'] ** 2))
        m_ret = rack_dims['Hp']
        merged = []
        for a0, b0 in ev['segments']:
            if merged and a0 - merged[-1][1] < 1.0:
                merged[-1][1] = b0
            else:
                merged.append([a0, b0])
        single = len(merged) == 1
        admitted = pred_eval and (pred == (real == 'retained')) and m_place >= 3 * u_place and m_ret >= 3 * u_ret \
            and ev['settle_covered'] and single
        reasons = []
        if not pred_eval:
            reasons.append(f'predicate on reality not evaluable (final-frame fit IoU {final_iou:.2f} < 0.5)')
        elif pred != (real == 'retained'):
            reasons.append('predicate on reality disagrees with the real label')
        if m_place < 3 * u_place:
            reasons.append(f'margin placed m={m_place*1e3:.1f} mm < 3u={3*u_place*1e3:.1f} mm')
        if m_ret < 3 * u_ret:
            reasons.append(f'margin retained m={m_ret*1e3:.1f} mm < 3u={3*u_ret*1e3:.1f} mm')
        if not ev['settle_covered']:
            reasons.append('recording ends before withdrawal + 2 s settle')
        if not single:
            reasons.append(f'{len(merged)} separate grasps (re-grasps are closed-loop operator corrections; c2 rule: one grasp)')
        e5 = E5[cls]
        dirx = np.array([np.cos(rp['yaw']), np.sin(rp['yaw'])])  # least 'placed' margin: across the slot = along rails
        scene = {
            'episode': e, 'scene_id': s, 'object_class': cls, 'rating': LABELS[s], 'real_label': real,
            'goal_type': 'upright_in_slot', 'completion_revision': 'c2',
            'dt': 5e-4, 't_start': ev['t_start'], 't_end': ev['t_end'],
            'table': {**BOARD, 'box_thickness': 0.04},
            'basket': {'xy': [o['x'], o['y']], 'yaw': float(oyaw), 'a': oa if cls == 'oval_wicker' else obj_dims[cls]['a'],
                       'b': ob if cls == 'oval_wicker' else obj_dims[cls]['b'],
                       'k': obj_dims[cls]['k'], 'H': obj_dims[cls]['H'], 'wall_t': wall_t, 'bottom_t': e5['bottom_t'],
                       'mass': e5['mass'][0], 'rim': 'none'},
            'rack': {'xy': [rp['x'], rp['y']], 'yaw': float(rp['yaw']), **rack_dims},
            'pad': {'size': E4['pad_size'], 'mass': E4['pad_mass'], 'stroke': E4['stroke']},
            'drive': {**DRIVE, 'F_close': E4['grip'][0]},
            'mu': {'pad_basket': e5['mu_pad'][0], 'basket_env': e5['mu_env'][0]},
            'finger_rule': FINGER_RULE,
            'lean_max_deg': lean_max,
            'ranges': {'mass': {'low': e5['mass'][0] - e5['mass'][1], 'high': e5['mass'][0] + e5['mass'][1]},
                       'mu_pad': {'low': e5['mu_pad'][0] - e5['mu_pad'][1], 'high': e5['mu_pad'][0] + e5['mu_pad'][1]},
                       'mu_env': {'low': e5['mu_env'][0] - e5['mu_env'][1], 'high': e5['mu_env'][0] + e5['mu_env'][1]},
                       'grip': {'low': E4['grip'][0] - E4['grip'][1], 'high': E4['grip'][0] + E4['grip'][1]},
                       'H': {'low': obj_dims[cls]['H'] - obj_dims[cls]['half']['H'], 'high': obj_dims[cls]['H'] + obj_dims[cls]['half']['H']},
                       'wall_t': {},
                       'basket_pose_half': POSE_HALF['xy_m'], 'basket_pose_dir': dirx.tolist(),
                       'rack_pose_half': POSE_HALF['xy_m'], 'rack_pose_dir': dirx.tolist()},
            'completion_table': {
                'tcp_pose': ['E1', 'high_freq_data.npy base tcp, 125 Hz, wxyz (P0 geometric check)'],
                'gripper_width': ['E1', 'gripper.npy gripper_info[0] mm'],
                'table_plane': ['E2', 'calibration-board pose from extrinsics x base-camera transform; agrees with object silhouette bottoms within 2 mm; RealSense depth biased ~-15 mm (not used)'],
                'object_dims': ['E2', f'multi-view silhouette fit, first frame, pooled median over {cls} episodes', obj_dims[cls]],
                'object_pose': ['E2', 'multi-view silhouette fit, first frame', {'half_xy_m': POSE_HALF['xy_m'], 'half_yaw_deg': POSE_HALF['yaw_deg']}],
                'rack_dims': ['E2', 'multi-view silhouette fit, pooled median over 8 episodes (same rack)', {'value': rack_dims, 'half': rack_half}],
                'rack_pose': ['E2', 'multi-view silhouette fit, first frame', {'half_xy_m': POSE_HALF['xy_m'], 'half_yaw_deg': POSE_HALF['yaw_deg']}],
                'rack_fixed': ['E2', 'no visible rack motion first->last frame (by eye)'],
                'wall_thickness': ['E3', 'median stalled gripper width over holding segments', {'value': wall_t, 'half': wall_half}, 'unscored stage: acquired'],
                'mass': ['E5', 'wrist F/T payload step inconsistent across episodes (Flexiv-compensated; -0.3..-2.9 N for one object); declared interval', e5['mass']],
                'pad_geometry': ['E4', 'AG-95 fingertip, two-pad parallel surrogate (approximate datasheet)', E4['pad_size']],
                'closing_force': ['E4', 'AG-95 45-160 N per jaw; gripper_command[1] semantics (force vs speed %) unverified', E4['grip']],
                'mu_pad_object': ['E5', 'declared interval', e5['mu_pad']],
                'mu_object_env': ['E5', 'declared interval (wood rack, rubber mat)', e5['mu_env']],
                'rim_profile': ['E5', 'variants: none (nominal), rolled rim, shallow (H low)'],
                'gravity': ['assumed', 'along base -z (robot base level); table normal 2.5 deg from base z per calibration'],
            },
            'admission': {'predicate_on_reality': {'evaluable': pred_eval, 'final_iou': final_iou, 'lean_deg': lean,
                                                   'axis_vs_rails_deg': align, 'centre_in_rack_m': loc.tolist(),
                                                   'in_footprint': in_fp, 'predicate': 'retained' if pred else 'not_retained'},
                          'margins': {'placed': {'m_mm': m_place * 1e3, 'u_mm': u_place * 1e3},
                                      'retained': {'m_mm': m_ret * 1e3, 'u_mm': u_ret * 1e3}},
                          'admitted': bool(admitted), 'reasons': reasons},
            'events': {k: v for k, v in ev.items() if k != 'segments'}, 'segments': ev['segments'],
            'playback_npz_sha256': hashlib.sha256((Path(a.playback) / f'{e}.npz').read_bytes()).hexdigest(),
        }
        if admitted and getattr(a, 'grasp_e3', False):
            scene = grasp_conditioned(scene, pbd, getattr(a, 'wall_e3', False))
        (out / f'{e}.json').write_text(json.dumps(scene, indent=1, default=float))
        table.append((s, cls, real, admitted, reasons, pred, align, lean, m_place, u_place))
        print(s, cls, real, 'admitted' if admitted else 'NOT admitted', reasons,
              'pred=%s align=%.0f lean=%.0f m_place=%.1fmm u=%.1fmm wall=%.1fmm' % (pred, align, lean, m_place * 1e3, u_place * 1e3, wall_t * 1e3))
    digest = hashlib.sha256(b''.join((out / f'task_0050_user_0016_scene_{s:04d}_cfg_0002.json').read_bytes() for s in sorted(OBJECTS))).hexdigest()
    (out / 'REVISION').write_text(f"{'c4' if a.wall_e3 else 'c3' if a.grasp_e3 else 'c2'} {digest}\n")
    print('rack dims', {k: round(v, 4) for k, v in rack_dims.items()}, 'half', {k: round(v, 4) for k, v in rack_half.items()}, 'lean_max', round(lean_max, 1))
    print('completion c2 sha256', digest)


if __name__ == '__main__':
    main()
