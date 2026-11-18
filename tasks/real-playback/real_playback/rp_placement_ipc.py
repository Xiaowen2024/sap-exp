"""Reproduce the corrected v5 placement segment with Genesis + IPC.

This is a single-episode exploratory replay, not the full-task B=5 protocol.
Scene corrections are explicit and saved separately from solver candidates.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import pickle
import resource
import time
import importlib.metadata
from pathlib import Path

import numpy as np

from .rp_engines import initial_state, pad_local_center
from .rp_sim import Observer, Playback, build_assets, qmat, write_json


def clean_entry(scene, data, height, duration):
    """Explicit alternate protocol: close fingers above the rack, then lower to the original entry.

    The original recorded segment follows unchanged after a 0.1 s hold and a
    smooth descent. This is not a replay from the published overlapping state.
    """
    pb = Playback(data, scene['finger_rule']); t0 = scene['t_start']; hold = .1
    span = hold + duration; n = np.array(scene['table']['normal'], float); n /= np.linalg.norm(n)
    t = np.arange(0., span, .008)
    u = np.clip((t-hold)/duration, 0, 1); smooth = 10*u**3-15*u**4+6*u**5
    p = pb.pos(t0); q = pb.rot(t0).as_quat()[[3,0,1,2]]
    poses = np.c_[p[None,:]+height*(1-smooth[:,None])*n[None,:], np.tile(q,(len(t),1))]
    keep = data['tcp_t'] > t0 + 1e-4
    out = dict(data)
    out['tcp_t'] = np.r_[t0+t, t0+span, data['tcp_t'][keep]+span]
    out['tcp_pose_wxyz'] = np.vstack([poses, np.r_[p,q], data['tcp_pose_wxyz'][keep]])
    gt = np.arange(0., span, .01); keepg = data['grip_t'] > t0 + 1e-4
    out['grip_t'] = np.r_[t0+gt,t0+span,data['grip_t'][keepg]+span]
    for key in ['grip_width_m','grip_cmd_m']:
        w = float(np.interp(t0,data['grip_t'],data[key]))
        out[key] = np.r_[np.full(len(gt)+1,w),data[key][keepg]]
    scene['basket']['pose_override']['pos'] = (np.array(scene['basket']['pose_override']['pos'])+height*n).tolist()
    scene['t_end'] += span
    for k in ['t_release','t_withdraw','t_end']:
        scene['events'][k] += span
    scene['segment']['clean_entry'] = {'height_m':height,'hold_s':hold,'descent_s':duration,
        'rule':'same dish and tool lifted together; quintic vertical entry, then the unchanged original recorded segment',
        'comparison':'alternate initialization and command prefix; not a direct ranking against published v5'}
    return out


def judge(trace, scene, meta, real_along=-0.07799972453363645):
    if not trace:
        return {'success': False, 'reason': 'no recorded states'}
    obs = Observer(scene, meta)
    s = trace[-1]
    dr, dt, dp = obs.distances(s['basket'][:3], s['basket'][3:], [(p[:3], p[3:]) for p in s['pads']])
    n, rail = obs.table_R[:, 2], obs.rack_pose[1][:, 0]
    axis = qmat(s['basket'][3:])[:, 2]
    lean = float(np.degrees(np.arcsin(np.clip(abs(axis @ n), 0, 1))))
    horiz = axis - (axis @ n) * n
    align = float(np.degrees(np.arccos(np.clip(abs(horiz @ rail) / max(np.linalg.norm(horiz), 1e-12), 0, 1))))
    loc = obs.rack_pose[1].T @ (s['basket'][:3] - obs.rack_pose[0])
    V = np.array([x['basket_v'] for x in trace if x['t'] >= s['t'] - .5])
    rest = bool(np.all(np.linalg.norm(V[:, :3], axis=1) <= .01) and np.all(np.linalg.norm(V[:, 3:], axis=1) <= .05))
    same_slot = bool(abs(loc[0] - real_along) <= scene['rack']['sp'] / 2 and abs(loc[1]) <= (scene['rack']['D'] + scene['rack']['w']) / 2)
    complete = s['t'] >= scene['t_end'] - .02
    checks = {'touch_rack': bool(dr <= .003), 'same_slot_as_real': same_slot,
              'aligned': align <= 45, 'lean_ok': lean <= scene['lean_max_deg'], 'at_rest': rest, 'complete': bool(complete)}
    grasp_samples = [x for x in trace if scene['t_start'] + .1 <= x['t'] <= scene['events']['t_release'] - .3]
    held = []
    for x in grasp_samples[::5]:
        pad_gap = obs.distances(x['basket'][:3], x['basket'][3:], [(p[:3], p[3:]) for p in x['pads']])[2]
        held.append(max(pad_gap) <= .003 and sum(x['slider']) >= .8 * scene['basket']['wall_t'])
    grasp_fraction = float(np.mean(held)) if held else None
    return {**checks, 'success': all(checks.values()), 'rack_dist_mm': dr * 1e3,
            'table_dist_mm': dt * 1e3, 'along_rail_offset_from_real_mm': (loc[0] - real_along) * 1e3,
            'lean_deg': lean, 'axis_vs_rails_deg': align, 'leans_on_finger': bool(min(dp) <= .003),
            't_end_rel_release_s': s['t'] - scene['events']['t_release'],
            'grasp_fraction_before_release': grasp_fraction,
            'grasp_diagnostic': 'both sampled pad gaps <=3mm and joint opening >=0.8 wall thickness; not direct contact-force measurement',
            'finger_opening_mm_at_0.5s': sum(trace[min(50, len(trace)-1)]['slider']) * 1e3}


def main():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', required=True); ap.add_argument('--playback', required=True)
    ap.add_argument('--out', required=True); ap.add_argument('--params', default='{}')
    ap.add_argument('--label', default='native'); ap.add_argument('--duration', type=float)
    ap.add_argument('--extra-width-mm', type=float, default=0.)
    ap.add_argument('--entry-height-mm', type=float, default=0.)
    ap.add_argument('--entry-descent-s', type=float, default=1.)
    ap.add_argument('--engine', choices=['genesis_ipc', 'mujoco'], default='genesis_ipc')
    ap.add_argument('--no-objects', action='store_true')
    a = ap.parse_args(); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    sc = json.loads(Path(a.scene).read_text())
    if a.duration: sc['t_end'] = sc['t_start'] + a.duration
    sc.setdefault('segment', {})['init_extra_width_m'] = a.extra_width_mm / 1000
    pb = dict(np.load(a.playback, allow_pickle=False))
    if a.entry_height_mm:
        pb = clean_entry(sc, pb, a.entry_height_mm/1000, a.entry_descent_s)
        np.savez_compressed(out/'actual_playback.npz', **pb)
    meta = build_assets(sc, out / 'assets'); write_json(out / 'scene.json', sc)
    candidate = {'id': a.label, 'parameters': json.loads(a.params)}
    write_json(out / 'request.json', {'candidate': candidate, 'engine': a.engine, 'objects': not a.no_objects,
        'extra_width_mm': a.extra_width_mm, 'source_scene_sha256': hashlib.sha256(Path(a.scene).read_bytes()).hexdigest(),
        'playback_sha256': hashlib.sha256(Path(a.playback).read_bytes()).hexdigest()})
    if not a.no_objects:
        _, p, q, width, bp, bq = initial_state(sc, Playback(pb, sc['finger_rule']), meta)
        pads = [(p + qmat(q) @ pad_local_center(sc, side, width/2), q) for side in (-1, 1)]
        ds = Observer(sc, meta).distances(bp, bq, pads)
        write_json(out / 'initial_geometry.json', {'rack_m': ds[0], 'table_m': ds[1], 'pads_m': ds[2], 'width_m': width})
    start = time.perf_counter()
    if a.engine == 'genesis_ipc':
        from .rp_genesis_ipc import run_genesis_ipc
        traces, healths = run_genesis_ipc(sc, meta, pb, candidate, [[0,0,0]], out / 'work', objects=not a.no_objects)
        trace, health = traces[0], healths[0]
    else:
        from .rp_engines import run_mujoco
        trace, health = run_mujoco(sc, meta, pb, candidate, [0,0,0], objects=not a.no_objects)
    with gzip.open(out / 'trace.pkl.gz', 'wb') as f: pickle.dump(trace, f)
    playback = Playback(pb, sc['finger_rule'])
    pos_error, rot_error = [], []
    from scipy.spatial.transform import Rotation
    for s in trace:
        p, q, *_ = playback.target(s['t'])
        pos_error.append(np.linalg.norm(s['base'][:3] - p))
        rot_error.append(Rotation.from_matrix(qmat(q).T @ qmat(s['base'][3:])).magnitude())
    result = {'label': a.label, 'health': health, 'total_wall_s': time.perf_counter()-start,
              'tracking': {'max_base_position_error_mm': max(pos_error)*1e3,
                           'max_base_rotation_error_deg': float(np.degrees(max(rot_error)))},
              'versions': {n: importlib.metadata.version(n) for n in ['numpy','scipy']}}
    if a.engine == 'genesis_ipc':
        result['versions'].update({n:importlib.metadata.version(n) for n in ['genesis-world','pyuipc','torch','quadrants']})
    if not a.no_objects:
        real_along = -.06873756295991096 if 'v5b' in a.scene else -.07799972453363645
        result['judge'] = judge(trace, sc, meta, real_along)
    write_json(out / 'result.json', result); print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
