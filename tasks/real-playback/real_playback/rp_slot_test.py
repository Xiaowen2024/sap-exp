"""Diagnostic: can each engine hold the basket standing in the rack slot?

The basket is placed at the E2 final-frame pose fitted to the real video (bottom
centre + rotation vector, rh20t_fit.basket_parts6), lifted along the table normal
just enough to clear the rack and table, then released under gravity for 3 s
with the gripper parked far away.  The 'retained' predicate is applied at the end.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from .rp_sim import Observer, build_assets, qmat, table_pose
from .rp_task import apply_cell


def real_final_pose(fit):
    fb = fit['basket_final']['params']
    q = R.from_rotvec([fb['rx'], fb['ry'], fb['rz']]).as_quat()[[3, 0, 1, 2]]
    return np.array([fb['x'], fb['y'], fb['z']]), q


def clear_lift(scene, meta, pos, q, margin=5e-4):
    """Smallest lift along the table normal giving >= margin clearance to rack and table."""
    obs = Observer(scene, meta)
    n = table_pose(scene)[1][:, 2]
    com = np.asarray(meta['basket_com_local'])
    far = (np.array([0., 0., 5.]), np.array([1., 0, 0, 0]))
    for k in range(0, 400):
        h = k * 2.5e-4
        p = pos + n * h
        dr, dt, _ = obs.distances(p + qmat(q) @ com, q, [far, far])
        if dr >= margin and dt >= margin:
            return h, p
    raise RuntimeError('no clear pose within 100 mm')


def synth_playback(scene, t0=1000., T=3.0):
    """Gripper parked 0.4 m above the rack, open, not moving."""
    rp = np.asarray(scene['rack']['xy'] + [0.45])
    t = np.arange(0, T + 0.5, 0.008) + t0
    q = np.array([0., 1., 0., 0.])  # tool z pointing down
    pose = np.c_[np.tile(rp, (len(t), 1)), np.tile(q, (len(t), 1))]
    gt = np.arange(0, T + 0.5, 0.01) + t0
    return {'tcp_t': t, 'tcp_pose_wxyz': pose, 'grip_t': gt, 'grip_width_m': np.full(len(gt), 0.095),
            'grip_cmd_m': np.full(len(gt), 0.095)}


def judge(trace, scene, meta, start):
    obs = Observer(scene, meta)
    s = trace[-1]
    far = (np.array([0., 0., 5.]), np.array([1., 0, 0, 0]))
    dr, dt, _ = obs.distances(s['basket'][:3], s['basket'][3:], [far, far])
    ax = qmat(s['basket'][3:])[:, 2]
    n = obs.table_R[:, 2]
    lean = float(np.degrees(np.arcsin(min(1., abs(ax @ n)))))
    xr = obs.rack_pose[1][:, 0]
    h = ax - (ax @ n) * n
    align = float(np.degrees(np.arccos(min(1., abs(h @ xr) / max(np.linalg.norm(h), 1e-9)))))
    V = np.array([x['basket_v'] for x in trace[-50:]])
    rest = bool(np.all(np.linalg.norm(V[:, :3], axis=1) <= 0.01) and np.all(np.linalg.norm(V[:, 3:], axis=1) <= 0.05))
    infp = obs.in_footprint(s['basket'][:3])
    ok = bool(dr < 0.003 and infp and align <= 45 and lean <= scene['lean_max_deg'] and rest)
    disp = float(np.linalg.norm(s['basket'][:3] - start['basket'][:3]) * 1e3)
    rot = float(np.degrees((R.from_quat(s['basket'][[4, 5, 6, 3]]).inv() * R.from_quat(start['basket'][[4, 5, 6, 3]])).magnitude()))
    return {'stands_in_slot': ok, 'touch_rack': bool(dr < 0.003), 'in_footprint': bool(infp), 'lean_deg': lean,
            'axis_vs_rails_deg': align, 'at_rest': rest, 'moved_mm': disp, 'rotated_deg': rot,
            'start_lean_deg': float(np.degrees(np.arcsin(min(1., abs(qmat(start['basket'][3:])[:, 2] @ n))))),
            'final_z_mm': float(s['basket'][2] * 1e3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--engine', required=True); ap.add_argument('--scene', required=True); ap.add_argument('--fit', required=True)
    ap.add_argument('--pose', default=None, help='JSON file with a pre-cleared bottom-centre pose (pos, quat_wxyz)')
    ap.add_argument('--params', default='{}'); ap.add_argument('--label', default='native'); ap.add_argument('--out', required=True)
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    scene = apply_cell(json.loads(Path(a.scene).read_text()), {'axis': 'dt', 'dt': 1e-3} if a.engine == 'ipc' else {'axis': 'none'})
    meta = build_assets(scene, out / 'assets')
    pos, q = real_final_pose(json.loads(Path(a.fit).read_text()))
    if a.pose:
        pz = json.loads(Path(a.pose).read_text()); pos, q = np.asarray(pz['pos']), np.asarray(pz['quat_wxyz'])
    lift, pos2 = clear_lift(scene, meta, pos, q)
    scene['basket']['pose_override'] = {'pos': (pos2 + qmat(q) @ np.asarray(meta['basket_com_local'])).tolist(), 'quat_wxyz': q.tolist()}
    pbd = synth_playback(scene)
    scene['t_start'], scene['t_end'] = 1000., 1003.
    cand = {'id': a.label, 'parameters': json.loads(a.params)}
    if a.engine == 'mujoco':
        from .rp_engines import run_mujoco
        tr, h = run_mujoco(scene, meta, pbd, cand, [0, 0, 0])
    elif a.engine == 'sap':
        from .rp_sap_graph import run_sap_graph
        trs, hs = run_sap_graph(scene, meta, pbd, cand, [[0, 0, 0]], out / 'work', cap_per_world=3000)
        tr, h = trs[0], hs[0]
    else:
        from .rp_ipc import run_ipc
        tr, h = run_ipc(scene, meta, pbd, cand, [0, 0, 0], out / 'work')
    res = {'engine': a.engine, 'label': a.label, 'params': cand['parameters'], 'lift_to_clear_mm': lift * 1e3,
           'finite': h['finite'], **judge(tr, scene, meta, tr[0])}
    (out / 'slot_test.json').write_text(json.dumps(res, indent=1))
    import gzip, pickle
    with gzip.open(out / 'trace.pkl.gz', 'wb') as f:
        pickle.dump(tr, f)
    print(json.dumps(res))


if __name__ == '__main__':
    main()
