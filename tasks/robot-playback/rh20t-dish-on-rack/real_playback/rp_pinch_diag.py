"""Diagnostic (not a benchmark cell): isolated pinch-and-lift of the completed object.

Same pads, finger force law, object geometry/mass/friction and engines as the
playback, but a synthetic gripper motion: vertical gripper, wall centred
between the pads, close, lift 100 mm in 1 s, hold.  Separates "the engine
cannot hold this thin wall under this grip force" from "the replay /
reconstruction puts the grasp in the wrong place".
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from .rp_sim import build_assets, place_on_table, qmat, table_pose, Observer


def synth(scene, h_tcp=0.004, open_w=0.03, lift=0.10):
    """Synthetic playback dict and timings around the object's +x wall point."""
    b = scene['basket']
    pos0, q0 = place_on_table(scene, b['xy'], b['yaw'])
    Rb = qmat(q0)
    _, tR = table_pose(scene)
    n = tR[:, 2]
    # wall mid-surface at angle 0 (local +x), height of the pad overlap centre
    hc = h_tcp + 0.016
    r_out = (b['k'] + (1 - b['k']) * hc / b['H']) * b['a']
    wall = pos0 + Rb @ np.array([r_out - b['wall_t'] / 2, 0., 0.])
    u = Rb[:, 0]
    tcp0 = wall - (wall - pos0) @ n * n + n * h_tcp + (pos0 @ n) * n  # same xy, TCP h_tcp above the bottom plane
    tcp0 = wall + n * (h_tcp - (wall - pos0) @ n)
    z = -n; y = u - (u @ z) * z; y /= np.linalg.norm(y); x = np.cross(y, z)
    q = R.from_matrix(np.c_[x, y, z]).as_quat()[[3, 0, 1, 2]]
    t = np.arange(0, 4.0 + 1e-9, 0.008)
    s = np.clip((t - 1.2) / 1.0, 0, 1)
    s = s * s * (3 - 2 * s)
    P = tcp0[None] + n[None] * (lift * s)[:, None]
    gt = np.arange(0, 4.0 + 1e-9, 0.01)
    w = np.where(gt < 0.5, open_w, np.maximum(b['wall_t'], open_w - (gt - 0.5) / 0.1 * (open_w - b['wall_t'])))
    cmd = np.where(gt < 0.5, open_w, 0.)
    pbd = {'tcp_t': t + 1000., 'tcp_pose_wxyz': np.c_[P, np.tile(q, (len(t), 1))],
           'grip_t': gt + 1000., 'grip_width_m': w, 'grip_cmd_m': cmd}
    return pbd, 1000.0, 1004.0


def metrics(tr, scene, meta, t0):
    obs = Observer(scene, meta)
    z0 = tr[0]['basket'][:3] @ table_pose(scene)[1][:, 2]
    out = {}
    for name, tt in (('after_close', 1.1), ('end', 3.99)):
        s = min(tr, key=lambda x: abs(x['t'] - (t0 + tt)))
        dr, dt, dp = obs.distances(s['basket'][:3], s['basket'][3:], [(p[:3], p[3:]) for p in s['pads']])
        n = table_pose(scene)[1][:, 2]
        out[name] = {'object_rise_mm': float((s['basket'][:3] @ n - z0) * 1e3),
                     'base_rise_mm': float(((s['base'][:3] - tr[0]['base'][:3]) @ n) * 1e3),
                     'pad_separation_mm': float(sum(s['slider']) * 1e3),
                     'pad_overlap_mm': float(max(0., -min(dp)) * 1e3)}
    e = out['end']
    out['held'] = bool(e['object_rise_mm'] > 0.8 * e['base_rise_mm'])
    out['full_closure'] = bool(e['pad_separation_mm'] < 0.5)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', required=True); ap.add_argument('--engine', required=True, choices=['mujoco', 'sap'])
    ap.add_argument('--candidates', default='native'); ap.add_argument('--forces', default='102.5,45')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    base = json.loads(Path(a.scene).read_text())
    from .rp_run import CANDIDATES
    rows = []
    for F in [float(x) for x in a.forces.split(',')]:
        sc = json.loads(json.dumps(base))
        sc['drive']['F_close'] = F
        pbd, t0, t1 = synth(sc)
        sc['t_start'], sc['t_end'] = t0, t1
        sc['finger_rule'] = {**sc['finger_rule'], 'min_hold_s': 0.3}
        from .rp_task import apply_cell
        sc = apply_cell(sc, {'axis': 'none'})
        meta = build_assets(sc, out / 'assets')
        for c in a.candidates.split(','):
            cand = {'id': c, 'parameters': CANDIDATES[a.engine][c]}
            if a.engine == 'mujoco':
                from .rp_engines import run_mujoco
                tr, h = run_mujoco(sc, meta, pbd, cand, [0, 0, 0])
            else:
                from .rp_sap_graph import run_sap_graph
                trs, hs = run_sap_graph(sc, meta, pbd, cand, [[0, 0, 0], [0, 0, 0]], out / 'work', cap_per_world=3000)
                tr, h = trs[0], hs[0]
            m = metrics(tr, sc, meta, t0)
            row = {'engine': a.engine, 'candidate': c, 'F_close_N': F, 'wall_mm': sc['basket']['wall_t'] * 1e3,
                   'object': sc['object_class'], 'finite': h['finite'], **m}
            rows.append(row)
            print(json.dumps(row), flush=True)
    (out / f'pinch_{a.engine}.json').write_text(json.dumps(rows, indent=1))


if __name__ == '__main__':
    main()
