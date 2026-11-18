"""Runner CLI for RH20T dish-on-rack playback (P3a drive gate, P3b smoke, P4, P5).

  python -m real_playback.rp_run drive --engine mujoco --scene S.json --playback P.npz --out DIR
  python -m real_playback.rp_run cell  --engine sap --scene S.json --playback P.npz --cell nominal \
         --candidate native --R 20 --seed 91100 --out DIR
"""
from __future__ import annotations

import argparse
import gzip
import json
import pickle
import time
from pathlib import Path

import numpy as np

from .rp_sim import build_assets, qmat, qmul, qconj, qrotvec
from .rp_task import apply_cell, evaluate, events

CANDIDATES = {
    'mujoco': {
        'native': {},
        'tc2_imp50': {'solref': [0.002, 1], 'impratio': 50},
        'tc4_imp50': {'solref': [0.004, 1], 'impratio': 50},
        'tc2_imp100': {'solref': [0.002, 1], 'impratio': 100},
        'tc2_imp50_ns2': {'solref': [0.002, 1], 'impratio': 50, 'noslip_iterations': 2},
    },
    'sap': {
        'native': {},
        'ke3e4_tau3': {'ke': 3e4, 'tau': 3e-3},
        'ke1e5_tau3': {'ke': 1e5, 'tau': 3e-3},
        'ke3e5_tau3': {'ke': 3e5, 'tau': 3e-3},
        'ke1e5_tau3_s1e-4': {'ke': 1e5, 'tau': 3e-3, 'sigma': 1e-4},
    },
}

CELLS = {
    'nominal': {'axis': 'none'},
    'dt_small': {'axis': 'dt', 'dt': 2.5e-4}, 'dt_large': {'axis': 'dt', 'dt': 1e-3},
    'mass_low': {'axis': 'mass', 'level': 'low'}, 'mass_high': {'axis': 'mass', 'level': 'high'},
    'mu_pad_low': {'axis': 'mu_pad', 'level': 'low'}, 'mu_pad_high': {'axis': 'mu_pad', 'level': 'high'},
    'mu_env_low': {'axis': 'mu_env', 'level': 'low'}, 'mu_env_high': {'axis': 'mu_env', 'level': 'high'},
    'grip_low': {'axis': 'grip', 'level': 'low'}, 'grip_high': {'axis': 'grip', 'level': 'high'},
    'coupling_soft': {'axis': 'coupling', 'level': 'low'}, 'coupling_stiff': {'axis': 'coupling', 'level': 'high'},
    'profile_shallow': {'axis': 'profile_H', 'level': 'low'}, 'profile_rolled': {'axis': 'profile', 'level': 'rolled'},
    'basket_pose_minus': {'axis': 'basket_pose', 'level': 'minus'}, 'basket_pose_plus': {'axis': 'basket_pose', 'level': 'plus'},
    'rack_pose_minus': {'axis': 'rack_pose', 'level': 'minus'}, 'rack_pose_plus': {'axis': 'rack_pose', 'level': 'plus'},
}


def perturbations(seed, R):
    rng = np.random.default_rng(seed)
    return np.c_[rng.uniform(-5e-4, 5e-4, (R, 2)), np.deg2rad(rng.uniform(-.5, .5, R))]


def cell_scene(scene, cell_id):
    c = CELLS[cell_id]
    if c['axis'] == 'profile_H':
        s = json.loads(json.dumps(scene))
        s['basket']['H'] = s['ranges']['H']['low']
        return apply_cell(s, {'axis': 'none'})
    return apply_cell(scene, c)


def tracking(trace, pbdata, scene):
    from .rp_sim import Playback
    pb = Playback(pbdata, scene['finger_rule'])
    ep, er = [], []
    for s in trace:
        tp, tq, *_ = pb.target(s['t'])
        ep.append(np.linalg.norm(s['base'][:3] - tp))
        er.append(np.linalg.norm(qrotvec(qmul(tq, qconj(s['base'][3:])))))
    ep, er = np.array(ep), np.array(er)
    return {'max_pos_err_mm': float(ep.max() * 1e3), 'p99_pos_err_mm': float(np.percentile(ep, 99) * 1e3),
            'max_rot_err_deg': float(np.degrees(er.max())), 'p99_rot_err_deg': float(np.degrees(np.percentile(er, 99))),
            'passed': bool(ep.max() <= 1e-3 and np.degrees(er.max()) <= 0.5)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['drive', 'cell'])
    ap.add_argument('--engine', required=True, choices=['mujoco', 'sap', 'ipc', 'genesis'])
    ap.add_argument('--t-max', type=float, default=None, help='debug: truncate the replay window (s)')
    ap.add_argument('--scene', required=True); ap.add_argument('--playback', required=True)
    ap.add_argument('--cell', default='nominal'); ap.add_argument('--candidate', default='native')
    ap.add_argument('--R', type=int, default=20); ap.add_argument('--seed', type=int, default=91100)
    ap.add_argument('--rollouts', type=str, default=None, help='comma list of rollout indices (MuJoCo sharding)')
    ap.add_argument('--keep-traces', action='store_true')
    ap.add_argument('--params', type=str, default=None, help='JSON of native parameters (adaptive tuning iteration); overrides --candidate')
    ap.add_argument('--no-perturb', action='store_true', help='single deterministic run without the numerical perturbation')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    scene0 = json.loads(Path(a.scene).read_text())
    pbd = dict(np.load(a.playback))
    if a.engine == 'ipc':
        from .rp_ipc import CANDIDATES as IPC_C
        CANDIDATES['ipc'] = IPC_C
    if a.engine == 'genesis':
        from .rp_genesis import CANDIDATES as GEN_C
        CANDIDATES['genesis'] = GEN_C
    if a.params:
        cand = {'id': a.candidate, 'parameters': json.loads(a.params)}
    else:
        cand = {'id': a.candidate, 'parameters': CANDIDATES[a.engine][a.candidate]}
    if a.t_max:
        scene0 = json.loads(json.dumps(scene0)); scene0['t_end'] = min(scene0['t_end'], scene0['t_start'] + a.t_max)
    if a.cmd == 'drive':
        scene = cell_scene(scene0, 'nominal')
        meta = build_assets(scene, out / 'assets')
        if a.engine == 'mujoco':
            from .rp_engines import run_mujoco
            tr, h = run_mujoco(scene, meta, pbd, cand, [0, 0, 0], objects=False)
            trs = [tr]
        elif a.engine == 'ipc':
            from .rp_ipc import run_ipc
            tr, h = run_ipc(scene, meta, pbd, cand, [0, 0, 0], out / 'work', objects=False)
            trs = [tr]
        elif a.engine == 'genesis':
            from .rp_genesis import run_genesis
            trs, hs = run_genesis(scene, meta, pbd, cand, [[0, 0, 0]], out / 'work', objects=False)
            h = hs[0]
        else:
            from .rp_sap_graph import run_sap_graph
            trs, hs = run_sap_graph(scene, meta, pbd, cand, [[0, 0, 0]], out / 'work', objects=False)
            h = hs[0]
        r = {'phase': 'P3a', 'engine': a.engine, 'episode': scene['episode'], 'objects': False,
             **tracking(trs[0], pbd, scene), 'health': h}
        (out / 'drive.json').write_text(json.dumps(r, indent=1, default=float))
        print(json.dumps({k: v for k, v in r.items() if k != 'health'}, default=float))
        return
    scene = cell_scene(scene0, a.cell)
    meta = build_assets(scene, out / 'assets')
    ev = scene0['events']
    ev = {**ev, 'segments': scene0['segments']}
    P = perturbations(a.seed, a.R)
    if a.no_perturb:
        P = np.zeros_like(P)
    idx = list(range(a.R)) if a.rollouts is None else [int(x) for x in a.rollouts.split(',')]
    rows = []
    if a.engine in ('mujoco', 'ipc'):
        if a.engine == 'mujoco':
            from .rp_engines import run_mujoco as runner
        else:
            from .rp_ipc import run_ipc
            runner = lambda sc_, me_, pd_, ca_, pr_: run_ipc(sc_, me_, pd_, ca_, pr_, out / f'work_r{i:02d}')
        for i in idx:
            tr, h = runner(scene, meta, pbd, cand, P[i])
            row = evaluate(tr, h, scene, meta, ev)
            row.update(rollout=i, perturb=P[i].tolist(), wall_s=h['wall_s'], steps=h['steps'])
            rows.append(row)
            if a.keep_traces and i == idx[0]:
                with gzip.open(out / f'trace_r{i:02d}.pkl.gz', 'wb') as f:
                    pickle.dump(tr, f)
            print(i, row['outcome'], row['first_failed_stage'], row['flags'], '%.0fs' % h['wall_s'], flush=True)
    else:
        if a.engine == 'genesis':
            from .rp_genesis import run_genesis
            trs, hs = run_genesis(scene, meta, pbd, cand, [P[i] for i in idx], out / 'work')
        else:
            from .rp_sap_graph import run_sap_graph
            trs, hs = run_sap_graph(scene, meta, pbd, cand, [P[i] for i in idx], out / 'work', cap_per_world=3000)
        for j, i in enumerate(idx):
            row = evaluate(trs[j], hs[j], scene, meta, ev)
            row.update(rollout=i, perturb=P[i].tolist(), wall_s=hs[j]['wall_s'], steps=hs[j]['steps'])
            rows.append(row)
            print(i, row['outcome'], row['first_failed_stage'], row['flags'], flush=True)
        if a.keep_traces:
            with gzip.open(out / f'trace_r{idx[0]:02d}.pkl.gz', 'wb') as f:
                pickle.dump(trs[0], f)
    res = {'engine': a.engine, 'episode': scene['episode'], 'cell': a.cell, 'candidate': cand,
           'seed': a.seed, 'R': a.R, 'real_label': scene['real_label'], 'completion_revision': scene['completion_revision'],
           'rows': rows, 'n_correct': sum(r['correct'] for r in rows), 'n_valid': sum(r['valid'] for r in rows)}
    name = 'rows.json' if a.rollouts is None else f'rows_{idx[0]:02d}.json'
    (out / name).write_text(json.dumps(res, indent=1, default=float))


if __name__ == '__main__':
    main()
