#!/usr/bin/env python3
"""Trial-matched GAUGE cradle/pendulum, using the released meshes and real metric references.

Works with separately pinned MuJoCo/MuJoCo Warp 3.17 and 4.0 environments.
The contact-free pendulum's CUDA graph is checked against uncaptured steps.
Cradle uses uncaptured native steps: contact-order sensitivity was seen in preflight.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import copy
import hashlib
import importlib.metadata as metadata
import json
import math
from pathlib import Path
import time
from xml.etree import ElementTree as ET

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

import gauge_cradle_v4_common as gc
import gauge_pendulum_v3_common as gp
from mujoco_capacity_audit import CapacityAudit
from strict_v3_common import json_safe, load_manifest

ROOT = REPO_ROOT
ASSETS = ROOT / 'exp-results/gauge-rigid-reproduction/assets'


@wp.kernel
def sample_state(q: wp.array2d(dtype=float), v: wp.array2d(dtype=float),
                 qs: wp.array3d(dtype=float), vs: wp.array3d(dtype=float), frame: int):
    w, j = wp.tid()
    qs[frame, w, j] = q[w, j]
    vs[frame, w, j] = v[w, j]


def parse_candidate(task, text):
    if text == 'native default':
        return {'id': 'native_default', 'native': True}
    if task == 'pendulum':
        return {'id': text, 'native': False, 'integrator': text.split('=')[1]}
    d = dict(s.split('=') for s in text.split(','))
    return {'id': text, 'native': False, 'tc': float(d['tc'].removesuffix('ms')) * .001,
            'dr': float(d['dr'])}


def model_xml(task, candidate, cell):
    mod = gc if task == 'cradle' else gp
    tree = ET.parse(ASSETS / 'source-gauge' / (task + '.xml')).getroot()
    ET.SubElement(tree, 'compiler', meshdir=str(ASSETS))
    tree.find('asset/mesh').set('file', Path(mod.MESH).name)
    sps = round((1000 / mod.FPS) / cell['dt_ms'])
    dt = 1 / (mod.FPS * sps)
    if abs(dt - cell['dt_ms'] * .001) > 1e-8:
        raise ValueError('Declared dt is not compatible with the 30-Hz sample clock')
    option = ET.SubElement(tree, 'option', timestep=str(dt), gravity='0 0 -9.81')
    if task == 'pendulum' and not candidate['native']:
        option.set('integrator', candidate['integrator'])
    world = tree.find('worldbody')
    prototype = copy.deepcopy(world.find('body'))
    world.remove(world.find('body'))
    anchors = gc.ANCHORS if task == 'cradle' else [gp.ANCHOR]
    for i, anchor in enumerate(anchors):
        pivot = copy.deepcopy(prototype)
        pivot.set('name', f'pivot{i}')
        pivot.set('pos', ' '.join(map(str, anchor)))
        pivot.find('joint').set('name', f'h{i}')
        ball = pivot.find('body'); ball.set('name', f'ball{i}')
        geom = ball.find('geom'); geom.set('name', f'ball_geom{i}')
        geom.set('mass', str(mod.MASS * cell.get('mass_scale', 1.0)))
        geom.set('friction', '0.01 0 0')
        if task == 'cradle' and not candidate['native']:
            geom.set('solref', f"{candidate['tc']} {candidate['dr']}")
        if task == 'pendulum':
            geom.set('contype', '0'); geom.set('conaffinity', '0')
        world.append(pivot)
    return ET.tostring(tree, encoding='unicode'), sps


def simulate(task, candidate, cell, angles, *, graph=None):
    if graph is None:
        graph = task == 'pendulum'
    xml, sps = model_xml(task, candidate, cell)
    model = mujoco.MjModel.from_xml_string(xml)
    gm = mjw.put_model(model); W, S = len(angles), cell['frames']
    d = mjw.make_data(model, nworld=W, nconmax=32, njmax=128)
    init = np.tile(model.qpos0, (W, 1)).astype(np.float32)
    init[:, 0] = angles; d.qpos.assign(init); mjw.forward(gm, d)
    audit = CapacityAudit(d, W); audit.record(d)
    Q = wp.zeros((S, W, model.nq), dtype=float)
    V = wp.zeros((S, W, model.nv), dtype=float)

    def frame_step():
        for _ in range(sps):
            mjw.step(gm, d)
            audit.record(d)

    if graph:
        with wp.ScopedCapture() as capture:
            frame_step()
    for frame in range(S):
        wp.launch(sample_state, dim=(W, model.nq), inputs=[d.qpos, d.qvel, Q, V, frame])
        if frame < S - 1:
            if graph:
                wp.capture_launch(capture.graph)
            else:
                frame_step()
    q, v = Q.numpy(), V.numpy()
    bad, health = audit.result()
    body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f'ball{i}')
                for i in range(model.nq)]
    compiled = {'dt_s': model.opt.timestep, 'steps_per_frame': sps,
                'integrator': int(model.opt.integrator), 'solver': int(model.opt.solver),
                'cone': int(model.opt.cone), 'impratio': model.opt.impratio,
                'iterations': model.opt.iterations, 'tolerance': model.opt.tolerance,
                'dof_damping': model.dof_damping.tolist(),
                'geom_solref': model.geom_solref.tolist(), 'geom_solimp': model.geom_solimp.tolist(),
                'geom_friction': model.geom_friction.tolist(),
                'body_mass': model.body_mass[body_ids].tolist(),
                'body_inertia': model.body_inertia[body_ids].tolist(),
                'hinge_inertia': model.dof_M0.tolist(),
                'body_inertial_offset': model.body_ipos[body_ids].tolist(),
                'xml_sha256': hashlib.sha256(xml.encode()).hexdigest(), 'xml': xml,
                'dispatch': 'captured_one_sample_interval' if graph else 'uncaptured_native_steps'}
    health['capacity_ok_by_world'] = (~bad).tolist()
    return q, v, np.asarray(health['finite_by_world']), health, compiled


def real_reference(task, trial, frames):
    if task == 'cradle':
        return gc.real_reference(trial, frames)
    p = trial['p'][:frames]
    return {'pd_s': gp.period_fft(p[:300, 0]), 'pd_fine_s': gp.period_fft(p[:300, 0], refine=True),
            'el': gp.energy_loss(p[:, 2], gp.ANCHOR[2] - gp.L)}


def summarize(rows):
    keys = sorted({k for r in rows for k, v in r.items()
                   if isinstance(v, (int, float)) and not isinstance(v, bool)
                   and k not in ('trial', 'repeat')})
    result = {}
    for key in keys:
        values = [r[key] for r in rows if r.get(key) is not None and math.isfinite(r[key])]
        result[key] = {'mean': float(np.mean(values)) if values else None,
                       'sd': float(np.std(values, ddof=1)) if len(values) > 1 else None,
                       'n': len(values)}
    return result


def run_cell(task, candidate, cell, trials, seed, R, raw_path):
    start = time.perf_counter(); mod = gc if task == 'cradle' else gp
    chosen = [t for t in trials if t['id'] in mod.trial_ids(cell['trials'])]
    rng = np.random.default_rng(seed)
    angles = np.concatenate([mod.release_angle(t) + rng.uniform(-.003, .003, R) for t in chosen])
    q, v, finite, health, compiled = simulate(task, candidate, cell, angles)
    raw_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(raw_path, qpos=q, qvel=v, initial_angles=angles,
                        trial_ids=np.repeat([t['id'] for t in chosen], R), repeat_ids=np.tile(np.arange(R), len(chosen)))
    rows = []; examples = []
    if task == 'cradle':
        # Same sampled mechanical-energy diagnostic as v4, using the compiled hinge inertia.
        mass = gc.MASS * cell.get('mass_scale', 1.0)
        energy = (mass * 9.81 * gc.L * (1 - np.cos(q)) +
                  .5 * np.asarray(compiled['hinge_inertia'])[None, None, :] * v ** 2).sum(axis=-1)
    for j, trial in enumerate(chosen):
        ref = real_reference(task, trial, cell['frames'])
        for r in range(R):
            w = j * R + r
            row = {'trial': trial['id'], 'repeat': r, 'finite': bool(finite[w]),
                   'capacity_ok': health['capacity_ok_by_world'][w], 'valid': False}
            if finite[w] and row['capacity_ok']:
                if task == 'cradle':
                    pa, pp = gc.positions(q[:, w], 0), gc.positions(q[:, w], 4)
                    mte, apex = gc.first_impact_mte(pa[:, 2], pp[:, 2], gc.Z_EQ)
                    er = float(np.max(energy[:, w]) / energy[0, w])
                    row.update(mte=mte, mte_real=ref['mte'], mte_ratio=mte / ref['mte'],
                               mte_abs_error=abs(mte - ref['mte']), apex_frame=apex,
                               lsd_active_s=gc.lsd(pa), lsd_passive_s=gc.lsd(pp),
                               lsd_real_active_s=ref['lsd_active_s'], lsd_real_passive_s=ref['lsd_passive_s'],
                               max_sampled_energy_ratio=er, valid=er <= 1.05,
                               active_rmse_mm=float(np.sqrt(np.mean(np.sum((pa-trial['ball_active'][:len(pa)])**2, axis=1)))*1000),
                               passive_rmse_mm=float(np.sqrt(np.mean(np.sum((pp-trial['ball_passive'][:len(pp)])**2, axis=1)))*1000),
                               loss=abs(mte-ref['mte']))
                    if r == 0:
                        examples.append({'trial': trial['id'], 'sim_active': pa.tolist(), 'sim_passive': pp.tolist()})
                else:
                    p = gp.positions(q[:, w, 0]); pd = gp.period_fft(p[:300, 0])
                    pdf = gp.period_fft(p[:300, 0], refine=True)
                    el = gp.energy_loss(p[:, 2], gp.ANCHOR[2]-gp.L)
                    valid = el is not None and ref['el'] is not None
                    n = min(300, len(p), len(trial['p']))
                    row.update(pd_s=pd, pd_real_s=ref['pd_s'], pd_ratio=pd/ref['pd_s'],
                               pd_fine_s=pdf, pd_fine_real_s=ref['pd_fine_s'],
                               el=el, el_real=ref['el'], el_abs_error=abs(el-ref['el']) if valid else None,
                               rmse_300_mm=float(np.sqrt(np.mean(np.sum((p[:n]-trial['p'][:n])**2, axis=1)))*1000),
                               valid=valid, loss=abs(pd/ref['pd_s']-1)+abs(el-ref['el']) if valid else None)
                    if r == 0:
                        examples.append({'trial': trial['id'], 'sim': p.tolist()})
            rows.append(row)
    valid = [r for r in rows if r['valid']]
    return {'cell': cell, 'candidate': candidate, 'seed': seed, 'R': R, 'n': len(rows),
            'valid_count': len(valid), 'gate': len(valid) == len(rows),
            'loss': float(np.mean([r['loss'] for r in valid])) if len(valid) == len(rows) else None,
            'metrics': summarize(rows), 'rows': rows, 'examples': examples,
            'capacity_audit': health, 'compiled': compiled,
            'raw_npz': raw_path.name, 'raw_sha256': hashlib.sha256(raw_path.read_bytes()).hexdigest(),
            'reference_scope': cell['reference_scope'], 'wall_s': time.perf_counter()-start}


def save(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(json_safe(data), indent=1) + '\n')
    tmp.replace(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--task', choices=['cradle', 'pendulum'], required=True)
    ap.add_argument('--manifest', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--smoke', action='store_true')
    ap.add_argument('--expected-version', choices=['3.17.0', '4.0.0'], required=True)
    a = ap.parse_args(); man, sha = load_manifest(a.manifest)
    if mujoco.__version__ != a.expected_version or metadata.version('mujoco-warp') != a.expected_version:
        raise ValueError('MuJoCo and MuJoCo Warp must both match the declared version')
    for name, expected in man['data']['file_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != expected:
            raise ValueError('GAUGE source changed: ' + name)
    mod = gc if a.task == 'cradle' else gp
    trials = mod.load_trials(ROOT)
    candidates = [parse_candidate(a.task, t) for t in man['candidate_rule']['mujoco_warp_candidates']]
    if not wp.is_cuda_available():
        raise RuntimeError('CUDA is required for evaluation')
    wp.set_device('cuda:0')
    if a.smoke:
        cell = dict(man['selection'], frames=40)
        angle = np.array([mod.release_angle(trials[0])-.001, mod.release_angle(trials[0])+.001])
        for c in (candidates[0], candidates[-1]):
            result = simulate(a.task, c, cell, angle)
            np.testing.assert_allclose(result[0][0, :, 0], angle, atol=1e-7)
            if a.task == 'pendulum':
                assert result[2].all() and all(result[3]['capacity_ok_by_world'])
                slow = simulate(a.task, c, cell, angle, graph=False)
                np.testing.assert_allclose(result[0], slow[0], rtol=1e-5, atol=1e-6)
                np.testing.assert_allclose(result[1], slow[1], rtol=1e-5, atol=1e-6)
            else:
                print('Cradle smoke health', c['id'], result[3], flush=True)
        print('PASS initial-state/API smoke', a.task, mujoco.__version__,
              '; pendulum graph equivalence verified; cradle candidate health is scored in the full screen', flush=True)
        return
    a.out.mkdir(parents=True, exist_ok=True)
    files = [ROOT / mod.MESH, ASSETS/'source-gauge'/(a.task+'.xml'),
             ASSETS/'source-gauge'/("newton's cradle.json" if a.task == 'cradle' else 'pendulum.json')]
    files += sorted((ROOT/mod.DATA).glob('*.json'))
    data = {'protocol': man['protocol'], 'task': man['task'], 'revision': man['revision'],
            'manifest_sha256': sha, 'engine': 'MuJoCo Warp', 'complete': False,
            'mujoco_version': mujoco.__version__, 'mujoco_warp_version': metadata.version('mujoco-warp'),
            'warp_version': metadata.version('warp-lang'), 'device': str(wp.get_device()),
            'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
            'real_reference': [{'trial': t['id'], **real_reference(a.task,t,man['selection']['frames'])} for t in trials],
            'selection_candidates': [], 'frozen_candidate': None, 'heldout': []}
    path = a.out / f'mjwarp_gauge_{a.task}_real.json'
    for i, candidate in enumerate(candidates):
        r = run_cell(a.task, candidate, man['selection'], trials, man['seed0'], man['R'], a.out/f'selection_{i+1}.npz')
        r['candidate_index'] = i+1; data['selection_candidates'].append(r); save(path,data)
        print('selection',i+1,candidate,'valid',r['valid_count'],'/',r['n'],'loss',r['loss'],flush=True)
    valid = [r for r in data['selection_candidates'] if r['gate'] and r['loss'] is not None]
    if not valid:
        data['stop_reason']='No valid selection candidate; held-out evaluation not started'
        data['selection_complete']=True
        data['heldout_status']='not_run_no_valid_frozen_candidate'
        save(path,data)
        print(data['stop_reason'],flush=True)
        return
    winner=min(valid,key=lambda r:(r['loss'],r['wall_s']))
    data['frozen_candidate']=winner['candidate']; data['frozen_index']=winner['candidate_index']; save(path,data)
    for j, cell in enumerate(man['held_out_conditions']):
        r=run_cell(a.task,winner['candidate'],cell,trials,man['seed0']+1000+j,man['R'],a.out/f'heldout_{j+1}.npz')
        data['heldout'].append(r); save(path,data)
        print('heldout',cell['id'],'valid',r['valid_count'],'/',r['n'],'loss',r['loss'],flush=True)
    data['complete']=True; save(path,data)


if __name__ == '__main__':
    main()
