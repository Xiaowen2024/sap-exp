"""Adaptive tuning loop (user rule 2026-11-18): at most 5 runs per engine; run 1 is
the native default; each next setting is chosen from the previous run's evidence
by ONE rule set shared by all engines (written down before any run, because the
loop has to run unattended):

  1. invalid / ejected                -> more dissipation / tighter solver
  2. penetration > 1 mm (any pair)    -> stiffer contact, scaled by the overlap
  3. slip > 10 mm, or not carried     -> harder friction / stiction
  4. otherwise (fails later stages)   -> harder friction first, then stiffer contact
  stop early when the run reproduces the real label.

Only native numerical parameters change.  Geometry, mass, friction coefficients,
grip force, trajectory and dt never change.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path

DT = {'mujoco': 5e-4, 'sap': 5e-4, 'genesis': 5e-4, 'ipc': 1e-3}


def evidence(row):
    pen = max(row['observer_peak_overlap_mm'].values())
    return {'invalid': not row['valid'] or 'ejected' in row['flags'], 'pen_mm': pen,
            'slip_mm': row.get('slip_mm', 0.) or 0., 'stage': row['first_failed_stage'],
            'carried': row['stage']['carried'], 'correct': row['correct']}


def next_params(engine, p, ev, history):
    p = json.loads(json.dumps(p))
    tried = [h['rule'] for h in history]
    if ev['invalid']:
        rule = 'dissipation'
    elif ev['pen_mm'] > 1.0:
        rule = 'stiffer'
    elif ev['slip_mm'] > 10 or not ev['carried']:
        rule = 'friction'
    else:
        rule = 'friction' if tried.count('friction') <= tried.count('stiffer') else 'stiffer'
    f = min(max(ev['pen_mm'] / 0.5, 3.), 100.)
    if engine == 'mujoco':
        tc, dr = p.get('solref', [0.02, 1.0])
        if rule == 'stiffer':
            tc = max(tc / f ** 0.5, 2 * DT[engine]); p['solref'] = [tc, dr]
        elif rule == 'friction':
            if p.get('impratio', 1) < 100:
                p['impratio'] = p.get('impratio', 1) * 10
            else:
                p['noslip_iterations'] = max(2, 2 * p.get('noslip_iterations', 0))
        else:
            p['solref'] = [tc, dr * 2]
    elif engine == 'sap':
        if rule == 'stiffer':
            p['ke'] = p.get('ke', 1250.) * f          # default pair ke = harmonic(2500, 2500)
        elif rule == 'friction':
            p['sigma'] = p.get('sigma', 1e-3) / 10
            s = p.setdefault('solver', {}); s['contact_stiction_tolerance'] = s.get('contact_stiction_tolerance', 1e-4) / 10
        else:
            p['tau'] = p.get('tau', 0.01) * 3
    elif engine == 'ipc':
        if rule == 'friction':
            if p.get('newton_tol', 0.05) > 1e-4:
                p['newton_tol'] = 1e-4
            p['eps_v'] = p.get('eps_v', 0.01) / 10
        elif rule == 'stiffer':
            p['d_hat'] = p.get('d_hat', 0.01) / 10
        else:
            p['newton_tol'] = p.get('newton_tol', 0.05) / 10
    elif engine == 'genesis':
        p.setdefault('n_sap_iterations', 5)
        if rule == 'stiffer':
            p['hydroelastic_stiffness'] = p.get('hydroelastic_stiffness', 1e8) * f
        elif rule == 'friction':
            p['n_sap_iterations'] = max(20, 2 * p['n_sap_iterations'])
            p['sap_sigma'] = p.get('sap_sigma', 1e-3) / 10
        else:
            p['sap_taud'] = p.get('sap_taud', 0.1) * 3
    return p, rule


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--engine', required=True); ap.add_argument('--root', default='.')
    ap.add_argument('--iters', type=int, default=5)
    a = ap.parse_args()
    root = Path(a.root).resolve()
    logp = root / f'results/tune/{a.engine}/tuning_log.json'
    logp.parent.mkdir(parents=True, exist_ok=True)
    history, p = [], {}
    for it in range(1, a.iters + 1):
        t0 = time.time()
        subprocess.run(['bash', str(root / 'tune_run.sh'), a.engine, str(it), json.dumps(p)], cwd=root)
        rows = root / f'results/tune/{a.engine}/iter{it}/rows.json'
        if not rows.exists():
            history.append({'iter': it, 'params': p, 'error': 'no rows (crash); see log', 'wall_s': time.time() - t0, 'rule': 'none'})
            logp.write_text(json.dumps(history, indent=1))
            break
        row = json.loads(rows.read_text())['rows'][0]
        logtxt = (root / f'results/tune/{a.engine}/iter{it}/log.txt').read_text(errors='replace')
        if 'Contact buffer overflowed' in logtxt:   # SAP pipeline truncation: dropped contacts -> invalid run
            row['valid'] = False; row['correct'] = False
            row['flags'] = row['flags'] + ['contact_overflow']
        ev = evidence(row)
        rec = {'iter': it, 'params': p, 'outcome': row['outcome'], 'first_failed_stage': row['first_failed_stage'],
               'flags': row['flags'], 'evidence': ev, 'wall_s': time.time() - t0}
        if ev['correct']:
            rec['rule'] = 'stop: reproduces the real label'
            history.append(rec); logp.write_text(json.dumps(history, indent=1)); break
        p_next, rule = next_params(a.engine, p, ev, history)
        rec['rule'] = rule; rec['next_params'] = p_next
        history.append(rec); logp.write_text(json.dumps(history, indent=1))
        p = p_next
    (logp.parent / 'DONE').write_text('done\n')


if __name__ == '__main__':
    main()
