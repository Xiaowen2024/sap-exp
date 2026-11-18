"""Aggregate RH20T dish-rack playback rows into cell results and episode verdicts.

Cell: pass >= 18/20 valid+correct, fail <= 2/20, mixed otherwise (README 4.2).
Episode verdict per engine/candidate: reproduces (all cells pass), fails (all
cells fail), sensitive (otherwise; axes that differ are named).
"""
from __future__ import annotations

import argparse
import collections
import glob
import json
from pathlib import Path


def load(root):
    out = collections.defaultdict(list)  # (engine, cand, cell, episode) -> rows
    for f in glob.glob(f'{root}/*/*/*/*/rows*.json'):
        r = json.loads(Path(f).read_text())
        key = (r['engine'], r['candidate']['id'], r['cell'], r['episode'])
        out[key].extend(r['rows'])
    return out


def cell_status(rows, R=20):
    n = len({r['rollout'] for r in rows})
    k = sum(r['correct'] for r in rows)
    if n < R:
        return 'incomplete', k, n
    return ('pass' if k >= 18 else 'fail' if k <= 2 else 'mixed'), k, n


def summarize(data):
    cells = {}
    for key, rows in sorted(data.items()):
        st, k, n = cell_status(rows)
        stages = collections.Counter(r['first_failed_stage'] or 'none' for r in rows)
        flags = collections.Counter(f for r in rows for f in r['flags'])
        inval = sum(not r['valid'] for r in rows)
        cells[key] = {'status': st, 'correct': k, 'n': n, 'invalid': inval, 'first_failed_stage': dict(stages),
                      'flags': dict(flags), 'real_label': rows[0].get('real_label') if rows else None,
                      'outcomes': dict(collections.Counter(r['outcome'] for r in rows))}
    return cells


def verdicts(cells):
    by = collections.defaultdict(dict)
    for (eng, cand, cell, ep), c in cells.items():
        by[(eng, cand, ep)][cell] = c
    out = {}
    for key, cs in by.items():
        st = [c['status'] for c in cs.values()]
        if 'incomplete' in st:
            v = 'incomplete'
        elif all(s == 'pass' for s in st):
            v = 'reproduces'
        elif all(s == 'fail' for s in st):
            v = 'fails'
        else:
            v = 'sensitive'
        out[key] = {'verdict': v, 'cells_run': len(cs), 'by_status': dict(collections.Counter(st)),
                    'differing': sorted(c for c, x in cs.items() if x['status'] != cs.get('nominal', {}).get('status'))}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', required=True); ap.add_argument('--out', required=True)
    a = ap.parse_args()
    cells = summarize(load(a.root))
    ver = verdicts(cells)
    res = {'cells': {'|'.join(k): v for k, v in cells.items()}, 'verdicts': {'|'.join(k): v for k, v in ver.items()}}
    Path(a.out).write_text(json.dumps(res, indent=1))
    for k, v in sorted(cells.items()):
        print('%-7s %-16s %-18s %s  %-10s %2d/%-2d inv=%d stages=%s flags=%s' % (k[0], k[1], k[2], k[3][15:29], v['status'], v['correct'], v['n'], v['invalid'], v['first_failed_stage'], v['flags']))
    for k, v in sorted(ver.items()):
        print('VERDICT', k, v['verdict'], v['by_status'])


if __name__ == '__main__':
    main()
