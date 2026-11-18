#!/usr/bin/env python3
"""Second-GPU helper for a Genesis SAP task already running elsewhere (execution only; no scoring here).

Issues exactly the same isolated calls as the task's own runner (same module, function, candidate dicts, cells, seeds,
R and buffer environment), so results land in the shared call cache (/dev/shm/genesis_sap_call_cache) under the same
keys.  It walks each phase in REVERSE order while the primary runner walks forward; a bidirectional cache sync makes
the primary pick up these results as cache hits.  The frozen candidate is computed with the task's own choose rule
from the cached selection rows, so both machines freeze identically.  The task's result JSON is still written only by
the primary runner, with the original scoring code.

Assignment rule (fixed before any held-out result): a cell's result is whichever machine's result is in the cache
when the primary runner reaches it.  Duplicated cells (both machines computed) are kept as a cross-GPU check.
"""
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import argparse, json, os, socket, time
from pathlib import Path

os.environ.setdefault('SAP_BUFFER_CAPACITY', '32768'); os.environ.setdefault('SAP_QUERY_FACTOR', '32')
os.environ.setdefault('SAP_PAIR_FACTOR', '2')
import genesis_sap_common as G

M = G.ROOT / 'protocol/manifests'
LOG = None


def log(s):
    line = f"{time.strftime('%Y-%m-%dT%H:%M:%S%z')} {socket.gethostname()} {s}"
    print(line, flush=True)
    if LOG:
        with open(LOG, 'a') as f: f.write(line + '\n')


def timed(tag, fn):
    t = time.time(); r = fn(); log(f'done {tag} {time.time() - t:.0f}s'); return r


def three_box():
    import run_threebox_genesis_sap as T
    import strict_v3_common as common
    mf = M / 'three_box_push_strict_v4.json'
    _, spec, _ = G.registry_task(T.TASK, mf); man, _ = G.load_manifest(mf)
    R, seed0 = man['R'], man['seed0']
    cands = [G.parse_candidate(t, spec) for t in spec['candidates']]
    sel = {}
    for i in reversed(range(len(cands))):
        for k in reversed(range(len(man['selection']))):
            s = man['selection'][k]
            sel[(i, k)] = timed(f'select {i + 1} {s["id"]}', lambda: T.run_cell(cands[i], s, seed0 + k, R))
    rows = [[sel[(i, k)] for k in range(len(man['selection']))] for i in range(len(cands))]
    walls = [sum(r['wall_s'] for r in rr) for rr in rows]
    win = common.box_v4_choose(rows, walls); frozen = cands[win]; log(f'frozen {win + 1} {frozen["id"]}')
    byid = {c['id']: c for c in man['held_out_conditions']}
    sens = []
    if not frozen.get('native'):
        k = 0
        for cid in man['parameter_sensitivity']['cells']:
            for s in man['parameter_sensitivity']['scales']:
                c = dict(frozen); c['hydroelastic_stiffness'] = frozen['hydroelastic_stiffness'] * s
                c['id'] = f'{frozen["id"]} | hydroelastic_stiffness x{s}'; sens.append((c, byid[cid], seed0 + 2000 + k)); k += 1
    held = list(enumerate(man['held_out_conditions']))
    # sensitivity is last in the primary's order, so do it first here; then held-out from the end
    for c, cell, seed in reversed(sens):
        timed(f'sens {c["id"]} {cell["id"]}', lambda: T.run_cell(c, cell, seed, R))
    for j, cell in reversed(held):
        timed(f'held {j} {cell["id"]}', lambda: T.run_cell(frozen, cell, seed0 + 1000 + j, R))


def simple_pinch():
    import run_pinch_genesis_sap as T
    import pinch_v3_common as pc
    mf = M / 'simple_pinch_force_controlled_v3.json'
    _, spec, _ = G.registry_task(T.TASK, mf); man0, _ = G.load_manifest(mf); man = G.with_candidates(man0, spec)
    R, seed0 = man['R'], man['seed0']
    cands = [G.parse_candidate(t, spec) for t in man['candidate_rule']['genesis_sap_candidates']]
    sel = {}
    for i in reversed(range(len(cands))):
        for k in reversed(range(len(man['selection']))):
            cell = man['selection'][k]
            t = time.perf_counter(); r = timed(f'select {i + 1} {cell["id"]}', lambda: T.run_cell(cands[i], cell, seed0 + k, R))
            r['wall_s'] = time.perf_counter() - t; r.update({'cell': cell, 'candidate': cands[i]}); sel[(i, k)] = r
    rows = [[sel[(i, k)] for k in range(len(man['selection']))] for i in range(len(cands))]
    walls = [sum(r['wall_s'] for r in rr) for rr in rows]
    win = pc.choose(rows, walls); frozen = cands[win]; log(f'frozen {win + 1} {frozen["id"]}')
    for j, cell in reversed(list(enumerate(man['held_out_conditions']))):
        timed(f'held {j} {cell["id"]}', lambda: T.run_cell(frozen, cell, seed0 + 1000 + j, R))


def main():
    global LOG
    ap = argparse.ArgumentParser(); ap.add_argument('task', choices=['three-box', 'simple-pinch']); ap.add_argument('--log', required=True)
    a = ap.parse_args(); LOG = a.log
    log(f'reverse worker start {a.task} env cap={os.environ["SAP_BUFFER_CAPACITY"]} qf={os.environ["SAP_QUERY_FACTOR"]}')
    {'three-box': three_box, 'simple-pinch': simple_pinch}[a.task]()
    log(f'reverse worker finished {a.task}')


if __name__ == '__main__':
    main()
