"""Replay the historical frozen strict-v4 winner with corrected validity telemetry.

No reselection or retuning: the historical selection artifact fixes the winner.
Selection rows in the output retain their historical provenance; two additional
selection replays measure repeatability of that same frozen configuration.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import warp as wp

import strict_v3_common as common
from run_threebox_strict_v4_mjwarp import parse_candidate, run_cell, fmt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--selection-artifact', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    man, sha = common.load_manifest(args.manifest)
    raw = args.selection_artifact.read_bytes(); historical = json.loads(raw)
    assert historical['manifest_sha256'] == sha
    frozen = historical['frozen_candidate']
    assert frozen == parse_candidate(man['candidate_rule']['mujoco_warp_candidates'][historical['frozen_index']-1])
    args.out.mkdir(parents=True, exist_ok=True)
    replay, held = [], []
    for s, cell in enumerate(man['selection']):
        row = run_cell(frozen, cell, man['seed0']+s, man['R']); replay.append(row)
        print('frozen selection replay', cell['id'], fmt(row), flush=True)
    out = {'protocol': man['protocol'], 'task': man['task'], 'revision': man['revision'],
           'audit_revision': 'historical-frozen-capacity-replay-20261109', 'manifest_sha256': sha,
           'engine': 'MuJoCo Warp', 'mujoco_version': mujoco.__version__, 'warp_version': wp.config.version,
           'device': str(wp.get_device()), 'selection_source_sha256': hashlib.sha256(raw).hexdigest(),
           'selection_provenance': 'B1 and selection_candidates are copied from the historical artifact; no reselection. selection_replay is newly executed with the same frozen winner.',
           'B1': historical['B1'], 'selection_candidates': historical['selection_candidates'],
           'frozen_index': historical['frozen_index'], 'frozen_candidate': frozen,
           'selection_replay': replay, 'heldout': held, 'complete': False}
    target = args.out/'mjwarp_three_box_strict_v4.json'
    def checkpoint():
        out['heldout_accuracy_total'] = [sum(r['agree_and_valid_count'] for r in held), man['R']*len(held)]
        staging = target.with_suffix('.tmp')
        staging.write_text(json.dumps(common.json_safe(out), indent=1)+'\n'); staging.replace(target)
    checkpoint()
    for j, cell in enumerate(man['held_out_conditions']):
        row = run_cell(frozen, cell, man['seed0']+1000+j, man['R']); held.append(row); checkpoint()
        print('held',cell['id'],fmt(row),'overflow=',row['capacity_overflow_worlds'],flush=True)
    out['complete'] = True; checkpoint()


if __name__ == '__main__':
    main()
