"""Validate complete corrected artifacts against the declared matrices and raw rows."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'exp-results/contact-correction-20261109'


def validate(path, manifest_path, box=False):
    path = path.resolve()
    data = json.loads(path.read_text())
    assert data.get('complete', True), path
    raw = manifest_path.read_bytes(); manifest = json.loads(raw)
    assert data['manifest_sha256'] == hashlib.sha256(raw).hexdigest(), path
    assert len(data['selection_candidates']) == 5, path
    selected = data['selection_candidates'][data['frozen_index']-1]
    if box:
        for candidate_cells in data['selection_candidates']:
            assert [c['cell'] for c in candidate_cells] == manifest['selection']
            assert all(c['R'] == 20 and len(c['rollouts']) == 20 for c in candidate_cells)
        assert selected[0]['candidate'] == data['frozen_candidate']
    else:
        assert all(c['cell'] == manifest['selection'] for c in data['selection_candidates'])
        assert all(c['R'] == 20 and len(c['rollouts']) == 20 for c in data['selection_candidates'])
        assert selected['candidate'] == data['frozen_candidate']
    assert [c['cell'] for c in data['heldout']] == manifest['held_out_conditions'], path
    count = 0
    for cell in data['heldout']:
        assert cell['R'] == 20 and len(cell['rollouts']) == 20, cell['cell']['id']
        assert cell['candidate'] == data['frozen_candidate'], cell['cell']['id']
        if box:
            rows = cell['rollouts']
            assert cell['agree_and_valid_count'] == sum(r['agrees'] and r['valid'] for r in rows)
            assert cell['finite_count'] == sum(r['finite'] for r in rows)
            assert all(r['finite'] or (r['observed'] == 'invalid' and not r['agrees'] and not r['valid']) for r in rows)
            assert all(not r['valid'] or (r['finite'] and r['capacity_ok'] and r['contact_gate_pass']) for r in rows)
            audit = cell['capacity_audit']
            assert len(audit['overflow_flags_by_world']) == 20
            assert audit['capacity_overflow_worlds'] == sum(not r['capacity_ok'] for r in rows)
            count += cell['agree_and_valid_count']
        else:
            assert cell['pass_count'] == sum(r['passes'] for r in cell['rollouts'])
            if data['engine'] == 'MuJoCo Warp':
                assert all(abs(v[0]-.8) < 1e-12 for v in cell['compiled']['geom_friction'])
                assert 'capacity_audit' in cell
            count += cell['pass_count']
    total = data['heldout_accuracy_total'] if box else data['heldout_pass_total']
    assert total == [count, 20 * len(data['heldout'])], path
    print('PASS', path.relative_to(ROOT), total)


def main():
    plate_manifest = ROOT / 'sim_protocol/manifests/three_plate_stack_strict_v3p1.json'
    for name in ['sap', 'mujoco']:
        prefix = 'sap' if name == 'sap' else 'mjwarp'
        validate(BASE / f'three-plate/{name}/{prefix}_three_plate_strict_v3.json', plate_manifest)
    validate(BASE / 'three-box/mujoco/mjwarp_three_box_strict_v4.json',
             ROOT / 'sim_protocol/manifests/three_box_push_strict_v4.json', box=True)


if __name__ == '__main__':
    main()
