"""Collect the 5-iteration tuning log per engine from tubes_eval.json + params."""
import json, sys
from pathlib import Path
root = Path(sys.argv[1]); out = {}
for eng, pfx in (('mujoco_4.0', 'mj'), ('sap_old_fix-gamma_legacy', 'sapold'), ('sap_new_2325082_newton', 'sapnew')):
    rows = []
    for it in range(1, 6):
        run = root / 'runs' / f'{pfx}_it{it}'; e = json.loads((run / 'tubes_eval.json').read_text())
        pf = root / 'params' / f"{'mj' if pfx == 'mj' else 'sap'}_it{it}.json"; p = json.loads(pf.read_text()) if pf.exists() else {'why': 'native defaults'}
        rows.append({'iter': it, 'params': {k: v for k, v in p.items() if k != 'why'}, 'why': p['why'], 'n_inserted': e['n_inserted'], 'official_score': e['official_score'],
                     'modes': {k: v['mode'] for k, v in e['tubes'].items()}, 'depth_mm': {k: round(v['depth_m'] * 1e3, 1) for k, v in e['tubes'].items()},
                     'tilt_deg': {k: round(v['tilt_deg'], 1) for k, v in e['tubes'].items()}, 'wall_s': e['health']['wall_s'],
                     'drive_gate_pass': e['health']['drive_gate_pass'], 'peak_overlap_mm': round(1e3 * e['health']['engine_peak_overlap_m'], 2)})
    best = max(rows, key=lambda r: (r['n_inserted'], -r['iter']))
    out[eng] = {'iterations': rows, 'best_iter': best['iter'], 'best_n_inserted': best['n_inserted'], 'best_score': best['official_score']}
    print(eng, 'best it%d: %d/3 (score %d)' % (best['iter'], best['n_inserted'], best['official_score']), [r['n_inserted'] for r in rows])
(root / 'tuning_log.json').write_text(json.dumps(out, indent=1))
