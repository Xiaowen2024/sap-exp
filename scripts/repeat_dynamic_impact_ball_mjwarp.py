#!/usr/bin/env python3
"""R=20 local-perturbation evaluation for frozen MuJoCo-Warp ball settings.

State metrics intentionally match the original Warp ball runner.  This is a
separate artifact from force/impulse telemetry, which needs EFC sampling at
the integrator rate.
"""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np

from run_dynamic_impact_ball_budget import ci, dtw, load_trials, peaks
from run_dynamic_impact_ball_budget_mjwarp import simulate_batch

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'exp-results/dynamic-impact-v1/ball-bounce/mjwarp-3.17'

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--stage',choices=('b1','b5','b100'),required=True); ap.add_argument('--replicates',type=int,default=20); ap.add_argument('--seed',type=int,default=20261027); a=ap.parse_args()
    source=json.loads((OUT/f'{a.stage}.json').read_text()); names,real=load_trials()
    ids=[names.index(str(x['trial'])) for x in source['heldout_metrics']['per_rollout']]
    rows=[]
    for i in ids:
        rng=np.random.default_rng(a.seed+i); initials=np.asarray([real[i][0]+rng.normal(0.,5e-4,3) for _ in range(a.replicates)])
        sims=simulate_batch(source['frozen_config'], initials, [len(real[i])]*a.replicates)
        vals=[]
        for s in sims:
            p,q=s['peaks'][:3],peaks(real[i][:,2])[:3]
            vals.append({'rmse_m':float(np.sqrt(np.mean(np.sum((s['xyz']-real[i])**2,axis=1)))), 'dtw_m':dtw(s['xyz'],real[i]),'e':s['e'],'tc_s':s['tc_s'], 'apex':float(np.mean([abs(x-y)/max(abs(y),1e-6) for x,y in zip(p,q)])) if p and q else float('nan')})
        rows.append({'trial': names[i], **{k: ci([v[k] for v in vals]) for k in ('rmse_m', 'dtw_m', 'e', 'tc_s', 'apex')}})
    metrics={k:{'across_sample_mean_of_perturbed_means':ci([r[k]['mean'] for r in rows]),'pooled_within_sample_sd_mean':float(np.mean([r[k]['std'] for r in rows if r[k]['std'] is not None]))} for k in ('rmse_m','dtw_m','e','tc_s','apex')}
    out={'protocol':'dynamic-impact-v1 local-repeats v1','task':'GAUGE public bouncing ball','engine':'MuJoCo Warp','stage':a.stage,'frozen_config':source['frozen_config'],'heldout_samples':len(rows),'local_perturbation_repeats_per_sample':a.replicates,'perturbation':{'initial_position_sigma_m':5e-4},'metrics':metrics,'per_sample':rows,'scope':'state-derived local robustness; force/impulse reported separately'}
    (OUT/f'{a.stage}_repeats_r{a.replicates}.json').write_text(json.dumps(out,indent=2)+'\n'); print(OUT/f'{a.stage}_repeats_r{a.replicates}.json')
if __name__=='__main__': main()
