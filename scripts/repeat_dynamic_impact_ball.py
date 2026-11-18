#!/usr/bin/env python3
"""R=20 local-robustness evaluation for frozen GAUGE bouncing-ball settings."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import numpy as np
import mujoco
from run_dynamic_impact_ball_budget import (OUT_DEFAULT,ci,dtw,load_trials,mjcf,peaks,simulate)

METRICS={'trajectory_rmse_m':'rmse_m','dtw_m':'dtw_m','effective_restitution':'e','contact_duration_s':'tc_s','apex_height_decay_error':'apex'}

def one(m,x,ref):
 s=simulate(m,x,len(ref));p,q=s['peaks'][:3],peaks(ref[:,2])[:3]
 return {'rmse_m':float(np.sqrt(np.mean(np.sum((s['xyz']-ref)**2,axis=1)))),'dtw_m':dtw(s['xyz'],ref),'e':s['e'],'tc_s':s['tc_s'],'apex':float(np.mean([abs(a-b)/max(abs(b),1e-6) for a,b in zip(p,q)])) if p and q else float('nan'),'xyz':s['xyz']}

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--stage',choices=('b1','b5','b100'),required=True);ap.add_argument('--replicates',type=int,default=20);ap.add_argument('--seed',type=int,default=20261024);ap.add_argument('--results-dir',type=Path,default=OUT_DEFAULT);a=ap.parse_args()
 src=json.loads((a.results_dir/f'{a.stage}.json').read_text());names,real=load_trials();held=[int(r['trial']) for r in src['heldout_metrics']['per_rollout']];ids=[names.index(str(i)) for i in held];m=mujoco.MjModel.from_xml_string(mjcf(src['frozen_config']))
 rows=[];maxdiff=0.
 for i in ids:
  base=one(m,real[i][0],real[i])
  for _ in range(a.replicates-1):maxdiff=max(maxdiff,float(np.max(abs(one(m,real[i][0],real[i])['xyz']-base['xyz']))))
  rng=np.random.default_rng(a.seed+i);draw=[one(m,real[i][0]+rng.normal(0,5e-4,3),real[i]) for _ in range(a.replicates)]
  row={'trial':names[i]}
  for k,v in METRICS.items():row[k]=ci([d[v] for d in draw])
  rows.append(row)
 out={'protocol':'dynamic-impact-v1 local-repeats v1','task':'GAUGE public bouncing ball','engine':'MuJoCo','stage':a.stage,'frozen_config':src['frozen_config'],'heldout_samples':len(rows),'exact_deterministic_repeats_per_sample':a.replicates,'exact_deterministic_replay_max_abs_position_difference_m':maxdiff,'local_perturbation_repeats_per_sample':a.replicates,'perturbation':{'label':'stress-v1; numerical-local-robustness, not measured sensor noise','initial_position_sigma_m':5e-4},'metrics':{},'per_sample':rows}
 for k in METRICS:
  means=[x[k]['mean'] for x in rows if x[k]['mean'] is not None];sds=[x[k]['std'] for x in rows if x[k]['std'] is not None]
  out['metrics'][k]={'across_sample_mean_of_perturbed_means':ci(means),'pooled_within_sample_sd_mean':float(np.mean(sds)) if sds else None,'pooled_within_sample_sd_max':float(np.max(sds)) if sds else None}
 dst=a.results_dir/f'{a.stage}_repeats_r{a.replicates}.json';dst.write_text(json.dumps(out,indent=2)+'\n');print(dst)
if __name__=='__main__':main()
