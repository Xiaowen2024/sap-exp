#!/usr/bin/env python3
"""Warp-batched R=20 local robustness for frozen ContactNets cube-toss settings."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import argparse,json,math
from pathlib import Path
import numpy as np
from run_dynamic_impact_cube_budget import HALF,SRC,ci,dtw,qangle,qangle_sym,upward_face
from run_dynamic_impact_cube_budget_mjwarp import simulate_batch
from repeat_dynamic_impact_cube import perturb
ROOT=REPO_ROOT; OUT=ROOT/'exp-results/dynamic-impact-v1/cube-toss/mjwarp-3.17'
KEYS=('position_rmse_m','position_dtw_m','linear_velocity_rmse_m_s','final_position_error_m','final_orientation_error_rad','final_orientation_sym_rad','final_up_face_agreement','e_com_proxy','contact_duration_s','postimpact_linear_speed_error_m_s','postimpact_angular_speed_error_rad_s')
def one(real, sim):
 s,e,tc,o=sim
 return {'position_rmse_m':float(np.sqrt(np.mean(np.sum((s[:,:3]-real[:,:3])**2,axis=1)))),'position_dtw_m':dtw(s[:,:3],real[:,:3]),'linear_velocity_rmse_m_s':float(np.sqrt(np.mean(np.sum((s[:,7:10]-real[:,7:10])**2,axis=1)))),'final_position_error_m':float(np.linalg.norm(s[-1,:3]-real[-1,:3])),'final_orientation_error_rad':qangle(s[-1,3:7],real[-1,3:7]),'final_orientation_sym_rad':qangle_sym(s[-1,3:7],real[-1,3:7]),'final_up_face_agreement':float(upward_face(s[-1,3:7])==upward_face(real[-1,3:7])),'e_com_proxy':e,'contact_duration_s':tc,'postimpact_linear_speed_error_m_s':float(abs(np.linalg.norm(s[-1,7:10])-np.linalg.norm(real[-1,7:10]))),'postimpact_angular_speed_error_rad_s':float(abs(np.linalg.norm(s[-1,10:13])-np.linalg.norm(real[-1,10:13])))}
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--stage',choices=('b1','b5','b100'),required=True);ap.add_argument('--replicates',type=int,default=20);ap.add_argument('--seed',type=int,default=20261027);z=ap.parse_args()
 src=json.loads((OUT/f'{z.stage}.json').read_text());data=np.load(SRC);states,L=data['state'],data['lengths'];ids=[x['id'] for x in src['heldout_metrics']['per_rollout']];rows=[]
 for sid in ids:
  real=states[sid,:int(L[sid])];rng=np.random.default_rng(z.seed+sid);initial=np.asarray([perturb(real[0],rng) for _ in range(z.replicates)]); sims=simulate_batch(src['frozen_config'],initial,[len(real)]*z.replicates); vals=[one(real,s) for s in sims]; rows.append({'id':int(sid), **{k:ci([v[k] for v in vals]) for k in KEYS}})
 metrics={k:{'across_sample_mean_of_perturbed_means':ci([r[k]['mean'] for r in rows]),'pooled_within_sample_sd_mean':float(np.mean([r[k]['std'] for r in rows if r[k]['std'] is not None]))} for k in KEYS}
 out={'protocol':'dynamic-impact-v1 local-repeats v1','task':'ContactNets real cube toss','engine':'MuJoCo Warp','stage':z.stage,'frozen_config':src['frozen_config'],'heldout_samples':len(rows),'local_perturbation_repeats_per_sample':z.replicates,'perturbation':{'initial_position_sigma_m':5e-4,'initial_orientation_sigma_rad':float(np.deg2rad(.5)),'initial_linear_velocity_sigma_m_s':.01,'initial_angular_velocity_sigma_rad_s':.05},'metrics':metrics,'per_sample':rows,'scope':'state-derived local robustness; contact impulse is a multi-point proxy'}
 (OUT/f'{z.stage}_repeats_r{z.replicates}.json').write_text(json.dumps(out,indent=2)+'\n');print(OUT/f'{z.stage}_repeats_r{z.replicates}.json')
if __name__=='__main__':main()
