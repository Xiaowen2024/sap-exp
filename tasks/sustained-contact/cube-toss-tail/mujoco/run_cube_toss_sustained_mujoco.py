#!/usr/bin/env python3
"""CPU counterpart of the frozen-config Cube Toss sustained-tail protocol."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import argparse,json
from pathlib import Path
import mujoco,numpy as np
from run_dynamic_impact_cube_budget import DT,SRC,make_model
ROOT=REPO_ROOT;OUT=ROOT/'exp-results/protocol-v2-native-default/b5-sustained-contact-v1/cube-toss'
def main():
 a=argparse.ArgumentParser();a.add_argument('--replicates',type=int,default=20);a.add_argument('--duration',type=float,default=6.);z=a.parse_args();src=json.loads((ROOT/'exp-results/protocol-v2-native-default/b5-dynamic-impact-v1/cube-toss/mujoco/b5_sop_r20.json').read_text());raw=np.load(SRC);ids=src['heldout_sample_indices'];m=make_model(src['frozen_config']);cases=[]
 for sid in ids:
  rng=np.random.default_rng(20261027+sid);base=raw['state'][sid,0]
  for _ in range(z.replicates):
   x=base.copy();x[:3]+=rng.normal(0,5e-4,3);x[7:10]+=rng.normal(0,.01,3);x[10:13]+=rng.normal(0,.05,3);d=mujoco.MjData(m);d.qpos[:7]=x[:7];d.qvel[:6]=x[7:13];mujoco.mj_forward(m,d);cases.append(d)
 every=max(1,round(.01/DT));hist=[[] for _ in cases]
 for k in range(round(z.duration/DT)+1):
  for i,d in enumerate(cases):
   if k%every==0:
    fn=0.; overlap=0.
    for ci in range(d.ncon):
     force=np.zeros(6); mujoco.mj_contactForce(m,d,ci,force)
     fn += max(0.,force[0]); overlap=max(overlap,max(0.,-d.contact[ci].dist))
    hist[i].append((k*DT,d.qpos[:3].copy(),d.qvel[:6].copy(),fn>1e-5,fn,overlap))
   if k<round(z.duration/DT):mujoco.mj_step(m,d)
 rec=[]
 for i,h in enumerate(hist):
  t=np.array([x[0] for x in h]);p=np.array([x[1] for x in h]);v=np.array([x[2] for x in h]);du=np.array([x[3] for x in h]);fn=np.array([x[4] for x in h]);ov=np.array([x[5] for x in h]);good=(np.linalg.norm(v[:,:3],axis=1)<.01)&(np.linalg.norm(v[:,3:],axis=1)<.05);start=next((j for j in range(len(t)-20) if good[j:j+20].all() and t[-1]-t[j]>=1),None)
  if start is None:rec.append({'world':i,'settled':False,'contact_duty':float(du.mean())});continue
  tail=slice(start,None); fn_tail=fn[tail]; mean_fn=float(fn_tail.mean())
  rec.append({'world':i,'settled':True,'settle_time_s':float(t[start]),'drift_m_s':float(np.linalg.norm(p[-1,:2]-p[start,:2])/(t[-1]-t[start])),'final_height_m':float(p[-1,2]),'contact_duty':float(du[tail].mean()),'tail_v_rms_m_s':float(np.sqrt(np.mean(np.sum(v[tail,:3]**2,axis=1)))),'normal_force_mean_N':mean_fn,'normal_force_cv':float(fn_tail.std()/max(mean_fn,1e-12)),'normal_load_balance_error':float(abs(mean_fn-m.body_mass[1]*9.81)/(m.body_mass[1]*9.81)),'equilibrium_overlap_max_mm':float(ov[tail].max()*1000)})
 ok=[r for r in rec if r['settled']]
 metric_keys=('settle_time_s','drift_m_s','final_height_m','contact_duty','tail_v_rms_m_s','normal_force_mean_N','normal_force_cv','normal_load_balance_error','equilibrium_overlap_max_mm')
 out={'protocol':'sustained-contact-v1','task':'Cube Toss sustained tail','engine':'MuJoCo','stage':'B5_guided','frozen_config':src['frozen_config'],'heldout_samples':len(ids),'R':z.replicates,'duration_s':z.duration,'settled_fraction':len(ok)/len(rec),'metrics':{k:float(np.mean([r[k] for r in ok])) if ok else None for k in metric_keys},'per_world':rec};OUT.mkdir(parents=True,exist_ok=True);(OUT/f'mujoco_b5_r{z.replicates}.json').write_text(json.dumps(out,indent=2)+'\n')
if __name__=='__main__':main()
