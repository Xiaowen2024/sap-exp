#!/usr/bin/env python3
"""Post-impact sustained-contact extension for ContactNets Cube Toss, Warp GPU."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import argparse,json
from pathlib import Path
import mujoco,mujoco_warp as mw,numpy as np
from run_dynamic_impact_cube_budget import DT,SRC,make_model
ROOT=REPO_ROOT; OUT=ROOT/'exp-results/protocol-v2-native-default/b5-sustained-contact-v1/cube-toss'
def main():
 a=argparse.ArgumentParser();a.add_argument('--replicates',type=int,default=20);a.add_argument('--duration',type=float,default=6.0);z=a.parse_args();srcp=ROOT/'exp-results/protocol-v2-native-default/b5-dynamic-impact-v1/cube-toss/mujoco_warp/b5_sop_r20.json';src=json.loads(srcp.read_text());raw=np.load(SRC); ids=src['heldout_sample_indices']; initial=[]
 for sid in ids:
  rng=np.random.default_rng(20261027+sid); base=raw['state'][sid,0]
  for _ in range(z.replicates):
   x=base.copy();x[:3]+=rng.normal(0,5e-4,3);x[7:10]+=rng.normal(0,.01,3);x[10:13]+=rng.normal(0,.05,3);initial.append(x)
 m=make_model(src['frozen_config']);gm=mw.put_model(m);n=len(initial);d=mw.make_data(m,nworld=n,nconmax=128,njmax=512);q=d.qpos.numpy();v=d.qvel.numpy();ini=np.asarray(initial);q[:,:7]=ini[:,:7];v[:,:6]=ini[:,7:13];d.qpos.assign(q);d.qvel.assign(v);mw.forward(gm,d)
 every=max(1,round(.01/DT)); samples=[]
 for k in range(round(z.duration/DT)+1):
  if k%every==0:
   qp,qv=d.qpos.numpy(),d.qvel.numpy(); nac=int(d.nacon.numpy()[0]);wid=d.contact.worldid.numpy()[:nac] if nac else np.empty(0,int); duty=np.bincount(wid,minlength=n)>0
   # In pyramidal contacts `efc.force` is a constraint-row value, not a
   # physical normal wrench.  The free-joint z constraint force is the
   # correctly aggregated support reaction for this one-body task.
   fn=d.qfrc_constraint.numpy()[:,2].copy(); overlap=np.zeros(n)
   if nac:
    addr=d.contact.efc_address.numpy()[:nac]; dist=d.contact.dist.numpy()[:nac]; force=d.efc.force.numpy()
    for ci,w in enumerate(wid):
     overlap[w]=max(overlap[w],max(0.,-float(dist[ci])))
   samples.append((k*DT,qp[:,:3].copy(),qv[:,:6].copy(),duty,fn,overlap))
  if k<round(z.duration/DT):mw.step(gm,d)
 t=np.array([s[0] for s in samples]);p=np.array([s[1] for s in samples]);vel=np.array([s[2] for s in samples]);duty=np.array([s[3] for s in samples]);fn=np.array([s[4] for s in samples]);overlap=np.array([s[5] for s in samples]);rec=[]
 for w in range(n):
  good=(np.linalg.norm(vel[:,w,:3],axis=1)<.01)&(np.linalg.norm(vel[:,w,3:],axis=1)<.05); start=None
  for j in range(len(t)-20):
   if good[j:j+20].all() and t[-1]-t[j]>=1.:start=j;break
  if start is None:rec.append({'world':w,'settled':False,'contact_duty':float(duty[:,w].mean())});continue
  tail=slice(start,None);drift=float(np.linalg.norm(p[-1,w,:2]-p[start,w,:2])/(t[-1]-t[start])); fn_tail=fn[tail,w]; mean_fn=float(fn_tail.mean()); rec.append({'world':w,'settled':True,'settle_time_s':float(t[start]),'drift_m_s':drift,'final_height_m':float(p[-1,w,2]),'contact_duty':float(duty[tail,w].mean()),'tail_v_rms_m_s':float(np.sqrt(np.mean(np.sum(vel[tail,w,:3]**2,axis=1)))),'normal_force_mean_N':mean_fn,'normal_force_cv':float(fn_tail.std()/max(mean_fn,1e-12)),'normal_load_balance_error':float(abs(mean_fn-m.body_mass[1]*9.81)/(m.body_mass[1]*9.81)),'equilibrium_overlap_max_mm':float(overlap[tail,w].max()*1000)})
 ok=[x for x in rec if x['settled']]; metric_keys=('settle_time_s','drift_m_s','final_height_m','contact_duty','tail_v_rms_m_s','normal_force_mean_N','normal_force_cv','normal_load_balance_error','equilibrium_overlap_max_mm');out={'protocol':'sustained-contact-v1','task':'Cube Toss sustained tail','engine':'MuJoCo Warp','stage':'B5_guided','frozen_config':src['frozen_config'],'heldout_samples':len(ids),'R':z.replicates,'duration_s':z.duration,'settled_fraction':len(ok)/n,'metrics':{k:float(np.mean([x[k] for x in ok])) if ok else None for k in metric_keys},'per_world':rec}
 OUT.mkdir(parents=True,exist_ok=True);(OUT/f'mjwarp_b5_r{z.replicates}.json').write_text(json.dumps(out,indent=2)+'\n')
if __name__=='__main__':main()
