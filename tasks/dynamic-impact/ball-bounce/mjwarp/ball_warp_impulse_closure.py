#!/usr/bin/env python3
"""Native-EFC impulse closure for frozen Warp ball-bounce configurations."""
from __future__ import annotations
import argparse,json,math
from pathlib import Path
import mujoco,mujoco_warp as mw,numpy as np
from run_dynamic_impact_ball_budget import DT,MASS,load_trials,mjcf
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'exp-results/dynamic-impact-v1/ball-bounce/mjwarp-3.17'
def main():
 a=argparse.ArgumentParser();a.add_argument('--stage',choices=('b1','b5','b100'),required=True);a.add_argument('--replicates',type=int,default=20);z=a.parse_args();src=json.loads((OUT/f'{z.stage}.json').read_text());names,real=load_trials();ids=[names.index(str(r['trial'])) for r in src['heldout_metrics']['per_rollout']];ini=[]
 for i in ids:
  rng=np.random.default_rng(20261027+i)
  ini.extend([real[i][0]+rng.normal(0,5e-4,3) for _ in range(z.replicates)])
 m=mujoco.MjModel.from_xml_string(mjcf(src['frozen_config']));gm=mw.put_model(m);n=len(ini);d=mw.make_data(m,nworld=n,nconmax=16,njmax=64);q=d.qpos.numpy();q[:,:3]=np.asarray(ini);q[:,3]=1;d.qpos.assign(q);mw.forward(gm,d);active=np.zeros(n,bool);J=np.zeros(n);contact_time=np.zeros(n);vin=np.full(n,np.nan);vout=np.full(n,np.nan)
 for step in range(round(2./DT)):
  nac=int(d.nacon.numpy()[0]);wid=d.contact.worldid.numpy()[:nac] if nac else np.empty(0,int);addr=d.contact.efc_address.numpy()[:nac] if nac else np.empty((0,4),int);f=d.efc.force.numpy();now=np.zeros(n); 
  for k,w in enumerate(wid):
   row=int(addr[k,0]);now[w]+=max(0.,float(f[w,row])) if row>=0 else 0.
  vz=d.qvel.numpy()[:,2]; entering=(now>1e-5)&~active; leaving=(now<=1e-5)&active;vin[entering & ~np.isfinite(vin)]=vz[entering & ~np.isfinite(vin)];vout[leaving & ~np.isfinite(vout)]=vz[leaving & ~np.isfinite(vout)];J+=now*DT;contact_time += (now>1e-5)*DT;active=now>1e-5;mw.step(gm,d)
 rows=[]
 for i in range(n):
  dp=MASS*(vout[i]-vin[i]) if np.isfinite(vin[i]) and np.isfinite(vout[i]) else np.nan
  # During contact, J_n - m*g*t_contact = m*(v_out-v_in).
  required=dp + MASS*9.81*contact_time[i] if np.isfinite(dp) else np.nan
  rows.append({'world':i,'normal_impulse_Ns':float(J[i]),'contact_duration_s':float(contact_time[i]),'momentum_change_Ns':float(dp),'normal_impulse_required_Ns':float(required) if np.isfinite(required) else None,'closure_abs_Ns':float(abs(J[i]-required)) if np.isfinite(required) else None,'closure_rel':float(abs(J[i]-required)/max(abs(required),1e-9)) if np.isfinite(required) else None})
 good=[r for r in rows if r['closure_rel'] is not None];out={'protocol':'dynamic-impact-v1','task':'GAUGE ball native-EFC impulse closure','engine':'MuJoCo Warp','stage':z.stage,'R':z.replicates,'rows':rows,'mean_relative_closure_error':float(np.mean([r['closure_rel'] for r in good])) if good else None,'valid_episodes':len(good)};(OUT/f'{z.stage}_impulse_closure_r{z.replicates}.json').write_text(json.dumps(out,indent=2)+'\n')
if __name__=='__main__':main()
