#!/usr/bin/env python3
"""CUDA-batched MuJoCo Warp counterpart of the ContactNets cube toss runner."""
from __future__ import annotations
import argparse,json,math,time
from pathlib import Path
import mujoco,mujoco_warp as mjw,numpy as np
from run_dynamic_impact_cube_budget import (DT,HALF,INERTIA,MASS,MU,SRC,ci,configs,dtw,make_model,qangle,qangle_sym)

def simulate_batch(c,initial,lengths):
 mjm=make_model(c);m=mjw.put_model(mjm);nw=len(initial);d=mjw.make_data(mjm,nworld=nw,nconmax=128,njmax=512)
 q=d.qpos.numpy();v=d.qvel.numpy();q[:,:7]=initial[:,:7];v[:,:6]=initial[:,7:13];d.qpos.assign(q);d.qvel.assign(v);mjw.forward(m,d)
 maxl=max(lengths);out=[np.zeros((L,13)) for L in lengths];contact=np.zeros((nw,maxl),bool);vz=np.zeros((nw,maxl));
 for s in range(maxl):
  qp=d.qpos.numpy();qv=d.qvel.numpy()
  for w,L in enumerate(lengths):
   if s<L:out[w][s,:7]=qp[w,:7];out[w][s,7:13]=qv[w,:6]
  nac=int(d.nacon.numpy()[0])
  if nac: contact[d.contact.worldid.numpy()[:nac],s]=True
  vz[:,s]=qv[:,2]
  if s<maxl-1:mjw.step(m,d)
 overflow=d.overflow.numpy();ans=[]
 for w,L in enumerate(lengths):
  z=contact[w,:L];vel=vz[w,:L];st=np.flatnonzero(np.diff(np.r_[False,z,False].astype(int))==1);sp=np.flatnonzero(np.diff(np.r_[False,z,False].astype(int))==-1)-1;tc=e=float('nan')
  if len(st) and len(sp):
   a=int(st[0]);b=int(sp[sp>=a][0]);tc=(b-a+1)*DT
   if a and b+1<L and vel[a-1]<0:e=float(-vel[b+1]/vel[a-1])
  ans.append((out[w],e,tc,bool(overflow[w])))
 return ans

def score(c,ids,a,lengths,keep=False):
 real=[a[i,:int(lengths[i])] for i in ids];sims=simulate_batch(c,np.asarray([x[0] for x in real]),[len(x) for x in real]);rows=[];t=time.perf_counter()
 for i,r,(s,e,tc,o) in zip(ids,real,sims):
  rows.append({'id':int(i),'position_rmse_m':float(np.sqrt(np.mean(np.sum((s[:,:3]-r[:,:3])**2,axis=1)))),'position_dtw_m':dtw(s[:,:3],r[:,:3]),'linear_velocity_rmse_m_s':float(np.sqrt(np.mean(np.sum((s[:,7:10]-r[:,7:10])**2,axis=1)))),'final_position_error_m':float(np.linalg.norm(s[-1,:3]-r[-1,:3])),'final_orientation_error_rad':qangle(s[-1,3:7],r[-1,3:7]),'final_orientation_sym_rad':qangle_sym(s[-1,3:7],r[-1,3:7]),'e_com_proxy':e,'contact_duration_s':tc,'postimpact_linear_speed_error_m_s':float(abs(np.linalg.norm(s[-1,7:10])-np.linalg.norm(r[-1,7:10]))),'postimpact_angular_speed_error_rad_s':float(abs(np.linalg.norm(s[-1,10:13])-np.linalg.norm(r[-1,10:13]))),'overflow':o})
 keys={'trajectory_position_rmse_m':'position_rmse_m','trajectory_position_dtw_m':'position_dtw_m','linear_velocity_rmse_m_s':'linear_velocity_rmse_m_s','final_position_error_m':'final_position_error_m','final_orientation_error_rad':'final_orientation_error_rad','final_orientation_sym_rad':'final_orientation_sym_rad','effective_restitution_com_proxy':'e_com_proxy','first_contact_duration_s':'contact_duration_s','postimpact_linear_speed_error_m_s':'postimpact_linear_speed_error_m_s','postimpact_angular_speed_error_rad_s':'postimpact_angular_speed_error_rad_s'}
 out={'config':c,'rollouts':len(ids),'batch_worlds':len(ids),'postprocess_wall_s':time.perf_counter()-t,'overflow_worlds':int(sum(x['overflow'] for x in rows))}
 out.update({k:ci([x[v] for x in rows]) for k,v in keys.items()});out['selection_loss']=out['trajectory_position_rmse_m']['mean']+.05*out['final_orientation_error_rad']['mean']
 if keep:out['per_rollout']=rows
 return out

def main():
 p=argparse.ArgumentParser();p.add_argument('--stage',choices=('b1','b5','b100'),required=True);p.add_argument('--output',type=Path,required=True);q=p.parse_args();d=np.load(SRC);a=d['state'];L=d['lengths'];eligible=[i for i,l in enumerate(L) if a[i,0,2]>2*HALF and np.nanmin(a[i,:l,2])<.09];train=eligible[::2][:32];test=eligible[1::2][:32];grid=configs(q.stage);rows=[score(c,train,a,L) for c in grid];best=min(rows,key=lambda x:x['selection_loss']);held=score(best['config'],test,a,L,True)
 result={'protocol':'dynamic-impact-v1','task':'ContactNets real cube toss','engine':'MuJoCo Warp','engine_version':mujoco.__version__,'stage':q.stage,'budget_configurations':len(grid),'execution':'CUDA batched; 32 worlds/configuration','split':'32 alternating eligible train / 32 held-out','physics':{'dt_s':DT,'mass_kg':MASS,'mu':MU,'cube_half_width_m':HALF,'inertia_kg_m2':INERTIA},'alignment':'released position/quaternion/linear+angular velocity directly initialized; zero controls after release','selection_metric':'train position RMSE + .05*final orientation error (rad)','train_candidates':rows,'frozen_config':best['config'],'heldout_metrics':held,'caveat':'COM restitution is only a multi-point-impact proxy; it is not a material restitution coefficient.'}
 q.output.mkdir(parents=True,exist_ok=True);(q.output/(q.stage+'.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'stage':q.stage,'frozen':best['config'],'heldout':held},indent=2))
if __name__=='__main__':main()
