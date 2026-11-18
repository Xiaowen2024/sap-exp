#!/usr/bin/env python3
"""ContactNets real cube-toss replay: MuJoCo 3.17 budgeted evaluation."""
from __future__ import annotations
import argparse,json,math,time
from pathlib import Path
import mujoco,numpy as np
from cube_orientation_metrics import cube_symmetric_qangle as qangle_sym

ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'exp-results/dynamic-impact-v1/cube-toss/contactnets_si.npz'
OUT=ROOT/'exp-results/dynamic-impact-v1/cube-toss/mujoco-3.17'
DT=1/148; MASS=.37; MU=.18; HALF=.0524; INERTIA=.2950002913583124*HALF**2; G=9.81

def configs(stage):
 if stage=='b1': return [{'id':'default','kind':'default','integrator':'Euler'}]
 if stage=='b5': return configs('b1') + [{'id':f'k{k:g}_b{b:g}','kind':'direct','k':k,'b':b,'integrator':'implicitfast'} for k,b in [(1e3,0),(3e3,5),(1e4,15),(3e4,30)]]
 return [{'id':f'k{k:g}_b{b:g}','kind':'direct','k':float(k),'b':float(b),'integrator':'implicitfast'} for k in np.geomspace(1e3,1e5,10) for b in np.linspace(0,60,10)]

def make_model(c):
 a='' if c['kind']=='default' else f' solref="{-c["k"]} {-c["b"]}" solimp=".9 .9 .001 .5 2"'
 numerical_option='' if c['kind']=='default' else f' integrator="{c["integrator"]}" solver="Newton"'
 xml=f'''<mujoco model="contactnets_cube_toss"><option timestep="{DT}" gravity="0 0 -9.81"{numerical_option}/>
 <worldbody><geom type="plane" size="2 2 .1" friction="{MU} 0 0"{a}/><body name="cube"><freejoint/><inertial pos="0 0 0" mass="{MASS}" diaginertia="{INERTIA} {INERTIA} {INERTIA}"/><geom type="box" size="{HALF} {HALF} {HALF}" friction="{MU} 0 0"{a}/></body></worldbody></mujoco>'''
 return mujoco.MjModel.from_xml_string(xml)

def qangle(a,b):
 return float(2*np.arccos(np.clip(abs(np.dot(a,b)),-1,1)))
def upward_face(q):
 # Which signed local cube axis has greatest world-z component.
 m=np.empty(9);mujoco.mju_quat2Mat(m,q);R=m.reshape(3,3)
 j=int(np.argmax(np.abs(R[2,:])));return int(2*j+(R[2,j]<0))
def dtw(a,b):
 n,m=len(a),len(b);c=np.full((n+1,m+1),np.inf);l=np.zeros((n+1,m+1),int);c[0,0]=0
 for i in range(1,n+1):
  for j in range(1,m+1):
   p=min(((c[i-1,j],l[i-1,j]),(c[i,j-1],l[i,j-1]),(c[i-1,j-1],l[i-1,j-1])),key=lambda x:x[0]);c[i,j]=p[0]+np.linalg.norm(a[i-1]-b[j-1]);l[i,j]=p[1]+1
 return float(c[n,m]/max(1,l[n,m]))
def ci(x):
 a=np.asarray(x,float);a=a[np.isfinite(a)]
 if not len(a):return {'mean':None,'std':None,'ci95':None,'n':0}
 s=float(a.std(ddof=1)) if len(a)>1 else 0.;return {'mean':float(a.mean()),'std':s,'ci95':float(1.96*s/math.sqrt(len(a))),'n':int(len(a))}

def simulate(m,x,L):
 d=mujoco.MjData(m);d.qpos[:7]=x[0,:7];d.qvel[:6]=x[0,7:13];mujoco.mj_forward(m,d)
 out=np.zeros((L,13)); touch=np.zeros(L,bool);vz=np.zeros(L)
 for i in range(L):
  out[i,:7]=d.qpos[:7];out[i,7:13]=d.qvel[:6];touch[i]=d.ncon>0;vz[i]=d.qvel[2]
  if i<L-1:mujoco.mj_step(m,d)
 starts=np.flatnonzero(np.diff(np.r_[False,touch,False].astype(int))==1);stops=np.flatnonzero(np.diff(np.r_[False,touch,False].astype(int))==-1)-1
 tc=e=float('nan')
 if len(starts) and len(stops):
  s=int(starts[0]);p=int(stops[stops>=s][0]);tc=(p-s+1)*DT
  if s and p+1<L and vz[s-1]<0:e=float(-vz[p+1]/vz[s-1])
 return out,e,tc

def score(c,ids,a,lengths,keep=False):
 m=make_model(c);runs=[];t=time.perf_counter()
 for i in ids:
  L=int(lengths[i]);real=a[i,:L];sim,e,tc=simulate(m,real,L)
  # quaternion sign is physically equivalent; positions and linear velocity
  # are directly aligned in SI units.
  pos_rmse=float(np.sqrt(np.mean(np.sum((sim[:,:3]-real[:,:3])**2,axis=1))))
  vel_rmse=float(np.sqrt(np.mean(np.sum((sim[:,7:10]-real[:,7:10])**2,axis=1))))
  final_pos=float(np.linalg.norm(sim[-1,:3]-real[-1,:3]));final_rot=qangle(sim[-1,3:7],real[-1,3:7]);final_rot_sym=qangle_sym(sim[-1,3:7],real[-1,3:7])
  runs.append({'id':int(i),'position_rmse_m':pos_rmse,'position_dtw_m':dtw(sim[:,:3],real[:,:3]),'linear_velocity_rmse_m_s':vel_rmse,'final_position_error_m':final_pos,'final_orientation_error_rad':final_rot,'final_orientation_sym_rad':final_rot_sym,'final_up_face_agreement':float(upward_face(sim[-1,3:7])==upward_face(real[-1,3:7])),'e_com_proxy':e,'contact_duration_s':tc,'postimpact_linear_speed_error_m_s':float(abs(np.linalg.norm(sim[-1,7:10])-np.linalg.norm(real[-1,7:10]))),'postimpact_angular_speed_error_rad_s':float(abs(np.linalg.norm(sim[-1,10:13])-np.linalg.norm(real[-1,10:13])))})
 out={'config':c,'rollouts':len(ids),'wall_s':time.perf_counter()-t,
 'trajectory_position_rmse_m':ci([r['position_rmse_m'] for r in runs]),'trajectory_position_dtw_m':ci([r['position_dtw_m'] for r in runs]),'linear_velocity_rmse_m_s':ci([r['linear_velocity_rmse_m_s'] for r in runs]),'final_position_error_m':ci([r['final_position_error_m'] for r in runs]),'final_orientation_error_rad':ci([r['final_orientation_error_rad'] for r in runs]),'final_orientation_sym_rad':ci([r['final_orientation_sym_rad'] for r in runs]),'final_up_face_agreement':ci([r['final_up_face_agreement'] for r in runs]),'effective_restitution_com_proxy':ci([r['e_com_proxy'] for r in runs]),'first_contact_duration_s':ci([r['contact_duration_s'] for r in runs]),'postimpact_linear_speed_error_m_s':ci([r['postimpact_linear_speed_error_m_s'] for r in runs]),'postimpact_angular_speed_error_rad_s':ci([r['postimpact_angular_speed_error_rad_s'] for r in runs])}
 if keep:out['per_rollout']=runs
 out['selection_loss']=out['trajectory_position_rmse_m']['mean']+.05*out['final_orientation_error_rad']['mean'];return out

def main():
 p=argparse.ArgumentParser();p.add_argument('--stage',choices=('b1','b5','b100'),required=True);p.add_argument('--output',type=Path,default=OUT);q=p.parse_args()
 d=np.load(SRC);a=d['state'];lengths=d['lengths'];eligible=[i for i,L in enumerate(lengths) if a[i,0,2]>2*HALF and np.nanmin(a[i,:L,2])<.09]
 # Fixed stratified-by-file-order 32+32 sample; remaining released tosses are not used for tuning.
 train=eligible[::2][:32];test=eligible[1::2][:32];grid=configs(q.stage);rows=[score(c,train,a,lengths) for c in grid];chosen=min(rows,key=lambda x:x['selection_loss']);held=score(chosen['config'],test,a,lengths,True)
 out={'protocol':'dynamic-impact-v1','task':'ContactNets real cube toss','engine':'MuJoCo','engine_version':mujoco.__version__,'stage':q.stage,'budget_configurations':len(grid),'split':'32 alternating eligible train / 32 held-out','physics':{'dt_s':DT,'mass_kg':MASS,'mu':MU,'cube_half_width_m':HALF,'inertia_kg_m2':INERTIA},'alignment':'released position/quaternion/linear+angular velocity directly initialized; zero controls after release','selection_metric':'train position RMSE + .05*final orientation error (rad)','train_candidates':rows,'frozen_config':chosen['config'],'heldout_metrics':held,'caveat':'COM restitution is only a multi-point-impact proxy; it is not a material restitution coefficient.'}
 q.output.mkdir(parents=True,exist_ok=True);(q.output/(q.stage+'.json')).write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'stage':q.stage,'frozen':chosen['config'],'heldout':held},indent=2))
if __name__=='__main__':main()
