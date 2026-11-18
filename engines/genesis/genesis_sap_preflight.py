#!/usr/bin/env python3
"""API and direct-force checks only; no task metric used in candidate choice."""
import argparse
import numpy as np
import genesis_sap_common as G
from genesis_rigid_preflight import check

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=G.Path,required=True);a=ap.parse_args()
 out={'kind':'preflight_not_benchmark','checks':[]}; gs=G.gs_init();nat=G.parse_candidate('native default',{});probe=G.parse_candidate('hydroelastic_stiffness=1e8,sap_taud=0.01',{'nonnative':{'n_sap_iterations':20}})
 box=lambda i:G.mjcf(f'<body pos="0 0 {.02+.04*i}"><freejoint/><geom type="box" size=".02 .02 .02" mass="1" friction=".3 0 0"/></body>')
 b=G.build({'ground':G.mjcf('<geom type="plane" size="2 2 .1" friction=".3 0 0"/>'),**{f'b{i}':box(i) for i in range(3)}},dt=.001,cand=probe,n_envs=2)
 print('BUILT',flush=True)
 pairs=[('ground','b0'),('b0','b1'),('b1','b2')]
 history=[];residual=[]
 for step in range(500):
  v0=[G.np_(b.ents[f'b{i}'].get_dofs_velocity())[:,:3].copy() for i in range(3)] if step>=400 else None
  b.scene.step()
  if step>=400:
   con=b.contacts();loads=[b.pair_normal_load(x,y,con)[0] for x,y in pairs];history.append(loads)
   # Whole-stack direct resultant versus observed COM momentum. Internal forces cancel.
   f=b.force_on('b0','ground',con)[0]
   dv=sum(G.np_(b.ents[f'b{i}'].get_dofs_velocity())[:,:3]-v0[i] for i in range(3))/.001
   residual.append(f-dv-np.array([0,0,3*9.81]))
  if step%200==199:print('stack step',step+1,flush=True)
 loads=np.mean(history,axis=0)
 check('time-averaged direct SAP stacked support 3/2/1 mg',np.allclose(loads,np.array([[3],[2],[1]])*9.81,rtol=.01),loads.tolist(),out)
 rr=np.asarray(residual)
 check('direct force agrees with whole-stack momentum',np.max(np.abs(rr.mean(0)))/29.43<.01,{'mean_residual_N':rr.mean(0).tolist(),'max_residual_N':float(np.abs(rr).max())},out)
 f=b.force_on('b1','b0')[0];check('direct pair sign',np.all(f[:,2]>0),f.tolist(),out)
 p=b.provenance();check('preflight convergence configuration compiled',p['sap_options']['n_sap_iterations']==20 and p['sap_options']['sap_taud']==.01,p['sap_options'],out)
 check('literal mass and zero armature',np.allclose(p['entities']['b0']['links_mass'],1) and np.allclose(p['entities']['b0']['dofs_armature'],0),p['entities']['b0'],out)
 check('floor friction is .3',np.allclose(G.np_(b.rs.dyn_info.geoms.coup_friction.to_numpy())[1:],.3),{},out)
 out['compiled']=p;b.destroy()
 # Free flight with anisotropic orientation, then external force, checks actual state propagation.
 free=G.mjcf('<body pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass=".37" diaginertia=".00081 .00081 .00081"/><geom type="box" size=".05 .05 .05" friction=".18 0 0"/></body>')
 pad=G.mjcf('<body pos="0 0 3"><joint type="slide" axis="1 0 0"/><joint type="slide" axis="0 0 1"/><geom type="box" size=".01 .02 .04" mass="1" friction=".18 0 0"/></body>')
 b=G.build({'cube':free,'pad':pad},dt=.001,gravity=(0,0,0),cand=nat,n_envs=2);e=b.ents['cube']
 check('B1 compiled defaults remain literal',b.cp.options.model_dump()==gs.options.SAPCouplerOptions().model_dump(),b.cp.options.model_dump(),out)
 check('explicit manifest-style inertia retained',np.allclose(G.np_(e.get_links_mass()),.37) and np.allclose(G.np_(e.get_links_inertia())[0],np.eye(3)*.00081),G.np_(e.get_links_inertia()).tolist(),out)
 q=np.tile([0,0,1,np.cos(np.pi/4),np.sin(np.pi/4),0,0],(2,1));e.set_qpos(q)
 e.set_dofs_velocity(np.tile([.3,0,0,0,0,1.],(2,1)))
 for _ in range(100):b.scene.step()
 check('world linear velocity propagates',np.allclose(G.np_(e.get_qpos())[:,0],.03,atol=1e-5),G.np_(e.get_qpos()).tolist(),out)
 check('angular velocity body frame',np.allclose(G.np_(e.get_ang()),[[0,-1,0]]*2,atol=1e-3),G.np_(e.get_ang()).tolist(),out)
 v0=G.np_(e.get_dofs_velocity()).copy()
 for _ in range(100):
  b.ents['pad'].control_dofs_force(np.tile([4.,0.],(2,1)))
  e.apply_links_external_wrench(force=np.tile([[[.37,0,0]]],(2,1,1)),links_idx_local=[0],ref=gs.link_ref_frame.link_COM);b.scene.step()
 check('external force F/m acceleration',np.allclose(G.np_(e.get_dofs_velocity())[:,0]-v0[:,0],.1,atol=1e-4),G.np_(e.get_dofs_velocity()).tolist(),out)
 check('slide-joint control force and zero armature',np.allclose(G.np_(b.ents['pad'].get_dofs_position())[:,0],.0202,atol=2e-4) and np.allclose(G.np_(b.ents['pad'].get_dofs_armature()),0),G.np_(b.ents['pad'].get_dofs_position()).tolist(),out)
 b.destroy();out['all_pass']=all(x['pass'] for x in out['checks']);G.save_json(a.out,out)
 print('ALL PASS' if out['all_pass'] else 'PREFLIGHT FAILED',flush=True)
 if not out['all_pass']:raise SystemExit(1)
if __name__=='__main__':main()
