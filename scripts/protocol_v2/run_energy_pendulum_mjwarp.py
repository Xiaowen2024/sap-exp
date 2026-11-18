#!/usr/bin/env python3
"""Energy-conservation-v1 pendulum: literal B=1 and predeclared B=5 on MJWarp.

The task is deliberately analytic (one hinge, no contact): energy drift and
nonlinear period are directly observable, while R=20 consists only of small
initial-angle perturbations.
"""
from __future__ import annotations
import json, math, time
from pathlib import Path
import mujoco, mujoco_warp as mjw
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'exp-results/protocol-v2-native-default/energy-conservation-v1/mujoco_warp'
G=9.81; M=1.; L=1.; T=10.; R=20

def xml(dt: float|None)->str:
    opt='' if dt is None else f'<option timestep="{dt}" integrator="implicitfast" solver="Newton"/>'
    return f'''<mujoco model="energy_pendulum"><compiler angle="radian"/>{opt}
    <worldbody><body name="bob" pos="0 0 0"><joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
    <geom type="sphere" pos="0 0 -{L}" size=".02" mass="{M}"/></body></worldbody></mujoco>'''

def metric(q:np.ndarray,v:np.ndarray,dt:float)->dict:
    e=.5*M*L*L*v*v+M*G*L*(1-np.cos(q)); e0=e[0]
    loss=float(np.max(np.abs(e-e0))/max(e0,1e-12)); growth=float(np.max(e/e0))
    x=(q-q.mean())*np.hanning(len(q)); f=np.fft.rfftfreq(16384,dt); p=np.abs(np.fft.rfft(x,16384)); mask=(f>.1)&(f<2.)
    period=float(1/f[mask][np.argmax(p[mask])])
    return {'max_relative_energy_drift':loss,'max_energy_ratio':growth,'fft_period_s':period,'final_energy_ratio':float(e[-1]/e0)}

def run(c:dict, seed:int)->dict:
    m=mujoco.MjModel.from_xml_string(xml(c['dt']))
    gm=mjw.put_model(m); d=mjw.make_data(m,nworld=R,nconmax=4,njmax=8)
    rng=np.random.default_rng(seed); q0=np.pi/2+rng.uniform(-.003,.003,R)
    qp=d.qpos.numpy(); qp[:,0]=q0; d.qpos.assign(qp); mjw.forward(gm,d)
    steps=round(T/m.opt.timestep); sample=max(1,round(.01/m.opt.timestep)); qs=[];vs=[]; start=time.perf_counter()
    for i in range(steps+1):
        if i%sample==0:
            qs.append(d.qpos.numpy()[:,0]);vs.append(d.qvel.numpy()[:,0])
        if i<steps:mjw.step(gm,d)
    qs=np.asarray(qs);vs=np.asarray(vs); dt=m.opt.timestep*sample
    rows=[metric(qs[:,i],vs[:,i],dt) for i in range(R)]
    keys=rows[0].keys(); agg={k:{'mean':float(np.mean([x[k] for x in rows])),'std':float(np.std([x[k] for x in rows],ddof=1))} for k in keys}
    return {'candidate':c,'R':R,'dt_ms':m.opt.timestep*1e3,'duration_s':T,'metrics':agg,'overflow_worlds':int(np.sum(d.overflow.numpy()>0)),'wall_s':time.perf_counter()-start}

def main():
    # None preserves compiled MuJoCo defaults: literal out-of-box B=1.
    cs=[{'id':'native_default','dt':None},{'id':'dt_0.5ms','dt':.0005},{'id':'dt_1ms','dt':.001},{'id':'dt_2ms','dt':.002},{'id':'dt_5ms','dt':.005}]
    rows=[run(c,20261028+i) for i,c in enumerate(cs)]
    # energy drift is primary; reject overflow then use wall time only as tie-break.
    valid=[x for x in rows if x['overflow_worlds']==0]
    chosen=min(valid,key=lambda x:(x['metrics']['max_relative_energy_drift']['mean'],x['wall_s']))
    out={'protocol':'energy-conservation-v1','task':'analytic_pendulum','engine':'MuJoCo Warp','engine_version':mujoco.__version__,'B1':rows[0],'B5_candidates':rows,'frozen_B5':chosen,'selection':'minimum mean max-relative-energy-drift; R=20 small-angle perturbations'}
    OUT.mkdir(parents=True,exist_ok=True);(OUT/'pendulum_b1_b5_r20.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'B1':rows[0]['metrics'],'B5':chosen['candidate'],'B5_metrics':chosen['metrics']},indent=2))
if __name__=='__main__':main()
