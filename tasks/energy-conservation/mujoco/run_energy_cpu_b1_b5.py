#!/usr/bin/env python3
"""Strict B=1/B=5, R=20 CPU counterparts for energy-conservation-v1."""
from __future__ import annotations
import json, math, time
from pathlib import Path
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'exp-results/protocol-v2-native-default/energy-conservation-v1/mujoco_cpu'
G=9.81

def pend_xml(dt):
    opt='' if dt is None else f'<option timestep="{dt}" integrator="implicitfast" solver="Newton"/>'
    return f'<mujoco model="pendulum">{opt}<worldbody><body><joint type="hinge" axis="0 1 0" damping="0"/><geom type="sphere" pos="0 0 -1" size=".02" mass="1"/></body></worldbody></mujoco>'

def cradle_xml(c):
    bodies=''.join(f'<body pos="0 {i*.04} .2"><joint type="hinge" axis="1 0 0" damping="0"/><body pos="0 0 -.13263"><geom type="sphere" size=".02" mass=".033"/></body></body>' for i in range(5))
    if c.get('native'): return f'<mujoco model="cradle"><size nconmax="32" njmax="128"/><worldbody>{bodies}</worldbody></mujoco>'
    return f'<mujoco model="cradle"><option timestep="{c["dt"]}" integrator="implicitfast" solver="Newton"/><size nconmax="32" njmax="128"/><default><geom solref="{c["tc"]} {c["dr"]}" solimp=".99 .99 .001 .5 2"/></default><worldbody>{bodies}</worldbody></mujoco>'

def pend_one(c,seed):
    m=mujoco.MjModel.from_xml_string(pend_xml(c['dt']));d=mujoco.MjData(m);rng=np.random.default_rng(seed);d.qpos[0]=math.pi/2+rng.uniform(-.003,.003);mujoco.mj_forward(m,d)
    vals=[];q=[];v=[];sample=max(1,round(.01/m.opt.timestep)); st=time.perf_counter()
    for k in range(round(10/m.opt.timestep)+1):
        if k%sample==0:q.append(d.qpos[0]);v.append(d.qvel[0])
        if k<round(10/m.opt.timestep):mujoco.mj_step(m,d)
    q=np.asarray(q);v=np.asarray(v);e=.5*v*v+G*(1-np.cos(q));return {'max_relative_energy_drift':float(np.max(abs(e-e[0]))/e[0]),'final_energy_ratio':float(e[-1]/e[0]),'wall_s':time.perf_counter()-st}

def cradle_one(c,seed):
    m=mujoco.MjModel.from_xml_string(cradle_xml(c));d=mujoco.MjData(m);d.qpos[0]=-math.pi/2+np.random.default_rng(seed).uniform(-.003,.003);mujoco.mj_forward(m,d); sample=max(1,round(.01/m.opt.timestep));v=[];e=[];st=time.perf_counter()
    for k in range(round(3/m.opt.timestep)+1):
        if k%sample==0:v.append(d.qvel.copy());e.append(.5*.033*.13263**2*np.sum(d.qvel*d.qvel)+.033*G*.13263*np.sum(1-np.cos(d.qpos)))
        if k<round(3/m.opt.timestep):mujoco.mj_step(m,d)
    v=np.asarray(v);e=np.asarray(e);mte=np.max(abs(v[:,4]))/max(np.max(abs(v[:,0])),1e-9);quiet=np.max(abs(v[:,1:4]),axis=1)<.01;long=cur=0
    for x in quiet:cur=cur+1 if x else 0;long=max(long,cur)
    return {'mte':float(mte),'lsd_s':float(long*sample*m.opt.timestep),'final_energy_ratio':float(e[-1]/e[0]),'wall_s':time.perf_counter()-st}

def agg(rows):
    d={'R':len(rows)}
    for k in rows[0]:d[k]={'mean':float(np.mean([r[k] for r in rows])),'std':float(np.std([r[k] for r in rows],ddof=1))}
    return d

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    pc=[{'id':'native_default','dt':None},{'id':'dt0p5ms','dt':.0005},{'id':'dt1ms','dt':.001},{'id':'dt2ms','dt':.002},{'id':'dt5ms','dt':.005}]
    pr=[]
    for i,c in enumerate(pc):pr.append({'candidate':c,'metrics':agg([pend_one(c,10100+i*100+r) for r in range(20)])})
    pb=min(pr,key=lambda x:(x['metrics']['max_relative_energy_drift']['mean'],x['metrics']['wall_s']['mean']))
    cc=[{'id':'native_default','native':True},{'id':'tc1','dt':.001,'tc':.001,'dr':2},{'id':'tc2','dt':.001,'tc':.002,'dr':2},{'id':'tc4','dt':.001,'tc':.004,'dr':2},{'id':'tc8','dt':.001,'tc':.008,'dr':2}]
    cr=[]
    for i,c in enumerate(cc):cr.append({'candidate':c,'metrics':agg([cradle_one(c,20100+i*100+r) for r in range(20)])})
    cb=max(cr,key=lambda x:(x['metrics']['final_energy_ratio']['mean'],x['metrics']['mte']['mean']))
    out={'protocol':'energy-conservation-v1','engine':'MuJoCo CPU','engine_version':mujoco.__version__,'R':20,'pendulum':{'B1':pr[0],'B5_candidates':pr,'frozen_B5':pb,'selection_rule':'min energy drift'},'newtons_cradle':{'B1':cr[0],'B5_candidates':cr,'frozen_B5':cb,'selection_rule':'max final energy retention; MTE tie-breaker'}}
    (OUT/'energy_b1_b5_r20.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'pendulum':pb,'cradle':cb},indent=2))
if __name__=='__main__':main()
