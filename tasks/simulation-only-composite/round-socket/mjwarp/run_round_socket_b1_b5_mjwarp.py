#!/usr/bin/env python3
"""Strict B=1/B=5, R=20 MuJoCo Warp round peg--socket evaluation."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import json, re, time
from pathlib import Path
import mujoco, mujoco_warp as mjw, warp as wp
import numpy as np

ROOT=REPO_ROOT
OUT=ROOT/'exp-results/protocol-v2-native-default/composite-v1/round-socket/mujoco_warp'
ASSET=ROOT/'assets/round_socket_collision'
R,T,SEED,NCON=20,2.,20261029,8192

@wp.kernel
def _accumulate_peg_min_distance(
    ncon: wp.array(dtype=int), worldid: wp.array(dtype=int),
    geom: wp.array(dtype=wp.vec2i), dist: wp.array(dtype=float),
    peg: int, min_dist: wp.array(dtype=float),
):
    """Keep the most-negative peg contact distance per world on the GPU."""
    i = wp.tid()
    if i < ncon[0]:
        pair = geom[i]
        if pair[0] == peg or pair[1] == peg:
            wp.atomic_min(min_dist, worldid[i], dist[i])

def candidates():
 return [
  {'id':'native_default','native':True},
  {'id':'tc0p75ms_dr1_d95','dt':.0005,'tc':.00075,'dr':1.,'d':.95},
  {'id':'tc1ms_dr1_d95','dt':.0005,'tc':.001,'dr':1.,'d':.95},
  {'id':'tc2ms_dr2_d95','dt':.0005,'tc':.002,'dr':2.,'d':.95},
  {'id':'tc4ms_dr2_d99','dt':.001,'tc':.004,'dr':2.,'d':.99},
 ]

def xml(c,clearance,offset):
 assets=''.join(f'<mesh name="p{i}" file="socket_piece_{i:02d}.obj"/>' for i in range(32))
 pieces=''.join(f'<geom name="piece{i}" type="mesh" mesh="p{i}"/>' for i in range(32))
 # B=1 keeps the task's fixed 0.5-ms clock and gravity but inherits all
 # compiled native solver/contact defaults.  It must not silently switch dt.
 option='<option timestep="0.0005" gravity="0 0 -9.81"/>' if c.get('native') else f'<option timestep="{c["dt"]}" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" solver="Newton"/>'
 default='<default><geom friction=".8 .005 .0001"/></default>' if c.get('native') else f'<default><geom friction=".8 .005 .0001" solref="{c["tc"]} {c["dr"]}" solimp="{c["d"]} {c["d"]} .001 .5 2"/></default>'
 r=(12.-clearance)/1000.
 return f'<mujoco model="round_socket">{option}<compiler meshdir="{ASSET}"/><size nconmax="{NCON}" njmax="32768"/>{default}<asset>{assets}</asset><worldbody>{pieces}<body name="peg" pos="{offset/1000} 0 .12" euler="0 1 0"><freejoint/><geom name="peg_geom" type="cylinder" size="{r} .025" mass=".08"/></body></worldbody></mujoco>'

def run(c,clearance,offset,seed):
 m=mujoco.MjModel.from_xml_string(xml(c,clearance,offset)); gm=mjw.put_model(m);d=mjw.make_data(m,nworld=R,nconmax=NCON,njmax=32768)
 q=d.qpos.numpy(); rng=np.random.default_rng(seed);q[:,0]+=rng.uniform(-.00005,.00005,R);q[:,1]+=rng.uniform(-.00005,.00005,R);d.qpos.assign(q.astype(np.float32));mjw.forward(gm,d)
 peg=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'peg_geom');
 # This avoids host synchronization at every 0.5-ms step while retaining an
 # exact, all-step peak-overlap diagnostic rather than a sparse sample.
 min_dist=wp.zeros(R,dtype=float,device='cuda'); final_dist=wp.zeros(R,dtype=float,device='cuda');st=time.perf_counter()
 for _ in range(round(T/m.opt.timestep)):
  mjw.step(gm,d)
  wp.launch(_accumulate_peg_min_distance,dim=NCON,inputs=[d.nacon,d.contact.worldid,d.contact.geom,d.contact.dist,peg,min_dist],device='cuda')
 wp.launch(_accumulate_peg_min_distance,dim=NCON,inputs=[d.nacon,d.contact.worldid,d.contact.geom,d.contact.dist,peg,final_dist],device='cuda')
 peak=np.maximum(0.,-min_dist.numpy())*1e3;pf=np.maximum(0.,-final_dist.numpy())*1e3
 q=d.qpos.numpy(); x,y,z=q[:,0]*1e3,q[:,1]*1e3,q[:,2]*1e3;rad=np.hypot(x,y); overflow=int(np.sum(d.overflow.numpy()>0))
 rows=[]
 for w in range(R):
  entered=bool(z[w]<=45 and rad[w]<=3);escaped=bool(rad[w]>50 or z[w]<-10 or z[w]>160)
  cred=bool(entered and rad[w]<=clearance and not escaped and overflow==0 and peak[w]<=.1*clearance and pf[w]<=.1*clearance)
  rows.append({'seed':seed+w,'final_xyz_mm':[float(x[w]),float(y[w]),float(z[w])],'final_radial_mm':float(rad[w]),'entered':entered,'escaped':escaped,'overflow_worlds':overflow,'pmax_mm':float(peak[w]),'pfinal_mm':float(pf[w]),'pmax_over_clearance':float(peak[w]/clearance),'credible':cred})
 return {'candidate':c,'R':R,'dt_ms':m.opt.timestep*1e3,'solver':int(m.opt.solver),'integrator':int(m.opt.integrator),'clearance_mm':clearance,'initial_offset_mm':offset,'credible_count':sum(x['credible'] for x in rows),'credible_rate':float(np.mean([x['credible'] for x in rows])),'pmax_over_clearance_mean':float(np.mean([x['pmax_over_clearance'] for x in rows])),'final_radial_mm_mean':float(np.mean([x['final_radial_mm'] for x in rows])),'overflow_worlds':overflow,'wall_s':time.perf_counter()-st,'rollouts':rows}

def key(x):return(-x['credible_rate'],x['pmax_over_clearance_mean'],x['final_radial_mm_mean'],x['wall_s'])
def main():
 OUT.mkdir(parents=True,exist_ok=True);sel=[]
 for i,c in enumerate(candidates()):
  x=run(c,2.,1.,SEED+i*1000);sel.append(x);print(json.dumps({k:x[k] for k in x if k not in ('candidate','rollouts')}),flush=True)
 win=min(sel,key=key);held=[]
 for i,(cl,off) in enumerate((z for z in [(a,b) for a in (2.,1.5,1.,.5,.25) for b in (0.,1.)] if z!=(2.,1.))):
  x=run(win['candidate'],cl,off,SEED+100000+i*1000);held.append(x);print(json.dumps({k:x[k] for k in x if k not in ('candidate','rollouts')}),flush=True)
 out={'protocol':'composite-v1 round peg--socket strict B=1/B=5','engine':'MuJoCo Warp','engine_version':mujoco.__version__,'representation':'shared 32-piece CoACD','selection_condition':'2-mm clearance, 1-mm offset, 1-degree tilt, 2 s','perturbation':'independent uniform lateral x/y ±0.05 mm, R=20','selection_rule':'max credible rate -> min mean pmax/clearance -> min final radial -> wall time','B1':sel[0],'B5_candidates':sel,'frozen_B5':win,'heldout':held}
 (OUT/'round_socket_b1_b5_r20.json').write_text(json.dumps(out,indent=2)+'\n')
if __name__=='__main__':main()
