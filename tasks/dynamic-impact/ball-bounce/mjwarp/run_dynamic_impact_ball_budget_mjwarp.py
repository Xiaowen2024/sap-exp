#!/usr/bin/env python3
"""MJWarp batched counterpart of run_dynamic_impact_ball_budget.py.

Run on the CUDA host after copying this script, its MuJoCo helper, GAUGE JSON
files, and ball OBJ into one staging directory.  It evaluates all 11 initial
conditions in a single ``nworld=11`` batch for every candidate configuration.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse, json, math, time
from pathlib import Path

import mujoco
import mujoco_warp as mjw
import numpy as np

from run_dynamic_impact_ball_budget import (DATA, DT, GROUND_Z, MASS, MESH, MU,
    ROOT, ci, configs, dtw, load_trials, mjcf, peaks)


def simulate_batch(cfg: dict, initials: np.ndarray, frame_counts: list[int]) -> list[dict]:
    mjm = mujoco.MjModel.from_xml_string(mjcf(cfg))
    m = mjw.put_model(mjm)
    nw = len(initials); d = mjw.make_data(mjm, nworld=nw, nconmax=16, njmax=64)
    q = d.qpos.numpy(); q[:, :3] = initials; q[:, 3] = 1.0; d.qpos.assign(q)
    mjw.forward(m, d)
    nsteps = int(math.ceil((max(frame_counts)-1) / 30.0 / DT))
    sample = [np.zeros((n,3)) for n in frame_counts]
    contact = np.zeros((nw, nsteps+1), dtype=bool); vz = np.zeros((nw,nsteps+1))
    wants = [{int(round((i / 30.0) / DT)): i for i in range(n)} for n in frame_counts]
    for step in range(nsteps+1):
        qp, qv = d.qpos.numpy(), d.qvel.numpy()
        for w in range(nw):
            ix = wants[w].get(step)
            if ix is not None: sample[w][ix] = qp[w,:3]
        # ``nacon`` is the active collision-candidate count (global across
        # worlds); contact.worldid maps those first nacon records back to a
        # batch world.  Unlike stale EFC buffer entries, this is valid for
        # detecting touch/liftoff in a batched run.
        nac = int(d.nacon.numpy()[0])
        if nac:
            worlds = d.contact.worldid.numpy()[:nac]
            contact[worlds, step] = True
        vz[:,step] = qv[:,2]
        if step < nsteps: mjw.step(m,d)
    overflow = d.overflow.numpy()
    result=[]
    for w in range(nw):
        c, v = contact[w], vz[w]; starts=np.flatnonzero(np.diff(np.r_[False,c,False].astype(int))==1)
        stops=np.flatnonzero(np.diff(np.r_[False,c,False].astype(int))==-1)-1
        tc=e=float('nan')
        if len(starts) and len(stops):
            st=int(starts[0]); sp=int(stops[stops>=st][0]); tc=(sp-st+1)*DT
            if st>0 and sp+1<len(v) and v[st-1]<0: e=float(-v[sp+1]/v[st-1])
        result.append({'xyz':sample[w], 'e':e, 'tc_s':tc, 'peaks':peaks(sample[w][:,2]),
                       'overflow':bool(overflow[w])})
    return result


def score(cfg, ids, names, real, keep=False):
    initial=np.asarray([real[i][0] for i in ids]); sims=simulate_batch(cfg, initial, [len(real[i]) for i in ids])
    runs=[]; t0=time.perf_counter()
    for i,sim in zip(ids,sims):
        ref=real[i]; rmse=float(np.sqrt(np.mean(np.sum((sim['xyz']-ref)**2,axis=1))))
        p,q=sim['peaks'][:3],peaks(ref[:,2])[:3]
        apex=float(np.mean([abs(a-b)/max(abs(b),1e-6) for a,b in zip(p,q)])) if p and q else float('nan')
        runs.append({'trial':names[i],'rmse_m':rmse,'dtw_m':dtw(sim['xyz'],ref),'e':sim['e'],'tc_s':sim['tc_s'],
                     'apex_height_rel_error':apex,'sim_apex_heights_m':p,'real_apex_heights_m':q,'overflow':sim['overflow']})
    out={'config':cfg,'rollouts':len(ids),'batch_worlds':len(ids),'postprocess_wall_s':time.perf_counter()-t0,
         'trajectory_rmse_m':ci([r['rmse_m'] for r in runs]),'dtw_m':ci([r['dtw_m'] for r in runs]),
         'effective_restitution':ci([r['e'] for r in runs]),'contact_duration_s':ci([r['tc_s'] for r in runs]),
         'apex_height_decay_error':ci([r['apex_height_rel_error'] for r in runs]),'overflow_worlds':int(sum(r['overflow'] for r in runs))}
    if keep: out['per_rollout']=runs
    out['selection_loss']=out['trajectory_rmse_m']['mean']+0.05*(out['apex_height_decay_error']['mean'] or 1.0)
    return out


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--stage',choices=('b1','b5','b100'),required=True); ap.add_argument('--output',type=Path,required=True); a=ap.parse_args()
    names,real=load_trials(); train,test=list(range(0,22,2)),list(range(1,22,2)); grid=configs(a.stage)
    rows=[score(c,train,names,real) for c in grid]; selected=min(rows,key=lambda x:x['selection_loss']); held=score(selected['config'],test,names,real,True)
    out={'protocol':'dynamic-impact-v1','task':'GAUGE public bouncing ball','engine':'MuJoCo Warp','engine_version':mujoco.__version__,
         'stage':a.stage,'budget_configurations':len(grid),'execution':'CUDA batched; 11 worlds/configuration','split':'odd file IDs train; even file IDs held-out',
         'initialization':'each real frame-0 position; zero release velocity (public-data limitation)',
         'physics':{'dt_s':DT,'mass_kg':MASS,'friction':MU,'ground_z_m':GROUND_Z,'mesh':str(MESH)},
         'selection_metric':'train mean trajectory RMSE + 0.05*train mean apex-height relative error','train_candidates':rows,
         'frozen_config':selected['config'],'heldout_metrics':held,
         'limitations':{'real_tc':'unavailable at 30 Hz; simulation-only value','real_e':'apex proxy only; no released high-rate velocity'}}
    a.output.mkdir(parents=True,exist_ok=True); (a.output/(a.stage+'.json')).write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({'stage':a.stage,'frozen':selected['config'],'heldout':held},indent=2))
if __name__=='__main__': main()
