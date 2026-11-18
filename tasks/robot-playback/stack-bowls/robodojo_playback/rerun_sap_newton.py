"""Matched collision-backend reruns of the frozen RoboDojo pilot scenes.

Run on the GPU host in an isolated directory containing scripts/ and sap/src/.
Input scenes/targets are frozen by the staging step; no tuning is performed.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import numpy as np
from scipy.spatial.transform import Rotation


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',required=True);ap.add_argument('--resume',action='store_true');a=ap.parse_args()
    root=Path(a.root).resolve();status=json.loads((root/'status.json').read_text()) if a.resume and (root/'status.json').exists() else {'kind':'matched_exploratory_rerun_not_benchmark','jobs':{}}
    (root/'tmp').mkdir(exist_ok=True)
    def checkpoint():
        (root/'status.json').write_text(json.dumps(status,indent=2))
    def run(name,scene,backend,drive=False,steps=None,execution='graph'):
        out=root/'runs'/name
        if a.resume and (out/'health.json').exists():
            h=json.loads((out/'health.json').read_text())
            if (h.get('convex_backend')==backend and h.get('sap_module_path','').startswith(str(root/'sap/src')+'/')
                and h['finite'] and h['steps']==h['expected_steps'] and not h.get('truncated')):
                print('REUSE',name,flush=True)
                return out,h
        env=dict(os.environ,PYTHONPATH=str(root/'sap/src')+':'+str(root),
                 SAP_WARP_CONVEX_BACKEND=backend,ROBODOJO_SAP_EXECUTION=execution,
                 ROBODOJO_WARP_CACHE=str(root/'warp_cache'/backend),
                 WARP_CACHE_PATH=str(root/'warp_cache'/backend),
                 TMPDIR=str(root/'tmp'),
                 OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        cmd=[sys.executable,'-u','-m','scripts.robodojo_playback.run','--engine','sap',
             '--scene',str(root/'scenes'/scene),'--out',str(out)]
        if drive:cmd.append('--drive-only')
        if scene=='r2':cmd.append('--sap-graph')
        if steps is not None:cmd+=['--max-steps',str(steps)]
        status['jobs'][name]={'state':'running','backend':backend,'scene':scene};checkpoint()
        print('START',name,flush=True)
        with (root/(name+'.log')).open('w') as log:
            result=subprocess.run(cmd,env=env,cwd=root,stdout=log,stderr=subprocess.STDOUT)
        status['jobs'][name].update(state='finished',exit_code=result.returncode);checkpoint()
        if result.returncode:raise RuntimeError(name+' failed; inspect '+str(root/(name+'.log')))
        h=json.loads((out/'health.json').read_text())
        if not h['finite'] or h['steps']!=h['expected_steps'] or h.get('truncated'):
            raise RuntimeError(name+' incomplete, nonfinite, or contact overflow')
        if h['convex_backend']!=backend or not h['sap_module_path'].startswith(str(root/'sap/src')+'/'):
            raise RuntimeError(name+' imported the wrong SAP source or collision backend')
        status['jobs'][name]['health']={k:h[k] for k in ('finite','steps','drive_gate_pass','wall_s','max_contacts','engine_peak_overlap_m')};checkpoint()
        print('DONE',name,status['jobs'][name]['health'],flush=True)
        return out,h
    for backend in ('newton','legacy'):
        host,_=run('validate_r3_'+backend+'_host','r3',backend,steps=100,execution='host')
        graph,_=run('validate_r3_'+backend+'_graph','r3',backend,steps=100)
        with np.load(host/'trace.npz') as h,np.load(graph/'trace.npz') as g:
            errors={k:float(np.max(np.abs(h[k]-g[k]))) for k in h.files}
            pos,angle,twist=0.,0.,0.
            for key in h.files:
                if key.endswith('__pose'):
                    pos=max(pos,float(np.linalg.norm(h[key][:,:3]-g[key][:,:3],axis=1).max()))
                    ra=Rotation.from_quat(h[key][:,[4,5,6,3]]);rb=Rotation.from_quat(g[key][:,[4,5,6,3]])
                    angle=max(angle,float(np.degrees((ra*rb.inv()).magnitude()).max()))
                elif key.endswith('__velocity'):twist=max(twist,errors[key])
            clocks_match=bool(np.allclose(h['time'],g['time'],atol=1e-12,rtol=0))
        # Same physical tolerances as validate_graph.py; pose and twist have different units.
        status.setdefault('graph_validation',{})[backend]={'max_position_m':pos,'max_rotation_deg':angle,
            'max_twist_component_difference':twist,'clocks_match':clocks_match,'fields':errors,
            'thresholds':{'position_m':1e-4,'rotation_deg':.05,'twist_component':.01},
            'pass':bool(clocks_match and pos<=1e-4 and angle<=.05 and twist<=.01)};checkpoint()
        if not status['graph_validation'][backend]['pass']:raise RuntimeError('graph/host state mismatch')
        _,h=run('sap_drive_r3_'+backend,'r3',backend,drive=True)
        if not h['drive_gate_pass']:raise RuntimeError('no-object drive gate failed')
        run('sap_objects_r3_'+backend,'r3',backend)
        run('sap_objects_r2_'+backend,'r2',backend)
    status['complete']=True;checkpoint()


if __name__=='__main__':main()
