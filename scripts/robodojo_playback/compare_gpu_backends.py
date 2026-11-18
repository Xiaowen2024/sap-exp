"""Serial, single-GPU correctness gates and warmed playback timing.

No tuning. The same frozen scene/targets are used for both native backends.
Resource samples identify competing compute processes; none are stopped.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import threading
import time


def gpu_snapshot():
    def call(args):
        p=subprocess.run(['nvidia-smi',*args],capture_output=True,text=True,timeout=15)
        if p.returncode:raise RuntimeError(p.stderr)
        return p.stdout.strip()
    return {'unix_s':time.time(),'gpu':call(['--query-gpu=name,driver_version,memory.used,memory.total,utilization.gpu,power.draw,temperature.gpu,clocks.sm','--format=csv,noheader']),
        'processes':call(['--query-compute-apps=pid,process_name,used_memory','--format=csv,noheader'])}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',required=True);ap.add_argument('--python',required=True);ap.add_argument('--repeats',type=int,default=3);a=ap.parse_args()
    root=Path(a.root).resolve();(root/'tmp').mkdir(exist_ok=True);allowed=set();report={'mode':'same GPU, serial, one world, unchanged frozen r3','jobs':{},'resource_samples':[]}
    stop=threading.Event()
    def monitor():
        while not stop.is_set():
            try:report['resource_samples'].append(gpu_snapshot())
            except Exception as e:report['resource_samples'].append({'error':str(e),'unix_s':time.time()})
            stop.wait(2.)
    thread=threading.Thread(target=monitor,daemon=True);thread.start()
    def checkpoint():(root/'comparison_status.json').write_text(json.dumps(report,indent=2))
    def job(engine,label,drive=False,steps=None,repeats=1):
        name=engine+'_'+label;out=root/'runs'/name
        env=dict(os.environ,PYTHONPATH=str(root/'deps')+':'+str(root/'sap/src')+':'+str(root),
            WARP_CACHE_PATH=str(root/'cache'/engine),ROBODOJO_WARP_CACHE=str(root/'cache'/engine),
            ROBODOJO_SAP_EXECUTION='graph',ROBODOJO_BENCHMARK_REPEATS=str(repeats),ROBODOJO_STACK_EVAL='0',
            SAP_WARP_CONVEX_BACKEND='newton',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',TMPDIR=str(root/'tmp'),PYTHONDONTWRITEBYTECODE='1')
        cmd=[a.python,'-u','-m','scripts.robodojo_playback.run','--scene',str(root/'scene'),'--engine',engine,'--out',str(out)]
        if drive:cmd.append('--drive-only')
        if steps is not None:cmd+=['--max-steps',str(steps)]
        report['jobs'][name]={'status':'running','out':str(out)};checkpoint();start=time.perf_counter();print('START',name,flush=True)
        with (root/(name+'.log')).open('w') as log:
            p=subprocess.Popen(cmd,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT);allowed.add(p.pid);exit_code=p.wait()
        elapsed=time.perf_counter()-start;row=report['jobs'][name];row.update(status='completed' if exit_code==0 else 'failed',exit_code=exit_code,process_total_wall_s=elapsed);checkpoint()
        if exit_code:raise RuntimeError(name+' failed; inspect log')
        h=json.loads((out/'health.json').read_text());row['health']={k:h.get(k) for k in ('engine','finite','steps','expected_steps','drive_gate_pass','truncated','wall_s')}
        row['benchmark']=h.get('benchmark');checkpoint();print('FINISH',name,row['health'],flush=True)
        if not h['finite'] or h['steps']!=h['expected_steps'] or h.get('truncated'):raise RuntimeError(name+' invalid native run')
        if drive and not h['drive_gate_pass']:raise RuntimeError(name+' drive gate failed')
    try:
        for engine in ('mujoco_warp','sap'):
            job(engine,'drive_smoke',drive=True,steps=100)
            job(engine,'objects_smoke',steps=100)
            job(engine,'drive_gate',drive=True)
        for engine in ('mujoco_warp','sap'):job(engine,'performance',repeats=a.repeats)
        report['complete']=True
    finally:
        stop.set();thread.join(timeout=16.)
        competitors=[]
        for sample in report['resource_samples']:
            for line in sample.get('processes','').splitlines():
                first=line.split(',')[0].strip()
                if first.isdigit() and int(first) not in allowed:competitors.append({'unix_s':sample['unix_s'],'process':line})
        report['competing_compute_processes']=competitors;report['timing_environment_uncontended']=not competitors
        report['timing_scope']='full 10560-step trajectory; first full rollout discarded, then measured repeats; reset/setup/JIT/capture/readback/I/O/render excluded'
        report['startup_scope']='setup and initial kernel warm/capture timings recorded separately; validation jobs seed caches, so not a pure cold-cache comparison'
        report['input_sha256']={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [root/'scene/scene.json',root/'scene/targets.npz']}
        checkpoint()


if __name__=='__main__':main()
