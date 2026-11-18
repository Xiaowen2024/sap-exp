"""One full rollout per requested candidate, serially on one GPU. No repeats."""
import argparse,json,os,subprocess,time,hashlib
from pathlib import Path


def main(root,python,engines,iteration):
    root=Path(root).resolve();runtime=Path('/root/autodl-tmp/robodojo-warp-benchmark-v1');env=os.environ.copy()
    env.update(PYTHONPATH=f'{runtime}/deps:{runtime}/sap/src:{root}',TMPDIR=str(root/'tmp'),XDG_CACHE_HOME=str(root/'cache'),PYTHONDONTWRITEBYTECODE='1',
        SAP_WARP_CONVEX_BACKEND='newton',ROBODOJO_STACK_EVAL='1',ROBODOJO_SINGLE_ROLLOUT='1',ROBODOJO_BENCHMARK_REPEATS='0',ROBODOJO_SAP_EXECUTION='graph',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    (root/'tmp').mkdir(exist_ok=True);(root/'runs').mkdir(exist_ok=True)
    status_path=root/'run_status.json';status=json.loads(status_path.read_text()) if status_path.exists() else {'jobs':[]}
    for engine in engines:
        name=f'{engine}_it{iteration}';out=root/'runs'/name
        if out.exists():raise FileExistsError('Never overwrite or silently repeat a candidate: '+str(out))
        log=root/'runs'/(name+'.log');params=root/'parameters'/f'{engine}_{iteration:02d}.json'
        if not params.exists():raise FileNotFoundError('Write candidate parameters and reason before launching')
        env['ROBODOJO_WARP_CACHE']=str(root/'cache'/engine)
        cmd=[python,'-u','-m','scripts.robodojo_playback.run','--scene',str(root/'scene_v1'),'--out',str(out),'--engine',engine,'--params',str(params)]
        gpu=subprocess.run(['nvidia-smi','--query-gpu=name,utilization.gpu,memory.used','--format=csv,noheader'],capture_output=True,text=True).stdout.strip()
        row=dict(name=name,engine=engine,iteration=iteration,status='running',cmd=cmd,parameters=json.loads(params.read_text()),parameters_sha256=hashlib.sha256(params.read_bytes()).hexdigest(),gpu_at_start=gpu,start_unix=time.time(),rollouts_requested=1)
        status['jobs'].append(row);status_path.write_text(json.dumps(status,indent=2));started=time.perf_counter()
        with log.open('w') as f:
            p=subprocess.Popen(cmd,env=env,cwd=root,stdout=f,stderr=subprocess.STDOUT);row['pid']=p.pid;status_path.write_text(json.dumps(status,indent=2));rc=p.wait()
        row.update(exit_code=rc,process_wall_s=time.perf_counter()-started,status='finished' if rc==0 else 'adapter_or_runtime_error')
        if (out/'health.json').exists():row['health']=json.loads((out/'health.json').read_text())
        status_path.write_text(json.dumps(status,indent=2));print(name,row['status'],row['process_wall_s'],flush=True)

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--root',required=True);a.add_argument('--python',required=True);a.add_argument('--engines',nargs='+',default=['mujoco_warp','sap']);a.add_argument('--iteration',type=int,default=1);v=a.parse_args();main(v.root,v.python,v.engines,v.iteration)
