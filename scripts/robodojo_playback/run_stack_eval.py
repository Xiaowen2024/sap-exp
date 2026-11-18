"""Run the frozen Stack Bowls scene with read-only native-step telemetry."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',required=True);ap.add_argument('--engine',choices=['sap','mujoco','both'],default='both');a=ap.parse_args()
    root=Path(a.root).resolve();status={}
    for engine in (['sap','mujoco'] if a.engine=='both' else [a.engine]):
        name=engine+'_native_1ms'
        env=dict(os.environ,PYTHONPATH=str(root/'sap/src')+':'+str(root),SAP_WARP_CONVEX_BACKEND='newton',
            ROBODOJO_STACK_EVAL='1',ROBODOJO_SAP_EXECUTION='graph',WARP_CACHE_PATH=str(root/'warp_cache'),
            ROBODOJO_WARP_CACHE=str(root/'warp_cache'),TMPDIR=str(root/'tmp'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        cmd=[sys.executable,'-u','-m','scripts.robodojo_playback.run','--scene',str(root/'scene'),'--engine',engine,'--out',str(root/'runs'/name)]
        status[engine]={'state':'running'};(root/'run_status.json').write_text(json.dumps(status,indent=2));print('START',engine,flush=True)
        with (root/(name+'.log')).open('w') as log:result=subprocess.run(cmd,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
        status[engine]={'state':'finished','exit_code':result.returncode};(root/'run_status.json').write_text(json.dumps(status,indent=2));print('FINISH',engine,result.returncode,flush=True)
        if result.returncode:raise RuntimeError(engine+' telemetry run failed')


if __name__=='__main__':main()
