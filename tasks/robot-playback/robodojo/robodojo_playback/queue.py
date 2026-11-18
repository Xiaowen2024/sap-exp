"""Serial GPU-host pilot queue, with a mandatory no-object drive gate."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--stage',choices=['drive','objects'],required=True);ap.add_argument('--revision',default='r2')
    a=ap.parse_args();root=Path.cwd();report={};revision=a.revision
    if a.stage=='objects':
        for engine in ('mujoco','sap','ipc'):
            h=json.loads((root/'runs'/f'{engine}_drive_{revision}'/'health.json').read_text())
            if not h['drive_gate_pass'] or not h['finite'] or h['steps']!=h['expected_steps']:
                raise RuntimeError(f'{engine} no-object drive gate failed')
    for engine in ('mujoco','sap','ipc'):
        name=f'{engine}_{a.stage}_{revision}';out=root/'runs'/name
        py='/root/autodl-tmp/genesis-runtime/bin/python-genesis' if engine=='ipc' else '/root/autodl-tmp/venvs/mjwarp400-cu12/bin/python'
        env=dict(os.environ,PYTHONPATH='/root/sap-fix-gamma/src:'+str(root),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
        command=[py,'-u','-m','scripts.robodojo_playback.run','--scene',str(root/'prepared'),'--engine',engine,'--out',str(out)]
        if a.stage=='drive':command.append('--drive-only')
        if engine=='sap' and a.stage=='objects':
            validation=root/'graph_validation.json'
            if validation.exists() and json.loads(validation.read_text()).get('pass'):command.append('--sap-graph')
        with (root/(name+'.log')).open('w') as log:
            status=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,env=env).returncode
        report[engine]={'exit_code':status,'out':str(out)}
        (root/f'queue_{a.stage}_{revision}.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)


if __name__=='__main__':main()
