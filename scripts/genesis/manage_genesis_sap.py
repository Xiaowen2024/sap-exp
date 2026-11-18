#!/usr/bin/env python3
"""Resume-aware bounded task queue; wait for an adopted SAP child before starting duplicate work."""
import argparse,json,os,subprocess,time
from pathlib import Path

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--adopt-pid',type=int);ap.add_argument('--adopt-task',default='three-box');ap.add_argument('--lanes',type=int,default=3);ap.add_argument('--adopt',action='append',default=[],help='task=pid, repeatable');ap.add_argument('--skip',default='',help='comma list of tasks run elsewhere');ap.add_argument('--only',default='',help='comma list; run only these tasks');ap.add_argument('--preflight-exception',default='',help='disclosed reason to run despite a failed preflight check');a=ap.parse_args()
 root=Path(__file__).resolve().parents[2];m=root/'sim_protocol/manifests';py='/root/autodl-tmp/genesis-runtime/bin/python-genesis'
 tasks=[('three-box','run_threebox_genesis_sap.py','three_box_push_strict_v4.json','genesis_sap_three_box_strict_v4.json',[]),
 ('simple-pinch','run_pinch_genesis_sap.py','simple_pinch_force_controlled_v3.json','genesis_sap_simple_pinch_v3.json',[]),
 ('three-plate','run_threeplate_genesis_sap.py','three_plate_stack_strict_v3p1.json','genesis_sap_three_plate_strict_v3.json',[]),
 ('cube-toss','run_impact_genesis_sap.py','contactnets_cube_toss_dynamic_impact_v2.json','genesis_sap_cube_dynamic_impact_v2.json',['--task','cube']),
 ('gauge-slope','run_slope_genesis_sap.py','gauge_slope_slider_dynamic_friction_v3.json','genesis_sap_gauge_slope_v3.json',[]),
 ('mit-push','run_mitpush_genesis_sap.py','mit_push_dynamic_friction_v3.json','genesis_sap_mit_push_v3.json',[]),
 ('cube-tail','run_tail_genesis_sap.py','contactnets_cube_tail_sustained_v3.json','genesis_sap_cube_tail_v3.json',['--cube-toss-artifact',str(a.out/'cube-toss/genesis_sap_cube_dynamic_impact_v2.json')])]
 pf=json.loads((a.out/'preflight.json').read_text())
 if not pf['all_pass']:
  assert a.preflight_exception,'preflight failed; pass --preflight-exception with the disclosed reason'
  bad=[c['name'] for c in pf['checks'] if not c['pass']]
  with (a.out/'status.txt').open('a') as f:f.write(time.strftime('%Y-%m-%dT%H:%M:%S%z')+' PREFLIGHT EXCEPTION failed='+repr(bad)+' reason='+a.preflight_exception+'\n')
 def done(t):
  p=a.out/t[0]/t[3]
  return p.exists() and json.loads(p.read_text()).get('complete') is True
 def alive(pid):
  try:
   os.kill(pid,0)
   return Path(f'/proc/{pid}/stat').read_text().split()[2]!='Z'
  except ProcessLookupError:return False
 def log(s):
  with (a.out/'status.txt').open('a') as f:f.write(time.strftime('%Y-%m-%dT%H:%M:%S%z')+' '+s+'\n')
  print(s,flush=True)
 running={};failed={};pending=[t for t in tasks if not done(t)]
 skip=set(filter(None,a.skip.split(',')));only=set(filter(None,a.only.split(',')))
 pending=[t for t in pending if t[0] not in skip and (not only or t[0] in only)]
 if skip or only:log('task filter skip='+','.join(sorted(skip))+' only='+','.join(sorted(only)))
 adopts=[(a.adopt_task,a.adopt_pid)] if a.adopt_pid else []
 adopts+=[(x.split('=')[0],int(x.split('=')[1])) for x in a.adopt]
 for at,ap_ in adopts:
  if alive(ap_):
   running[at]=(ap_,None,None);pending=[t for t in pending if t[0]!=at];log('adopt '+at+' pid='+str(ap_))
 while pending or running:
  for name,(pid,proc,fh) in list(running.items()):
   if (proc.poll() is not None) if proc is not None else (not alive(pid)):
    t=next(t for t in tasks if t[0]==name)
    if fh:fh.close()
    if done(t):log(name+' complete')
    else:failed[name]={'exit_code':proc.returncode if proc else None};log(name+' FAILED (remaining independent tasks continue)')
    del running[name]
  if pending and len(running)<a.lanes:
   free=int(subprocess.check_output(['nvidia-smi','--query-gpu=memory.free','--format=csv,noheader,nounits'],text=True).strip().splitlines()[0])
   if free>=3000:
    for t in pending:
     name,script,mf,result,extra=t
     if name=='cube-tail' and not done(next(x for x in tasks if x[0]=='cube-toss')):continue
     fh=(a.out/(name+'.log')).open('w')
     proc=subprocess.Popen([py,'-u',str(root/'scripts/genesis'/script),'--manifest',str(m/mf),'--out',str(a.out/name),*extra],stdout=fh,stderr=subprocess.STDOUT,cwd=root/'scripts/genesis',stdin=subprocess.DEVNULL)
     running[name]=(proc.pid,proc,fh);pending.remove(t);log('start '+name+' pid='+str(proc.pid)+' gpu_free_MiB='+str(free));break
  if not running and pending and all(t[0]=='cube-tail' for t in pending) and 'cube-toss' in failed:
   failed['cube-tail']={'blocked_by':'cube-toss'};pending=[]
  state={'complete':not pending and not running and not failed,'running':{k:v[0] for k,v in running.items()},'pending':[t[0] for t in pending],'failed':failed}
  p=a.out/'manager-state.json';tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2));tmp.replace(p)
  if pending or running:time.sleep(30)
 log('ALLDONE' if not failed else 'TASK FAILURES REQUIRE REPAIR')
 if failed:raise SystemExit(1)
if __name__=='__main__':main()
