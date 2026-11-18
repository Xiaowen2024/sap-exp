#!/usr/bin/env python3
"""Audit completed SAP-coupler matrices and emit the five-engine comparison as JSON."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
from compare_genesis_rigid import E,TASKS,count,loss,frozen
from common.paths import find_manifest  # importable once compare_genesis_rigid has run the bootstrap

PRIMARY={'three-plate','three-box','simple-pinch','cube-toss','cube-tail','gauge-slope','mit-push'}

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--sap-coupler',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 regp=E.parent/'protocol/registries/genesis_sap_rerun.json';reg=json.loads(regp.read_text());regsha=hashlib.sha256(regp.read_bytes()).hexdigest()
 pre=json.loads((a.sap_coupler/'preflight.json').read_text());assert pre['all_pass']
 rows=[]
 for tid,kind,sp,mp,gp in TASKS:
  if tid not in PRIMARY:continue
  sapfile=a.sap_coupler/gp.replace('genesis_rigid_','genesis_sap_');d=json.loads(sapfile.read_text())
  spec=next(t for t in reg['tasks'] if t['id']==tid)
  man=json.loads(find_manifest(spec['manifest']).read_text())
  assert d['complete'] is True and d['engine']=='Genesis SAP coupler',(tid,'incomplete')
  assert d['manifest_sha256']==spec['sha256'] and d['registry_sha256']==regsha
  assert len(d['heldout'])==len(man['held_out_conditions']),(tid,'held-out coverage')
  ids=[r['cell']['id'] for r in d['heldout']]
  assert ids==[c['id'] for c in man['held_out_conditions']],(tid,'held-out identities/order')
  row={'task':tid,'metric_kind':kind,'frozen':frozen(d),'sources':{},'values':{}}
  for label,path in [('SAP Warp',E/sp),('MuJoCo Warp 4.0',E/mp),('Genesis Rigid',E/'genesis-rigid-20261114'/gp),('Genesis IPC a3',E/'genesis-ipc-a3-20261116'/gp.replace('genesis_rigid_','genesis_ipc_')),('Genesis SAP',sapfile)]:
   x=json.loads(path.read_text());assert x.get('complete') is not False,(label,tid)
   row['sources'][label]=str(path.relative_to(E.parent));row['values'][label]=count(x) if kind=='count' else loss(x)
  rows.append(row)
 out={'complete':True,'registry_sha256':regsha,'preflight_all_pass':True,'scope':'seven primary IPC a3 contact tasks; identical manifests, R=20, same selection and heldout scoring','rows':rows}
 a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
if __name__=='__main__':main()
