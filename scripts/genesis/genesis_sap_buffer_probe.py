"""Prove a capacity-only constructor replacement retains the original contact solution."""
import argparse,os,numpy as np
import genesis_sap_common as G

def sim_probe(repeat=0):
 c=G.parse_candidate('hydroelastic_stiffness=1e8,sap_taud=0.01',{'nonnative':{'n_sap_iterations':20}})
 box=lambda i:G.mjcf(f'<body pos="0 0 {.02+.04*i}"><freejoint/><geom type="box" size=".02 .02 .02" mass="1" friction=".3 0 0"/></body>')
 b=G.build({'ground':G.mjcf('<geom type="plane" size="2 2 .1" friction=".3 0 0"/>'),**{f'b{i}':box(i) for i in range(3)}},dt=.001,cand=c,n_envs=2)
 qp=[];loads=[]
 for _ in range(30):
  b.scene.step();qp.append(np.stack([G.np_(b.ents[f'b{i}'].get_qpos()) for i in range(3)]))
  con=b.contacts();loads.append(np.stack([b.pair_normal_load(x,y,con)[0] for x,y in [('ground','b0'),('b0','b1'),('b1','b2')]]))
 out={'qpos':np.asarray(qp),'loads':np.asarray(loads),'compiled':b.provenance()};b.destroy();return out

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=G.Path,required=True);a=ap.parse_args();results=[]
 for index,cap in enumerate((None,None,'8192')):
  if cap:os.environ['SAP_BUFFER_CAPACITY']=cap;os.environ['SAP_QUERY_FACTOR']='2'
  else:os.environ.pop('SAP_BUFFER_CAPACITY',None);os.environ.pop('SAP_QUERY_FACTOR',None)
  raw,why=G.ISO.call('genesis_sap_buffer_probe','sim_probe',tag='buffer equivalence '+str(index),repeat=index)
  if raw is None:
   G.save_json(a.out,{'all_pass':False,'failure':why});raise RuntimeError(why)
  results.append(raw);print('probe completed capacity',cap,flush=True)
 repeat_diff={k:float(np.sqrt(np.mean((results[0][k]-results[1][k])**2))) for k in ('qpos','loads')}
 capacity_diff={k:float(np.sqrt(np.mean((results[0][k]-results[2][k])**2))) for k in ('qpos','loads')}
 checks={k:capacity_diff[k]<=max(repeat_diff[k]*3,1e-8 if k=='qpos' else .01) for k in ('qpos','loads')}
 checks['same_tetrahedra']=len({r['compiled']['tetrahedra_sha256'] for r in results})==1
 checks['capacity_changed']=results[2]['compiled']['contact_capacities'][0]['max_pairs']!=results[0]['compiled']['contact_capacities'][0]['max_pairs']
 out={'kind':'buffer_capacity_equivalence_not_benchmark','all_pass':all(checks.values()),'checks':checks,'native':results[0]['compiled'],'bounded':results[2]['compiled'],'native_repeat_rms_difference':repeat_diff,'capacity_change_rms_difference':capacity_diff,'max_qpos_difference':float(np.abs(results[0]['qpos']-results[2]['qpos']).max()),'max_load_difference_N':float(np.abs(results[0]['loads']-results[2]['loads']).max())}
 G.save_json(a.out,out);print('ALL PASS' if out['all_pass'] else 'FAIL',checks,flush=True)
 if not out['all_pass']:raise SystemExit(1)
if __name__=='__main__':main()
