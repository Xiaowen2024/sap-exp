"""Common source collision decomposition, fixed before any engine run."""
import argparse,json,time
from pathlib import Path
import numpy as np,trimesh,coacd

def main(root):
 root=Path(root);out=root/'geometry';report={};coacd.set_log_level('warn')
 for name,limit in [('broom',32),('broom_shovel',48)]:
  dest=out/(name+'_pieces');dest.mkdir(exist_ok=True)
  if list(dest.glob('*.obj')):raise FileExistsError('do not overwrite geometry')
  mesh=trimesh.load(out/(name+'_0_simplified.obj'),force='mesh');started=time.perf_counter()
  par=dict(threshold=.005,max_convex_hull=limit,preprocess_mode='auto',resolution=3000,mcts_nodes=10,mcts_iterations=60,mcts_max_depth=3,pca=False,merge=True,decimate=True,max_ch_vertex=64,seed=20261118)
  pcs=coacd.run_coacd(coacd.Mesh(np.asarray(mesh.vertices),np.asarray(mesh.faces)),**par)
  for i,(v,f) in enumerate(pcs):
   m=trimesh.Trimesh(v,f,process=False);assert len(m.vertices)<=64;m.export(dest/f'piece_{i:03d}.obj')
  report[name]={'parameters':par,'pieces':len(pcs),'max_vertices':max(len(v) for v,f in pcs),'wall_s':time.perf_counter()-started,'representation':'common rigid convex decomposition of simplified official collision mesh; source cooked PhysX hulls unavailable'}
  print(name,report[name],flush=True);(out/'decomposition.json').write_text(json.dumps(report,indent=2))
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('root');main(a.parse_args().root)
