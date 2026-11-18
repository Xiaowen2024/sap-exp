"""Source-only pan regions preserve the tray floor and thin entrance."""
from pathlib import Path
import argparse,json
import trimesh,numpy as np

def crop(mesh,lo,hi):
 m=mesh
 for ax in range(3):
  if np.isfinite(lo[ax]):
   p=np.zeros(3);p[ax]=lo[ax];normal=np.zeros(3);normal[ax]=1;m=m.slice_plane(p,normal)
  if np.isfinite(hi[ax]):
   p=np.zeros(3);p[ax]=hi[ax];normal=np.zeros(3);normal[ax]=-1;m=m.slice_plane(p,normal)
 return m

def hull64(v):
 h=trimesh.convex.convex_hull(v)
 if len(h.vertices)<=64:return h
 v=np.asarray(h.vertices);ids=list(np.unique(np.r_[v.argmin(0),v.argmax(0)]));dist=np.min(np.sum((v[:,None]-v[ids][None])**2,axis=2),axis=1)
 while len(ids)<64:
  j=int(np.argmax(dist));ids.append(j);dist=np.minimum(dist,np.sum((v-v[j])**2,axis=1))
 return trimesh.convex.convex_hull(v[ids])

def main(root):
 root=Path(root);m=trimesh.load(root/'geometry/broom_shovel_0.obj',force='mesh');out=root/'geometry/broom_shovel_regions_v2';out.mkdir(exist_ok=True);regions=[];cut=-.021
 regions.append(('floor',[-np.inf,-np.inf,-np.inf],[np.inf,np.inf,cut]))
 for j,(a,b) in enumerate(zip(np.linspace(-.075,.075,7)[:-1],np.linspace(-.075,.075,7)[1:])):
  for k,(c,d) in enumerate(zip(np.linspace(-.132,.012,9)[:-1],np.linspace(-.132,.012,9)[1:])):regions.append((f'tray_{j}_{k}',[a,c,cut],[b,d,np.inf]))
 for side in [-1,1]:
  xmin,xmax=(-np.inf,-.075) if side<0 else (.075,np.inf)
  for j,(a,b) in enumerate(zip(np.linspace(-.14,.032,10)[:-1],np.linspace(-.14,.032,10)[1:])):regions.append((f'wall_{side}_{j}',[xmin,a,cut],[xmax,b,np.inf]))
 for j,(a,b) in enumerate(zip(np.linspace(-.075,.075,7)[:-1],np.linspace(-.075,.075,7)[1:])):regions.append((f'lip_{j}',[a,-np.inf,cut],[b,-.132,np.inf]))
 for j,(a,b) in enumerate(zip(np.linspace(-.075,.075,7)[:-1],np.linspace(-.075,.075,7)[1:])):regions.append((f'back_{j}',[a,.012,cut],[b,.032,np.inf]))
 # Handle ring: each small rectangular patch cannot fill its through-hole.
 for j,(a,b) in enumerate(zip(np.linspace(-.016,.016,5)[:-1],np.linspace(-.016,.016,5)[1:])):
  for k,(c,d) in enumerate(zip(np.linspace(.032,.141,10)[:-1],np.linspace(.032,.141,10)[1:])):regions.append((f'handle_{j}_{k}',[a,c,cut],[b,d,np.inf]))
 report=[]
 for name,lo,hi in regions:
  sub=crop(m,lo,hi)
  if len(sub.vertices)<4:continue
  try:h=hull64(sub.vertices)
  except Exception:continue
  if h.volume<1e-11:continue
  h.export(out/f'piece_{len(report):03d}.obj');report.append({'region':name,'vertices':len(h.vertices),'bounds_m':h.bounds.tolist(),'volume_m3':float(h.volume)})
 (root/'geometry/pan_regions_v2.json').write_text(json.dumps({'method':'source mesh clipped into low floor, separate side-wall/lip/back strips and handle patches; farthest-point convex approximation <=64 vertices','floor_partition_z_m':cut,'pieces':report},indent=2));print('pan pieces',len(report),flush=True)
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('root');main(a.parse_args().root)
