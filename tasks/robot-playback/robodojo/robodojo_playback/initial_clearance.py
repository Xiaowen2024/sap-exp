"""Check strict initial convex-piece intersections independently of engines."""
import argparse
import json
from pathlib import Path
import numpy as np
import trimesh
from scipy.optimize import linprog
from scipy.spatial import ConvexHull
from .common import load,rotation


def check(root):
    root=Path(root);scene,_=load(root);bodies={};floor={}
    for b in scene['bodies']:
        rot=rotation(b['q']).as_matrix();p=np.asarray(b['p']);parts=[]
        for f in scene['assets'][b['asset']]['pieces']:
            mesh=trimesh.load(f,force='mesh',process=False);v=np.asarray(mesh.vertices)@rot.T+p;e=ConvexHull(v).equations
            parts.append((v.min(0),v.max(0),e))
        bodies[b['name']]=parts;floor[b['name']]=min(a[0][2] for a in parts)-scene['table_top_z']
    rows=[];names=list(bodies)
    for i,a in enumerate(names):
        for b in names[i+1:]:
            if a.split('_')[0]==b.split('_')[0] and a.startswith(('left_','right_')):continue
            maximum=0.;tested=0
            for alo,ahi,ae in bodies[a]:
                for blo,bhi,be in bodies[b]:
                    if np.any(ahi<=blo) or np.any(bhi<=alo):continue
                    e=np.vstack([ae,be]);result=linprog([0.,0.,0.,-1.],A_ub=np.c_[e[:,:3],np.ones(len(e))],b_ub=-e[:,3],bounds=[(None,None)]*4,method='highs')
                    if not result.success:raise RuntimeError('initial intersection LP failed')
                    maximum=max(maximum,float(result.x[3]));tested+=1
            rows.append({'pair':a+'/'+b,'max_intersection_insphere_radius_m':maximum,'AABB_overlapping_pairs_tested':tested})
    report={'kind':'initial_convex_geometry_admission','pairs':rows,'table_clearance_m':floor,
            'pass':all(r['max_intersection_insphere_radius_m']<=1e-7 for r in rows) and min(floor.values())>=-1e-7,
            'criterion':'every cross-body convex intersection has no positive-volume insphere above 0.1 micrometre; no floor crossing'}
    (root/'initial_clearance.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('root');check(ap.parse_args().root)
