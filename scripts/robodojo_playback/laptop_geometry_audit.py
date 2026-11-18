"""Metric-space audit of original and convex rack slot clearances, no dynamics."""
from pathlib import Path
import argparse,json
import numpy as np,trimesh
from scipy.spatial import ConvexHull


def main(root):
    r=Path(root);m=trimesh.load(r/'geometry/rack_reference.obj',force='mesh');tri=np.asarray(m.triangles);xz=tri[:,:,[0,2]];a=xz[:,0];u=xz[:,1]-a;v=xz[:,2]-a;det=u[:,0]*v[:,1]-u[:,1]*v[:,0];valid=abs(det)>1e-14
    eq=[ConvexHull(trimesh.load(p,force='mesh').vertices).equations for p in sorted((r/'geometry/rack_pieces').glob('*.obj'))];rows=[]
    for z in [-.018,-.005,.01,.022]:
        for x in [-.045,0,.045]:
            d=np.array([x,z])-a;s=np.zeros(len(a));t=s.copy();s[valid]=(d[valid,0]*v[valid,1]-d[valid,1]*v[valid,0])/det[valid];t[valid]=(u[valid,0]*d[valid,1]-u[valid,1]*d[valid,0])/det[valid];hit=valid&(s>=-1e-8)&(t>=-1e-8)&(s+t<=1+1e-8)
            ys=tri[:,0,1]+s*(tri[:,1,1]-tri[:,0,1])+t*(tri[:,2,1]-tri[:,0,1]);ys=ys[hit];orig=[float(max(ys[ys<0])),float(min(ys[ys>0]))];intervals=[]
            for e in eq:
                aa=e[:,1];bb=-(e[:,0]*x+e[:,2]*z+e[:,3]);lo=-np.inf;hi=np.inf;ok=True
                for A,B in zip(aa,bb):
                    if abs(A)<1e-10:
                        if B<0:ok=False;break
                    elif A>0:hi=min(hi,B/A)
                    else:lo=max(lo,B/A)
                if ok and lo<hi:intervals.append((lo,hi))
            if any(a<=0<=b for a,b in intervals):raise RuntimeError('decomposition closes slot')
            approx=[float(max(b for a,b in intervals if b<0)),float(min(a for a,b in intervals if a>0))];err=max(abs(np.array(approx)-orig));rows.append(dict(x_m=x,z_m=z,original_boundaries_m=orig,convex_boundaries_m=approx,boundary_error_m=float(err),original_gap_m=orig[1]-orig[0],convex_gap_m=approx[1]-approx[0]))
    report=dict(method='triangle/axis-line crossings versus convex half-space interval union; 12 source-only lines',rows=rows,max_sampled_boundary_error_m=max(a['boundary_error_m'] for a in rows),coverage='sampled slot boundaries, not a global Hausdorff guarantee')
    (r/'geometry/rack_slot_audit.json').write_text(json.dumps(report,indent=2));print('max sampled boundary error mm',report['max_sampled_boundary_error_m']*1e3)
    if report['max_sampled_boundary_error_m']>.0002:raise RuntimeError('slot boundary error above 0.2 mm admission criterion')

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('root');main(a.parse_args().root)
