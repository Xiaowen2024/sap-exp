"""Compare common pan top surfaces to source collision mesh, no dynamics."""
from pathlib import Path
import argparse,json
import trimesh,numpy as np
from scipy.spatial import ConvexHull

def main(root,folder):
 r=Path(root);ref=trimesh.load(r/'geometry/broom_shovel_0.obj',force='mesh');pcs=[trimesh.load(p,force='mesh') for p in sorted((r/'geometry'/folder).glob('*.obj'))]
 xx,yy=np.meshgrid(np.linspace(-.075,.075,31),np.linspace(-.132,-.025,31));orig=np.c_[xx.ravel(),yy.ravel(),np.full(xx.size,.1)];direction=np.tile([0,0,-1.],(len(orig),1));loc,idx,_=ref.ray.intersects_location(orig,direction,multiple_hits=True);rt=np.full(len(orig),np.nan)
 for p,i in zip(loc,idx):rt[i]=p[2] if not np.isfinite(rt[i]) else max(rt[i],p[2])
 pt=np.full(len(orig),-np.inf)
 for m in pcs:
  e=ConvexHull(m.vertices).equations;norm=e[:,:3];d=e[:,3];upper=norm[:,2]>1e-9;lower=norm[:,2]<-1e-9;vertical=abs(norm[:,2])<=1e-9
  zhi=np.min(-(orig[:,:2]@norm[upper,:2].T+d[upper])/norm[upper,2],axis=1);zlo=np.max(-(orig[:,:2]@norm[lower,:2].T+d[lower])/norm[lower,2],axis=1)
  okay=(zhi>=zlo)&(np.all(orig[:,:2]@norm[vertical,:2].T+d[vertical]<=1e-8,axis=1));pt[okay]=np.maximum(pt[okay],zhi[okay])
 valid=np.isfinite(rt)&np.isfinite(pt);err=pt[valid]-rt[valid];report={'method':'vertical top-surface rays, local tray rectangle x[-.075,.075] y[-.132,-.025],31x31','folder':folder,'valid_rays':int(sum(valid)),'max_abs_surface_error_mm':float(np.max(abs(err))*1000),'max_protrusion_mm':float(err.max()*1000),'p95_abs_mm':float(np.percentile(abs(err),95)*1000),'reference_top_range_m':[float(np.nanmin(rt)),float(np.nanmax(rt))],'worst_xy':orig[valid][abs(err).argmax(),:2].tolist()};print(report);(r/'geometry'/('cavity_audit_'+folder+'.json')).write_text(json.dumps(report,indent=2))
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('root');a.add_argument('folder');v=a.parse_args();main(v.root,v.folder)
