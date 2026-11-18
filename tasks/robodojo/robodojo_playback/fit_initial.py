"""Fit candidate official bowl assets to the initial image only.

No solver output or final demonstration frame may be used by this step.
This estimates completion, and does not prove an unreleased object identity.
"""
import json
from pathlib import Path
import argparse
import numpy as np
from scipy import ndimage
from scipy.optimize import minimize
from scipy.spatial import ConvexHull, cKDTree
from PIL import Image, ImageDraw
from pxr import Usd, UsdGeom


def fit(root):
    root=Path(root); data=np.load(root/'stack_bowls/numeric.npz',allow_pickle=False)
    K=data['vision__cam_head__intrinsic_matrix']; T=data['vision__cam_head__extrinsic_matrix'][0]
    image=Image.open(root/'stack_bowls/cam_head_0000.jpg').convert('RGB'); rgb=np.asarray(image).astype(int)
    regions={'left':(160,160,260,240),'middle':(300,155,403,245),'right':(400,145,495,230)}
    contours={}; observed=[]
    for name,(x0,y0,x1,y1) in regions.items():
        patch=rgb[y0:y1,x0:x1];mask=(patch.min(2)>135)&(patch.max(2)-patch.min(2)<65)
        mask=ndimage.binary_fill_holes(ndimage.binary_closing(mask,iterations=1))
        labels,n=ndimage.label(mask); sizes=np.bincount(labels.ravel());sizes[0]=0; mask=labels==sizes.argmax()
        edge=mask & ~ndimage.binary_erosion(mask); yy,xx=np.nonzero(edge)
        pts=np.column_stack([xx+x0,yy+y0]).astype(float); contours[name]=pts
    def project(v):
        cam=(v-T[:3,3])@T[:3,:3];cam*=[1,-1,-1];uv=cam@K.T;return uv[:,:2]/uv[:,2:3]
    def boundary(v):
        uv=project(v);h=uv[ConvexHull(uv).vertices];points=[]
        for a,b in zip(h,np.roll(h,-1,axis=0)):
            n=max(2,int(np.linalg.norm(b-a)*1.5));points.extend(a+(b-a)*np.linspace(0,1,n)[:,None])
        return np.asarray(points),h
    reports=[]
    for model in [2,3,5,7,10]:
        folder=root/f'bowl{model}';files=list(folder.glob('*.usd*'))
        if not files:continue
        stage=Usd.Stage.Open(str(files[0])); prim=stage.GetPrimAtPath('/root/collision/model'); mesh=UsdGeom.Mesh(prim)
        v=np.asarray(mesh.GetPointsAttr().Get(),float);xform=np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)).T
        v=v@xform[:3,:3].T+xform[:3,3];z=.765-float(v[:,2].min());v=v[ConvexHull(v).vertices]
        poses={};losses={};predictions={}
        for name,pts in contours.items():
            tree=cKDTree(pts);uv=pts.mean(0);ray=np.array([(uv[0]-K[0,2])/K[0,0],-(uv[1]-K[1,2])/K[1,1],-1.])@T[:3,:3].T
            guess=T[:3,3]+ray*((z-T[2,3])/ray[2])
            def objective(xy):
                b,_=boundary(v+np.r_[xy,z]);d=tree.query(b)[0];inv=cKDTree(b).query(pts)[0]
                return float(np.mean(d*d)+np.mean(inv*inv))
            opt=minimize(objective,guess[:2],method='Nelder-Mead',options={'maxiter':300,'xatol':1e-7,'fatol':1e-6})
            pose=np.r_[opt.x,z]; poses[name]=pose.tolist();losses[name]=float(np.sqrt(opt.fun/2)); predictions[name]=boundary(v+pose)[1]
        canvas=image.copy();draw=ImageDraw.Draw(canvas)
        for name,h in predictions.items():
            poly=[tuple(p) for p in h];draw.line(poly+[poly[0]],fill=(255,80,30),width=2)
        canvas.save(root/f'initial_fit_bowl{model}.jpg')
        reports.append({'model':model,'positions_m':poses,'silhouette_rms_px':losses,'mean_rms_px':float(np.mean(list(losses.values()))),
                        'source':'initial head frame; fixed table z=.765, upright pose and yaw=0'})
    reports.sort(key=lambda r:r['mean_rms_px'])
    report={'kind':'initial_scene_completion_not_solver_selection','candidates':reports,
            'identity_status':'inferred from shape only; not a released episode object-ID field',
            'coordinate_uncertainty':'not yet quantified; image segmentation and inferred asset identity remain uncertainty'}
    (root/'initial_fit.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('root');fit(ap.parse_args().root)
