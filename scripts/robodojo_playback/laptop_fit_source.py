"""Fit the source initialization without simulator state or outcomes."""
import argparse,itertools,json
from pathlib import Path
import cv2,numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from .laptop_prepare import project


def main(root):
    root=Path(root);p=np.load(root/'reference/numeric.npz');K=p['vision__cam_head__intrinsic_matrix'];T=p['vision__cam_head__extrinsic_matrix'][450];w=p['state__right_ee_poses'][450];Rw=R.from_quat(w[[4,5,6,3]]);pw=w[:3]
    obj=np.array([[-.08965,-.0782,.009],[.08965,-.0782,.009],[.08965,.0477,.009],[-.08965,.0477,.009]]);uv=np.array([[490,147],[521,240],[594,211],[563,118]],float);best=[]
    for flip,off in itertools.product([0,1],range(4)):
        pix=np.roll(uv[::-1] if flip else uv,off,axis=0).copy();weights=np.roll(np.array([1,1,1,0])[::-1] if flip else [1,1,1,0],off);answer=cv2.solvePnPGeneric(obj,pix,K,None,flags=cv2.SOLVEPNP_IPPE)
        for rv,tv in zip(answer[1],answer[2]):
            rc=cv2.Rodrigues(rv)[0];anchor=R.from_matrix(T[:3,:3]@np.diag([1,-1,-1])@rc);po=T[:3,:3]@np.diag([1,-1,-1])@tv.ravel()+T[:3,3];v0=np.r_[Rw.inv().apply(po-pw),[0.,0.,0.]]
            def residual(v):
                rr=R.from_rotvec(v[3:])*anchor;pos=pw+Rw.apply(v[:3]);error=(project(rr.apply(obj)+pos,K,T)-pix)*weights[:,None];normal=rr.apply([0,0,1]);cross=Rw.inv().apply(normal);point=pos+rr.apply([0,0,.0045]);plane=np.dot(point-(pw+Rw.apply([.14,0.,0.])),normal)
                return np.r_[error.ravel(),plane/.002,cross[[0,2]]/.12]
            f=least_squares(residual,v0,max_nfev=200);rr=R.from_rotvec(f.x[3:])*anchor;po=pw+Rw.apply(f.x[:3]);best.append((float(np.sum(f.fun**2)),f.x,po,rr.as_quat()[[3,0,1,2]],pix,project(rr.apply(obj)+po,K,T)-pix))
    best.sort(key=lambda a:a[0]);a=best[0];result=dict(cost=a[0],gripper_local_origin=a[1][:3].tolist(),origin=a[2].tolist(),q=a[3].tolist(),landmarks=a[4].tolist(),residuals=a[5].tolist(),method='3 visible face corners; occluded corner zero weight; soft parallel-jaw normal and grasp plane priors')
    (root/'geometry/laptop_fit_provisional.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('root');main(a.parse_args().root)
