"""Source-only Sweep Blocks reconstruction. No simulator outcome is consulted."""
import argparse,json,hashlib,shutil
from pathlib import Path
import numpy as np,cv2,trimesh
from scipy.optimize import differential_evolution,minimize
from scipy.spatial.transform import Rotation as R,Slerp
from scipy.spatial import ConvexHull
from .laptop_prepare import project

TABLE=.765

def fit_source(root):
    root=Path(root);out=root/'geometry';data=np.load(root/'source/episode/numeric.npz');K=data['vision__cam_head__intrinsic_matrix'];T=data['vision__cam_head__extrinsic_matrix'][0]
    im=cv2.imread(str(root/'source/episode/cam_head_0000.jpg'));hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);mask=((hsv[:,:,0]>15)&(hsv[:,:,0]<40)&(hsv[:,:,1]>65)&(hsv[:,:,2]>110)).astype(np.uint8)
    mask[:,350:]=0;mask[320:]=0
    n,labels,stats,_=cv2.connectedComponentsWithStats(mask);components=sorted(range(1,n),key=lambda j:stats[j,cv2.CC_STAT_AREA],reverse=True)[:2]
    found={};overlay=im.copy()
    for name,label in zip(['broom_shovel','broom'],components):
        observed=(labels==label).astype(np.uint8);obsd=cv2.distanceTransform(1-observed,cv2.DIST_L2,3);m=trimesh.load(out/(name+'_0_simplified.obj'),force='mesh');v=np.asarray(m.vertices);faces=np.asarray(m.faces)
        meta=json.loads((root/'source/assets'/(name+'_0')/'metadata.json').read_text());q=np.array(next(iter(meta['active']['place'].values()))['orientation']);base=R.from_quat(q[[1,2,3,0]]);vv=base.apply(v);z=TABLE-vv[:,2].min()+.0001
        def rendering(x):
            rr=R.from_euler('z',x[2])*base;uv=project(rr.apply(v)+[x[0],x[1],z],K,T);pred=np.zeros_like(observed);poly=np.rint(uv[faces]).astype(np.int32);
            for triangle in poly:cv2.fillConvexPoly(pred,triangle,1)
            return pred
        def loss(x):
            pred=rendering(x);inter=np.sum(pred&observed);union=np.sum(pred|observed)
            return 1-inter/max(1,union)
        def uv_to_plane(uv):
            ray=T[:3,:3]@(np.linalg.inv(K)@np.r_[uv,1]*[1,-1,-1]);return (T[:3,3]+ray*((z-T[2,3])/ray[2]))[:2]
        obs_uv=np.array(cv2.moments(observed)['m10':'m01']) if False else np.array([np.nonzero(observed)[1].mean(),np.nonzero(observed)[0].mean()])
        obs_xy=uv_to_plane(obs_uv);candidates=[]
        for yaw in np.linspace(-np.pi,np.pi,73)[:-1]:
            x=np.r_[obs_xy,yaw]
            for _ in range(3):
                pred=rendering(x);yy,xx=np.nonzero(pred)
                if len(xx)==0:break
                x[:2]+=obs_xy-uv_to_plane([xx.mean(),yy.mean()])
            candidates.append((loss(x),x.copy()))
        best=min(candidates,key=lambda a:a[0])
        for _,init in sorted(candidates,key=lambda a:a[0])[:5]:
            ref=minimize(loss,init,method='Powell',options={'xtol':1e-5,'ftol':1e-7,'maxiter':60})
            if ref.fun<best[0]:best=(ref.fun,ref.x)
        x=best[1];pred=rendering(x);rr=R.from_euler('z',x[2])*base
        pe=pred-cv2.erode(pred,np.ones((3,3),np.uint8));oe=observed-cv2.erode(observed,np.ones((3,3),np.uint8));pd=cv2.distanceTransform(1-pe,cv2.DIST_L2,3);od=cv2.distanceTransform(1-oe,cv2.DIST_L2,3);rms=float(np.sqrt((np.mean(pd[oe>0]**2)+np.mean(od[pe>0]**2))/2))
        found[name]={'origin_m':[float(x[0]),float(x[1]),float(z)],'q_wxyz':rr.as_quat()[[3,0,1,2]].tolist(),'silhouette_iou':float(1-loss(x)),'edge_rms_px':rms,'method':'first-head-frame silhouette; source place orientation plus fitted yaw, supported-height assumption','uncertainty':'not calibrated; sensitivity required'}
        overlay[pe>0]=[0,0,255] if name=='broom' else [0,255,0];print(name,found[name],flush=True)
    # Three unique-colour cubes. Fit actual mesh silhouette; metadata size differs.
    for name,lo,hi in [('cube_green',[35,80,80],[85,255,255]),('cube_cyan',[80,80,80],[105,255,255]),('cube_yellow',[15,80,100],[40,255,255])]:
        mask=cv2.inRange(hsv,np.array(lo),np.array(hi));mask[:150]=0;mask[270:]=0;mask[:,:350]=0
        n,lab,stat,cents=cv2.connectedComponentsWithStats(mask);j=max(range(1,n),key=lambda j:stat[j,4]);observed=(lab==j).astype(np.uint8);m=trimesh.load(out/'small_cube_0.obj',force='mesh');v=np.asarray(m.vertices);faces=np.asarray(m.faces);z=TABLE-v[:,2].min()+.0001
        pixel=np.r_[cents[j],1];ray=T[:3,:3]@(np.linalg.inv(K)@pixel*np.array([1,-1,-1]));xy=(T[:3,3]+ray*((z-T[2,3])/ray[2]))[:2]
        def render(x):
            uv=project(R.from_euler('z',x[2]).apply(v)+[x[0],x[1],z],K,T);pred=np.zeros_like(observed);cv2.fillConvexPoly(pred,cv2.convexHull(np.rint(uv).astype(np.int32)),1);return pred
        def loss(x):
            pred=render(x);return 1-np.sum(pred&observed)/max(1,np.sum(pred|observed))
        res=differential_evolution(loss,[(xy[0]-.015,xy[0]+.015),(xy[1]-.015,xy[1]+.015),(-np.pi/4,np.pi/4)],seed=20261118,popsize=8,maxiter=40,polish=False);x=res.x;rr=R.from_euler('z',x[2]);pred=render(x);edge=pred-cv2.erode(pred,np.ones((3,3),np.uint8));overlay[edge>0]=[255,0,255]
        found[name]={'origin_m':[float(x[0]),float(x[1]),float(z)],'q_wxyz':rr.as_quat()[[3,0,1,2]].tolist(),'silhouette_iou':float(1-res.fun),'initial_observed_centroid_px':cents[j].tolist(),'method':'visible unique-colour first-frame cube silhouette, known collision mesh, table support assumption','uncertainty':'not calibrated'};print(name,found[name],flush=True)
    (root/'source_fit.json').write_text(json.dumps(found,indent=2));cv2.imwrite(str(root/'source_fit_overlay.jpg'),overlay)

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('root');v=a.parse_args();fit_source(v.root)
