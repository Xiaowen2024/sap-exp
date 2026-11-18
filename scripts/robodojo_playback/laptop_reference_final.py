"""Estimated final pose from visible corners, without nesting/physics assumptions.

Source-only multi-frame head view and final right-wrist view. Uncertainty is
pixel/model sensitivity, not calibrated statistical ground-truth uncertainty.
"""
import argparse,json
from pathlib import Path
import numpy as np,cv2
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
from PIL import Image,ImageDraw
from .laptop_prepare import project


def main(root):
    root=Path(root);num=np.load(root/'reference/numeric.npz');scene=json.loads((root/'scene_v1/scene.json').read_text());initial=json.loads((root/'scene_fit.json').read_text())['laptop'];rw0=R.from_quat(num['state__right_ee_poses'][450,[4,5,6,3]]);rw=R.from_quat(num['state__right_ee_poses'][514,[4,5,6,3]]);delta=rw*rw0.inv();anchor=delta*R.from_quat(np.array(initial['q'])[[1,2,3,0]]);guess=num['state__right_ee_poses'][514,:3]+delta.apply(np.asarray(initial['origin'])-num['state__right_ee_poses'][450,:3])
    corners=np.array([[-.08965,-.0782,.009],[.08965,-.0782,.009],[.08965,.0477,.009],[-.08965,.0477,.009]]);observations=[];cap=cv2.VideoCapture(str(root/'reference/source_head.mp4'));seeds=np.array([[485,154],[528,227],[453,192]],float)
    for frame in [538,544,549]:
        cap.set(cv2.CAP_PROP_POS_FRAMES,frame);ok,b=cap.read()
        if not ok:raise ValueError('missing source frame')
        im=cv2.cvtColor(b,cv2.COLOR_BGR2RGB);mask=((im.min(2)>165)&((im.max(2).astype(float)-im.min(2))<45)).astype(np.uint8)*255;roi=np.zeros(mask.shape,np.uint8);roi[140:260,435:537]=1;mask*=roi;contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);contour=max(contours,key=cv2.contourArea);poly=cv2.approxPolyDP(contour,1.5,True).reshape(-1,2);points=np.array([poly[np.argmin(np.linalg.norm(poly-s,axis=1))] for s in seeds]);observations.append(dict(camera='cam_head',frame=frame,ids=[0,1,3],points=points))
        Image.fromarray(im).save(root/'reference'/f'final_head_{frame:04d}.png')
    cap.release();observations.append(dict(camera='cam_right_wrist',frame=549,ids=[0,1],points=np.array([[359.,133.],[470.,255.]])))
    def fit(obs,vertices,initial_v=None):
        def residual(v):
            rr=R.from_rotvec(v[3:])*anchor;position=v[:3];result=[]
            for o in obs:
                T=num['vision__'+o['camera']+'__extrinsic_matrix'][o['frame']];K=num['vision__'+o['camera']+'__intrinsic_matrix'];pred=project(rr.apply(vertices[o['ids']])+position,K,T);result.extend((pred-o['points']).ravel())
            return result
        f=least_squares(residual,np.r_[guess,[0.,0.,0.]] if initial_v is None else initial_v,max_nfev=100);return f,R.from_rotvec(f.x[3:])*anchor
    f,rr=fit(observations,corners);com=np.asarray(scene['assets']['laptop']['com']);pc=f.x[:3]+rr.apply(com);error=np.asarray(f.fun).reshape(-1,2);pixel_rms=float(np.sqrt(np.mean(np.sum(error**2,axis=1))))
    if pixel_rms>2.:raise ValueError('source-only final fit residual exceeds 2 px: '+str(pixel_rms))
    rng=np.random.default_rng(20261118);centres=[];angles=[]
    for _ in range(128):
        # Shared head-corner perturbations avoid counting identical stills as
        # independent camera measurements. Per-corner model offsets are shared.
        head=rng.normal(0,1.5,(3,2));wrist=rng.normal(0,1.5,(2,2));vv=corners+rng.normal(0,.001,corners.shape);perturbed=[]
        for o in observations:
            bias=head if o['camera']=='cam_head' else wrist;perturbed.append(dict(o,points=o['points']+bias+rng.normal(0,.3,o['points'].shape)))
        ff,rot=fit(perturbed,vv,f.x);centres.append(ff.x[:3]+rot.apply(com)+rng.normal(0,.001,3));angles.append(float((rot*rr.inv()).magnitude()))
    centre95=float(np.quantile(np.linalg.norm(np.asarray(centres)-pc,axis=1),.95));angle95=float(np.rad2deg(np.quantile(angles,.95)));result=dict(kind='estimated_reference_not_ground_truth',window_source_s=[21.5,21.96],head_frames=[538,544,549],right_wrist_frames=[549],object_coverage='1/1',head_frame_coverage='3/12 source frames in final window',trajectory_coverage=None,origin_m=f.x[:3].tolist(),q_wxyz=rr.as_quat()[[3,0,1,2]].tolist(),modeled_COM_m=pc.tolist(),landmark_rms_px=pixel_rms,observations=[dict(o,points=o['points'].tolist()) for o in observations],
        uncertainty=dict(method='128 deterministic Monte Carlo pixel/model perturbations; shared head-corner bias across stills',pixel_sigma_px=1.5,model_corner_sigma_m=.001,camera_translation_extra_sigma_m=.001,position_sensitivity_p95_m=centre95,rotation_sensitivity_p95_deg=angle95,calibration_rotation_error_not_included=True,not_a_calibrated_confidence_interval=True),limitations=['initial/wrist-based pose only seeds optimization; final solution has no grasp, nesting or support constraints','rounded cover corner model and camera calibration may introduce additional systematic error','axial full orientation is identifiable for rectangular laptop; yaw symmetry ambiguity resolved by source-visible cover and motion association'])
    (root/'reference_final_estimate.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ['observations','limitations']},indent=2))
    for cam,frame,file in [('cam_head',549,'head_0549.png'),('cam_right_wrist',549,'cam_right_wrist_0549.jpg')]:
        im=Image.open(root/'reference'/file).convert('RGB');dr=ImageDraw.Draw(im);uv=project(rr.apply(corners)+f.x[:3],num['vision__'+cam+'__intrinsic_matrix'],num['vision__'+cam+'__extrinsic_matrix'][frame]);dr.line([tuple(p) for p in np.r_[uv,uv[:1]]],fill=(255,50,50),width=2);im.save(root/'reference'/('estimated_final_'+cam+'.png'))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('root');main(a.parse_args().root)
