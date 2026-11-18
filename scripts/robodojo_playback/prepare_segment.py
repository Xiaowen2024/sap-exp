"""Complete the first two-bowl placement window using its first image only.

The carried pose is inferred, with uncertainty. No simulated outcome is used
to fit it, and it is never presented as an official object-pose recording.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import trimesh
from PIL import Image,ImageDraw
from scipy.optimize import minimize
from scipy.spatial import ConvexHull,cKDTree
from scipy.spatial.transform import Rotation as R


def main(root):
    root=Path(root);out=root/'segment';out.mkdir(exist_ok=True)
    scene=json.loads((root/'prepared/scene.json').read_text());data=np.load(root/'stack_bowls/numeric.npz',allow_pickle=False)
    full=np.load(root/'prepared/targets.npz',allow_pickle=False);K=data['vision__cam_head__intrinsic_matrix'];T=data['vision__cam_head__extrinsic_matrix'][90]
    reference=trimesh.load(root/'prepared/bowl_reference.obj',process=False);v=np.asarray(reference.vertices);v=v[ConvexHull(v).vertices]
    poses=data['state__right_ee_poses'];rot=R.from_quat(poses[:,[4,5,6,3]])
    anchor=rot[90]*rot[58].inv()
    landmarks=np.array([[312,178],[315,164],[327,149],[345,141],[361,142],[376,148],
                        [395,187],[391,201],[378,213],[360,218],[341,216],[325,208],[315,195]],float)
    def project(points):
        camera=(points-T[:3,3])@T[:3,:3];camera*=[1,-1,-1];uv=camera@K.T;return uv[:,:2]/uv[:,2:3]
    def outline(parameters):
        rotation=R.from_rotvec(np.r_[parameters[3:],0.])*anchor
        uv=project(rotation.apply(v)+parameters[:3]);poly=uv[ConvexHull(uv).vertices];boundary=[]
        for a,b in zip(poly,np.roll(poly,-1,axis=0)):
            boundary.extend(a+(b-a)*np.linspace(0,1,max(2,int(np.linalg.norm(b-a)*2)))[:,None])
        return np.asarray(boundary),poly,rotation
    def loss(parameters):
        points,_,_=outline(parameters);dist=cKDTree(points).query(landmarks)[0]
        return float(np.mean(dist**2)+.1*np.sum(np.asarray(parameters[3:])**2)/.2**2)
    uv=landmarks.mean(0);ray=np.array([(uv[0]-K[0,2])/K[0,0],-(uv[1]-K[1,2])/K[1,1],-1.])@T[:3,:3].T
    guess=T[:3,3]+ray*((.86-T[2,3])/ray[2])
    fit=minimize(loss,np.r_[guess,0.,0.],method='Powell',bounds=[(-.1,.25),(-.2,.2),(.82,1.),(-.3,.3),(-.3,.3)],options={'maxiter':180,'xtol':1e-6,'ftol':1e-7})
    boundary,polygon,rotation=outline(fit.x);origin=fit.x[:3];com=np.asarray(scene['assets']['bowl']['com']);position=origin+rotation.apply(com)
    omega=(rot[91]*rot[89].inv()).as_rotvec()/.08;linear=(poses[91,:3]-poses[89,:3])/.08+np.cross(omega,position-poses[90,:3])
    bodies=[]
    for body in scene['bodies']:
        if body['name'] in ('bowl0','bowl2','right_link7','right_link8'):bodies.append(body)
        if body['name']=='bowl2':body.update(p=position.tolist(),q=rotation.as_quat()[[3,0,1,2]].tolist(),v0=linear.tolist(),w0=omega.tolist())
        if body['name'].startswith('right_'):
            body.update(p=full[body['name']+'__pose'][3600,:3].tolist(),q=full[body['name']+'__pose'][3600,3:].tolist(),
                        v0=full[body['name']+'__velocity'][3600,:3].tolist(),w0=full[body['name']+'__velocity'][3600,3:].tolist())
    for asset in scene['assets'].values():asset['pieces']=['../prepared/'+p for p in asset['pieces']]
    target={k:full[k][3600:6001].copy() for k in full.files if not k.startswith('left_')};target['time']=np.arange(2401)*scene['dt']
    np.savez_compressed(out/'targets.npz',**target)
    scene.update(bodies=bodies,source_duration_s=2.4,tail_s=0.,source_start_frame=90,source_end_frame=150,
                 kind='inferred_two_bowl_placement_segment',contact_capacity=16384)
    scene['completion']['segment']={'initial_source_frame':90,'carried_pose':'fit to visible rim landmarks, anchored to recorded wrist rotation change',
        'origin_m':origin.tolist(),'q_wxyz':rotation.as_quat()[[3,0,1,2]].tolist(),'world_v0_m_s':linear.tolist(),'world_w0_rad_s':omega.tolist(),
        'visible_rim_landmarks_px':landmarks.tolist(),'rim_landmark_rms_px':float(np.sqrt(np.mean(cKDTree(boundary).query(landmarks)[0]**2))),
        'limitations':'pose and initial twist are inferred, not recorded; uncertainty is not admitted for a formal fidelity claim'}
    scene['ipc_parameters']={'d_hat':.0001,'eps_velocity':.001,'velocity_tol':.0001}
    scene['numerical_comparison_status']='IPC uses the predeclared geometry-scale diagnostic C5; not an equal-budget engine ranking'
    scene['deviations'].append('two-bowl placement fragment starts from an inferred carried pose; initial twist inferred from wrist motion')
    (out/'scene.json').write_text(json.dumps(scene,indent=2))
    image=Image.open(root/'frame90.jpg').convert('RGB');draw=ImageDraw.Draw(image);poly=[tuple(p) for p in polygon];draw.line(poly+[poly[0]],fill=(255,80,30),width=2)
    for x,y in landmarks:draw.ellipse((x-2,y-2,x+2,y+2),fill=(30,180,200))
    image.save(out/'carried_pose_overlay.jpg')
    print(json.dumps(scene['completion']['segment'],indent=2),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('root');main(ap.parse_args().root)
