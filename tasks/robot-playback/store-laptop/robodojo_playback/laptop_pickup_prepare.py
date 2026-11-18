"""Extend the frozen closed-laptop model to source pickup + handoff + insertion.

Source-only scene reconstruction; transferred numerical settings are immutable.
No simulator is imported or advanced here.
"""
import argparse,json,hashlib
from pathlib import Path
import numpy as np,trimesh
from PIL import Image,ImageDraw
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R,Slerp
from .laptop_prepare import project


def main(root):
    root=Path(root).resolve();out=root/'scene_v1';scene=json.loads((out/'scene.json').read_text());data=np.load(root/'reference/numeric.npz');T=data['vision__cam_head__extrinsic_matrix'][225];K=data['vision__cam_head__intrinsic_matrix']
    mesh=trimesh.load(root/'geometry/laptop_stand_reference.obj',force='mesh');v=np.asarray(mesh.vertices);zstand=.765-mesh.bounds[0,2];vv=v[ConvexHull(v).vertices];marks=np.array([[284.,149.],[421.,162.],[416.,222.],[274.,209.]])
    def poly(vv,pa):
        uv=project(R.from_euler('z',pa[2]).apply(vv)+[pa[0],pa[1],zstand],K,T);return uv[ConvexHull(uv).vertices]
    def fit_res(pa):
        points=poly(vv,pa);a=points;b=np.roll(a,-1,axis=0);d=b-a;u=np.clip(np.sum((marks[:,None]-a[None])*d[None],axis=2)/np.sum(d*d,axis=1),0,1);return np.linalg.norm(marks[:,None]-(a[None]+u[:,:,None]*d[None]),axis=2).min(1)
    fit=least_squares(fit_res,[.06,.02,np.deg2rad(-7)],bounds=([.02,-.01,np.deg2rad(-20)],[.12,.06,np.deg2rad(5)]),diff_step=1e-5,max_nfev=200);sp=np.r_[fit.x[:2],zstand];sq=R.from_euler('z',fit.x[2]).as_quat()[[3,0,1,2]]
    # Keep the open underside: a separate deck, side legs and phone bracket.
    # Filled holes in the side legs are outside the nominal pickup corridor;
    # their collision forces remain included in telemetry and must be audited.
    regions=[('deck',v[v[:,2]>=.02195]),('left_leg',v[(v[:,0]<-.138)&(v[:,2]<=.02198)]),('right_leg',v[(v[:,0]>.138)&(v[:,2]<=.02198)]),('phone_bracket',v[(v[:,0]>.045)&(v[:,0]<.135)&(v[:,2]<.02198)])];folder=out/'laptop_stand';folder.mkdir(exist_ok=True);paths=[]
    for i,(name,vertices) in enumerate(regions):
        m=trimesh.convex.convex_hull(vertices);p=folder/f'{i:02d}_{name}.obj';m.export(p);paths.append(str(p.relative_to(out)))
    scene['assets']['laptop_stand']={'pieces':paths,'static':True,'representation':'source deck, separate side-leg envelopes and underside phone bracket; underside between legs remains open'}
    # Supported initial state. The unconstrained planar PnP branch puts the
    # laptop through the stand. Fit the visible corners at the source-supported
    # deck height instead, documenting this reconstruction assumption.
    top=float(np.quantile(v[(abs(v[:,0])<.08)&(abs(v[:,1])<.035),2],.99));origin_z=zstand+top+.0001
    corners=np.array([[-.08965,-.0782,.009],[.08965,-.0782,.009],[.08965,.0477,.009],[-.08965,.0477,.009]]);pixels=np.array([[301.,199.],[389.,213.],[395.,168.],[315.,156.]])
    def lp_res(x):return (project(R.from_euler('z',x[2]).apply(corners)+[x[0],x[1],origin_z],K,T)-pixels).ravel()
    lp=least_squares(lp_res,[.065,.02,np.deg2rad(-12)],max_nfev=200);lr=R.from_euler('z',lp.x[2]);origin=np.r_[lp.x[:2],origin_z];com=np.asarray(scene['assets']['laptop']['com']);pcom=origin+lr.apply(com)
    start=225;end=len(data['state__left_ee_poses'])-1;hz=float(data['additional_info__frequency']);dt=scene['dt'];duration=(end-start)/hz;steps=int(round((duration+1.5)/dt));steps+=(-steps)%10;time=np.arange(steps+1)*dt;source=np.arange(end+1)/hz;cl=np.minimum(time+start/hz,source[-1]);targets={'time':time};bodies=[dict(name='laptop',asset='laptop',p=pcom.tolist(),q=lr.as_quat()[[3,0,1,2]].tolist(),driven=False,v0=[0,0,0],w0=[0,0,0])]
    old={b['name']:b for b in scene['bodies']}
    for side in ['left','right']:
        P=data['state__'+side+'_ee_poses'];pos=np.column_stack([np.interp(cl,source,P[:,j]) for j in range(3)]);rot=Slerp(source,R.from_quat(P[:,[4,5,6,3]]))(cl);q=rot.as_quat()[:,[3,0,1,2]];vel=np.gradient(pos,dt,axis=0);omega=(rot[1:]*rot[:-1].inv()).as_rotvec()/dt;omega=np.vstack([omega,omega[-1]]);cmd=-.01+.054*np.interp(cl,source,data['action__'+side+'_ee_joint_states'][:,0]);targets[side+'_base__pose']=np.c_[pos,q];targets[side+'_base__velocity']=np.c_[vel,omega];targets['command_'+side]=cmd
        b=dict(old[side+'_base']);b.update(p=pos[0].tolist(),q=q[0].tolist(),v0=vel[0].tolist(),w0=omega[0].tolist());bodies.append(b)
        for link in ['link7','link8']:
            b=dict(old[side+'_'+link]);initial=float(np.clip(cmd[0],0,.044));b.update(joint_initial=initial,p=(pos[0]+rot[0].apply(np.asarray(b['zero_p'])+np.asarray(b['axis'])*initial)).tolist(),q=q[0].tolist());bodies.append(b)
    rack=scene['static_bodies'][0];scene.update(kind='closed_laptop_pickup_handoff_insertion_transfer',bodies=bodies,static_bodies=[rack,dict(name='laptop_stand',asset='laptop_stand',p=sp.tolist(),q=sq.tolist())],source_start_frame=start,source_start_s=start/hz,source_end_frame=end,source_duration_s=duration,tail_s=steps*dt-duration)
    scene['telemetry_nodes']=list(dict.fromkeys(scene['telemetry_nodes']+['laptop_stand']));scene['phases']={'grasp_s':[8.3,11.6],'left_grasp_s':[3.2,7.6],'rack_support_s':[12.3,steps*dt],'source_final_s':[12.5,duration],'tail_final_s':[steps*dt-.5,steps*dt]};scene['contact_windows']=[['laptop','left_link7','left_grasp_s'],['laptop','left_link8','left_grasp_s'],['laptop','right_link7','grasp_s'],['laptop','right_link8','grasp_s'],['laptop','rack','rack_support_s']];scene['closeup_window_s']=[9.4,12.96]
    scene['deviations']=[x for x in scene['deviations'] if not x.startswith(('loaded moving grasp','convex closed-laptop'))]+['initial closed laptop is supported on the reconstructed original stand, not initialized inside a grasp','closed laptop envelope and shared rack/stand convex regions replace source collision representations','initial resting pose fitted at reconstructed stand deck height; monocular PnP depth branch is physically ambiguous','stand side-leg holes and phone bracket approximated by regional convex envelopes; relevant contacts must be checked','frozen short-segment winner parameters transferred without tuning; no resets at pickup or handoff']
    (out/'scene.json').write_text(json.dumps(scene,indent=2));np.savez_compressed(out/'targets.npz',**targets)
    fitdoc={'stand':{'p':sp.tolist(),'q':sq.tolist(),'silhouette_rms_px':float(np.sqrt(np.mean(fit_res(fit.x)**2))),'landmarks':marks.tolist()},'laptop':{'origin_m':origin.tolist(),'q':lr.as_quat()[[3,0,1,2]].tolist(),'corner_rms_px':float(np.sqrt(np.mean(np.sum(lp_res(lp.x).reshape(-1,2)**2,axis=1)))),'landmarks':pixels.tolist(),'supported_height_assumption':True,'deck_top_local_z_m':top,'initial_gap_m':.0001},'not_ground_truth':True}
    (root/'initial_fit.json').write_text(json.dumps(fitdoc,indent=2));im=Image.open(root/'reference/head_0225.png').convert('RGB');dr=ImageDraw.Draw(im);p=project(lr.apply(corners)+origin,K,T);dr.line([tuple(x) for x in np.r_[p,p[:1]]],fill=(255,40,40),width=2);p=poly(vv,fit.x);dr.line([tuple(x) for x in np.r_[p,p[:1]]],fill=(20,250,30),width=2);im.save(root/'initial_pose_overlay.png')
    for eng,from_name in [('sap','sap_02.json'),('mujoco_warp','mujoco_warp_03.json')]:
        cfg=json.loads((root/'parameters'/from_name).read_text());cfg['why']='Frozen transfer from short insertion winner '+from_name+'; one full pickup/handoff/insertion rollout, no tuning.';(root/'parameters'/(eng+'_01.json')).write_text(json.dumps(cfg,indent=2))
    hashes={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest() for f in out.rglob('*') if f.is_file()};(root/'scene_manifest.json').write_text(json.dumps(hashes,indent=2));print(json.dumps({'steps':steps,'source_duration_s':duration,'fit':fitdoc},indent=2))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('root');main(a.parse_args().root)
