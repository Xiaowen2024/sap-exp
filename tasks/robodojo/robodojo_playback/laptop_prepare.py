"""Source-only preparation of the closed-laptop insertion fragment.

Object initialization is inferred, never an official object-state recording.
Run before inspecting simulator outcomes. Does not advance any simulator.
"""
import argparse, hashlib, json, shutil
from pathlib import Path
import numpy as np
import trimesh
from PIL import Image, ImageDraw
from scipy.optimize import least_squares, linprog
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R, Slerp


def project(v,K,T):
    c=(np.asarray(v)-T[:3,3])@T[:3,:3]*[1,-1,-1]
    p=c@K.T
    return p[:,:2]/p[:,2:]


def prepare(root):
    root=Path(root).resolve();out=root/'scene_v1';out.mkdir(exist_ok=True)
    data=np.load(root/'reference/numeric.npz');K=data['vision__cam_head__intrinsic_matrix'];T=data['vision__cam_head__extrinsic_matrix'][450]
    rack=trimesh.load(root/'geometry/rack_reference.obj',force='mesh')
    rv=np.asarray(rack.vertices);rv=rv[ConvexHull(rv).vertices]
    marks=np.array([[446.8,202.8],[472.3,196.5],[491.0,234.5],[483.1,254.3],[468.4,254.7]])
    z=.765-rack.bounds[0,2]
    def outline(v):
        verts=project(R.from_euler('z',v[2]).apply(rv)+[v[0],v[1],z],K,T)
        return verts[ConvexHull(verts).vertices]
    def residual(v):
        poly=outline(v);a=poly;b=np.roll(poly,-1,axis=0);d=b-a
        u=np.clip(np.sum((marks[:,None,:]-a[None,:,:])*d[None,:,:],axis=2)/np.sum(d*d,axis=1),0,1)
        distances=np.linalg.norm(marks[:,None,:]-(a[None,:,:]+u[:,:,None]*d[None,:,:]),axis=2)
        return distances.min(axis=1)
    fit=least_squares(residual,[.32,-.10,np.pi/2],bounds=([.25,-.2,np.deg2rad(65)],[.4,.05,np.deg2rad(115)]),diff_step=1e-5,xtol=1e-12,ftol=1e-12,gtol=1e-12)
    rp=np.r_[fit.x[:2],z];rq=R.from_euler('z',fit.x[2]).as_quat()[[3,0,1,2]]
    pf=json.loads((root/'geometry/laptop_fit_provisional.json').read_text());orr=R.from_quat(np.asarray(pf['q'])[[1,2,3,0]]);origin=np.asarray(pf['origin'])
    # A single convex outer envelope of the already closed assembly; original
    # meshes remain available for independent geometry auditing.
    laptop=trimesh.load(root/'geometry/laptop_q0.obj',force='mesh').convex_hull
    props=laptop.mass_properties;com=props.center_mass;mass=.1
    assets={'laptop':{'pieces':['laptop/envelope.obj'],'mass':mass,'com':com.tolist(),'inertia':(props.inertia*mass/props.mass).tolist(),
                      'representation':'convex outer envelope of pinned official closed assembly; hinge q=0 deg fixed'}}
    (out/'laptop').mkdir(exist_ok=True);laptop.vertices-=com;laptop.export(out/'laptop/envelope.obj')
    pcs=sorted((root/'geometry/rack_pieces').glob('piece_*.obj'))
    if not pcs:raise RuntimeError('rack convex decomposition is not ready')
    (out/'rack').mkdir(exist_ok=True)
    for p in pcs:shutil.copy(p,out/'rack'/p.name)
    assets['rack']={'pieces':['rack/'+p.name for p in pcs],'static':True}
    x5root=root/'shared_x5/articulated';x5=json.loads((x5root/'scene.json').read_text())
    for kind in ['base','link7','link8']:
        assets[kind]=x5['assets'][kind]
        for p in assets[kind]['pieces']:
            dst=out/p;dst.parent.mkdir(exist_ok=True,parents=True);shutil.copy(x5root/p,dst)
    hz=float(data['additional_info__frequency']);start=450;end=len(data['state__right_ee_poses'])-1;dt=.001;duration=(end-start)/hz;tail=1.5
    steps=int(round((duration+tail)/dt));steps+=(-steps)%10;time=np.arange(steps+1)*dt;source=np.arange(end+1)/hz;cl=np.minimum(time+start/hz,source[-1]);target={'time':time}
    grippers=[]
    for side in ['left','right']:
        P=data['state__'+side+'_ee_poses'];pos=np.column_stack([np.interp(cl,source,P[:,j]) for j in range(3)])
        rr=Slerp(source,R.from_quat(P[:,[4,5,6,3]]))(cl);q=rr.as_quat()[:,[3,0,1,2]];v=np.gradient(pos,dt,axis=0);w=(rr[1:]*rr[:-1].inv()).as_rotvec()/dt;w=np.vstack([w,w[-1]])
        target[side+'_base__pose']=np.c_[pos,q];target[side+'_base__velocity']=np.c_[v,w];cmd=-.01+.054*np.interp(cl,source,data['action__'+side+'_ee_joint_states'][:,0]);target['command_'+side]=cmd
        grippers.append(dict(name=side+'_base',asset='base',p=pos[0].tolist(),q=q[0].tolist(),driven=True,is_gripper=True,v0=v[0].tolist(),w0=w[0].tolist()))
        for old in x5['bodies']:
            if old.get('parent')!=side+'_base':continue
            b=dict(old);initial=float(np.clip(cmd[0],0,.044));b['joint_initial']=initial;b['p']=(pos[0]+rr[0].apply(np.asarray(b['zero_p'])+np.asarray(b['axis'])*initial)).tolist();b['q']=q[0].tolist();grippers.append(b)
    pcom=origin+orr.apply(com);wr=target['right_base__velocity'][0,3:];vr=target['right_base__velocity'][0,:3];vobj=vr+np.cross(wr,pcom-target['right_base__pose'][0,:3])
    # Native joint command requests preload; it is not the measured loaded
    # finger opening. Initialize each finger at geometric touching, before
    # either simulator has run, rather than embedding fingers in the plate.
    le=ConvexHull(orr.apply(laptop.vertices)+pcom).equations;initial_grasp={}
    base=next(b for b in grippers if b['name']=='right_base');br=R.from_quat(np.asarray(base['q'])[[1,2,3,0]])
    for b in grippers:
        if b.get('parent')!='right_base':continue
        vertices=[np.asarray(trimesh.load(out/p,force='mesh').vertices) for p in assets[b['asset']]['pieces']]
        def overlap(jaw):
            maximum=0.
            for vv in vertices:
                fe=ConvexHull(br.apply(vv+np.asarray(b['zero_p'])+np.asarray(b['axis'])*jaw)+base['p']).equations;e=np.vstack([le,fe])
                result=linprog([0.,0.,0.,-1.],A_ub=np.c_[e[:,:3],np.ones(len(e))],b_ub=-e[:,3],bounds=[(None,None)]*4,method='highs')
                if result.success:maximum=max(maximum,float(result.x[3]))
            return maximum
        lo,hi=0.,.044
        for _ in range(24):
            mid=(lo+hi)/2
            if overlap(mid)>1e-8:lo=mid
            else:hi=mid
        b['joint_initial']=min(.044,hi+1e-6);b['p']=(np.asarray(base['p'])+br.apply(np.asarray(b['zero_p'])+np.asarray(b['axis'])*b['joint_initial'])).tolist()
        initial_grasp[b['name']]=dict(joint_initial_m=b['joint_initial'],requested_command_m=float(target['command_right'][0]),geometric_initial_overlap_radius_m=overlap(b['joint_initial']))
    bodies=[dict(name='laptop',asset='laptop',p=pcom.tolist(),q=pf['q'],driven=False,v0=vobj.tolist(),w0=wr.tolist()),*grippers]
    scene=dict(kind='reduced_closed_laptop_insertion',task='store_laptop_and_headphones',episode=0,dt=dt,gravity=[0.,0.,-9.81],mu=.45,table_top_z=.765,
        source_start_frame=start,source_end_frame=end,source_hz=hz,source_start_s=start/hz,source_duration_s=duration,tail_s=steps*dt-duration,
        assets=assets,bodies=bodies,static_bodies=[dict(name='rack',asset='rack',p=rp.tolist(),q=rq.tolist())],articulated_grippers=True,contact_capacity=8192,explicit_finger_self_collision_exclusions=True,
        drive=x5['drive'],finger_actuator=x5['finger_actuator'],source_control_audit=x5['source_control_audit'],telemetry_nodes=['laptop','right_link7','right_link8','left_link7','left_link8','rack','table'],
        phases={'grasp_s':[0.,2.60],'rack_support_s':[3.30,steps*dt],'source_final_s':[3.50,duration],'tail_final_s':[steps*dt-.5,steps*dt]},
        deviations=['official simulation demo, not real-world ground truth','closed hinge fixed at q=0 degrees; no headphone or closing stage',
            'loaded moving grasp initialized from source-only pose fit and wrist twist; contact/preload history absent',
            'convex closed-laptop envelope and separate floor/wall rack hulls replace official collision representations',
            'detached articulated X5 grippers replace full arms; nominal mass/inertia and controller reused from stack-bowls r3',
            'single Coulomb friction .45 shared on all contacts; no torsional/rolling friction; not a full official material port',
            'static rack follows task Geometry designation; source USD rigid-body API does not determine task mobility',
            '1.5 s appended hold after source end is a diagnostic, not source pose ground truth'])
    fitdoc=dict(laptop=pf,initial_grasp=initial_grasp,rack=dict(position_m=rp.tolist(),q_wxyz=rq.tolist(),source_frame=0,source_landmarks_px=marks.tolist(),silhouette_rms_px=float(np.sqrt(np.mean(residual(fit.x)**2)))),
        uncertainties={'method':'not statistically calibrated; source-only pose sensitivity to be reported','initial_state_is_estimated':True,'no_fidelity_ranking_below_uncertainty':True})
    (root/'scene_fit.json').write_text(json.dumps(fitdoc,indent=2));(out/'scene.json').write_text(json.dumps(scene,indent=2));np.savez_compressed(out/'targets.npz',**target)
    im=Image.open(root/'reference/head_0450.png').convert('RGB');draw=ImageDraw.Draw(im)
    uv=project(orr.apply(laptop.vertices+com)+origin,K,T);poly=uv[ConvexHull(uv).vertices];draw.line([tuple(x) for x in np.r_[poly,poly[:1]]],fill=(255,40,40),width=2)
    poly=outline(fit.x);draw.line([tuple(x) for x in np.r_[poly,poly[:1]]],fill=(30,230,50),width=2);im.save(root/'initial_pose_overlay.png')
    hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(out.rglob('*')) if p.is_file()}
    (root/'scene_manifest.json').write_text(json.dumps(hashes,indent=2));print(json.dumps(dict(steps=steps,source_duration_s=duration,rack_pose=rp.tolist(),rack_yaw_deg=float(np.rad2deg(fit.x[2])),rack_fit_px=fitdoc['rack']['silhouette_rms_px'],laptop_pose=pcom.tolist(),initial_twist=np.r_[vobj,wr].tolist()),indent=2))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('root');prepare(a.parse_args().root)
