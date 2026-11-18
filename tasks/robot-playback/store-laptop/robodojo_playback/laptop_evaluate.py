"""Offline R=1 insertion metrics. Never advances or replays a simulator."""
import argparse, csv, hashlib, json, time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from .common import load


def rms(t,v):return float(np.sqrt(np.trapezoid(np.sum(v*v,axis=1),t)/(t[-1]-t[0])))


def longest_false(on,dt):
    best=cur=0
    for x in on:
        cur=0 if x else cur+1;best=max(best,cur)
    return float(best*dt)


def contact_metrics(scene,p):
    t=p['time'];names=list(p['pair_names']);rows={}
    wanted=scene.get('contact_windows',[('laptop','right_link7','grasp_s'),('laptop','right_link8','grasp_s'),('laptop','rack','rack_support_s')])
    for a,b,phase in wanted:
        pair=a+'/'+b if a+'/'+b in names else b+'/'+a;j=names.index(pair);lo,hi=scene['phases'][phase];mask=(t>=lo-1e-9)&(t<hi-1e-9);force=p['normal_force_sum_N'][mask,j];gap=p['min_native_gap_m'][mask,j];dt=scene['dt']
        row={'window_s':[lo,hi],'samples':len(force),'candidate_duty':float(np.mean(p['candidate_count'][mask,j]>0)),
            'nonpositive_native_gap_duty':float(np.mean(gap<=0)),'normal_force_mean_N':float(np.mean(force)),'normal_force_peak_N':float(np.max(force)),'thresholds':{}}
        for threshold in [.001,.01,.1]:
            on=force>threshold;row['thresholds'][str(threshold)]={'force_bearing_duty':float(on.mean()),'longest_no_force_s':longest_false(on,dt),'recontact_count':int(np.sum(on[1:]&~on[:-1]))}
        rows[pair]=row
    return rows


def settling(scene,tr):
    result={};t=tr['time'];pose=tr['laptop__pose'];vel=tr['laptop__velocity']
    for phase in ['source_final_s','tail_final_s']:
        lo,hi=scene['phases'][phase];mask=(t>=lo-1e-8)&(t<=hi+1e-8);tt=t[mask];pp=pose[mask];vv=vel[mask];rr=R.from_quat(pp[:,[4,5,6,3]])
        result[phase]={'window_s':[lo,hi],'samples':len(tt),'linear_speed_rms_mm_s':rms(tt,vv[:,:3])*1000,'angular_speed_rms_rad_s':rms(tt,vv[:,3:]),
            'linear_speed_peak_mm_s':float(np.linalg.norm(vv[:,:3],axis=1).max()*1000),'net_COM_drift_mm':float(np.linalg.norm(pp[-1,:3]-pp[0,:3])*1000),
            'orientation_change_deg':float(np.rad2deg((rr[-1]*rr[0].inv()).magnitude())),'final_COM_m':pp[-1,:3].tolist()}
    return result


def geometry_fcl_unvalidated(scene,tr,out):
    import fcl,trimesh
    # Collision lib independent of either simulator. Convex component MTDs do
    # not equal the minimum translation needed to separate the full union.
    assets={};parts=[];request=fcl.CollisionRequest(num_max_contacts=1,enable_contact=True)
    for kind in ['laptop','rack','link7','link8']:
        assets[kind]=[]
        for file in scene['assets'][kind]['pieces']:
            mesh=trimesh.load(file,force='mesh',process=False);v=np.ascontiguousarray(mesh.vertices,dtype=np.float64);faces=np.c_[np.full(len(mesh.faces),3),mesh.faces].astype(np.int32).ravel()
            shape=fcl.Convex(v,len(mesh.faces),faces);assets[kind].append((shape,mesh.bounds.copy()))
    names=['laptop','right_link7','right_link8','left_link7','left_link8','rack','table'];objects={}
    for n in names:
        if n=='table':objects[n]=[(fcl.CollisionObject(fcl.Box(1.4,1.1,.05),fcl.Transform(np.eye(3),[0,-.05,.74])),None)];continue
        kind='laptop' if n=='laptop' else 'rack' if n=='rack' else n.split('_')[1];objects[n]=[(fcl.CollisionObject(s),bounds) for s,bounds in assets[kind]]
    static={s['name']:s for s in scene['static_bodies']};pose={};rows=[];peaks={n:dict(depth_m=0.,time_s=None) for n in names[1:]};indices=np.arange(0,len(tr['time']),10)
    for ii,i in enumerate(indices):
        boxes={}
        for n in names:
            if n=='table':boxes[n]=[(np.array([-.7,-.6,.715]),np.array([.7,.5,.765]))];continue
            p=np.r_[static[n]['p'],static[n]['q']] if n in static else tr[n+'__pose'][i];rot=R.from_quat(p[[4,5,6,3]]).as_matrix();position=p[:3];boxes[n]=[]
            for obj,bounds in objects[n]:
                obj.setTransform(fcl.Transform(rot,position));corner=np.array([[x,y,z] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])@rot.T+position;boxes[n].append((corner.min(0),corner.max(0)))
        row={'time_s':float(tr['time'][i])}
        for n in names[1:]:
            depth=0.
            for j,(obj,_) in enumerate(objects[n]):
                alo,ahi=boxes['laptop'][0];blo,bhi=boxes[n][j]
                if np.any(ahi<blo) or np.any(bhi<alo):continue
                res=fcl.CollisionResult();hit=fcl.collide(objects['laptop'][0][0],obj,request,res)
                if hit:
                    if not res.contacts:raise RuntimeError('independent observer returned collision without depth')
                    value=max(float(c.penetration_depth) for c in res.contacts)
                    if not np.isfinite(value) or value<0:raise RuntimeError('invalid FCL depth')
                    depth=max(depth,value)
            row[n+'_depth_m']=depth
            if depth>peaks[n]['depth_m']:peaks[n]=dict(depth_m=depth,time_s=row['time_s'])
        rows.append(row)
        if ii%100==0:print('offline geometry',ii,len(indices),flush=True)
    with (out/'geometric_depth.csv').open('w',newline='') as f:
        wr=csv.DictWriter(f,fieldnames=list(rows[0]));wr.writeheader();wr.writerows(rows)
    return dict(method='python-fcl convex-convex collision penetration depth; maximum per component pair in shared geometry',
        meaning='sampled maximum component-pair separation depth, not exact global union penetration or a continuous-time maximum',dt_s=.01,samples=len(rows),fcl_version=getattr(fcl,'__version__','unknown'),pairs={'laptop/'+k:v for k,v in peaks.items()},initial_depths_m={k:rows[0][k+'_depth_m'] for k in names[1:]})


def geometry(scene,tr,out):
    """Proved bounds on convex component translation penetration, independent of FCL.

    If a common inscribed ball has radius r, translations shorter than 2r
    cannot separate the bodies. A face-normal translation that separates their
    projections supplies an upper bound. Report both, never an unverified EPA
    depth. All source convex planes retain their original normalized values.
    """
    import trimesh
    from scipy.spatial import ConvexHull
    from scipy.optimize import linprog
    names=scene.get('telemetry_nodes',['laptop','right_link7','right_link8','left_link7','left_link8','rack','table']);parts={};objects={b['name']:b for b in scene['bodies']+scene['static_bodies']};static={b['name']:b for b in scene['static_bodies']}
    for n in names:
        meshes=[trimesh.creation.box(extents=[1.4,1.1,.05])] if n=='table' else [trimesh.load(p,force='mesh',process=False) for p in scene['assets'][objects[n]['asset']]['pieces']]
        parts[n]=[]
        for mesh in meshes:
            v=np.asarray(mesh.vertices);h=ConvexHull(v);eq=h.equations;_,unique=np.unique(np.round(eq,10),axis=0,return_index=True);parts[n].append((v[h.vertices],eq[np.sort(unique)]))
    rows=[];peaks={n:dict(depth_m=0.,lower_bound_m=0.,time_s=None,lower_bound_time_s=None) for n in names[1:]};indices=np.arange(0,len(tr['time']),10);eps=2e-7
    for ii,i in enumerate(indices):
        world={}
        for n in names:
            if n=='table':position=np.array([0.,-.05,.74]);rotation=np.eye(3)
            else:
                b=static[n] if n in static else None;p=np.r_[b['p'],b['q']] if b else tr[n+'__pose'][i];position=p[:3];rotation=R.from_quat(p[[4,5,6,3]]).as_matrix()
            world[n]=[]
            for vertices,eq in parts[n]:
                vv=vertices@rotation.T+position;nn=eq[:,:3]@rotation.T;ee=np.c_[nn,eq[:,3]-nn@position];world[n].append((vv,ee,vv.min(0),vv.max(0)))
        a,ae,alo,ahi=world['laptop'][0];row={'time_s':float(tr['time'][i])}
        for n in names[1:]:
            lower=upper=0.
            for b,be,blo,bhi in world[n]:
                if np.any(ahi<blo) or np.any(bhi<alo):continue
                ee=np.vstack([ae,be]);res=linprog([0.,0.,0.,-1.],A_ub=np.c_[ee[:,:3],np.ones(len(ee))],b_ub=-ee[:,3],bounds=[(None,None)]*4,method='highs')
                if not res.success:raise RuntimeError('independent intersection LP failed: '+res.message)
                radius=float(res.x[3])
                if radius<=1e-8:continue
                lo=max(0.,2*radius-2*eps)
                hi=min(float(np.min(-ae[:,3]-np.min(b@ae[:,:3].T,axis=0))),float(np.min(-be[:,3]-np.min(a@be[:,:3].T,axis=0))))+2*eps
                if hi+1e-7<lo:raise RuntimeError('invalid independent penetration bracket')
                lower=max(lower,lo);upper=max(upper,hi)
            row[n+'_lower_m']=lower;row[n+'_upper_m']=upper;row[n+'_depth_m']=upper
            if upper>peaks[n]['depth_m']:peaks[n]['depth_m']=upper;peaks[n]['time_s']=row['time_s']
            if lower>peaks[n]['lower_bound_m']:peaks[n]['lower_bound_m']=lower;peaks[n]['lower_bound_time_s']=row['time_s']
        rows.append(row)
        if ii%100==0:print('validated geometry bounds',ii,len(indices),flush=True)
    with (out/'geometric_depth.csv').open('w',newline='') as f:
        wr=csv.DictWriter(f,fieldnames=list(rows[0]));wr.writeheader();wr.writerows(rows)
    return dict(method='convex intersection maximum-inscribed-ball LP lower bound (2r), face-normal translation upper bound',meaning='sampled maximum component-pair penetration bracket; depth_m is the conservative upper bound, not exact depth; not a global union or continuous-time maximum',dt_s=.01,samples=len(rows),numerical_margin_m=2*eps,pairs={'laptop/'+k:v for k,v in peaks.items()},initial_depths_m={k:rows[0][k+'_upper_m'] for k in names[1:]},observer_validation='FCL EPA depth rejected after it exceeded a proved separation upper bound; original FCL outputs remain archived as invalid')


def main(root,run_names):
    root=Path(root).resolve();scene,_=load(root/'scene_v1');existing=root/'metrics_v2.json';results=json.loads(existing.read_text())['results'] if existing.exists() else {}
    for name in run_names:
        run=root/'runs'/name;out=run/'evaluation_v2';out.mkdir(exist_ok=True);health=json.loads((run/'health.json').read_text());tr=np.load(run/'dense_trace.npz');contacts=np.load(run/'contact_pairs.npz');start=time.perf_counter()
        if not health['finite'] or health['steps']!=health['expected_steps']:raise RuntimeError('cannot score an invalid/incomplete rollout')
        g=geometry(scene,tr,out);s=settling(scene,tr);cm=contact_metrics(scene,contacts);release=(contacts['time']>=scene['phases']['rack_support_s'][0]);pn=list(contacts['pair_names']);gripper_names=[x for x in pn if x.startswith('laptop/') and ('link7' in x or 'link8' in x)];maxgrip=max(float(contacts['normal_force_sum_N'][release,pn.index(x)].max()) for x in gripper_names)
        last=tr['laptop__pose'][-1];rot=R.from_quat(last[[4,5,6,3]]);rack=scene['static_bodies'][0];rackr=R.from_quat(np.array(rack['q'])[[1,2,3,0]]);rel=rackr.inv().apply(last[:3]-rack['p']);depth=g['pairs']['laptop/rack']['depth_m']
        result=dict(name=name,engine=health['engine'],health={'finite':health['finite'],'steps':health['steps'],'expected_steps':health['expected_steps'],'truncated':health.get('truncated'),'drive_gate_pass':health['drive_gate_pass'],'tracking':health['tracking'],'native_peak_overlap_m':health['engine_peak_overlap_m'],'wall_s':health['wall_s']},
            geometry=g,contact=cm,settling=s,release_check={'window_start_s':scene['phases']['rack_support_s'][0],'peak_laptop_gripper_normal_force_N':maxgrip},final_diagnostics={'COM_in_rack_frame_m':rel.tolist(),'plate_short_axis_tilt_deg':float(np.rad2deg(np.arccos(np.clip(abs(rot.as_matrix()[2,1]),0,1))))},
            reference_errors={'trajectory_rmse_m':None,'final_pose_error_m':None,'reason':'no admitted per-frame estimated reference with calibrated uncertainty; source-only initial fit is not a trajectory truth'},
            task_outcome={'observed_insertion':None,'valid_success':None,'reason':'requires video plus wall-crossing/support audit; no official policy reward used'},
            timing_note='single rollout; native wall time includes cold JIT/capture work with telemetry; not a warmed performance benchmark',offline_analysis_wall_s=time.perf_counter()-start)
        result['source_hashes']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [run/'dense_trace.npz',run/'contact_pairs.npz',run/'input_scene.json']};(out/'metrics.json').write_text(json.dumps(result,indent=2));results[name]=result
        print(name,'rack penetration upper bound mm',depth*1000,'tail',s['tail_final_s'],'gripper force tail',maxgrip,flush=True)
    (root/'metrics_v2.json').write_text(json.dumps(dict(protocol='robodojo-laptop-insertion-v1',rollouts_per_config=1,results=results),indent=2))

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--root',required=True);a.add_argument('--runs',nargs='+',required=True);v=a.parse_args();main(v.root,v.runs)
