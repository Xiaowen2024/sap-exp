"""GPU execution of the same physical drive, with native SAP integration.

Only host diagnostic collection is disabled for outer CUDA graph capture.
Validate against the host-driven path before interpreting any object run.
"""
import hashlib
import json
from pathlib import Path
import time
import os
import numpy as np
import warp as wp
from .run import sap_config


@wp.kernel
def drive(counter:wp.array(dtype=wp.int32),target:wp.array3d(dtype=wp.float32),indices:wp.array(dtype=wp.int32),
          params:wp.array2d(dtype=wp.float32),q:wp.array(dtype=wp.transformf),v:wp.array(dtype=wp.spatial_vectorf),f:wp.array(dtype=wp.spatial_vectorf)):
    i=wp.tid();b=indices[i];k=counter[0]
    tp=wp.vec3(target[k,i,0],target[k,i,1],target[k,i,2]);tq=wp.quat(target[k,i,4],target[k,i,5],target[k,i,6],target[k,i,3])
    tv=wp.vec3(target[k,i,7],target[k,i,8],target[k,i,9]);tw=wp.vec3(target[k,i,10],target[k,i,11],target[k,i,12])
    p=wp.transform_get_translation(q[b]);qq=wp.transform_get_rotation(q[b]);vv=v[b]
    linear=wp.vec3(vv[0],vv[1],vv[2]);angular=wp.vec3(vv[3],vv[4],vv[5])
    force=params[i,0]*(tp-p)+params[i,1]*(tv-linear);nf=wp.length(force)
    if nf>params[i,2]:force=force*(params[i,2]/nf)
    force=force+wp.vec3(0.0,0.0,params[i,6])
    qe=tq*wp.quat_inverse(qq)
    if qe[3]<0.0:qe=wp.quat(-qe[0],-qe[1],-qe[2],-qe[3])
    xyz=wp.vec3(qe[0],qe[1],qe[2]);length=wp.length(xyz);er=2.0*xyz
    if length>1.0e-9:er=xyz*(2.0*wp.atan2(length,qe[3])/length)
    tau=params[i,3]*er+params[i,4]*(tw-angular);nt=wp.length(tau)
    if nt>params[i,5]:tau=tau*(params[i,5]/nt)
    f[b]=wp.spatial_vector(force[0],force[1],force[2],tau[0],tau[1],tau[2])


@wp.kernel
def tick(counter:wp.array(dtype=wp.int32)):
    counter[0]=counter[0]+1


@wp.kernel
def record(counter:wp.array(dtype=wp.int32),indices:wp.array(dtype=wp.int32),q:wp.array(dtype=wp.transformf),v:wp.array(dtype=wp.spatial_vectorf),
           values:wp.array3d(dtype=wp.float32),bad:wp.array(dtype=wp.int32)):
    i=wp.tid();b=indices[i];k=counter[0];p=wp.transform_get_translation(q[b]);qq=wp.transform_get_rotation(q[b]);vv=v[b]
    for j in range(7):
        if not wp.isfinite(q[b][j]):wp.atomic_max(bad,0,1)
    for j in range(6):
        if not wp.isfinite(vv[j]):wp.atomic_max(bad,0,1)
    if k%10==0:
        r=k/10
        values[r,i,0]=p[0];values[r,i,1]=p[1];values[r,i,2]=p[2]
        values[r,i,3]=qq[3];values[r,i,4]=qq[0];values[r,i,5]=qq[1];values[r,i,6]=qq[2]
        for j in range(6):values[r,i,7+j]=vv[j]


@wp.kernel
def contacts_health(count:wp.array(dtype=wp.int32),phi:wp.array2d(dtype=wp.float32),stats:wp.array(dtype=wp.int32),minimum:wp.array(dtype=wp.float32)):
    i=wp.tid();n=count[0]
    if i==0:
        wp.atomic_max(stats,0,n)
        if n>0:wp.atomic_add(stats,1,1)
    if i<n:wp.atomic_min(minimum,0,phi[0,i])


def run(scene,target,out,objects):
    from dataclasses import replace
    from sap_warp import SolverSAP,load_sap_scene
    from sap_warp.collision.pipeline import SapCollisionPipeline
    from sap_warp.resources.collision_model import sap_collision_state_from_state
    from sap_warp.sap_kinematics import sap_eval_fk
    if os.environ.get('ROBODOJO_WARP_CACHE'):wp.config.kernel_cache_dir=os.environ['ROBODOJO_WARP_CACHE']
    wp.init();device='cuda:0';cap=scene['contact_capacity'];n=len(target['time'])-1
    if n%10:raise ValueError('graph trace requires step count divisible by 10')
    path=out/'sap_scene.json';path.write_text(json.dumps(sap_config(scene,objects),indent=2))
    loaded=load_sap_scene(path,device=device,naconmax=cap,strict=True)
    model,A,control=loaded.sap_model,loaded.sap_state,loaded.sap_control;B=model.state();labels=list(model.body_label)
    ids={b['name']:labels.index(b['name']) for b in scene['bodies']};jl=list(model.joint_label);qs=model.joint_q_start.numpy();vs=model.joint_qd_start.numpy();q=A.joint_q.numpy();velocity=A.joint_qd.numpy();velocity[:]=0
    for b in scene['bodies']:
        offset=qs[jl.index(b['name']+'_free')];q[offset:offset+7]=np.r_[b['p'],np.asarray(b['q'])[[1,2,3,0]]]
        vi=vs[jl.index(b['name']+'_free')];velocity[vi:vi+6]=np.r_[b.get('v0',[0.,0.,0.]),b.get('w0',[0.,0.,0.])]
    for state in (A,B):
        state.joint_q.assign(q);state.joint_qd.assign(velocity);sap_eval_fk(model,state.joint_q,state.joint_qd,state)
    for b in scene['bodies']:
        if not np.allclose(A.body_qd.numpy()[ids[b['name']]],np.r_[b.get('v0',[0.,0.,0.]),b.get('w0',[0.,0.,0.])],atol=1e-5):raise RuntimeError('SAP graph initial world twist mismatch')
    cm=loaded.collision_model;shape_body=cm.shape_body.numpy();exclude=set()
    for side in ('left','right'):
        if side+'_link7' not in ids:continue
        exclude.update(tuple(sorted((int(a),int(b)))) for a in np.flatnonzero(shape_body==ids[side+'_link7']) for b in np.flatnonzero(shape_body==ids[side+'_link8']))
    pairs=[p for p in cm.shape_contact_pairs.numpy() if tuple(sorted(map(int,p))) not in exclude]
    cm=replace(cm,shape_collision_filter_pairs=cm.shape_collision_filter_pairs|exclude,
               shape_contact_pairs=wp.array(np.asarray(pairs,np.int32).reshape(-1,2),dtype=wp.vec2i,device=device),shape_contact_pair_count=len(pairs))
    solver=SolverSAP(model,nconmax=cap,collect_iteration_stats=False,check_line_search_errors=False)
    pipe=SapCollisionPipeline(cm,naconmax=cap,broad_phase='explicit');con=pipe.contacts()
    driven=[b for b in scene['bodies'] if b['driven']];tg=np.stack([np.c_[target[b['name']+'__pose'],target[b['name']+'__velocity']] for b in driven],axis=1)
    prm=[]
    for b in driven:
        a=scene['assets'][b['asset']];d=scene['drive'];prm.append([d['Kt'],2*np.sqrt(a['mass']*d['Kt']),d['force_cap_N'],d['Kr'],
            2*np.sqrt(d['Kr']*np.linalg.eigvalsh(a['inertia']).max()),d['torque_cap_Nm'],a['mass']*9.81])
    tg=wp.array(tg.astype(np.float32),dtype=wp.float32,device=device);prm=wp.array(np.asarray(prm,np.float32),dtype=wp.float32,device=device)
    driven_ids=wp.array([ids[b['name']] for b in driven],dtype=wp.int32,device=device)
    all_ids=wp.array([ids[b['name']] for b in scene['bodies']],dtype=wp.int32,device=device)
    counter=wp.zeros(1,dtype=wp.int32,device=device);bad=wp.zeros(1,dtype=wp.int32,device=device)
    values=wp.zeros((n//10+1,len(scene['bodies']),13),dtype=wp.float32,device=device)
    stats=wp.zeros(2,dtype=wp.int32,device=device);minimum=wp.zeros(1,dtype=wp.float32,device=device)
    def one(s0,s1):
        s0.clear_forces();wp.launch(drive,len(driven),inputs=[counter,tg,driven_ids,prm,s0.body_q,s0.body_qd,s0.body_f],device=device)
        pipe.collide(sap_collision_state_from_state(s0),con);solver.step(s0,s1,control,con,scene['dt'])
        cj=solver.contact_jacobian;wp.launch(contacts_health,cap,inputs=[cj.contact_env_count_wp,cj.contact_env_phi0_wp,stats,minimum],device=device)
        wp.launch(tick,1,inputs=[counter],device=device);wp.launch(record,len(scene['bodies']),inputs=[counter,all_ids,s1.body_q,s1.body_qd,values,bad],device=device)
    wp.launch(record,len(scene['bodies']),inputs=[counter,all_ids,A.body_q,A.body_qd,values,bad],device=device)
    start=time.perf_counter();one(A,B);one(B,A)
    with wp.ScopedCapture(device=device) as capture:one(A,B);one(B,A)
    for k in range((n-2)//2):wp.capture_launch(capture.graph)
    wp.synchronize_device(device);wall=time.perf_counter()-start;overflow=None
    try:solver.check_buffers()
    except RuntimeError as e:overflow=str(e)
    v=values.numpy().astype(float);trace={'time':np.arange(n//10+1)*scene['dt']*10}
    for i,b in enumerate(scene['bodies']):trace[b['name']+'__pose']=v[:,i,:7];trace[b['name']+'__velocity']=v[:,i,7:]
    native={k:getattr(cm,k).numpy().tolist() for k in ('shape_material_ke','shape_material_tau','shape_material_mu','shape_margin','shape_gap')}
    return trace,{'engine':'SAP Warp','execution':'outer CUDA graph; same controller and numerical settings',
                  'diagnostics_only_overrides':{'collect_iteration_stats':False,'check_line_search_errors':False},
                  'graph_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'warp_version':wp.__version__,'device':wp.get_device(device).name,
                  'steps':int(counter.numpy()[0]),'expected_steps':n,'finite':bool(not bad.numpy()[0] and np.isfinite(v).all()),'wall_s':wall,
                  'max_contacts':int(stats.numpy()[0]),'contact_steps':int(stats.numpy()[1]),'truncated':int(overflow is not None),
                  'overflow':overflow,'engine_peak_overlap_m':float(-minimum.numpy()[0]),'native':native}
