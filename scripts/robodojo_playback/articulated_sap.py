"""Native SAP integration of the physical X5 gripper articulations."""
import hashlib
import json
import os
from pathlib import Path
import time
import numpy as np
import warp as wp
from .sap_graph import drive,record,tick,contacts_health


@wp.kernel
def fingers(counter:wp.array(dtype=wp.int32),commands:wp.array2d(dtype=wp.float32),indices:wp.array2d(dtype=wp.int32),params:wp.array2d(dtype=wp.float32),
            q:wp.array(dtype=wp.float32),v:wp.array(dtype=wp.float32),joint_f:wp.array(dtype=wp.float32),body_f:wp.array(dtype=wp.spatial_vectorf)):
    i=wp.tid();qi=indices[i,0];vi=indices[i,1];body=indices[i,2];side=indices[i,3];k=counter[0]
    force=wp.clamp(params[i,0]*(commands[k,side]-q[qi])-params[i,1]*v[vi],-params[i,2],params[i,2])
    joint_f[vi]=force;body_f[body]=wp.spatial_vector(0.0,0.0,params[i,3],0.0,0.0,0.0)


def config(scene,objects):
    bodies,joints,arts=[],[],[]
    def tf(p,q):return {'p':list(p),'q':[q[1],q[2],q[3],q[0]]}
    par=scene.get('sap_parameters',{})
    material={'mu':scene['mu'],'mu_torsional':0.,'mu_rolling':0.,'density':0.}
    if 'pair_ke' in par:material['ke']=2.*par['pair_ke']
    if 'pair_tau' in par:material['tau']=.5*par['pair_tau']
    hand_joints={}
    for b in scene['bodies']:
        a=scene['assets'][b['asset']];entry={'id':b['name'],'transform':tf(b['p'],b['q']),'mass':a['mass'],'inertia':a['inertia']}
        if b['asset']!='base':entry['shapes']=[{'type':'convex_mesh','label':b['name']+f'_{i}','path':p,'is_solid':True,'cfg':material} for i,p in enumerate(a['pieces'])]
        bodies.append(entry);j=b['name']+'_joint'
        if b.get('parent'):
            joints.append({'id':j,'type':'prismatic','parent':b['parent'],'child':b['name'],'parent_xform':{'p':b['zero_p'],'q':'identity'},
                'axis':b['axis'],'limit_lower':b['joint_lower'],'limit_upper':b['joint_upper'],'target_ke':0.,'target_kd':0.})
            hand_joints[b['parent']].append(j)
        else:
            joints.append({'id':j,'type':'free','parent':'world','child':b['name']})
            if b['asset']=='base':hand_joints[b['name']]=[j]
            else:arts.append({'id':b['name']+'_art','joints':[j]})
    arts.extend({'id':n+'_art','joints':j} for n,j in hand_joints.items())
    if objects:
        for sb in scene.get('static_bodies',[]):
            bodies.append({'id':sb['name'],'transform':tf(sb['p'],sb['q']),'shapes':[{'type':'convex_mesh','label':sb['name']+f'_{i}','path':p,'is_solid':True,'cfg':material} for i,p in enumerate(scene['assets'][sb['asset']]['pieces'])]})
    if objects:bodies.append({'id':'table','transform':tf([0.,-.05,.74],[1.,0.,0.,0.]),'shapes':[{'type':'box','label':'table_0','hx':.7,'hy':.55,'hz':.025,'cfg':material}]})
    builder={'gravity':-9.81}
    if 'gap' in par:builder['rigid_gap']=par['gap']
    return {'schema_version':1,'name':'robodojo_articulated_gripper_pilot','simulation':{'dt':scene['dt'],'num_worlds':1,'nconmax':scene['contact_capacity']},
            'builder':builder,'ground':{'enabled':False},'bodies':bodies,'joints':joints,'articulations':arts}


def run(scene,target,out,objects):
    setup_started=time.perf_counter()
    from sap_warp import SolverSAP,load_sap_scene
    from sap_warp.collision.pipeline import SapCollisionPipeline
    from sap_warp.resources.collision_model import sap_collision_state_from_state
    from sap_warp.sap_kinematics import sap_eval_fk
    from sap_warp.collision import core as collision_core
    import sap_warp
    if os.environ.get('ROBODOJO_WARP_CACHE'):wp.config.kernel_cache_dir=os.environ['ROBODOJO_WARP_CACHE']
    wp.init();device='cuda:0';n=len(target['time'])-1;cap=scene['contact_capacity']
    if n%10:raise ValueError('SAP trace requires step count divisible by 10')
    execution=os.environ.get('ROBODOJO_SAP_EXECUTION','graph')
    if execution not in ('graph','host'):raise ValueError('ROBODOJO_SAP_EXECUTION must be graph or host')
    path=out/'sap_scene.json';path.write_text(json.dumps(config(scene,objects),indent=2));loaded=load_sap_scene(path,device=device,naconmax=cap,strict=True)
    model,A,control=loaded.sap_model,loaded.sap_state,loaded.sap_control;B=model.state();labels=list(model.body_label);jl=list(model.joint_label)
    ids={b['name']:labels.index(b['name']) for b in scene['bodies']};qs=model.joint_q_start.numpy();vs=model.joint_qd_start.numpy();q=A.joint_q.numpy();v_initial=A.joint_qd.numpy();v_initial[:]=0
    for b in scene['bodies']:
        offset=qs[jl.index(b['name']+'_joint')]
        if b.get('parent'):q[offset]=b['joint_initial']
        else:
            q[offset:offset+7]=np.r_[b['p'],np.asarray(b['q'])[[1,2,3,0]]]
            vi=vs[jl.index(b['name']+'_joint')];v_initial[vi:vi+6]=np.r_[b.get('v0',[0.,0.,0.]),b.get('w0',[0.,0.,0.])]
    for state in (A,B):state.joint_q.assign(q);state.joint_qd.assign(v_initial);sap_eval_fk(model,state.joint_q,state.joint_qd,state)
    kwargs={'nconmax':cap,'collect_iteration_stats':False,'check_line_search_errors':False}
    if 'sigma' in scene.get('sap_parameters',{}):kwargs['contact_sigma']=scene['sap_parameters']['sigma']
    kwargs.update(scene.get('sap_parameters',{}).get('solver_kwargs',{}))
    solver=SolverSAP(model,**kwargs)
    cm=loaded.collision_model
    if scene.get('explicit_finger_self_collision_exclusions'):
        from dataclasses import replace
        shape_body=cm.shape_body.numpy();excluded=set()
        for side in ('left','right'):
            a=np.flatnonzero(shape_body==ids[side+'_link7']);b=np.flatnonzero(shape_body==ids[side+'_link8'])
            excluded.update(tuple(sorted((int(i),int(j)))) for i in a for j in b)
        cm.shape_collision_filter_pairs.update(excluded)
        pairs=[p for p in cm.shape_contact_pairs.numpy() if tuple(sorted(map(int,p))) not in excluded]
        cm=replace(cm,shape_contact_pairs=wp.array(np.asarray(pairs,dtype=np.int32).reshape(-1,2),dtype=wp.vec2i,device=cm.device),shape_contact_pair_count=len(pairs))
    pipe=SapCollisionPipeline(cm,naconmax=cap,broad_phase='explicit');con=pipe.contacts()
    bases=[b for b in scene['bodies'] if b['driven']];tg=np.stack([np.c_[target[b['name']+'__pose'],target[b['name']+'__velocity']] for b in bases],axis=1)
    params=[]
    for b in bases:
        a=scene['assets'][b['asset']];d=scene['drive'];params.append([d['Kt'],2*np.sqrt(a['mass']*d['Kt']),d['force_cap_N'],d['Kr'],
            2*np.sqrt(d['Kr']*np.linalg.eigvalsh(a['inertia']).max()),d['torque_cap_Nm'],a['mass']*9.81])
    tg=wp.array(tg.astype(np.float32),dtype=wp.float32,device=device);params=wp.array(np.asarray(params,np.float32),dtype=wp.float32,device=device)
    bi=wp.array([ids[b['name']] for b in bases],dtype=wp.int32,device=device);all_ids=wp.array([ids[b['name']] for b in scene['bodies']],dtype=wp.int32,device=device)
    fi=[];fp=[];a=scene['finger_actuator']
    for b in scene['bodies']:
        if b.get('parent'):
            j=jl.index(b['name']+'_joint');fi.append([qs[j],vs[j],ids[b['name']],0 if b['name'].startswith('left') else 1])
            fp.append([a['Kp'],a['Kd'],a['force_cap_N'],scene['assets'][b['asset']]['mass']*9.81])
    fi=wp.array(np.asarray(fi,np.int32),dtype=wp.int32,device=device);fp=wp.array(np.asarray(fp,np.float32),dtype=wp.float32,device=device)
    commands=wp.array(np.c_[target['command_left'],target['command_right']].astype(np.float32),dtype=wp.float32,device=device)
    counter=wp.zeros(1,dtype=wp.int32,device=device);bad=wp.zeros(1,dtype=wp.int32,device=device);stats=wp.zeros(2,dtype=wp.int32,device=device);minimum=wp.zeros(1,dtype=wp.float32,device=device)
    values=wp.zeros((n//10+1,len(scene['bodies']),13),dtype=wp.float32,device=device)
    telemetry=None
    if os.environ.get('ROBODOJO_STACK_EVAL')=='1':
        if scene.get('telemetry_nodes'):
            from .pair_telemetry import sap_telemetry
            telemetry=sap_telemetry(scene,target,out,model,loaded,pipe,con,solver,ids,qs,vs,jl,device)
        else:
            from .stack_telemetry import sap_telemetry
            telemetry=sap_telemetry(scene,target,out,model,cm,pipe,con,solver,ids,qs,vs,jl,device)
        telemetry.state(counter,A,0)
    def one(s0,s1):
        s0.clear_forces();wp.launch(drive,len(bases),inputs=[counter,tg,bi,params,s0.body_q,s0.body_qd,s0.body_f],device=device)
        wp.launch(fingers,len(fi),inputs=[counter,commands,fi,fp,s0.joint_q,s0.joint_qd,control.joint_f,s0.body_f],device=device)
        pipe.collide(sap_collision_state_from_state(s0),con);solver.step(s0,s1,control,con,scene['dt'])
        cj=solver.contact_jacobian;wp.launch(contacts_health,cap,inputs=[cj.contact_env_count_wp,cj.contact_env_phi0_wp,stats,minimum],device=device)
        if telemetry is not None:telemetry.contacts(counter);telemetry.state(counter,s1,1)
        wp.launch(tick,1,inputs=[counter],device=device);wp.launch(record,len(scene['bodies']),inputs=[counter,all_ids,s1.body_q,s1.body_qd,values,bad],device=device)
    wp.launch(record,len(scene['bodies']),inputs=[counter,all_ids,A.body_q,A.body_qd,values,bad],device=device)
    wp.synchronize_device(device);setup_s=time.perf_counter()-setup_started
    start=time.perf_counter()
    benchmark=None
    if execution=='host':
        for _ in range(n//2):one(A,B);one(B,A)
    else:
        one(A,B);one(B,A)
        wp.synchronize_device(device)
        with wp.ScopedCapture(device=device) as capture:one(A,B);one(B,A)
        wp.synchronize_device(device);jit_capture_s=time.perf_counter()-start
        repeats=int(os.environ.get('ROBODOJO_BENCHMARK_REPEATS','0'))
        if repeats:
            if telemetry is not None:raise ValueError('benchmark requires the common 10-ms observer, not additional telemetry')
            def reset():
                for state in (A,B):
                    state.joint_q.assign(q);state.joint_qd.assign(v_initial);sap_eval_fk(model,state.joint_q,state.joint_qd,state)
                control.joint_f.zero_();solver.reset_runtime_state();counter.zero_();bad.zero_();stats.zero_();minimum.zero_();values.zero_()
                wp.launch(record,len(scene['bodies']),inputs=[counter,all_ids,A.body_q,A.body_qd,values,bad],device=device)
            def check():
                overflow=False
                try:solver.check_buffers();pipe.check_buffers(con)
                except RuntimeError:overflow=True
                return {'finite':bool(not bad.numpy()[0]),'overflow':overflow,'steps':int(counter.numpy()[0]),'expected_steps':n,'max_contacts':int(stats.numpy()[0])}
            from .gpu_benchmark import graph_rollouts
            benchmark=graph_rollouts(wp,device,capture.graph,n//2,reset,check,repeats)
            benchmark.update(setup_s=setup_s,jit_and_capture_s=jit_capture_s,steps=n,simulated_duration_s=n*scene['dt'],real_time_factor=n*scene['dt']/benchmark['median_wall_s'])
        else:
            for _ in range((n-2)//2):wp.capture_launch(capture.graph)
    wp.synchronize_device(device);wall=time.perf_counter()-start;overflow=None
    try:solver.check_buffers()
    except RuntimeError as e:overflow=str(e)
    if telemetry is not None:telemetry.finish()
    values=values.numpy().astype(float);trace={'time':np.arange(n//10+1)*scene['dt']*10}
    for i,b in enumerate(scene['bodies']):trace[b['name']+'__pose']=values[:,i,:7];trace[b['name']+'__velocity']=values[:,i,7:]
    return trace,{'engine':'SAP Warp articulated grippers','warp_version':wp.__version__,'device':wp.get_device(device).name,
        'execution':execution,'sap_module_path':str(Path(sap_warp.__file__).resolve()),
        'convex_backend':getattr(collision_core,'SAP_CONVEX_BACKEND','legacy_unversioned'),
        'collision_core_sha256':hashlib.sha256(Path(collision_core.__file__).read_bytes()).hexdigest(),
        'steps':int(counter.numpy()[0]),'expected_steps':n,'finite':bool(not bad.numpy()[0] and np.isfinite(values).all()),'wall_s':benchmark['median_wall_s'] if benchmark else wall,
        'benchmark':benchmark,
        'single_run_timing':{'setup_s':setup_s,'first_two_steps_jit_and_capture_s':jit_capture_s,'remaining_steps':n-2,'remaining_steps_wall_s':wall-jit_capture_s} if benchmark is None and execution=='graph' else None,
        'truncated':int(overflow is not None),'overflow':overflow,'max_contacts':int(stats.numpy()[0]),'engine_peak_overlap_m':float(-minimum.numpy()[0]),
        'native':{k:getattr(cm,k).numpy().tolist() for k in ('shape_material_ke','shape_material_tau','shape_material_mu','shape_margin','shape_gap')},
        'joint_metadata':{'labels':jl,'q_start':qs.tolist(),'qd_start':vs.tolist(),
                          'limit_lower':model.joint_limit_lower.numpy().tolist(),'limit_upper':model.joint_limit_upper.numpy().tolist()},
        'source_control_audit':scene['source_control_audit'],'graph_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
