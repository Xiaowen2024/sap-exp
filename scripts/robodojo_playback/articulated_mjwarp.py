"""MuJoCo 4.0 Warp playback of the unchanged articulated-gripper MJCF.

The controller and state observer use qpos/qvel, as the CPU adapter does;
derived xpos can lag integration. All per-step input and logging stay on GPU.
"""
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import time
import numpy as np
import warp as wp
from scipy.spatial.transform import Rotation


@wp.kernel
def controller(counter:wp.array(dtype=wp.int32),target:wp.array3d(dtype=wp.float32),base_slots:wp.array2d(dtype=wp.int32),
               params:wp.array2d(dtype=wp.float32),finger_slots:wp.array2d(dtype=wp.int32),finger_params:wp.array2d(dtype=wp.float32),
               commands:wp.array2d(dtype=wp.float32),qp:wp.array2d(dtype=wp.float32),qv:wp.array2d(dtype=wp.float32),
               xf:wp.array2d(dtype=wp.spatial_vectorf),qf:wp.array2d(dtype=wp.float32)):
    i=wp.tid();k=counter[0]
    if i<2:
        qi=base_slots[i,0];vi=base_slots[i,1];body=base_slots[i,2]
        p=wp.vec3(qp[0,qi],qp[0,qi+1],qp[0,qi+2]);q=wp.quat(qp[0,qi+4],qp[0,qi+5],qp[0,qi+6],qp[0,qi+3])
        v=wp.vec3(qv[0,vi],qv[0,vi+1],qv[0,vi+2]);w=wp.quat_rotate(q,wp.vec3(qv[0,vi+3],qv[0,vi+4],qv[0,vi+5]))
        tp=wp.vec3(target[k,i,0],target[k,i,1],target[k,i,2]);tq=wp.quat(target[k,i,4],target[k,i,5],target[k,i,6],target[k,i,3])
        tv=wp.vec3(target[k,i,7],target[k,i,8],target[k,i,9]);tw=wp.vec3(target[k,i,10],target[k,i,11],target[k,i,12])
        f=params[i,0]*(tp-p)+params[i,1]*(tv-v);nf=wp.length(f)
        if nf>params[i,2]:f=f*(params[i,2]/nf)
        f=f+wp.vec3(0.0,0.0,params[i,6])
        qe=tq*wp.quat_inverse(q)
        if qe[3]<0.0:qe=wp.quat(-qe[0],-qe[1],-qe[2],-qe[3])
        xyz=wp.vec3(qe[0],qe[1],qe[2]);length=wp.length(xyz);er=2.0*xyz
        if length>1.0e-9:er=xyz*(2.0*wp.atan2(length,qe[3])/length)
        tau=params[i,3]*er+params[i,4]*(tw-w);nt=wp.length(tau)
        if nt>params[i,5]:tau=tau*(params[i,5]/nt)
        xf[0,body]=wp.spatial_vector(f,tau)
    if i<finger_slots.shape[0]:
        qi=finger_slots[i,0];vi=finger_slots[i,1];body=finger_slots[i,2];side=finger_slots[i,3]
        force=wp.clamp(finger_params[i,0]*(commands[k,side]-qp[0,qi])-finger_params[i,1]*qv[0,vi],-finger_params[i,2],finger_params[i,2])
        qf[0,vi]=force;xf[0,body]=wp.spatial_vector(0.0,0.0,finger_params[i,3],0.0,0.0,0.0)


@wp.kernel
def observe(counter:wp.array(dtype=wp.int32),offset:int,stride:int,slots:wp.array2d(dtype=wp.int32),local:wp.array2d(dtype=wp.float32),
            qp:wp.array2d(dtype=wp.float32),qv:wp.array2d(dtype=wp.float32),values:wp.array3d(dtype=wp.float32),bad:wp.array(dtype=wp.int32)):
    i=wp.tid();k=counter[0]+offset;qi=slots[i,0];vi=slots[i,1];pqi=slots[i,2];pvi=slots[i,3]
    base_qi=qi;base_vi=vi
    if pqi>=0:base_qi=pqi;base_vi=pvi
    p=wp.vec3(qp[0,base_qi],qp[0,base_qi+1],qp[0,base_qi+2]);q=wp.quat(qp[0,base_qi+4],qp[0,base_qi+5],qp[0,base_qi+6],qp[0,base_qi+3])
    v=wp.vec3(qv[0,base_vi],qv[0,base_vi+1],qv[0,base_vi+2]);w=wp.quat_rotate(q,wp.vec3(qv[0,base_vi+3],qv[0,base_vi+4],qv[0,base_vi+5]))
    if pqi>=0:
        zero=wp.vec3(local[i,0],local[i,1],local[i,2]);axis=wp.vec3(local[i,3],local[i,4],local[i,5]);r=wp.quat_rotate(q,zero+axis*qp[0,qi])
        p=p+r;v=v+wp.cross(w,r)+wp.quat_rotate(q,axis)*qv[0,vi]
    if not wp.isfinite(p[0]) or not wp.isfinite(p[1]) or not wp.isfinite(p[2]) or not wp.isfinite(q[0]) or not wp.isfinite(q[1]) or not wp.isfinite(q[2]) or not wp.isfinite(q[3]):wp.atomic_max(bad,0,1)
    for j in range(3):
        if not wp.isfinite(v[j]) or not wp.isfinite(w[j]):wp.atomic_max(bad,0,1)
    if k%stride==0:
        row=k//stride
        values[row,i,0]=p[0];values[row,i,1]=p[1];values[row,i,2]=p[2]
        values[row,i,3]=q[3];values[row,i,4]=q[0];values[row,i,5]=q[1];values[row,i,6]=q[2]
        for j in range(3):values[row,i,7+j]=v[j];values[row,i,10+j]=w[j]


@wp.kernel
def contact_health(nacon:wp.array(dtype=wp.int32),nefc:wp.array(dtype=wp.int32),dist:wp.array(dtype=wp.float32),
                   stats:wp.array(dtype=wp.int32),minimum:wp.array(dtype=wp.float32),cap:int,njmax:int):
    i=wp.tid();count=nacon[0]
    if i==0:
        wp.atomic_max(stats,0,count);wp.atomic_max(stats,2,nefc[0])
        if count>0:wp.atomic_add(stats,1,1)
        if count>cap or nefc[0]>njmax:wp.atomic_max(stats,3,1)
    if i<count:wp.atomic_min(minimum,0,dist[i])


@wp.kernel
def tick(counter:wp.array(dtype=wp.int32)):
    counter[0]=counter[0]+1


def run(scene,target,out,objects):
    from .articulated import xml
    import mujoco as mj
    import mujoco_warp as mjw
    start_setup=time.perf_counter()
    if os.environ.get('ROBODOJO_WARP_CACHE'):wp.config.kernel_cache_dir=os.environ['ROBODOJO_WARP_CACHE']
    wp.init();device='cuda:0';n=len(target['time'])-1;cap=scene['contact_capacity'];njmax=4096
    if n%10:raise ValueError('trace requires a step count divisible by 10')
    text=xml(scene,objects);(out/'scene.xml').write_text(text);m=mj.MjModel.from_xml_string(text);cpu=mj.MjData(m)
    ids={b['name']:mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,b['name']) for b in scene['bodies']};slots={}
    for b in scene['bodies']:
        j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,b['name']+'_joint');slots[b['name']]=(int(m.jnt_qposadr[j]),int(m.jnt_dofadr[j]))
        if b.get('parent'):cpu.qpos[slots[b['name']][0]]=b['joint_initial']
        else:
            vi=slots[b['name']][1];cpu.qvel[vi:vi+3]=b.get('v0',[0.,0.,0.]);cpu.qvel[vi+3:vi+6]=Rotation.from_quat(np.asarray(b['q'])[[1,2,3,0]]).inv().apply(b.get('w0',[0.,0.,0.]))
    mj.mj_forward(m,cpu);q_initial=cpu.qpos.copy()[None,:].astype(np.float32);v_initial=cpu.qvel.copy()[None,:].astype(np.float32)
    gm=mjw.put_model(m);d=mjw.put_data(m,cpu,nworld=1,nconmax=cap,njmax=njmax)
    bases=[b for b in scene['bodies'] if b['driven']];fingers=[b for b in scene['bodies'] if b.get('parent')]
    if len(bases)!=2:raise ValueError('frozen bimanual controller expects two bases')
    tg=np.stack([np.c_[target[b['name']+'__pose'],target[b['name']+'__velocity']] for b in bases],axis=1);params=[]
    for b in bases:
        a=scene['assets'][b['asset']];x=scene['drive'];params.append([x['Kt'],2*np.sqrt(a['mass']*x['Kt']),x['force_cap_N'],x['Kr'],2*np.sqrt(x['Kr']*np.linalg.eigvalsh(a['inertia']).max()),x['torque_cap_Nm'],a['mass']*9.81])
    def array(x,dtype=wp.float32):return wp.array(np.asarray(x),dtype=dtype,device=device)
    tg=array(tg.astype(np.float32));params=array(np.asarray(params,np.float32));base_slots=array([(*slots[b['name']],ids[b['name']]) for b in bases],wp.int32)
    fi=array([(*slots[b['name']],ids[b['name']],0 if b['name'].startswith('left') else 1) for b in fingers],wp.int32)
    a=scene['finger_actuator'];fp=array([[a['Kp'],a['Kd'],a['force_cap_N'],scene['assets'][b['asset']]['mass']*9.81] for b in fingers])
    commands=array(np.c_[target['command_left'],target['command_right']].astype(np.float32))
    oslots=[];local=[]
    for b in scene['bodies']:
        parent=slots[b['parent']] if b.get('parent') else (-1,-1);oslots.append([*slots[b['name']],*parent]);local.append([*b.get('zero_p',[0.,0.,0.]),*b.get('axis',[0.,0.,0.])])
    oslots=array(oslots,wp.int32);local=array(local);counter=wp.zeros(1,dtype=wp.int32,device=device)
    bad=wp.zeros(1,dtype=wp.int32,device=device);stats=wp.zeros(4,dtype=wp.int32,device=device);minimum=wp.zeros(1,dtype=wp.float32,device=device)
    telemetry=None;stride=10
    if os.environ.get('ROBODOJO_STACK_EVAL')=='1' and scene.get('telemetry_nodes'):
        from .pair_telemetry import mjwarp_telemetry
        telemetry=mjwarp_telemetry(scene,target,out,m,gm,d,device);stride=1
    values=wp.zeros((n//stride+1,len(scene['bodies']),13),dtype=wp.float32,device=device)
    def record(offset):wp.launch(observe,len(scene['bodies']),inputs=[counter,offset,stride,oslots,local,d.qpos,d.qvel,values,bad],device=device)
    def one():
        d.xfrc_applied.zero_();d.qfrc_applied.zero_()
        wp.launch(controller,max(2,len(fingers)),inputs=[counter,tg,base_slots,params,fi,fp,commands,d.qpos,d.qvel,d.xfrc_applied,d.qfrc_applied],device=device)
        mjw.step(gm,d)
        wp.launch(contact_health,cap,inputs=[d.nacon,d.nefc,d.contact.dist,stats,minimum,cap,njmax],device=device)
        if telemetry is not None:telemetry.contacts(counter)
        record(1);wp.launch(tick,1,inputs=[counter],device=device)
    def reset():
        mjw.reset_data(gm,d);d.qpos.assign(q_initial);d.qvel.assign(v_initial);mjw.forward(gm,d)
        counter.zero_();bad.zero_();stats.zero_();minimum.zero_();values.zero_();record(0)
    reset();wp.synchronize_device(device);setup_s=time.perf_counter()-start_setup
    start=time.perf_counter();one();one();wp.synchronize_device(device)
    with wp.ScopedCapture(device=device) as capture:one();one()
    wp.synchronize_device(device);jit_capture_s=time.perf_counter()-start
    def check():
        st=stats.numpy();return {'finite':bool(not bad.numpy()[0]),'overflow':bool(st[3]),'steps':int(counter.numpy()[0]),'expected_steps':n,'max_contacts':int(st[0]),'max_constraints':int(st[2])}
    bench=None
    if os.environ.get('ROBODOJO_SINGLE_ROLLOUT')=='1':
        for _ in range((n-2)//2):wp.capture_launch(capture.graph)
        wp.synchronize_device(device);wall=time.perf_counter()-start
    else:
        from .gpu_benchmark import graph_rollouts
        repeats=int(os.environ.get('ROBODOJO_BENCHMARK_REPEATS','1'));bench=graph_rollouts(wp,device,capture.graph,n//2,reset,check,repeats)
        bench.update(setup_s=setup_s,jit_and_capture_s=jit_capture_s,steps=n,simulated_duration_s=n*scene['dt'],real_time_factor=n*scene['dt']/bench['median_wall_s']);wall=bench['median_wall_s']
    v=values.numpy().astype(float)
    if telemetry is not None:
        dense={'time':target['time']}
        for i,b in enumerate(scene['bodies']):dense[b['name']+'__pose']=v[:,i,:7];dense[b['name']+'__velocity']=v[:,i,7:]
        np.savez_compressed(out/'dense_trace.npz',**dense);telemetry.finish()
    v=v[::10//stride];trace={'time':np.arange(n//10+1)*scene['dt']*10}
    for i,b in enumerate(scene['bodies']):trace[b['name']+'__pose']=v[:,i,:7];trace[b['name']+'__velocity']=v[:,i,7:]
    row=check()
    health={'engine':'MuJoCo Warp articulated grippers','version':mj.__version__,'mujoco_warp_version':importlib.metadata.version('mujoco-warp'),
        'warp_version':wp.__version__,'device':wp.get_device(device).name,'execution':'two-step CUDA graph; device controller and logging',
        'finite':bool(row['finite'] and np.isfinite(v).all()),'steps':row['steps'],'expected_steps':n,'wall_s':wall,
        'max_contacts':row['max_contacts'],'max_constraints':row['max_constraints'],'truncated':int(row['overflow']),'engine_peak_overlap_m':float(-minimum.numpy()[0]),
        'benchmark':bench,'single_run_timing':{'setup_s':setup_s,'first_two_steps_jit_and_capture_s':jit_capture_s,'remaining_steps':n-2,'remaining_steps_wall_s':wall-jit_capture_s} if bench is None else None,
        'gpu_data_dtype':str(d.qpos.dtype),'native':{'integrator':int(m.opt.integrator),'solver':int(m.opt.solver),
            'iterations':int(m.opt.iterations),'tolerance':float(m.opt.tolerance),'timestep':float(m.opt.timestep),
            'geom_solref':m.geom_solref.tolist(),'geom_solimp':m.geom_solimp.tolist(),'jnt_range':m.jnt_range.tolist(),
            'nconmax':cap,'njmax':njmax,'nv':m.nv,'nbody':m.nbody,'ngeom':m.ngeom},
        'source_control_audit':scene['source_control_audit'],'adapter_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    return trace,health
