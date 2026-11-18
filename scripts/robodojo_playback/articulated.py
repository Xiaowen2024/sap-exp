"""Detached X5 gripper articulations with the official command-driven PD law."""
import json
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from .common import rotation,wrench
from .run import nums,trace_buffer,sample,finish


def motor(scene,command,q,v):
    a=scene['finger_actuator'];return float(np.clip(a['Kp']*(command-q)-a['Kd']*v,-a['force_cap_N'],a['force_cap_N']))


def xml(scene,objects):
    root=ET.Element('mujoco',model='robodojo_articulated_gripper_pilot')
    ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false')
    par=scene.get('mujoco_parameters',{})
    option=ET.SubElement(root,'option',timestep=str(scene['dt']),gravity=nums(scene['gravity']))
    for key in ('impratio','noslip_iterations','cone','integrator','iterations','tolerance'):
        if key in par:option.set(key,str(par[key]))
    if par.get('flags'):ET.SubElement(option,'flag',**{k:str(v) for k,v in par['flags'].items()})
    aset=ET.SubElement(root,'asset');wb=ET.SubElement(root,'worldbody')
    for kind,a in scene['assets'].items():
        for i,p in enumerate(a['pieces']):ET.SubElement(aset,'mesh',name=f'{kind}_{i}',file=p)
    if objects:ET.SubElement(wb,'geom',name='table',type='box',pos='0 -.05 .74',size='.7 .55 .025',friction=nums([scene['mu'],0,0]),condim='3')
    if objects:
        for sb in scene.get('static_bodies',[]):
            for i in range(len(scene['assets'][sb['asset']]['pieces'])):
                ET.SubElement(wb,'geom',name=f"{sb['name']}_{i}",type='mesh',mesh=f"{sb['asset']}_{i}",pos=nums(sb['p']),quat=nums(sb['q']),friction=nums([scene['mu'],0,0]),condim='3')
    nodes={}
    for b in scene['bodies']:
        a=scene['assets'][b['asset']];parent=nodes[b['parent']] if b.get('parent') else wb
        body=ET.SubElement(parent,'body',name=b['name'],pos=nums(b['zero_p'] if b.get('parent') else b['p']),quat='1 0 0 0' if b.get('parent') else nums(b['q']))
        nodes[b['name']]=body;I=np.asarray(a['inertia'])
        if b.get('parent'):ET.SubElement(body,'joint',name=b['name']+'_joint',type='slide',axis=nums(b['axis']),limited='true',range=nums([b['joint_lower'],b['joint_upper']]),armature='0',damping='0')
        else:ET.SubElement(body,'freejoint',name=b['name']+'_joint')
        ET.SubElement(body,'inertial',pos='0 0 0',mass=str(a['mass']),fullinertia=nums([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
        if b['asset']!='base':
            for i in range(len(a['pieces'])):ET.SubElement(body,'geom',type='mesh',mesh=f'{b["asset"]}_{i}',friction=nums([scene['mu'],0,0]),condim='3')
    for geom in wb.iter('geom'):
        for key in ('solref','solimp'):
            if key in par:geom.set(key,nums(par[key]))
    if 'joint_solref' in par:
        for joint in wb.iter('joint'):joint.set('solreflimit',nums(par['joint_solref']))
    if scene.get('explicit_finger_self_collision_exclusions'):
        contact=ET.SubElement(root,'contact')
        for side in ('left','right'):ET.SubElement(contact,'exclude',body1=side+'_link7',body2=side+'_link8')
    return ET.tostring(root,encoding='unicode')


def run_mujoco(scene,target,out,objects):
    import mujoco as mj
    text=xml(scene,objects);(out/'scene.xml').write_text(text);m=mj.MjModel.from_xml_string(text);d=mj.MjData(m)
    ids={b['name']:mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,b['name']) for b in scene['bodies']}
    slots={}
    for b in scene['bodies']:
        j=mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,b['name']+'_joint');slots[b['name']]=(int(m.jnt_qposadr[j]),int(m.jnt_dofadr[j]))
        if b.get('parent'):d.qpos[slots[b['name']][0]]=b['joint_initial']
    mj.mj_forward(m,d)
    def states():
        st={}
        for b in scene['bodies']:
            qi,vi=slots[b['name']]
            if not b.get('parent'):
                q=d.qpos[qi+3:qi+7].copy();st[b['name']]=(d.qpos[qi:qi+3].copy(),q,d.qvel[vi:vi+3].copy(),rotation(q).apply(d.qvel[vi+3:vi+6]))
            else:
                p,q,v,w=st[b['parent']];rot=rotation(q);axis=rot.apply(b['axis']);r=rot.apply(np.asarray(b['zero_p'])+np.asarray(b['axis'])*d.qpos[qi])
                st[b['name']]=(p+r,q.copy(),v+np.cross(w,r)+axis*d.qvel[vi],w.copy())
        return st
    telemetry=None
    if os.environ.get('ROBODOJO_STACK_EVAL')=='1':
        from .stack_telemetry import MuJoCoTelemetry
        telemetry=MuJoCoTelemetry(scene,target,out,m,d,ids,slots)
    trace=trace_buffer(scene);n=len(target['time'])-1;start=time.perf_counter();maxcon=0;peak=0.;motor_max=0.;native_error=0.
    for k in range(n+1):
        st=states()
        if telemetry is not None:telemetry.state(k,st)
        if k%10==0 or k==n:
            mj.mj_kinematics(m,d)
            native_error=max(native_error,max(np.linalg.norm(st[name][0]-d.xpos[i]) for name,i in ids.items()))
            sample(trace,st,k*scene['dt'])
        if k==n:break
        d.xfrc_applied[:]=0;d.qfrc_applied[:]=0
        for b in scene['bodies']:
            if b['driven']:
                f,tau=wrench(scene,b,target,st[b['name']],k);d.xfrc_applied[ids[b['name']]]=np.r_[f,tau]
            elif b.get('is_gripper'):
                qi,vi=slots[b['name']];cmd=target['command_'+b['name'].split('_')[0]][k]
                f=motor(scene,cmd,d.qpos[qi],d.qvel[vi]);d.qfrc_applied[vi]=f;motor_max=max(motor_max,abs(f))
                d.xfrc_applied[ids[b['name']],:3]=-scene['assets'][b['asset']]['mass']*np.asarray(scene['gravity'])
        mj.mj_step(m,d);maxcon=max(maxcon,d.ncon)
        if telemetry is not None:telemetry.contacts(k)
        if d.ncon:peak=max(peak,-min(float(c.dist) for c in d.contact[:d.ncon]))
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():break
        if k%1000==0:print('articulated mujoco',k,n,flush=True)
    if telemetry is not None:telemetry.finish()
    warnings={str(i):int(w.number) for i,w in enumerate(d.warning) if w.number}
    return finish(trace),{'engine':'MuJoCo CPU articulated grippers','version':mj.__version__,'finite':bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all() and native_error<1e-7 and not warnings),
          'steps':k,'expected_steps':n,'wall_s':time.perf_counter()-start,'max_contacts':maxcon,'engine_peak_overlap_m':float(peak),'warnings':warnings,
          'motor_force_abs_max_N':motor_max,'native_FK_max_position_error_m':float(native_error),
          'native':{'integrator':int(m.opt.integrator),'iterations':int(m.opt.iterations),'tolerance':float(m.opt.tolerance),
                    'geom_solref':m.geom_solref.tolist(),'geom_solimp':m.geom_solimp.tolist(),'jnt_range':m.jnt_range.tolist()},
          'source_control_audit':scene['source_control_audit']}


def run_sap(scene,target,out,objects):
    from .articulated_sap import run
    return run(scene,target,out,objects)


def run_mujoco_warp(scene,target,out,objects):
    from .articulated_mjwarp import run
    return run(scene,target,out,objects)


def run_ipc(scene,target,out,objects):
    from .articulated_ipc import run
    return run(scene,target,out,objects)
