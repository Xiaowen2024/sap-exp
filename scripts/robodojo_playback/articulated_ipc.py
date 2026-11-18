"""Native libuipc ABD gripper joints and equal/opposite motor forces."""
import importlib.metadata
import time
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as R
from .common import rotation,wrench
from .articulated import motor
from .run import trace_buffer,sample,finish


def skew(t):
    x,y,z=t;return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def run(scene,target,out,objects):
    import uipc
    from uipc.core import Engine,Scene,World,AffineBodyStateAccessorFeature
    from uipc.geometry import trimesh as umesh,label_surface,ground,linemesh
    from uipc.constitution import AffineBodyConstitution,AffineBodyExternalBodyForce,AffineBodyPrismaticJoint,AffineBodyPrismaticJointLimit
    uipc.Logger.set_level(uipc.Logger.Level.Warn)
    cfg=Scene.default_config();cfg['dt']=scene['dt'];cfg['gravity']=[[0.],[0.],[-9.81]]
    par=scene.get('ipc_parameters',{})
    for key in ('d_hat','eps_velocity'):
        if key in par:cfg['contact'][key]=par[key]
    if 'velocity_tol' in par:cfg['newton']['velocity_tol']=par['velocity_tol']
    if 'max_iter' in par:cfg['newton']['max_iter']=int(par['max_iter'])
    eng=Engine('cuda',str(out/'workspace'));world=World(eng);sc=Scene(cfg)
    abd,ext,joint,limit=AffineBodyConstitution(),AffineBodyExternalBodyForce(),AffineBodyPrismaticJoint(),AffineBodyPrismaticJointLimit()
    keep=[abd,ext,joint,limit]
    for c in keep:sc.constitution_tabular().insert(c)
    tab=sc.contact_tabular();tab.default_model(scene['mu'],1e9);el={n:tab.create(n) for n in ('base','finger','bowl','env')}
    for i,a in enumerate(el):
        for b in list(el)[i:]:tab.insert(el[a],el[b],scene['mu'],1e9,a!='base' and b!='base')
    slots,forces={},{}
    by_name={b['name']:b for b in scene['bodies']}
    for b in scene['bodies']:
        a=scene['assets'][b['asset']];merged=trimesh.util.concatenate([trimesh.load(p,force='mesh',process=False) for p in a['pieces']])
        mesh=umesh(np.asarray(merged.vertices,float),np.asarray(merged.faces,np.int32));label_surface(mesh)
        abd.apply_to(mesh,1e8,mass_density=a['mass']/a['additive_volume_m3'])
        T=np.eye(4);T[:3,:3]=rotation(b['q']).as_matrix();T[:3,3]=b['p'];uipc.view(mesh.transforms())[0]=T
        el['base' if b['asset']=='base' else 'finger' if b.get('parent') else 'bowl'].apply_to(mesh)
        obj=sc.objects().create(b['name']);keep.extend([mesh,obj])
        if b.get('is_gripper'):
            ext.apply_to(mesh,np.zeros(12),enable=True);forces[b['name']]=np.zeros(12)
            def animate(info,name=b['name']):
                for slot in info.geo_slots():
                    array=uipc.view(slot.geometry().instances().find('external_force'));array[:]=forces[name].reshape(array.shape)
            sc.animator().insert(obj,animate)
        slot,_=obj.geometries().create(mesh);slots[b['name']]=slot
    for b in scene['bodies']:
        if b.get('parent'):
            parent=by_name[b['parent']];p=np.asarray(parent['p']);axis=rotation(parent['q']).apply(b['axis'])
            line=linemesh(np.array([p,p+axis*.05]),np.array([[0,1]],np.int32))
            joint.apply_to(line,[slots[b['parent']]],np.array([0],np.int32),[slots[b['name']]],np.array([0],np.int32),np.array([100.]))
            limit.apply_to(line,b['joint_lower']-b['joint_initial'],b['joint_upper']-b['joint_initial'],100.)
            obj=sc.objects().create(b['name']+'_joint');obj.geometries().create(line);keep.extend([line,obj])
    if objects:
        g=ground(scene['table_top_z'],np.array([[0.],[0.],[1.]]));el['env'].apply_to(g);obj=sc.objects().create('table');obj.geometries().create(g);keep.extend([g,obj])
    world.init(sc)
    if not world.is_valid():raise RuntimeError('IPC invalid articulated initial state')
    acc=world.features().find(AffineBodyStateAccessorFeature);geo=acc.create_geometry();geo.instances().create(uipc.builtin.transform,np.eye(4));geo.instances().create(uipc.builtin.velocity,np.zeros((4,4)))
    offsets={n:int(np.asarray(uipc.view(s.geometry().meta().find(uipc.builtin.backend_abd_body_offset))).ravel()[0]) for n,s in slots.items()}
    def states():
        acc.copy_to(geo);trans=uipc.view(geo.instances().find(uipc.builtin.transform));vel=uipc.view(geo.instances().find(uipc.builtin.velocity));st={};aff={}
        for name,i in offsets.items():
            T,V=trans[i],vel[i];A=T[:3,:3];U,_,Vt=np.linalg.svd(A);r=R.from_matrix(U@Vt);spin=V[:3,:3]@np.linalg.inv(A)
            w=np.array([spin[2,1]-spin[1,2],spin[0,2]-spin[2,0],spin[1,0]-spin[0,1]])/2
            st[name]=(T[:3,3].copy(),r.as_quat()[[3,0,1,2]],V[:3,3].copy(),w);aff[name]=A.copy()
        return st,aff
    trace=trace_buffer(scene);n=len(target['time'])-1;start=time.perf_counter();deform=0.;joint_transverse=0.;joint_rotation=0.;motor_max=0.
    for k in range(n+1):
        st,aff=states()
        if k%10==0 or k==n:
            sample(trace,st,k*scene['dt']);deform=max(deform,max(np.linalg.norm(A.T@A-np.eye(3)) for A in aff.values()))
        if k==n:break
        for name in forces:forces[name][:]=0
        for b in scene['bodies']:
            if b['driven']:
                f,tau=wrench(scene,b,target,st[b['name']],k);forces[b['name']][:3]=f
                forces[b['name']][3:]=(.5*skew(tau)@np.linalg.inv(aff[b['name']]).T).ravel()
        for b in scene['bodies']:
            if not b.get('parent'):continue
            name,parent=b['name'],b['parent'];bp,bq,bv,bw=st[parent];p,q,v,w=st[name];rot=rotation(bq);axis=rot.apply(b['axis'])
            r=p-bp;delta=r-rot.apply(b['zero_p']);slide=float(delta@axis);speed=float((v-bv-np.cross(bw,r))@axis)
            joint_transverse=max(joint_transverse,float(np.linalg.norm(delta-axis*slide)))
            joint_rotation=max(joint_rotation,float((rotation(q)*rot.inv()).magnitude()))
            F=motor(scene,target['command_'+name.split('_')[0]][k],slide,speed);motor_max=max(motor_max,abs(F));fv=axis*F
            forces[name][:3]=fv-scene['assets'][b['asset']]['mass']*np.asarray(scene['gravity'])
            forces[parent][:3]-=fv
            forces[parent][3:]-=(.5*skew(np.cross(r,fv))@np.linalg.inv(aff[parent]).T).ravel()
        world.advance();world.retrieve()
        if not world.is_valid():break
        if k%1000==0:print('articulated ipc',k,n,flush=True)
    return finish(trace),{'engine':'libuipc ABD articulated grippers','version':importlib.metadata.version('pyuipc'),'device':'CUDA',
        'steps':k,'expected_steps':n,'finite':bool(world.is_valid() and all(np.isfinite(v).all() for v in finish(trace).values())),
        'wall_s':time.perf_counter()-start,'affine_deformation_max':float(deform),'joint_transverse_error_max_m':joint_transverse,
        'joint_relative_rotation_max_deg':float(np.degrees(joint_rotation)),'motor_force_abs_max_N':motor_max,
        'native_config':cfg,'required_model_constants':{'abd_kappa_Pa':1e8,'joint_strength_ratio':100.,'joint_limit_strength':100.,'contact_kappa_requested':1e9},
        'engine_peak_overlap_m':None,'contact_capacity_audited':False,'source_control_audit':scene['source_control_audit']}
