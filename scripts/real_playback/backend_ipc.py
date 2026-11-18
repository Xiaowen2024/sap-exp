"""Native libuipc ABD gripper, prismatic fingers and common external wrench.

No Genesis rigid-body solver or rigid prescribed transforms enter this path.
Adapter verification is required on the pinned server before any task claim.
"""
from __future__ import annotations

import importlib.metadata
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from .assets import mass_properties
from .io_utils import GateError
from .observer import load_obj, transform
from .scene import box_mesh


def skew(t):
    x,y,z=t
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])


def affine_wrench(force, torque, A):
    """Virtual-work-equivalent world torque for a nonsingular affine frame."""
    return np.r_[force, (.5*skew(torque)@np.linalg.inv(A).T).ravel()]


class IPCBackend:
    def __init__(self, scene, assets, base_pose, width, candidate, output, objects=True):
        import uipc
        from uipc.core import Engine, Scene, World
        from uipc.geometry import trimesh, label_surface, ground, linemesh
        from uipc.constitution import (AffineBodyConstitution, AffineBodyExternalBodyForce,
                                      AffineBodyPrismaticJoint, AffineBodyPrismaticJointLimit)
        self.uipc, self.scene_cfg, self.objects, self.candidate = uipc, scene, objects, candidate
        uipc.Logger.set_level(uipc.Logger.Level.Warn)
        uipc.Timer.disable_all()
        self.keepalive=[]
        output=Path(output);output.mkdir(parents=True,exist_ok=True)
        cfg=Scene.default_config();cfg['dt']=scene['dt_s'];cfg['gravity']=np.array(scene['drive']['gravity_m_s2']).reshape(3,1).tolist()
        if not candidate['native']:
            p=candidate['parameters']
            if 'd_hat_m' in p: cfg['contact']['d_hat']=p['d_hat_m']
            if 'eps_velocity_m_s' in p: cfg['contact']['eps_velocity']=p['eps_velocity_m_s']
            if 'newton_velocity_tol' in p: cfg['newton']['velocity_tol']=p['newton_velocity_tol']
        self.config=cfg
        self.engine=Engine('cuda',str(output/'workspace'));self.world=World(self.engine);self.scene=Scene(cfg)
        abd,ext,joint,limit=AffineBodyConstitution(),AffineBodyExternalBodyForce(),AffineBodyPrismaticJoint(),AffineBodyPrismaticJointLimit()
        self.keepalive.extend((abd,ext,joint,limit))
        for c in (abd,ext,joint,limit):self.scene.constitution_tabular().insert(c)
        table=self.scene.contact_tabular();table.default_model(0.,1e9)
        elements={n:table.create(n) for n in ('base','left','right','plate','rack','table')}
        for a,ea in elements.items():
            for b,eb in elements.items():
                if list(elements).index(a)>list(elements).index(b):continue
                enabled=False;mu=0.
                if 'plate' in (a,b) and a!=b and 'base' not in (a,b):
                    enabled=True;mu=scene['mu_pad_plate'] if {'left','right'} & {a,b} else scene['mu_plate_environment']
                elif ({'left','right'} & {a,b}) and ({'rack','table'} & {a,b}):
                    enabled=True;mu=scene['mu_pad_environment']
                # Resistance is a native contact-law argument; use the exact
                # library default rather than a candidate-dependent physical gap.
                table.insert(ea,eb,mu,1e9,enabled)
        self.slots={};self.force_targets={};self.initial_T={};self.mass={}
        def add(name,v,f,pose,mass,fixed=False):
            mesh=trimesh(v,f);label_surface(mesh)
            self.keepalive.extend((mesh,v,f))
            volume,centre,I=mass_properties(v,f)
            if np.linalg.norm(centre)>1e-7:raise GateError('IPC mesh origin must equal COM')
            # pyuipc requires ABD kappa explicitly: it has no default. This
            # fixed model constant is declared before selection, not tuned.
            abd.apply_to(mesh,scene['ipc_abd_kappa_Pa'],mass_density=mass/volume)
            T=np.eye(4);T[:3,:3]=Rotation.from_quat(pose[3:]).as_matrix();T[:3,3]=pose[:3]
            uipc.view(mesh.transforms())[0]=T
            uipc.view(mesh.instances().find(uipc.builtin.is_fixed))[:]=int(fixed)
            elements[name].apply_to(mesh)
            obj=self.scene.objects().create(name)
            self.keepalive.append(obj)
            if not fixed:
                ext.apply_to(mesh,np.zeros(12),enable=True)
                self.force_targets[name]=np.zeros(12)
                def animate(info,name=name):
                    for slot in info.geo_slots():
                        attr=slot.geometry().instances().find('external_force')
                        if attr is None:raise GateError('missing IPC external force attribute')
                        uipc.view(attr)[:]=self.force_targets[name].reshape(np.asarray(uipc.view(attr)).shape)
                self.scene.animator().insert(obj,animate)
            slot,_=obj.geometries().create(mesh)
            self.slots[name]=slot;self.initial_T[name]=T;self.mass[name]=mass
        I=np.asarray(scene['drive']['base_inertia_kg_m2']);m=scene['drive']['base_mass_kg']
        half=np.sqrt(1.5*(I.sum()-2*I)/m)
        v,f=box_mesh(half);add('base',v,f,base_pose,m)
        pv,pf=load_obj(assets['pad']['path']);R=Rotation.from_quat(base_pose[3:]);hx=scene['pad']['size_m'][0]/2
        for name,sign in (('left',-1),('right',1)):
            p=np.r_[base_pose[:3]+R.apply([sign*(width/2+hx),0,0]),base_pose[3:]]
            add(name,pv,pf,p,scene['pad']['mass_kg'])
            start=base_pose[:3];axis=R.apply([sign,0,0])
            line=linemesh(np.array([start,start+axis]),np.array([[0,1]],dtype=np.int32))
            self.keepalive.append(line)
            joint.apply_to(line,[self.slots['base']],[0],[self.slots[name]],[0],[100.])
            limit.apply_to(line,-width/2,scene['pad']['stroke_m']/2-width/2,100.)
            self.scene.objects().create(name+'_prismatic').geometries().create(line)
        if objects:
            vertices,faces=[],[];offset=0
            for piece in assets['plate']['pieces']:
                v,f=load_obj(piece['path']);vertices.extend(v);faces.extend(f+offset);offset+=len(v)
            add('plate',np.asarray(vertices),np.asarray(faces),np.r_[scene['plate']['position_m'],scene['plate']['quaternion_xyzw']],scene['plate']['mass_kg'])
            # Rack pieces share a fixed body and do not need a physical inertia.
            for i,piece in enumerate(assets['rack']):
                v,f=load_obj(piece['path']);pose_local=np.r_[piece['position_m'],piece['quaternion_xyzw']]
                v=transform(v,pose_local)
                # Separate fixed geometries in one object preserve their shared
                # surface files without adding an artificial enclosing hull.
                mesh=trimesh(v,f);label_surface(mesh);abd.apply_to(mesh,scene['ipc_abd_kappa_Pa'])
                T=np.eye(4);T[:3,:3]=Rotation.from_quat(scene['rack']['quaternion_xyzw']).as_matrix();T[:3,3]=scene['rack']['position_m']
                uipc.view(mesh.transforms())[0]=T;uipc.view(mesh.instances().find(uipc.builtin.is_fixed))[:]=1
                elements['rack'].apply_to(mesh);self.scene.objects().create(f'rack_{i}').geometries().create(mesh)
            floor=ground(scene['table_z_m'],np.array([0.,0.,1.]));elements['table'].apply_to(floor)
            self.scene.objects().create('table').geometries().create(floor)
        self.world.init(self.scene)
        if not self.world.is_valid():raise GateError('IPC initialization failed; no scene shift is allowed')
        from uipc.core import AffineBodyStateAccessorFeature
        self.state_accessor=self.world.features().find(AffineBodyStateAccessorFeature)
        if self.state_accessor is None:raise GateError('IPC native state accessor unavailable')
        self.state_geo=self.state_accessor.create_geometry()
        self.state_geo.instances().create(uipc.builtin.transform,np.eye(4,dtype=np.float64))
        self.state_geo.instances().create(uipc.builtin.velocity,np.zeros((4,4),dtype=np.float64))
        self.body_offsets={}
        for name,slot in self.slots.items():
            offset=slot.geometry().meta().find(uipc.builtin.backend_abd_body_offset)
            if offset is None:raise GateError('IPC native body offset unavailable')
            self.body_offsets[name]=int(np.asarray(uipc.view(offset)).ravel()[0])

    def snapshot(self):
        poses,twists={},{}
        self.affine={}
        self.state_accessor.copy_to(self.state_geo)
        Ts=np.asarray(self.uipc.view(self.state_geo.instances().find(self.uipc.builtin.transform)))
        Vs=np.asarray(self.uipc.view(self.state_geo.instances().find(self.uipc.builtin.velocity)))
        affine_transforms={}
        for name,slot in self.slots.items():
            offset=self.body_offsets[name];T=Ts[offset].copy();affine_transforms[name]=T.copy()
            self.affine[name]=T[:3,:3]
            U,_,V=np.linalg.svd(T[:3,:3]);R=U@V
            if np.linalg.det(R)<=0:raise GateError('IPC affine body inverted')
            poses[name]=np.r_[T[:3,3],Rotation.from_matrix(R).as_quat()]
            velocity=Vs[offset]
            spin=velocity[:3,:3]@np.linalg.inv(T[:3,:3])
            omega=np.array([spin[2,1]-spin[1,2],spin[0,2]-spin[2,0],spin[1,0]-spin[0,1]])/2
            twists[name]=np.r_[velocity[:3,3],omega]
        base=poses['base'];R=Rotation.from_quat(base[3:]);hx=self.scene_cfg['pad']['size_m'][0]/2
        q,qd=[],[]
        for name,sign in (('left',-1),('right',1)):
            axis=R.apply([sign,0,0]);delta=poses[name][:3]-base[:3]
            q.append(float(delta@axis-hx))
            rel=twists[name][:3]-twists['base'][:3]-np.cross(twists['base'][3:],delta)
            qd.append(float(rel@axis))
        return {'poses':poses,'twists':twists,'slider_q_m':np.array(q),'slider_qd_m_s':np.array(qd),
                'affine_transforms':affine_transforms,
                'contact_truncated':False,'valid':bool(self.world.is_valid()),'contact_count':None,
                'affine_deformation_max':max(float(np.linalg.norm(A.T@A-np.eye(3))) for A in self.affine.values())}

    def step(self,base_wrench,fingers):
        state=self.snapshot();forces={n:np.zeros(6) for n in self.force_targets}
        forces['base'][:]=base_wrench
        R=Rotation.from_quat(state['poses']['base'][3:])
        for (name,sign),f in zip((('left',-1),('right',1)),fingers):
            F=R.apply([sign*f,0,0]);forces[name][:3]+=F;forces['base'][:3]-=F
            r=state['poses'][name][:3]-state['poses']['base'][:3]
            forces['base'][3:]-=np.cross(r,F)
        for name,F in forces.items():
            self.force_targets[name][:]=affine_wrench(F[:3],F[3:],self.affine[name])
        self.world.advance();self.world.retrieve()

    def audit(self):
        return {'engine':'libuipc ABD','version':importlib.metadata.version('pyuipc'),
                'device':'CUDA','candidate':self.candidate,'actual_scene_config':self.config,
                'mass_kg':self.mass,'initial_transforms':{n:T.tolist() for n,T in self.initial_T.items()},
                'capacity_policy':'native dynamic buffers; rejected world is invalid',
                'prismatic_strength_ratio':100.,'abd_kappa_Pa':self.scene_cfg['ipc_abd_kappa_Pa'],
                'contact_readout':'common frozen geometric-contact predicate; native barrier distance is recorded',
                'external_wrench':'virtual-work-equivalent affine generalized force'}

    def close(self):
        pass
