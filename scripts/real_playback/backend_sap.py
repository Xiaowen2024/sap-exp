"""SAP runtime adapter. Internal implementation stays in its own checkout.

The public runner supplies physical bodies, joints and external wrench only.
Explicit task pair friction is installed after native Jacobian construction,
before contact solution; it is physical input shared with the other engines.
"""
from __future__ import annotations

from pathlib import Path
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation

from .io_utils import GateError, write_json


class SAPBackend:
    def __init__(self, scene, assets, base_pose, width, candidate, output, objects=True):
        import warp as wp
        import sap_warp
        from sap_warp import SolverSAP, load_sap_scene
        from sap_warp.collision.pipeline import SapCollisionPipeline
        from sap_warp.resources.collision_model import sap_collision_state_from_state
        from sap_warp.sap_kinematics import sap_eval_fk
        self.wp,self.scene,self.candidate,self.objects=wp,scene,candidate,objects
        self.sap_eval_fk,self.collision_state=sap_eval_fk,sap_collision_state_from_state
        if scene['table_z_m']!=0:
            raise GateError('SAP adapter requires the audited table-aligned frame with table z=0')
        output=Path(output);output.mkdir(parents=True,exist_ok=True)
        def pose(p):return {'p':np.asarray(p[:3]).tolist(),'q':np.asarray(p[3:]).tolist()}
        def material():
            m={'mu':1.,'mu_torsional':0.,'mu_rolling':0.,'density':0.}
            if not candidate['native']:
                p=candidate['parameters']
                if 'ke' in p:m['ke']=p['ke']
                if 'tau_s' in p:m['tau']=p['tau_s']
            return m
        d=scene['drive'];half=np.array(scene['pad']['size_m'])/2
        bodies=[{'id':'base','transform':pose(base_pose),'mass':d['base_mass_kg'],
                 'inertia':np.diag(d['base_inertia_kg_m2']).tolist()}]
        joints=[{'id':'base_free','type':'free','parent':'world','child':'base'}]
        moving=['base_free']
        R=Rotation.from_quat(base_pose[3:])
        for name,sign in (('left',-1),('right',1)):
            p=np.r_[base_pose[:3]+R.apply([sign*(width/2+half[0]),0,0]),base_pose[3:]]
            mass=scene['pad']['mass_kg'];I=mass/3*(np.sum(half**2)-half**2)
            bodies.append({'id':name,'transform':pose(p),'mass':mass,'inertia':np.diag(I).tolist(),
                           'shapes':[{'type':'convex_mesh','label':name+'_pad','path':assets['pad']['path'],'cfg':material()}]})
            joints.append({'id':name+'_slide','type':'prismatic','parent':'base','child':name,
                           'parent_xform':{'p':[sign*half[0],0,0],'q':'identity'},'axis':[sign,0,0],
                           'limit_lower':0.,'limit_upper':scene['pad']['stroke_m']/2,
                           'target_ke':0.,'target_kd':0.})
            moving.append(name+'_slide')
        arts=[{'id':'gripper','joints':moving}]
        if objects:
            plate=scene['plate'];I=np.asarray(assets['plate']['inertia_per_kg_m2'])*plate['mass_kg']
            bodies.append({'id':'plate','transform':pose(np.r_[plate['position_m'],plate['quaternion_xyzw']]),
                           'mass':plate['mass_kg'],'inertia':I.tolist(),
                           'shapes':[{'type':'convex_mesh','label':f'plate_{i:04d}','path':p['path'],'cfg':material()}
                                     for i,p in enumerate(assets['plate']['pieces'])]})
            joints.append({'id':'plate_free','type':'free','parent':'world','child':'plate'})
            arts.append({'id':'plate_art','joints':['plate_free']})
            Rrack=Rotation.from_quat(scene['rack']['quaternion_xyzw'])
            for i,piece in enumerate(assets['rack']):
                name=f'rack_{i:03d}'
                p=np.r_[np.array(scene['rack']['position_m'])+Rrack.apply(piece['position_m']),
                         (Rrack*Rotation.from_quat(piece['quaternion_xyzw'])).as_quat()]
                bodies.append({'id':name,'transform':pose(p),'mass':0.,'inertia':np.zeros((3,3)).tolist(),
                               'shapes':[{'type':'convex_mesh','label':name+'_geom','path':piece['path'],'cfg':material()}]})
                joints.append({'id':name+'_fixed','type':'fixed','parent':'world','child':name,
                               'parent_xform':pose(p)})
                arts.append({'id':name+'_art','joints':[name+'_fixed']})
        gravity=np.asarray(d['gravity_m_s2'])
        if not np.allclose(gravity,[0,0,gravity[2]]):
            raise GateError('SAP loader gravity vector must be verified before nonvertical gravity is used')
        cfg={'schema_version':1,'name':'real_playback_v1','simulation':{'dt':scene['dt_s'],'num_worlds':1,'nconmax':scene['contact_capacity']},
             'builder':{'gravity':float(gravity[2])},'ground':{'enabled':objects,'label':'table','cfg':material()},
             'bodies':bodies,'joints':joints,'articulations':arts}
        write_json(output/'scene.json',cfg);wp.init()
        loaded=load_sap_scene(output/'scene.json',device='cuda:0',naconmax=scene['contact_capacity'],strict=True)
        self.model,self.state,self.control=loaded.sap_model,loaded.sap_state,loaded.sap_control
        self.other=self.model.state();self.labels=list(self.model.body_label)
        self.ids={n:self.labels.index(n) for n in ['base','left','right']+(['plate'] if objects else [])}
        self.qstart=self.model.joint_q_start.numpy();self.vstart=self.model.joint_qd_start.numpy()
        self.jids={n:list(self.model.joint_label).index(n) for n in moving+(['plate_free'] if objects else [])}
        q=self.state.joint_q.numpy();q[:]=0
        q[self.qstart[self.jids['base_free']]:self.qstart[self.jids['base_free']]+7]=base_pose
        for n in ('left','right'):q[self.qstart[self.jids[n+'_slide']]]=width/2
        if objects:
            start=self.qstart[self.jids['plate_free']]
            q[start:start+7]=np.r_[scene['plate']['position_m'],scene['plate']['quaternion_xyzw']]
        self.state.joint_q.assign(q);self.state.joint_qd.zero_();sap_eval_fk(self.model,self.state.joint_q,self.state.joint_qd,self.state)
        kwargs={'nconmax':scene['contact_capacity']}
        if 'sigma' in candidate['parameters']:kwargs['contact_sigma']=candidate['parameters']['sigma']
        self.solver=SolverSAP(self.model,**kwargs)
        self.pipeline=SapCollisionPipeline(loaded.collision_model,naconmax=scene['contact_capacity']);self.contacts=self.pipeline.contacts()
        original=self.solver.contact_jacobian.compute
        self.actual_pairs=set()
        def compute(*args,**kwargs):
            result=original(*args,**kwargs)
            mu=result.contact_env_mu.numpy();a=result.contact_env_body0.numpy().reshape(-1);b=result.contact_env_body1.numpy().reshape(-1)
            n=int(result.contact_env_count.numpy().ravel()[0]);flat=mu.reshape(-1)
            ke=result.contact_env_stiffness.numpy().ravel();tau=result.contact_env_tau_d.numpy().ravel()
            for i in range(n):
                names=[self.labels[j] if j>=0 else 'table' for j in (int(a[i]),int(b[i]))]
                if 'plate' in names:
                    flat[i]=scene['mu_pad_plate'] if {'left','right'} & set(names) else scene['mu_plate_environment']
                elif {'left','right'} & set(names):flat[i]=scene['mu_pad_environment']
                self.actual_pairs.add((tuple(sorted(names)),float(flat[i]),float(ke[i]),float(tau[i])))
            result.contact_env_mu.assign(mu)
            return result
        self.solver.contact_jacobian.compute=compute
        self.truncated=0
        self.module_path=Path(sap_warp.__file__).resolve()
        for name,mass in [('base',d['base_mass_kg']),('left',scene['pad']['mass_kg']),('right',scene['pad']['mass_kg'])]+([('plate',scene['plate']['mass_kg'])] if objects else []):
            actual=float(self.model.body_mass.numpy()[self.ids[name]])
            if not np.isclose(actual,mass,rtol=1e-5):raise GateError(f'SAP compiled mass mismatch for {name}: {actual} vs {mass}')

    def snapshot(self):
        p=self.state.body_q.numpy();v=self.state.body_qd.numpy()
        q,qd=self.state.joint_q.numpy(),self.state.joint_qd.numpy()
        poses={n:p[i].copy() for n,i in self.ids.items()};twists={n:v[i].copy() for n,i in self.ids.items()}
        return {'poses':poses,'twists':twists,
                'slider_q_m':np.array([q[self.qstart[self.jids[n+'_slide']]] for n in ('left','right')]),
                'slider_qd_m_s':np.array([qd[self.vstart[self.jids[n+'_slide']]] for n in ('left','right')]),
                'contact_truncated':self.truncated>0,'valid':bool(np.isfinite(p).all() and np.isfinite(v).all()),'contact_count':None}

    def step(self,base_wrench,fingers):
        self.state.clear_forces();body_f=self.state.body_f.numpy();body_f[self.ids['base']]=base_wrench;self.state.body_f.assign(body_f)
        jf=self.control.joint_f.numpy();jf[:]=0
        for n,f in zip(('left','right'),fingers):jf[self.vstart[self.jids[n+'_slide']]]=f
        self.control.joint_f.assign(jf)
        self.pipeline.collide(self.collision_state(self.state),self.contacts)
        self.solver.step(self.state,self.other,self.control,self.contacts,self.scene['dt_s'])
        self.truncated+=int(self.solver.last_truncated_contact_count or 0)
        self.state,self.other=self.other,self.state

    def audit(self):
        try:
            commit=subprocess.check_output(['git','-C',str(self.module_path.parents[2]),'rev-parse','HEAD'],text=True).strip()
        except subprocess.CalledProcessError:commit='unavailable'
        return {'engine':'SAP Warp','warp_version':self.wp.__version__,'device':'cuda:0','candidate':self.candidate,
                'sap_module':str(self.module_path),'sap_commit':commit,
                'compiled_mass_kg':self.model.body_mass.numpy().tolist(),
                'actual_pair_mu_ke_tau':sorted(self.actual_pairs),
                'pair_friction_route':'physical pair values applied to native Jacobian before native contact solution',
                'candidate_ke_tau_semantics':'shape-level overrides; combined pair values recorded separately',
                'body_twist_order':'world linear, world angular; requires native preflight',
                'contact_capacity':self.scene['contact_capacity'],'truncated_contacts':self.truncated}

    def close(self):
        pass
