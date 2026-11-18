"""Genesis 1.4.6 SAP coupler adapter; same physical MJCF and scoring callbacks as the rigid track."""
from __future__ import annotations
import hashlib
import os
import json
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import genesis_rigid_common as R
from genesis_rigid_common import (ROOT, mjcf, np_, finite_rows, Timer, save_json, load_manifest, json_safe, versions)

REGISTRY = ROOT / 'protocol/registries/genesis_sap_rerun.json'
ENGINE = 'Genesis SAP coupler'
MESH_KW = R.MESH_KW


_GS = None

def gs_init():
    global _GS
    if _GS is None:
        import importlib.metadata as metadata
        import os
        import torch
        import genesis as gs
        if metadata.version('genesis-world') != '1.4.6' or not torch.cuda.is_available():
            raise RuntimeError('Genesis 1.4.6 CUDA required')
        gs.init(backend=gs.cuda, precision='64', seed=20261114, logging_level=os.environ.get('GS_LOG','warning'))
        if os.environ.get('SAP_BUFFER_CAPACITY'):
            from genesis_sap_buffers import configure
            configure(int(os.environ['SAP_BUFFER_CAPACITY']),int(os.environ.get('SAP_QUERY_FACTOR','2')),int(os.environ.get('SAP_PAIR_FACTOR','2')))
        _GS=gs
    return _GS


def registry_task(task_id, manifest_path):
    reg=json.loads(REGISTRY.read_text()); spec=next(t for t in reg['tasks'] if t['id']==task_id)
    man,sha=load_manifest(manifest_path)
    if Path(manifest_path).name!=spec['manifest'] or sha!=spec['sha256']:
        raise ValueError('manifest differs from frozen SAP registry')
    return reg,spec,sha


def registry_id_for(manifest_path):
    return next(t['id'] for t in json.loads(REGISTRY.read_text())['tasks'] if t['manifest']==Path(manifest_path).name)


def with_candidates(man,spec,key='genesis_sap_candidates'):
    m=json.loads(json.dumps(man));m['candidate_rule'][key]=list(spec['candidates']);return m


def parse_candidate(text,spec):
    if text.strip()=='native default':
        return {'id':'native_default','native':True,'text':text}
    c={'id':text,'native':False,'text':text,**spec.get('nonnative',{})}
    for part in text.replace(' ','').split(','):
        k,v=part.split('=');c[k]=float(v)
    return c


def sol_vector(c,**kw):
    return None  # RigidSolver sol_params do not parameterise SAP contact.


def header(man,sha,spec):
    return {'protocol':man['protocol'],'task':man['task'],'revision':man['revision'],'manifest_sha256':sha,
            'engine':ENGINE,'versions':versions(),'registry':str(REGISTRY.relative_to(ROOT)),
            'registry_sha256':hashlib.sha256(REGISTRY.read_bytes()).hexdigest(),
            'adapter_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'genesis_candidates':spec['candidates'],'nonnative_settings':spec.get('nonnative',{}),
            'contact_force_readout':'SAP handler contact impulse gamma / dt; world frame on link_idx0 (rigid pair) or link_idx (floor)',
            'representation':'Genesis native tetrahedral hydroelastic mesh; exact manifest mass/inertia and unchanged surface geometry'}


class Built(R.Built):
    def __init__(self,*args):
        super().__init__(*args)
        self.cp=self.scene.sim.coupler
        self.link_owner={int(l.idx):name for name,e in self.ents.items() for l in e.links}
        self.max_rigid_solver_contacts=None

    def contacts(self):
        rows=[]
        for h in self.cp.contact_handlers:
            n=int(h.n_contact_pairs[None])
            if n==0: continue
            # These are SAP's actual contact impulses, never inferred from momentum balance.
            p=h.contact_pairs.to_numpy()
            g=p['sap_info']['gamma'][:n]/self.dt
            batch=p['batch_idx'][:n]
            if 'link_idx0' in p:
                force=g[:,0,None]*p['tangent0'][:n]+g[:,1,None]*p['tangent1'][:n]+g[:,2,None]*p['normal'][:n]
                la,lb=p['link_idx1'][:n],p['link_idx0'][:n]
                normal=p['normal'][:n]
            else:
                force=g; lb=p['link_idx'][:n]
                floor=int(self.rs.geoms[self.cp._ground_geom_idx].link.idx)
                la=np.full(n,floor); normal=np.tile([0.,0.,1.],(n,1))
            rows.extend((int(batch[i]),int(la[i]),int(lb[i]),force[i],normal[i]) for i in range(n))
        counts=np.bincount([x[0] for x in rows],minlength=self.B) if rows else np.zeros(self.B,int)
        cap=max(1,int(counts.max())); ga=np.full((self.B,cap),-1,int); gb=ga.copy()
        f=np.zeros((self.B,cap,3)); norm=f.copy(); idx=np.zeros(self.B,int)
        # Parent helper expects geometry ids; virtual ids here are link ids, with the same ownership map.
        self.geom_owner=self.link_owner
        for env,a,b,ff,nn in rows:
            j=idx[env];idx[env]+=1;ga[env,j]=a;gb[env,j]=b;f[env,j]=ff;norm[env,j]=nn
        self.max_contacts_seen=max(self.max_contacts_seen,int(counts.max()))
        return {'geom_a':ga,'geom_b':gb,'force':f,'normal':norm}

    def provenance(self):
        p=super().provenance()
        p.update(sap_options=self.cp.options.model_dump(),
                 rigid_constraint_solve='bypassed by upstream SAPCoupler branch',
                 precision_bits=64,
                 contact_capacities=[{'handler':getattr(h,'name',type(h).__name__),'max_pairs':h.max_contact_pairs,'max_candidates':getattr(h,'max_contact_candidates',None)} for h in self.cp.contact_handlers],
                 sap_last_batch_active=self.cp.batch_active.to_numpy().tolist(),
                 sap_last_state={k:v.tolist() for k,v in self.cp.sap_state.to_numpy().items()},
                 n_rigid_volume_elems=self.cp.n_rigid_volume_elems,
                 tetrahedra_sha256=hashlib.sha256(self.cp.rigid_volume_elems.to_numpy().tobytes()).hexdigest(),
                 tetrahedral_vertices_sha256=hashlib.sha256(self.cp.rigid_volume_verts_rest.to_numpy().tobytes()).hexdigest(),
                 geoms_coup_friction=self.rs.dyn_info.geoms.coup_friction.to_numpy().tolist(),
                 friction_rule='floor: body coup_friction; rigid pair: sqrt(mu_a*mu_b)')
        return p


def build(entities,*,dt,gravity=(0.,0.,-9.81),cand,n_envs,sol=None):
    gs=gs_init()
    kw={} if cand.get('native') else {k:v for k,v in cand.items() if k in gs.options.SAPCouplerOptions.model_fields}
    for k in ('n_sap_iterations','n_pcg_iterations','n_linesearch_iterations'):
        if k in kw:kw[k]=int(kw[k])
    scene=gs.Scene(sim_options=gs.options.SimOptions(dt=dt,substeps=1,gravity=tuple(gravity)),
                   coupler_options=gs.options.SAPCouplerOptions(**kw),show_viewer=False)
    ents={}; tmpdir=Path(tempfile.mkdtemp(prefix='gssap-',dir='/dev/shm'))
    for name,xml in entities.items():
        root=ET.fromstring(xml); geoms=root.findall('.//geom')
        mu=float(geoms[0].get('friction','1').split()[0])
        # Pair friction follows sqrt, floor reads coup_friction. MIT pusher/table require distinct values.
        rigid_mu=mu
        if name=='pusher':rigid_mu=0.25**2/0.22
        f=tmpdir/f'{name}.xml';f.write_text(xml)
        ents[name]=scene.add_entity(gs.morphs.MJCF(file=str(f),default_armature=None,**MESH_KW),
                                  material=gs.materials.Rigid(friction=rigid_mu,coup_friction=mu))
    scene.build(n_envs=n_envs)
    return Built(scene,ents,n_envs,dt,cand,entities)


# Each SAP scene owns field-backed buffers that must be released before the next scene.
# Reuse the already audited spawn/pipe transport (no IPC simulation runs here).
from genesis_ipc_common import Isolated as _Transport

class SAPIsolated(_Transport):
    OOM_WAIT_S, OOM_RETRIES = 60, 720

    def _call_once(self, *args, **kwargs):
        result,why,oom=super()._call_once(*args, **kwargs)
        self._stop()
        if why and 'Overflowed In Contact Query' in (why+str(self.failures[-1:] )) and os.environ.get('SAP_BUFFER_CAPACITY'):
            capacity=int(os.environ['SAP_BUFFER_CAPACITY']);factor=int(os.environ.get('SAP_QUERY_FACTOR','2'));pairfactor=int(os.environ.get('SAP_PAIR_FACTOR','2'))
            if capacity<1048576 or factor<32:
                os.environ['SAP_BUFFER_CAPACITY']=str(min(1048576,capacity*2))
                os.environ['SAP_PAIR_FACTOR']=str(pairfactor*2)
                os.environ['SAP_QUERY_FACTOR']=str(min(32,factor*2))
                print('CAPACITY-RETRY',os.environ['SAP_BUFFER_CAPACITY'],os.environ['SAP_QUERY_FACTOR'],flush=True)
                # Remove the transport's capacity exception from numerical failures; rerun the same physical call.
                self.failures.pop()
                return self._call_once(*args, **kwargs)
        if why:why=why.replace('IPC process died','SAP process died').replace('libuipc abort, e.g. diverged PCG/Newton solve','CUDA/kernel or numerical failure')
        return result,why,oom

    def call(self, mod, fn, tag='', **kw):
        import pickle
        from common.paths import find_script
        source=find_script(mod+'.py')
        key=hashlib.sha256(pickle.dumps((mod,fn,kw,hashlib.sha256(source.read_bytes()).hexdigest(),os.environ.get('SAP_BUFFER_CAPACITY'),os.environ.get('SAP_QUERY_FACTOR')),protocol=5)).hexdigest()
        cache=Path('/dev/shm/genesis_sap_call_cache');cache.mkdir(exist_ok=True)
        path=cache/(key+'.pkl')
        if path.exists():
            with path.open('rb') as f:return pickle.load(f),None
        result,why=super().call(mod,fn,tag,**kw)
        if why and 'environment failure' in why:
            raise RuntimeError(why)
        if result is not None:
            temp=path.with_suffix('.tmp')
            with temp.open('wb') as f:pickle.dump(result,f,protocol=5)
            temp.replace(path)
        return result,why

ISO = SAPIsolated()
_save_json_base = save_json

def save_json(path, data):
    if isinstance(data, dict):
        data = {**data, 'isolation_log': {'failures': ISO.failures, 'oom_retries': ISO.oom_retries}}
    return _save_json_base(path, data)
