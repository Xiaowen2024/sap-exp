"""Three native backends for a shared, explicitly reduced RoboDojo pilot.

Run each engine in its own process. No parameter selection or benchmark
ranking is performed here. Numeric contact/solver controls remain native.
"""
import argparse
import importlib.metadata
import json
from pathlib import Path
import resource
import time
import xml.etree.ElementTree as ET
import hashlib
import os

import numpy as np
from scipy.spatial.transform import Rotation as R

from .common import load, rotation, save, wrench

SOURCE_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def nums(values): return ' '.join(f'{float(x):.12g}' for x in np.ravel(values))


def trace_buffer(scene):
    return {'time': [], **{b['name'] + '__' + key: [] for b in scene['bodies'] for key in ('pose', 'velocity')}}


def sample(trace, states, t):
    trace['time'].append(t)
    for name, (p, q, v, w) in states.items():
        trace[name + '__pose'].append(np.r_[p, q]); trace[name + '__velocity'].append(np.r_[v, w])


def finish(trace): return {k: np.asarray(v) for k, v in trace.items()}


def make_xml(scene, objects):
    root = ET.Element('mujoco', model='robodojo_reduced_gripper_pilot')
    ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false')
    ET.SubElement(root, 'option', timestep=str(scene['dt']), gravity=nums(scene['gravity']))
    asset = ET.SubElement(root, 'asset'); wb = ET.SubElement(root, 'worldbody')
    for kind, a in scene['assets'].items():
        for i, path in enumerate(a['pieces']): ET.SubElement(asset, 'mesh', name=f'{kind}_{i}', file=path)
    if objects:
        ET.SubElement(wb, 'geom', name='table', type='box', pos='0 -.05 .74', size='.7 .55 .025',
                      friction=nums([scene['mu'], 0, 0]), condim='3')
    for b in scene['bodies']:
        a = scene['assets'][b['asset']]; I = np.asarray(a['inertia'])
        body = ET.SubElement(wb, 'body', name=b['name'], pos=nums(b['p']), quat=nums(b['q']))
        ET.SubElement(body, 'freejoint', name=b['name'] + '_free')
        ET.SubElement(body, 'inertial', pos='0 0 0', mass=str(a['mass']),
                      fullinertia=nums([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]))
        for i in range(len(a['pieces'])):
            ET.SubElement(body, 'geom', type='mesh', mesh=f'{b["asset"]}_{i}', friction=nums([scene['mu'], 0, 0]), condim='3')
    contact = ET.SubElement(root, 'contact')
    names={b['name'] for b in scene['bodies']}
    for side in ('left', 'right'):
        if side+'_link7' in names:ET.SubElement(contact, 'exclude', body1=side+'_link7', body2=side+'_link8')
    return ET.tostring(root, encoding='unicode')


def run_mujoco(scene, target, out, objects):
    import mujoco as mj
    xml = make_xml(scene, objects); (out / 'scene.xml').write_text(xml)
    m = mj.MjModel.from_xml_string(xml); d = mj.MjData(m)
    ids = {b['name']: mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, b['name']) for b in scene['bodies']}
    slots = {b['name']: (int(m.jnt_qposadr[i]), int(m.jnt_dofadr[i])) for i, b in enumerate(scene['bodies'])}
    for b in scene['bodies']:
        qi,vi=slots[b['name']];d.qvel[vi:vi+3]=b.get('v0',[0.,0.,0.]);d.qvel[vi+3:vi+6]=rotation(b['q']).inv().apply(b.get('w0',[0.,0.,0.]))
    mj.mj_forward(m, d)
    def states():
        result = {}
        for name, (qi, vi) in slots.items():
            q = d.qpos[qi+3:qi+7].copy()
            result[name] = (d.qpos[qi:qi+3].copy(), q, d.qvel[vi:vi+3].copy(), rotation(q).apply(d.qvel[vi+3:vi+6]))
        return result
    trace = trace_buffer(scene); start = time.perf_counter(); maximum = 0; peak = 0.; finite = True
    force_peak = 0.; contact_steps = 0
    n = len(target['time']) - 1
    for k in range(n + 1):
        st = states()
        if k % 10 == 0 or k == n: sample(trace, st, k * scene['dt'])
        if k == n: break
        d.xfrc_applied[:] = 0
        for b in scene['bodies']:
            if b['driven']:
                f, tau = wrench(scene, b, target, st[b['name']], k)
                d.xfrc_applied[ids[b['name']]] = np.r_[f, tau]
        mj.mj_step(m, d)
        maximum = max(maximum, d.ncon); contact_steps += bool(d.ncon)
        if d.ncon:
            peak = max(peak, -min(float(c.dist) for c in d.contact[:d.ncon]))
            for i in range(d.ncon):
                cf = np.zeros(6); mj.mj_contactForce(m, d, i, cf); force_peak = max(force_peak, np.linalg.norm(cf[:3]))
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all(): finite = False; break
        if k % 1000 == 0: print(f'mujoco step {k}/{n}', flush=True)
    warnings = {str(i): int(w.number) for i, w in enumerate(d.warning) if w.number}
    finite = finite and not any(d.warning[int(w)].number for w in (mj.mjtWarning.mjWARN_BADQACC,mj.mjtWarning.mjWARN_BADQPOS,mj.mjtWarning.mjWARN_BADQVEL))
    health = {'engine': 'MuJoCo CPU', 'version': mj.__version__, 'device': 'GPU host CPU', 'steps': k,
              'expected_steps': n, 'finite': finite, 'wall_s': time.perf_counter() - start,
              'warnings': warnings, 'max_contacts': maximum, 'contact_steps': contact_steps,
              'engine_peak_overlap_m': max(0., peak), 'peak_contact_force_N': float(force_peak),
              'native': {'integrator': int(m.opt.integrator), 'iterations': int(m.opt.iterations), 'tolerance': float(m.opt.tolerance),
                         'geom_solref': m.geom_solref.tolist(), 'geom_solimp': m.geom_solimp.tolist(), 'geom_friction': m.geom_friction.tolist(),
                         'body_mass': m.body_mass.tolist(), 'body_inertia': m.body_inertia.tolist()}}
    return finish(trace), health


def sap_config(scene, objects):
    bodies, joints, arts = [], [], []
    def tf(p, q): return {'p': list(p), 'q': [q[1], q[2], q[3], q[0]]}
    mat = {'mu': scene['mu'], 'mu_torsional': 0., 'mu_rolling': 0., 'density': 0.}
    for b in scene['bodies']:
        a = scene['assets'][b['asset']]
        bodies.append({'id': b['name'], 'transform': tf(b['p'], b['q']), 'mass': a['mass'], 'inertia': a['inertia'],
                       'shapes': [{'type': 'convex_mesh', 'label': b['name'] + f'_{i}', 'path': p, 'is_solid': True, 'cfg': mat}
                                  for i, p in enumerate(a['pieces'])]})
        j = b['name'] + '_free'; joints.append({'id': j, 'type': 'free', 'parent': 'world', 'child': b['name']})
        arts.append({'id': b['name'] + '_art', 'joints': [j]})
    if objects:
        bodies.append({'id': 'table', 'transform': tf([0., -.05, .74], [1., 0., 0., 0.]),
                       'shapes': [{'type': 'box', 'hx': .7, 'hy': .55, 'hz': .025, 'cfg': mat}]})
    return {'schema_version': 1, 'name': 'robodojo_reduced_gripper_pilot',
            'simulation': {'dt': scene['dt'], 'num_worlds': 1, 'nconmax': scene['contact_capacity']},
            'builder': {'gravity': -9.81}, 'ground': {'enabled': False}, 'bodies': bodies, 'joints': joints, 'articulations': arts}


def run_sap(scene, target, out, objects):
    import warp as wp
    from sap_warp import SolverSAP, load_sap_scene
    from sap_warp.collision.pipeline import SapCollisionPipeline
    from sap_warp.resources.collision_model import sap_collision_state_from_state
    from sap_warp.sap_kinematics import sap_eval_fk
    if os.environ.get('ROBODOJO_WARP_CACHE'):wp.config.kernel_cache_dir=os.environ['ROBODOJO_WARP_CACHE']
    wp.init()
    cfg = sap_config(scene, objects); path = out / 'sap_scene.json'; path.write_text(json.dumps(cfg, indent=2))
    cap = scene['contact_capacity']; loaded = load_sap_scene(path, device='cuda:0', naconmax=cap, strict=True)
    model, state, control = loaded.sap_model, loaded.sap_state, loaded.sap_control; other = model.state()
    bi = {b['name']: list(model.body_label).index(b['name']) for b in scene['bodies']}
    jl = list(model.joint_label); qs = model.joint_q_start.numpy();vs=model.joint_qd_start.numpy()
    jointq = state.joint_q.numpy()
    jointv=state.joint_qd.numpy();jointv[:]=0
    for b in scene['bodies']:
        a = qs[jl.index(b['name'] + '_free')]
        jointq[a:a+7] = np.r_[b['p'], np.asarray(b['q'])[[1, 2, 3, 0]]]
        vi=vs[jl.index(b['name']+'_free')];jointv[vi:vi+6]=np.r_[b.get('v0',[0.,0.,0.]),b.get('w0',[0.,0.,0.])]
    state.joint_q.assign(jointq); state.joint_qd.assign(jointv); sap_eval_fk(model, state.joint_q, state.joint_qd, state)
    for b in scene['bodies']:
        if not np.allclose(state.body_qd.numpy()[bi[b['name']]],np.r_[b.get('v0',[0.,0.,0.]),b.get('w0',[0.,0.,0.])],atol=1e-5):raise RuntimeError('SAP initial world twist mismatch')
    # Match the source robot's disabled finger self-collision in every backend.
    cm = loaded.collision_model; shape_body = cm.shape_body.numpy()
    excluded = set()
    for side in ('left', 'right'):
        if side+'_link7' not in bi:continue
        a = np.flatnonzero(shape_body == bi[side+'_link7']); b = np.flatnonzero(shape_body == bi[side+'_link8'])
        excluded.update(tuple(sorted((int(i),int(j)))) for i in a for j in b)
    cm.shape_collision_filter_pairs.update(excluded)
    pairs = [p for p in cm.shape_contact_pairs.numpy() if tuple(sorted(map(int,p))) not in excluded]
    from dataclasses import replace
    cm = replace(cm, shape_contact_pairs=wp.array(np.asarray(pairs,dtype=np.int32).reshape(-1,2),dtype=wp.vec2i,device=cm.device),
                 shape_contact_pair_count=len(pairs))
    solver = SolverSAP(model, nconmax=cap); pipe = SapCollisionPipeline(cm, naconmax=cap, broad_phase='explicit'); contacts = pipe.contacts()
    def states():
        q, v = state.body_q.numpy(), state.body_qd.numpy()
        return {name: (q[i,:3].copy(), q[i,[6,3,4,5]].copy(), v[i,:3].copy(), v[i,3:6].copy()) for name,i in bi.items()}
    trace = trace_buffer(scene); start = time.perf_counter(); peak = 0.; maximum = 0; truncated = 0; finite = True
    contact_steps = 0; mu_seen = []
    n = len(target['time']) - 1
    for k in range(n + 1):
        st = states()
        if k % 10 == 0 or k == n: sample(trace, st, k * scene['dt'])
        if k == n: break
        state.clear_forces(); force = state.body_f.numpy(); force[:] = 0
        for b in scene['bodies']:
            if b['driven']:
                f, tau = wrench(scene, b, target, st[b['name']], k); force[bi[b['name']]] = np.r_[f, tau]
        state.body_f.assign(force)
        pipe.collide(sap_collision_state_from_state(state), contacts)
        solver.step(state, other, control, contacts, scene['dt'])
        cj = solver.contact_jacobian; count = int(cj.contact_env_count_wp.numpy().ravel()[0])
        maximum = max(maximum, count); contact_steps += bool(count)
        truncated = max(truncated, int(solver.last_truncated_contact_count or 0))
        if count:
            phi = cj.contact_env_phi0_wp.numpy()[0, :count]; peak = max(peak, -float(phi.min()))
            if not mu_seen and hasattr(cj, 'contact_env_mu_wp'): mu_seen = np.unique(cj.contact_env_mu_wp.numpy()[0,:count]).tolist()
        state, other = other, state
        if not np.isfinite(state.body_q.numpy()).all() or not np.isfinite(state.body_qd.numpy()).all(): finite = False; break
        if k % 1000 == 0: print(f'sap step {k}/{n}', flush=True)
    native = {}
    for attr in ('shape_material_ke', 'shape_material_kd', 'shape_material_mu', 'shape_material_tau', 'body_mass', 'body_inertia'):
        val = getattr(model, attr, None)
        if val is not None: native[attr] = val.numpy().tolist() if hasattr(val, 'numpy') else str(val)
    for attr in ('shape_material_ke','shape_material_tau','shape_material_mu','shape_margin','shape_gap'):
        val = getattr(cm,attr,None)
        if val is not None: native['collision_'+attr] = val.numpy().tolist()
    import sap_warp,subprocess
    package=Path(sap_warp.__file__).resolve().parent
    git=subprocess.run(['git','-C',str(package.parents[1]),'rev-parse','HEAD'],text=True,capture_output=True)
    commit=git.stdout.strip() if git.returncode==0 else None
    fingerprint=hashlib.sha256()
    for file in sorted(package.rglob('*.py')):
        fingerprint.update(str(file.relative_to(package)).encode());fingerprint.update(file.read_bytes())
    return finish(trace), {'engine': 'SAP Warp', 'source_commit': commit,'private_source_tree_sha256':fingerprint.hexdigest(),
                           'device': wp.get_device('cuda:0').name, 'warp_version': wp.__version__,
                           'steps': k, 'expected_steps': n, 'finite': finite, 'wall_s': time.perf_counter()-start,
                           'max_contacts': maximum, 'contact_steps': contact_steps, 'truncated': truncated,
                           'engine_peak_overlap_m': max(0., peak), 'native': native, 'observed_pair_mu': mu_seen}


def run_ipc(scene, target, out, objects):
    import uipc
    import trimesh
    from uipc.core import Engine, Scene, World, AffineBodyStateAccessorFeature
    from uipc.geometry import trimesh as umesh, label_surface, ground
    from uipc.constitution import AffineBodyConstitution, AffineBodyExternalBodyForce
    uipc.Logger.set_level(uipc.Logger.Level.Warn)
    cfg = Scene.default_config(); cfg['dt'] = scene['dt']; cfg['gravity'] = [[0.], [0.], [-9.81]]
    numerical=scene.get('ipc_parameters',{})
    for key in ('d_hat','eps_velocity'):
        if key in numerical:cfg['contact'][key]=numerical[key]
    if 'velocity_tol' in numerical:cfg['newton']['velocity_tol']=numerical['velocity_tol']
    eng = Engine('cuda', str(out / 'workspace')); world = World(eng); sc = Scene(cfg)
    abd, ext = AffineBodyConstitution(), AffineBodyExternalBodyForce()
    sc.constitution_tabular().insert(abd); sc.constitution_tabular().insert(ext)
    tab = sc.contact_tabular(); tab.default_model(scene['mu'], 1e9)
    elements = {name:tab.create(name) for name in ('left','right','object','env')}
    for i,a in enumerate(elements):
        for b in list(elements)[i:]:
            tab.insert(elements[a],elements[b],scene['mu'],1e9,not (a == b and a in ('left','right')))
    slots, forces, keep = {}, {}, [abd, ext]
    for b in scene['bodies']:
        a = scene['assets'][b['asset']]
        merged = trimesh.util.concatenate([trimesh.load(p, force='mesh', process=False) for p in a['pieces']])
        mesh = umesh(np.asarray(merged.vertices, float), np.asarray(merged.faces, np.int32)); label_surface(mesh)
        abd.apply_to(mesh, 1e8, mass_density=a['mass'] / a['additive_volume_m3'])
        elements[b['name'].split('_')[0] if b['driven'] else 'object'].apply_to(mesh)
        T = np.eye(4); T[:3,:3] = rotation(b['q']).as_matrix(); T[:3,3] = b['p']; uipc.view(mesh.transforms())[0] = T
        if 'v0' in b or 'w0' in b:
            w=np.asarray(b.get('w0',[0.,0.,0.]));x,y,z=w;skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
            velocity=np.zeros((4,4));velocity[:3,:3]=skew@T[:3,:3];velocity[:3,3]=b.get('v0',[0.,0.,0.])
            attr=mesh.instances().find(uipc.builtin.velocity)
            if attr is None:attr=mesh.instances().create(uipc.builtin.velocity,np.zeros((4,4)))
            uipc.view(attr)[0]=velocity
        obj = sc.objects().create(b['name']); keep.extend([mesh,obj])
        if b['driven']:
            ext.apply_to(mesh, np.zeros(12), enable=True); forces[b['name']] = np.zeros(12)
            def animate(info, name=b['name']):
                for s in info.geo_slots():
                    array = uipc.view(s.geometry().instances().find('external_force')); array[:] = forces[name].reshape(array.shape)
            sc.animator().insert(obj, animate)
        slot, _ = obj.geometries().create(mesh); slots[b['name']] = slot
    if objects:
        g = ground(scene['table_top_z'], np.array([[0.],[0.],[1.]])); elements['env'].apply_to(g)
        obj = sc.objects().create('table'); obj.geometries().create(g); keep.extend([g,obj])
    world.init(sc)
    if not world.is_valid(): raise RuntimeError('IPC invalid initial scene')
    acc = world.features().find(AffineBodyStateAccessorFeature); geo = acc.create_geometry()
    geo.instances().create(uipc.builtin.transform, np.eye(4)); geo.instances().create(uipc.builtin.velocity, np.zeros((4,4)))
    offsets = {name:int(np.asarray(uipc.view(s.geometry().meta().find(uipc.builtin.backend_abd_body_offset))).ravel()[0]) for name,s in slots.items()}
    def states():
        acc.copy_to(geo); transforms = uipc.view(geo.instances().find(uipc.builtin.transform)); velocities = uipc.view(geo.instances().find(uipc.builtin.velocity))
        st, aff = {}, {}
        for name,i in offsets.items():
            T, V = transforms[i], velocities[i]; A = T[:3,:3]; U,_,Vt = np.linalg.svd(A); rot = R.from_matrix(U@Vt)
            spin = V[:3,:3] @ np.linalg.inv(A); w = np.array([spin[2,1]-spin[1,2],spin[0,2]-spin[2,0],spin[1,0]-spin[0,1]])/2
            st[name] = (T[:3,3].copy(), rot.as_quat()[[3,0,1,2]], V[:3,3].copy(), w); aff[name] = A.copy()
        return st, aff
    trace = trace_buffer(scene); start = time.perf_counter(); deform = 0.; n = len(target['time']) - 1
    for k in range(n + 1):
        st, aff = states()
        if k % 10 == 0 or k == n:
            sample(trace, st, k*scene['dt']); deform = max(deform, max(np.linalg.norm(A.T@A-np.eye(3)) for A in aff.values()))
        if k == n: break
        for b in scene['bodies']:
            if b['driven']:
                f,tau = wrench(scene,b,target,st[b['name']],k); x,y,z = tau
                skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                forces[b['name']][:] = np.r_[f, (.5*skew@np.linalg.inv(aff[b['name']]).T).ravel()]
        world.advance(); world.retrieve()
        if not world.is_valid(): break
        if k % 1000 == 0: print(f'ipc step {k}/{n}', flush=True)
    return finish(trace), {'engine': 'libuipc ABD', 'version': importlib.metadata.version('pyuipc'), 'device': 'CUDA',
                           'finite': bool(world.is_valid()), 'steps': k, 'expected_steps': n, 'wall_s': time.perf_counter()-start,
                           'affine_deformation_max': float(deform), 'native_config': cfg,
                           'required_model_constants': {'abd_kappa_Pa': 1e8, 'contact_stiffness_requested_Pa': 1e9},
                           'contact_stiffness_runtime': 'native engine may clamp requested kappa; retain per-model clamp warnings in log',
                           'engine_peak_overlap_m': None, 'contact_capacity_audited': False,
                           'table_representation': 'plane at common finite table top; all bodies stay inside footprint'}


def main():
    resource.setrlimit(resource.RLIMIT_CORE, (0,0))
    ap = argparse.ArgumentParser(); ap.add_argument('--scene', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--engine', choices=['mujoco','mujoco_warp','sap','ipc'], required=True); ap.add_argument('--drive-only', action='store_true')
    ap.add_argument('--sap-graph',action='store_true')
    ap.add_argument('--max-steps',type=int,default=None,help='bounded adapter validation only')
    ap.add_argument('--params',default=None,help='JSON object merged into scene (engine-numerical settings only)')
    args = ap.parse_args(); out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    scene, target = load(args.scene, args.drive_only)
    if args.params:
        extra=json.loads(Path(args.params).read_text() if args.params.endswith('.json') else args.params)
        for k,v in extra.items():
            if k not in ('mujoco_parameters','sap_parameters','contact_capacity','why'):raise ValueError('only engine-numerical keys may be overridden: '+k)
            scene[k]=v
    source_hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in Path(__file__).parent.glob('*.py')}
    if args.max_steps is not None:
        if args.max_steps<1:raise ValueError('max-steps must be positive')
        target={k:target[k][:args.max_steps+1] for k in target}
        scene['adapter_validation_steps']=args.max_steps
    (out / 'input_scene.json').write_text(json.dumps(scene,indent=2))
    if scene.get('articulated_grippers'):
        from . import articulated
        trace,health=getattr(articulated,'run_'+args.engine)(scene,target,out,not args.drive_only)
    elif args.engine=='sap' and args.sap_graph:
        from .sap_graph import run
        trace,health=run(scene,target,out,not args.drive_only)
    else:
        trace, health = globals()['run_' + args.engine](scene, target, out, not args.drive_only)
    if args.engine=='sap':
        import sap_warp
        from sap_warp.collision import core as collision_core
        health.update(sap_module_path=str(Path(sap_warp.__file__).resolve()),
                      convex_backend=getattr(collision_core,'SAP_CONVEX_BACKEND','legacy_unversioned'),
                      collision_core_sha256=hashlib.sha256(Path(collision_core.__file__).read_bytes()).hexdigest())
    health['source_sha256'] = SOURCE_SHA256
    health['package_sources_sha256'] = source_hashes
    health['task_deviations'] = scene['deviations']
    health['collision_exclusions'] = ['left_link7/left_link8','right_link7/right_link8']
    save(out, scene, target, trace, health)


if __name__ == '__main__': main()
