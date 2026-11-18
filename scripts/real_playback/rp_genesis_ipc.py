"""Placement replay through the Genesis IPC coupler. See experiment README for limitations."""
from __future__ import annotations

import os
import tempfile
import time
from pathlib import Path

import numpy as np

from .rp_engines import TRACE_HZ, drive_cfg, initial_state, nums, pad_local_center, base_wrench_batch
from .rp_sim import Playback, finger_force, place_on_table, qmat, qmul, table_pose



_GS = None


def gs_init():
    global _GS
    if _GS is None:
        import genesis as gs
        gs.init(backend=gs.cuda, precision='64', seed=20261117, logging_level=os.environ.get('GS_LOG', 'warning'))
        _GS = gs
    return _GS


def sqrt_mu(mu_pb, mu_be):
    """Entity friction for a geometric-mean combiner reproducing two pair values."""
    b = max(mu_pb, mu_be)
    p, e = mu_pb ** 2 / b, mu_be ** 2 / b
    return {'pad': p, 'basket': b, 'env': e, 'pad_env_derived': float(np.sqrt(p * e))}


def np_(x):
    return x.detach().cpu().numpy() if hasattr(x, 'detach') else np.asarray(x)


def run_genesis_ipc(scene, meta, pbdata, candidate, perturbs, workdir, objects=True):
    gs = gs_init()
    from scipy.spatial.transform import Rotation as R
    Rn = len(perturbs)
    if Rn != 1 or np.any(np.asarray(perturbs)):
        raise ValueError('This exploratory adapter accepts one nominal replay per process')
    workdir = Path(workdir); workdir.mkdir(parents=True, exist_ok=True)
    par = dict(candidate.get('parameters', {}))
    simopt = gs.options.SimOptions(dt=scene['dt'], substeps=1, gravity=(0., 0., -9.81))
    ip = {}
    for a, b in [('d_hat','contact_d_hat'), ('eps_v','contact_eps_velocity'), ('newton_tol','newton_tolerance')]:
        if a in par: ip[b] = par[a]
    strength = par.get('strength', 100.)
    ip.update(constraint_strength_translation=strength, constraint_strength_rotation=strength)
    gscene = gs.Scene(sim_options=simopt, coupler_options=gs.options.IPCCouplerOptions(**ip), show_viewer=False)
    mus = sqrt_mu(scene['mu']['pad_basket'], scene['mu']['basket_env'])
    d = drive_cfg(scene)
    pieces = meta['pieces']
    ph = np.asarray(scene['pad']['size']) / 2
    Ip = scene['pad']['mass'] / 3 * (np.sum(ph ** 2) - ph ** 2)

    def mj(body, assets=''):
        return f'<mujoco><compiler angle="radian"/><asset>{assets}</asset><worldbody>{body}</worldbody></mujoco>'
    pads = ''.join(
        f'<body name="{n}" pos="{nums(pad_local_center(scene, sd, 0.))}">'
        f'<joint name="{n}_slide" type="slide" axis="0 {sd} 0" limited="true" range="0 {scene["pad"]["stroke"] / 2}" armature="0"/>'
        f'<inertial pos="0 0 0" mass="{scene["pad"]["mass"]}" diaginertia="{nums(Ip)}"/>'
        f'<geom type="mesh" mesh="pad"/></body>' for sd, n in ((-1, 'left'), (1, 'right')))
    grip = mj(f'<body name="base" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="{d["base_mass"]}" '
              f'diaginertia="{nums([d["base_inertia"]] * 3)}"/>{pads}</body>', f'<mesh name="pad" file="{pieces["pad"]}"/>')
    ents = {}
    files = {}
    tmp = Path(tempfile.mkdtemp(prefix='rpgen-', dir='/dev/shm' if Path('/dev/shm').is_dir() else None))

    def add(name, xml, mu):
        f = tmp / f'{name}.xml'; f.write_text(xml); files[name] = xml
        ents[name] = gscene.add_entity(gs.morphs.MJCF(file=str(f), default_armature=None, convexify=True, decimate=False,
                                                      decompose_object_error_threshold=float('inf')),
                                       material=gs.materials.Rigid(friction=mu, coup_friction=mu, rho=(scene['pad']['mass'] / np.prod(scene['pad']['size']) if name == 'gripper' else scene['basket']['mass'] / meta['basket_volume'] if name == 'basket' else 1000.), coup_type='two_way_soft_constraint' if name == 'gripper' else 'ipc_only', needs_coup=True))
    add('gripper', grip, mus['pad'])
    if objects:
        I = np.asarray(meta['basket_inertia_per_kg']) * scene['basket']['mass']
        bas = ''.join(f'<mesh name="bk{i}" file="{p}"/>' for i, p in enumerate(pieces['basket']))
        bg = ''.join(f'<geom type="mesh" mesh="bk{i}"/>' for i in range(len(pieces['basket'])))
        add('basket', mj(f'<body name="basket" pos="0 0 .5"><freejoint/><inertial pos="0 0 0" mass="{scene["basket"]["mass"]}" '
                         f'fullinertia="{nums([I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]])}"/>{bg}</body>', bas), mus['basket'])
        rp, rq = place_on_table(scene, scene['rack']['xy'], scene['rack']['yaw'])
        ras = ''.join(f'<mesh name="rk{i}" file="{p}"/>' for i, p in enumerate(pieces['rack']))
        rg = ''.join(f'<geom type="mesh" mesh="rk{i}"/>' for i in range(len(pieces['rack'])))
        add('rack', mj(f'<body name="rack" pos="{nums(rp)}" quat="{nums(rq)}">{rg}</body>', ras), mus['env'])
        tc, tR = table_pose(scene)
        tq = R.from_matrix(tR).as_quat()[[3, 0, 1, 2]]
        th = scene['table']['box_thickness']
        add('table', mj(f'<body name="table" pos="{nums(tc)}" quat="{nums(tq)}"><geom type="box" size="{nums([.6, .6, th / 2])}"/></body>'), mus['env'])
    pb = Playback(pbdata, scene['finger_rule'])
    if objects:
        t0, p, q, width, bpos, bq = initial_state(scene, pb, meta)
    else:
        t0 = scene['t_start']; p, q, _, _, width, _ = pb.target(t0)
    import uipc
    uipc.Logger.set_level(uipc.Logger.Level.Warn)
    uipc.Timer.disable_all()
    cp = gscene.sim.coupler
    finalize = cp._finalize_ipc
    def init_ipc():
        for link, slots in cp._abd_slots_by_link.items():
            if link.entity is ents.get('basket'):
                pos, quat = bpos, bq
            elif link.entity is ents['gripper']:
                side = -1 if 'left' in link.name else 1
                pos, quat = p + qmat(q) @ pad_local_center(scene, side, width / 2), q
            else:
                continue
            T = np.eye(4); T[:3,:3] = qmat(quat); T[:3,3] = pos
            for slot in slots: uipc.view(slot.geometry().transforms())[0] = T
        # Fixed fixtures carry no relative dynamics; Genesis' rigid solver also skips fixed-fixed contact.
        # Disable this pair explicitly so the IPC sanity checker permits the rack's prescribed table attachment.
        if objects:
            cp._ipc_contact_tabular.insert(cp._ipc_abd_contacts[ents['rack']], cp._ipc_abd_contacts[ents['table']], 0., 1e9, False)
        audit = {}; arrays = {}
        for link, slots in cp._abd_slots_by_link.items():
            for j, slot in enumerate(slots):
                geo=slot.geometry(); T=np.asarray(uipc.view(geo.transforms()))[0]
                vs=np.asarray(geo.vertices().find('position').view()).reshape(-1,3)
                fs=np.asarray(geo.triangles().find('topo').view()).reshape(-1,3)
                entity_name = next(n for n,e in ents.items() if e is link.entity)
                key=f'{entity_name}_{link.name}_{j}'
                arrays[key+'_v']=vs@T[:3,:3].T+T[:3,3]; arrays[key+'_f']=fs
                audit[key]={'n_vertices':len(vs),'n_faces':len(fs),'transform':T.tolist(),'rho':link.entity.material.rho,'fixed':bool(link.is_fixed)}
        import json
        (workdir/'ipc_initial_meshes.json').write_text(json.dumps(audit,indent=2))
        np.savez_compressed(workdir/'ipc_initial_meshes.npz',**arrays)
        finalize()
        if not cp._ipc_world.is_valid():
            raise RuntimeError('IPC initialization rejected: intersecting geometry (native sanity check enabled)')
    cp._finalize_ipc = init_ipc
    gscene.build(n_envs=Rn)
    G, B = ents['gripper'], ents.get('basket')
    basket_link = next((l for l in cp._abd_slots_by_link if l.entity is B), None)
    pad_links = [next(l for l in cp._abd_slots_by_link if l.entity is G and nm in l.name) for nm in ('left','right')]
    max_rigid_contacts = 0
    max_affine_deformation = 0.
    qg = np.tile(np.r_[p, q, width / 2, width / 2], (Rn, 1))
    G.set_qpos(qg, zero_velocity=True)
    if objects:
        _, tR = table_pose(scene)
        qb = np.zeros((Rn, 7))
        for w, pr in enumerate(perturbs):
            rz = R.from_rotvec(tR[:, 2] * pr[2]).as_quat()[[3, 0, 1, 2]]
            qb[w] = np.r_[bpos + tR[:, 0] * pr[0] + tR[:, 1] * pr[1], qmul(rz, bq)]
        B.set_qpos(qb, zero_velocity=True)
    if not cp._ipc_world.is_valid():
        raise RuntimeError('IPC world rejected the initial geometry; see sanity_check artifacts')
    cp._retrieve_rigid_states()
    n = int(round((scene['t_end'] - scene['t_start']) / scene['dt']))
    every = max(1, int(round(1 / (TRACE_HZ * scene['dt']))))
    traces = [[] for _ in range(Rn)]
    t = t0
    tstart = time.perf_counter()
    slide_dofs = [6, 7]

    def record():
        lp = np_(G.get_links_pos()); lq = np_(G.get_links_quat())  # (R, 3 links, 3/4), quat wxyz
        sl = np_(G.get_qpos())[:, 7:9]
        if objects:
            from scipy.spatial.transform import Rotation
            if cp._abd_data_by_link and basket_link in cp._abd_data_by_link:
                states = cp._abd_data_by_link[basket_link]
                bqp, bv, bw = [], [], []
                for st in states:
                    A = st.transform[:3,:3]; U, _, Vt = np.linalg.svd(A); rot = U @ Vt
                    spin = st.velocity[:3,:3] @ np.linalg.inv(A)
                    bqp.append(np.r_[st.transform[:3,3], Rotation.from_matrix(rot).as_quat()[[3,0,1,2]]])
                    bv.append(st.velocity[:3,3]); bw.append([spin[2,1]-spin[1,2],spin[0,2]-spin[2,0],spin[1,0]-spin[0,1]])
                bqp, bv, bw = np.asarray(bqp), np.asarray(bv), np.asarray(bw)/2
            else:
                bqp = np.tile(np.r_[bpos,bq],(Rn,1)); bv = bw = np.zeros((Rn,3))
        nonlocal max_rigid_contacts, max_affine_deformation
        c = gscene.rigid_solver.collider.get_contacts(as_tensor=True, to_torch=False)
        ga = np.asarray(c['geom_a']); max_rigid_contacts = max(max_rigid_contacts, int((ga >= 0).sum(axis=-1).max()) if ga.size else 0)
        for sts in cp._abd_data_by_link.values():
            for st in sts:
                A=st.transform[:3,:3]; max_affine_deformation=max(max_affine_deformation,float(np.linalg.norm(A.T@A-np.eye(3))))
        for w in range(Rn):
            ipc_pads = []
            for i,l in enumerate(pad_links):
                if l in cp._abd_data_by_link:
                    T=cp._abd_data_by_link[l][w].transform; U,_,Vt=np.linalg.svd(T[:3,:3])
                    ipc_pads.append(np.r_[T[:3,3],R.from_matrix(U@Vt).as_quat()[[3,0,1,2]]])
                else: ipc_pads.append(np.r_[lp[w,i+1],lq[w,i+1]])
            s = {'t': t, 'base': np.r_[lp[w, 0], lq[w, 0]], 'pads': ipc_pads,
                 'slider': [float(sl[w, 0]), float(sl[w, 1])]}
            if objects:
                s['basket'] = bqp[w].astype(float); s['basket_v'] = np.r_[bv[w], bw[w]]
            traces[w].append(s)
    record()
    k = 0
    for k in range(n):
        tp, tq, tv, tw, width, closed = pb.target(t)
        lp = np_(G.get_links_pos(links_idx_local=[0]))[:, 0]; lq = np_(G.get_links_quat(links_idx_local=[0]))[:, 0]
        lv = np_(G.get_links_vel(links_idx_local=[0]))[:, 0]; la = np_(G.get_links_ang(links_idx_local=[0]))[:, 0]
        f, tau = base_wrench_batch(lp, lq, lv, la, tp, tq, tv, tw, d)
        G.apply_links_external_wrench(force=f[:, None, :], torque=tau[:, None, :], links_idx_local=[0])
        qs = np_(G.get_qpos())[:, 7:9]; qd = np_(G.get_dofs_velocity(dofs_idx_local=slide_dofs))
        ff = np.array([[finger_force(qs[w, c], qd[w, c], width, closed, d) for c in range(2)] for w in range(Rn)])
        G.control_dofs_force(ff, dofs_idx_local=slide_dofs)
        gscene.step()
        t += scene['dt']
        if (k + 1) % every == 0:
            record()
            if (k+1) % max(1, round(.5 / scene['dt'])) == 0:
                print('PROGRESS', k+1, '/', n, 'wall_s', round(time.perf_counter()-tstart,1), flush=True)
                import gzip,pickle
                with gzip.open(workdir / 'partial.pkl.gz','wb') as f: pickle.dump(traces[0],f)
            if not all(np.isfinite(tr[-1]['base']).all() for tr in traces):
                break
    wall = time.perf_counter() - tstart
    healths = []
    for w in range(Rn):
        fin = bool(all(np.isfinite(s['base']).all() and (not objects or np.isfinite(s['basket']).all()) for s in traces[w][-3:]))
        healths.append({'finite': fin, 'truncated': 0, 'wall_s': wall / Rn, 'wall_batch_s': wall, 'steps': k + 1,
                        'friction_entities': mus, 'engine_peak_overlap_m': {},
                        'engine': 'Genesis 1.4.6 + IPC 0.0.29', 'max_rigid_solver_contacts': max_rigid_contacts,
                        'affine_deformation_max':max_affine_deformation,
                        'coupler_options': {k:str(v) for k,v in cp.options.model_dump().items()},
                        'routes': {'basket':'ipc_only', 'rack':'ipc_only', 'table':'ipc_only', 'pads':'two_way_soft_constraint'},
                        'pad_trace':'IPC transforms; slider values from Genesis joints',
                        'ipc_mass_kg':{'basket':scene['basket']['mass'],'pad':scene['pad']['mass']}})
    return traces, healths
