"""IPC (libuipc 0.0.29, affine bodies) backend for RH20T dish-on-rack playback.

Same assets, drive law and observer as the MuJoCo / SAP backends.  The base
wrench enters as AffineBodyExternalBodyForce: force in the translational
slots and torque as the virtual-work-equivalent affine generalised force
0.5*[tau]x A^-T (row-major), verified by a free-body probe (ratio 1.01).
Finger forces are applied to each pad along the slide axis with the reaction
on the base.  IPC cannot start at zero distance: resting contacts start one
d_hat apart (registry a3 start-gap rule).  One rollout per process.
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from .rp_engines import TRACE_HZ, drive_cfg, initial_state, shape_mu
from .rp_sim import (Playback, base_wrench, finger_force, hull, box_pts, load_obj, mass_props, place_on_table,
                     qmat, qmul, table_pose)

START_GAP_MAX = 1.2e-3  # a3 rule: start one d_hat off the table, capped so the rim does not reach the pads

CANDIDATES = {
    'native': {},
    'dh1_nt': {'d_hat': 1e-3, 'newton_tol': 1e-4},
    'dh1_ev_nt': {'d_hat': 1e-3, 'eps_v': 1e-3, 'newton_tol': 1e-4},
    'dh03_ev_nt': {'d_hat': 3e-4, 'eps_v': 1e-3, 'newton_tol': 1e-4},
    'dh01_ev_nt': {'d_hat': 1e-4, 'eps_v': 1e-3, 'newton_tol': 1e-4},
}


def skew(t):
    x, y, z = t
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0.]])


def merge(meshes):
    V, F, off = [], [], 0
    for v, f in meshes:
        V.append(v); F.append(f + off); off += len(v)
    return np.vstack(V), np.vstack(F).astype(np.int32)


def run_ipc(scene, meta, pbdata, candidate, perturb, workdir, objects=True, max_wall_s=6 * 3600):
    import uipc
    from uipc.core import Engine, Scene, World, AffineBodyStateAccessorFeature
    from uipc.geometry import trimesh, label_surface, ground, linemesh
    from uipc.constitution import (AffineBodyConstitution, AffineBodyExternalBodyForce,
                                   AffineBodyPrismaticJoint, AffineBodyPrismaticJointLimit)
    from scipy.spatial.transform import Rotation as R
    uipc.Logger.set_level(uipc.Logger.Level.Warn)
    workdir = Path(workdir); workdir.mkdir(parents=True, exist_ok=True)
    par = candidate.get('parameters', {})
    cfg = Scene.default_config()
    cfg['dt'] = scene['dt']
    cfg['gravity'] = [[0.0], [0.0], [-9.81]]
    if 'd_hat' in par:
        cfg['contact']['d_hat'] = par['d_hat']
    if 'eps_v' in par:
        cfg['contact']['eps_velocity'] = par['eps_v']
    if 'newton_tol' in par:
        cfg['newton']['velocity_tol'] = par['newton_tol']
    d_hat = cfg['contact']['d_hat']
    eng = Engine('cuda', str(workdir / 'ws'))
    world = World(eng)
    sc = Scene(cfg)
    abd, ext = AffineBodyConstitution(), AffineBodyExternalBodyForce()
    joint, limit = AffineBodyPrismaticJoint(), AffineBodyPrismaticJointLimit()
    for c in (abd, ext, joint, limit):
        sc.constitution_tabular().insert(c)
    keep = [abd, ext, joint, limit]
    tab = sc.contact_tabular()
    tab.default_model(0.0, 1e9)
    el = {n: tab.create(n) for n in ('base', 'pad', 'basket', 'env', 'table')}
    mus = scene['mu']
    pairs = {('pad', 'basket'): mus['pad_basket'], ('basket', 'env'): mus['basket_env'], ('basket', 'table'): mus['basket_env'],
             ('pad', 'env'): mus['pad_env'], ('pad', 'table'): mus['pad_env']}
    names = list(el)
    for i, a in enumerate(names):
        for b in names[i:]:
            key = (a, b) if (a, b) in pairs else (b, a)
            on = key in pairs
            tab.insert(el[a], el[b], float(pairs.get(key, 0.0)), 1e9, on)
    pb = Playback(pbdata, scene['finger_rule'])
    if objects:
        t0, p, q, width, bpos, bq = initial_state(scene, pb, meta)
        _, tR = table_pose(scene)
        bpos = bpos + tR[:, 0] * perturb[0] + tR[:, 1] * perturb[1] + tR[:, 2] * min(d_hat + 1e-4, START_GAP_MAX)
        rz = R.from_rotvec(tR[:, 2] * perturb[2]).as_quat()[[3, 0, 1, 2]]
        bq = qmul(rz, bq)
    else:
        t0 = scene['t_start']; p, q, _, _, width, _ = pb.target(t0)
    d = drive_cfg(scene)
    slots, forces, offs = {}, {}, {}

    def Tm(pos, qw):
        T = np.eye(4); T[:3, :3] = qmat(qw); T[:3, 3] = pos; return T

    def add(name, v, f, T, mass, element, fixed=False, driven=True):
        mesh = trimesh(np.asarray(v, float), np.asarray(f, np.int32))
        label_surface(mesh)
        vol = mass_props([(np.asarray(v, float), np.asarray(f))])[0]
        abd.apply_to(mesh, 1e8, mass_density=(mass / vol) if mass else 1000.0)
        uipc.view(mesh.transforms())[0] = T
        uipc.view(mesh.instances().find(uipc.builtin.is_fixed))[:] = int(fixed)
        el[element].apply_to(mesh)
        obj = sc.objects().create(name)
        keep.extend((mesh, obj))
        if driven and not fixed:
            ext.apply_to(mesh, np.zeros(12), enable=True)
            forces[name] = np.zeros(12)

            def anim(info, name=name):
                for s in info.geo_slots():
                    a = uipc.view(s.geometry().instances().find('external_force'))
                    a[:] = forces[name].reshape(a.shape)
            sc.animator().insert(obj, anim)
        slot, _ = obj.geometries().create(mesh)
        slots[name] = slot
    hb = np.sqrt(6 * d['base_inertia'] / d['base_mass']) / 2
    bv, bf = hull(box_pts(np.zeros(3), np.full(3, 2 * hb)))
    add('base', bv, bf, Tm(p, q), d['base_mass'], 'base')
    pv, pf = load_obj(meta['pieces']['pad'])
    Rb = qmat(q)
    lines = []
    for side, name in ((-1, 'left'), (1, 'right')):
        c = np.array([0., side * (width / 2 + scene['pad']['size'][1] / 2), -scene['pad']['size'][2] / 2])
        add(name, pv, pf, Tm(p + Rb @ c, q), scene['pad']['mass'], 'pad')
        axis = Rb @ np.array([0., side, 0.])
        line = linemesh(np.array([p, p + axis * 0.05]), np.array([[0, 1]], dtype=np.int32))
        joint.apply_to(line, [slots['base']], np.array([0], np.int32), [slots[name]], np.array([0], np.int32), np.array([100.]))
        limit.apply_to(line, -width / 2, scene['pad']['stroke'] / 2 - width / 2, 100.)
        o = sc.objects().create(name + '_joint'); o.geometries().create(line)
        keep.extend((line, o))
    if objects:
        bmesh = merge([load_obj(x) for x in meta['pieces']['basket']])
        add('basket', bmesh[0], bmesh[1], Tm(bpos, bq), scene['basket']['mass'], 'basket', driven=False)
        rp, rq = place_on_table(scene, scene['rack']['xy'], scene['rack']['yaw'])
        rmesh = merge([load_obj(x) for x in meta['pieces']['rack']])
        add('rack', rmesh[0], rmesh[1], Tm(rp, rq), 0., 'env', fixed=True, driven=False)
        tc, tRm = table_pose(scene)
        n = tRm[:, 2]
        top = tc + n * scene['table']['box_thickness'] / 2
        g = ground(float(n @ top), n.reshape(3, 1))
        el['table'].apply_to(g)
        og = sc.objects().create('table'); og.geometries().create(g); keep.extend((g, og))
    world.init(sc)
    if not world.is_valid():
        return [], {'finite': False, 'truncated': 0, 'init_invalid': True, 'wall_s': 0., 'steps': 0,
                    'engine_peak_overlap_m': {}}
    acc = world.features().find(AffineBodyStateAccessorFeature)
    geo = acc.create_geometry()
    geo.instances().create(uipc.builtin.transform, np.eye(4, dtype=np.float64))
    geo.instances().create(uipc.builtin.velocity, np.zeros((4, 4), dtype=np.float64))
    for nm, s in slots.items():
        o = s.geometry().meta().find(uipc.builtin.backend_abd_body_offset)
        offs[nm] = int(np.asarray(uipc.view(o)).ravel()[0])

    def snap():
        acc.copy_to(geo)
        Ts = np.asarray(uipc.view(geo.instances().find(uipc.builtin.transform)))
        Vs = np.asarray(uipc.view(geo.instances().find(uipc.builtin.velocity)))
        out = {}
        for nm, o in offs.items():
            T = Ts[o]; A = T[:3, :3]
            U, _, Vt = np.linalg.svd(A); Rm = U @ Vt
            qw = R.from_matrix(Rm).as_quat()[[3, 0, 1, 2]]
            V = Vs[o]
            spin = V[:3, :3] @ np.linalg.inv(A)
            om = np.array([spin[2, 1] - spin[1, 2], spin[0, 2] - spin[2, 0], spin[1, 0] - spin[0, 1]]) / 2
            out[nm] = (T[:3, 3].copy(), qw, V[:3, 3].copy(), om, A.copy())
        return out
    nst = int(round((scene['t_end'] - scene['t_start']) / scene['dt']))
    every = max(1, int(round(1 / (TRACE_HZ * scene['dt']))))
    tr = []
    t = t0
    deform = 0.
    tstart = time.perf_counter()
    k = 0
    for k in range(nst + 1):
        st = snap()
        bp_, bq_, bv_, bw_, bA = st['base']
        Rb = qmat(bq_)
        sl, sld = [], []
        for side, nm in ((-1, 'left'), (1, 'right')):
            ax = Rb @ np.array([0., side, 0.])
            dlt = st[nm][0] - bp_ - Rb @ np.array([0., 0., -scene['pad']['size'][2] / 2])
            sl.append(float(dlt @ ax - scene['pad']['size'][1] / 2))
            rel = st[nm][2] - bv_ - np.cross(bw_, st[nm][0] - bp_)
            sld.append(float(rel @ ax))
        if k % every == 0:
            s = {'t': t, 'base': np.r_[bp_, bq_], 'pads': [np.r_[st[n][0], st[n][1]] for n in ('left', 'right')], 'slider': sl}
            if objects:
                s['basket'] = np.r_[st['basket'][0], st['basket'][1]]
                s['basket_v'] = np.r_[st['basket'][2], st['basket'][3]]
            tr.append(s)
            deform = max(deform, max(float(np.linalg.norm(x[4].T @ x[4] - np.eye(3))) for x in st.values()))
            if not np.isfinite(s['base']).all() or time.perf_counter() - tstart > max_wall_s:
                break
        if k == nst:
            break
        tp, tq, tv, tw, wdt, closed = pb.target(t)
        f, tau = base_wrench(bp_, bq_, bv_, bw_, tp, tq, tv, tw, d)
        wb = [f.copy(), tau.copy()]
        for side, nm, qj, qdj in ((-1, 'left', sl[0], sld[0]), (1, 'right', sl[1], sld[1])):
            Fj = finger_force(qj, qdj, wdt, closed, d)
            Fv = Rb @ np.array([0., side * Fj, 0.])
            forces[nm][:] = 0
            forces[nm][:3] = Fv
            wb[0] -= Fv
            wb[1] -= np.cross(st[nm][0] - bp_, Fv)
        forces['base'][:3] = wb[0]
        forces['base'][3:] = (0.5 * skew(wb[1]) @ np.linalg.inv(bA).T).ravel()
        world.advance(); world.retrieve()
        t += scene['dt']
        if not world.is_valid():
            break
    wall = time.perf_counter() - tstart
    fin = bool(world.is_valid()) and all(np.isfinite(x['base']).all() for x in tr[-3:])
    return tr, {'finite': fin, 'truncated': 0, 'wall_s': wall, 'steps': k, 'd_hat': d_hat,
                'start_gap_m': min(d_hat + 1e-4, START_GAP_MAX), 'affine_deformation_max': deform, 'engine_peak_overlap_m': {}}
