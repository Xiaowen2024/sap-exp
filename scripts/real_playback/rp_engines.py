"""Engine backends for real-playback v1 (MuJoCo CPU, SAP Warp batched, IPC).

All backends consume the same scene dict, the same asset files and the same
drive; each returns, per rollout, a 100-Hz trace of base / pad / basket poses
and the engine's own peak-overlap and capacity readings.
"""
from __future__ import annotations

import time
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from .rp_sim import (G, Playback, base_wrench, basket_body_pose, finger_force, load_obj, place_on_table,
                     qmat, qmul, table_pose)

TRACE_HZ = 100


def shape_mu(mu_pb, mu_be):
    """Shape-level mu (pad, basket, env) for engines that combine by harmonic
    mean, reproducing pad-basket and basket-env exactly; pad-env is derived."""
    b = max(mu_pb, mu_be)
    p = 1 / (2 / mu_pb - 1 / b)
    e = 1 / (2 / mu_be - 1 / b)
    return {'pad': p, 'basket': b, 'env': e, 'pad_env_derived': 2 * p * e / (p + e)}


def drive_cfg(scene):
    d = dict(scene['drive'])
    d['m_total'] = d['base_mass'] + 2 * scene['pad']['mass']
    d['pad_mass'] = scene['pad']['mass']
    d['I_base'] = d['base_inertia']
    return d


def initial_state(scene, pb, meta):
    t0 = scene['t_start']
    p, q, v, w, width, closed = pb.target(t0)
    width = float(np.interp(t0, pb.gt, pb.gw))  # fingers start at the recorded (actual) width
    # segment replays (object already in hand): optional extra opening so the pads start
    # touching, not overlapping, the fitted in-hand pose; the normal width law then closes them
    width += float(scene.get('segment', {}).get('init_extra_width_m', 0.0))
    bpos, bq = basket_body_pose(scene, meta)
    return t0, p, q, width, bpos, bq


def pad_local_center(scene, side, qslide):
    t = scene['pad']['size'][1]
    return np.array([0., side * (qslide + t / 2), -scene['pad']['size'][2] / 2])


# ======================================================================== MuJoCo
def nums(x):
    return ' '.join('%.10g' % v for v in np.ravel(x))


def make_mjcf(scene, meta, candidate, objects=True):
    pieces = meta['pieces']
    root = ET.Element('mujoco', model='rh20t_playback')
    ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false')
    opt = ET.SubElement(root, 'option', timestep='%.10g' % scene['dt'], gravity=nums(G))
    par = candidate.get('parameters', {})
    for k in ('impratio', 'noslip_iterations', 'cone', 'integrator', 'iterations', 'tolerance'):
        if k in par:
            opt.set(k, str(par[k]))
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'mesh', name='pad', file=pieces['pad'])
    if objects:
        for i, p in enumerate(pieces['basket']):
            ET.SubElement(asset, 'mesh', name=f'bk{i}', file=p)
        for i, p in enumerate(pieces['rack']):
            ET.SubElement(asset, 'mesh', name=f'rk{i}', file=p)
    wb = ET.SubElement(root, 'worldbody')
    from scipy.spatial.transform import Rotation as R
    d = drive_cfg(scene)
    base = ET.SubElement(wb, 'body', name='base', pos='0 0 1', quat='1 0 0 0')
    ET.SubElement(base, 'freejoint', name='base_free')
    ET.SubElement(base, 'inertial', pos='0 0 0', mass='%.10g' % d['base_mass'], diaginertia=nums([d['base_inertia']] * 3))
    ph = np.asarray(scene['pad']['size']) / 2
    Ip = scene['pad']['mass'] / 3 * (np.sum(ph ** 2) - ph ** 2)
    for side, name in ((-1, 'left'), (1, 'right')):
        body = ET.SubElement(base, 'body', name=name, pos=nums(pad_local_center(scene, side, 0.)))
        ET.SubElement(body, 'joint', name=name + '_slide', type='slide', axis=nums([0, side, 0]), limited='true',
                      range=nums([0, scene['pad']['stroke'] / 2]))
        ET.SubElement(body, 'inertial', pos='0 0 0', mass='%.10g' % scene['pad']['mass'], diaginertia=nums(Ip))
        ET.SubElement(body, 'geom', name=name + '_pad', type='mesh', mesh='pad', contype='0', conaffinity='0')
    if not objects:
        return ET.tostring(root, encoding='unicode')
    tc, tR = table_pose(scene)
    th = scene['table']['box_thickness']
    tq = R.from_matrix(tR).as_quat()[[3, 0, 1, 2]]
    ET.SubElement(wb, 'geom', name='table', type='box', size=nums([.6, .6, th / 2]), pos=nums(tc), quat=nums(tq),
                  contype='0', conaffinity='0')
    rp, rq = place_on_table(scene, scene['rack']['xy'], scene['rack']['yaw'])
    rack = ET.SubElement(wb, 'body', name='rack', pos=nums(rp), quat=nums(rq))
    for i in range(len(pieces['rack'])):
        ET.SubElement(rack, 'geom', name=f'rk{i}', type='mesh', mesh=f'rk{i}', contype='0', conaffinity='0')
    bk = ET.SubElement(wb, 'body', name='basket', pos='0 0 .5')
    ET.SubElement(bk, 'freejoint', name='basket_free')
    I = np.asarray(meta['basket_inertia_per_kg']) * scene['basket']['mass']
    ET.SubElement(bk, 'inertial', pos='0 0 0', mass='%.10g' % scene['basket']['mass'],
                  fullinertia=nums([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]]))
    for i in range(len(pieces['basket'])):
        ET.SubElement(bk, 'geom', name=f'bk{i}', type='mesh', mesh=f'bk{i}', contype='0', conaffinity='0')
    con = ET.SubElement(root, 'contact')
    mu = scene['mu']
    env = ['table'] + [f'rk{i}' for i in range(len(pieces['rack']))]

    def pair(a, b, m):
        at = {'geom1': a, 'geom2': b, 'condim': '3', 'friction': nums([m, m, .005, .0001, .0001])}
        if 'solref' in par:
            at['solref'] = nums(par['solref'])
        if 'solimp' in par:
            at['solimp'] = nums(par['solimp'])
        ET.SubElement(con, 'pair', **at)
    for i in range(len(pieces['basket'])):
        for p in ('left_pad', 'right_pad'):
            pair(f'bk{i}', p, mu['pad_basket'])
        for e in env:
            pair(f'bk{i}', e, mu['basket_env'])
    for p in ('left_pad', 'right_pad'):
        for e in env:
            pair(p, e, mu['pad_env'])
    return ET.tostring(root, encoding='unicode')


class MuJoCoRollout:
    engine = 'mujoco'

    def __init__(self, scene, meta, pb, candidate, perturb, objects=True):
        import mujoco as mj
        self.mj = mj
        self.objects = objects
        self.scene, self.pb, self.cand = scene, pb, candidate
        self.xml = make_mjcf(scene, meta, candidate, objects)
        self.m = mj.MjModel.from_xml_string(self.xml)
        self.d = mj.MjData(self.m)
        m, d = self.m, self.d
        if objects:
            t0, p, q, width, bpos, bq = initial_state(scene, pb, meta)
            # numerical perturbation: basket xy and yaw about the table normal
            _, tR = table_pose(scene)
            bpos = bpos + tR[:, 0] * perturb[0] + tR[:, 1] * perturb[1]
            from scipy.spatial.transform import Rotation as R
            rz = R.from_rotvec(tR[:, 2] * perturb[2]).as_quat()[[3, 0, 1, 2]]
            bq = qmul(rz, bq)
        else:
            t0 = scene['t_start']; p, q, _, _, width, _ = pb.target(t0)
        self.jb = m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, 'base_free')]
        self.vb = m.jnt_dofadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, 'base_free')]
        if objects:
            self.jk = m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, 'basket_free')]
            self.vk = m.jnt_dofadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, 'basket_free')]
        self.js = [m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n + '_slide')] for n in ('left', 'right')]
        self.vs = [m.jnt_dofadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n + '_slide')] for n in ('left', 'right')]
        self.bid = {n: mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, n) for n in ('base', 'left', 'right') + (('basket',) if objects else ())}
        d.qpos[self.jb:self.jb + 3] = p; d.qpos[self.jb + 3:self.jb + 7] = q
        for j in self.js:
            d.qpos[j] = width / 2
        if objects:
            d.qpos[self.jk:self.jk + 3] = bpos; d.qpos[self.jk + 3:self.jk + 7] = bq
        mj.mj_forward(m, d)
        self.t = t0
        self.drive = drive_cfg(scene)
        gid = lambda n: mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, n)
        self.g_pad = {gid('left_pad'), gid('right_pad')}
        self.g_bk = {gid(f'bk{i}') for i in range(len(meta['pieces']['basket']))} if objects else set()
        self.g_table = gid('table') if objects else -5
        self.peak = {'pad_basket': 0., 'basket_rack': 0., 'basket_table': 0.}
        self.w0 = np.array([w.number for w in d.warning])

    def state(self):
        d = self.d
        R = qmat(d.qpos[self.jb + 3:self.jb + 7])
        return (d.qpos[self.jb:self.jb + 3].copy(), d.qpos[self.jb + 3:self.jb + 7].copy(),
                d.qvel[self.vb:self.vb + 3].copy(), R @ d.qvel[self.vb + 3:self.vb + 6])

    def step(self):
        d, m = self.d, self.m
        tp, tq, tv, tw, width, closed = self.pb.target(self.t)
        pos, q, v, w = self.state()
        f, tau = base_wrench(pos, q, v, w, tp, tq, tv, tw, self.drive)
        d.xfrc_applied[self.bid['base'], :3] = f
        d.xfrc_applied[self.bid['base'], 3:] = tau
        for j, vj in zip(self.js, self.vs):
            d.qfrc_applied[vj] = finger_force(d.qpos[j], d.qvel[vj], width, closed, self.drive)
        self.mj.mj_step(m, d)
        self.t += self.scene['dt']
        for c in d.contact[:d.ncon]:
            g = {int(c.geom[0]), int(c.geom[1])}
            pen = max(0., -float(c.dist))
            if g & self.g_bk:
                if g & self.g_pad:
                    k = 'pad_basket'
                elif self.g_table in g:
                    k = 'basket_table'
                else:
                    k = 'basket_rack'
                if pen > self.peak[k]:
                    self.peak[k] = pen

    def sample(self):
        d = self.d
        pos, q, v, w = self.state()
        pads = [np.r_[d.xpos[self.bid[n]], d.xquat[self.bid[n]]] for n in ('left', 'right')]
        s = {'t': self.t, 'base': np.r_[pos, q], 'pads': pads, 'slider': [float(d.qpos[j]) for j in self.js]}
        if self.objects:
            Rk = qmat(d.qpos[self.jk + 3:self.jk + 7])
            s['basket'] = np.r_[d.qpos[self.jk:self.jk + 3], d.qpos[self.jk + 3:self.jk + 7]]
            s['basket_v'] = np.r_[d.qvel[self.vk:self.vk + 3], Rk @ d.qvel[self.vk + 3:self.vk + 6]]
        return s

    def health(self):
        d, mj = self.d, self.mj
        w = np.array([x.number for x in d.warning]) - self.w0
        trunc = int(w[int(mj.mjtWarning.mjWARN_CONTACTFULL)] + w[int(mj.mjtWarning.mjWARN_CNSTRFULL)])
        bad = int(w[int(mj.mjtWarning.mjWARN_BADQACC)] + w[int(mj.mjtWarning.mjWARN_BADQPOS)] + w[int(mj.mjtWarning.mjWARN_BADQVEL)])
        return {'finite': bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()) and bad == 0,
                'truncated': trunc, 'engine_peak_overlap_m': dict(self.peak)}


def run_mujoco(scene, meta, pbdata, candidate, perturb, max_wall_s=3600, objects=True):
    pb = Playback(pbdata, scene['finger_rule'])
    sim = MuJoCoRollout(scene, meta, pb, candidate, perturb, objects)
    n = int(round((scene['t_end'] - scene['t_start']) / scene['dt']))
    every = max(1, int(round(1 / (TRACE_HZ * scene['dt']))))
    tr = [sim.sample()]
    t0 = time.perf_counter()
    for k in range(n):
        sim.step()
        if (k + 1) % every == 0:
            s = sim.sample()
            tr.append(s)
            if not np.isfinite(s['base']).all() or (objects and not np.isfinite(s['basket']).all()) or time.perf_counter() - t0 > max_wall_s:
                break
    h = sim.health()
    h['wall_s'] = time.perf_counter() - t0
    h['steps'] = k + 1
    return tr, h


# ========================================================================== SAP
def base_wrench_batch(P, Q, V, W, tp, tq, tv, tw, d):
    """Vectorised base_wrench over worlds. Q, tq wxyz (n,4)/(4,)."""
    f = d['K_t'] * (tp - P) + 2 * np.sqrt(d['K_t'] * d['m_total']) * (tv - V)
    n = np.linalg.norm(f, axis=1, keepdims=True)
    f = np.where(n > d['F_cap'], f * d['F_cap'] / np.maximum(n, 1e-12), f)
    # er = rotvec(tq * conj(Q))
    w1, x1, y1, z1 = tq
    w2, x2, y2, z2 = Q[:, 0], -Q[:, 1], -Q[:, 2], -Q[:, 3]
    qe = np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                   w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], 1)
    qe = np.where(qe[:, :1] < 0, -qe, qe)
    s = np.linalg.norm(qe[:, 1:], axis=1)
    ang = 2 * np.arctan2(s, qe[:, 0])
    er = qe[:, 1:] * (ang / np.maximum(s, 1e-12))[:, None]
    tau = d['K_r'] * er + 2 * np.sqrt(d['K_r'] * d['I_base']) * (tw - W)
    n = np.linalg.norm(tau, axis=1, keepdims=True)
    tau = np.where(n > d['T_cap'], tau * d['T_cap'] / np.maximum(n, 1e-12), tau)
    return f - d['m_total'] * G, tau


def sap_scene_cfg(scene, meta, candidate, objects=True):
    """SAP scene file (schema v1) from the same assets; xyzw quaternions."""
    from scipy.spatial.transform import Rotation as R
    par = candidate.get('parameters', {})
    mus = shape_mu(scene['mu']['pad_basket'], scene['mu']['basket_env'])

    def mat(mu):
        m = {'mu': float(mu), 'mu_torsional': 0., 'mu_rolling': 0., 'density': 0.}
        if 'ke' in par:
            m['ke'] = 2. * par['ke']          # harmonic pair combine: pair = shape / 2
        if 'tau' in par:
            m['tau'] = 0.5 * par['tau']       # pair tau = shape tau sum
        return m

    def tf(p, q_wxyz):
        return {'p': [float(x) for x in p], 'q': [float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3]), float(q_wxyz[0])]}
    d = drive_cfg(scene)
    bodies, joints, arts = [], [], []
    bodies.append({'id': 'base', 'transform': tf([0, 0, 1], [1, 0, 0, 0]), 'mass': d['base_mass'],
                   'inertia': (np.eye(3) * d['base_inertia']).tolist()})
    joints.append({'id': 'base_free', 'type': 'free', 'parent': 'world', 'child': 'base'})
    ph = np.asarray(scene['pad']['size']) / 2
    Ip = scene['pad']['mass'] / 3 * (np.sum(ph ** 2) - ph ** 2)
    moving = ['base_free']
    for side, name in ((-1, 'left'), (1, 'right')):
        c = pad_local_center(scene, side, 0.)
        bodies.append({'id': name, 'transform': tf(np.r_[0, 0, 1] + c, [1, 0, 0, 0]), 'mass': scene['pad']['mass'],
                       'inertia': np.diag(Ip).tolist(),
                       'shapes': [{'type': 'convex_mesh', 'label': name + '_pad', 'path': meta['pieces']['pad'], 'is_solid': True, 'cfg': mat(mus['pad'])}]})
        joints.append({'id': name + '_slide', 'type': 'prismatic', 'parent': 'base', 'child': name,
                       'parent_xform': {'p': c.tolist(), 'q': 'identity'}, 'axis': [0, side, 0],
                       'limit_lower': 0., 'limit_upper': scene['pad']['stroke'] / 2, 'target_ke': 0., 'target_kd': 0.})
        moving.append(name + '_slide')
    arts.append({'id': 'gripper', 'joints': moving})
    if objects:
        I = (np.asarray(meta['basket_inertia_per_kg']) * scene['basket']['mass']).tolist()
        bodies.append({'id': 'basket', 'transform': tf([0, 0, .5], [1, 0, 0, 0]), 'mass': scene['basket']['mass'], 'inertia': I,
                       'shapes': [{'type': 'convex_mesh', 'label': f'bk{i}', 'path': p, 'is_solid': True, 'cfg': mat(mus['basket'])}
                                  for i, p in enumerate(meta['pieces']['basket'])]})
        joints.append({'id': 'basket_free', 'type': 'free', 'parent': 'world', 'child': 'basket'})
        arts.append({'id': 'basket_art', 'joints': ['basket_free']})
        rp, rq = place_on_table(scene, scene['rack']['xy'], scene['rack']['yaw'])
        # static bodies: no joint, no mass, no articulation (as the round-socket fixture)
        bodies.append({'id': 'rack', 'transform': tf(rp, rq),
                       'shapes': [{'type': 'convex_mesh', 'label': f'rk{i}', 'path': p, 'is_solid': True, 'cfg': mat(mus['env'])}
                                  for i, p in enumerate(meta['pieces']['rack'])]})
        tc, tR = table_pose(scene)
        tq = R.from_matrix(tR).as_quat()[[3, 0, 1, 2]]
        th = scene['table']['box_thickness']
        bodies.append({'id': 'table', 'transform': tf(tc, tq),
                       'shapes': [{'type': 'box', 'label': 'table', 'hx': .6, 'hy': .6, 'hz': th / 2, 'cfg': mat(mus['env'])}]})
    return {'schema_version': 1, 'name': 'rh20t_playback',
            'simulation': {'dt': scene['dt'], 'num_worlds': 1, 'nconmax': scene.get('contact_capacity', 512)},
            'builder': {'gravity': float(G[2])}, 'ground': {'enabled': False},
            'bodies': bodies, 'joints': joints, 'articulations': arts}


def run_sap(scene, meta, pbdata, candidate, perturbs, workdir, objects=True, device='cuda:0'):
    """R rollouts as R batched worlds of one SAP model. Returns (traces, healths)."""
    import json as _json
    import warp as wp
    from sap_warp import SolverSAP, load_sap_scene
    from sap_warp.collision.pipeline import SapCollisionPipeline
    from sap_warp.resources.collision_model import sap_collision_state_from_state
    from sap_warp.sap_kinematics import sap_eval_fk
    from scipy.spatial.transform import Rotation as R
    wp.init()
    Rn = len(perturbs)
    workdir = Path(workdir); workdir.mkdir(parents=True, exist_ok=True)
    cfg = sap_scene_cfg(scene, meta, candidate, objects)
    (workdir / 'sap_scene.json').write_text(_json.dumps(cfg, indent=1))
    cap = scene.get('contact_capacity', 512)
    loaded = load_sap_scene(workdir / 'sap_scene.json', device=device, naconmax=cap * Rn, strict=True, num_worlds=Rn)
    model, state, control = loaded.sap_model, loaded.sap_state, loaded.sap_control
    other = model.state()
    labels = list(model.body_label); nb = state.body_q.shape[0] // Rn if 'state' in dir() else A.body_q.shape[0] // Rn
    jl = list(model.joint_label); nj = (len(model.joint_q_start.numpy()) - 1) // Rn
    qs = model.joint_q_start.numpy(); vs = model.joint_qd_start.numpy()
    bidx = {n: [w * nb + labels[:nb].index(n) for w in range(Rn)] for n in (['base', 'left', 'right'] + (['basket'] if objects else []))}
    jidx = {n: [w * nj + jl[:nj].index(n) for w in range(Rn)] for n in (['base_free', 'left_slide', 'right_slide'] + (['basket_free'] if objects else []))}
    pb = Playback(pbdata, scene['finger_rule'])
    if objects:
        t0, p, q, width, bpos, bq = initial_state(scene, pb, meta)
    else:
        t0 = scene['t_start']; p, q, _, _, width, _ = pb.target(t0)
    qv = state.joint_q.numpy(); qv[:] = 0
    _, tR = table_pose(scene)
    for w in range(Rn):
        a = qs[jidx['base_free'][w]]
        qv[a:a + 3] = p; qv[a + 3:a + 7] = q[[1, 2, 3, 0]]
        for n in ('left_slide', 'right_slide'):
            qv[qs[jidx[n][w]]] = width / 2
        if objects:
            pr = perturbs[w]
            bp = bpos + tR[:, 0] * pr[0] + tR[:, 1] * pr[1]
            rz = R.from_rotvec(tR[:, 2] * pr[2]).as_quat()[[3, 0, 1, 2]]
            bqq = qmul(rz, bq)
            a = qs[jidx['basket_free'][w]]
            qv[a:a + 3] = bp; qv[a + 3:a + 7] = bqq[[1, 2, 3, 0]]
    state.joint_q.assign(qv); state.joint_qd.zero_()
    sap_eval_fk(model, state.joint_q, state.joint_qd, state)
    kw = {'nconmax': cap}  # SolverSAP capacity is per world; pipeline capacity is the total
    if 'sigma' in candidate.get('parameters', {}):
        kw['contact_sigma'] = candidate['parameters']['sigma']
    solver = SolverSAP(model, **kw)
    pipe = SapCollisionPipeline(loaded.collision_model, naconmax=cap * Rn)
    contacts = pipe.contacts()
    d = drive_cfg(scene)
    n = int(round((scene['t_end'] - scene['t_start']) / scene['dt']))
    every = max(1, int(round(1 / (TRACE_HZ * scene['dt']))))
    iB = np.array(bidx['base']); iL = np.array(bidx['left']); iRr = np.array(bidx['right'])
    iK = np.array(bidx['basket']) if objects else None
    sl = np.array([[vs[jidx['left_slide'][w]], vs[jidx['right_slide'][w]]] for w in range(Rn)])
    slq = np.array([[qs[jidx['left_slide'][w]], qs[jidx['right_slide'][w]]] for w in range(Rn)])
    traces = [[] for _ in range(Rn)]
    trunc = 0
    minphi = np.full(Rn, np.inf)
    t = t0
    tstart = time.perf_counter()

    def record():
        bq_ = state.body_q.numpy(); bv_ = state.body_qd.numpy(); jq = state.joint_q.numpy()
        for w in range(Rn):
            def pose(i):
                x = bq_[i]; return np.r_[x[:3], x[6], x[3:6]]
            s = {'t': t, 'base': pose(iB[w]), 'pads': [pose(iL[w]), pose(iRr[w])],
                 'slider': [float(jq[slq[w, 0]]), float(jq[slq[w, 1]])]}
            if objects:
                s['basket'] = pose(iK[w]); s['basket_v'] = bv_[iK[w]].copy()
            traces[w].append(s)
    record()
    for k in range(n):
        tp, tq, tv, tw, width, closed = pb.target(t)
        bq_ = state.body_q.numpy(); bv_ = state.body_qd.numpy()
        P = bq_[iB, :3]; Q = np.c_[bq_[iB, 6], bq_[iB, 3:6]]
        V = bv_[iB, :3]; W = bv_[iB, 3:6]
        f, tau = base_wrench_batch(P, Q, V, W, tp, tq, tv, tw, d)
        state.clear_forces()
        bf = state.body_f.numpy(); bf[:] = 0
        bf[iB, :3] = f; bf[iB, 3:] = tau
        state.body_f.assign(bf)
        jq = state.joint_q.numpy(); jqd = state.joint_qd.numpy()
        jf = control.joint_f.numpy(); jf[:] = 0
        for w in range(Rn):
            for c in range(2):
                jf[sl[w, c]] = finger_force(jq[slq[w, c]], jqd[sl[w, c]], width, closed, d)
        control.joint_f.assign(jf)
        pipe.collide(sap_collision_state_from_state(state), contacts)
        solver.step(state, other, control, contacts, scene['dt'])
        trunc = max(trunc, int(solver.last_truncated_contact_count or 0))
        state, other = other, state
        t += scene['dt']
        if objects and (k % 10 == 0):
            cnt = solver.contact_jacobian.contact_env_count_wp.numpy().astype(int).ravel()
            phi = solver.contact_jacobian.contact_env_phi0_wp.numpy()
            for w in range(Rn):
                if cnt[w]:
                    minphi[w] = min(minphi[w], float(phi[w, :cnt[w]].min()))
        if (k + 1) % every == 0:
            record()
            if not np.isfinite(state.body_q.numpy()).all():
                break
    wall = time.perf_counter() - tstart
    healths = []
    for w in range(Rn):
        fin = all(np.isfinite(s['base']).all() and (not objects or np.isfinite(s['basket']).all()) for s in traces[w][-3:])
        healths.append({'finite': bool(fin), 'truncated': trunc, 'wall_s': wall / Rn, 'wall_batch_s': wall, 'steps': k + 1,
                        'engine_peak_overlap_m': {'any_contact': float(max(0., -minphi[w])) if np.isfinite(minphi[w]) else 0.}})
    return traces, healths
