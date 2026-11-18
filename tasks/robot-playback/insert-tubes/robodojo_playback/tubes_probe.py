"""Contact probe for the insert-tubes jam: replay the trajectory exactly as the
runner does (host-stepped) and, inside a time window, dump every contact that
involves one tube: point, normal, gap, and the impulse the solver applied.

Output: probe.npz-like JSON list of frames, each with contacts
  {other, point[3], normal[3] (pointing from other body INTO the tube), gap_m,
   force_world[3] (force ON the tube, N)}
"""
import argparse, json, time
from pathlib import Path
import numpy as np
from .common import load, rotation, wrench


def classify(scene, other, p):
    """Name the rack feature a world point lies on (rack frame)."""
    if other != 'rack': return other
    rk = scene['static_bodies'][0]; loc = rotation(rk['q']).inv().apply(np.asarray(p) - np.asarray(rk['p']))
    if loc[2] > .0245: return 'rack_top_plate'
    if loc[2] < -.0235: return 'rack_floor'
    return 'rack_post'


def probe_mujoco(scene, target, tube, t0, t1, every):
    import mujoco as mj
    from .articulated import xml, motor
    m = mj.MjModel.from_xml_string(xml(scene, True)); d = mj.MjData(m)
    ids = {b['name']: mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, b['name']) for b in scene['bodies']}; names = {v: k for k, v in ids.items()}
    slots = {}
    for b in scene['bodies']:
        j = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, b['name'] + '_joint'); slots[b['name']] = (int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j]))
        if b.get('parent'): d.qpos[slots[b['name']][0]] = b['joint_initial']
    mj.mj_forward(m, d); frames = []; tb = ids[tube]; n = len(target['time']) - 1; k1 = int(round(t1 / scene['dt']))
    for k in range(min(n, k1)):
        d.xfrc_applied[:] = 0; d.qfrc_applied[:] = 0
        for b in scene['bodies']:
            qi, vi = slots[b['name']]
            if b['driven']:
                st = (d.qpos[qi:qi + 3].copy(), d.qpos[qi + 3:qi + 7].copy(), d.qvel[vi:vi + 3].copy(), rotation(d.qpos[qi + 3:qi + 7]).apply(d.qvel[vi + 3:vi + 6]))
                f, tau = wrench(scene, b, target, st, k); d.xfrc_applied[ids[b['name']]] = np.r_[f, tau]
            elif b.get('is_gripper'):
                d.qfrc_applied[vi] = motor(scene, target['command_' + b['name'].split('_')[0]][k], d.qpos[qi], d.qvel[vi])
                d.xfrc_applied[ids[b['name']], :3] = -scene['assets'][b['asset']]['mass'] * np.asarray(scene['gravity'])
        mj.mj_step(m, d)
        if k * scene['dt'] >= t0 and k % every == 0:
            cs = []
            for i in range(d.ncon):
                c = d.contact[i]; b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
                if tb not in (b1, b2): continue
                f6 = np.zeros(6); mj.mj_contactForce(m, d, i, f6); R = c.frame.reshape(3, 3)   # rows: normal, t1, t2 ; normal geom1 -> geom2
                fw = R.T @ f6[:3]                       # force on geom2 from geom1, world
                sign = 1. if b2 == tb else -1.; other = names.get(b1 if b2 == tb else b2, 'rack' if m.geom_bodyid[c.geom1 if b2 == tb else c.geom2] == 0 else '?')
                if (b1 if b2 == tb else b2) == 0: other = (mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, c.geom1 if b2 == tb else c.geom2) or 'table').split('_')[0]
                cs.append({'other': classify(scene, other, c.pos), 'point': c.pos.tolist(), 'normal': (sign * R[0]).tolist(), 'gap_m': float(c.dist), 'force_world': (sign * fw).tolist()})
            frames.append({'t': k * scene['dt'] + scene['dt'], 'tube_pose': np.r_[d.xpos[tb], d.xquat[tb]].tolist(), 'contacts': cs})
    return frames


def probe_sap(scene, target, tube, t0, t1, every):
    import warp as wp, os
    from sap_warp import SolverSAP, load_sap_scene
    from sap_warp.collision.pipeline import SapCollisionPipeline
    from sap_warp.resources.collision_model import sap_collision_state_from_state
    from sap_warp.sap_kinematics import sap_eval_fk
    from .articulated_sap import config, fingers
    from .sap_graph import drive
    if os.environ.get('ROBODOJO_WARP_CACHE'): wp.config.kernel_cache_dir = os.environ['ROBODOJO_WARP_CACHE']
    wp.init(); device = 'cuda:0'; cap = scene['contact_capacity']; out = Path(os.environ.get('TMPDIR', '/tmp')) / f'probe_{os.getpid()}.json'
    out.write_text(json.dumps(config(scene, True))); loaded = load_sap_scene(out, device=device, naconmax=cap, strict=True)
    model, A, control = loaded.sap_model, loaded.sap_state, loaded.sap_control; B = model.state(); labels = list(model.body_label); jl = list(model.joint_label)
    ids = {b['name']: labels.index(b['name']) for b in scene['bodies']}; qs = model.joint_q_start.numpy(); vs = model.joint_qd_start.numpy(); q = A.joint_q.numpy()
    for b in scene['bodies']:
        o = qs[jl.index(b['name'] + '_joint')]
        if b.get('parent'): q[o] = b['joint_initial']
        else: q[o:o + 7] = np.r_[b['p'], np.asarray(b['q'])[[1, 2, 3, 0]]]
    for s in (A, B): s.joint_q.assign(q); s.joint_qd.zero_(); sap_eval_fk(model, s.joint_q, s.joint_qd, s)
    kw = {'nconmax': cap, 'collect_iteration_stats': False, 'check_line_search_errors': False}
    par = scene.get('sap_parameters', {})
    if 'sigma' in par: kw['contact_sigma'] = par['sigma']
    kw.update(par.get('solver_kwargs', {})); solver = SolverSAP(model, **kw)
    pipe = SapCollisionPipeline(loaded.collision_model, naconmax=cap, broad_phase='explicit'); con = pipe.contacts()
    bases = [b for b in scene['bodies'] if b['driven']]; tg = np.stack([np.c_[target[b['name'] + '__pose'], target[b['name'] + '__velocity']] for b in bases], axis=1)
    prm = []
    for b in bases:
        a = scene['assets'][b['asset']]; dd = scene['drive']
        prm.append([dd['Kt'], 2 * np.sqrt(a['mass'] * dd['Kt']), dd['force_cap_N'], dd['Kr'], 2 * np.sqrt(dd['Kr'] * np.linalg.eigvalsh(a['inertia']).max()), dd['torque_cap_Nm'], a['mass'] * 9.81])
    tg = wp.array(tg.astype(np.float32), dtype=wp.float32, device=device); prm = wp.array(np.asarray(prm, np.float32), dtype=wp.float32, device=device)
    bi = wp.array([ids[b['name']] for b in bases], dtype=wp.int32, device=device)
    fi, fp = [], []; act = scene['finger_actuator']
    for b in scene['bodies']:
        if b.get('parent'):
            j = jl.index(b['name'] + '_joint'); fi.append([qs[j], vs[j], ids[b['name']], 0 if b['name'].startswith('left') else 1])
            fp.append([act['Kp'], act['Kd'], act['force_cap_N'], scene['assets'][b['asset']]['mass'] * 9.81])
    fi = wp.array(np.asarray(fi, np.int32), dtype=wp.int32, device=device); fp = wp.array(np.asarray(fp, np.float32), dtype=wp.float32, device=device)
    cmds = wp.array(np.c_[target['command_left'], target['command_right']].astype(np.float32), dtype=wp.float32, device=device)
    counter = wp.zeros(1, dtype=wp.int32, device=device); names = {i: l for i, l in enumerate(labels)}; tb = ids[tube]; dt = scene['dt']
    frames = []; cj = solver.contact_jacobian; k1 = int(round(t1 / dt)); s0, s1 = A, B
    for k in range(k1):
        s0.clear_forces(); wp.launch(drive, len(bases), inputs=[counter, tg, bi, prm, s0.body_q, s0.body_qd, s0.body_f], device=device)
        wp.launch(fingers, len(fi), inputs=[counter, cmds, fi, fp, s0.joint_q, s0.joint_qd, control.joint_f, s0.body_f], device=device)
        pipe.collide(sap_collision_state_from_state(s0), con); solver.step(s0, s1, control, con, dt)
        counter.assign(np.array([k + 1], np.int32))
        if k * dt >= t0 and k % every == 0:
            n = int(cj.contact_env_count_wp.numpy()[0]); b0 = cj.contact_env_body0_wp.numpy()[0, :n]; b1 = cj.contact_env_body1_wp.numpy()[0, :n]
            P = cj.contact_env_point_wp.numpy()[0, :n]; phi = cj.contact_env_phi0_wp.numpy()[0, :n]; RWC = cj.contact_env_R_WC_wp.numpy()[0, :n]
            g = solver.contact_solve.contact_gamma.numpy()[0, :n]
            cs = []
            for i in range(n):
                if tb not in (b0[i], b1[i]): continue
                fw = RWC[i] @ g[i] / dt                       # impulse on body1 (normal cz points body0 -> body1), world
                sign = 1. if b1[i] == tb else -1.; ob = b0[i] if b1[i] == tb else b1[i]
                other = names.get(int(ob), 'world') if ob >= 0 else 'world'
                if other in ('rack', 'world') and any(sb['name'] == 'rack' for sb in scene.get('static_bodies', [])):
                    other = 'rack' if np.linalg.norm(P[i][:2] - np.asarray(scene['static_bodies'][0]['p'][:2])) < .12 and P[i][2] > scene['table_top_z'] + .002 else 'table'
                cs.append({'other': classify(scene, other, P[i]), 'point': P[i].tolist(), 'normal': (sign * RWC[i][:, 2]).tolist(), 'gap_m': float(phi[i]),
                           'force_world': (sign * fw).tolist()})
            bq = s1.body_q.numpy()[tb]; frames.append({'t': (k + 1) * dt, 'tube_pose': [float(x) for x in (*bq[:3], bq[6], *bq[3:6])], 'contacts': cs})
        s0, s1 = s1, s0
    return frames


def summarize(frames):
    rows = []
    for f in frames:
        r = {'t': round(f['t'], 3), 'tube_z': round(f['tube_pose'][2], 4)}
        for grp in ('rack_top_plate', 'rack_floor', 'left_link7', 'left_link8', 'right_link7', 'right_link8'):
            cs = [c for c in f['contacts'] if c['other'] == grp and np.linalg.norm(c['force_world']) > 1e-4]
            if not cs: continue
            F = np.sum([c['force_world'] for c in cs], 0); nz = [abs(c['normal'][2]) for c in cs]
            r[grp] = {'n': len(cs), 'F': np.round(F, 2).tolist(), 'normal_z_range': [round(min(nz), 2), round(max(nz), 2)]}
        rows.append(r)
    return rows


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--scene'); ap.add_argument('--engine', choices=['mujoco', 'sap']); ap.add_argument('--params', default=None)
    ap.add_argument('--tube', default='tube1'); ap.add_argument('--t0', type=float); ap.add_argument('--t1', type=float); ap.add_argument('--every', type=int, default=5); ap.add_argument('--out')
    a = ap.parse_args(); scene, target = load(a.scene)
    if a.params: scene.update({k: v for k, v in json.loads(Path(a.params).read_text()).items() if k != 'why'})
    st = time.time(); fr = (probe_mujoco if a.engine == 'mujoco' else probe_sap)(scene, target, a.tube, a.t0, a.t1, a.every)
    Path(a.out).write_text(json.dumps({'frames': fr, 'summary': summarize(fr)})); print('frames', len(fr), 'wall', round(time.time() - st, 1))
