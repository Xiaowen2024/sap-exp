"""Isolated narrow-phase test of the rim jam: upright tube, axis offset from the
hole centre, cone tip touching the hole edge; lower it in small steps and record
the contact normal/gap each engine's collision detection returns for the
tube-rack pair. No dynamics, no solver: collision detection only.

Analytic reference: the rack's top inner edge (a horizontal line, locally) touches
the tube's cone face; the separating normal is the cone-face normal, i.e. n_z =
sin(cone half-angle) and its horizontal part points at the hole centre.
"""
import argparse, json, os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from .common import load


def poses(scene, offset_m, dzs):
    rk = scene['static_bodies'][0]; rr = R.from_quat(np.r_[rk['q'][1:], rk['q'][0]])
    hole = np.asarray(rk['p']) + rr.apply([.072, -.018, .03])            # hole 2 centre at rack top
    u = rr.apply([1., 0., 0.]); u[2] = 0; u /= np.linalg.norm(u)         # offset direction (rack +x)
    com = np.asarray(next(b for b in scene['bodies'] if b['name'] == 'tube1')['origin_offset'])
    r_hole = scene['assets']['rack']['hole_radius_m']; cone_len, r_body = .0159, .014
    h_touch = (r_hole - offset_m) * cone_len / r_body                     # height above tip where cone radius reaches the rim
    q_up = R.from_euler('x', np.pi / 2)                                   # tube +y -> world +z
    out = []
    for dz in dzs:
        tip = hole + offset_m * u; tip[2] = hole[2] - h_touch - dz        # dz>0: pushed down (penetration)
        p = tip + q_up.apply(com); out.append((dz, p, q_up.as_quat()[[3, 0, 1, 2]]))
    cone_half = np.arctan(r_body / cone_len)
    return out, hole, u, {'n_z': float(np.sin(cone_half)), 'n_toCentre': float(np.cos(cone_half)), 'cone_half_angle_deg': float(np.degrees(cone_half))}


def sap_normals(scene, cases, hole, u):
    import warp as wp
    from sap_warp import load_sap_scene
    from sap_warp.collision.pipeline import SapCollisionPipeline
    from sap_warp.resources.collision_model import sap_collision_state_from_state
    from sap_warp.sap_kinematics import sap_eval_fk
    from sap_warp.collision import core
    from .articulated_sap import config
    wp.init(); dev = 'cuda:0'; sc = dict(scene); sc['bodies'] = [dict(b) for b in scene['bodies'] if b['name'] == 'tube1']
    cfgp = Path(os.environ.get('TMPDIR', '/tmp')) / f'pair_{os.getpid()}.json'; cfgp.write_text(json.dumps(config(sc, True)))
    L = load_sap_scene(cfgp, device=dev, naconmax=4096, strict=True); model, st = L.sap_model, L.sap_state
    pipe = SapCollisionPipeline(L.collision_model, naconmax=4096, broad_phase='explicit'); con = pipe.contacts()
    shape_body = model.shape_body.numpy()
    rows = []
    for dz, p, q in cases:
        qv = st.joint_q.numpy(); qv[0:7] = np.r_[p, np.asarray(q)[[1, 2, 3, 0]]]; st.joint_q.assign(qv); st.joint_qd.zero_(); sap_eval_fk(model, st.joint_q, st.joint_qd, st)
        pipe.collide(sap_collision_state_from_state(st), con); n = int(con.rigid_contact_count.numpy()[0])
        s0 = con.rigid_contact_shape0.numpy()[:n]; s1 = con.rigid_contact_shape1.numpy()[:n]; N = con.rigid_contact_normal.numpy()[:n]
        P0 = con.rigid_contact_point0.numpy()[:n]; P1 = con.rigid_contact_point1.numpy()[:n]; m0 = con.rigid_contact_margin0.numpy()[:n]; m1 = con.rigid_contact_margin1.numpy()[:n]
        cs = []
        for i in range(n):
            b0, b1 = int(shape_body[s0[i]]), int(shape_body[s1[i]])
            # world points: shapes on the tube body move; rack/table are static (body -1)
            if 0 not in (b0, b1): continue
            BQ = st.body_q.numpy(); tw = lambda b, x: (R.from_quat(BQ[b][3:7]).apply(x) + BQ[b][:3]) if b >= 0 else x
            w0 = tw(b0, P0[i]); w1 = tw(b1, P1[i])
            nn = N[i] * (1. if b1 == 0 else -1.)                      # orient normal to point INTO the tube (push on tube)
            gap = float(np.dot(w1 - w0, N[i]) - m0[i] - m1[i])
            mid = .5 * (w0 + w1)
            if os.environ.get('PAIR_DEBUG') and len(rows) == 0 and i < 8: print('raw', i, b0, b1, np.round(mid, 4), np.round(nn, 3), round(gap * 1e3, 3), 'hole', np.round(hole, 4))
            if mid[2] < hole[2] - .0045 or np.linalg.norm(mid[:2] - hole[:2]) > .03: continue   # keep only the top-plate rim region
            lat = -(mid[:2] - hole[:2]); lat /= max(np.linalg.norm(lat), 1e-12)
            cs.append({'gap_mm': round(gap * 1e3, 3), 'n_z': round(float(nn[2]), 3), 'n_toCentre': round(float(np.dot(nn[:2], -u[:2])), 3), 'point': np.round(mid, 4).tolist()})
        rows.append({'dz_mm': round(dz * 1e3, 2), 'contacts': cs})
    return rows, getattr(core, 'SAP_CONVEX_BACKEND', 'legacy_unversioned')


def mujoco_normals(scene, cases, hole, u):
    import mujoco as mj
    from .articulated import xml
    sc = dict(scene); sc['bodies'] = [dict(b) for b in scene['bodies'] if b['name'] == 'tube1']
    m = mj.MjModel.from_xml_string(xml(sc, True)); d = mj.MjData(m); bid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, 'tube1')
    rows = []
    for dz, p, q in cases:
        d.qpos[:7] = np.r_[p, q]; mj.mj_forward(m, d); cs = []
        for i in range(d.ncon):
            c = d.contact[i]; g1, g2 = c.geom1, c.geom2
            if bid not in (m.geom_bodyid[g1], m.geom_bodyid[g2]): continue
            other = g1 if m.geom_bodyid[g2] == bid else g2
            if not (mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, other) or '').startswith('rack'): continue
            nn = c.frame[:3] * (1. if m.geom_bodyid[g2] == bid else -1.)
            if c.pos[2] < hole[2] - .0045: continue
            cs.append({'gap_mm': round(float(c.dist) * 1e3, 3), 'n_z': round(float(nn[2]), 3), 'n_toCentre': round(float(np.dot(nn[:2], -u[:2])), 3), 'point': np.round(c.pos, 4).tolist()})
        rows.append({'dz_mm': round(dz * 1e3, 2), 'contacts': cs})
    return rows, mj.__version__


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--scene'); ap.add_argument('--engine', choices=['sap', 'mujoco']); ap.add_argument('--offset-mm', type=float, default=6.)
    ap.add_argument('--out'); a = ap.parse_args(); scene, _ = load(a.scene)
    dzs = np.array([-4., -3., -2.5, -2., -1.5, -1., -.5, 0., .5, 1.]) * 1e-3
    cases, hole, u, ref = poses(scene, a.offset_mm * 1e-3, dzs)
    rows, ver = (sap_normals if a.engine == 'sap' else mujoco_normals)(scene, cases, hole, u)
    res = {'engine': a.engine, 'version': ver, 'offset_mm': a.offset_mm, 'analytic': ref, 'rows': rows}
    Path(a.out).write_text(json.dumps(res, indent=1))
    print(a.engine, ver, 'analytic', ref)
    for r in rows:
        print('  push %+5.2f mm:' % r['dz_mm'], [(c['gap_mm'], c['n_z'], c['n_toCentre']) for c in r['contacts']][:6])
