"""SAP backend with the playback drive on the GPU and the whole step captured
in a CUDA graph (SolverSAP supports outer capture with host diagnostics off).

Same physics and drive law as rp_engines.run_sap (host-side drive); the drive
is evaluated per world by a Warp kernel from a precomputed per-step target
table, so no host synchronisation happens inside the time loop.  Validated
against the host-drive path before use (see validate()).
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import warp as wp

from .rp_engines import TRACE_HZ, drive_cfg, initial_state, sap_scene_cfg
from .rp_sim import Playback, qmul, table_pose

NREC = 36  # base p,q(wxyz) | left p,q | right p,q | basket p,q | basket v, w | sliders 2


@wp.kernel
def _drive(counter: wp.array(dtype=wp.int32), tgt: wp.array2d(dtype=wp.float32),
           body_q: wp.array(dtype=wp.transformf), body_qd: wp.array(dtype=wp.spatial_vectorf),
           body_f: wp.array(dtype=wp.spatial_vectorf), joint_q: wp.array(dtype=wp.float32),
           joint_qd: wp.array(dtype=wp.float32), joint_f: wp.array(dtype=wp.float32),
           iB: wp.array(dtype=wp.int32), sq: wp.array2d(dtype=wp.int32), sv: wp.array2d(dtype=wp.int32),
           prm: wp.array2d(dtype=wp.float32)):
    w = wp.tid()
    k = counter[0]
    tp = wp.vec3(tgt[k, 0], tgt[k, 1], tgt[k, 2])
    tq = wp.quat(tgt[k, 4], tgt[k, 5], tgt[k, 6], tgt[k, 3])
    tv = wp.vec3(tgt[k, 7], tgt[k, 8], tgt[k, 9])
    tw = wp.vec3(tgt[k, 10], tgt[k, 11], tgt[k, 12])
    width = tgt[k, 13]
    closed = tgt[k, 14]
    b = iB[w]
    X = body_q[b]
    p = wp.transform_get_translation(X)
    q = wp.transform_get_rotation(X)
    s6 = body_qd[b]
    v = wp.vec3(s6[0], s6[1], s6[2])
    om = wp.vec3(s6[3], s6[4], s6[5])
    Kt = prm[w, 0]; Dt = prm[w, 1]; Fc = prm[w, 2]; Kr = prm[w, 3]; Dr = prm[w, 4]; Tc = prm[w, 5]
    mg = prm[w, 6]; Fcl = prm[w, 7]; fK = prm[w, 8]; fDo = prm[w, 9]; fDc = prm[w, 10]
    f = Kt * (tp - p) + Dt * (tv - v)
    nf = wp.length(f)
    if nf > Fc:
        f = f * (Fc / nf)
    f = f + wp.vec3(0.0, 0.0, mg)
    qe = tq * wp.quat_inverse(q)
    if qe[3] < 0.0:
        qe = wp.quat(-qe[0], -qe[1], -qe[2], -qe[3])
    xyz = wp.vec3(qe[0], qe[1], qe[2])
    s = wp.length(xyz)
    er = 2.0 * xyz
    if s > 1.0e-9:
        er = xyz * (2.0 * wp.atan2(s, qe[3]) / s)
    tau = Kr * er + Dr * (tw - om)
    nt = wp.length(tau)
    if nt > Tc:
        tau = tau * (Tc / nt)
    body_f[b] = wp.spatial_vector(f[0], f[1], f[2], tau[0], tau[1], tau[2])
    for c in range(2):
        qj = joint_q[sq[w, c]]
        qdj = joint_qd[sv[w, c]]
        F = float(0.0)
        if closed > 0.5:
            F = -Fcl * wp.min(1.0, wp.max(qj, 0.0) / 5.0e-4) - fDc * qdj
        else:
            F = wp.clamp(fK * (0.5 * width - qj) - fDo * qdj, -Fcl, Fcl)
        joint_f[sv[w, c]] = F


@wp.kernel
def _tick(counter: wp.array(dtype=wp.int32)):
    counter[0] = counter[0] + 1


@wp.kernel
def _record(counter: wp.array(dtype=wp.int32), every: int, body_q: wp.array(dtype=wp.transformf),
            body_qd: wp.array(dtype=wp.spatial_vectorf), joint_q: wp.array(dtype=wp.float32),
            idx: wp.array2d(dtype=wp.int32), sq: wp.array2d(dtype=wp.int32), has_obj: int,
            rec: wp.array3d(dtype=wp.float32)):
    w = wp.tid()
    k = counter[0]
    if k % every != 0:
        return
    r = k / every
    if r >= rec.shape[0]:
        return
    for j in range(4):
        if j == 3 and has_obj == 0:
            continue
        X = body_q[idx[w, j]]
        p = wp.transform_get_translation(X)
        q = wp.transform_get_rotation(X)
        o = 7 * j
        rec[r, w, o + 0] = p[0]; rec[r, w, o + 1] = p[1]; rec[r, w, o + 2] = p[2]
        rec[r, w, o + 3] = q[3]; rec[r, w, o + 4] = q[0]; rec[r, w, o + 5] = q[1]; rec[r, w, o + 6] = q[2]
    if has_obj == 1:
        s6 = body_qd[idx[w, 3]]
        for i in range(6):
            rec[r, w, 28 + i] = s6[i]
    rec[r, w, 34] = joint_q[sq[w, 0]]
    rec[r, w, 35] = joint_q[sq[w, 1]]


def target_table(pb, t0, dt, n):
    T = np.zeros((n + 2, 15), np.float32)
    for k in range(n + 2):
        p, q, v, w, width, closed = pb.target(t0 + k * dt)
        T[k] = np.r_[p, q, v, w, width, float(closed)]
    return T


def run_sap_graph(scene, meta, pbdata, candidate, perturbs, workdir, objects=True, device='cuda:0',
                  world_scenes=None, cap_per_world=160, n_steps=None):
    """R rollouts as R batched worlds; world_scenes (optional) gives per-world
    drive parameters (coupling / grip cells share one model)."""
    from sap_warp import SolverSAP, load_sap_scene
    from sap_warp.collision.pipeline import SapCollisionPipeline
    from sap_warp.resources.collision_model import sap_collision_state_from_state
    from sap_warp.sap_kinematics import sap_eval_fk
    from scipy.spatial.transform import Rotation as R
    wp.init()
    Rn = len(perturbs)
    workdir = Path(workdir); workdir.mkdir(parents=True, exist_ok=True)
    cfg = sap_scene_cfg(scene, meta, candidate, objects)
    cap = cap_per_world * Rn
    cfg['simulation']['nconmax'] = cap
    (workdir / 'sap_scene.json').write_text(json.dumps(cfg, indent=1))
    L = load_sap_scene(workdir / 'sap_scene.json', device=device, naconmax=cap, strict=True, num_worlds=Rn)
    model, A, ctl = L.sap_model, L.sap_state, L.sap_control
    B = model.state()
    labels = list(model.body_label); nb = state.body_q.shape[0] // Rn if 'state' in dir() else A.body_q.shape[0] // Rn
    jl = list(model.joint_label); nj = (len(model.joint_q_start.numpy()) - 1) // Rn
    qs = model.joint_q_start.numpy(); vs = model.joint_qd_start.numpy()
    names = ['base', 'left', 'right'] + (['basket'] if objects else [])
    bidx = np.array([[w * nb + labels[:nb].index(n) for n in names] + ([0] if not objects else []) for w in range(Rn)], np.int32)
    J = lambda n, w: w * nj + jl[:nj].index(n)
    pb = Playback(pbdata, scene['finger_rule'])
    if objects:
        t0, p, q, width, bpos, bq = initial_state(scene, pb, meta)
    else:
        t0 = scene['t_start']; p, q, _, _, width, _ = pb.target(t0)
    qv = A.joint_q.numpy(); qv[:] = 0
    _, tR = table_pose(scene)
    for w in range(Rn):
        a = qs[J('base_free', w)]
        qv[a:a + 3] = p; qv[a + 3:a + 7] = q[[1, 2, 3, 0]]
        for n in ('left_slide', 'right_slide'):
            qv[qs[J(n, w)]] = width / 2
        if objects:
            pr = perturbs[w]
            bp = bpos + tR[:, 0] * pr[0] + tR[:, 1] * pr[1]
            rz = R.from_rotvec(tR[:, 2] * pr[2]).as_quat()[[3, 0, 1, 2]]
            bqq = qmul(rz, bq)
            a = qs[J('basket_free', w)]
            qv[a:a + 3] = bp; qv[a + 3:a + 7] = bqq[[1, 2, 3, 0]]
    A.joint_q.assign(qv); A.joint_qd.zero_()
    sap_eval_fk(model, A.joint_q, A.joint_qd, A)
    B.joint_q.assign(A.joint_q); B.joint_qd.zero_()
    sap_eval_fk(model, B.joint_q, B.joint_qd, B)
    kw = {'nconmax': cap_per_world, 'collect_iteration_stats': False, 'check_line_search_errors': False}
    if 'sigma' in candidate.get('parameters', {}):
        kw['contact_sigma'] = candidate['parameters']['sigma']
    kw.update(candidate.get('parameters', {}).get('solver', {}))  # any other SolverSAP option (tuning iterations)
    solver = SolverSAP(model, **kw)
    pipe = SapCollisionPipeline(L.collision_model, naconmax=cap)
    con = pipe.contacts()
    dt = scene['dt']
    n = n_steps or int(round((scene['t_end'] - scene['t_start']) / dt))
    n += n % 2
    every = max(1, int(round(1 / (TRACE_HZ * dt))))
    tgt = wp.array(target_table(pb, t0, dt, n), dtype=wp.float32, device=device)
    prm = np.zeros((Rn, 11), np.float32)
    for w in range(Rn):
        d = drive_cfg(world_scenes[w] if world_scenes else scene)
        prm[w] = [d['K_t'], 2 * np.sqrt(d['K_t'] * d['m_total']), d['F_cap'], d['K_r'], 2 * np.sqrt(d['K_r'] * d['I_base']),
                  d['T_cap'], d['m_total'] * 9.81, d['F_close'], d['finger_K'], 2 * np.sqrt(d['finger_K'] * d['pad_mass']),
                  d['finger_D_closed']]
    prm = wp.array(prm, dtype=wp.float32, device=device)
    iB = wp.array(bidx[:, 0].copy(), dtype=wp.int32, device=device)
    idx = wp.array(bidx if objects else np.c_[bidx[:, :3], bidx[:, :1]], dtype=wp.int32, device=device)
    sq = wp.array(np.array([[qs[J('left_slide', w)], qs[J('right_slide', w)]] for w in range(Rn)], np.int32), dtype=wp.int32, device=device)
    sv = wp.array(np.array([[vs[J('left_slide', w)], vs[J('right_slide', w)]] for w in range(Rn)], np.int32), dtype=wp.int32, device=device)
    counter = wp.zeros(1, dtype=wp.int32, device=device)
    nrec = n // every + 1
    rec = wp.zeros((nrec, Rn, NREC), dtype=wp.float32, device=device)
    ho = 1 if objects else 0

    def one(S0, S1):
        S0.clear_forces()
        wp.launch(_drive, Rn, inputs=[counter, tgt, S0.body_q, S0.body_qd, S0.body_f, S0.joint_q, S0.joint_qd,
                                      ctl.joint_f, iB, sq, sv, prm], device=device)
        pipe.collide(sap_collision_state_from_state(S0), con)
        solver.step(S0, S1, ctl, con, dt)
        wp.launch(_tick, 1, inputs=[counter], device=device)
        wp.launch(_record, Rn, inputs=[counter, every, S1.body_q, S1.body_qd, S1.joint_q, idx, sq, ho, rec], device=device)

    wp.launch(_record, Rn, inputs=[counter, every, A.body_q, A.body_qd, A.joint_q, idx, sq, ho, rec], device=device)
    tstart = time.perf_counter()
    # first two steps uncaptured (allocations), then capture a two-step graph
    one(A, B); one(B, A)
    with wp.ScopedCapture(device=device) as capg:
        one(A, B); one(B, A)
    for _ in range((n - 2) // 2):  # capture records but does not execute
        wp.capture_launch(capg.graph)
    wp.synchronize_device(device)
    wall = time.perf_counter() - tstart
    overflow = None
    try:
        solver.check_buffers()
    except RuntimeError as e:
        overflow = str(e)
    cnt = None
    for attr in ('rigid_contact_count', 'contact_count'):
        if hasattr(con, attr):
            try:
                cnt = int(np.asarray(getattr(con, attr).numpy()).max())
            except Exception:
                pass
    Rc = rec.numpy()
    traces = []
    for w in range(Rn):
        tr = []
        for r in range(nrec):
            x = Rc[r, w].astype(float)
            s = {'t': t0 + r * every * dt, 'base': x[0:7], 'pads': [x[7:14], x[14:21]], 'slider': [x[34], x[35]]}
            if objects:
                s['basket'] = x[21:28]; s['basket_v'] = x[28:34]
            tr.append(s)
        traces.append(tr)
    healths = []
    for w in range(Rn):
        fin = bool(np.isfinite(Rc[:, w]).all())
        healths.append({'finite': fin, 'truncated': 1 if overflow else 0, 'overflow': overflow,
                        'contact_count_last': cnt, 'contact_capacity': cap,
                        'wall_s': wall / Rn, 'wall_batch_s': wall, 'steps': n, 'cuda_graph': True,
                        'engine_peak_overlap_m': {}})
    return traces, healths
