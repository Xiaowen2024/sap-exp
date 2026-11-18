"""Genesis 1.4.6 SAP coupler backend for RH20T dish-on-rack playback.

Same asset files, drive law and observer as the other backends.  Each body is
its own MJCF entity (gripper = free base + two slide pads; object = free body;
rack and table = fixed bodies).  R rollouts are R Genesis environments.  The
base wrench is applied with apply_links_external_wrench (world frame, link
origin = COM); finger forces with control_dofs_force on the slide dofs.
Friction: Genesis combines rigid pairs as sqrt(mu_a mu_b), so entity values
are chosen to reproduce the declared pad-object and object-environment pairs
exactly (pad-environment derived).  Precision 64 as in the Genesis SAP SOP.
"""
from __future__ import annotations

import os
import tempfile
import time
from pathlib import Path

import numpy as np

from .rp_engines import TRACE_HZ, drive_cfg, initial_state, nums, pad_local_center, base_wrench_batch
from .rp_sim import Playback, finger_force, place_on_table, qmat, qmul, table_pose

CANDIDATES = {
    'native': {},
    'hs1e7_td01': {'hydroelastic_stiffness': 1e7, 'sap_taud': 0.01, 'n_sap_iterations': 20},
    'hs1e8_td01': {'hydroelastic_stiffness': 1e8, 'sap_taud': 0.01, 'n_sap_iterations': 20},
    'hs1e9_td01': {'hydroelastic_stiffness': 1e9, 'sap_taud': 0.01, 'n_sap_iterations': 20},
    'hs1e8_td001': {'hydroelastic_stiffness': 1e8, 'sap_taud': 0.001, 'n_sap_iterations': 20},
}

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


def run_genesis(scene, meta, pbdata, candidate, perturbs, workdir, objects=True):
    gs = gs_init()
    from scipy.spatial.transform import Rotation as R
    Rn = len(perturbs)
    workdir = Path(workdir); workdir.mkdir(parents=True, exist_ok=True)
    par = dict(candidate.get('parameters', {}))
    if 'n_sap_iterations' in par:
        par['n_sap_iterations'] = int(par['n_sap_iterations'])
    simopt = gs.options.SimOptions(dt=scene['dt'], substeps=1, gravity=(0., 0., -9.81))
    gscene = gs.Scene(sim_options=simopt, coupler_options=gs.options.SAPCouplerOptions(**par), show_viewer=False)
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
                                       material=gs.materials.Rigid(friction=mu, coup_friction=mu))
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
    gscene.build(n_envs=Rn)
    G, B = ents['gripper'], ents.get('basket')
    pb = Playback(pbdata, scene['finger_rule'])
    if objects:
        t0, p, q, width, bpos, bq = initial_state(scene, pb, meta)
    else:
        t0 = scene['t_start']; p, q, _, _, width, _ = pb.target(t0)
    qg = np.tile(np.r_[p, q, width / 2, width / 2], (Rn, 1))
    G.set_qpos(qg, zero_velocity=True)
    if objects:
        _, tR = table_pose(scene)
        qb = np.zeros((Rn, 7))
        for w, pr in enumerate(perturbs):
            rz = R.from_rotvec(tR[:, 2] * pr[2]).as_quat()[[3, 0, 1, 2]]
            qb[w] = np.r_[bpos + tR[:, 0] * pr[0] + tR[:, 1] * pr[1], qmul(rz, bq)]
        B.set_qpos(qb, zero_velocity=True)
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
            bqp = np_(B.get_qpos()); bv = np_(B.get_links_vel(links_idx_local=[0]))[:, 0]; bw = np_(B.get_links_ang(links_idx_local=[0]))[:, 0]
        for w in range(Rn):
            s = {'t': t, 'base': np.r_[lp[w, 0], lq[w, 0]], 'pads': [np.r_[lp[w, 1], lq[w, 1]], np.r_[lp[w, 2], lq[w, 2]]],
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
            if not all(np.isfinite(tr[-1]['base']).all() for tr in traces):
                break
    wall = time.perf_counter() - tstart
    healths = []
    for w in range(Rn):
        fin = bool(all(np.isfinite(s['base']).all() and (not objects or np.isfinite(s['basket']).all()) for s in traces[w][-3:]))
        healths.append({'finite': fin, 'truncated': 0, 'wall_s': wall / Rn, 'wall_batch_s': wall, 'steps': k + 1,
                        'friction_entities': mus, 'engine_peak_overlap_m': {}})
    return traces, healths
