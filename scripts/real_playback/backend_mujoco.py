"""MuJoCo CPU integrator on the GPU host; explicit external base wrench.

This is MuJoCo, not MuJoCo Warp. Runtime is never compared to CUDA SAP/IPC.
The adapter still requires the GPU-host preflight and P3a before object runs.
"""
from __future__ import annotations

from pathlib import Path
import numpy as np

from .scene import make_mjcf


class MuJoCoBackend:
    def __init__(self, scene, assets, base_pose, width, candidate, output, objects=True):
        import mujoco as mj
        self.mj, self.scene, self.objects = mj, scene, objects
        self.xml = make_mjcf(scene, assets, base_pose, width, objects=objects, candidate=candidate)
        Path(output).mkdir(parents=True, exist_ok=True)
        (Path(output)/'scene.xml').write_text(self.xml)
        self.model = m = mj.MjModel.from_xml_string(self.xml)
        self.data = d = mj.MjData(m)
        self.ids = {n: mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, n)
                    for n in ['base', 'left', 'right'] + (['plate', 'rack'] if objects else [])}
        self.slides = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n+'_slide') for n in ('left', 'right')]
        for j in self.slides:
            d.qpos[m.jnt_qposadr[j]] = width/2
        mj.mj_forward(m, d)
        self.warnings0 = np.array([w.number for w in d.warning])
        self.candidate = candidate

    def snapshot(self):
        mj, m, d = self.mj, self.model, self.data
        # Refresh collision at the actual accepted state, never initial or stale.
        mj.mj_forward(m, d)
        poses, twists = {}, {}
        for name, i in self.ids.items():
            poses[name] = np.r_[d.xpos[i], d.xquat[i][[1,2,3,0]]].copy()
            velocity = np.zeros(6)
            mj.mj_objectVelocity(m, d, mj.mjtObj.mjOBJ_BODY, i, velocity, 0)
            twists[name] = np.r_[velocity[3:], velocity[:3]]
        q = np.array([d.qpos[m.jnt_qposadr[j]] for j in self.slides])
        qd = np.array([d.qvel[m.jnt_dofadr[j]] for j in self.slides])
        pairs = []
        for contact in d.contact[:d.ncon]:
            names = tuple(mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, int(g)) for g in contact.geom)
            pairs.append((names, float(contact.dist)))
        trunc = any(d.warning[int(kind)].number > self.warnings0[int(kind)] for kind in
                    (mj.mjtWarning.mjWARN_CONTACTFULL, mj.mjtWarning.mjWARN_CNSTRFULL))
        bad = any(d.warning[int(kind)].number > self.warnings0[int(kind)] for kind in
                  (mj.mjtWarning.mjWARN_BADQPOS, mj.mjtWarning.mjWARN_BADQVEL, mj.mjtWarning.mjWARN_BADQACC))
        return {'poses': poses, 'twists': twists, 'slider_q_m': q, 'slider_qd_m_s': qd,
                'contact_pairs': pairs, 'contact_truncated': bool(trunc),
                'valid': not bad and bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()),
                'contact_count': int(d.ncon)}

    def step(self, base_wrench, fingers):
        d, m = self.data, self.model
        d.xfrc_applied[:] = 0
        d.qfrc_applied[:] = 0
        d.xfrc_applied[self.ids['base']] = base_wrench  # MuJoCo: force then torque, world COM.
        for j, f in zip(self.slides, fingers):
            d.qfrc_applied[m.jnt_dofadr[j]] = f
        self.mj.mj_step(m, d)

    def audit(self):
        m = self.model
        return {'engine': 'MuJoCo CPU', 'version': self.mj.__version__,
                'candidate': self.candidate, 'dt_s': float(m.opt.timestep),
                'device': 'CPU on GPU host', 'timing_comparable_to_cuda': False,
                'compiled': {'solver': int(m.opt.solver), 'integrator': int(m.opt.integrator),
                             'iterations': int(m.opt.iterations), 'tolerance': float(m.opt.tolerance),
                             'impratio': float(m.opt.impratio), 'noslip_iterations': int(m.opt.noslip_iterations),
                             'pair_solref': m.pair_solref.tolist(), 'pair_solimp': m.pair_solimp.tolist(),
                             'pair_friction': m.pair_friction.tolist(),
                             'body_mass': m.body_mass.tolist(), 'body_inertia': m.body_inertia.tolist()},
                'contact_capacity': self.scene['contact_capacity']}

    def close(self):
        pass
