"""Analytic, object-free API probes. These are never task benchmark rows."""
from __future__ import annotations

from pathlib import Path
import numpy as np

from .io_utils import GateError, write_json
from .runner import backend, gpu_host_gate, package_hash
from .scene import catalog


def fixture():
    return {'dt_s':.0005,'table_z_m':0.,'contact_capacity':256,'jacobian_capacity':4096,'ipc_abd_kappa_Pa':1e8,
            'contact_tolerance_m':1e-5,'mu_pad_plate':.5,'mu_plate_environment':.3,'mu_pad_environment':.3,
            'pad':{'size_m':[.01,.04,.02],'mass_kg':.05,'stroke_m':.1},
            'drive':{'base_mass_kg':1.,'base_inertia_kg_m2':[.002,.002,.002],
                     'gravity_m_s2':[0,0,0],'K_t_N_m':1e4,'K_r_Nm_rad':300.,'force_cap_N':50.,'torque_cap_Nm':5.,
                     'closing_force_N':5.,'finger_damping_Ns_m':1.,'finger_K_N_m':1e4,'finger_force_cap_N':10.},
            'plate':{'mass_kg':.1,'profile':'shallow_rim','sectors':12,
                     'profiles_radius_lower_upper_m':{
                         'shallow_rim':[[0,-.001,.001],[.08,-.001,.001],[.1,.004,.006]],
                         'flat_coupe':[[0,-.001,.001],[.1,-.001,.001]],
                         'raised_rim':[[0,-.001,.001],[.08,-.001,.001],[.1,.009,.011]]}},
            'rack':{'members':[]}}


def api_probe(engine, output):
    import resource
    resource.setrlimit(resource.RLIMIT_CORE,(0,0))
    gpu_host_gate()
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    scene=fixture();assets=catalog(scene,output/'assets')
    candidate={'id':'native_default','native':True,'parameters':{}}
    report={'kind':'analytic_api_probe_not_task_benchmark','engine':engine,'runner_sha256':package_hash(),
            'objects_present':False,'passed':False}
    sim=None
    try:
        sim=backend(engine,scene,assets,np.array([0,0,.5,0,0,0,1.]),.04,candidate,output,objects=False)
        first=sim.snapshot();dt=scene['dt_s'];steps=40;duration=dt*steps
        for _ in range(steps):sim.step(np.array([0,0,.1,0,0,0.]),[0.,0.])
        linear=sim.snapshot();expected=.1/(1.+2*.05)*duration
        actual=float(linear['twists']['base'][2]-first['twists']['base'][2])
        force_pass=abs(actual-expected)<=max(1e-6,.1*abs(expected))
        # 0.1 Nm produces an angular increment above the native IPC stopping
        # tolerance; a sub-resolution load must not certify the wrench route.
        torque=.1
        for _ in range(steps):sim.step(np.array([0,0,0,0,0,torque]),[0.,0.])
        rotated=sim.snapshot()
        padIz=.05/12*(.01**2+.04**2)
        effectiveI=.002+2*(padIz+.05*.025**2)
        expected_w=torque/effectiveI*duration
        actual_w=float(rotated['twists']['base'][5]-linear['twists']['base'][5])
        torque_pass=abs(actual_w-expected_w)<=max(1e-6,.1*abs(expected_w))
        before=rotated['slider_qd_m_s'].copy()
        for _ in range(steps):sim.step(np.zeros(6),[.1,.1])
        slid=sim.snapshot();actual_slide=slid['slider_qd_m_s']-before
        expected_slide=.1/.05*duration
        slider_pass=bool(np.all(abs(actual_slide-expected_slide)<=.1*expected_slide))
        report.update(force_world_z={'expected_delta_v_m_s':expected,'actual_delta_v_m_s':actual,'passed':force_pass},
                      torque_world_z={'expected_delta_omega_rad_s':expected_w,'actual_delta_omega_rad_s':actual_w,'passed':torque_pass},
                      sliders_outward={'expected_delta_qd_m_s':expected_slide,'actual_delta_qd_m_s':actual_slide.tolist(),'passed':slider_pass},
                      runtime=sim.audit(),passed=bool(force_pass and torque_pass and slider_pass and slid['valid']))
    except Exception as exc:
        report.update(error_type=type(exc).__name__,error=str(exc))
    finally:
        if sim is not None:sim.close()
    write_json(output/'api_probe.json',report)
    return report
