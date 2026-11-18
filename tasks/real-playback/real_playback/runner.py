"""GPU-host rollout execution with P3/P4/P5 gates and immutable outputs."""
from __future__ import annotations

from pathlib import Path
import copy
import importlib
import platform
import shutil
import subprocess
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .drive import Playback, base_wrench, finger_force
from .io_utils import GateError, digest, read_json, sha256, verified_file, write_json
from .manifest import validate
from .matrix import apply_cell
from .metrics import evaluate
from .observer import Observer
from .scene import catalog


def package_hash():
    return digest({p.name:sha256(p) for p in sorted(Path(__file__).parent.glob('*.py'))})


def gpu_host_gate():
    if platform.system()!='Linux' or shutil.which('nvidia-smi') is None:
        raise GateError('raw data and simulation must run on the GPU server')
    subprocess.run(['nvidia-smi','--query-gpu=name','--format=csv,noheader'],check=True,capture_output=True)


def backend(engine, *args, **kwargs):
    names={'mujoco':'MuJoCoBackend','sap':'SAPBackend','ipc':'IPCBackend'}
    module=importlib.import_module('.backend_'+engine,__package__)
    return getattr(module,names[engine])(*args,**kwargs)


def load_playback(root, episode):
    path=verified_file(root,episode['playback_npz'])
    with np.load(path,allow_pickle=False) as data:
        playback=Playback(data['tcp_t_s'],data['tcp_pose_xyzw'],data['gripper_t_s'],data['gripper_width_m'],
                          episode['tool_to_gripper_base'])
    for key,actual in [('start_s',playback.start),('release_s',playback.release),
                       ('withdrawal_s',playback.withdrawal),('end_s',playback.end)]:
        if abs(episode['events'][key]-actual)>1e-6:
            raise GateError('frozen event disagrees with real-only state machine: '+key)
    return playback


def integrate(scene, episode, playback, assets, engine, candidate, output, *, objects, verified_observer=False):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    target,_,width,_=playback.target(playback.start)
    sim=backend(engine,scene,assets,target,width,candidate,output,objects=objects)
    observer=Observer(scene,assets) if objects else None
    trace={k:[] for k in ('t_s','base_pose_xyzw','base_twist_world','width_m','coupling_wrench_world',
                         'plate_pose_xyzw','plate_twist_world','pad_contacts','rack_contact','table_contact',
                         'plate_lowest_z_m','contact_truncated','tunnelling','affine_deformation_max','pad_pose_xyzw','slider_q_m')}
    trace['overlap_m']={k:[] for k in ('pad_plate','plate_rack','plate_table')}
    trace['health_observer_verified']=verified_observer;trace['engine_valid']=True
    errors_p,errors_r=[],[]
    steps=round((playback.end-playback.start)/scene['dt_s']);start=time.perf_counter()
    try:
        for k in range(steps+1):
            t=playback.start+k*scene['dt_s']
            target,target_v,width,closed=playback.target(min(t,playback.end))
            s=sim.snapshot()
            if not s['valid']:
                trace['engine_valid']=False
                raise GateError(f'engine invalid at native step {k}')
            bp,bv=s['poses']['base'],s['twists']['base']
            wrench=base_wrench(bp,bv,target,target_v,scene['drive'])
            errors_p.append(float(np.linalg.norm(bp[:3]-target[:3])))
            errors_r.append(float((Rotation.from_quat(bp[3:]).inv()*Rotation.from_quat(target[3:])).magnitude()))
            trace['t_s'].append(t);trace['base_pose_xyzw'].append(bp.tolist());trace['base_twist_world'].append(bv.tolist())
            trace['width_m'].append(float(np.sum(s['slider_q_m'])));trace['coupling_wrench_world'].append(wrench.tolist())
            trace['pad_pose_xyzw'].append([s['poses'][n].tolist() for n in ('left','right')])
            trace['slider_q_m'].append(s['slider_q_m'].tolist())
            if objects:
                measure=observer.sample(s)
                trace['plate_pose_xyzw'].append(s['poses']['plate'].tolist());trace['plate_twist_world'].append(s['twists']['plate'].tolist())
                for key in ('pad_contacts','rack_contact','table_contact','plate_lowest_z_m','tunnelling'):trace[key].append(measure[key])
                trace['contact_truncated'].append(s['contact_truncated'])
                trace['affine_deformation_max'].append(s.get('affine_deformation_max',0.))
                for key,value in measure['overlap_m'].items():trace['overlap_m'][key].append(value)
            if k<steps:
                fingers=[finger_force(q,v,width,closed,scene['drive']) for q,v in zip(s['slider_q_m'],s['slider_qd_m_s'])]
                sim.step(wrench,fingers)
        audit=sim.audit()
        write_json(output/'runtime.json',audit)
        # Dense traces stay on the server. They contain no real RGB-D data.
        write_json(output/'trace.json',trace)
        tracking={'max_position_error_m':max(errors_p),'max_orientation_error_rad':max(errors_r),
                  'passed':max(errors_p)<=.001 and max(errors_r)<=np.deg2rad(.5)}
        row=evaluate(trace,scene,episode) if objects else {'drive_gate':tracking['passed']}
        return {**row,'tracking':tracking,'wall_s':time.perf_counter()-start,'runtime':audit}
    finally:
        sim.close()


def execute_job(manifest, job, input_root, output_root, evidence):
    """Execute one R=20 cell only after its required phase evidence passes.

    An immutable job result is reused on resume only when the complete input
    fingerprint matches. An interrupted directory is never counted as a row.
    """
    gpu_host_gate();validate(manifest,input_root)
    split=job['split'];engine=job['engine']
    episode=next(e for e in manifest['episodes'][split] if e['id']==job['episode_id'])
    sha=manifest['declaration_sha256']
    expected={'manifest_sha256':sha,'runner_sha256':package_hash()}
    for phase in ('preflight','drive'):
        artifact=evidence[phase][engine]
        if not artifact['passed'] or any(artifact[k]!=v for k,v in expected.items()):
            raise GateError(f'{phase} evidence missing, failed or stale')
    if not evidence['preflight'][engine]['geometry_observer_verified']:
        raise GateError('common overlap/contact/tunnelling observer has not passed QA')
    if split=='held_out':
        selection=evidence['selection'][engine]
        if selection['manifest_sha256']!=sha or not selection['complete']:
            raise GateError('held-out objects are forbidden before frozen P4 selection')
        if job['budget']==5 and job['candidate']['id']!=selection['winner']['id']:
            raise GateError('held-out candidate differs from frozen selection')
    else:
        smoke=evidence['smoke'][engine]
        if not smoke['passed'] or smoke['manifest_sha256']!=sha or not smoke['video_verified']:
            raise GateError('P4 requires selection-only P3b smoke and its video')
    path=Path(output_root)/engine/split/job['job_id']
    fingerprint=digest({'manifest_sha256':sha,'job':job,'runner_sha256':package_hash()})
    result_path=path/'result.json'
    if result_path.exists():
        result=read_json(result_path)
        if result['input_sha256']!=fingerprint:raise GateError('resume input fingerprint differs')
        return result
    if path.exists():raise GateError('interrupted job exists; preserve it and choose a new attempt directory')
    path.mkdir(parents=True)
    write_json(path/'job.json',job)
    playback=load_playback(input_root,episode)
    physical=apply_cell(episode['scene'],job['cell_id'])
    assets=catalog(physical,path/'assets')
    rows=[]
    for r,noise in enumerate(job['perturbations_xy_yaw']):
        scene=copy.deepcopy(physical)
        scene['plate']['position_m']= (np.asarray(scene['plate']['position_m'])+np.r_[noise[:2],0.]).tolist()
        scene['plate']['quaternion_xyzw']=(Rotation.from_rotvec([0,0,noise[2]])*Rotation.from_quat(scene['plate']['quaternion_xyzw'])).as_quat().tolist()
        attempt=path/f'R{r:02d}'
        try:
            row=integrate(scene,episode,playback,assets,engine,job['candidate'],attempt,objects=True,verified_observer=True)
        except GateError as exc:
            # Runtime setup failure is explicit; it is never silently a valid
            # not-retained outcome on a real-failure episode.
            row={'valid':False,'outcome':'unknown','real_label':episode['real_label'],
                 'flags':['invalid'],'invalid_reasons':[str(exc)],'first_failed_stage':None}
        row.update(rollout=r,episode_id=episode['id'],completion_sha256=episode['completion_sha256'],
                   cell_id=job['cell_id'],seed=job['seed'],engine=engine,candidate=job['candidate']['id'])
        rows.append(row);write_json(attempt/'row.json',row)
    result={'input_sha256':fingerprint,'manifest_sha256':sha,'runner_sha256':package_hash(),'job':job,
            'complete':True,'rows':rows}
    write_json(result_path,result)
    return result
