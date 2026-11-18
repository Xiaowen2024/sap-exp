"""Describe smoke outcomes without treating an exploratory run as B=5 evidence."""
import argparse
import json
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R


def summarize(root, revision):
    root=Path(root);scene=json.loads((root/'prepared/scene.json').read_text());asset=scene['assets']['bowl']
    meshes=[trimesh.load(root/'prepared'/p,force='mesh',process=False) for p in asset['pieces']]
    vertices=np.vstack([m.vertices for m in meshes]);eqs=[ConvexHull(m.vertices).equations for m in meshes]
    rng=np.random.default_rng(20261117);samples=vertices[rng.choice(len(vertices),min(1024,len(vertices)),replace=False)]
    def penetration(points):
        depth=np.zeros(len(points))
        for e in eqs:depth=np.maximum(depth,-np.max(points@e[:,:3].T+e[:,3],axis=1))
        return float(depth.max())
    report={'kind':'exploratory_R1_object_smoke_not_benchmark','comparison_complete':False,
            'reference':'official simulated demonstration; no recorded object-pose or contact-force ground truth',
            'completion':scene['completion'],'deviations':scene['deviations'],
            'thresholds':{'xy_m':.02,'min_vertical_separation_m':.005,'max_vertical_separation_m':.7*.06,
                          'upright_deg':10.,'tail_speed_m_s':.01,'tail_angular_speed_rad_s':.1},'engines':{}}
    for engine in ('mujoco','sap','ipc'):
        path=root/'runs'/f'{engine}_objects_{revision}'
        if not (path/'health.json').exists():
            report['engines'][engine]={'status':'not_run_or_incomplete'};continue
        health=json.loads((path/'health.json').read_text());tr=np.load(path/'trace.npz',allow_pickle=False);t=tr['time']
        names=['bowl0','bowl1','bowl2'];origins={};tilt={}
        for n in names:
            p=tr[n+'__pose'];rot=R.from_quat(p[:,[4,5,6,3]])
            origins[n]=p[:,:3]-rot.apply(asset['com']);tilt[n]=np.degrees(np.arccos(np.clip(rot.as_matrix()[:,2,2],-1,1)))
        def nest_at(index, wanted):
            positions=np.array([origins[n][index] for n in wanted]);order=np.argsort(positions[:,2]);positions=positions[order]
            return bool(np.all(np.linalg.norm(np.diff(positions[:,:2],axis=0),axis=1)<.02)
                        and np.all(np.diff(positions[:,2])>.005) and np.all(np.diff(positions[:,2])<.042)
                        and all(tilt[n][index]<10 for n in wanted))
        tail=t>=t[-1]-.5;final_nest=nest_at(-1,names);speed={}
        for n in names:
            v=tr[n+'__velocity'][tail];speed[n]={'linear_max_m_s':float(np.linalg.norm(v[:,:3],axis=1).max()),
                                              'angular_max_rad_s':float(np.linalg.norm(v[:,3:],axis=1).max())}
        settled=all(v['linear_max_m_s']<.01 and v['angular_max_rad_s']<.1 for v in speed.values())
        overlap={}
        for a in names:
            pa=tr[a+'__pose'][-1];ra=R.from_quat(pa[[4,5,6,3]]);world=ra.apply(samples)+pa[:3]
            overlap[a+'/table']=float(max(0.,scene['table_top_z']-np.min(ra.apply(vertices)[:,2]+pa[2])))
            for b in names:
                if a==b:continue
                pb=tr[b+'__pose'][-1];rb=R.from_quat(pb[[4,5,6,3]]);overlap[a+'/'+b]=penetration(rb.inv().apply(world-pb[:3]))
        flags=[]
        if max(overlap.values())>.001:flags.append('sampled_final_overlap_exceeds_1mm')
        if not health['finite'] or health['steps']!=health['expected_steps']:flags.append('incomplete_or_nonfinite')
        if health.get('truncated',0):flags.append('contact_capacity_truncation')
        if health.get('affine_deformation_max',0)>.01:flags.append('affine_strain_exceeds_rigid_tolerance')
        if engine=='ipc':flags.append('native_contact_capacity_and_overlap_not_audited')
        row={'status':'completed_smoke','R':1,'solver_configurations_tested':1,'finite':health['finite'],'steps':health['steps'],
             'lifted_right_bowl':bool((origins['bowl2'][:,2]-origins['bowl2'][0,2]).max()>.02),
             'first_two_nested_at_5p2s':nest_at(int(np.argmin(abs(t-5.2))),['bowl0','bowl2']),
             'lifted_left_bowl':bool((origins['bowl1'][:,2]-origins['bowl1'][0,2]).max()>.02),
             'final_nested_geometry':final_nest,'settled':settled,
             'geometry_and_settling_reproduced':bool(final_nest and settled),
             'task_smoke_success':bool(final_nest and settled and not flags),
             'final_origins_m':{n:origins[n][-1].tolist() for n in names},
             'final_tilt_deg':{n:float(tilt[n][-1]) for n in names},'tail_speed':speed,
             'sampled_final_overlap_m':overlap,'flags':flags,
             'overlap_caveat':'sampled convex-union interior depth is a lower bound, not a continuous maximum penetration certificate'}
        report['engines'][engine]=row
    (root/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('root');ap.add_argument('--revision',default='r2');a=ap.parse_args();summarize(a.root,a.revision)
