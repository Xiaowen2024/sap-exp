"""Inspect matched pilot traces without promoting them to benchmark evidence."""
import argparse
import hashlib
import json
from pathlib import Path
from functools import lru_cache
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
import trimesh


@lru_cache(maxsize=4)
def bowl_geometry(paths):
    vertices=[np.asarray(trimesh.load(p,force='mesh',process=False).vertices) for p in paths]
    all_vertices=np.vstack(vertices);rng=np.random.default_rng(20261117)
    sample=all_vertices[rng.choice(len(all_vertices),min(1024,len(all_vertices)),replace=False)]
    return all_vertices,sample,[ConvexHull(v).equations for v in vertices]


def inspect(path):
    scene=json.loads((path/'input_scene.json').read_text());health=json.loads((path/'health.json').read_text())
    with np.load(path/'trace.npz',allow_pickle=False) as tr:
        times=tr['time'];tail=times>=times[-1]-.5;bowls={};origins={};tilts={}
        for name in ('bowl0','bowl1','bowl2'):
            pose=tr[name+'__pose'];r=Rotation.from_quat(pose[:,[4,5,6,3]])
            origin=pose[:,:3]-r.apply(scene['assets']['bowl']['com']);origins[name]=origin
            tilt=np.degrees(np.arccos(np.clip(r.as_matrix()[:,2,2],-1,1)));tilts[name]=tilt
            v=tr[name+'__velocity'][tail]
            bowls[name]={'max_lift_m':float((origin[:,2]-origin[0,2]).max()),'final_origin_m':origin[-1].tolist(),
                'final_tilt_deg':float(tilt[-1]),'tail_linear_max_m_s':float(np.linalg.norm(v[:,:3],axis=1).max()),
                'tail_angular_max_rad_s':float(np.linalg.norm(v[:,3:],axis=1).max())}
        def nesting(names,index):
            p=np.array([origins[n][index] for n in names]);p=p[np.argsort(p[:,2])]
            xy=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1);dz=np.diff(p[:,2])
            return {'nested':bool(np.all(xy<.02) and np.all(dz>.005) and np.all(dz<.042)
                and all(tilts[n][index]<10 for n in names)),'adjacent_xy_m':xy.tolist(),'adjacent_dz_m':dz.tolist()}
        final=nesting(list(bowls),-1);first=nesting(['bowl0','bowl2'],int(np.argmin(abs(times-5.2))))
        vertices,sample,equations=bowl_geometry(tuple(scene['assets']['bowl']['pieces']));overlap={}
        for name in bowls:
            pose=tr[name+'__pose'][-1];r=Rotation.from_quat(pose[[4,5,6,3]]);world=r.apply(sample)+pose[:3]
            overlap[name+'/table']=float(max(0.,scene['table_top_z']-(r.apply(vertices)[:,2]+pose[2]).min()))
            for other in bowls:
                if name==other:continue
                target=tr[other+'__pose'][-1];rt=Rotation.from_quat(target[[4,5,6,3]])
                points=rt.inv().apply(world-target[:3]);depth=np.zeros(len(points))
                for eq in equations:depth=np.maximum(depth,-np.max(points@eq[:,:3].T+eq[:,3],axis=1))
                overlap[name+'/'+other]=float(depth.max())
    settled=all(b['tail_linear_max_m_s']<.01 and b['tail_angular_max_rad_s']<.1 for b in bowls.values())
    return {'path':str(path),'health':{k:v for k,v in health.items() if not isinstance(v,(dict,list))},
        'tracking':health['tracking'],'bowls':bowls,'final_geometry':final,'first_pair_at_5p2s':first,
        'settled':settled,'geometry_and_settling_reproduced':bool(final['nested'] and settled),
        'sampled_final_overlap_m':overlap,'sampled_final_overlap_exceeds_1mm':bool(max(overlap.values())>.001),
        'overlap_caveat':'convex-union interior depth at sampled hull vertices; lower bound, not a continuous penetration certificate',
        'trace_sha256':hashlib.sha256((path/'trace.npz').read_bytes()).hexdigest(),
        'caveat':'exploratory single rollout; sampled traces; final overlap/contact-force audit and uncertainty sweep not performed'}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',required=True);ap.add_argument('--original',required=True);a=ap.parse_args()
    root=Path(a.root);original=Path(a.original)
    paths={name:root/'runs'/name for name in ('sap_objects_r3_newton','sap_objects_r3_legacy','sap_objects_r2_newton','sap_objects_r2_legacy')}
    paths.update({name:original/'runs'/name for name in ('mujoco_objects_r3','sap_objects_r2')})
    report={'kind':'matched_single_rollout_diagnostic_not_benchmark','comparison_complete':False,
        'thresholds':{'adjacent_xy_m':.02,'vertical_separation_m':[.005,.042],'upright_deg':10.,
                      'tail_linear_m_s':.01,'tail_angular_rad_s':.1},'runs':{name:inspect(path) for name,path in paths.items()}}
    (root/'summary.json').write_text(json.dumps(report,indent=2))
    for name,row in report['runs'].items():
        print(name,json.dumps({'nested':row['final_geometry']['nested'],'settled':row['settled'],
            'first_pair':row['first_pair_at_5p2s']['nested'],'lift_m':{n:b['max_lift_m'] for n,b in row['bowls'].items()},
            'tilt_deg':{n:b['final_tilt_deg'] for n,b in row['bowls'].items()},
            'tracking_pass':row['health']['drive_gate_pass'],'peak_overlap_m':row['health']['engine_peak_overlap_m']}),flush=True)


if __name__=='__main__':main()
