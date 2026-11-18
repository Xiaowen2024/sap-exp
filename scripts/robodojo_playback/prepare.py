"""Build a shared reduced-gripper scene from pinned official assets.

The four fingers are independently driven free bodies. Arms/palms/cameras
are omitted. This is explicitly a reproduction pilot, not native RoboDojo.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R, Slerp


def decompose(mesh, out, *, threshold, limit):
    import coacd
    coacd.set_log_level('warn')
    out.mkdir(parents=True, exist_ok=True)
    params = dict(threshold=threshold, max_convex_hull=limit, preprocess_mode='auto', resolution=2000,
                  mcts_nodes=10, mcts_iterations=80, mcts_max_depth=3, pca=False, merge=True, seed=20261117)
    pieces = coacd.run_coacd(coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.faces)), **params)
    for i, (v, f) in enumerate(pieces): trimesh.Trimesh(v, f, process=False).export(out / f'piece_{i:03d}.obj')
    return params


def main(root):
    from pxr import Usd, UsdGeom
    root = Path(root)
    out = root / 'prepared'; out.mkdir(exist_ok=True)
    completion = json.loads((root / 'initial_fit.json').read_text())['candidates'][0]
    selected_model = completion['model']
    source_folder = root / f'bowl{selected_model}'
    metadata = json.loads((source_folder / 'metadata.json').read_text())
    stage = Usd.Stage.Open(str(next(source_folder.glob('*.usd*'))))
    prim = stage.GetPrimAtPath('/root/collision/model'); usd = UsdGeom.Mesh(prim)
    if not np.all(np.asarray(usd.GetFaceVertexCountsAttr().Get()) == 3): raise ValueError('requires triangulated mesh')
    v = np.asarray(usd.GetPointsAttr().Get(), float)
    T = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0), float).T
    v = v @ T[:3, :3].T + T[:3, 3]
    mesh = trimesh.Trimesh(v, np.asarray(usd.GetFaceVertexIndicesAttr().Get(), int).reshape(-1, 3), process=True)
    mesh = mesh.simplify_quadric_decimation(face_count=8000)
    mesh.export(out / 'bowl_reference.obj')
    params = {}
    source_pieces = source_folder / 'pieces'
    if not list(source_pieces.glob('piece_*.obj')):
        params['bowl'] = decompose(mesh, source_pieces, threshold=.01, limit=64)
    else:
        params['bowl'] = {'threshold': .01, 'max_convex_hull': 64, 'seed': 20261117,
                          'resolution': 2000, 'mcts_nodes': 10, 'mcts_iterations': 80, 'mcts_max_depth': 3,
                          'pca': False, 'merge': True, 'preprocess_mode': 'auto'}
    urdf = ET.parse(root / 'X5A.urdf').getroot()
    assets = {}
    for kind in ('bowl', 'link7', 'link8'):
        folder = source_pieces if kind == 'bowl' else root / (kind + '_pieces')
        if kind != 'bowl' and not list(folder.glob('piece_*.obj')):
            params[kind] = decompose(trimesh.load(root / (kind + '.STL'), force='mesh'), folder, threshold=.01, limit=8)
        parts = [trimesh.load(f, force='mesh', process=False) for f in sorted(folder.glob('piece_*.obj'))]
        aggregate = trimesh.util.concatenate(parts)
        mass = metadata['physics']['mass'] if kind == 'bowl' else .064798
        props = aggregate.mass_properties; com = props.center_mass
        dst = out / kind; dst.mkdir(exist_ok=True)
        for old in dst.glob('piece_*.obj'): old.unlink()
        paths = []
        for i, piece in enumerate(parts):
            piece.vertices -= com
            f = dst / f'piece_{i:03d}.obj'; piece.export(f); paths.append(str(f.relative_to(out)))
        assets[kind] = {'pieces': paths, 'mass': mass, 'com': com.tolist(),
                        'inertia': (props.inertia * mass / props.mass).tolist(),
                        'additive_volume_m3': float(props.volume)}
    layout = json.loads((root / 'stack_bowls_0.json').read_text())
    data = np.load(root / 'stack_bowls/numeric.npz', allow_pickle=False)
    n = len(data['state__left_ee_poses']); hz = float(data['additional_info__frequency'])
    dt, settle = .001, 1.
    times = np.arange(int(round(((n - 1) / hz + settle) / dt)) + 1) * dt
    source_t = np.arange(n) / hz; clamped = np.clip(times, 0, source_t[-1])
    bodies, targets = [], {}
    image_labels = {'bowl0':'middle','bowl1':'left','bowl2':'right'}
    for obj in layout['Rigid']['bowl']:
        q = np.asarray(obj['default_ori'], float); rot = R.from_quat(q[[1, 2, 3, 0]])
        p = np.asarray(completion['positions_m'][image_labels[obj['label']]]) + rot.apply(assets['bowl']['com'])
        p[2] += .0002  # shared positive initial separation, including MuJoCo/SAP
        bodies.append({'name': obj['label'], 'asset': 'bowl', 'p': p.tolist(), 'q': q.tolist(), 'driven': False})
    for side in ('left', 'right'):
        poses = data[f'state__{side}_ee_poses']
        positions = np.column_stack([np.interp(clamped, source_t, poses[:, i]) for i in range(3)])
        rotations = Slerp(source_t, R.from_quat(poses[:, [4, 5, 6, 3]]))(clamped)
        jaw = -.01 + .054 * np.interp(clamped, source_t, data[f'state__{side}_ee_joint_states'][:, 0])
        for link in ('link7', 'link8'):
            joint = urdf.find("joint[@name='joint" + link[-1] + "']")
            origin = np.fromstring(joint.find('origin').get('xyz'), sep=' ')
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            local = origin + jaw[:, None] * axis + np.asarray(assets[link]['com'])
            p = positions + rotations.apply(local)
            q = rotations.as_quat()[:, [3, 0, 1, 2]]
            v = np.gradient(p, dt, axis=0)
            omega = (rotations[1:] * rotations[:-1].inv()).as_rotvec() / dt
            omega = np.vstack([omega, omega[-1]])
            name = side + '_' + link
            targets[name + '__pose'] = np.column_stack([p, q]); targets[name + '__velocity'] = np.column_stack([v, omega])
            bodies.append({'name': name, 'asset': link, 'p': p[0].tolist(), 'q': q[0].tolist(), 'driven': True})
    targets['time'] = times; np.savez_compressed(out / 'targets.npz', **targets)
    scene = {'kind': 'reduced_gripper_reproduction_pilot', 'task': 'stack_bowls', 'dt': dt,
             'table_top_z': .765, 'initial_gap_m': .0002, 'mu': .6, 'gravity': [0., 0., -9.81],
             'bodies': bodies, 'assets': assets, 'source_hz': hz, 'source_duration_s': (n - 1) / hz,
             'tail_s': settle, 'contact_capacity': 2048,
             'drive': {'Kt': 3000., 'Kr': 1., 'force_cap_N': 40., 'torque_cap_Nm': 1.},
             'completion': {'selected_model': selected_model, 'source': 'initial_fit.json; initial image only',
                            'identity': 'representative best silhouette fit; not a confirmed episode asset ID',
                            'silhouette_rms_px': completion['silhouette_rms_px'],
                            'alternative_models': [r['model'] for r in json.loads((root/'initial_fit.json').read_text())['candidates'][1:] if r['mean_rms_px'] < 1.5]},
             'deviations': ['independent spring driven CAD fingers; arms and palms omitted',
                            'new common CoACD geometry; native PhysX cooked hulls not released',
                            'single Coulomb mu=.6; source material static=.6 and dynamic=1.5 need runtime audit',
                            'inertia from shared additive convex-piece mass properties; not native cooked inertia',
                            'dt=1ms vs source native 4ms; 0.2mm initial separation in every engine',
                            'initial object poses fitted to first camera image; asset ID and uncertainty not confirmed'],
             'comparison_complete': False}
    (out / 'scene.json').write_text(json.dumps(scene, indent=2))
    hashes = {str(f.relative_to(out)): hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(out.rglob('*')) if f.is_file()}
    (out / 'provenance.json').write_text(json.dumps({'coacd_parameters': params, 'files': hashes}, indent=2))
    print(json.dumps({'bodies': len(bodies), 'steps': len(times)-1, 'pieces': {k:len(a['pieces']) for k,a in assets.items()}}), flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('root'); main(ap.parse_args().root)
