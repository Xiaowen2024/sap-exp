"""Build the shared Insert-Tubes playback scene (RoboDojo insert_tubes, arx_x5 episode 0).

Initial object poses are not released with the demo. They are reconstructed
from the recorded gripper trajectory (grasp geometry, release positions vs the
rack hole grid) and checked against the first head-camera frame (cap pixels);
see tubes_fit.py / scene_fit.json. Geometry is shared by every engine:
  * tube  = official collision mesh split into two exact convex pieces
            (tip+body r=14 mm, cap r=17.5 mm); one convex hull would bulge
            past the 15 mm hole radius and is therefore not used.
  * rack  = static; top plate built from convex sectors whose inner chords are
            tangent to the official r=15 mm holes (holes never shrink), four
            posts, solid floor.  Rack is a Geometry (fixed) object officially.
  * X5 gripper = Codex's articulated model reused unchanged (official finger
            hulls, slide joints, PD 2300/100, 100 N cap; free bases servoed).
"""
import argparse, json, hashlib, shutil
from pathlib import Path
import numpy as np, trimesh
from scipy.spatial.transform import Rotation as R, Slerp

TABLE = .765


def tube_pieces(mesh_path, out):
    t = trimesh.load(mesh_path, force='mesh', process=True)
    v = t.vertices.copy(); v[:, 1] -= v[:, 1].min()          # origin at tip, axis +y (official frame)
    body = trimesh.convex.convex_hull(v[v[:, 1] <= .1046]); cap = trimesh.convex.convex_hull(v[v[:, 1] >= .1040])
    agg = trimesh.util.concatenate([body, cap]); props = trimesh.convex.convex_hull(v).mass_properties
    # mass properties from the solid envelope of the two pieces (union volume approx = hull of each, overlap 1 mm)
    com = (body.mass_properties.center_mass * body.volume + cap.mass_properties.center_mass * cap.volume) / (body.volume + cap.volume)
    I = body.mass_properties.inertia + cap.mass_properties.inertia
    def shift(Ic, m, d): return Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    Ic = np.zeros((3, 3)); vol = body.volume + cap.volume
    for p in (body, cap):
        mp = p.mass_properties; Ic += shift(mp.inertia, mp.volume, mp.center_mass - com)   # density 1
    mass = .06; Ic *= mass / vol
    paths = []
    for i, p in enumerate((body, cap)):
        q = p.copy(); q.vertices -= com; f = out / f'tube/piece_{i}.obj'; f.parent.mkdir(parents=True, exist_ok=True); q.export(f); paths.append(str(f.relative_to(out)))
    radii = [float(np.hypot(p.vertices[:, 0], p.vertices[:, 2]).max()) for p in (body, cap)]
    return {'pieces': paths, 'mass': mass, 'com': com.tolist(), 'inertia': Ic.tolist(), 'piece_max_radius_m': radii,
            'origin': 'tube tip (official object frame, axis +y)'}


def prism(poly2d, z0, z1):
    poly2d = np.asarray(poly2d)
    v = np.vstack([np.c_[poly2d, np.full(len(poly2d), z0)], np.c_[poly2d, np.full(len(poly2d), z1)]])
    return trimesh.convex.convex_hull(v)


def rack_pieces(meta, out, sectors=16):
    sup = meta['passive']['support']; holes = [np.array(v['center'][0][:2]) for v in sup.values()]; r = sup['test_tube_slot/0']['radius'][0]
    pieces = []; z0, z1 = .026, .030
    rv = r / np.cos(np.pi / sectors)            # chord tangent to the true hole circle
    for h in holes:
        cell = np.array([[h[0] - .018, h[1] - .018], [h[0] + .018, h[1] + .018]])
        for k in range(sectors):
            a0, a1 = 2 * np.pi * k / sectors, 2 * np.pi * (k + 1) / sectors
            inner = [h + rv * np.array([np.cos(a), np.sin(a)]) for a in (a0, a1)]
            # clip wedge to the cell square: sample the boundary of the square along the wedge
            pts = list(inner)
            for a in np.linspace(a0, a1, 9):
                d = np.array([np.cos(a), np.sin(a)]); t = min([((cell[1 if d[i] > 0 else 0][i] - h[i]) / d[i]) for i in range(2) if abs(d[i]) > 1e-12])
                pts.append(h + t * d)
            pieces.append(prism(np.asarray(pts), z0, z1))
    # border strips of the top plate (outside the 180 x 72 mm hole grid), rack is 205 x 96 mm
    for (x0, x1, y0, y1) in [(-.1025, -.090, -.048, .048), (.090, .1025, -.048, .048), (-.090, .090, -.048, -.036), (-.090, .090, .036, .048)]:
        pieces.append(prism([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], z0, z1))
    for sx in (-1, 1):                       # four posts (mesh: |x| .068-.076, |y| .037-.044)
        for sy in (-1, 1):
            c = np.array([sx * .072, sy * .0405]); pieces.append(prism([c + [-.004, -.0035], c + [.004, -.0035], c + [.004, .0035], c + [-.004, .0035]], -.024, .026))
    pieces.append(prism([[-.1025, -.048], [.1025, -.048], [.1025, .048], [-.1025, .048]], -.030, -.024))   # floor, top at -24 mm
    paths = []
    for i, p in enumerate(pieces):
        f = out / f'rack/piece_{i:03d}.obj'; f.parent.mkdir(parents=True, exist_ok=True); p.export(f); paths.append(str(f.relative_to(out)))
    return {'pieces': paths, 'static': True, 'hole_radius_m': r, 'sector_count': sectors, 'top_plate_z_m': [z0, z1], 'floor_top_z_m': -.024,
            'max_hole_enlargement_m': float(rv - r)}


def main(root, x5, out):
    root, x5, out = Path(root), Path(x5), Path(out); out.mkdir(parents=True, exist_ok=True)
    fit = json.loads((root / 'scene_fit.json').read_text()); data = np.load(root / 'insert_tubes/numeric.npz', allow_pickle=False)
    meta_rack = json.loads((root / 'dl/Geometry_test_tube_slot_00000/metadata.json').read_text())
    meta_tube = json.loads((root / 'dl/Rigid_test_tube_00002/metadata.json').read_text())
    x5scene = json.loads((x5 / 'scene.json').read_text())
    assets = {'tube': tube_pieces(root / 'dl/tube.obj', out), 'rack': rack_pieces(meta_rack, out)}
    for k in ('link7', 'link8'):
        shutil.copytree(x5 / k, out / k, dirs_exist_ok=True); assets[k] = x5scene['assets'][k]
    shutil.copy(x5 / 'base.obj', out / 'base.obj'); assets['base'] = x5scene['assets']['base']
    bodies = []
    rk = fit['rack']; static = [{'name': 'rack', 'asset': 'rack', 'p': rk['pos'], 'q': rk['quat_wxyz']}]
    com = np.asarray(assets['tube']['com'])
    for name, t in fit['tubes'].items():
        rot = R.from_quat(np.asarray(t['quat_wxyz'])[[1, 2, 3, 0]]); p = np.asarray(t['origin_tip']) + rot.apply(com); p[2] += .0002
        bodies.append({'name': name, 'asset': 'tube', 'p': p.tolist(), 'q': t['quat_wxyz'], 'driven': False, 'origin_offset': com.tolist()})
    hz = float(data['additional_info__frequency']); n = len(data['state__right_ee_poses']); dt = .001; tail = 2.0
    steps = int(round(((n - 1) / hz + tail) / dt)); steps += (-steps) % 10
    times = np.arange(steps + 1) * dt; src = np.arange(n) / hz; cl = np.minimum(times, src[-1]); targets = {'time': times}
    for side in ('left', 'right'):
        P = data[f'state__{side}_ee_poses']; pos = np.column_stack([np.interp(cl, src, P[:, i]) for i in range(3)])
        rot = Slerp(src, R.from_quat(P[:, [4, 5, 6, 3]]))(cl); q = rot.as_quat()[:, [3, 0, 1, 2]]
        v = np.gradient(pos, dt, axis=0); w = (rot[1:] * rot[:-1].inv()).as_rotvec() / dt; w = np.vstack([w, w[-1]])
        name = side + '_base'; targets[name + '__pose'] = np.c_[pos, q]; targets[name + '__velocity'] = np.c_[v, w]
        cmd = -.01 + .054 * np.interp(cl, src, data[f'action__{side}_ee_joint_states'][:, 0]); targets['command_' + side] = cmd
        bodies.append({'name': name, 'asset': 'base', 'p': pos[0].tolist(), 'q': q[0].tolist(), 'driven': True, 'is_gripper': True})
        for b in x5scene['bodies']:
            if b.get('parent') == name:
                c = dict(b); init = float(np.clip(cmd[0], 0., .044)); c['joint_initial'] = init
                local = np.asarray(c['zero_p']) + np.asarray(c['axis']) * init; c['p'] = (pos[0] + rot[0].apply(local)).tolist(); c['q'] = q[0].tolist(); bodies.append(c)
    mu = .225
    scene = {'kind': 'reduced_gripper_reproduction_pilot', 'task': 'insert_tubes', 'episode': 0, 'dt': dt, 'table_top_z': TABLE, 'initial_gap_m': .0002,
             'mu': mu, 'gravity': [0., 0., -9.81], 'bodies': bodies, 'static_bodies': static, 'assets': assets, 'source_hz': hz,
             'source_duration_s': (n - 1) / hz, 'tail_s': steps * dt - (n - 1) / hz, 'contact_capacity': 16384, 'articulated_grippers': True,
             'drive': x5scene['drive'], 'finger_actuator': x5scene['finger_actuator'], 'source_control_audit': x5scene['source_control_audit'],
             'success': {'source': 'task/RoboDojo/tasks/insert_tubes.py + env/reward_manager/func_parser.py (fbfbbcb)',
                         'depth_z_threshold_m': .045, 'axis_up_threshold_deg': 30., 'tube_oriented_bbox': meta_tube['geometry']['oriented_bbox']['vertices'],
                         'rack_oriented_bbox': meta_rack['geometry']['oriented_bbox']['vertices']},
             'reconstruction': fit,
             'deviations': ['detached free gripper bases replace robot arms; same base servo in all engines (as stack_bowls r3)',
                            'object poses reconstructed from gripper trajectory + first head frame; not released by the dataset',
                            'single Coulomb mu=0.225 for all contacts = official tube(0.45)/rack(0.0) PhysX average; fingers/table use the same value',
                            'rack = convex sector approximation of the official SDF mesh (holes enlarged <= %.2f mm at sector vertices)' % (assets['rack']['max_hole_enlargement_m'] * 1e3),
                            'tube = two exact convex pieces of the official collision mesh (official: 64-hull PhysX decomposition)',
                            'dt=1 ms; 0.2 mm initial lift of tubes; %.2f s settle tail after the demo' % (steps * dt - (n - 1) / hz)]}
    (out / 'scene.json').write_text(json.dumps(scene, indent=1)); np.savez_compressed(out / 'targets.npz', **targets)
    hashes = {str(f.relative_to(out)): hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(out.rglob('*')) if f.is_file()}
    (out / 'provenance.json').write_text(json.dumps(hashes, indent=1))
    print('bodies', len(bodies), 'steps', steps, 'rack pieces', len(assets['rack']['pieces']), 'tube radii', assets['tube']['piece_max_radius_m'])


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('root'); ap.add_argument('x5'); ap.add_argument('out'); a = ap.parse_args(); main(a.root, a.x5, a.out)
