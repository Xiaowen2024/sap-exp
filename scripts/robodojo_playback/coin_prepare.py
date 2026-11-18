"""Build the shared Deposit-Coin playback scene (RoboDojo deposit_coin, arx_x5 episode 0).

Object poses are reconstructed from the gripper trajectory (fit_coin.py ->
scene_fit.json). Every convex piece has <= 64 vertices (SAP silently truncates
larger hulls), built by sampling / CoACD decimation, never by truncation.
  coin  : 30-gon prism, r 14.25 mm, 1.94 mm thick, 5 g (official collision disc)
  stand : static; CoACD(decimate, <=64 verts) of the official collision mesh + 2 official stop cubes
  bank  : static; zero-thickness official shell thickened 2 mm inward, patch-wise convex
          pieces; the 5.5 x 36.9 mm slot channel kept clear (checked)
  X5 gripper: Codex's articulated model, unchanged.
"""
import argparse, json, hashlib, shutil, glob
from pathlib import Path
import numpy as np, trimesh
from scipy.spatial.transform import Rotation as R, Slerp

TABLE = .765


def coin_piece(out, r=.01425, h=.00194, n=30):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False); ring = np.c_[r * np.cos(a), r * np.sin(a)]
    v = np.vstack([np.c_[ring, np.zeros(n)], np.c_[ring, np.full(n, h)]]); m = trimesh.convex.convex_hull(v)
    com = m.center_mass; mass = .005; I = m.moment_inertia * mass / m.mass
    q = m.copy(); q.vertices -= com; f = out / 'coin/piece_0.obj'; f.parent.mkdir(parents=True, exist_ok=True); q.export(f)
    return {'pieces': ['coin/piece_0.obj'], 'mass': mass, 'com': com.tolist(), 'inertia': I.tolist(), 'origin': 'bottom face centre (official frame)', 'n_vertices': len(m.vertices)}


def static_pieces(files, out, name):
    paths = []
    (out / name).mkdir(parents=True, exist_ok=True)
    for i, f in enumerate(files):
        m = trimesh.load(f, force='mesh'); assert len(m.vertices) <= 64, (f, len(m.vertices))
        p = out / name / f'piece_{i:03d}.obj'; m.export(p); paths.append(str(p.relative_to(out)))
    return {'pieces': paths, 'static': True}


def stand_exact(root):
    """Two tapered walls + base plate, measured on the official collision mesh by ray probes
    (slot inner faces y=+-1.10 mm at z=-6.2 mm -> +-1.52 mm at 7.8 mm; outer faces 4.2 -> 2.3 mm;
    wall ends x=+-12.9 mm at the base -> +-11.2 mm at the top z=8.5 mm; base 32.2 x 11 x 1.2 mm).
    Exact convex pieces (8 vertices each); CoACD's decimated pieces narrowed the slot below the coin thickness."""
    zb, zt = -.0074, .0085
    def wall(sign, yin_b, yin_t, yout_b, yout_t):
        v = [[x, sign * y, zb] for x in (-.0129, .0129) for y in (yin_b, yout_b)] + [[x, sign * y, zt] for x in (-.0112, .0112) for y in (yin_t, yout_t)]
        return trimesh.convex.convex_hull(np.asarray(v))
    pieces = [wall(1, .001064, .001541, .00439, .00223), wall(-1, .001056, .001518, .00439, .00223),
              trimesh.creation.box(extents=[.0322, .011, .0012], transform=trimesh.transformations.translation_matrix([0, 0, -.008]))]
    files = []
    d = root / 'geom/stand_exact'; d.mkdir(parents=True, exist_ok=True)
    for i, m in enumerate(pieces): f = d / f'piece_{i}.obj'; m.export(f); files.append(str(f))
    return files


def main(root, x5, out):
    root, x5, out = Path(root), Path(x5), Path(out); out.mkdir(parents=True, exist_ok=True)
    fit = json.loads((root / 'scene_fit.json').read_text()); data = np.load(root / 'episode/numeric.npz', allow_pickle=False)
    x5scene = json.loads((x5 / 'scene.json').read_text())
    assets = {'coin': coin_piece(out),
              'stand': static_pieces(stand_exact(root) + [str(root / 'geom/stand_cube_0.obj'), str(root / 'geom/stand_cube_1.obj')], out, 'stand'),
              'bank': static_pieces(sorted(glob.glob(str(root / 'geom/bank_shell/*.obj'))), out, 'bank')}
    for k in ('link7', 'link8'):
        shutil.copytree(x5 / k, out / k, dirs_exist_ok=True); assets[k] = x5scene['assets'][k]
    shutil.copy(x5 / 'base.obj', out / 'base.obj'); assets['base'] = x5scene['assets']['base']
    static = [{'name': 'stand', 'asset': 'stand', 'p': fit['stand']['pos'], 'q': fit['stand']['quat_wxyz']},
              {'name': 'bank', 'asset': 'bank', 'p': fit['bank']['pos'], 'q': fit['bank']['quat_wxyz']}]
    com = np.asarray(assets['coin']['com']); rc = R.from_quat(np.asarray(fit['coin']['quat_wxyz'])[[1, 2, 3, 0]])
    p = np.asarray(fit['coin']['origin']) + rc.apply(com); p[2] += .0002
    bodies = [{'name': 'coin0', 'asset': 'coin', 'p': p.tolist(), 'q': fit['coin']['quat_wxyz'], 'driven': False, 'origin_offset': com.tolist()}]
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
    bank_meta = json.loads((root / 'dl/Geometry_piggy_bank_00000/metadata.json').read_text()); coin_meta = json.loads((root / 'dl/Rigid_coin_00000/metadata.json').read_text())
    scene = {'kind': 'reduced_gripper_reproduction_pilot', 'task': 'deposit_coin', 'episode': 0, 'dt': dt, 'table_top_z': TABLE, 'initial_gap_m': .0002,
             'mu': .3, 'gravity': [0., 0., -9.81], 'bodies': bodies, 'static_bodies': static, 'assets': assets, 'source_hz': hz,
             'source_duration_s': (n - 1) / hz, 'tail_s': steps * dt - (n - 1) / hz, 'contact_capacity': 16384, 'articulated_grippers': True,
             'drive': x5scene['drive'], 'finger_actuator': x5scene['finger_actuator'], 'source_control_audit': x5scene['source_control_audit'],
             'success': {'source': 'task/RoboDojo/tasks/deposit_coin.py + func_parser.is_A_bbox_in_B_bbox (fbfbbcb)',
                         'coin_oriented_bbox': coin_meta['geometry']['oriented_bbox']['vertices'], 'bank_oriented_bbox': bank_meta['geometry']['oriented_bbox']['vertices'],
                         'bank_bottom_point': bank_meta['passive']['functional']['bottom']['frame'][0][:3], 'bank_center_point': bank_meta['passive']['functional']['center']['frame'][0][:3],
                         'lift_threshold_m': .08},
             'reconstruction': fit,
             'deviations': ['detached free gripper bases replace robot arms; same base servo in all engines (as stack_bowls r3 / insert_tubes)',
                            'object poses reconstructed from gripper trajectory + first head frame; not released by the dataset',
                            'single Coulomb mu=0.3 = PhysX average of official coin default (static 0.6) and Geometry default (0.0)',
                            'bank = zero-thickness official shell thickened 2 mm inward, convex patches (<=64 verts), slot channel kept clear',
                            'stand = exact convex walls + base measured on the official mesh (slot 2.2-3.0 mm) + official stop cubes; coin = 30-gon prism (<=64 verts)',
                            'coin centred in the stand slot (official origin is the coin face); dt=1 ms; 0.2 mm initial lift; %.2f s tail' % (steps * dt - (n - 1) / hz)]}
    (out / 'scene.json').write_text(json.dumps(scene, indent=1)); np.savez_compressed(out / 'targets.npz', **targets)
    hashes = {str(f.relative_to(out)): hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(out.rglob('*')) if f.is_file()}
    (out / 'provenance.json').write_text(json.dumps(hashes, indent=1))
    print('bodies', len(bodies), 'steps', steps, 'pieces', {k: len(a['pieces']) for k, a in assets.items()})


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('root'); ap.add_argument('x5'); ap.add_argument('out'); a = ap.parse_args(); main(a.root, a.x5, a.out)
