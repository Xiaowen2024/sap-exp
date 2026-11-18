"""Engine-independent target sampling, drive and trace format."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R


def load(root, drive_only=False):
    root = Path(root).resolve(); scene = json.loads((root / 'scene.json').read_text())
    if drive_only: scene['bodies'] = [b for b in scene['bodies'] if b['driven'] or (scene.get('articulated_grippers') and b.get('parent'))]
    for a in scene['assets'].values(): a['pieces'] = [str(root / p) for p in a['pieces']]
    with np.load(root / 'targets.npz',allow_pickle=False) as archive:
        targets={name:archive[name] for name in archive.files}
    return scene,targets


def rotation(q): return R.from_quat(np.asarray(q)[[1, 2, 3, 0]])


def wrench(scene, body, target, state, step):
    p, q, v, w = state
    desired = target[body['name'] + '__pose'][step]
    vel = target[body['name'] + '__velocity'][step]
    a, d = scene['assets'][body['asset']], scene['drive']
    f = d['Kt'] * (desired[:3] - p) + 2 * np.sqrt(a['mass'] * d['Kt']) * (vel[:3] - v)
    err = (rotation(desired[3:]) * rotation(q).inv()).as_rotvec()
    # Isotropic rotational damping uses the largest common body principal inertia.
    damping = 2 * np.sqrt(d['Kr'] * np.linalg.eigvalsh(a['inertia']).max())
    tau = d['Kr'] * err + damping * (vel[3:] - w)
    f *= min(1., d['force_cap_N'] / max(np.linalg.norm(f), 1e-12))
    tau *= min(1., d['torque_cap_Nm'] / max(np.linalg.norm(tau), 1e-12))
    return f - a['mass'] * np.asarray(scene['gravity']), tau


def save(out, scene, target, trace, health):
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out / 'trace.npz', **trace)
    health['comparison_complete'] = False
    health['kind'] = 'object_smoke_pilot' if any(not b['driven'] and not b.get('is_gripper') for b in scene['bodies']) else 'drive_preflight'
    health['tracking'] = {}
    steps = np.rint(trace['time'] / scene['dt']).astype(int)
    for b in scene['bodies']:
        if not b['driven']: continue
        poses = trace[b['name'] + '__pose']; desired = target[b['name'] + '__pose'][steps]
        errors = np.linalg.norm(poses[:, :3] - desired[:, :3], axis=1)
        ang = (R.from_quat(poses[:, [4, 5, 6, 3]]) * R.from_quat(desired[:, [4, 5, 6, 3]]).inv()).magnitude()
        health['tracking'][b['name']] = {'max_position_m': float(errors.max()), 'rms_position_m': float(np.sqrt(np.mean(errors**2))),
                                        'max_rotation_deg': float(np.rad2deg(ang).max())}
    health['drive_gate_pass'] = all(v['max_position_m'] <= .005 and v['max_rotation_deg'] <= 5 for v in health['tracking'].values())
    (out / 'health.json').write_text(json.dumps(health, indent=2))
    print(json.dumps({k:v for k,v in health.items() if k not in ('native','native_config','task_deviations')}, indent=2), flush=True)
