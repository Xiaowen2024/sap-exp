"""E2 measurement helpers and P1/P2 admission. Never uses simulation output."""
from __future__ import annotations

import numpy as np

from .io_utils import GateError
from .metrics import retained_predicate


def backproject(depth_m, intrinsics, camera_to_base, mask):
    depth_m, mask = np.asarray(depth_m, float), np.asarray(mask, bool)
    K, T = np.asarray(intrinsics, float), np.asarray(camera_to_base, float)
    if depth_m.shape != mask.shape or K.shape != (3, 3) or T.shape != (4, 4):
        raise GateError('depth/mask/calibration dimensions disagree')
    y, x = np.nonzero(mask & np.isfinite(depth_m) & (depth_m > 0))
    rays = np.linalg.solve(K, np.stack((x, y, np.ones(len(x)))))
    xyz = rays.T * depth_m[y, x, None]
    return xyz @ T[:3, :3].T + T[:3, 3]


def scaled_intrinsics(K, original_wh, image_wh):
    scale = np.diag([image_wh[0] / original_wh[0], image_wh[1] / original_wh[1], 1.])
    return scale @ np.asarray(K, float)


def plane_fit(points):
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 20:
        raise GateError('plane needs >=20 measured points')
    centre = points.mean(axis=0)
    _, _, vh = np.linalg.svd(points-centre, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal *= -1
    residual = (points-centre) @ normal
    return {'normal': normal.tolist(), 'offset_m': -float(normal@centre),
             'rms_m': float(np.sqrt(np.mean(residual**2))),
             'max_abs_residual_m': float(np.max(abs(residual))), 'points': len(points)}


def reprojection_residual(points_base, observed_uv, intrinsics, camera_to_base):
    points, uv = np.asarray(points_base, float), np.asarray(observed_uv, float)
    T = np.asarray(camera_to_base, float)
    cam = (points-T[:3, 3]) @ T[:3, :3]
    if np.any(cam[:, 2] <= 0) or len(points) < 3:
        raise GateError('need >=3 visible geometric calibration checks')
    image = cam @ np.asarray(intrinsics).T
    pred = image[:, :2] / image[:, 2, None]
    error = np.linalg.norm(pred-uv, axis=1)
    return {'rms_px': float(np.sqrt(np.mean(error**2))), 'max_px': float(error.max()),
             'per_point_px': error.tolist()}


REQUIRED_QUANTITIES = ('table_plane', 'plate_diameter_mm', 'plate_height_mm',
                       'plate_initial_pose', 'rack_geometry', 'rack_pose', 'rack_fixed',
                       'rim_thickness_mm', 'plate_mass_kg', 'pad_geometry',
                       'closing_force_N', 'plate_profile', 'mu_pad_plate',
                       'mu_plate_environment', 'plate_inertia')


def admission(episode):
    q = episode['completion']
    missing = set(REQUIRED_QUANTITIES) - set(q)
    if missing:
        raise GateError(f'missing completed quantities: {sorted(missing)}')
    for name, quantity in q.items():
        for key in ('value', 'range', 'class', 'source', 'method'):
            if key not in quantity or quantity[key] is None:
                raise GateError(f'{name}: missing {key}')
        if not quantity['source'] or not quantity['method']:
            raise GateError(f'{name}: source and method must be nonempty')
        if quantity['class'] not in ('E1', 'E2', 'E3', 'E4', 'E5', 'derived'):
            raise GateError(f'{name}: invalid evidence class')
        if quantity['class'] == 'E3':
            if not quantity.get('event') or 'unscored_stage' not in quantity:
                raise GateError(f'{name}: E3 event and stage cost must be explicit')
            stage = quantity['unscored_stage']
            if stage != 'none' and stage not in episode['unscored_stages']:
                raise GateError(f'{name}: E3 stage still scored')
            if stage == 'none' and name != 'plate_mass_kg':
                raise GateError(f'{name}: only the declared mass exception has no stage cost')
    for name in ('rack_geometry', 'rack_pose', 'rack_fixed'):
        if q[name]['class'] not in ('E1', 'E2'):
            raise GateError('goal fixture must be E1/E2, independent of release/outcome')
    if q['rack_fixed']['value'] is not True:
        raise GateError('moving rack unsupported by the v1 static-fixture adapter')
    checks = {}
    for stage in ('placed', 'retained'):
        axes = episode['admission_margins'][stage]
        if not axes:
            raise GateError(f'{stage}: no geometric margins')
        checks[stage] = []
        for axis in axes:
            margin = float(axis['margin_m'])
            uncertainties = np.asarray(axis['uncertainty_half_ranges_m'], float)
            if not len(uncertainties) or np.any(uncertainties < 0) or not np.isfinite(uncertainties).all() or not np.isfinite(margin):
                raise GateError('invalid margin/uncertainty inputs')
            u = float(np.linalg.norm(uncertainties))
            checks[stage].append({'axis': axis['axis'], 'm_m': margin, 'u_m': u,
                                  'pass': margin >= 3*u and margin > 0})
    reality = retained_predicate(episode['real_final_state'], episode['scene']['goal'])
    matches = reality == (episode['real_label'] == 'retained')
    return {'episode_id': episode['id'], 'margins': checks,
             'predicate_on_reality': 'retained' if reality else 'not_retained',
             'real_label': episode['real_label'], 'predicate_matches': matches,
             'admitted': matches and all(a['pass'] for axes in checks.values() for a in axes)}
