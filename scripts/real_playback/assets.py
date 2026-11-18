"""Deterministic convex-wedge plate assets shared by all backends.

Every cross section is declared in the completion; no dimensions or wall
profile are silently filled in. OBJ vertices are SI in each body local frame.
"""
from __future__ import annotations

from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull

from .io_utils import GateError, digest, sha256, write_json


def convex_mesh(vertices):
    vertices = np.unique(np.asarray(vertices, float), axis=0)
    hull = ConvexHull(vertices)
    faces = hull.simplices.copy()
    for i, f in enumerate(faces):
        normal = np.cross(vertices[f[1]] - vertices[f[0]], vertices[f[2]] - vertices[f[0]])
        if normal @ hull.equations[i, :3] < 0:
            faces[i, [1, 2]] = faces[i, [2, 1]]
    return vertices, faces


def mass_properties(vertices, faces):
    """Exact volume, centroid and central inertia for an oriented surface."""
    v = np.asarray(vertices)[np.asarray(faces)]
    a, b, c = v[:, 0], v[:, 1], v[:, 2]
    volumes = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6
    volume = float(volumes.sum())
    if volume <= 0:
        raise GateError('asset has nonpositive signed volume')
    centroid = (volumes[:, None] * (a + b + c) / 4).sum(axis=0) / volume
    # Integral xx^T over tetrahedra with the fourth vertex at the origin.
    moment = np.zeros((3, 3))
    for vi, ai, bi, ci in zip(volumes, a, b, c):
        u = ai + bi + ci
        moment += vi / 20 * (np.outer(u, u) + np.outer(ai, ai) + np.outer(bi, bi) + np.outer(ci, ci))
    central = moment - volume * np.outer(centroid, centroid)
    inertia_density1 = np.trace(central) * np.eye(3) - central
    return volume, centroid, inertia_density1


def save_obj(path, vertices, faces):
    lines = ['# real-playback SI convex asset']
    lines += ['v ' + ' '.join(format(float(x), '.17g') for x in p) for p in vertices]
    lines += ['f ' + ' '.join(str(int(x) + 1) for x in f) for f in faces]
    Path(path).write_text('\n'.join(lines) + '\n')


def plate_pieces(profile, sectors):
    profile = np.asarray(profile, float)
    if profile.ndim != 2 or profile.shape[1] != 3 or len(profile) < 2:
        raise GateError('profile must be [radius_m, lower_z_m, upper_z_m] rows')
    if profile[0, 0] != 0 or np.any(np.diff(profile[:, 0]) <= 0) or np.any(profile[:, 2] <= profile[:, 1]):
        raise GateError('profile must be a solid, increasing-radius shell starting at r=0')
    if type(sectors) is not int or sectors < 12:
        raise GateError('at least 12 declared angular sectors required')
    result = []
    for j in range(len(profile) - 1):
        for i in range(sectors):
            angles = np.array([i, i + 1]) * 2 * np.pi / sectors
            points = []
            for r, low, high in profile[j:j+2]:
                for theta in angles:
                    for z in (low, high):
                        points.append([r*np.cos(theta), r*np.sin(theta), z])
            result.append(convex_mesh(points))
    return result


def build_plate(scene, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    cfg = scene['plate']
    params = {'profiles': cfg['profiles_radius_lower_upper_m'], 'sectors': cfg['sectors']}
    results = {}
    for name, profile in params['profiles'].items():
        folder = output / name
        folder.mkdir(exist_ok=True)
        chunks = plate_pieces(profile, params['sectors'])
        properties = [mass_properties(v, f) for v, f in chunks]
        volume = sum(p[0] for p in properties)
        centre = sum(p[0] * p[1] for p in properties) / volume
        inertia = sum(p[2] + p[0] * (np.dot(p[1]-centre, p[1]-centre)*np.eye(3) -
                                    np.outer(p[1]-centre, p[1]-centre)) for p in properties)
        files = []
        for i, (v, f) in enumerate(chunks):
            path = folder / f'wedge_{i:04d}.obj'
            # Recenter all shared files at the physical COM.
            save_obj(path, v-centre, f)
            files.append({'path': str(path.relative_to(output)), 'sha256': sha256(path)})
        results[name] = {'pieces': files, 'volume_m3': volume,
                         'profile_origin_to_com_m': centre.tolist(),
                         'inertia_per_kg_m2': (inertia/volume).tolist()}
    index = {'builder': 'scripts.real_playback.assets/build_plate-v1',
             'parameters_sha256': digest(params), 'parameters': params,
             'profiles': results}
    write_json(output/'assets.json', index)
    return index
