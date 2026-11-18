"""Common convex geometry overlap observer, independent of contact laws.

Native contact activity is supplied separately by the backend. SAT measures
overlap on the *current* shared meshes. Swept disjoint endpoints are treated
conservatively as possible tunnelling and cannot pass the health gate.
"""
from __future__ import annotations

import itertools
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


def load_obj(path):
    vertices, faces = [], []
    with open(path) as stream:
        for line in stream:
            if line.startswith('v '):
                vertices.append([float(v) for v in line.split()[1:4]])
            elif line.startswith('f '):
                faces.append([int(v.split('/')[0])-1 for v in line.split()[1:4]])
    return np.asarray(vertices), np.asarray(faces)


def transform(vertices, pose):
    return Rotation.from_quat(pose[3:]).apply(vertices) + pose[:3]


def axes(vertices, faces):
    edges = sorted({tuple(sorted(e)) for f in faces for e in itertools.combinations(f, 2)})
    edge_v = np.array([vertices[b]-vertices[a] for a,b in edges])
    tri = vertices[faces]
    normals = np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    return normals, edge_v


def sat_overlap(a, af, b, bf):
    if np.any(a.max(0) < b.min(0)) or np.any(b.max(0) < a.min(0)):
        return 0.0
    an, ae = axes(a, af)
    bn, be = axes(b, bf)
    candidates = np.vstack((an, bn, np.cross(ae[:,None,:],be[None,:,:]).reshape(-1,3)))
    norm = np.linalg.norm(candidates, axis=1)
    candidates = candidates[norm > 1e-12]/norm[norm > 1e-12,None]
    ap, bp = a@candidates.T, b@candidates.T
    distance = np.minimum(ap.max(0)-bp.min(0), bp.max(0)-ap.min(0))
    return max(0., float(distance.min()))


def geometric_contact(a, af, b, bf, tolerance):
    if np.any(a.max(0)+tolerance < b.min(0)) or np.any(b.max(0)+tolerance < a.min(0)):
        return False
    an, ae = axes(a, af)
    bn, be = axes(b, bf)
    candidates = np.vstack((an,bn,np.cross(ae[:,None,:],be[None,:,:]).reshape(-1,3)))
    norm = np.linalg.norm(candidates,axis=1)
    candidates = candidates[norm>1e-12]/norm[norm>1e-12,None]
    ap,bp = a@candidates.T,b@candidates.T
    gaps = np.maximum(ap.min(0)-bp.max(0),bp.min(0)-ap.max(0))
    return bool(np.max(gaps)<=tolerance)


class Observer:
    def __init__(self, scene, assets):
        self.scene = scene
        self.plate = [load_obj(p['path']) for p in assets['plate']['pieces']]
        self.pad = load_obj(assets['pad']['path'])
        self.rack = []
        for p in assets['rack']:
            v, f = load_obj(p['path'])
            v = transform(v, np.r_[p['position_m'],p['quaternion_xyzw']])
            v = transform(v, np.r_[scene['rack']['position_m'],scene['rack']['quaternion_xyzw']])
            self.rack.append((v,f))
        self.previous = None

    def sample(self, snapshot):
        poses = snapshot['poses']
        def actual(v,name):
            if 'affine_transforms' in snapshot:
                T=snapshot['affine_transforms'][name]
                return v@T[:3,:3].T+T[:3,3]
            return transform(v,poses[name])
        plate = [(actual(v,'plate'),f) for v,f in self.plate]
        pads = [(actual(self.pad[0],n),self.pad[1]) for n in ('left','right')]
        obstacles = pads+self.rack
        pad_overlap = rack_overlap = 0.
        contacts = np.zeros(len(obstacles),bool)
        tunnelling = False
        for i,(v,f) in enumerate(plate):
            for j,(ov,of) in enumerate(obstacles):
                overlap = sat_overlap(v,f,ov,of)
                contacts[j] |= geometric_contact(v,f,ov,of,self.scene['contact_tolerance_m'])
                if j < 2:
                    pad_overlap = max(pad_overlap, overlap)
                else:
                    rack_overlap = max(rack_overlap, overlap)
                if self.previous is not None and overlap == 0:
                    prevp, prevo = self.previous
                    oldv = prevp[i][0]
                    oldov = prevo[j][0]
                    if sat_overlap(oldv,f,oldov,of) == 0:
                        # Express the previous plate in the current obstacle's
                        # translated frame. Rapid rotation requires the stricter
                        # native preflight; it is never silently certified here.
                        relative_old = oldv + (ov.mean(0)-oldov.mean(0))
                        swept = np.vstack((relative_old,v))
                        if np.all(swept.max(0) >= ov.min(0)) and np.all(ov.max(0) >= swept.min(0)):
                            hull = ConvexHull(swept)
                            tunnelling |= sat_overlap(swept,hull.simplices,ov,of) > 1e-10
        self.previous = (plate,obstacles)
        lowest = min(v[:,2].min() for v,f in plate)
        return {'plate_lowest_z_m': float(lowest), 'tunnelling': bool(tunnelling),
                'pad_contacts': contacts[:2].tolist(), 'rack_contact': bool(contacts[2:].any()),
                'table_contact': bool(lowest-self.scene['table_z_m']<=self.scene['contact_tolerance_m']),
                'overlap_m': {'pad_plate': pad_overlap, 'plate_rack': rack_overlap,
                              'plate_table': max(0.,self.scene['table_z_m']-float(lowest))}}
