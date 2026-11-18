"""Check initial camera correspondence and cavity preservation, before outcomes."""
import argparse
import json
from pathlib import Path
import numpy as np
import trimesh
from PIL import Image, ImageDraw
from scipy.spatial import ConvexHull


def audit(root):
    root = Path(root); out = root / 'prepared'; ref = trimesh.load(out / 'bowl_reference.obj', process=False)
    scene = json.loads((out / 'scene.json').read_text()); asset = scene['assets']['bowl']; com = np.asarray(asset['com'])
    pieces = [trimesh.load(out / p, process=False) for p in asset['pieces']]
    equations = [ConvexHull(p.vertices + com).equations for p in pieces]
    def contains(points):
        inside = np.zeros(len(points), bool)
        for e in equations: inside |= (points @ e[:, :3].T + e[:, 3] <= 1e-9).all(1)
        return inside
    bounds = ref.bounds
    x, z = np.linspace(bounds[0,0]-.003,bounds[1,0]+.003,257), np.linspace(bounds[0,2]-.003,bounds[1,2]+.003,115)
    xx,zz = np.meshgrid(x,z); points = np.column_stack([xx.ravel(),np.zeros(xx.size),zz.ravel()])
    original = np.concatenate([ref.contains(part) for part in np.array_split(points, max(1,len(points)//512))]); collision = contains(points)
    colors = np.full((len(z),len(x),3),245,np.uint8)
    colors.reshape(-1,3)[original & collision] = [35,65,90]
    colors.reshape(-1,3)[collision & ~original] = [220,80,45]
    colors.reshape(-1,3)[original & ~collision] = [40,165,190]
    Image.fromarray(colors[::-1]).resize((1010,458)).save(out / 'bowl_cross_section.png')
    # Source-surface validation is geometric only; no task outcomes are consulted.
    probe = np.vstack([p.vertices + com for p in pieces])
    _, distances, _ = trimesh.proximity.closest_point(ref, probe)
    outside = ~ref.contains(probe)
    center = np.column_stack([np.zeros(501),np.zeros(501),np.linspace(bounds[0,2],bounds[1,2],501)])
    center_occupied = center[contains(center),2]
    report = {'cross_section_legend': {'navy':'common material','orange':'collision overfill','cyan':'source material omitted'},
              'common_pieces': len(pieces), 'max_hull_vertex_to_simplified_source_m': float(distances.max()),
              'max_outside_hull_vertex_distance_m': float(distances[outside].max()) if outside.any() else 0.,
              'center_occupied_z_range_m': [float(center_occupied.min()),float(center_occupied.max())],
              'central_cavity_clear_above_z_m': float(bounds[0,2] + .35*np.ptp(bounds[:,2])),
              'central_cavity_gate': bool(not contains(center[center[:,2] > bounds[0,2]+.35*np.ptp(bounds[:,2])]).any()),
              'caveat': 'not a symmetric Hausdorff bound; source is the 8000-face simplification; native PhysX hulls unavailable'}
    data = np.load(root / 'stack_bowls/numeric.npz', allow_pickle=False)
    K = data['vision__cam_head__intrinsic_matrix']; T = data['vision__cam_head__extrinsic_matrix'][0]
    def project(v):
        cam = (v - T[:3,3]) @ T[:3,:3]; cam *= [1,-1,-1]
        uv = cam @ K.T; return uv[:,:2] / uv[:,2:3]
    image = Image.open(root / 'stack_bowls/cam_head_0000.jpg').convert('RGB'); draw = ImageDraw.Draw(image)
    centers = {}
    for b in scene['bodies']:
        if b['driven']:continue
        # Mesh is COM-centred; all three official initial orientations are identity.
        v = np.asarray(ref.vertices) - com + np.asarray(b['p'])
        uv = project(v); hull = ConvexHull(uv); poly = [tuple(p) for p in uv[hull.vertices]]
        draw.line(poly+[poly[0]],fill=(255,80,30),width=2)
        centers[b['name']] = project(np.asarray(b['p'])[None,:]-com)[0].tolist()
    image.save(out / 'initial_layout_overlay.jpg')
    report.update(initial_projected_body_origins_px=centers,
                  initial_silhouette_rms_px=scene['completion']['silhouette_rms_px'],
                  correspondence='initial-image fit; released eval layout0 was rejected; asset ID remains inferred')
    (out / 'geometry_audit.json').write_text(json.dumps(report,indent=2)); print(json.dumps(report,indent=2),flush=True)


if __name__ == '__main__':
    ap=argparse.ArgumentParser();ap.add_argument('root');audit(ap.parse_args().root)
