"""Thin-wall convex decomposition of a closed zero-thickness shell with a slot.
Each piece = convex hull of a surface patch and the same patch offset inward by T.
<=64 hull vertices (farthest-point sampling, never truncation); no piece may enter the slot channel."""
import numpy as np, trimesh, os, sys
from scipy.cluster.vq import kmeans2
from scipy.spatial import Delaunay
T = .002; MAXP = 30
b = trimesh.load('geom/bank.obj', process=True); b.merge_vertices()
cen = b.vertices.mean(0)
vn = b.vertex_normals.copy()
# make normals outward (winding is consistent; flip globally if most point inward)
if np.mean(np.einsum('ij,ij->i', b.face_normals, b.triangles_center - cen)) < 0: vn = -vn
# slot keep-out channel (bank frame): slot loop extent x +-2.75 mm, y -22.5..14.4 mm, top z ~0.109
ko = np.array([[x, y, z] for x in np.linspace(-.0022, .0022, 7) for y in np.linspace(-.021, .013, 35) for z in np.r_[np.linspace(.060, .100, 9), np.linspace(.1005, .116, 32)]])
def fps(P, k):
    if len(P) <= k: return P
    idx = [0]; d = np.linalg.norm(P - P[0], axis=1)
    for _ in range(k - 1):
        i = int(d.argmax()); idx.append(i); d = np.minimum(d, np.linalg.norm(P - P[i], axis=1))
    return P[idx]
def piece(faces):
    vid = np.unique(b.faces[faces]); P = b.vertices[vid]; N = vn[vid]
    keep = fps(np.arange(len(P))[:, None].astype(float), 1) if False else None
    sel = fps(np.c_[P, np.arange(len(P))], MAXP)[:, 3].astype(int) if len(P) > MAXP else np.arange(len(P))
    pts = np.vstack([P[sel], P[sel] - T * N[sel]])
    try: h = trimesh.convex.convex_hull(pts)
    except Exception: return None
    return h
def bad(h):
    if h is None: return True
    if len(h.vertices) > 64: return True
    try: inside = Delaunay(h.vertices).find_simplex(ko) >= 0
    except Exception: return True
    return bool(inside.any())
fc = b.triangles_center; out = []
_, lab = kmeans2(fc, 420, seed=20261118, minit='++', iter=30)
stack = [np.nonzero(lab == k)[0] for k in range(lab.max() + 1)]
stack = [s for s in stack if len(s)]
while stack:
    f = stack.pop(); h = piece(f)
    if not bad(h): out.append(h); continue
    if len(f) <= 2:
        if h is not None and len(h.vertices) <= 64: out.append(h)  # tiny piece; record even if touching (should not happen)
        continue
    _, l2 = kmeans2(fc[f], 2, seed=1, minit='++', iter=20)
    if l2.min() == l2.max(): l2 = (np.arange(len(f)) % 2)
    stack += [f[l2 == 0], f[l2 == 1]]
os.makedirs('geom/bank_shell', exist_ok=True)
for o in os.listdir('geom/bank_shell'): os.remove('geom/bank_shell/' + o)
for i, h in enumerate(out): h.export(f'geom/bank_shell/piece_{i:03d}.obj')
nv = [len(h.vertices) for h in out]
allv = np.vstack([h.vertices for h in out])
print('pieces', len(out), 'max verts', max(nv), 'mean', np.mean(nv).round(1))
# checks: slot open? ray down the slot centre and at slot ends; interior hollow at centre
from trimesh import util
U = util.concatenate(out)
for x, y in [(0, -.004), (0, -.018), (0, .010), (.0015, -.004), (-.0015, -.004)]:
    hit = [h for h in out if Delaunay(h.vertices).find_simplex(np.array([[x, y, z] for z in np.linspace(.03, .112, 40)])).max() >= 0]
    print('column at', (x, y), 'blocked by', len(hit), 'pieces')
print('interior centre (0,-.009,.05) inside any piece:', any(Delaunay(h.vertices).find_simplex([[0, -.009, .05]])[0] >= 0 for h in out))
