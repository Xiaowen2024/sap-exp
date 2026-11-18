import numpy as np, json, trimesh, glob
from scipy.spatial.transform import Rotation as R
from scipy.spatial import ConvexHull, cKDTree
from scipy.optimize import minimize
from scipy import ndimage
from PIL import Image, ImageDraw
d = np.load('episode/numeric.npz'); TABLE = .765; REACH = .1445
K = d['vision__cam_head__intrinsic_matrix']; Tc = d['vision__cam_head__extrinsic_matrix'][0]
def project(v): cam = (v - Tc[:3, 3]) @ Tc[:3, :3]; cam = cam * [1, -1, -1]; uv = cam @ K.T; return uv[:, :2] / uv[:, 2:3]
def ee(side, i):
    P = d[f'state__{side}_ee_poses'][i]; T = np.eye(4); T[:3, :3] = R.from_quat(P[[4, 5, 6, 3]]).as_matrix(); T[:3, 3] = P[:3]; return T
coin_meta = json.load(open('dl/Rigid_coin_00000/metadata.json')); stand_meta = json.load(open('dl/Geometry_vertical_coin_stand_00000/metadata.json'))
bank_meta = json.load(open('dl/Geometry_piggy_bank_00000/metadata.json'))
# --- stand + coin from the right-arm grasp (frame 45: closed, before lift)
G = ee('right', 45); xg, yg = G[:3, 0], G[:3, 1]; tip = G[:3, 3] + REACH * xg
n = yg / np.linalg.norm(yg)                              # coin normal = closing axis
yaw_s = np.arctan2(-n[0], n[1])                          # stand local +y along n (choose |yaw|<=90)
if abs(yaw_s) > np.pi / 2: yaw_s -= np.sign(yaw_s) * np.pi
Rs = R.from_euler('z', yaw_s); stand_z = TABLE + .0086
coin_c = np.r_[tip[:2], stand_z + .0068]                 # coin centre: below the fingertip centre, held in the stand slot
stand_p = coin_c - Rs.apply([0, 0, .0068]); stand_p[2] = stand_z
coin_top = coin_c[2] + .01425; finger_low = G[2, 3] - .1575 * abs(xg[2])
q_sup = np.array([0., 0., .7071068, .7071068])           # wxyz from stand support frame: coin z -> stand y
Rc = Rs * R.from_quat(q_sup[[1, 2, 3, 0]])
coin_origin = coin_c - Rc.apply([0, 0, .00097])          # coin origin is its bottom face; centre the coin in the slot
Tcoin = np.eye(4); Tcoin[:3, :3] = Rc.as_matrix(); Tcoin[:3, 3] = coin_origin
print('stand yaw %.1f deg pos %s | coin top %.4f finger bottom %.4f stand top %.4f' % (np.degrees(yaw_s), stand_p.round(4), coin_top, finger_low, stand_z + .0085))
# --- rigid carry: right grasp (45) -> handover (right open starts 115; left closed 115) -> release (left opens 158)
Th = ee('right', 112) @ np.linalg.inv(G) @ Tcoin
Tr = ee('left', 157) @ np.linalg.inv(ee('left', 113)) @ Th
cc_h = Th[:3, :3] @ [0, 0, .00097] + Th[:3, 3]; cc_r = Tr[:3, :3] @ [0, 0, .00097] + Tr[:3, 3]
nr = Tr[:3, 2]; print('coin centre at handover', cc_h.round(4), ' left fingertip', (ee('left', 113)[:3, 3] + REACH * ee('left', 113)[:3, 0]).round(4))
print('coin centre at release', cc_r.round(4), 'normal', nr.round(3), 'coin bottom z %.4f' % (cc_r[2] - .01425))
# --- bank: slot long axis (bank local y) lies in the coin plane & horizontal; slot centre under coin centre
inplane = np.cross(nr, [0, 0, 1]); inplane /= np.linalg.norm(inplane)
slot_c = np.array(bank_meta['passive']['functional']['slot_opening']['frame'][0][:3])
mesh = trimesh.load('geom/bank.obj'); V = mesh.vertices[ConvexHull(mesh.vertices).vertices]
img = np.asarray(Image.open('episode/cam_head_0000.jpg').convert('RGB')).astype(int)
pink = (img[:, :, 0] > 170) & (img[:, :, 0] - img[:, :, 2] > 45) & (img[:, :, 1] < 160) & (img[:, :, 0] - img[:, :, 1] > 45)
lab, nl = ndimage.label(pink); sz = ndimage.sum(pink, lab, range(1, nl + 1)); pink = lab == (np.argmax(sz) + 1)
edge = pink & ~ndimage.binary_erosion(pink); yy, xx = np.nonzero(edge); obs = np.c_[xx, yy].astype(float); tree = cKDTree(obs)
best = None
for sgn in (1, -1):
    yaw_b = np.arctan2(-sgn * inplane[0], sgn * inplane[1]); Rb = R.from_euler('z', yaw_b)
    bp = cc_r.copy(); bp[:2] -= Rb.apply(slot_c)[:2]; bp[2] = TABLE
    uv = project(Rb.apply(V) + bp); h = uv[ConvexHull(uv).vertices]
    pts = np.vstack([a + (b - a) * t for a, b in zip(h, np.roll(h, -1, 0)) for t in np.linspace(0, 1, 12)])
    err = np.sqrt((np.mean(tree.query(pts)[0] ** 2) + np.mean(cKDTree(pts).query(obs)[0] ** 2)) / 2)
    print('bank yaw %+.1f deg pos %s silhouette rms %.1f px' % (np.degrees(yaw_b), bp.round(4), err))
    if best is None or err < best[0]: best = (err, yaw_b, bp, h)
err, yaw_b, bp, h = best
canvas = Image.open('episode/cam_head_0000.jpg').convert('RGB'); dr = ImageDraw.Draw(canvas)
dr.line([tuple(p) for p in h] + [tuple(h[0])], fill=(0, 255, 0), width=1)
sv = trimesh.load('geom/stand.obj').vertices; uv = project(Rs.apply(sv) + stand_p); hs = uv[ConvexHull(uv).vertices]; dr.line([tuple(p) for p in hs] + [tuple(hs[0])], fill=(0, 200, 255), width=1)
cu = project(np.array([coin_c]))[0]; dr.ellipse([cu[0] - 3, cu[1] - 3, cu[0] + 3, cu[1] + 3], outline=(255, 255, 0))
canvas.resize((1280, 960)).save('fit_overlay.png')
json.dump({'stand': {'pos': stand_p.tolist(), 'yaw_deg': float(np.degrees(yaw_s)), 'quat_wxyz': list(Rs.as_quat()[[3, 0, 1, 2]])},
           'coin': {'origin': coin_origin.tolist(), 'quat_wxyz': list(Rc.as_quat()[[3, 0, 1, 2]]), 'centre': coin_c.tolist()},
           'bank': {'pos': bp.tolist(), 'yaw_deg': float(np.degrees(yaw_b)), 'quat_wxyz': list(R.from_euler('z', yaw_b).as_quat()[[3, 0, 1, 2]]), 'silhouette_rms_px': float(err)},
           'rigid_carry': {'coin_centre_release': cc_r.tolist(), 'coin_bottom_release': float(cc_r[2] - .01425), 'slot_top_world': float(TABLE + .1089)}}, open('scene_fit.json', 'w'), indent=1)
