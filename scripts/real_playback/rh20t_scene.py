"""E2 scene measurement for RH20T task_0050 (cfg2): calibrated multi-view
silhouette carving of the basket and the rack, and table-plane fit.

Camera model: for every static camera the base-to-camera transform is taken
from the dataset's own pair of TCP streams (`tcp.npy` in camera frame and
`tcp_base.npy` in base frame, same timestamps); P0 verified it is constant to
machine precision with wxyz quaternions.  Intrinsics come from the official
calibration (1280x720) scaled by 1/4 to the released 320x180 colour video.
The wrist camera is excluded (its camera-frame TCP is a constant transform).
"""
from __future__ import annotations

from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

from .rh20t_io import load_npy

WRIST = '104422070042'
SCALE = 0.25


def T_from_wxyz(p):
    M = np.eye(4)
    M[:3, :3] = R.from_quat(np.asarray(p)[[4, 5, 6, 3]]).as_matrix()
    M[:3, 3] = p[:3]
    return M


class Episode:
    def __init__(self, lowdim_dir, color_dir, calib_dir):
        self.lowdim = Path(lowdim_dir)
        self.color = Path(color_dir)
        self.calib = Path(calib_dir)
        tr = self.lowdim / 'transformed'
        tc, tb = load_npy(tr / 'tcp.npy'), load_npy(tr / 'tcp_base.npy')
        K = load_npy(self.calib / 'intrinsics.npy')
        self.cams = {}
        for cam in sorted(tc):
            if cam == WRIST or not (self.color / f'cam_{cam}').exists() or not len(tc[cam]) or not len(tb[cam]):
                continue
            Ts = [T_from_wxyz(a['tcp']) @ np.linalg.inv(T_from_wxyz(b['tcp']))
                  for a, b in zip(tc[cam][:50], tb[cam][:50])]
            spread = max(np.abs(T - Ts[0]).max() for T in Ts)
            Kc = np.asarray(K[cam])[:, :3] * np.array([[SCALE], [SCALE], [1]])
            ts = np.asarray(load_npy(self.color / f'cam_{cam}' / 'color' / 'timestamps.npy'))
            self.cams[cam] = {'T_cam_base': Ts[0], 'K': Kc, 'spread': float(spread), 'ts': ts}
        hf = load_npy(tr / 'high_freq_data.npy')['base']
        self.hf_t = np.array([x['timestamp'] for x in hf], float) / 1e3
        self.hf_pose = np.array([x['tcp'] for x in hf], float)  # xyz + wxyz
        self.hf_ft = np.array([x['zeroed'] for x in hf], float)
        self.hf_raw = np.array([x['raw'] for x in hf], float)
        g = load_npy(tr / 'gripper.npy')
        rows = {}
        for cam in g:
            for t, v in g[cam].items():
                rows[t] = (float(v['gripper_info'][0]), float(v['gripper_command'][0]),
                           float(v['gripper_command'][1]), float(v['gripper_info'][1]))
        gt = np.array(sorted(rows))
        self.g_t = gt / 1e3
        self.g_width_mm = np.array([rows[t][0] for t in gt])
        self.g_cmd_mm = np.array([rows[t][1] for t in gt])
        self.g_force_pct = np.array([rows[t][2] for t in gt])

    def frame(self, cam, t):
        import cv2
        ts = self.cams[cam]['ts'] / 1e3
        k = int(np.argmin(abs(ts - t)))
        cap = cv2.VideoCapture(str(self.color / f'cam_{cam}' / 'color' / 'color.mp4'))
        cap.set(cv2.CAP_PROP_POS_FRAMES, k)
        ok, f = cap.read()
        cap.release()
        if not ok:
            raise IOError(f'cannot read frame {k} of {cam}')
        return f, float(ts[k])

    def project(self, cam, pts):
        c = self.cams[cam]
        pc = pts @ c['T_cam_base'][:3, :3].T + c['T_cam_base'][:3, 3]
        uv = pc @ c['K'].T
        z = uv[:, 2]
        return uv[:, :2] / np.where(np.abs(z) < 1e-9, 1e-9, z)[:, None], pc[:, 2]


def masks(frame_bgr):
    import cv2
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV).astype(int)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    yellow = (h >= 8) & (h <= 32) & (s >= 60) & (v >= 90)
    white = (s <= 45) & (v >= 165)
    return {'basket': yellow & (s >= 140), 'rack': yellow & (s < 140), 'any': yellow, 'white': white}


def carve(ep, t, lo, hi, voxel=0.002, label='any', allow_miss=1, frames=None):
    """Voxel centres in [lo, hi] (base frame) that project inside `label` mask in
    all but `allow_miss` of the static views that see them (>=4 views needed)."""
    xs = [np.arange(lo[i] + voxel / 2, hi[i], voxel) for i in range(3)]
    grid = np.stack(np.meshgrid(*xs, indexing='ij'), -1).reshape(-1, 3)
    votes = np.zeros(len(grid), int)
    seen = np.zeros(len(grid), int)
    used = {}
    for cam in ep.cams:
        f, tf = frames[cam] if frames else ep.frame(cam, t)
        m = masks(f)[label]
        uv, z = ep.project(cam, grid)
        u, v = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
        inside = (z > 0) & (u >= 0) & (v >= 0) & (u < m.shape[1]) & (v < m.shape[0])
        seen += inside
        hit = np.zeros(len(grid), bool)
        hit[inside] = m[v[inside], u[inside]]
        votes += hit
        used[cam] = tf
    keep = (seen >= 4) & (votes >= seen - allow_miss)
    return grid[keep], used


def components(pts, voxel, min_size=50):
    """26-connected components of a voxel set; largest first."""
    from scipy import ndimage
    if not len(pts):
        return []
    lo = pts.min(0)
    idx = np.round((pts - lo) / voxel).astype(int)
    vol = np.zeros(idx.max(0) + 1, bool)
    vol[tuple(idx.T)] = True
    lab, n = ndimage.label(vol, np.ones((3, 3, 3)))
    out = []
    for k in range(1, n + 1):
        sel = lab[tuple(idx.T)] == k
        if sel.sum() >= min_size:
            out.append(pts[sel])
    return sorted(out, key=len, reverse=True)


def fit_table_plane(calib_dir, ep, cams=None):
    """Plane z = a x + b y + c in base frame from the session's calibration depth
    images (green-mat pixels only), robust (median residual trimming)."""
    import cv2
    calib_dir = Path(calib_dir)
    K = load_npy(calib_dir / 'intrinsics.npy')
    pts_all = []
    per_cam = {}
    for cam in (cams or ep.cams):
        dfile = calib_dir / 'imgs' / f'cam_{cam}_d.npy'
        cfile = calib_dir / 'imgs' / f'cam_{cam}_c.png'
        if not dfile.exists():
            continue
        d = np.asarray(load_npy(dfile), float)
        c = cv2.imread(str(cfile))
        if d.shape[:2] != c.shape[:2]:
            continue
        hsv = cv2.cvtColor(c, cv2.COLOR_BGR2HSV).astype(int)
        green = (hsv[..., 0] >= 60) & (hsv[..., 0] <= 100) & (hsv[..., 1] >= 80) & (d > 0)
        v, u = np.nonzero(green)
        sel = np.random.default_rng(0).choice(len(u), min(20000, len(u)), replace=False)
        u, v = u[sel], v[sel]
        z = d[v, u] / 1000.0
        Kf = np.asarray(K[cam])[:, :3]
        x = (u - Kf[0, 2]) * z / Kf[0, 0]
        y = (v - Kf[1, 2]) * z / Kf[1, 1]
        pc = np.c_[x, y, z]
        Tbc = np.linalg.inv(ep.cams[cam]['T_cam_base'])
        pb = pc @ Tbc[:3, :3].T + Tbc[:3, 3]
        pts_all.append(pb)
        per_cam[cam] = float(np.median(pb[:, 2]))
    P = np.vstack(pts_all)
    keep = np.ones(len(P), bool)
    for _ in range(5):
        A = np.c_[P[keep, 0], P[keep, 1], np.ones(keep.sum())]
        coef, *_ = np.linalg.lstsq(A, P[keep, 2], rcond=None)
        r = P[:, 2] - (P[:, 0] * coef[0] + P[:, 1] * coef[1] + coef[2])
        keep = np.abs(r) < max(3 * 1.4826 * np.median(np.abs(r[keep])), 0.002)
    return {'coef_abc': coef.tolist(), 'residual_mad_m': float(1.4826 * np.median(np.abs(r[keep]))),
            'n_points': int(keep.sum()), 'per_camera_median_z_m': per_cam}
