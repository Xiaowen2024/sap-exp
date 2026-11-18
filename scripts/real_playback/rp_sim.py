"""Real-task playback v1 — RH20T dish(basket)-on-rack simulation core.

Shared by every engine: convex assets, the playback drive (capped 6-DoF
spring-damper on the gripper base + finger mode machine), and an
engine-independent observer (analytic signed distances of the fixtures and
pads evaluated at basket surface samples).  Engines only integrate.

World frame = robot base frame of the recording (metres); gravity -z.
Quaternions in this module are wxyz unless a name says xyzw.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R, Slerp

G = np.array([0., 0., -9.81])


# ----------------------------------------------------------------- quaternions
def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qrotvec(q):
    """rotation vector of unit quaternion q (wxyz), shortest arc."""
    if q[0] < 0:
        q = -q
    v = q[1:]
    s = np.linalg.norm(v)
    if s < 1e-12:
        return 2 * v
    return 2 * np.arctan2(s, q[0]) * v / s


def qmat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# -------------------------------------------------------------------- geometry
def hull(points):
    p = np.unique(np.round(np.asarray(points, float), 12), axis=0)
    h = ConvexHull(p)
    faces = h.simplices.copy()
    for i, f in enumerate(faces):
        n = np.cross(p[f[1]] - p[f[0]], p[f[2]] - p[f[0]])
        if n @ h.equations[i, :3] < 0:
            faces[i, [1, 2]] = faces[i, [2, 1]]
    return p, faces


def box_pts(center, size, Rm=np.eye(3)):
    c = np.array([[sx, sy, sz] for sx in (-.5, .5) for sy in (-.5, .5) for sz in (-.5, .5)]) * size
    return c @ Rm.T + center


def cyl_pts(p0, p1, r, n=12):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    a = (p1 - p0) / np.linalg.norm(p1 - p0)
    u = np.cross(a, [0, 0, 1] if abs(a[2]) < .9 else [1, 0, 0]); u /= np.linalg.norm(u)
    v = np.cross(a, u)
    th = (np.arange(n) + .5) * 2 * np.pi / n
    ring = r * (np.cos(th)[:, None] * u + np.sin(th)[:, None] * v)
    return np.vstack([p0 + ring, p1 + ring])


def mass_props(pieces):
    """Uniform-density volume, COM, inertia per unit density (about COM)."""
    vol, mom1, mom2 = 0., np.zeros(3), np.zeros((3, 3))
    for v, f in pieces:
        t = v[f]
        a, b, c = t[:, 0], t[:, 1], t[:, 2]
        dv = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6
        vol += dv.sum()
        mom1 += (dv[:, None] * (a + b + c) / 4).sum(0)
        for d, ai, bi, ci in zip(dv, a, b, c):
            u = ai + bi + ci
            mom2 += d / 20 * (np.outer(u, u) + np.outer(ai, ai) + np.outer(bi, bi) + np.outer(ci, ci))
    com = mom1 / vol
    cen = mom2 - vol * np.outer(com, com)
    return vol, com, np.trace(cen) * np.eye(3) - cen


def basket_geometry(b, n_seg=24):
    """Open elliptical tray.  Local frame: origin at bottom centre, z = axis.
    b: a, b (top outer semi-axes), k (bottom/top), H, wall_t, bottom_t, rim (variant)."""
    a, bb, k, H, t, tb = b['a'], b['b'], b['k'], b['H'], b['wall_t'], b['bottom_t']
    pieces = []
    th = np.linspace(0, 2 * np.pi, n_seg + 1)
    for i in range(n_seg):
        pts = []
        for ang in th[i:i + 2]:
            c, s = np.cos(ang), np.sin(ang)
            for (ra, rb, z) in [(k * a, k * bb, 0.), (a, bb, H)]:
                pts.append([ra * c, rb * s, z])
                pts.append([(ra - t) * c, (rb - t) * s, z])
        if b.get('rim', 'none') == 'rolled':
            # rolled rim: a thicker band at the top (declared variant)
            for ang in th[i:i + 2]:
                c, s = np.cos(ang), np.sin(ang)
                for z in (H - 1.5 * t, H):
                    pts.append([(a + .5 * t) * c, (bb + .5 * t) * s, z])
        pieces.append(hull(pts))
    tt = np.linspace(0, 2 * np.pi, 33)[:-1]
    bot = [[k * a * np.cos(x), k * bb * np.sin(x), z] for x in tt for z in (0., tb)]
    pieces.append(hull(bot))
    return pieces


def basket_samples(b, n_ang=72, n_z=6):
    """Surface sample points (outer, inner, rim, bottom) in the basket local frame."""
    a, bb, k, H, t = b['a'], b['b'], b['k'], b['H'], b['wall_t']
    th = np.linspace(0, 2 * np.pi, n_ang, endpoint=False)
    pts = []
    for s in np.linspace(0, 1, n_z):
        ra, rb, z = (k + (1 - k) * s) * a, (k + (1 - k) * s) * bb, s * H
        for off in (0., t):
            pts.append(np.c_[(ra - off) * np.cos(th), (rb - off) * np.sin(th), np.full(n_ang, z)])
    for f in (.25, .5, .75):
        pts.append(np.c_[f * k * a * np.cos(th), f * k * bb * np.sin(th), np.zeros(n_ang)])
    pts.append([[0, 0, 0.]])
    return np.vstack(pts)


def rack_members(r):
    """Rack local frame: origin at rack centre on the table, x along rails, z up."""
    mem = []
    for side in (-1, 1):
        yc = side * r['D'] / 2
        mem.append(('box', np.array([0, yc, r['h'] / 2]), np.array([r['L'], r['w'], r['h']])))
        for kk in range(4):
            xk = (kk - 1.5) * r['sp']
            mem.append(('cyl', np.array([xk, yc, r['h']]), np.array([xk, yc, r['h'] + r['Hp']]), r['dp'] / 2))
    inset = r['L'] / 2 - .02
    for sx in (-1, 1):
        mem.append(('cyl', np.array([sx * inset, -r['D'] / 2, r['h'] / 2]), np.array([sx * inset, r['D'] / 2, r['h'] / 2]), r['dc'] / 2))
    return mem


def rack_geometry(r):
    out = []
    for m in rack_members(r):
        if m[0] == 'box':
            out.append(hull(box_pts(m[1], m[2])))
        else:
            out.append(hull(cyl_pts(m[1], m[2], m[3])))
    return out


# --------------------------------------------------------------- signed distances
def sdf_box(p, center, half, Rm):
    q = (p - center) @ Rm
    d = np.abs(q) - half
    return np.linalg.norm(np.maximum(d, 0), axis=-1) + np.minimum(d.max(-1), 0)


def sdf_cyl(p, p0, p1, r):
    a = p1 - p0
    L = np.linalg.norm(a); a = a / L
    q = p - (p0 + p1) / 2
    h = q @ a
    rad = np.linalg.norm(q - h[:, None] * a, axis=1)
    d = np.c_[rad - r, np.abs(h) - L / 2]
    return np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(d.max(1), 0)


# ----------------------------------------------------------------------- scene
def solve_shape_mu(mu_pb, mu_be, mu_pe):
    """Shape-level mu for pad, basket, environment so that SAP's harmonic
    combine 2ab/(a+b) reproduces the three declared pair values exactly."""
    A = np.array([[1, 1, 0], [0, 1, 1], [1, 0, 1.]])
    inv = np.linalg.solve(A, 2 / np.array([mu_pb, mu_be, mu_pe]))
    if np.any(inv <= 0):
        raise ValueError('pair friction set not representable by harmonic combine')
    return dict(zip(('pad', 'basket', 'env'), 1 / inv))


def table_pose(scene):
    n = np.asarray(scene['table']['normal'], float); n /= np.linalg.norm(n)
    z = n; x = np.cross([0, 1, 0], z); x /= np.linalg.norm(x); y = np.cross(z, x)
    Rm = np.c_[x, y, z]
    th = scene['table']['box_thickness']
    c = np.asarray(scene['table']['point'], float) - n * th / 2
    return c, Rm


def place_on_table(scene, xy, yaw, z_off=0.):
    """Pose (pos, wxyz) of a body whose local z=0 plane rests on the table at xy with yaw about table normal."""
    _, Rt = table_pose(scene)
    n = Rt[:, 2]
    p0 = np.asarray(scene['table']['point'], float)
    # height of the table plane at xy
    z = p0[2] - (n[0] * (xy[0] - p0[0]) + n[1] * (xy[1] - p0[1])) / n[2]
    Rm = Rt @ R.from_euler('z', yaw).as_matrix()
    pos = np.array([xy[0], xy[1], z]) + n * z_off
    q = R.from_matrix(Rm).as_quat()[[3, 0, 1, 2]]
    return pos, q


# ----------------------------------------------------------------------- drive
class Playback:
    """Recorded TCP (125 Hz, base frame, wxyz) and gripper width/command."""

    def __init__(self, d, cfg):
        self.t = np.asarray(d['tcp_t'], float)
        P = np.asarray(d['tcp_pose_wxyz'], float)
        keep = np.r_[True, np.diff(self.t) > 1e-4]
        self.t, P = self.t[keep], P[keep]
        self.pos = CubicSpline(self.t, P[:, :3])
        self.vel = self.pos.derivative()
        Q = P[:, 3:].copy()
        for i in range(1, len(Q)):
            if Q[i] @ Q[i - 1] < 0:
                Q[i] = -Q[i]
        self.Q = Q
        self.rot = Slerp(self.t, R.from_quat(Q[:, [1, 2, 3, 0]]))
        self.gt = np.asarray(d['grip_t'], float)
        self.gw = np.asarray(d['grip_width_m'], float)
        self.gc = np.asarray(d['grip_cmd_m'], float)
        self.cfg = cfg
        if cfg.get('mode') == 'width_squeeze':
            # One continuous law, no mode switching: the finger target is the recorded
            # (actual) width minus a fixed squeeze offset per finger; the finger servo is
            # force-capped at the grip force, so when an object blocks the fingers they
            # press with the grip force, independent of small thickness errors.
            off = 2 * cfg.get('squeeze_per_finger_m', 0.002)
            self.gtarget = np.maximum(self.gw - off, 0.)
            self.closed = np.zeros(len(self.gt), bool)
        else:
            self.gtarget = self.gw
            self.closed = finger_modes(self.gt, self.gw, self.gc, cfg)

    def tip_rise(self, t):
        """AG-95 linkage: fingertips move on an arc of radius r and rise as the gripper opens
        (the recorded TCP is the closed-finger tip).  rise = r - sqrt(r^2 - x^2), x = half width."""
        r = self.cfg.get('finger_arc_r_m')
        if not r:
            return 0.
        x = min(float(np.interp(t, self.gt, self.gw)) / 2, 0.999 * r)
        return r - np.sqrt(r * r - x * x)

    def target(self, t):
        t = float(np.clip(t, self.t[0], self.t[-1]))
        p = self.pos(t); v = self.vel(t)
        if self.cfg.get('finger_arc_r_m'):
            zt = self.rot(t).as_matrix()[:, 2]      # tool z: flange -> fingertips
            h = 1e-3
            p = p - zt * self.tip_rise(t)            # the gripper rides higher by the tip rise
            v = v - zt * (self.tip_rise(t + h) - self.tip_rise(t - h)) / (2 * h)
        i = int(np.clip(np.searchsorted(self.t, t) - 1, 0, len(self.t) - 2))
        q = self.rot(t).as_quat()[[3, 0, 1, 2]]
        dq = qmul(self.Q[i + 1], qconj(self.Q[i]))
        w = qrotvec(dq) / (self.t[i + 1] - self.t[i])
        k = int(np.clip(np.searchsorted(self.gt, t) - 1, 0, len(self.gt) - 1))
        width = float(np.interp(t, self.gt, self.gtarget))
        return p, q, v, w, width, bool(self.closed[k])


def finger_modes(t, w, cmd, cfg):
    """E1 rule: CLOSED where the command is >= delta below the measured width
    and the measured width has stalled (|dw| < stall over the window) at >= min width."""
    out = np.zeros(len(t), bool)
    win = cfg['stall_window_s']
    for i in range(len(t)):
        j = np.searchsorted(t, t[i] - win)
        stalled = abs(w[i] - w[j]) < cfg['stall_tol_m']
        out[i] = (cmd[i] <= w[i] - cfg['cmd_delta_m']) and stalled and (w[i] >= cfg['min_hold_width_m'])
    # keep only CLOSED segments of at least min_hold_s (command/response lag
    # during stepped closing otherwise produces sub-0.3-s false stalls)
    keep = np.zeros(len(t), bool)
    i = 0
    while i < len(t):
        if out[i]:
            j = i
            while j < len(t) and out[j]:
                j += 1
            if t[min(j, len(t) - 1)] - t[i] >= cfg.get('min_hold_s', 0.3):
                keep[i:j] = True
            i = j
        else:
            i += 1
    return keep


def segments(t, flags):
    segs, start = [], None
    for ti, f in zip(t, flags):
        if f and start is None:
            start = ti
        if not f and start is not None:
            segs.append((start, ti)); start = None
    if start is not None:
        segs.append((start, t[-1]))
    return segs


def base_wrench(pos, q, v, w, tp, tq, tv, tw, d):
    """World force/torque on the base from the capped spring-damper + gravity feed-forward."""
    f = d['K_t'] * (tp - pos) + 2 * np.sqrt(d['K_t'] * d['m_total']) * (tv - v)
    n = np.linalg.norm(f)
    if n > d['F_cap']:
        f *= d['F_cap'] / n
    er = qrotvec(qmul(tq, qconj(q)))
    tau = d['K_r'] * er + 2 * np.sqrt(d['K_r'] * d['I_base']) * (tw - w)
    n = np.linalg.norm(tau)
    if n > d['T_cap']:
        tau *= d['T_cap'] / n
    return f - d['m_total'] * G, tau


def finger_force(q, qd, width, closed, d):
    """Generalised force on each outward slider (positive opens)."""
    if closed:
        # full closing force above 0.5 mm per side, fading to 0 at full closure
        return -d['F_close'] * min(1., max(q, 0.) / 5e-4) - d['finger_D_closed'] * qd
    f = d['finger_K'] * (width / 2 - q) - 2 * np.sqrt(d['finger_K'] * d['pad_mass']) * qd
    return float(np.clip(f, -d['F_close'], d['F_close']))


# --------------------------------------------------------------------- assets
def build_assets(scene, out):
    out = Path(out).resolve(); out.mkdir(parents=True, exist_ok=True)
    b = scene['basket']
    bp = basket_geometry(b)
    vol, com, Iu = mass_props(bp)
    pieces = {'basket': [], 'rack': [], 'pad': None, 'table': None}
    for i, (v, f) in enumerate(bp):
        path = out / f'basket_{i:02d}.obj'
        save_obj(path, v - com, f)
        pieces['basket'].append(str(path))
    for i, (v, f) in enumerate(rack_geometry(scene['rack'])):
        path = out / f'rack_{i:02d}.obj'
        save_obj(path, v, f)
        pieces['rack'].append(str(path))
    pv, pf = hull(box_pts(np.zeros(3), np.asarray(scene['pad']['size'])))
    save_obj(out / 'pad.obj', pv, pf); pieces['pad'] = str(out / 'pad.obj')
    meta = {'pieces': pieces, 'basket_com_local': com.tolist(), 'basket_volume': vol,
            'basket_inertia_per_kg': (Iu / vol).tolist()}
    (out / 'assets.json').write_text(json.dumps(meta, indent=1))
    return meta


def save_obj(path, v, f):
    Path(path).write_text('\n'.join(['v %.9f %.9f %.9f' % tuple(x) for x in v] +
                                    ['f %d %d %d' % tuple(x + 1) for x in f]) + '\n')


def load_obj(path):
    v, f = [], []
    for line in Path(path).read_text().splitlines():
        if line.startswith('v '):
            v.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            f.append([int(x) - 1 for x in line.split()[1:4]])
    return np.array(v), np.array(f)


def basket_body_pose(scene, meta):
    """Initial basket COM pose (wxyz) from the completion (bottom centre on table)."""
    b = scene['basket']
    if 'pose_override' in b:   # diagnostics: explicit COM pose (wxyz)
        return np.asarray(b['pose_override']['pos'], float), np.asarray(b['pose_override']['quat_wxyz'], float)
    pos, q = place_on_table(scene, b['xy'], b['yaw'])
    com = qmat(q) @ np.asarray(meta['basket_com_local']) + pos
    return com, q


# ------------------------------------------------------------------- observer
class Observer:
    """Engine-independent geometry: basket samples vs rack/pads/table SDFs."""

    def __init__(self, scene, meta):
        self.scene = scene
        self.samples = basket_samples(scene['basket']) - np.asarray(meta['basket_com_local'])
        rp, rq = place_on_table(scene, scene['rack']['xy'], scene['rack']['yaw'])
        Rr = qmat(rq)
        self.rack = []
        for m in rack_members(scene['rack']):
            if m[0] == 'box':
                self.rack.append(('box', Rr @ m[1] + rp, m[2] / 2, Rr))
            else:
                self.rack.append(('cyl', Rr @ m[1] + rp, Rr @ m[2] + rp, m[3]))
        self.rack_pose = (rp, Rr)
        self.table_c, self.table_R = table_pose(scene)
        self.pad_half = np.asarray(scene['pad']['size']) / 2

    def basket_points(self, pos, q):
        return self.samples @ qmat(q).T + pos

    def distances(self, pos, q, pads):
        """min signed distance (m) from the basket to: rack, table, each pad."""
        P = self.basket_points(pos, q)
        dr = np.inf
        for m in self.rack:
            d = sdf_box(P, m[1], m[2], m[3]) if m[0] == 'box' else sdf_cyl(P, m[1], m[2], m[3])
            dr = min(dr, float(d.min()))
        n = self.table_R[:, 2]
        top = self.table_c + n * self.scene['table']['box_thickness'] / 2
        dt = float(((P - top) @ n).min())
        dp = [float(sdf_box(P, pp, self.pad_half, qmat(pq)).min()) for pp, pq in pads]
        return dr, dt, dp

    def in_footprint(self, pos):
        rp, Rr = self.rack_pose
        r = self.scene['rack']
        loc = Rr.T @ (pos - rp)
        return abs(loc[0]) <= r['L'] / 2 and abs(loc[1]) <= (r['D'] + r['w']) / 2

    def upright_angle(self, q):
        """Angle (rad) between the basket axis and the table plane (0 = standing on edge)."""
        ax = qmat(q)[:, 2]
        return float(np.arcsin(min(1., abs(ax @ self.table_R[:, 2]))))


def write_json(path, obj):
    Path(path).write_text(json.dumps(obj, indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else float(o)))
