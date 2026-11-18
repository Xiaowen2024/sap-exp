"""Parametric E2 fits (rack, basket) by multi-view silhouette IoU.

Every model part is convex, so its silhouette in a view is the convex hull of
its projected vertices; a model silhouette is the union of its parts.  The fit
maximises the mean IoU with the colour masks over the static views inside a
fixed region of interest per view.  Spread of the per-view optimum (leave-one-
view-out refits) is the E2 range.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R


def box_vertices(center, size, yaw):
    c = np.array([[sx, sy, sz] for sx in (-.5, .5) for sy in (-.5, .5) for sz in (-.5, .5)]) * size
    return c @ R.from_euler('z', yaw).as_matrix().T + center


def cyl_vertices(p0, p1, radius, n=12):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    a = p1 - p0
    a /= np.linalg.norm(a)
    u = np.cross(a, [0, 0, 1] if abs(a[2]) < .9 else [1, 0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    th = np.arange(n) * 2 * np.pi / n
    ring = radius * (np.cos(th)[:, None] * u + np.sin(th)[:, None] * v)
    return np.vstack([p0 + ring, p1 + ring])


# ---------------------------------------------------------------- rack model
RACK_KEYS = ['x', 'y', 'yaw', 'z0', 'L', 'w', 'h', 'D', 'dp', 'Hp', 'sp', 'dc']
RACK_INIT = {'L': .18, 'w': .025, 'h': .02, 'D': .10, 'dp': .010, 'Hp': .07, 'sp': .045, 'dc': .010}
N_PEGS = 4


def rack_parts(p):
    """Rack frame: x along the rails, y across, z up from the table."""
    Rz = R.from_euler('z', p['yaw']).as_matrix()
    o = np.array([p['x'], p['y'], p['z0']])
    parts = []
    local = []
    for side in (-1, 1):
        yc = side * p['D'] / 2
        local.append(('rail', 'box', np.array([0, yc, p['h'] / 2]), np.array([p['L'], p['w'], p['h']])))
        for k in range(N_PEGS):
            xk = (k - (N_PEGS - 1) / 2) * p['sp']
            local.append(('peg', 'cyl', np.array([xk, yc, p['h']]), np.array([xk, yc, p['h'] + p['Hp']])))
    inset = p['L'] / 2 - .02
    for sx in (-1, 1):
        local.append(('cross', 'cylx', np.array([sx * inset, -p['D'] / 2, p['h'] / 2]),
                      np.array([sx * inset, p['D'] / 2, p['h'] / 2])))
    for name, kind, a, b in local:
        if kind == 'box':
            v = box_vertices(np.zeros(3), b, 0) + a
            parts.append((name, v @ Rz.T + o))
        else:
            r = (p['dp'] if kind == 'cyl' else p['dc']) / 2
            parts.append((name, cyl_vertices(a @ Rz.T + o, b @ Rz.T + o, r)))
    return parts


# ------------------------------------------------------------- basket model
BASKET_KEYS = ['x', 'y', 'yaw', 'z0', 'a', 'b', 'k', 'H']


def basket_parts(p, n=32):
    """Elliptical frustum: top semi-axes (a, b) at height H, bottom (k a, k b)."""
    th = np.arange(n) * 2 * np.pi / n
    top = np.c_[p['a'] * np.cos(th), p['b'] * np.sin(th), np.full(n, p['H'])]
    bot = np.c_[p['k'] * p['a'] * np.cos(th), p['k'] * p['b'] * np.sin(th), np.zeros(n)]
    v = np.vstack([top, bot]) @ R.from_euler('z', p['yaw']).as_matrix().T + [p['x'], p['y'], p['z0']]
    return [('basket', v)]


def basket_parts6(p, n=32):
    """Same frustum with a full pose: bottom-centre (x, y, z) and rotation vector (rx, ry, rz)."""
    th = np.arange(n) * 2 * np.pi / n
    top = np.c_[p['a'] * np.cos(th), p['b'] * np.sin(th), np.full(n, p['H'])]
    bot = np.c_[p['k'] * p['a'] * np.cos(th), p['k'] * p['b'] * np.sin(th), np.zeros(n)]
    v = np.vstack([top, bot]) @ R.from_rotvec([p['rx'], p['ry'], p['rz']]).as_matrix().T + [p['x'], p['y'], p['z']]
    return [('basket', v)]


# ------------------------------------------------------------------ fitting
def render(ep, cam, parts, shape):
    import cv2
    m = np.zeros(shape, np.uint8)
    for _, v in parts:
        uv, z = ep.project(cam, v)
        if (z <= 0).any():
            continue
        hull = cv2.convexHull(np.round(uv * 4).astype(np.int32))
        cv2.fillConvexPoly(m, hull, 1, lineType=cv2.LINE_8, shift=2)
    return m.astype(bool)


def roi_for(ep, cam, parts, shape, margin=25):
    pts = np.vstack([v for _, v in parts])
    uv, _ = ep.project(cam, pts)
    x0, y0 = np.floor(uv.min(0)).astype(int) - margin
    x1, y1 = np.ceil(uv.max(0)).astype(int) + margin
    roi = np.zeros(shape, bool)
    roi[max(0, y0):max(0, y1), max(0, x0):max(0, x1)] = True
    return roi


def fit(ep, observed, builder, keys, init, fixed=None, views=None, rois=None, scales=None):
    """observed: {cam: bool mask}.  Returns (params, per-view IoU, rois)."""
    fixed = dict(fixed or {})
    views = list(views or observed)
    free = [k for k in keys if k not in fixed]
    x0 = np.array([init[k] for k in free])
    s = np.array([(scales or {}).get(k, .01) for k in free])
    shape = next(iter(observed.values())).shape
    if rois is None:
        parts0 = builder({**init, **fixed})
        rois = {c: roi_for(ep, c, parts0, shape) for c in views}

    def params(x):
        return {**fixed, **dict(zip(free, x * s))}

    def loss(x):
        p = params(x)
        if any(p[k] <= 0 for k in p if k not in ('x', 'y', 'yaw', 'z0')):
            return 2.0
        parts = builder(p)
        tot = 0.
        for c in views:
            r = render(ep, c, parts, shape) & rois[c]
            o = observed[c] & rois[c]
            tot += 1 - (r & o).sum() / max(1, (r | o).sum())
        return tot / len(views)

    res = minimize(loss, x0 / s, method='Powell', options={'xtol': 1e-3, 'ftol': 1e-5, 'maxfev': 6000})
    res = minimize(loss, res.x, method='Nelder-Mead', options={'xatol': 1e-3, 'fatol': 1e-6, 'maxfev': 4000})
    p = params(res.x)
    parts = builder(p)
    ious = {}
    for c in views:
        r = render(ep, c, parts, shape) & rois[c]
        o = observed[c] & rois[c]
        ious[c] = float((r & o).sum() / max(1, (r | o).sum()))
    return p, ious, rois


def fit_de(ep, observed, builder, bounds, fixed=None, views=None, roi_margin=40, seed=0, maxiter=120, workers=1):
    """Bounded global fit (differential evolution) of the silhouette IoU."""
    from scipy.optimize import differential_evolution
    fixed = dict(fixed or {})
    views = list(views or observed)
    keys = list(bounds)
    shape = next(iter(observed.values())).shape
    mid = {k: (lo + hi) / 2 for k, (lo, hi) in bounds.items()}
    parts0 = builder({**mid, **fixed})
    rois = {c: roi_for(ep, c, parts0, shape, roi_margin) for c in views}

    def loss(x):
        parts = builder({**fixed, **dict(zip(keys, x))})
        tot = 0.
        for c in views:
            r = render(ep, c, parts, shape) & rois[c]
            o = observed[c] & rois[c]
            tot += 1 - (r & o).sum() / max(1, (r | o).sum())
        return tot / len(views)

    res = differential_evolution(loss, [bounds[k] for k in keys], seed=seed, maxiter=maxiter,
                                 popsize=12, tol=1e-6, polish=False, workers=workers, updating='deferred' if workers != 1 else 'immediate')
    p = {**fixed, **dict(zip(keys, res.x))}
    parts = builder(p)
    ious = {}
    for c in views:
        r = render(ep, c, parts, shape) & rois[c]
        o = observed[c] & rois[c]
        ious[c] = float((r & o).sum() / max(1, (r | o).sum()))
    return p, ious, rois
