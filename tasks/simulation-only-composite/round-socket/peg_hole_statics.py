"""Analytic references for the round peg-in-hole (2D statics in the tilt plane; valid by symmetry for a cylinder in a round bore).

Bore radius R = 12 mm, rim top z = 40 mm, base z = 10 mm (depth 30 mm); peg radius r = R - c, length 50 mm, gravity only.
Two-point (wedge) contact at tilt theta (top leaning to -x): the peg's bottom-right corner A touches the right wall x = +R and
the left rim corner B = (-R, 40) touches the peg's left side.  Inserted side length s_B = (2R - 2r cos(theta)) / sin(theta).
Static equilibrium (released at rest) is feasible iff contact forces inside the Coulomb cones balance gravity (an LP).
mu_star(theta) = smallest mu for which it is feasible.
"""
from __future__ import annotations
import math
import numpy as np
from scipy.optimize import linprog

R, Z_TOP, Z_BASE, LP = 12.0, 40.0, 10.0, 50.0


def wedge_pose(c, theta_deg):
    r = R - c; t = math.radians(theta_deg); a = np.array([-math.sin(t), math.cos(t)]); n = np.array([math.cos(t), math.sin(t)])
    sB = (2 * R - 2 * r * math.cos(t)) / math.sin(t)
    Pb = np.array([R - r * math.cos(t), Z_TOP + r * math.sin(t) - sB * math.cos(t)])        # bottom-face centre
    A = Pb + r * n; B = np.array([-R, Z_TOP]); G = Pb + 0.5 * LP * a
    return {"r": r, "a": a, "n": n, "sB": sB, "Pb": Pb, "A": A, "B": B, "G": G,
            "A_depth_below_rim": Z_TOP - A[1], "valid": (0 < Z_TOP - A[1] < Z_TOP - Z_BASE) and sB < LP}


def feasible(c, theta_deg, mu):
    p = wedge_pose(c, theta_deg)
    if not p["valid"]:
        return None
    cross = lambda u, v: u[0] * v[1] - u[1] * v[0]
    nA, tA = np.array([-1.0, 0.0]), np.array([0.0, 1.0]); nB, tB = p["n"], p["a"]
    G = p["G"]; A, B = p["A"], p["B"]
    Aeq = np.array([[nA[0], tA[0], nB[0], tB[0]], [nA[1], tA[1], nB[1], tB[1]],
                    [cross(A - G, nA), cross(A - G, tA), cross(B - G, nB), cross(B - G, tB)]])
    beq = np.array([0.0, 1.0, 0.0])                                                          # unit weight
    Aub = np.array([[-mu, 1, 0, 0], [-mu, -1, 0, 0], [0, 0, -mu, 1], [0, 0, -mu, -1]])
    res = linprog(np.zeros(4), A_ub=Aub, b_ub=np.zeros(4), A_eq=Aeq, b_eq=beq, bounds=[(0, None), (None, None), (0, None), (None, None)])
    return res.status == 0


def mu_star(c, theta_deg, hi=5.0):
    if feasible(c, theta_deg, hi) is None:
        return None
    if not feasible(c, theta_deg, hi):
        return math.inf
    lo = 0.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if feasible(c, theta_deg, mid) else (mid, hi)
    return hi


def theta_range(c):
    """Tilts at which the two-point wedge exists (bottom corner between rim top and base, rim contact on the side)."""
    return [t for t in np.arange(0.5, 40.01, 0.05) if wedge_pose(c, t)["valid"]]


def rim_stable_offset(c):
    """Upright peg resting on the rim top is statically stable iff its centre offset exceeds sqrt(R^2 - r^2)."""
    r = R - c
    return math.sqrt(R * R - r * r)


def free_entry_tilt(c):
    """Largest tilt at which a fully inserted (30 mm) tilted peg cannot form a two-point contact: L_max(theta) >= 30 mm."""
    r = R - c
    lo, hi = 0.01, 30.0
    for _ in range(60):
        mid = 0.5 * (lo + hi); t = math.radians(mid)
        lo, hi = (mid, hi) if (2 * R - 2 * r * math.cos(t)) / math.sin(t) >= 30.0 else (lo, mid)
    return lo


AZIMUTH_DEG = 5.625          # tilt plane through a flat-face centre of the 32-gon bore (inscribed radius exactly 12 mm there and at +180 deg)


def free_entry_clear(c, bottom_offset, theta_deg, margin=0.1):
    """True if a peg with this bottom-face offset and tilt (top leaning to -x), lowered vertically to the base, never comes within
    `margin` mm of either bore wall (tilt plane; cylinder cross-section there is the widest)."""
    r = R - c; t = math.radians(theta_deg); a = np.array([-math.sin(t), math.cos(t)]); n = np.array([math.cos(t), math.sin(t)])
    pts = [np.array([bottom_offset, 0.0]) + s * r * n + l * a for s in (-1, 1) for l in np.linspace(0, LP, 501)]
    zmin = min(p[1] for p in pts); shift = Z_BASE - zmin                    # bottom-most point resting on the base
    inside = [p + np.array([0, shift]) for p in pts if Z_BASE <= p[1] + shift <= Z_TOP]
    return all(abs(p[0]) <= R - margin for p in inside), shift


def wedge_label(c, theta_deg, mu, margin=0.2):
    ms = mu_star(c, theta_deg)
    if ms is None:
        return None, ms
    return ("hold" if mu >= (1 + margin) * ms else "insert" if mu <= (1 - margin) * ms else None), ms


# ---------------------------------------------------------------- contact-mode (acceleration-level LCP) analysis
EPS = 1e-6


def _mode_feasible(c, theta_deg, mu, modeA, modeB):
    """Is the rigid-body contact mode (modeA, modeB) consistent at release from rest under gravity?
    modes: 'sep' (lambda = 0, gap acc >= 0), 'stick' (gap acc = 0, tangential acc = 0, |lt| <= mu ln),
    'slide+' / 'slide-' (gap acc = 0, tangential acc >= +eps / <= -eps, lt = -/+ mu ln).  Unit mass, g = 1, mm.
    A 'sep' mode must actually separate (gap acc >= eps) unless the other contact provides the motion."""
    p = wedge_pose(c, theta_deg); r = p["r"]
    m = 1.0; I = m * (3 * r * r + LP * LP) / 12.0                     # solid cylinder, transverse axis through the COM
    G = p["G"]; cont = [(p["A"], np.array([-1.0, 0.0]), np.array([0.0, 1.0])), (p["B"], p["n"], p["a"])]
    # unknowns x = [ax, az, alpha, lnA, ltA, lnB, ltB]
    Aeq, beq, Aub, bub = [], [], [], []
    J = []
    for (P, nrm, tng) in cont:
        rr = P - G
        # acceleration of the body point at P (at rest): (ax - alpha*rz, az + alpha*rx)
        Jn = np.array([nrm[0], nrm[1], -nrm[0] * rr[1] + nrm[1] * rr[0]])
        Jt = np.array([tng[0], tng[1], -tng[0] * rr[1] + tng[1] * rr[0]])
        J.append((Jn, Jt))
    # Newton-Euler: m a = sum(ln n + lt t) + m g ; I alpha = sum r x F
    row_x = [m, 0, 0]; row_z = [0, m, 0]; row_a = [0, 0, I]
    eq = [np.array(row_x + [0, 0, 0, 0], float), np.array(row_z + [0, 0, 0, 0], float), np.array(row_a + [0, 0, 0, 0], float)]
    for k, (P, nrm, tng) in enumerate(cont):
        rr = P - G
        for j, d in enumerate((nrm, tng)):
            col = 3 + 2 * k + j
            eq[0][col] -= d[0]; eq[1][col] -= d[1]; eq[2][col] -= (rr[0] * d[1] - rr[1] * d[0])
    Aeq += eq; beq += [0.0, -m * 1.0, 0.0]
    moving = False
    for k, mode in enumerate((modeA, modeB)):
        Jn, Jt = J[k]; zn = np.zeros(7); zn[:3] = Jn; zt = np.zeros(7); zt[:3] = Jt
        ln = np.zeros(7); ln[3 + 2 * k] = 1.0; lt = np.zeros(7); lt[4 + 2 * k] = 1.0
        if mode == "sep":
            Aeq += [ln, lt]; beq += [0.0, 0.0]; Aub.append(-zn); bub.append(0.0)
        else:
            Aeq.append(zn); beq.append(0.0); Aub.append(-ln); bub.append(0.0)
            if mode == "stick":
                Aeq.append(zt); beq.append(0.0); Aub += [lt - mu * ln, -lt - mu * ln]; bub += [0.0, 0.0]
            elif mode == "slide+":
                Aub.append(-zt); bub.append(-EPS); Aeq.append(lt + mu * ln); beq.append(0.0); moving = True
            else:
                Aub.append(zt); bub.append(-EPS); Aeq.append(lt - mu * ln); beq.append(0.0); moving = True
    if "sep" in (modeA, modeB) and not moving:
        # require real separation of at least one separating contact
        k = 0 if modeA == "sep" else 1; zn = np.zeros(7); zn[:3] = J[k][0]; Aub.append(-zn); bub.append(-EPS)
    res = linprog(np.zeros(7), A_ub=np.array(Aub) if Aub else None, b_ub=np.array(bub) if bub else None,
                  A_eq=np.array(Aeq), b_eq=np.array(beq), bounds=[(None, None)] * 7, method="highs")
    return res.status == 0


MODES = ("sep", "stick", "slide+", "slide-")


def release_modes(c, theta_deg, mu):
    """All consistent contact modes at release; 'hold is unique' iff only (stick, stick) is consistent."""
    if not wedge_pose(c, theta_deg)["valid"]:
        return None
    return [(a, b) for a in MODES for b in MODES if _mode_feasible(c, theta_deg, mu, a, b)]


def certain_hold(c, theta_deg, mu, margin=0.2):
    """Unique static solution with friction margin: (stick, stick) is the only consistent mode at mu / (1 + margin)."""
    ms = release_modes(c, theta_deg, mu / (1 + margin))
    return ms is not None and ms == [("stick", "stick")]


def certain_slide(c, theta_deg, mu, margin=0.2):
    """No static solution even at mu * (1 + margin) (and mu* only grows as the peg slides deeper)."""
    ms = release_modes(c, theta_deg, mu * (1 + margin))
    return ms is not None and ("stick", "stick") not in ms


def mu_unique(c, theta_deg, hi=10.0):
    """Smallest mu at which (stick, stick) is the only consistent release mode (inf if none up to hi)."""
    ok = lambda mu: release_modes(c, theta_deg, mu) == [("stick", "stick")]
    if wedge_pose(c, theta_deg)["valid"] is False:
        return None
    if not ok(hi):
        return math.inf
    lo = 0.0
    for _ in range(40):
        mid = 0.5 * (lo + hi); lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    return hi
