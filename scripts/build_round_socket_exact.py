#!/usr/bin/env python3
"""Build a round socket whose bore is exact by construction (replaces the 32-piece CoACD socket).

Same envelope as the CoACD asset: outer radius 50 mm, solid base z = 0-10 mm, wall z = 10-40 mm.  The wall is N
convex wedge prisms whose inner faces are flat chords at apothem exactly R_BORE, so the inscribed bore radius is
R_BORE everywhere (the inner corners lie at R_BORE / cos(pi/N)).  Writes triangulated OBJ files with outward faces
and verifies convexity, the inscribed bore, and the feasibility of each declared peg radius.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

R_BORE, R_OUT, Z_BASE, Z_TOP, N = 0.012, 0.050, 0.010, 0.040, 32


def prism(poly_xy, z0, z1):
    """Convex prism over a CCW convex polygon: vertices and outward triangles."""
    n = len(poly_xy)
    V = [(x, y, z0) for x, y in poly_xy] + [(x, y, z1) for x, y in poly_xy]
    F = [(0, i + 1, i) for i in range(1, n - 1)]                        # bottom (normal -z)
    F += [(n, n + i, n + i + 1) for i in range(1, n - 1)]               # top (normal +z)
    for i in range(n):
        j = (i + 1) % n
        F += [(i, j, n + j), (i, n + j, n + i)]                          # sides
    return np.array(V), F


def write_obj(path, V, F):
    with open(path, "w") as f:
        for v in V:
            f.write(f"v {v[0]:.9f} {v[1]:.9f} {v[2]:.9f}\n")
        for a, b, c in F:
            f.write(f"f {a + 1} {b + 1} {c + 1}\n")


def check_convex_outward(V, F):
    c = V.mean(0)
    for a, b, d in F:
        n = np.cross(V[b] - V[a], V[d] - V[a])
        if np.dot(n, V[a] - c) <= 0:
            return False
        if (np.dot(V - V[a], n) > 1e-12).any():
            return False
    return True


def build(out: Path):
    out.mkdir(parents=True, exist_ok=True)
    d = 2 * math.pi / N; rin = R_BORE / math.cos(d / 2); rout = R_OUT / math.cos(d / 2)
    files = []
    base = [(rout * math.cos(k * d), rout * math.sin(k * d)) for k in range(N)]
    V, F = prism(base, 0.0, Z_BASE); assert check_convex_outward(V, F); write_obj(out / "socket_base.obj", V, F); files.append("socket_base.obj")
    for k in range(N):
        a0, a1 = k * d, (k + 1) * d
        poly = [(rin * math.cos(a0), rin * math.sin(a0)), (rout * math.cos(a0), rout * math.sin(a0)),
                (rout * math.cos(a1), rout * math.sin(a1)), (rin * math.cos(a1), rin * math.sin(a1))]
        V, F = prism(poly, Z_BASE, Z_TOP); assert check_convex_outward(V, F)
        name = f"socket_wall_{k:02d}.obj"; write_obj(out / name, V, F); files.append(name)
    return files


def verify(out: Path, clearances_mm):
    # inscribed bore: min over wall pieces of the distance from the axis to the inner face plane
    d = 2 * math.pi / N; apothems = []
    for k in range(N):
        V = np.array([l.split()[1:4] for l in open(out / f"socket_wall_{k:02d}.obj") if l.startswith("v ")], float)
        p0, p1 = V[0, :2], V[3, :2]; t = p1 - p0; nrm = np.array([t[1], -t[0]]) / np.linalg.norm(t)
        apothems.append(abs(np.dot(p0, nrm)))
    r_ins = float(min(apothems))
    pegs = {f"{c:g}": {"peg_radius_mm": (R_BORE * 1e3 - c), "inscribed_bore_mm": r_ins * 1e3, "margin_mm": r_ins * 1e3 - (R_BORE * 1e3 - c),
                       "feasible": r_ins * 1e3 > (R_BORE * 1e3 - c)} for c in clearances_mm}
    return {"pieces": N + 1, "inscribed_bore_mm": r_ins * 1e3, "inner_corner_radius_mm": R_BORE / math.cos(d / 2) * 1e3,
            "outer_radius_mm": R_OUT * 1e3, "base_z_mm": [0, Z_BASE * 1e3], "wall_z_mm": [Z_BASE * 1e3, Z_TOP * 1e3], "pegs": pegs}


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / "assets/round_socket_exact")
    a = ap.parse_args()
    files = build(a.out)
    rep = verify(a.out, [0.5, 1.0, 2.0, 3.0]); rep["files"] = files
    (a.out / "geometry_audit.json").write_text(json.dumps(rep, indent=2) + "\n")
    print(json.dumps({k: v for k, v in rep.items() if k != "files"}, indent=2))
