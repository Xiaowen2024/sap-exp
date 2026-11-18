#!/usr/bin/env python3
"""Measure the effective bore radius of the shared 32-piece CoACD round socket.

For each height inside the bore, sample circles of radius r around the bore
axis and test the points against every convex piece (half-space form). The
effective radius at that height is the largest r with no sample inside any
piece. This is compared with the nominal 12-mm bore and with the peg radius
(12 - clearance) used in the clearance matrix.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull

ROOT = REPO_ROOT
ASSET = ROOT / "mujoco-sim/learn_mujoco/peg_in_hole_experiments/assets/collision"
OUT = ROOT / "exp-results/peg-in-hole/round-socket-coacd-bore-audit.json"

def load_obj(p):
    return np.array([[float(x) for x in l.split()[1:4]] for l in p.read_text().splitlines() if l.startswith("v ")])

pieces = [load_obj(ASSET / f"socket_piece_{i:02d}.obj") for i in range(32)]
hulls = [ConvexHull(v) for v in pieces]
allv = np.vstack(pieces)
lo, hi = allv.min(0), allv.max(0)
# Bore axis: the socket is centred at x=y=0 in the MJCF (peg starts at x=offset, y=0).
heights = np.linspace(lo[2] + 1e-4, hi[2] - 1e-4, 60)
theta = np.linspace(0, 2 * np.pi, 720, endpoint=False)
radii = np.arange(0.0100, 0.0126, 0.00002)

def inside_any(pts):
    hit = np.zeros(len(pts), bool)
    for h in hulls:
        A, b = h.equations[:, :3], h.equations[:, 3]
        hit |= np.all(pts @ A.T + b <= 1e-9, axis=1)
    return hit

rows = []
for z in heights:
    r_eff = None
    for r in radii:
        pts = np.c_[r * np.cos(theta), r * np.sin(theta), np.full_like(theta, z)]
        if inside_any(pts).any():
            break
        r_eff = r
    rows.append({"z_mm": round(float(z) * 1e3, 3), "effective_radius_mm": None if r_eff is None else round(r_eff * 1e3, 3)})
bore = [r for r in rows if r["effective_radius_mm"] is not None and r["effective_radius_mm"] < 12.55]
eff = np.array([r["effective_radius_mm"] for r in bore])
summary = {"pieces": len(pieces), "bounds_mm": {"min": (lo * 1e3).round(2).tolist(), "max": (hi * 1e3).round(2).tolist()},
           "nominal_bore_radius_mm": 12.0, "bore_slices": len(bore),
           "effective_radius_mm": {"min": float(eff.min()) if len(eff) else None, "median": float(np.median(eff)) if len(eff) else None,
                                    "max": float(eff.max()) if len(eff) else None},
           "peg_radius_by_clearance_mm": {str(c): 12.0 - c for c in (0.25, 0.5, 1.0, 1.5, 2.0)},
           "geometric_margin_by_clearance_mm": {str(c): (round(float(eff.min()) - (12.0 - c), 3) if len(eff) else None) for c in (0.25, 0.5, 1.0, 1.5, 2.0)}}
OUT.write_text(json.dumps({"summary": summary, "slices": rows}, indent=2) + "\n")
print(json.dumps(summary, indent=1))
