#!/usr/bin/env python3
"""Write contactnets_si_v2.npz: same data as contactnets_si.npz but with the angular velocity un-scaled.

prepare_contactnets_cube_toss.py multiplied state[..., 7:13] by the 0.0524-m length scale. That is right for
linear velocity (7:10) but wrong for angular velocity (10:13, rad/s). This script divides 10:13 by the scale
and verifies against finite-differenced quaternions that the result is the body-frame angular velocity.
"""
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
from pathlib import Path
import numpy as np

ROOT = REPO_ROOT
SRC = ROOT / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si.npz"
OUT = ROOT / "exp-results/dynamic-impact-v1/cube-toss/contactnets_si_v2.npz"


def qmul(a, b):
    return np.r_[a[0] * b[0] - a[1:] @ b[1:], a[0] * b[1:] + b[0] * a[1:] + np.cross(a[1:], b[1:])]


def fd_body_omega(q0, q1, dt):
    if q0 @ q1 < 0:
        q1 = -q1
    d = qmul(np.r_[q0[0], -q0[1:]], q1)
    ang = 2 * np.arctan2(np.linalg.norm(d[1:]), d[0])
    return d[1:] / max(np.linalg.norm(d[1:]), 1e-12) * ang / dt


def main():
    d = dict(np.load(SRC))
    s = d["state"].copy(); scale = float(d["scale_m"]); dt = float(d["dt_s"])
    s[:, :, 10:13] /= scale
    A, B = [], []
    for i in range(len(s)):
        for k in range(2, min(int(d["lengths"][i]) - 1, 12)):
            if s[i, k, 2] > 0.12:
                A.append(0.5 * (s[i, k, 10:13] + s[i, k + 1, 10:13])); B.append(fd_body_omega(s[i, k, 3:7], s[i, k + 1, 3:7], dt))
    A, B = np.array(A), np.array(B)
    fit = float((A * B).sum() / (A * A).sum()); res = float(np.linalg.norm(B - fit * A) / np.linalg.norm(B))
    assert abs(fit - 1) < 0.02 and res < 0.05, (fit, res)
    d["state"] = s
    d["angular_velocity_frame"] = np.array("body")
    d["fix_note"] = np.array("v2: angular velocity divided by scale_m (was wrongly length-scaled); body frame; "
                             f"free-flight check vs finite-differenced quaternions: scale {fit:.4f}, rel. residual {res:.3f}")
    np.savez_compressed(OUT, **d)
    print("wrote", OUT, "| check: stored/fd scale", round(fit, 4), "residual", round(res, 3), "n", len(A))


if __name__ == "__main__":
    main()
