"""Symmetry-aware orientation error for a cube (pure NumPy; no engine imports).

A cube is invariant under the 24 proper rotations of its octahedral group, so
two poses that differ by one of those rotations look identical.  The raw
quaternion angle `2 acos |<q_sim, q_ref>|` can report up to pi rad for such
visually identical poses.  Use `cube_symmetric_qangle` for any cube pose
fidelity metric; keep the raw angle only as a historical/diagnostic value.

Quaternions are scalar-first (w, x, y, z), matching MuJoCo `qpos[3:7]`.
"""
from __future__ import annotations

import itertools

import numpy as np


def _mat_to_quat(R: np.ndarray) -> np.ndarray:
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
    y = np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0
    z = np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def _cube_group() -> np.ndarray:
    """Return the 24 proper cube-symmetry rotations as (24, 4) quaternions."""
    quats = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1.0, -1.0), repeat=3):
            R = np.zeros((3, 3))
            for row, (col, s) in enumerate(zip(perm, signs)):
                R[row, col] = s
            if np.linalg.det(R) > 0:
                quats.append(_mat_to_quat(R))
    out = np.asarray(quats)
    assert out.shape == (24, 4)
    return out


CUBE_SYMMETRY_QUATS = _cube_group()


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = np.moveaxis(a, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(b, -1, 0)
    return np.stack([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], axis=-1)


def raw_qangle(q_sim, q_ref) -> float:
    a = np.asarray(q_sim, float); b = np.asarray(q_ref, float)
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    return float(2.0 * np.arccos(np.clip(abs(np.dot(a, b)), 0.0, 1.0)))


def cube_symmetric_qangle(q_sim, q_ref) -> float:
    """Smallest rotation angle (rad) between two cube poses modulo body symmetry.

    Body-frame symmetry: q_sim and q_sim * g describe the same cube for every
    g in the octahedral group.  Result lies in [0, ~1.0985] rad (62.8 deg).
    """
    a = np.asarray(q_sim, float); b = np.asarray(q_ref, float)
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    candidates = _qmul(np.broadcast_to(a, CUBE_SYMMETRY_QUATS.shape), CUBE_SYMMETRY_QUATS)
    dots = np.clip(np.abs(candidates @ b), 0.0, 1.0)
    return float(2.0 * np.arccos(dots.max()))


def _self_test() -> None:
    rng = np.random.default_rng(0)
    for _ in range(200):
        q = rng.normal(size=4); q /= np.linalg.norm(q)
        g = CUBE_SYMMETRY_QUATS[rng.integers(24)]
        assert cube_symmetric_qangle(_qmul(q, g), q) < 1e-6
        assert cube_symmetric_qangle(q, q) < 1e-6
    # A 90-deg turn about a face axis is symmetric; raw angle says pi/2.
    q90 = np.array([np.cos(np.pi/4), 0.0, 0.0, np.sin(np.pi/4)])
    ident = np.array([1.0, 0.0, 0.0, 0.0])
    assert abs(raw_qangle(q90, ident) - np.pi/2) < 1e-9
    assert cube_symmetric_qangle(q90, ident) < 1e-6
    # A small 10-deg rotation is unchanged by the reduction.
    q10 = np.array([np.cos(np.deg2rad(5)), np.sin(np.deg2rad(5)), 0.0, 0.0])
    assert abs(cube_symmetric_qangle(q10, ident) - np.deg2rad(10)) < 1e-9


if __name__ == "__main__":
    _self_test()
    print("cube_orientation_metrics self-test passed")
