"""Common world-frame, capped wrench and native-clock hand playback."""
from __future__ import annotations

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, Slerp

from .io_utils import GateError


def cap_norm(v, limit):
    v = np.asarray(v, float)
    return v * min(1.0, float(limit) / max(float(np.linalg.norm(v)), 1e-30))


def base_wrench(pose, twist, target_pose, target_twist, cfg):
    """xyzw quaternions; twists = world linear then world angular velocity.

    Saturation applies to the coupling; gravity feed-forward is then added.
    Both damping ratios and actual base inertia must be frozen explicitly.
    """
    pose, twist, target_pose, target_twist = map(np.asarray,
                                               (pose, twist, target_pose, target_twist))
    error_r = (Rotation.from_quat(target_pose[3:]) *
               Rotation.from_quat(pose[3:]).inv()).as_rotvec()
    force = cfg['K_t_N_m'] * (target_pose[:3] - pose[:3])
    force += 2 * np.sqrt(cfg['K_t_N_m'] * cfg['base_mass_kg']) * (target_twist[:3] - twist[:3])
    R = Rotation.from_quat(pose[3:]).as_matrix()
    I = R @ np.diag(cfg['base_inertia_kg_m2']) @ R.T
    # Principal-axis critical damping, rotated into world coordinates.
    damping = R @ np.diag(2 * np.sqrt(cfg['K_r_Nm_rad'] *
                                     np.asarray(cfg['base_inertia_kg_m2']))) @ R.T
    torque = cfg['K_r_Nm_rad'] * error_r + damping @ (target_twist[3:] - twist[3:])
    force = cap_norm(force, cfg['force_cap_N'])
    torque = cap_norm(torque, cfg['torque_cap_Nm'])
    force -= cfg['base_mass_kg'] * np.asarray(cfg['gravity_m_s2'])
    return np.r_[force, torque]


def closed_segments(t, width_m, delta_m=.002):
    """2-mm hysteresis, using running open plateau and closed minimum.

    The plateau/minimum rule is explicit, deterministic, and uses only real
    width. A final un-released grasp is rejected instead of inventing release.
    """
    t, width_m = np.asarray(t, float), np.asarray(width_m, float)
    if len(t) < 2 or len(t) != len(width_m) or not np.all(np.diff(t) > 0):
        raise GateError('invalid gripper clock')
    state = False
    plateau = width_m[0]
    stalled = None
    segments = []
    for ti, wi in zip(t, width_m):
        if not state:
            plateau = max(plateau, wi)
            if plateau - wi >= delta_m - 1e-12:
                start, stalled, state = ti, wi, True
        else:
            stalled = min(stalled, wi)
            if wi - stalled >= delta_m - 1e-12:
                segments.append((float(start), float(ti)))
                state, plateau = False, wi
    if state:
        raise GateError('last CLOSED segment has no recorded release')
    if not segments:
        raise GateError('no CLOSED segment in real width trace')
    return segments


class Playback:
    def __init__(self, t, pose_xyzw, width_t, width_m, tool_to_base):
        self.t = np.asarray(t, float)
        poses = np.asarray(pose_xyzw, float)
        self.width_t, self.width = np.asarray(width_t, float), np.asarray(width_m, float)
        if poses.shape != (len(self.t), 7) or len(self.t) < 4:
            raise GateError('TCP must contain >=4 native samples with xyz+xyzw')
        if not np.isfinite(poses).all() or not np.all(np.diff(self.t) > 0):
            raise GateError('non-finite TCP or non-monotone native timestamps')
        if not np.allclose(np.linalg.norm(poses[:, 3:], axis=1), 1, atol=.001):
            raise GateError('TCP quaternion norm not 1')
        if not np.isfinite(self.width).all() or np.any(self.width < 0):
            raise GateError('invalid gripper widths')
        self.position = CubicSpline(self.t, poses[:, :3], extrapolate=False)
        self.rotation = Slerp(self.t, Rotation.from_quat(poses[:, 3:]))
        self.tool_to_base = np.asarray(tool_to_base, float)
        self.segments = closed_segments(self.width_t, self.width)
        self.start = self.segments[0][0] - 1.0
        release = self.segments[-1][1]
        release_pos = self.position(release)
        withdrawn = self.t[(self.t > release) &
                           (np.linalg.norm(poses[:, :3] - release_pos, axis=1) >= .05)]
        if not len(withdrawn):
            raise GateError('recording does not show >=50-mm withdrawal')
        self.withdrawal = float(withdrawn[0])
        self.end = self.withdrawal + 2.0
        if self.start < max(self.t[0], self.width_t[0]) or self.end > min(self.t[-1], self.width_t[-1]):
            raise GateError('recording does not cover full replay and settle window')
        self.release = release

    def _pose(self, t):
        R = self.rotation(t)
        p = self.position(t) + R.apply(self.tool_to_base[:3, 3])
        q = (R * Rotation.from_matrix(self.tool_to_base[:3, :3])).as_quat()
        return np.r_[p, q]

    def target(self, t):
        if t < self.start - 1e-10 or t > self.end + 1e-10:
            raise GateError('playback extrapolation is prohibited')
        t = float(np.clip(t, self.start, self.end))
        pose = self._pose(t)
        # Slerp is piecewise constant in angular velocity; use its native segment.
        i = int(np.clip(np.searchsorted(self.t, t, side='right') - 1, 0, len(self.t)-2))
        a, b = self.rotation(self.t[i]), self.rotation(self.t[i+1])
        omega = (b * a.inv()).as_rotvec() / (self.t[i+1] - self.t[i])
        offset = self.rotation(t).apply(self.tool_to_base[:3, 3])
        velocity = self.position(t, 1) + np.cross(omega, offset)
        width = float(np.interp(t, self.width_t, self.width))
        mode = any(a <= t < b for a, b in self.segments)
        return pose, np.r_[velocity, omega], width, mode


def finger_force(position, velocity, recorded_width, closed, cfg):
    """Outward slider coordinate per pad; force is outward-positive."""
    if closed:
        return -cfg['closing_force_N'] - cfg['finger_damping_Ns_m'] * velocity
    raw = cfg['finger_K_N_m'] * (recorded_width / 2 - position)
    raw -= cfg['finger_damping_Ns_m'] * velocity
    return float(np.clip(raw, -cfg['finger_force_cap_N'], cfg['finger_force_cap_N']))
