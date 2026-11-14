from dataclasses import dataclass

import numpy as np


@dataclass
class LidarSpec:
    beams: int = 181
    fov: float = np.pi
    max_range: float = 10.0
    min_range: float = 0.1
    sigma: float = 0.01
    outlier_rate: float = 0.0
    dropout_rate: float = 0.0
    mount_x: float = 0.16

    @property
    def angles(self):
        return np.linspace(-self.fov / 2, self.fov / 2, self.beams)


def raycast(grid, pose, spec, extra_grid=None):
    x, y, th = pose
    sx = x + spec.mount_x * np.cos(th)
    sy = y + spec.mount_x * np.sin(th)
    angles = th + spec.angles
    step = grid.resolution * 0.5
    r = np.arange(spec.min_range, spec.max_range + step, step)
    px = sx + np.cos(angles)[:, None] * r[None, :]
    py = sy + np.sin(angles)[:, None] * r[None, :]
    hit = grid.is_occupied(px, py)
    if extra_grid is not None:
        hit |= extra_grid.is_occupied(px, py)
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    ranges = np.where(first >= 0, r[np.maximum(first, 0)], spec.max_range)
    return ranges


def noisy_scan(true_ranges, spec, rng):
    z = true_ranges + rng.normal(0.0, spec.sigma, true_ranges.shape)
    if spec.outlier_rate > 0:
        out = rng.random(z.shape) < spec.outlier_rate
        z[out] = rng.uniform(spec.min_range, true_ranges[out] + 1e-3)
    if spec.dropout_rate > 0:
        z[rng.random(z.shape) < spec.dropout_rate] = spec.max_range
    return np.clip(z, spec.min_range, spec.max_range)


@dataclass
class OdometryNoise:
    a1: float = 0.05
    a2: float = 0.01
    a3: float = 0.05
    a4: float = 0.01
    yaw_bias: float = 0.0


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def relative_motion(p0, p1):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    trans = np.hypot(dx, dy)
    rot1 = wrap(np.arctan2(dy, dx) - p0[2]) if trans > 1e-6 else 0.0
    rot2 = wrap(p1[2] - p0[2] - rot1)
    return rot1, trans, rot2


def apply_motion(pose, rot1, trans, rot2):
    x, y, th = pose
    th1 = th + rot1
    return np.array([x + trans * np.cos(th1), y + trans * np.sin(th1), wrap(th1 + rot2)])


def corrupt_odometry(odom_pose, true_prev, true_next, noise, rng):
    rot1, trans, rot2 = relative_motion(true_prev, true_next)
    s_rot1 = noise.a1 * abs(rot1) + noise.a2 * trans
    s_trans = noise.a3 * trans + noise.a4 * (abs(rot1) + abs(rot2))
    s_rot2 = noise.a1 * abs(rot2) + noise.a2 * trans
    r1 = rot1 + rng.normal(0, s_rot1) + noise.yaw_bias * trans
    t = trans + rng.normal(0, s_trans)
    r2 = rot2 + rng.normal(0, s_rot2)
    return apply_motion(odom_pose, r1, t, r2)
