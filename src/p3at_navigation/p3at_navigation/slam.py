import numpy as np
from scipy import ndimage

from p3at_navigation.sensors import wrap

L_OCC = 0.85
L_FREE = -0.4
L_MIN, L_MAX = -4.0, 4.0


class OccupancyMapper:
    def __init__(self, width, height, resolution=0.05, origin=(-1.0, -1.0)):
        self.resolution = resolution
        self.origin = np.asarray(origin, dtype=float)
        self.nx = int(np.ceil(width / resolution))
        self.ny = int(np.ceil(height / resolution))
        self.logodds = np.zeros((self.ny, self.nx), dtype=np.float32)
        self._field = None

    def cells(self, x, y):
        ix = np.floor((x - self.origin[0]) / self.resolution).astype(int)
        iy = np.floor((y - self.origin[1]) / self.resolution).astype(int)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return ix, iy, ok

    def integrate(self, pose, ranges, spec, stride=1):
        x, y, th = pose
        sx = x + spec.mount_x * np.cos(th)
        sy = y + spec.mount_x * np.sin(th)
        angles = (th + spec.angles)[::stride]
        ranges = np.minimum(ranges[::stride], spec.max_range)
        valid = ranges < spec.max_range - 1e-3
        step = self.resolution
        d = np.arange(0.0, spec.max_range, step)
        keep = d[None, :] < (ranges[:, None] - step)
        fx = sx + np.cos(angles)[:, None] * d[None, :]
        fy = sy + np.sin(angles)[:, None] * d[None, :]
        ix, iy, ok = self.cells(fx[keep], fy[keep])
        free = np.zeros_like(self.logodds, dtype=bool)
        free[iy[ok], ix[ok]] = True
        ex = sx + ranges[valid] * np.cos(angles[valid])
        ey = sy + ranges[valid] * np.sin(angles[valid])
        jx, jy, ok2 = self.cells(ex, ey)
        occ = np.zeros_like(free)
        occ[jy[ok2], jx[ok2]] = True
        self.logodds[free & ~occ] += L_FREE
        self.logodds[occ] += L_OCC
        np.clip(self.logodds, L_MIN, L_MAX, out=self.logodds)
        self._field = None

    def occupied(self, threshold=0.6):
        return self.logodds > np.log(threshold / (1 - threshold))

    def known(self):
        return np.abs(self.logodds) > 0.1

    def likelihood_field(self, sigma=0.08):
        if self._field is None:
            occ = self.occupied()
            if not occ.any():
                self._field = np.zeros_like(self.logodds)
            else:
                dist = ndimage.distance_transform_edt(~occ) * self.resolution
                self._field = np.exp(-0.5 * (dist / sigma) ** 2)
        return self._field


class ScanMatcher:
    def __init__(self, xy_window=0.2, th_window=np.radians(6.0), beams=60):
        self.stages = [
            (np.arange(-xy_window, xy_window + 1e-9, 0.05), np.arange(-th_window, th_window + 1e-9, np.radians(1.0))),
            (np.arange(-0.05, 0.05 + 1e-9, 0.0125), np.arange(-np.radians(1.0), np.radians(1.0) + 1e-9, np.radians(0.25))),
        ]
        self.beams = beams

    def score(self, mapper, field, pose_candidates, angles, ranges, mount_x):
        x, y, th = pose_candidates[:, 0:1], pose_candidates[:, 1:2], pose_candidates[:, 2:3]
        sx = x + mount_x * np.cos(th)
        sy = y + mount_x * np.sin(th)
        ex = sx + ranges[None, :] * np.cos(th + angles[None, :])
        ey = sy + ranges[None, :] * np.sin(th + angles[None, :])
        ix, iy, ok = mapper.cells(ex, ey)
        values = np.zeros(ex.shape)
        values[ok] = field[iy[ok], ix[ok]]
        return values.sum(axis=1)

    def match(self, mapper, guess, ranges, spec):
        valid = ranges < spec.max_range - 1e-3
        idx = np.flatnonzero(valid)
        if idx.size < 20:
            return np.asarray(guess, dtype=float), 0.0
        idx = idx[np.linspace(0, idx.size - 1, min(self.beams, idx.size)).astype(int)]
        angles = spec.angles[idx]
        r = ranges[idx]
        field = mapper.likelihood_field()
        best, best_score = np.asarray(guess, dtype=float), -1.0
        for xy, th in self.stages:
            gx, gy, gt = np.meshgrid(xy, xy, th, indexing="ij")
            cand = np.stack([best[0] + gx.ravel(), best[1] + gy.ravel(), best[2] + gt.ravel()], axis=1)
            s = self.score(mapper, field, cand, angles, r, spec.mount_x)
            k = int(np.argmax(s))
            if s[k] > best_score:
                best, best_score = cand[k], float(s[k])
        best = best.copy()
        best[2] = wrap(best[2])
        return best, best_score / len(idx)


class ScanMatchingSlam:
    def __init__(self, width, height, spec, resolution=0.05, origin=(-1.0, -1.0), min_score=0.25):
        self.mapper = OccupancyMapper(width, height, resolution, origin)
        self.matcher = ScanMatcher()
        self.spec = spec
        self.pose = None
        self.last_odom = None
        self.min_score = min_score
        self.scans = 0

    def update(self, odom_pose, ranges):
        from p3at_navigation.sensors import apply_motion, relative_motion

        if self.pose is None:
            self.pose = np.asarray(odom_pose, dtype=float).copy()
        else:
            guess = apply_motion(self.pose, *relative_motion(self.last_odom, odom_pose))
            if self.scans >= 3:
                matched, score = self.matcher.match(self.mapper, guess, ranges, self.spec)
                self.pose = matched if score >= self.min_score else guess
            else:
                self.pose = guess
        self.last_odom = np.asarray(odom_pose, dtype=float).copy()
        self.mapper.integrate(self.pose, ranges, self.spec)
        self.scans += 1
        return self.pose.copy()
