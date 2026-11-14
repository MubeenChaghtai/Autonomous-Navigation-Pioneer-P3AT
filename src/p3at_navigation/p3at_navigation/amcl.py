import numpy as np

from p3at_navigation.sensors import relative_motion, wrap


class ParticleFilter:
    def __init__(self, grid, spec, n=500, rng=None, sigma_hit=0.1, z_hit=0.9, z_rand=0.1, beams=60, alpha=(0.08, 0.02, 0.08, 0.02), augmented=True, alpha_slow=0.001, alpha_fast=0.1):
        self.grid = grid
        self.spec = spec
        self.n = n
        self.rng = rng or np.random.default_rng()
        self.sigma_hit = sigma_hit
        self.z_hit = z_hit
        self.z_rand = z_rand
        self.beams = beams
        self.alpha = alpha
        self.augmented = augmented
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.w_slow = 0.0
        self.w_fast = 0.0
        self.particles = np.zeros((n, 3))
        self.weights = np.full(n, 1.0 / n)
        self.last_odom = None
        dist = grid.distance_field()
        self.field = dist
        iy, ix = np.nonzero(dist > 0.3)
        self.free_xy = np.stack(grid.to_world(ix, iy), axis=1)

    def init_pose(self, pose, sigma_xy=0.2, sigma_th=0.1):
        self.particles = np.column_stack([
            self.rng.normal(pose[0], sigma_xy, self.n),
            self.rng.normal(pose[1], sigma_xy, self.n),
            wrap(self.rng.normal(pose[2], sigma_th, self.n)),
        ])
        self.weights[:] = 1.0 / self.n

    def random_particles(self, k):
        idx = self.rng.integers(0, len(self.free_xy), k)
        return np.column_stack([self.free_xy[idx] + self.rng.uniform(-0.025, 0.025, (k, 2)), self.rng.uniform(-np.pi, np.pi, k)])

    def init_global(self, n=None):
        if n:
            self.n = n
        self.particles = self.random_particles(self.n)
        self.weights = np.full(self.n, 1.0 / self.n)

    def predict(self, odom_pose):
        if self.last_odom is None:
            self.last_odom = np.asarray(odom_pose, dtype=float)
            return False
        rot1, trans, rot2 = relative_motion(self.last_odom, odom_pose)
        self.last_odom = np.asarray(odom_pose, dtype=float)
        if trans < 1e-4 and abs(rot1 + rot2) < 1e-4:
            return False
        a1, a2, a3, a4 = self.alpha
        n = len(self.particles)
        r1 = rot1 - self.rng.normal(0, a1 * abs(rot1) + a2 * trans + 1e-4, n)
        t = trans - self.rng.normal(0, a3 * trans + a4 * (abs(rot1) + abs(rot2)) + 1e-4, n)
        r2 = rot2 - self.rng.normal(0, a1 * abs(rot2) + a2 * trans + 1e-4, n)
        th = self.particles[:, 2] + r1
        self.particles[:, 0] += t * np.cos(th)
        self.particles[:, 1] += t * np.sin(th)
        self.particles[:, 2] = wrap(th + r2)
        return True

    def likelihood(self, ranges):
        valid = np.flatnonzero(ranges < self.spec.max_range - 1e-3)
        if valid.size == 0:
            return np.zeros(len(self.particles)), 0
        idx = valid[np.linspace(0, valid.size - 1, min(self.beams, valid.size)).astype(int)]
        angles = self.spec.angles[idx]
        r = ranges[idx]
        p = self.particles
        sx = p[:, 0:1] + self.spec.mount_x * np.cos(p[:, 2:3])
        sy = p[:, 1:2] + self.spec.mount_x * np.sin(p[:, 2:3])
        ex = sx + r[None, :] * np.cos(p[:, 2:3] + angles[None, :])
        ey = sy + r[None, :] * np.sin(p[:, 2:3] + angles[None, :])
        ix, iy = self.grid.to_cell(ex, ey)
        ok = self.grid.inside(ix, iy)
        d = np.full(ex.shape, 2.0)
        d[ok] = self.field[iy[ok], ix[ok]]
        prob = self.z_hit * np.exp(-0.5 * (d / self.sigma_hit) ** 2) + self.z_rand / self.spec.max_range
        log_w = np.log(prob).sum(axis=1)
        inside_free = np.zeros(len(p), dtype=bool)
        cx, cy = self.grid.to_cell(p[:, 0], p[:, 1])
        okc = self.grid.inside(cx, cy)
        inside_free[okc] = ~self.grid.occupied[cy[okc], cx[okc]]
        log_w[~inside_free] -= 50.0
        return log_w, idx.size

    def update(self, odom_pose, ranges, force=False):
        moved = self.predict(odom_pose)
        if not moved and not force:
            return self.estimate()
        log_w, m = self.likelihood(ranges)
        log_w = log_w + np.log(self.weights + 1e-300)
        mx = log_w.max()
        w = np.exp(log_w - mx)
        avg_lik = float(np.mean(np.exp((log_w - np.log(self.weights + 1e-300)) / max(m, 1))))
        self.weights = w / w.sum()
        if self.augmented:
            self.w_slow += self.alpha_slow * (avg_lik - self.w_slow) if self.w_slow else avg_lik
            self.w_fast += self.alpha_fast * (avg_lik - self.w_fast) if self.w_fast else avg_lik
        if 1.0 / np.sum(self.weights ** 2) < self.n / 2:
            self.resample()
        return self.estimate()

    def resample(self):
        n = self.n
        inject = 0
        if self.augmented and self.w_slow > 0:
            inject = int(n * max(0.0, 1.0 - self.w_fast / self.w_slow))
            inject = min(inject, n // 2)
        keep = n - inject
        positions = (self.rng.random() + np.arange(keep)) / keep
        idx = np.minimum(np.searchsorted(np.cumsum(self.weights), positions), len(self.weights) - 1)
        new = self.particles[idx]
        if inject:
            new = np.vstack([new, self.random_particles(inject)])
        self.particles = new
        self.weights = np.full(n, 1.0 / n)

    def estimate(self):
        w = self.weights
        x = np.sum(w * self.particles[:, 0])
        y = np.sum(w * self.particles[:, 1])
        th = np.arctan2(np.sum(w * np.sin(self.particles[:, 2])), np.sum(w * np.cos(self.particles[:, 2])))
        spread = float(np.sqrt(np.sum(w * ((self.particles[:, 0] - x) ** 2 + (self.particles[:, 1] - y) ** 2))))
        return np.array([x, y, th]), spread
