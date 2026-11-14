import heapq
import itertools

import numpy as np

from p3at_navigation.sensors import wrap

MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]


def costmap(occupied, resolution, robot_radius=0.35, inflation=0.6, extra=None):
    from scipy import ndimage

    occ = occupied if extra is None else (occupied | extra)
    dist = ndimage.distance_transform_edt(~occ) * resolution
    lethal = dist < robot_radius
    cost = np.where(lethal, np.inf, np.exp(-3.0 * np.clip(dist - robot_radius, 0, None) / max(inflation, 1e-6)) * 10.0)
    return cost


def astar(cost, start, goal):
    ny, nx = cost.shape
    if not (0 <= start[0] < nx and 0 <= start[1] < ny and 0 <= goal[0] < nx and 0 <= goal[1] < ny):
        return None
    if not np.isfinite(cost[goal[1], goal[0]]):
        return None
    counter = itertools.count()
    h = lambda c: np.hypot(c[0] - goal[0], c[1] - goal[1])
    frontier = [(h(start), next(counter), start)]
    g = {start: 0.0}
    parent = {start: None}
    closed = set()
    while frontier:
        _, _, cur = heapq.heappop(frontier)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        for dx, dy, step in MOVES:
            nb = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nb[0] < nx and 0 <= nb[1] < ny):
                continue
            c = cost[nb[1], nb[0]]
            if not np.isfinite(c):
                continue
            ng = g[cur] + step * (1.0 + c)
            if ng < g.get(nb, np.inf):
                g[nb] = ng
                parent[nb] = cur
                heapq.heappush(frontier, (ng + h(nb), next(counter), nb))
    return None


def plan(grid, cost, start_xy, goal_xy):
    sx, sy = grid.to_cell(start_xy[0], start_xy[1])
    gx, gy = grid.to_cell(goal_xy[0], goal_xy[1])
    start = (int(sx), int(sy))
    if not np.isfinite(cost[start[1], start[0]]):
        finite = np.argwhere(np.isfinite(cost))
        d = (finite[:, 1] - start[0]) ** 2 + (finite[:, 0] - start[1]) ** 2
        y0, x0 = finite[np.argmin(d)]
        start = (int(x0), int(y0))
    cells = astar(cost, start, (int(gx), int(gy)))
    if cells is None:
        return None
    xs, ys = grid.to_world(np.array([c[0] for c in cells]), np.array([c[1] for c in cells]))
    return np.stack([xs, ys], axis=1)


def path_length(path):
    return float(np.sum(np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1])))) if path is not None and len(path) > 1 else 0.0


class PurePursuit:
    def __init__(self, lookahead=0.6, max_v=0.5, max_w=1.2, goal_tolerance=0.25):
        self.lookahead = lookahead
        self.max_v = max_v
        self.max_w = max_w
        self.goal_tolerance = goal_tolerance

    def command(self, pose, path, front_clearance):
        x, y, th = pose
        d = np.hypot(path[:, 0] - x, path[:, 1] - y)
        nearest = int(np.argmin(d))
        ahead = np.flatnonzero(d[nearest:] >= self.lookahead)
        target = path[nearest + ahead[0]] if ahead.size else path[-1]
        if np.hypot(path[-1, 0] - x, path[-1, 1] - y) < self.goal_tolerance:
            return 0.0, 0.0, True
        alpha = wrap(np.arctan2(target[1] - y, target[0] - x) - th)
        if abs(alpha) > np.radians(60):
            return 0.0, float(np.clip(1.5 * alpha, -self.max_w, self.max_w)), False
        v = self.max_v * max(0.2, np.cos(alpha)) * float(np.clip((front_clearance - 0.35) / 0.6, 0.0, 1.0))
        w = 2.0 * v * np.sin(alpha) / max(self.lookahead, 1e-6) if v > 1e-3 else 1.5 * alpha
        return float(v), float(np.clip(w, -self.max_w, self.max_w)), False
