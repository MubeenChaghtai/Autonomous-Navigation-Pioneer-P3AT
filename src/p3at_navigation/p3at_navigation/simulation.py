import time
from dataclasses import dataclass, field

import numpy as np

from p3at_navigation.amcl import ParticleFilter
from p3at_navigation.planning import PurePursuit, costmap, path_length, plan
from p3at_navigation.sensors import LidarSpec, OdometryNoise, corrupt_odometry, noisy_scan, raycast, wrap
from p3at_navigation.slam import OccupancyMapper, ScanMatchingSlam
from p3at_navigation.world import GridMap, Rect

ROBOT_RADIUS = 0.28
PLANNING_RADIUS = 0.38


@dataclass
class NoiseProfile:
    name: str
    lidar_sigma: float
    outlier_rate: float
    dropout_rate: float
    odom: OdometryNoise = field(default_factory=OdometryNoise)
    slip: float = 0.05


NOISE_PROFILES = {
    "low": NoiseProfile("low", 0.01, 0.0, 0.0, OdometryNoise(0.02, 0.005, 0.02, 0.005, 0.0), 0.02),
    "moderate": NoiseProfile("moderate", 0.03, 0.02, 0.02, OdometryNoise(0.05, 0.01, 0.05, 0.01, 0.01), 0.05),
    "high": NoiseProfile("high", 0.06, 0.05, 0.05, OdometryNoise(0.10, 0.02, 0.10, 0.02, 0.03), 0.10),
    "severe": NoiseProfile("severe", 0.10, 0.10, 0.10, OdometryNoise(0.20, 0.04, 0.15, 0.04, 0.05), 0.15),
}


def lidar_for(profile):
    return LidarSpec(sigma=profile.lidar_sigma, outlier_rate=profile.outlier_rate, dropout_rate=profile.dropout_rate)


def step_robot(pose, v, w, dt, rng, slip):
    v_eff = v * (1.0 + rng.normal(0.0, slip))
    w_eff = w * (1.0 + rng.normal(0.0, 2 * slip)) + rng.normal(0.0, slip * 0.1) * (abs(v) > 1e-3)
    th = pose[2] + 0.5 * w_eff * dt
    return np.array([pose[0] + v_eff * dt * np.cos(th), pose[1] + v_eff * dt * np.sin(th), wrap(pose[2] + w_eff * dt)])


def front_clearance(ranges, spec, half_angle=np.radians(35)):
    sel = np.abs(spec.angles) <= half_angle
    return float(ranges[sel].min()) - spec.mount_x


def run_navigation(static_grid, true_grid, start, goal, profile, rng, localization="amcl", particles=500, dt=0.1, replan_period=2.0, record=False):
    spec = lidar_for(profile)
    controller = PurePursuit()
    pose = np.array(start, dtype=float)
    odom = pose.copy()
    pf = None
    if localization == "amcl":
        pf = ParticleFilter(static_grid, spec, particles, rng)
        pf.init_pose(pose + np.array([rng.normal(0, 0.1), rng.normal(0, 0.1), rng.normal(0, 0.05)]))
        pf.last_odom = odom.copy()
    layer = OccupancyMapper(static_grid.nx * static_grid.resolution, static_grid.ny * static_grid.resolution, static_grid.resolution, static_grid.origin)
    static_cost = costmap(static_grid.occupied, static_grid.resolution, PLANNING_RADIUS)
    estimate = pose.copy()
    path = plan(static_grid, static_cost, estimate[:2], goal[:2])
    if path is None:
        return {"success": False, "reason": "no_initial_path"}
    reference_length = path_length(path)
    time_limit = reference_length / 0.15 + 40.0
    t, last_replan, travelled, errors, replans = 0.0, 0.0, 0.0, [], 0
    min_clearance = np.inf
    compute = 0.0
    trace = {"truth": [], "estimate": [], "paths": [path.copy()]}
    while t < time_limit:
        true_ranges = raycast(true_grid, pose, spec)
        scan = noisy_scan(true_ranges, spec, rng)
        begin = time.perf_counter()
        if pf is not None:
            estimate, _ = pf.update(odom, scan)
        else:
            estimate = pose.copy()
        errors.append(float(np.hypot(*(estimate[:2] - pose[:2]))))
        if record:
            trace["truth"].append(pose.copy())
            trace["estimate"].append(np.asarray(estimate).copy())
        layer.integrate(estimate, scan, spec, stride=3)
        dynamic = layer.occupied(0.7) & ~static_grid.occupied
        need_replan = t - last_replan >= replan_period
        if need_replan:
            cost = costmap(static_grid.occupied, static_grid.resolution, PLANNING_RADIUS, extra=dynamic)
            new_path = plan(static_grid, cost, estimate[:2], goal[:2])
            if new_path is not None:
                path = new_path
                replans += 1
                if record:
                    trace["paths"].append(path.copy())
            last_replan = t
        v, w, done = controller.command(estimate, path, front_clearance(scan, spec))
        compute += time.perf_counter() - begin
        if done:
            break
        new_pose = step_robot(pose, v, w, dt, rng, profile.slip)
        odom = corrupt_odometry(odom, pose, new_pose, profile.odom, rng)
        travelled += float(np.hypot(*(new_pose[:2] - pose[:2])))
        pose = new_pose
        clearance = float(true_grid.distance_field()[tuple(reversed([int(c) for c in true_grid.to_cell(pose[0], pose[1])]))]) if true_grid.inside(*true_grid.to_cell(pose[0], pose[1])) else 0.0
        min_clearance = min(min_clearance, clearance)
        if clearance < ROBOT_RADIUS:
            return {"success": False, "reason": "collision", "time": t, "travelled": travelled, "reference_length": reference_length, "loc_error_mean": float(np.mean(errors)), "loc_error_max": float(np.max(errors)), "replans": replans, "min_clearance": min_clearance, "compute_ms_per_step": 1000 * compute / max(len(errors), 1)}
        t += dt
    reached = float(np.hypot(*(pose[:2] - np.asarray(goal[:2])))) < 0.35
    if record:
        return {"success": bool(reached), "trace": trace, "dynamic": layer.occupied(0.7) & ~static_grid.occupied}
    return {
        "success": bool(reached),
        "reason": "reached" if reached else ("stopped_short" if t < time_limit else "timeout"),
        "time": t,
        "travelled": travelled,
        "reference_length": reference_length,
        "loc_error_mean": float(np.mean(errors)),
        "loc_error_max": float(np.max(errors)),
        "replans": replans,
        "min_clearance": min_clearance,
        "compute_ms_per_step": 1000 * compute / max(len(errors), 1),
    }


def drive_tour(grid, waypoints, profile, rng, dt=0.1, scan_every=2):
    spec = lidar_for(profile)
    controller = PurePursuit(max_v=0.4)
    cost = costmap(grid.occupied, grid.resolution, PLANNING_RADIUS)
    pose = np.array(waypoints[0], dtype=float)
    odom = pose.copy()
    truth, odoms, scans = [], [], []
    k = 0
    for goal in waypoints[1:]:
        path = plan(grid, cost, pose[:2], goal[:2])
        if path is None:
            continue
        for _ in range(4000):
            ranges = raycast(grid, pose, spec)
            if k % scan_every == 0:
                truth.append(pose.copy())
                odoms.append(odom.copy())
                scans.append(noisy_scan(ranges, spec, rng))
            k += 1
            v, w, done = controller.command(pose, path, front_clearance(ranges, spec))
            if done:
                break
            new_pose = step_robot(pose, v, w, dt, rng, profile.slip)
            odom = corrupt_odometry(odom, pose, new_pose, profile.odom, rng)
            pose = new_pose
    return np.array(truth), np.array(odoms), scans, spec


def build_maps(grid, truth, odoms, scans, spec):
    size = (grid.nx * grid.resolution + 2.0, grid.ny * grid.resolution + 2.0)
    reference = OccupancyMapper(*size, grid.resolution, (-1.0, -1.0))
    dead = OccupancyMapper(*size, grid.resolution, (-1.0, -1.0))
    slam = ScanMatchingSlam(*size, spec, grid.resolution, (-1.0, -1.0))
    estimates = []
    begin = time.perf_counter()
    for t, o, z in zip(truth, odoms, scans):
        estimates.append(slam.update(o, z))
    slam_seconds = time.perf_counter() - begin
    for t, o, z in zip(truth, odoms, scans):
        reference.integrate(t, z, spec, stride=2)
        dead.integrate(o, z, spec, stride=2)
    return reference, dead, slam.mapper, np.array(estimates), slam_seconds / max(len(scans), 1)


def ate(estimates, truth):
    return float(np.sqrt(np.mean(np.sum((estimates[:, :2] - truth[:, :2]) ** 2, axis=1))))


def map_quality(mapper, reference, grid, tolerance=0.10):
    from scipy import ndimage

    pred = mapper.occupied()
    iy, ix = np.nonzero(pred)
    xs = mapper.origin[0] + (ix + 0.5) * mapper.resolution
    ys = mapper.origin[1] + (iy + 0.5) * mapper.resolution
    gx, gy = grid.to_cell(xs, ys)
    ok = grid.inside(gx, gy)
    dist = grid.distance_field()
    near_truth = np.zeros(len(xs), dtype=bool)
    near_truth[ok] = dist[gy[ok], gx[ok]] <= tolerance
    precision = float(near_truth.mean()) if len(xs) else 0.0
    ref = reference.occupied()
    pred_dist = ndimage.distance_transform_edt(~pred) * mapper.resolution
    recall = float((pred_dist[ref] <= tolerance).mean()) if ref.any() else 0.0
    f1 = 2 * precision * recall / max(precision + recall, 1e-9)
    return {"precision": precision, "recall": recall, "f1": f1}
