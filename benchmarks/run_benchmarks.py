import argparse
import csv
import json
import platform
import time

import numpy as np

import _paths
from p3at_navigation.amcl import ParticleFilter
from p3at_navigation.planning import costmap, plan
from p3at_navigation.simulation import NOISE_PROFILES, PLANNING_RADIUS, ate, build_maps, drive_tour, map_quality, run_navigation
from p3at_navigation.world import GridMap, Rect, free_points, office_layout

TOUR = [(2.5, 3.0, 0.0), (4.0, 1.0), (8.5, 1.0), (9.5, 4.0), (14.0, 3.2), (14.8, 6.5), (13.1, 8.8), (11.5, 10.5), (8.5, 8.5), (5.5, 8.2), (2.5, 10.8), (2.7, 6.0), (2.5, 3.0)]
PROFILES = ["low", "moderate", "high", "severe"]
PARTICLES = [50, 100, 250, 500, 1000]


def parse_args():
    parser = argparse.ArgumentParser(description="SLAM, localisation and navigation benchmark for the P3AT stack.")
    parser.add_argument("--seeds", type=int, default=5)
    parser.add_argument("--amcl-seeds", type=int, default=3)
    parser.add_argument("--global-trials", type=int, default=10)
    parser.add_argument("--nav-pairs", type=int, default=10)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--out", default=str(_paths.RESULTS))
    return parser.parse_args()


def write_csv(rows, path):
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def slam_study(grid, args, recordings):
    rows = []
    for profile in PROFILES:
        for s in range(args.seeds):
            truth, odoms, scans, spec = recordings[(profile, s)]
            reference, dead, slam_map, estimates, per_scan = build_maps(grid, truth, odoms, scans, spec)
            q_dead = map_quality(dead, reference, grid)
            q_slam = map_quality(slam_map, reference, grid)
            rows.append({
                "profile": profile, "seed": s, "scans": len(scans),
                "ate_odometry_m": ate(odoms, truth), "ate_slam_m": ate(estimates, truth),
                "final_error_odometry_m": float(np.hypot(*(odoms[-1, :2] - truth[-1, :2]))),
                "final_error_slam_m": float(np.hypot(*(estimates[-1, :2] - truth[-1, :2]))),
                "map_f1_odometry": q_dead["f1"], "map_f1_slam": q_slam["f1"],
                "map_precision_slam": q_slam["precision"], "map_recall_slam": q_slam["recall"],
                "slam_ms_per_scan": per_scan * 1000,
            })
        print(f"slam {profile} done", flush=True)
    return rows


def run_filter(grid, truth, odoms, scans, spec, n, rng, init="tracking", offset=None, augmented=True, kidnap_at=None):
    pf = ParticleFilter(grid, spec, n, rng, augmented=augmented)
    if init == "global":
        pf.init_global()
    else:
        pf.init_pose(truth[0] + np.array([rng.normal(0, 0.1), rng.normal(0, 0.1), rng.normal(0, 0.05)]), 0.15, 0.08)
    pf.last_odom = odoms[0].copy()
    errors, yaw_errors, seconds = [], [], 0.0
    for k, (t, o, z) in enumerate(zip(truth, odoms, scans)):
        if kidnap_at is not None and k == kidnap_at:
            pf.particles[:, :2] += offset[:2]
            pf.particles[:, 2] = (pf.particles[:, 2] + offset[2] + np.pi) % (2 * np.pi) - np.pi
        begin = time.perf_counter()
        est, _ = pf.update(o, z)
        seconds += time.perf_counter() - begin
        errors.append(float(np.hypot(*(est[:2] - t[:2]))))
        yaw_errors.append(float(abs((est[2] - t[2] + np.pi) % (2 * np.pi) - np.pi)))
    return np.array(errors), np.array(yaw_errors), seconds / len(scans)


def amcl_study(grid, args, recordings):
    rows = []
    for profile in PROFILES:
        for n in PARTICLES:
            for s in range(args.amcl_seeds):
                truth, odoms, scans, spec = recordings[(profile, s)]
                rng = np.random.default_rng([args.seed, 11, n, s, PROFILES.index(profile)])
                err, yaw, per = run_filter(grid, truth, odoms, scans, spec, n, rng)
                rows.append({"profile": profile, "particles": n, "seed": s, "mean_error_m": float(err.mean()), "p95_error_m": float(np.percentile(err, 95)), "mean_yaw_error_deg": float(np.degrees(yaw.mean())), "lost_fraction": float((err > 0.5).mean()), "final_error_m": float(err[-1]), "ms_per_update": per * 1000})
        print(f"amcl {profile} done", flush=True)
    return rows


def recovery_study(grid, args, recordings):
    rows, curves = [], {}
    for mode in ("global", "kidnapped"):
        for augmented in (True, False):
            series = []
            for k in range(args.global_trials):
                truth, odoms, scans, spec = recordings[("moderate", k % args.seeds)]
                rng = np.random.default_rng([args.seed, 13, k, int(augmented), mode == "global"])
                offset = None
                if mode == "kidnapped":
                    angle = rng.uniform(-np.pi, np.pi)
                    offset = np.array([3.0 * np.cos(angle), 3.0 * np.sin(angle), rng.uniform(-np.pi, np.pi)])
                n = 5000 if mode == "global" else 500
                kidnap_at = 60 if mode == "kidnapped" else None
                err, _, per = run_filter(grid, truth, odoms, scans, spec, n, rng, "global" if mode == "global" else "tracking", offset, augmented, kidnap_at)
                origin = kidnap_at or 0
                after = err[origin:]
                settled = None
                for i in np.flatnonzero(after < 0.3):
                    if np.all(after[i:i + 25] < 0.5):
                        settled = int(i)
                        break
                period = 0.4
                rows.append({"mode": mode, "augmented": augmented, "trial": k, "particles": n, "recovered": settled is not None and err[-1] < 0.5, "time_to_recover_s": None if settled is None else settled * period, "final_error_m": float(err[-1]), "ms_per_update": per * 1000})
                err = after
                series.append(err[:250].tolist())
            curves[f"{mode}_{'augmented' if augmented else 'standard'}"] = series
        print(f"recovery {mode} done", flush=True)
    return rows, curves


def unmapped_obstacles(layout, static_grid, rng, start, goal, count=3, tries=200):
    def feasible(candidate):
        truth = GridMap.from_layout(layout, extra=candidate)
        return plan(truth, costmap(truth.occupied, truth.resolution, PLANNING_RADIUS), start[:2], goal[:2]) is not None

    boxes = []
    route = plan(static_grid, costmap(static_grid.occupied, static_grid.resolution, PLANNING_RADIUS), start[:2], goal[:2])
    if route is not None and len(route) > 10:
        for _ in range(40):
            p = route[int(rng.uniform(0.3, 0.7) * (len(route) - 1))] + rng.normal(0.0, 0.05, 2)
            if np.hypot(*(p - start[:2])) < 1.5 or np.hypot(*(p - goal[:2])) < 1.5:
                continue
            size = rng.uniform(0.35, 0.55, 2)
            candidate = [Rect(p[0] - size[0] / 2, p[1] - size[1] / 2, size[0], size[1])]
            if feasible(candidate):
                boxes = candidate
                break
    for _ in range(tries):
        if len(boxes) == count:
            break
        p = free_points(static_grid, rng, 1, clearance=0.8)[0]
        if np.hypot(*(p - start[:2])) < 1.5 or np.hypot(*(p - goal[:2])) < 1.5:
            continue
        size = rng.uniform(0.35, 0.6, 2)
        candidate = boxes + [Rect(p[0] - size[0] / 2, p[1] - size[1] / 2, size[0], size[1])]
        if feasible(candidate):
            boxes = candidate
    return boxes


def navigation_study(layout, grid, args):
    rows = []
    rng = np.random.default_rng([args.seed, 17])
    cost = costmap(grid.occupied, grid.resolution, PLANNING_RADIUS)
    pairs = []
    while len(pairs) < args.nav_pairs:
        a, b = free_points(grid, rng, 2, clearance=0.6)
        if np.hypot(*(a - b)) > 6.0 and plan(grid, cost, a, b) is not None:
            pairs.append((np.array([a[0], a[1], rng.uniform(-np.pi, np.pi)]), b))
    examples = []
    for i, (start, goal) in enumerate(pairs):
        boxes = unmapped_obstacles(layout, grid, np.random.default_rng([args.seed, 19, i]), start, goal)
        truth_with = GridMap.from_layout(layout, extra=boxes)
        for profile in PROFILES:
            for scenario, truth in (("known map", grid), ("unmapped obstacles", truth_with)):
                trial_rng = np.random.default_rng([args.seed, 23, i, PROFILES.index(profile), scenario == "known map"])
                r = run_navigation(grid, truth, start, goal, NOISE_PROFILES[profile], trial_rng)
                rows.append({"pair": i, "profile": profile, "scenario": scenario, "distance_m": float(np.hypot(*(start[:2] - goal))), **r})
        examples.append({"start": start.tolist(), "goal": goal.tolist(), "boxes": [[b.x, b.y, b.w, b.h] for b in boxes]})
        print(f"navigation pair {i} done", flush=True)
    return rows, examples


def main():
    args = parse_args()
    out = _paths.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    layout = office_layout()
    grid = GridMap.from_layout(layout)
    start = time.time()
    recordings = {}
    for profile in PROFILES:
        for s in range(args.seeds):
            rng = np.random.default_rng([args.seed, 7, PROFILES.index(profile), s])
            recordings[(profile, s)] = drive_tour(grid, TOUR, NOISE_PROFILES[profile], rng, scan_every=4)
    print("tours recorded", flush=True)
    slam = slam_study(grid, args, recordings)
    write_csv(slam, out / "slam.csv")
    amcl = amcl_study(grid, args, recordings)
    write_csv(amcl, out / "amcl_tracking.csv")
    recovery, curves = recovery_study(grid, args, recordings)
    write_csv(recovery, out / "amcl_recovery.csv")
    nav, examples = navigation_study(layout, grid, args)
    write_csv(nav, out / "navigation.csv")
    profiles = {k: {"lidar_sigma_m": v.lidar_sigma, "outlier_rate": v.outlier_rate, "dropout_rate": v.dropout_rate, "odometry_alpha": [v.odom.a1, v.odom.a2, v.odom.a3, v.odom.a4], "yaw_bias_rad_per_m": v.odom.yaw_bias, "wheel_slip": v.slip} for k, v in NOISE_PROFILES.items()}
    summary = {
        "profiles": profiles,
        "recovery_curves": curves,
        "navigation_examples": examples,
        "setup": {"map_m": [layout.width, layout.height], "resolution_m": grid.resolution, "seeds": args.seeds, "amcl_seeds": args.amcl_seeds, "global_trials": args.global_trials, "nav_pairs": args.nav_pairs, "seed": args.seed, "python": platform.python_version(), "numpy": np.__version__, "wall_minutes": round((time.time() - start) / 60, 2)},
    }
    (out / "summary.json").write_text(json.dumps(summary), encoding="utf-8")
    print(f"finished in {summary['setup']['wall_minutes']} min")


if __name__ == "__main__":
    main()
