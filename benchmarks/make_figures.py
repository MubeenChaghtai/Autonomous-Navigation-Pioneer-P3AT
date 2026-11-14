import argparse
import csv
import json
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import _paths
from p3at_navigation.simulation import NOISE_PROFILES, build_maps, drive_tour, run_navigation
from p3at_navigation.world import GridMap, Rect, office_layout
from run_benchmarks import PARTICLES, PROFILES, TOUR

INK, MUTED, GRID = "#1f1f1e", "#5f5e5a", "#e4e3de"
BLUE, ORANGE, AQUA, YELLOW = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
PROFILE_COLOURS = {"low": AQUA, "moderate": BLUE, "high": YELLOW, "severe": ORANGE}


def style(ax, title=None, xlabel=None, ylabel=None):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    if title:
        ax.set_title(title, loc="left", fontsize=11, color=INK)
    if xlabel:
        ax.set_xlabel(xlabel, color=MUTED)
    if ylabel:
        ax.set_ylabel(ylabel, color=MUTED)


def read(path):
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def num(v):
    return float(v) if v not in ("", "None") else np.nan


def show_map(ax, occupancy, extent, title):
    ax.imshow(occupancy, origin="lower", extent=extent, cmap="Greys", vmin=0, vmax=1, interpolation="nearest")
    ax.set_title(title, loc="left", fontsize=10, color=INK)
    ax.set_xticks([])
    ax.set_yticks([])


def slam_maps_figure(out, seed, profile="high"):
    grid = GridMap.from_layout(office_layout())
    rng = np.random.default_rng([seed, 7, PROFILES.index(profile), 0])
    truth, odoms, scans, spec = drive_tour(grid, TOUR, NOISE_PROFILES[profile], rng, scan_every=4)
    reference, dead, slam_map, est, _ = build_maps(grid, truth, odoms, scans, spec)
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.6))
    show_map(axes[0], grid.occupied.astype(float), [0, 16, 0, 12], "Ground truth and driven route")
    axes[0].plot(truth[:, 0], truth[:, 1], color=BLUE, linewidth=1.5)
    ext = [-1, -1 + dead.nx * dead.resolution, -1, -1 + dead.ny * dead.resolution]
    show_map(axes[1], 1 / (1 + np.exp(-dead.logodds)), ext, f"Mapping from odometry only ({profile} noise)")
    axes[1].plot(odoms[:, 0], odoms[:, 1], color=ORANGE, linewidth=1.2)
    show_map(axes[2], 1 / (1 + np.exp(-slam_map.logodds)), ext, f"Scan-matching SLAM ({profile} noise)")
    axes[2].plot(est[:, 0], est[:, 1], color=AQUA, linewidth=1.2)
    for ax in axes[1:]:
        ax.set_xlim(-1, 17)
        ax.set_ylim(-1, 13)
    fig.tight_layout()
    fig.savefig(out / "slam_maps.png", dpi=140)
    plt.close(fig)


def slam_figure(rows, out):
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.2))
    x = np.arange(len(PROFILES))
    for key, label, colour, marker in (("odometry", "odometry only", ORANGE, "s"), ("slam", "scan-matching SLAM", AQUA, "o")):
        ate = [np.median([num(r[f"ate_{key}_m"]) for r in rows if r["profile"] == p]) for p in PROFILES]
        f1 = [np.median([num(r[f"map_f1_{key}"]) for r in rows if r["profile"] == p]) for p in PROFILES]
        axes[0].plot(x, ate, marker=marker, markersize=6, linewidth=2, color=colour, label=label)
        axes[1].plot(x, f1, marker=marker, markersize=6, linewidth=2, color=colour, label=label)
    axes[0].set_yscale("log")
    for ax in axes:
        ax.set_xticks(x, PROFILES)
    style(axes[0], "Trajectory error (median of 5 runs)", "noise profile", "ATE RMSE, m (log scale)")
    style(axes[1], "Map accuracy (median of 5 runs)", "noise profile", "occupied-cell F1 at 10 cm tolerance")
    axes[1].set_ylim(0, 1.03)
    axes[0].legend(frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(out / "slam_accuracy.png", dpi=160)
    plt.close(fig)


def amcl_figure(rows, out):
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.2))
    for p in PROFILES:
        mean = [np.mean([num(r["mean_error_m"]) for r in rows if r["profile"] == p and int(r["particles"]) == n]) for n in PARTICLES]
        lost = [np.mean([num(r["lost_fraction"]) for r in rows if r["profile"] == p and int(r["particles"]) == n]) for n in PARTICLES]
        axes[0].plot(PARTICLES, np.array(mean) * 100, marker="o", markersize=5, linewidth=2, color=PROFILE_COLOURS[p], label=p)
        axes[1].plot(PARTICLES, lost, marker="o", markersize=5, linewidth=2, color=PROFILE_COLOURS[p], label=p)
    for ax in axes:
        ax.set_xscale("log")
        ax.set_xticks(PARTICLES, [str(n) for n in PARTICLES])
        ax.minorticks_off()
    style(axes[0], "AMCL tracking error", "particles (log scale)", "mean position error, cm")
    style(axes[1], "Share of time lost (error > 0.5 m)", "particles (log scale)", "fraction of updates")
    axes[0].legend(frameon=False, fontsize=9, title="noise profile", title_fontsize=9)
    fig.tight_layout()
    fig.savefig(out / "amcl_tracking.png", dpi=160)
    plt.close(fig)


def recovery_figure(curves, out):
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.2))
    for ax, mode, title in ((axes[0], "kidnapped", "Kidnapped robot: displaced 3 m after 24 s of tracking"), (axes[1], "global", "Global localisation from a uniform prior")):
        for variant, colour, label in (("augmented", AQUA, "augmented MCL (random particle injection)"), ("standard", ORANGE, "standard MCL")):
            series = curves.get(f"{mode}_{variant}", [])
            if not series:
                continue
            length = min(len(s) for s in series)
            arr = np.array([s[:length] for s in series])
            t = np.arange(length) * 0.4
            ax.fill_between(t, np.percentile(arr, 25, axis=0), np.percentile(arr, 75, axis=0), color=colour, alpha=0.15, linewidth=0)
            ax.plot(t, np.median(arr, axis=0), color=colour, linewidth=2, label=label)
        ax.set_yscale("log")
        style(ax, title, "seconds since kidnapping" if mode == "kidnapped" else "seconds since start", "position error, m (log scale)")
    axes[0].legend(frameon=False, fontsize=8.5)
    fig.tight_layout()
    fig.savefig(out / "amcl_recovery.png", dpi=160)
    plt.close(fig)


def navigation_figure(rows, out):
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.2))
    x = np.arange(len(PROFILES))
    width = 0.36
    for k, (scenario, colour) in enumerate((("known map", BLUE), ("unmapped obstacles", ORANGE))):
        sel = lambda p: [r for r in rows if r["profile"] == p and r["scenario"] == scenario]
        success = [np.mean([r["success"] == "True" for r in sel(p)]) for p in PROFILES]
        times = [np.median([num(r["time"]) for r in sel(p) if r["success"] == "True"]) for p in PROFILES]
        axes[0].bar(x + (k - 0.5) * width, success, width * 0.92, color=colour, label=scenario)
        axes[1].bar(x + (k - 0.5) * width, times, width * 0.92, color=colour, label=scenario)
    for ax in axes:
        ax.set_xticks(x, PROFILES)
    axes[0].set_ylim(0, 1.05)
    style(axes[0], "Goals reached without collision", "noise profile", "success rate")
    style(axes[1], "Time to goal (successful runs, median)", "noise profile", "seconds")
    axes[0].legend(frameon=False, fontsize=9, loc="lower left")
    fig.tight_layout()
    fig.savefig(out / "navigation.png", dpi=160)
    plt.close(fig)


def detour_pair(nav):
    best, gain = 0, -np.inf
    for i in sorted({int(r["pair"]) for r in nav}):
        known = [r for r in nav if int(r["pair"]) == i and r["profile"] == "high" and r["scenario"] == "known map" and r["success"] == "True"]
        unmapped = [r for r in nav if int(r["pair"]) == i and r["profile"] == "high" and r["scenario"] == "unmapped obstacles" and r["success"] == "True"]
        if known and unmapped and num(unmapped[0]["travelled"]) - num(known[0]["travelled"]) > gain:
            best, gain = i, num(unmapped[0]["travelled"]) - num(known[0]["travelled"])
    return best


def navigation_example(summary, out, seed, pair=0):
    example = summary["navigation_examples"][pair]
    layout = office_layout()
    static = GridMap.from_layout(layout)
    boxes = [Rect(*b) for b in example["boxes"]]
    truth_grid = GridMap.from_layout(layout, extra=boxes)
    result = run_navigation(static, truth_grid, np.array(example["start"]), np.array(example["goal"]), NOISE_PROFILES["high"], np.random.default_rng([seed, 31]), record=True)
    fig, ax = plt.subplots(figsize=(9, 6.8))
    show = np.zeros(static.occupied.shape)
    show[static.occupied] = 1.0
    show[truth_grid.occupied & ~static.occupied] = 0.55
    ax.imshow(show, origin="lower", extent=[0, 16, 0, 12], cmap="Greys", vmin=0, vmax=1, interpolation="nearest")
    tr = np.array(result["trace"]["truth"])
    es = np.array(result["trace"]["estimate"])
    first = result["trace"]["paths"][0] if result["trace"]["paths"] else None
    if first is not None:
        ax.plot(first[:, 0], first[:, 1], color=MUTED, linewidth=1.2, linestyle="--", label="initial plan (static map only)")
    ax.plot(tr[:, 0], tr[:, 1], color=BLUE, linewidth=2, label="driven path (ground truth)")
    ax.plot(es[:, 0], es[:, 1], color=ORANGE, linewidth=1, alpha=0.8, label="AMCL estimate")
    ax.scatter(*example["start"][:2], s=70, color=INK, zorder=3, label="start")
    ax.scatter(*example["goal"][:2], s=110, marker="*", color=ORANGE, zorder=3, label="goal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(f"Navigation with high noise and unmapped obstacles (grey boxes): {'reached' if result['success'] else 'failed'}", loc="left", fontsize=10, color=INK)
    ax.legend(frameon=False, fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.01), ncol=5)
    fig.tight_layout()
    fig.savefig(out / "navigation_example.png", dpi=150)
    plt.close(fig)


def markdown(slam, amcl, recovery, nav, out):
    lines = ["| Noise profile | ATE odometry (m) | ATE SLAM (m) | Final error odometry (m) | Final error SLAM (m) | Map F1 odometry | Map F1 SLAM | SLAM time per scan (ms) |", "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for p in PROFILES:
        sel = [r for r in slam if r["profile"] == p]
        med = lambda k: np.median([num(r[k]) for r in sel])
        lines.append(f"| {p} | {med('ate_odometry_m'):.3f} | {med('ate_slam_m'):.3f} | {med('final_error_odometry_m'):.3f} | {med('final_error_slam_m'):.3f} | {med('map_f1_odometry'):.3f} | {med('map_f1_slam'):.3f} | {med('slam_ms_per_scan'):.1f} |")
    lines += ["", "| Noise profile | " + " | ".join(f"{n} particles" for n in PARTICLES) + " |", "|---|" + "---:|" * len(PARTICLES)]
    for p in PROFILES:
        cells = []
        for n in PARTICLES:
            sel = [r for r in amcl if r["profile"] == p and int(r["particles"]) == n]
            cells.append(f"{100 * np.mean([num(r['mean_error_m']) for r in sel]):.1f} cm / {np.mean([num(r['lost_fraction']) for r in sel]):.0%}")
        lines.append(f"| {p} | " + " | ".join(cells) + " |")
    lines += ["", "| Scenario | Variant | Particles | Recovered | Median time to recover (s) |", "|---|---|---:|---:|---:|"]
    for mode in ("kidnapped", "global"):
        for aug in ("True", "False"):
            sel = [r for r in recovery if r["mode"] == mode and r["augmented"] == aug]
            times = [num(r["time_to_recover_s"]) for r in sel if r["recovered"] == "True"]
            lines.append(f"| {mode} | {'augmented' if aug == 'True' else 'standard'} | {sel[0]['particles']} | {sum(r['recovered'] == 'True' for r in sel)}/{len(sel)} | {np.median(times):.1f} |" if times else f"| {mode} | {'augmented' if aug == 'True' else 'standard'} | {sel[0]['particles']} | 0/{len(sel)} | n/a |")
    lines += ["", "| Noise profile | Scenario | Success | Collisions | Median time (s) | Mean localisation error (cm) | Max localisation error (cm) | Mean replans |", "|---|---|---:|---:|---:|---:|---:|---:|"]
    for p in PROFILES:
        for scenario in ("known map", "unmapped obstacles"):
            sel = [r for r in nav if r["profile"] == p and r["scenario"] == scenario]
            ok = [r for r in sel if r["success"] == "True"]
            lines.append(f"| {p} | {scenario} | {len(ok)}/{len(sel)} | {sum(r['reason'] == 'collision' for r in sel)} | {np.median([num(r['time']) for r in ok]):.1f} | {100 * np.mean([num(r['loc_error_mean']) for r in sel]):.1f} | {100 * np.max([num(r['loc_error_max']) for r in sel]):.1f} | {np.mean([num(r['replans']) for r in sel]):.1f} |")
    (out / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Render figures and tables from the benchmark outputs.")
    parser.add_argument("--results", default=str(_paths.RESULTS))
    parser.add_argument("--seed", type=int, default=2026)
    args = parser.parse_args()
    results = _paths.Path(args.results)
    figures = results / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    slam = read(results / "slam.csv")
    amcl = read(results / "amcl_tracking.csv")
    recovery = read(results / "amcl_recovery.csv")
    nav = read(results / "navigation.csv")
    summary = json.loads((results / "summary.json").read_text(encoding="utf-8"))
    slam_maps_figure(figures, args.seed)
    slam_figure(slam, figures)
    amcl_figure(amcl, figures)
    recovery_figure(summary["recovery_curves"], figures)
    navigation_figure(nav, figures)
    navigation_example(summary, figures, args.seed, detour_pair(nav))
    markdown(slam, amcl, recovery, nav, results)
    print(f"figures in {figures}")


if __name__ == "__main__":
    main()
