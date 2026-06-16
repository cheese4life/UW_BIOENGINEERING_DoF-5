# note: this file was written by AI to automatically take csv results from
# benchmark and create plots for data visualization.

#!/usr/bin/env python3
"""Plot latency benchmark results from dof_latency_bench.py output.

Reads trial_summary.csv and raw_trace_*.csv from a benchmark run directory
and generates the 7 plots specified in LATENCY_SPEC.md Section 8.

Can run anywhere (Mac dev machine or bench machine) — no CAN hardware needed.

Usage:
    python3 scripts/plot_latency_bench.py ./bench_20260611_120000
    python3 scripts/plot_latency_bench.py ./bench_20260611_120000 --format svg
    python3 scripts/plot_latency_bench.py ./bench_20260611_120000 --dpi 150
"""
from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def load_trial_summary(bench_dir: Path) -> list[dict]:
    """Parse trial_summary.csv → list of dicts."""
    rows = []
    path = bench_dir / "trial_summary.csv"
    if not path.exists():
        print(f"[ERROR] {path} not found")
        sys.exit(1)
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Convert numeric fields
            for key in ("trial_id", "direction", "target_counts", "home_counts",
                        "final_pos_counts", "final_error_counts"):
                row[key] = int(row[key]) if row[key] else 0
            for key in ("t_cmd_ns", "t_react_ns", "t_engage_ns", "t_complete_ns"):
                row[key] = int(row[key]) if row[key] and row[key] != "" else None
            for key in ("reaction_us", "motion_us", "settle_us", "total_us"):
                val = row[key]
                row[key] = float(val) if val and val != "" else float("nan")
            rows.append(row)
    print(f"[load] {len(rows)} trials from trial_summary.csv")
    return rows


def load_raw_traces(bench_dir: Path) -> dict[str, list[tuple[int, list[tuple[float, float]]]]]:
    """Parse raw_trace_*.csv → {config_name: [(trial_id, [(t_ms, pos_um), ...]), ...]}.

    Converts ns → ms and counts → µm on load so plotting is trivial.
    """
    traces: dict[str, list] = defaultdict(list)
    for path in sorted(bench_dir.glob("raw_trace_*.csv")):
        config_name = path.stem.replace("raw_trace_", "")
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            current_trial = None
            current_trace = []
            for row in reader:
                tid = int(row["trial_id"])
                t_ms = float(row["t_ns_since_cmd"]) / 1e6  # ns → ms
                pos_um = float(row["position_counts"]) / 200.0  # counts → µm
                if current_trial is None:
                    current_trial = tid
                if tid != current_trial:
                    traces[config_name].append((current_trial, current_trace))
                    current_trial = tid
                    current_trace = []
                current_trace.append((t_ms, pos_um))
            if current_trace:
                traces[config_name].append((current_trial, current_trace))
        print(f"[load] {len(traces[config_name])} traces from {path.name}")
    return dict(traces)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _extract_distance_um(config_name: str) -> int:
    """'010um_pos' → 10"""
    return int(config_name[:3])


def _extract_direction(config_name: str) -> str:
    """'010um_pos' → 'pos'"""
    return config_name.split("_")[1]


def _sorted_configs(rows: list[dict]) -> list[str]:
    """Return unique config names sorted by distance then direction."""
    seen = set()
    configs = []
    for r in rows:
        cfg = r["config"]
        if cfg not in seen:
            seen.add(cfg)
            configs.append(cfg)
    configs.sort(key=lambda c: (_extract_distance_um(c), _extract_direction(c)))
    return configs


# ---------------------------------------------------------------------------
# Plot 1 — Total cycle histogram
# ---------------------------------------------------------------------------
def plot_total_histogram(rows: list[dict], out_dir: Path, fmt: str, dpi: int,
                         interactive: bool = False):
    total_us = [r["total_us"] for r in rows if not np.isnan(r["total_us"])]
    if not total_us:
        print("  skip — no valid data")
        return

    fig, ax = plt.subplots(figsize=(10, 5))
    bins = np.linspace(0, np.percentile(total_us, 99.5), 80)
    ax.hist(total_us, bins=bins, color="steelblue", edgecolor="white", alpha=0.85)
    ax.axvline(np.median(total_us), color="crimson", linestyle="--", linewidth=1.5,
               label=f"median = {np.median(total_us):.0f} µs")
    ax.set_xlabel("total_us (µs)")
    ax.set_ylabel("frequency")
    ax.set_title("1 — Total Cycle Time Distribution (all trials pooled)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / f"plot1_total_histogram.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot1_total_histogram.{fmt}")


# ---------------------------------------------------------------------------
# Plot 2 — Box plot by distance
# ---------------------------------------------------------------------------
def plot_box_by_distance(rows: list[dict], out_dir: Path, fmt: str, dpi: int,
                         interactive: bool = False):
    # Group by distance (average pos/neg together for cleaner view)
    dist_map: dict[int, list[float]] = defaultdict(list)
    for r in rows:
        if not np.isnan(r["total_us"]):
            dist_map[_extract_distance_um(r["config"])].append(r["total_us"] / 1000.0)

    dists = sorted(dist_map.keys())
    data = [dist_map[d] for d in dists]

    fig, ax = plt.subplots(figsize=(10, 5))
    bp = ax.boxplot(data, labels=[f"{d} µm" for d in dists], patch_artist=True,
                    widths=0.5)
    for patch, d in zip(bp["boxes"], dists):
        patch.set_facecolor(plt.cm.viridis(d / max(dists) * 0.8))

    ax.set_xlabel("move distance")
    ax.set_ylabel("total time (ms)")
    ax.set_title("2 — Total Cycle Time vs Move Distance")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / f"plot2_box_by_distance.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot2_box_by_distance.{fmt}")


# ---------------------------------------------------------------------------
# Plot 3 — Reaction time histogram
# ---------------------------------------------------------------------------
def plot_reaction_histogram(rows: list[dict], out_dir: Path, fmt: str, dpi: int,
                            interactive: bool = False):
    reaction_us = [r["reaction_us"] for r in rows if not np.isnan(r["reaction_us"])]
    if not reaction_us:
        print("  skip — no valid data")
        return

    fig, ax = plt.subplots(figsize=(10, 5))
    bins = np.linspace(min(reaction_us), np.percentile(reaction_us, 99.5), 60)
    ax.hist(reaction_us, bins=bins, color="darkorange", edgecolor="white", alpha=0.85)
    ax.axvline(np.median(reaction_us), color="black", linestyle="--", linewidth=1.5,
               label=f"median = {np.median(reaction_us):.0f} µs")
    ax.set_xlabel("reaction_us (µs)")
    ax.set_ylabel("frequency")
    ax.set_title("3 — Reaction Time Distribution (all trials pooled)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / f"plot3_reaction_histogram.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot3_reaction_histogram.{fmt}")


# ---------------------------------------------------------------------------
# Plot 4 — Stacked breakdown bar
# ---------------------------------------------------------------------------
def plot_stacked_breakdown(rows: list[dict], out_dir: Path, fmt: str, dpi: int,
                           interactive: bool = False):
    configs = _sorted_configs(rows)

    # Group by config, compute mean of each phase (in ms)
    labels = []
    reaction_ms, motion_ms, settle_ms = [], [], []
    for cfg in configs:
        cfg_rows = [r for r in rows if r["config"] == cfg
                    and not np.isnan(r["reaction_us"])
                    and not np.isnan(r["motion_us"])
                    and not np.isnan(r["settle_us"])]
        if not cfg_rows:
            continue
        labels.append(f"{_extract_distance_um(cfg)}µm\n{_extract_direction(cfg)}")
        reaction_ms.append(np.mean([r["reaction_us"] for r in cfg_rows]) / 1000.0)
        motion_ms.append(np.mean([r["motion_us"] for r in cfg_rows]) / 1000.0)
        settle_ms.append(np.mean([r["settle_us"] for r in cfg_rows]) / 1000.0)

    x = np.arange(len(labels))
    width = 0.6

    fig, ax = plt.subplots(figsize=(14, 6))
    p1 = ax.bar(x, reaction_ms, width, label="reaction", color="#66c2a5")
    p2 = ax.bar(x, motion_ms, width, bottom=reaction_ms, label="motion", color="#fc8d62")
    bottom2 = [a + b for a, b in zip(reaction_ms, motion_ms)]
    p3 = ax.bar(x, settle_ms, width, bottom=bottom2, label="settle", color="#8da0cb")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("mean time (ms)")
    ax.set_title("4 — Mean Phase Breakdown by Configuration")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / f"plot4_stacked_breakdown.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot4_stacked_breakdown.{fmt}")


# ---------------------------------------------------------------------------
# Plot 5 — Overlaid position traces
# ---------------------------------------------------------------------------
def plot_position_traces(traces: dict, out_dir: Path, fmt: str, dpi: int,
                         interactive: bool = False):
    """Overlaid position traces — one subplot per distance."""
    by_distance: dict[int, list] = defaultdict(list)
    for cfg_name, trial_list in traces.items():
        dist = _extract_distance_um(cfg_name)
        by_distance[dist].extend(trial_list)

    dists = sorted(by_distance.keys())
    ncols = min(3, len(dists))
    nrows = int(np.ceil(len(dists) / ncols))

    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 4 * nrows),
                             squeeze=False)
    fig.suptitle("5 — Overlaid Position Traces by Distance", fontsize=14)

    for idx, dist in enumerate(dists):
        ax = axes[idx // ncols][idx % ncols]
        trial_list = by_distance[dist]
        max_traces = min(100, len(trial_list))
        for tid, trace in trial_list[:max_traces]:
            if not trace:
                continue
            t_arr = [t for t, _ in trace]
            p0 = trace[0][1] if trace else 0
            p_arr = [p - p0 for _, p in trace]
            ax.plot(t_arr, p_arr, linewidth=0.3, alpha=0.5, color="steelblue")
        ax.set_title(f"{dist} µm")
        ax.set_xlabel("time (ms)")
        ax.set_ylabel("position (µm, rel. to start)")

    for idx in range(len(dists), nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)

    fig.tight_layout()
    fig.savefig(out_dir / f"plot5_position_traces.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot5_position_traces.{fmt}")


def plot_position_traces_individual(traces: dict, out_dir: Path, fmt: str,
                                    dpi: int, interactive: bool = False):
    """One figure per distance — each pops up in its own navigable window."""
    by_distance: dict[int, list] = defaultdict(list)
    for cfg_name, trial_list in traces.items():
        dist = _extract_distance_um(cfg_name)
        by_distance[dist].extend(trial_list)

    for dist in sorted(by_distance.keys()):
        trial_list = by_distance[dist]
        fig, ax = plt.subplots(figsize=(12, 6))
        fig.suptitle(f"Position Traces — {dist} µm (all trials)", fontsize=13)
        max_traces = min(100, len(trial_list))
        for tid, trace in trial_list[:max_traces]:
            if not trace:
                continue
            t_arr = [t for t, _ in trace]
            p0 = trace[0][1] if trace else 0
            p_arr = [p - p0 for _, p in trace]
            ax.plot(t_arr, p_arr, linewidth=0.3, alpha=0.5, color="steelblue")
        ax.set_xlabel("time (ms)")
        ax.set_ylabel("position (µm, rel. to start)")
        fig.tight_layout()
        fig.savefig(out_dir / f"plot5_traces_{dist:03d}um.{fmt}", dpi=dpi)
        if interactive:
            plt.show(block=False)
        else:
            plt.close(fig)
        print(f"  wrote plot5_traces_{dist:03d}um.{fmt}")


# ---------------------------------------------------------------------------
# Plot 6 — CDF (cumulative distribution)
# ---------------------------------------------------------------------------
def plot_cdf(rows: list[dict], out_dir: Path, fmt: str, dpi: int,
             interactive: bool = False):
    # Group by distance, plot CDF per distance
    by_distance: dict[int, list[float]] = defaultdict(list)
    for r in rows:
        if not np.isnan(r["total_us"]):
            by_distance[_extract_distance_um(r["config"])].append(r["total_us"] / 1000.0)

    dists = sorted(by_distance.keys())

    fig, ax = plt.subplots(figsize=(10, 6))
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(dists)))
    for dist, color in zip(dists, colors):
        values = sorted(by_distance[dist])
        n = len(values)
        y = np.arange(1, n + 1) / n * 100
        ax.plot(values, y, color=color, linewidth=1.5, label=f"{dist} µm")
        # Mark p95
        p95 = np.percentile(values, 95)
        ax.axvline(p95, color=color, linestyle=":", alpha=0.4)

    ax.set_xlabel("total time (ms)")
    ax.set_ylabel("cumulative %")
    ax.set_title("6 — Cumulative Distribution of Total Cycle Time")
    ax.legend(title="move distance")
    ax.grid(alpha=0.2)
    ax.set_xlim(left=0)
    ax.set_ylim(0, 105)
    fig.tight_layout()
    fig.savefig(out_dir / f"plot6_cdf.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot6_cdf.{fmt}")


# ---------------------------------------------------------------------------
# Plot 7 — Engage vs settle scatter
# ---------------------------------------------------------------------------
def plot_engage_vs_settle(rows: list[dict], out_dir: Path, fmt: str, dpi: int,
                          interactive: bool = False):
    by_distance: dict[int, tuple[list[float], list[float]]] = defaultdict(lambda: ([], []))
    for r in rows:
        if not np.isnan(r["motion_us"]) and not np.isnan(r["settle_us"]):
            dist = _extract_distance_um(r["config"])
            by_distance[dist][0].append(r["motion_us"] / 1000.0)
            by_distance[dist][1].append(r["settle_us"] / 1000.0)

    dists = sorted(by_distance.keys())

    fig, ax = plt.subplots(figsize=(10, 7))
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(dists)))
    for dist, color in zip(dists, colors):
        x_vals, y_vals = by_distance[dist]
        ax.scatter(x_vals, y_vals, s=2, alpha=0.4, color=color, label=f"{dist} µm")

    ax.set_xlabel("engage time = motion_us (ms)")
    ax.set_ylabel("settle time = settle_us (ms)")
    ax.set_title("7 — Engage Time vs Settle Time")
    ax.legend(title="move distance", markerscale=5)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(out_dir / f"plot7_engage_vs_settle.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot7_engage_vs_settle.{fmt}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Plot latency benchmark results from dof_latency_bench.py output.")
    ap.add_argument("bench_dir", type=Path,
                    help="path to benchmark output directory "
                         "(contains trial_summary.csv, raw_trace_*.csv)")
    ap.add_argument("--format", default="png", choices=["png", "svg", "pdf"],
                    help="output image format (default: png)")
    ap.add_argument("--dpi", type=int, default=150,
                    help="output resolution (default: 150)")
    ap.add_argument("--plots", default="1,2,3,4,5,6,7",
                    help="comma-separated plot numbers to generate (default: all)")
    ap.add_argument("--open", action="store_true",
                    help="open the plots directory in the file manager when done")
    ap.add_argument("--interactive", action="store_true",
                    help="show each plot in a navigable window (zoom/pan)")
    args = ap.parse_args()

    bench_dir = args.bench_dir.resolve()
    if not bench_dir.is_dir():
        print(f"[ERROR] {bench_dir} is not a directory")
        sys.exit(1)

    # Create plots subdirectory
    plots_dir = bench_dir / "plots"
    plots_dir.mkdir(exist_ok=True)

    interactive = args.interactive
    if interactive:
        # Use interactive backend — windows will pop up for zoom/pan
        matplotlib.use("TkAgg")
    else:
        matplotlib.use("Agg")  # non-interactive backend
    plt.rcParams.update({"font.size": 11})

    fmt = args.format
    dpi = args.dpi
    plot_nums = set(int(s.strip()) for s in args.plots.split(","))

    print(f"[plot] reading data from {bench_dir}")
    rows = load_trial_summary(bench_dir)
    traces = load_raw_traces(bench_dir)

    print(f"[plot] generating plots → {plots_dir}/")

    if 1 in plot_nums:
        print("[plot 1/7] total cycle histogram...")
        plot_total_histogram(rows, plots_dir, fmt, dpi, interactive)
    if 2 in plot_nums:
        print("[plot 2/7] box plot by distance...")
        plot_box_by_distance(rows, plots_dir, fmt, dpi, interactive)
    if 3 in plot_nums:
        print("[plot 3/7] reaction time histogram...")
        plot_reaction_histogram(rows, plots_dir, fmt, dpi, interactive)
    if 4 in plot_nums:
        print("[plot 4/7] stacked breakdown bar...")
        plot_stacked_breakdown(rows, plots_dir, fmt, dpi, interactive)
    if 5 in plot_nums:
        print("[plot 5/7] overlaid position traces...")
        if traces:
            plot_position_traces(traces, plots_dir, fmt, dpi, interactive)
            plot_position_traces_individual(traces, plots_dir, fmt, dpi, interactive)
        else:
            print("  skip — no raw trace files found")
    if 6 in plot_nums:
        print("[plot 6/7] CDF...")
        plot_cdf(rows, plots_dir, fmt, dpi, interactive)
    if 7 in plot_nums:
        print("[plot 7/7] engage vs settle scatter...")
        plot_engage_vs_settle(rows, plots_dir, fmt, dpi, interactive)

    if interactive:
        print("[plot] all figures open — close windows or press Ctrl+C to exit")
        try:
            plt.show(block=True)
        except KeyboardInterrupt:
            pass

    print(f"[plot] done — {len(plot_nums)} plots in {plots_dir}/")

    if args.open:
        try:
            subprocess.run(["xdg-open", str(plots_dir)], check=False)
        except FileNotFoundError:
            print("[plot] (could not open file manager — "
                  "xdg-open not available)")


if __name__ == "__main__":
    main()
