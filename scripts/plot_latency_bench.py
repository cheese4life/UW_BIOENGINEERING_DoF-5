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

# Cap the number of overlaid position traces per subplot to keep matplotlib
# rendering time sane for large datasets (2600-trial runs).
DEFAULT_MAX_TRACES_PER_PLOT = 50
# Cap the number of (trial, point) pairs we ever feed to a single line
# collection. Traces for long moves (4mm @ 125mm/s = ~32ms motion) can have
# hundreds of points each; downsample if a trace exceeds this.
MAX_POINTS_PER_TRACE = 400

# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def load_trial_summary(bench_dir: Path) -> list[dict]:
    """Parse trial_summary.csv → list of dicts.

    Robust to the schema change: old runs don't have the payload-
    characterization columns; we default any missing column to NaN/None
    so old and new bench data both load cleanly.
    """
    rows = []
    path = bench_dir / "trial_summary.csv"
    if not path.exists():
        print(f"[ERROR] {path} not found")
        sys.exit(1)
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # int fields
            for key in ("trial_id", "direction", "target_counts", "home_counts",
                        "final_pos_counts", "final_error_counts",
                        "peak_motor_cmd", "peak_following_error_counts"):
                val = row.get(key, "")
                row[key] = int(val) if val and val != "" else 0
            # Optional int fields (may not exist in old CSVs)
            for key in ("t_cmd_ns", "t_react_ns", "t_engage_ns", "t_complete_ns"):
                val = row.get(key, "")
                row[key] = int(val) if val and val != "" else None
            # Float fields — both old latency + new payload characterization
            float_keys = (
                "reaction_us", "motion_us", "settle_us", "total_us",
                "peak_velocity_mm_s", "peak_velocity_t_frac",
                "velocity_at_25pct_mm_s", "velocity_at_50pct_mm_s",
                "velocity_at_75pct_mm_s", "peak_motor_cmd_t_frac",
                "mean_following_error_counts",
                "calc_peak_velocity_mm_s", "calc_motion_us",
                "avg_accel_mm_s2", "avg_decel_mm_s2",
            )
            for key in float_keys:
                val = row.get(key, "")
                try:
                    row[key] = float(val) if val and val != "" else float("nan")
                except ValueError:
                    row[key] = float("nan")
            rows.append(row)
    print(f"[load] {len(rows)} trials from trial_summary.csv")
    return rows


def load_raw_traces(bench_dir: Path) -> dict[str, list[tuple[int, list[tuple]]]]:
    """Parse raw_trace_*.csv → {config_name: [(trial_id, samples), ...]}.

    Each sample is a tuple (t_ms, pos_um, vel_mm_s, cmd_um, motor_cmd).
    Old traces (2-column) load with NaN vel/cmd/motor; new traces
    (5-column) populate the multi-channel fields. Units are converted on
    load (ns → ms, counts → µm, counts → µm for commanded too).
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
                t_ms = float(row["t_ns_since_cmd"]) / 1e6
                pos_um = float(row["position_counts"]) / 200.0
                # New multi-channel columns; default NaN for old traces
                vel_str = row.get("velocity_mm_s", "")
                cmd_str = row.get("commanded_counts", "")
                mot_str = row.get("motor_cmd", "")
                vel = float(vel_str) if vel_str and vel_str != "" else float("nan")
                cmd_um = float(cmd_str) / 200.0 if cmd_str and cmd_str != "" else float("nan")
                mot = int(mot_str) if mot_str and mot_str != "" else 0
                if current_trial is None:
                    current_trial = tid
                if tid != current_trial:
                    traces[config_name].append((current_trial, current_trace))
                    current_trial = tid
                    current_trace = []
                current_trace.append((t_ms, pos_um, vel, cmd_um, mot))
            if current_trace:
                traces[config_name].append((current_trial, current_trace))
        print(f"[load] {len(traces[config_name])} traces from {path.name}")
    return dict(traces)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _extract_distance_um(config_name: str) -> int:
    """'010um_pos' → 10, '1000um_pos' → 1000, '4000um_neg' → 4000.

    Parses the full numeric prefix before 'um' — NOT just the first 3
    chars (which silently mangled 4-digit distances: '4000um' → 400).
    """
    return int(config_name.split("um")[0])


def _fmt_distance(um: int) -> str:
    """Human-readable distance label: 10→'10µm', 500→'500µm',
    1000→'1mm', 1500→'1.5mm', 4000→'4mm'."""
    if um >= 1000:
        return f"{um / 1000:g}mm"
    return f"{um}µm"


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


def _downsample_trace(trace, max_points=MAX_POINTS_PER_TRACE):
    """Decimate a trace [(t, pos), ...] to ≤ max_points, preserving first
    and last samples. Used to keep matplotlib rendering fast on long
    traces from 4mm moves (which can have 200+ samples each)."""
    if len(trace) <= max_points:
        return trace
    step = max(1, len(trace) // max_points)
    out = trace[::step]
    # always include the last point so traces end at the right place
    if out[-1] is not trace[-1]:
        out = out + (trace[-1],)
    return out


def _percentile(values, pct):
    """Simple percentile of an unsorted list. Returns NaN if empty."""
    if not values:
        return float("nan")
    s = sorted(values)
    n = len(s)
    k = max(0, min(n - 1, int(round(pct / 100.0 * (n - 1)))))
    return s[k]


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

    # Wider figure + rotated labels for when there are many distances
    # (13 distances is too many to fit horizontally otherwise)
    n_dist = len(dists)
    fig_w = max(10, 0.85 * n_dist + 4)
    fig, ax = plt.subplots(figsize=(fig_w, 6))
    # 'tick_labels' is the matplotlib ≥3.9 name; older versions use 'labels'.
    try:
        bp = ax.boxplot(data, tick_labels=[f"{d}" for d in dists],
                        patch_artist=True, widths=0.55, showfliers=False)
    except TypeError:
        bp = ax.boxplot(data, labels=[f"{d}" for d in dists],
                        patch_artist=True, widths=0.55, showfliers=False)
    for patch, d in zip(bp["boxes"], dists):
        patch.set_facecolor(plt.cm.viridis(np.log10(d + 1) / np.log10(max(dists) + 1) * 0.85))

    ax.set_xlabel("move distance (µm)")
    ax.set_ylabel("total time (ms)")
    ax.set_title("2 — Total Cycle Time vs Move Distance "
                 "(pos & neg pooled; whiskers = 1.5×IQR, outliers hidden)")
    ax.grid(axis="y", alpha=0.3)
    # Annotate sample sizes under each box so it's obvious if a config
    # had data loss (e.g. small-distance events missed at max speed).
    for i, d in enumerate(dists):
        ax.text(i + 1, ax.get_ylim()[0], f"n={len(data[i])}",
                ha="center", va="top", fontsize=7, color="gray")
    fig.tight_layout()
    fig.savefig(out_dir / f"plot2_box_by_distance.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot2_box_by_distance.{fmt}  ({n_dist} distances)")


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
# Plot 4 — Stacked breakdown with velocity/force overlays (3-panel)
# ---------------------------------------------------------------------------
def plot_stacked_breakdown(rows: list[dict], out_dir: Path, fmt: str, dpi: int,
                           interactive: bool = False):
    """Three-panel figure showing phase breakdown + peak velocity + peak
    motor command for every config. All three share the same x-axis so
    you can read across to see how the three quantities correlate with
    each other and with move distance.

    Top:    react/motion/settle stacked bars (ms)
    Middle: peak velocity per config (mm/s); measured vs calculated
            trapezoidal reference overlaid as a dashed line/scatter
    Bottom: peak motor command per config (voice-coil force proxy, raw
            register units) — the mechanical stress seen by the payload
    """
    configs = _sorted_configs(rows)

    labels = []
    reaction_ms, motion_ms, settle_ms = [], [], []
    peak_v_meas, peak_v_calc = [], []
    peak_motor = []
    peak_following_err = []
    n_per_cfg = []
    for cfg in configs:
        cfg_rows = [r for r in rows if r["config"] == cfg]
        if not cfg_rows:
            continue
        r_vals = [r["reaction_us"] for r in cfg_rows if not np.isnan(r["reaction_us"])]
        m_vals = [r["motion_us"] for r in cfg_rows if not np.isnan(r["motion_us"])]
        s_vals = [r["settle_us"] for r in cfg_rows if not np.isnan(r["settle_us"])]
        if not r_vals and not m_vals and not s_vals:
            continue
        labels.append(f"{_fmt_distance(_extract_distance_um(cfg))}\n{_extract_direction(cfg)}")
        reaction_ms.append(np.mean(r_vals) / 1000.0 if r_vals else 0.0)
        motion_ms.append(np.mean(m_vals) / 1000.0 if m_vals else 0.0)
        settle_ms.append(np.mean(s_vals) / 1000.0 if s_vals else 0.0)
        # Peak velocity: take median of per-trial peaks (more robust than max)
        pv = [r.get("peak_velocity_mm_s", float("nan")) for r in cfg_rows]
        pv = [v for v in pv if not np.isnan(v)]
        peak_v_meas.append(np.median(pv) if pv else float("nan"))
        pvc = [r.get("calc_peak_velocity_mm_s", float("nan")) for r in cfg_rows]
        pvc = [v for v in pvc if not np.isnan(v)]
        peak_v_calc.append(np.median(pvc) if pvc else float("nan"))
        # Peak motor command: median across trials
        pm = [float(r.get("peak_motor_cmd", 0)) for r in cfg_rows]
        pm = [v for v in pm if v > 0]
        peak_motor.append(np.median(pm) if pm else 0.0)
        # Peak following error
        pfe = [float(r.get("peak_following_error_counts", 0)) for r in cfg_rows]
        pfe = [v for v in pfe if v > 0]
        peak_following_err.append(np.median(pfe) if pfe else 0.0)
        n_per_cfg.append(min(len(r_vals), len(m_vals), len(s_vals)))

    x = np.arange(len(labels))
    width = 0.6
    fig_w = max(14, 0.5 * len(labels) + 6)
    # 3 stacked subplots sharing x-axis
    fig, (ax1, ax2, ax3) = plt.subplots(
        3, 1, figsize=(fig_w, 13), sharex=True,
        gridspec_kw={"height_ratios": [3, 2, 2]})
    fig.suptitle("4 — Phase breakdown, peak velocity, and peak force by config",
                 fontsize=13, y=0.995)

    # --- Top panel: latency stacks (the original plot4 content) ---
    ax1.bar(x, reaction_ms, width, label="reaction", color="#66c2a5")
    ax1.bar(x, motion_ms, width, bottom=reaction_ms,
            label="motion", color="#fc8d62")
    bottom2 = [a + b for a, b in zip(reaction_ms, motion_ms)]
    ax1.bar(x, settle_ms, width, bottom=bottom2,
            label="settle", color="#8da0cb")
    ax1.set_ylabel("mean time (ms)")
    ax1.set_title(f"Latency phase breakdown "
                  f"(n={n_per_cfg[0] if n_per_cfg else 0}-"
                  f"{max(n_per_cfg) if n_per_cfg else 0} per config)",
                  fontsize=10)
    ax1.legend(loc="upper left", fontsize=8)
    ax1.grid(axis="y", alpha=0.3)

    # --- Middle panel: peak velocity, measured vs calculated ---
    ax2.plot(x, peak_v_meas, "o-", color="#1f77b4", linewidth=1.6,
             markersize=6, label="measured peak v (median)", zorder=3)
    ax2.plot(x, peak_v_calc, "s--", color="#d62728", linewidth=1.2,
             markersize=5, alpha=0.7, label="calculated trapezoidal peak v",
             zorder=2)
    ax2.set_ylabel("peak velocity (mm/s)")
    ax2.set_title("Peak velocity: measured vs ideal trapezoid",
                  fontsize=10)
    ax2.legend(loc="lower right", fontsize=8)
    ax2.grid(axis="y", alpha=0.3)
    # Mark the v_max cap (125 mm/s typical)
    valid_calc = [v for v in peak_v_calc if not np.isnan(v)]
    if valid_calc:
        vmax = max(valid_calc)
        ax2.axhline(vmax, color="gray", linestyle=":", alpha=0.5)
        ax2.text(len(x) - 0.5, vmax, f" v_max={vmax:.0f}",
                 fontsize=7, color="gray", va="bottom", ha="right")

    # --- Bottom panel: peak motor command (force on payload) ---
    ax3.bar(x, peak_motor, width, color="#9467bd", alpha=0.85)
    ax3.set_ylabel("peak motor cmd (force reg)")
    ax3.set_title("Peak voice-coil force during motion "
                  "(mechanical stress on payload)", fontsize=10)
    ax3.grid(axis="y", alpha=0.3)
    # Annotate following error on the force plot as a thin line — high
    # following error often correlates with high force demand and is the
    # most direct signal that the servo is struggling.
    ax3b = ax3.twinx()
    ax3b.plot(x, peak_following_err, "^-", color="#2ca02c", linewidth=1,
              markersize=4, alpha=0.6, label="peak following error (cts)")
    ax3b.set_ylabel("peak following error (counts)", color="#2ca02c")
    ax3b.tick_params(axis="y", labelcolor="#2ca02c")

    ax3.set_xticks(x)
    ax3.set_xticklabels(labels, fontsize=7, rotation=45, ha="right")
    fig.tight_layout(rect=(0, 0, 1, 0.985))
    fig.savefig(out_dir / f"plot4_stacked_breakdown.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot4_stacked_breakdown.{fmt}  "
          f"({len(labels)} configs, 3-panel: latency+v+force)")


# ---------------------------------------------------------------------------
# Plot 5 — Overlaid position traces
# ---------------------------------------------------------------------------
def plot_position_traces(traces: dict, out_dir: Path, fmt: str, dpi: int,
                         interactive: bool = False):
    """Overlaid position traces — one subplot per distance.

    For large datasets (13 distances × 100 trials), we cap to
    DEFAULT_MAX_TRACES_PER_PLOT per subplot and downsample long traces.
    """
    by_distance: dict[int, list] = defaultdict(list)
    for cfg_name, trial_list in traces.items():
        dist = _extract_distance_um(cfg_name)
        by_distance[dist].extend(trial_list)

    dists = sorted(by_distance.keys())
    ncols = min(3, len(dists))
    nrows = int(np.ceil(len(dists) / ncols))

    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 3.5 * nrows),
                             squeeze=False)
    fig.suptitle("5 — Overlaid Position Traces by Distance", fontsize=14)

    max_traces = min(DEFAULT_MAX_TRACES_PER_PLOT,
                     max((len(v) for v in by_distance.values()), default=0))

    for idx, dist in enumerate(dists):
        ax = axes[idx // ncols][idx % ncols]
        trial_list = by_distance[dist]
        # Subsample trial selection if there are more than max_traces
        if len(trial_list) > max_traces:
            sel = np.linspace(0, len(trial_list) - 1, max_traces, dtype=int)
            trial_subset = [trial_list[i] for i in sel]
        else:
            trial_subset = trial_list
        for tid, trace in trial_subset:
            if not trace:
                continue
            trace = _downsample_trace(trace)
            t_arr = [t for t, _ in trace]
            p0 = trace[0][1] if trace else 0
            p_arr = [p - p0 for _, p in trace]
            ax.plot(t_arr, p_arr, linewidth=0.3, alpha=0.5, color="steelblue")
        ax.set_title(f"{dist} µm  (n={len(trial_list)}, shown {len(trial_subset)})")
        ax.set_xlabel("time (ms)")
        ax.set_ylabel("position (µm, rel. to start)")
        ax.grid(alpha=0.2)

    for idx in range(len(dists), nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)

    fig.tight_layout()
    fig.savefig(out_dir / f"plot5_position_traces.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot5_position_traces.{fmt}  "
          f"({max_traces} traces/subplot max)")


def plot_position_traces_individual(traces: dict, out_dir: Path, fmt: str,
                                    dpi: int, interactive: bool = False):
    """One figure per distance — each pops up in its own navigable window.

    For datasets with many distances this produces one PNG per distance
    (up to 13 files for our default 10µm–4mm matrix). To keep individual
    files readable and fast, we again cap traces per plot and downsample.
    """
    by_distance: dict[int, list] = defaultdict(list)
    for cfg_name, trial_list in traces.items():
        dist = _extract_distance_um(cfg_name)
        by_distance[dist].extend(trial_list)

    for dist in sorted(by_distance.keys()):
        trial_list = by_distance[dist]
        fig, ax = plt.subplots(figsize=(12, 6))
        fig.suptitle(f"Position Traces — {dist} µm "
                     f"(n={len(trial_list)})", fontsize=13)
        max_traces = min(DEFAULT_MAX_TRACES_PER_PLOT, len(trial_list))
        # Subsample trial selection if there are more than max_traces
        if len(trial_list) > max_traces:
            sel = np.linspace(0, len(trial_list) - 1, max_traces, dtype=int)
            trial_subset = [trial_list[i] for i in sel]
        else:
            trial_subset = trial_list
        for tid, trace in trial_subset:
            if not trace:
                continue
            trace = _downsample_trace(trace)
            t_arr = [t for t, _ in trace]
            p0 = trace[0][1] if trace else 0
            p_arr = [p - p0 for _, p in trace]
            ax.plot(t_arr, p_arr, linewidth=0.3, alpha=0.5, color="steelblue")
        ax.set_xlabel("time (ms)")
        ax.set_ylabel("position (µm, rel. to start)")
        ax.grid(alpha=0.2)
        fig.tight_layout()
        fig.savefig(out_dir / f"plot5_traces_{dist:04d}um.{fmt}", dpi=dpi)
        if interactive:
            plt.show(block=False)
        else:
            plt.close(fig)
        print(f"  wrote plot5_traces_{dist:04d}um.{fmt}  "
              f"({len(trial_subset)}/{len(trial_list)} traces)")


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

    fig, ax = plt.subplots(figsize=(11, 7))
    colors = plt.cm.viridis(np.linspace(0, 0.9, max(len(dists), 1)))
    for dist, color in zip(dists, colors):
        values = sorted(by_distance[dist])
        n = len(values)
        y = np.arange(1, n + 1) / n * 100
        ax.plot(values, y, color=color, linewidth=1.5, label=f"{dist} µm")
        # Mark p95
        if n > 0:
            p95 = np.percentile(values, 95)
            ax.axvline(p95, color=color, linestyle=":", alpha=0.4)

    ax.set_xlabel("total time (ms)")
    ax.set_ylabel("cumulative %")
    ax.set_title("6 — Cumulative Distribution of Total Cycle Time")
    # Many-distance datasets produce a cluttered legend; use 2 columns.
    ncol = 2 if len(dists) > 7 else 1
    ax.legend(title="move distance", ncol=ncol, fontsize=8)
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
    colors = plt.cm.viridis(np.linspace(0, 0.9, max(len(dists), 1)))
    for dist, color in zip(dists, colors):
        x_vals, y_vals = by_distance[dist]
        ax.scatter(x_vals, y_vals, s=2, alpha=0.4, color=color, label=f"{dist} µm")

    ax.set_xlabel("engage time = motion_us (ms)")
    ax.set_ylabel("settle time = settle_us (ms)")
    ax.set_title("7 — Engage Time vs Settle Time")
    ncol = 2 if len(dists) > 7 else 1
    ax.legend(title="move distance", markerscale=5, ncol=ncol, fontsize=8)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(out_dir / f"plot7_engage_vs_settle.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot7_engage_vs_settle.{fmt}")


# ---------------------------------------------------------------------------
# Plot 8 — Latency vs distance (log-log scaling curve)
# ---------------------------------------------------------------------------
def plot_latency_vs_distance(rows: list[dict], out_dir: Path, fmt: str, dpi: int,
                             interactive: bool = False):
    """Median + p95 latency vs distance, log-log axes.

    This is the headline plot for "how does stage latency scale with move
    size?" — exactly the question the expanded distance matrix (10µm to 4mm)
    is designed to answer. The trapezoidal-motion regime shows up here as a
    power-law region; below the polling-resolution limit (10–25µm at
    v=125mm/s) points flatten out because reaction/motion events can't be
    detected.
    """
    # Per-distance median/p95 of total, reaction, motion, settle
    by_dist: dict[int, dict[str, list[float]]] = defaultdict(
        lambda: {"total": [], "reaction": [], "motion": [], "settle": []})
    for r in rows:
        if np.isnan(r["total_us"]):
            continue
        d = _extract_distance_um(r["config"])
        by_dist[d]["total"].append(r["total_us"] / 1000.0)
        if not np.isnan(r["reaction_us"]):
            by_dist[d]["reaction"].append(r["reaction_us"] / 1000.0)
        if not np.isnan(r["motion_us"]):
            by_dist[d]["motion"].append(r["motion_us"] / 1000.0)
        if not np.isnan(r["settle_us"]):
            by_dist[d]["settle"].append(r["settle_us"] / 1000.0)

    dists = sorted(by_dist.keys())
    if not dists:
        print("  skip — no valid data")
        return

    def meds(field):
        return [np.median(by_dist[d][field]) if by_dist[d][field] else float("nan")
                for d in dists]
    total_med = meds("total")
    total_p95 = [_percentile(by_dist[d]["total"], 95) for d in dists]
    motion_med = meds("motion")
    settle_med = meds("settle")
    reaction_med = meds("reaction")

    fig, ax = plt.subplots(figsize=(11, 7))
    ax.loglog(dists, total_med, "o-", color="#1f77b4", linewidth=2,
              markersize=7, label="total (median)")
    ax.loglog(dists, total_p95, ":", color="#1f77b4", linewidth=1.5,
              label="total (p95)")
    ax.loglog(dists, motion_med, "s-", color="#ff7f0e", linewidth=1.5,
              markersize=6, label="motion (median)")
    ax.loglog(dists, settle_med, "^-", color="#2ca02c", linewidth=1.5,
              markersize=6, label="settle (median)")
    ax.loglog(dists, reaction_med, "d-", color="#d62728", linewidth=1.2,
              markersize=5, label="reaction (median)")

    ax.set_xlabel("move distance (µm)")
    ax.set_ylabel("time (ms)")
    ax.set_title("8 — Latency vs Move Distance (log-log)")
    ax.grid(which="both", alpha=0.3)
    ax.legend(loc="best")
    # Annotate the polling-resolution limit so users can see why small-move
    # data flattens out at high speed.
    ax.axvspan(0, 30, alpha=0.08, color="gray")
    ax.text(15, ax.get_ylim()[1] * 0.5,
            "← below CAN poll\n   resolution",
            fontsize=8, color="gray", ha="center", va="top")
    fig.tight_layout()
    fig.savefig(out_dir / f"plot8_latency_vs_distance.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot8_latency_vs_distance.{fmt}  "
          f"({len(dists)} distances, log-log)")


# ---------------------------------------------------------------------------
# Plot 9 — Direction comparison (pos vs neg) per distance
# ---------------------------------------------------------------------------
def plot_direction_comparison(rows: list[dict], out_dir: Path, fmt: str, dpi: int,
                              interactive: bool = False):
    """Compare median latency for positive vs negative moves at each distance.

    Useful to detect direction asymmetry — e.g. if the stage's positive
    end is mechanically constrained differently than the negative end,
    latency could differ between pos and neg moves of the same magnitude.
    With the asymmetric soft-limit setup (start at +1.5mm for +4mm neg
    moves vs -2.3mm for +4mm pos moves) this plot is also a good sanity
    check that the start position isn't biasing the timing.
    """
    pos_by_dist: dict[int, list[float]] = defaultdict(list)
    neg_by_dist: dict[int, list[float]] = defaultdict(list)
    for r in rows:
        if np.isnan(r["total_us"]):
            continue
        d = _extract_distance_um(r["config"])
        direction = _extract_direction(r["config"])
        if direction == "pos":
            pos_by_dist[d].append(r["total_us"] / 1000.0)
        else:
            neg_by_dist[d].append(r["total_us"] / 1000.0)

    dists = sorted(set(pos_by_dist.keys()) | set(neg_by_dist.keys()))
    if not dists:
        print("  skip — no valid data")
        return

    pos_med = [np.median(pos_by_dist[d]) if pos_by_dist[d] else float("nan")
               for d in dists]
    neg_med = [np.median(neg_by_dist[d]) if neg_by_dist[d] else float("nan")
               for d in dists]

    x = np.arange(len(dists))
    width = 0.38
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.bar(x - width / 2, pos_med, width, label="positive (+)", color="#4c72b0")
    ax.bar(x + width / 2, neg_med, width, label="negative (−)", color="#dd8452")
    ax.set_xticks(x)
    ax.set_xticklabels([str(d) for d in dists])
    ax.set_xlabel("move distance (µm)")
    ax.set_ylabel("median total time (ms)")
    ax.set_title("9 — Direction Comparison: Positive vs Negative Moves")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / f"plot9_direction_comparison.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot9_direction_comparison.{fmt}  ({len(dists)} distances)")


# ---------------------------------------------------------------------------
# Plot 10 — Distance vs Time (linear scaling)
# ---------------------------------------------------------------------------
def plot_distance_vs_time_linear(rows: list[dict], out_dir: Path, fmt: str,
                                  dpi: int, interactive: bool = False):
    """Linear-linear distance vs time showing how latency scales with move size.

    Companion to plot 8 (log-log). The linear view makes it easy to see:
      - The trapezoidal-motion regime (linear region where time ∝ distance
        once the move is long enough to reach cruise velocity)
      - The acceleration/deceleration overhead (flat region at small distances
        where moves never reach cruise velocity)
      - The settle time (roughly constant across distances)
    Pools pos + neg directions together per distance for cleaner trends.
    """
    by_dist: dict[int, dict[str, list[float]]] = defaultdict(
        lambda: {"total": [], "motion": [], "settle": [], "reaction": []})
    for r in rows:
        d = _extract_distance_um(r["config"])
        if not np.isnan(r["total_us"]):
            by_dist[d]["total"].append(r["total_us"] / 1000.0)
        if not np.isnan(r["motion_us"]):
            by_dist[d]["motion"].append(r["motion_us"] / 1000.0)
        if not np.isnan(r["settle_us"]):
            by_dist[d]["settle"].append(r["settle_us"] / 1000.0)
        if not np.isnan(r["reaction_us"]):
            by_dist[d]["reaction"].append(r["reaction_us"] / 1000.0)

    dists = sorted(by_dist.keys())
    if not dists:
        print("  skip — no valid data")
        return

    # Convert µm to mm for the x-axis so 4mm moves are readable
    dists_mm = [d / 1000.0 for d in dists]
    total_med = [np.median(by_dist[d]["total"]) if by_dist[d]["total"] else float("nan")
                 for d in dists]
    motion_med = [np.median(by_dist[d]["motion"]) if by_dist[d]["motion"] else float("nan")
                  for d in dists]
    settle_med = [np.median(by_dist[d]["settle"]) if by_dist[d]["settle"] else float("nan")
                  for d in dists]
    reaction_med = [np.median(by_dist[d]["reaction"]) if by_dist[d]["reaction"] else float("nan")
                    for d in dists]

    fig, ax = plt.subplots(figsize=(11, 7))
    ax.plot(dists_mm, total_med, "o-", color="#1f77b4", linewidth=2,
            markersize=7, label="total (median)", zorder=5)
    ax.plot(dists_mm, motion_med, "s-", color="#ff7f0e", linewidth=1.8,
            markersize=6, label="motion (median)", zorder=4)
    ax.plot(dists_mm, settle_med, "^-", color="#2ca02c", linewidth=1.6,
            markersize=6, label="settle (median)", zorder=3)
    ax.plot(dists_mm, reaction_med, "d-", color="#d62728", linewidth=1.2,
            markersize=5, label="reaction (median)", zorder=2)

    # Annotate each data point with its value for the total line so the
    # actual numbers are visible without estimation.
    for dm, tm in zip(dists_mm, total_med):
        if not np.isnan(tm):
            ax.annotate(f"{tm:.1f}ms", (dm, tm), textcoords="offset points",
                        xytext=(0, 8), ha="center", fontsize=7, color="#1f77b4")

    ax.set_xlabel("move distance (mm)")
    ax.set_ylabel("time (ms)")
    ax.set_title("10 — Distance vs Time (linear) — how latency scales with move size\n"
                 "(pos & neg pooled; medians)")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left")
    ax.set_xlim(left=-0.1)
    ax.set_ylim(bottom=0)
    fig.tight_layout()
    fig.savefig(out_dir / f"plot10_distance_vs_time.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot10_distance_vs_time.{fmt}  ({len(dists)} distances)")


# ---------------------------------------------------------------------------
# Plot 11 — Velocity profile overlay (measured vs calculated trapezoid)
# ---------------------------------------------------------------------------
def plot_velocity_profiles(traces: dict, rows: list[dict], out_dir: Path,
                           fmt: str, dpi: int, interactive: bool = False):
    """Overlaid velocity-vs-time traces, one subplot per distance.

    Each subplot overlays:
      - Real measured velocity (GetActualVelocity, from Juno) — many
        semi-transparent blue lines (one per trial)
      - Calculated ideal trapezoidal velocity profile — single red dashed
        line, derived from the v/a/distance from trial_summary.csv

    This directly answers "is the stage's velocity profile actually
    trapezoidal, and how much does it deviate from ideal?" — a key
    payload-characterization question. Triangular moves (short distances)
    should show a clean triangle; trapezoidal moves (long distances)
    should show a flat-topped plateau at v_max.
    """
    # Group traces by distance (pool pos + neg for the overlay)
    by_dist_traces: dict[int, list] = defaultdict(list)
    for cfg_name, trial_list in traces.items():
        dist = _extract_distance_um(cfg_name)
        by_dist_traces[dist].extend(trial_list)

    # Group trial summary rows by distance for the calculated reference
    by_dist_rows: dict[int, list] = defaultdict(list)
    for r in rows:
        by_dist_rows[_extract_distance_um(r["config"])].append(r)

    dists = sorted(by_dist_traces.keys())
    if not dists:
        print("  skip — no raw trace data")
        return
    ncols = min(3, len(dists))
    nrows = int(np.ceil(len(dists) / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 3.5 * nrows),
                             squeeze=False)
    fig.suptitle("11 — Velocity profiles (measured vs ideal trapezoid)",
                 fontsize=13)

    for idx, dist in enumerate(dists):
        ax = axes[idx // ncols][idx % ncols]
        trial_list = by_dist_traces[dist]
        # Subsample trials for overlay readability
        n_show = min(DEFAULT_MAX_TRACES_PER_PLOT, len(trial_list))
        if len(trial_list) > n_show:
            sel = np.linspace(0, len(trial_list) - 1, n_show, dtype=int)
            trial_subset = [trial_list[i] for i in sel]
        else:
            trial_subset = trial_list

        # Plot measured velocity per trial
        for tid, trace in trial_subset:
            if not trace:
                continue
            # Filter to samples with valid velocity (post-schema-upgrade)
            valid = [(t, v) for (t, _, v, _, _) in trace
                     if not np.isnan(v)]
            if len(valid) < 3:
                continue
            t_arr = [t for t, _ in valid]
            v_arr = [v for _, v in valid]
            ax.plot(t_arr, v_arr, linewidth=0.4, alpha=0.45, color="steelblue")

        # Overlay calculated trapezoidal velocity profile.
        # Take median calc values across trials at this distance.
        cfg_rows = by_dist_rows.get(dist, [])
        calc_pk = [r.get("calc_peak_velocity_mm_s", float("nan"))
                   for r in cfg_rows]
        calc_pk = [v for v in calc_pk if not np.isnan(v)]
        calc_mt = [r.get("calc_motion_us", float("nan")) for r in cfg_rows]
        calc_mt = [v for v in calc_mt if not np.isnan(v)]
        motion_us = [r.get("motion_us", float("nan")) for r in cfg_rows]
        motion_us = [v for v in motion_us if not np.isnan(v)]
        if calc_pk and calc_mt:
            peak_v = np.median(calc_pk)
            t_motion = np.median(calc_mt) / 1000.0  # µs → ms
            t_react_us = np.median([r.get("reaction_us", float("nan"))
                                   for r in cfg_rows
                                   if not np.isnan(r.get("reaction_us", float("nan")))])
            t_start = (t_react_us / 1000.0) if not np.isnan(t_react_us) else 0.0
            # Triangular: linear ramps up to peak at midpoint, down to 0.
            # Trapezoidal: ramp to v_max at t_accel, plateau, ramp down.
            # Either way the calculated curve is symmetric about midpoint.
            t_mid = t_start + t_motion / 2.0
            t_end = t_start + t_motion
            t_calc = [t_start, t_mid, t_end]
            v_calc = [0, peak_v, 0]
            ax.plot(t_calc, v_calc, "r--", linewidth=1.6, alpha=0.9,
                    label="ideal trapezoid")
            ax.legend(loc="upper right", fontsize=7)

        ax.set_title(f"{_fmt_distance(dist)}  (n={len(trial_list)})", fontsize=9)
        ax.set_xlabel("time since cmd (ms)", fontsize=8)
        ax.set_ylabel("velocity (mm/s)", fontsize=8)
        ax.grid(alpha=0.3)
        ax.axhline(0, color="gray", linewidth=0.5)

    # Hide unused subplots
    for idx in range(len(dists), nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_dir / f"plot11_velocity_profiles.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot11_velocity_profiles.{fmt}  ({len(dists)} distances)")


# ---------------------------------------------------------------------------
# Plot 12 — Following error overlay (commanded - actual position)
# ---------------------------------------------------------------------------
def plot_following_error(traces: dict, out_dir: Path, fmt: str, dpi: int,
                         interactive: bool = False):
    """Overlaid following-error-vs-time traces per distance.

    Following error = commanded position − actual position. This is the
    most direct signal of how well the servo keeps up with its own
    trajectory under payload. Sudden spikes during accel/decel indicate
    the payload inertia is loading the controller; sustained positive
    offset during cruise indicates steady-state tracking lag.

    One subplot per distance, multiple semi-transparent trial lines
    overlaid so you can see both the typical shape and the outliers.
    """
    by_distance: dict[int, list] = defaultdict(list)
    for cfg_name, trial_list in traces.items():
        dist = _extract_distance_um(cfg_name)
        by_distance[dist].extend(trial_list)

    dists = sorted(by_distance.keys())
    if not dists:
        print("  skip — no raw trace data")
        return
    ncols = min(3, len(dists))
    nrows = int(np.ceil(len(dists) / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 3.5 * nrows),
                             squeeze=False)
    fig.suptitle("12 — Following error (commanded − actual) per distance",
                 fontsize=13)

    for idx, dist in enumerate(dists):
        ax = axes[idx // ncols][idx % ncols]
        trial_list = by_distance[dist]
        n_show = min(DEFAULT_MAX_TRACES_PER_PLOT, len(trial_list))
        if len(trial_list) > n_show:
            sel = np.linspace(0, len(trial_list) - 1, n_show, dtype=int)
            trial_subset = [trial_list[i] for i in sel]
        else:
            trial_subset = trial_list

        any_plotted = False
        for tid, trace in trial_subset:
            if not trace:
                continue
            # Following error in µm = (cmd_counts - pos_counts)/200
            valid = [(t, (cmd - pos))
                     for (t, pos, _, cmd, _) in trace
                     if not np.isnan(cmd)]
            if len(valid) < 3:
                continue
            t_arr = [t for t, _ in valid]
            err_um = [e * 5.0 for _, e in valid]  # counts → µm (1 count = 5 nm)
            ax.plot(t_arr, err_um, linewidth=0.4, alpha=0.5, color="darkorange")
            any_plotted = True
        if any_plotted:
            ax.axhline(0, color="gray", linewidth=0.7)
        ax.set_title(f"{_fmt_distance(dist)}  (n={len(trial_list)})", fontsize=9)
        ax.set_xlabel("time since cmd (ms)", fontsize=8)
        ax.set_ylabel("following error (µm)", fontsize=8)
        ax.grid(alpha=0.3)

    for idx in range(len(dists), nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_dir / f"plot12_following_error.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot12_following_error.{fmt}  ({len(dists)} distances)")


# ---------------------------------------------------------------------------
# Plot 13 — Motor command (force on payload) profile overlay
# ---------------------------------------------------------------------------
def plot_motor_cmd_profiles(traces: dict, out_dir: Path, fmt: str, dpi: int,
                            interactive: bool = False):
    """Overlaid motor-command-vs-time traces per distance.

    Motor command is the raw register value sent to the voice coil ≈
    force applied to the payload. Sign = direction of force.

    For payload engineering this plot answers:
      - What's the peak force during a move of size X?
      - Is there sharp direction reversal (jerk) at accel→cruise and
        cruise→decel transitions? Those are vibration inputs to optics.
      - Does the force settle to zero cleanly, or does it oscillate
        during settle? Oscillation = ringing in the lens mount.

    One subplot per distance, multiple trials overlaid.
    """
    by_distance: dict[int, list] = defaultdict(list)
    for cfg_name, trial_list in traces.items():
        dist = _extract_distance_um(cfg_name)
        by_distance[dist].extend(trial_list)

    dists = sorted(by_distance.keys())
    if not dists:
        print("  skip — no raw trace data")
        return
    ncols = min(3, len(dists))
    nrows = int(np.ceil(len(dists) / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 3.5 * nrows),
                             squeeze=False)
    fig.suptitle("13 — Motor command (force on payload) per distance",
                 fontsize=13)

    for idx, dist in enumerate(dists):
        ax = axes[idx // ncols][idx % ncols]
        trial_list = by_distance[dist]
        n_show = min(DEFAULT_MAX_TRACES_PER_PLOT, len(trial_list))
        if len(trial_list) > n_show:
            sel = np.linspace(0, len(trial_list) - 1, n_show, dtype=int)
            trial_subset = [trial_list[i] for i in sel]
        else:
            trial_subset = trial_list

        any_plotted = False
        for tid, trace in trial_subset:
            if not trace:
                continue
            # Motor command is the 5th element of each sample tuple.
            # If trace is old format (2-tuple), skip silently.
            valid = [(t, mot) for (t, _, _, _, mot) in trace if mot != 0]
            if len(valid) < 3:
                continue
            t_arr = [t for t, _ in valid]
            m_arr = [m for _, m in valid]
            ax.plot(t_arr, m_arr, linewidth=0.4, alpha=0.5, color="purple")
            any_plotted = True
        if any_plotted:
            ax.axhline(0, color="gray", linewidth=0.7)
        ax.set_title(f"{_fmt_distance(dist)}  (n={len(trial_list)})", fontsize=9)
        ax.set_xlabel("time since cmd (ms)", fontsize=8)
        ax.set_ylabel("motor cmd (force reg)", fontsize=8)
        ax.grid(alpha=0.3)

    for idx in range(len(dists), nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_dir / f"plot13_motor_cmd_profiles.{fmt}", dpi=dpi)
    if interactive:
        plt.show(block=False)
    else:
        plt.close(fig)
    print(f"  wrote plot13_motor_cmd_profiles.{fmt}  ({len(dists)} distances)")


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
    ap.add_argument("--plots", default="1,2,3,4,5,6,7,8,9,10,11,12,13",
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
        print("[plot 1/13] total cycle histogram...")
        plot_total_histogram(rows, plots_dir, fmt, dpi, interactive)
    if 2 in plot_nums:
        print("[plot 2/13] box plot by distance...")
        plot_box_by_distance(rows, plots_dir, fmt, dpi, interactive)
    if 3 in plot_nums:
        print("[plot 3/13] reaction time histogram...")
        plot_reaction_histogram(rows, plots_dir, fmt, dpi, interactive)
    if 4 in plot_nums:
        print("[plot 4/13] stacked breakdown + velocity + force overlay...")
        plot_stacked_breakdown(rows, plots_dir, fmt, dpi, interactive)
    if 5 in plot_nums:
        print("[plot 5/13] overlaid position traces...")
        if traces:
            plot_position_traces(traces, plots_dir, fmt, dpi, interactive)
            plot_position_traces_individual(traces, plots_dir, fmt, dpi, interactive)
        else:
            print("  skip — no raw trace files found")
    if 6 in plot_nums:
        print("[plot 6/13] CDF...")
        plot_cdf(rows, plots_dir, fmt, dpi, interactive)
    if 7 in plot_nums:
        print("[plot 7/13] engage vs settle scatter...")
        plot_engage_vs_settle(rows, plots_dir, fmt, dpi, interactive)
    if 8 in plot_nums:
        print("[plot 8/13] latency vs distance (log-log)...")
        plot_latency_vs_distance(rows, plots_dir, fmt, dpi, interactive)
    if 9 in plot_nums:
        print("[plot 9/13] direction comparison (pos vs neg)...")
        plot_direction_comparison(rows, plots_dir, fmt, dpi, interactive)
    if 10 in plot_nums:
        print("[plot 10/13] distance vs time (linear scaling)...")
        plot_distance_vs_time_linear(rows, plots_dir, fmt, dpi, interactive)
    if 11 in plot_nums:
        print("[plot 11/13] velocity profiles (measured vs trapezoid)...")
        if traces:
            plot_velocity_profiles(traces, rows, plots_dir, fmt, dpi, interactive)
        else:
            print("  skip — no raw trace files found")
    if 12 in plot_nums:
        print("[plot 12/13] following error profiles...")
        if traces:
            plot_following_error(traces, plots_dir, fmt, dpi, interactive)
        else:
            print("  skip — no raw trace files found")
    if 13 in plot_nums:
        print("[plot 13/13] motor command (force) profiles...")
        if traces:
            plot_motor_cmd_profiles(traces, plots_dir, fmt, dpi, interactive)
        else:
            print("  skip — no raw trace files found")

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
