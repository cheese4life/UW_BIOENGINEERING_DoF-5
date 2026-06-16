import sys
import struct
import subprocess
import time
import can
import json
import argparse
from pathlib import Path
from dataclasses import dataclass
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# from /scripts/, used to init DOF so we can use it
from scripts import dof_init


@dataclass
class TrialResult:
    reaction_us: float = float('nan')
    motion_us: float = float('nan')
    settle_us: float = float('nan')
    total_us: float = float('nan')
    
    t_cmd_ns: int | None = None
    t_react_ns: int | None = None
    t_engage_ns: int | None = None
    t_complete_ns: int | None = None
    
    # metadata for the csv exports
    
    trial_id: int = -1
    config: str = ""
    direction: int = 0
    target_counts: int = 0
    home_counts: int = 0
    final_pos_counts: int = 0
    final_error_counts: int = 0


    

NOISE_COUNTS = 3
ENGAGE_BAND_COUNTS = 50
SETTLE_BAND_COUNTS = 3
SETTLE_HOLD_S = 0.005
TIMEOUT_S = 0.5


def run_single_trial(bus, target_counts, home_counts):
    
    t_cmd = time.perf_counter_ns()
    dof_init.sr(bus, dof_init.OP_SET_POSITION, struct.pack(">i", target_counts))
    dof_init.sr(bus, dof_init.OP_UPDATE)
    
    trace = []
    
    t_react = None
    t_engage = None
    t_complete = None
    in_settle_band_since = None
    
    while True:
        
        t_now = time.perf_counter_ns()
        position = dof_init.get_pos_counts(bus)
        # engage
        if t_engage is None and t_react is not None and abs(position - target_counts) <= ENGAGE_BAND_COUNTS:
            t_engage = t_now
        
        if(t_react is None and abs(position - home_counts) > NOISE_COUNTS):
            t_react = t_now
            
        trace.append((t_now - t_cmd, position))
        
        if abs(position - target_counts) <= SETTLE_BAND_COUNTS:
            if in_settle_band_since is None:
                in_settle_band_since = t_now
            elif (t_now - in_settle_band_since) / 1e9 >= SETTLE_HOLD_S:
                t_complete = t_now
                break
        else:
            in_settle_band_since = None
                    
        
        # safety timeout
        if (t_now - t_cmd) / 1e9 > TIMEOUT_S: break
        
    result = TrialResult()

    # raw timestamps
    result.t_cmd_ns = t_cmd
    result.t_react_ns = t_react
    result.t_engage_ns = t_engage
    result.t_complete_ns = t_complete

    # derived _us fields
    if t_react is not None and t_cmd is not None:
        result.reaction_us = (t_react - t_cmd) / 1000.0
    if t_engage is not None and t_react is not None:
        result.motion_us = (t_engage - t_react) / 1000.0
    if t_complete is not None and t_engage is not None:
        result.settle_us = (t_complete - t_engage) / 1000.0
    if t_complete is not None and t_cmd is not None:
        result.total_us = (t_complete - t_cmd) / 1000.0

    # metadata, always populated regardless of trial outcome
    result.target_counts = target_counts
    result.home_counts = home_counts
    result.final_pos_counts = position
    result.final_error_counts = abs(position - target_counts)

    return result, trace

DISTANCES_UM = [10, 25, 50, 100, 200]
DIRECTIONS = [("pos", +1), ("neg", -1)]


def run_benchmark_suite(bus, directory, vel_mm_s, acc_mm_s2, trial_overrides):
    configs = []
    
    for dist in DISTANCES_UM:
        for label, direction in DIRECTIONS:           
            configs.append({
                "name": f"{dist:03d}um_{label}",
                "distance_um": dist,
                "direction": direction,
                "trials": 100
            })
            
    # change values as needed for benchmarking
    dof_init.set_motion_params(bus, vel_mm_s=vel_mm_s, acc_mm_s2=acc_mm_s2)
    
    all_results = []
    all_traces = {}
    
    global_trial_id = 0
    
    for cfg in configs:
        for trial_i in range(cfg["trials"]):
            home = dof_init.get_pos_counts(bus)
            target = home + cfg["direction"] * int(cfg["distance_um"] * dof_init.COUNTS_PER_MM / 1000)
            result, trace = run_single_trial(bus, target, home)

            result.trial_id = global_trial_id
            result.config = cfg["name"]
            result.direction = cfg["direction"]
            
            global_trial_id = global_trial_id + 1
            
            all_results.append(result)
            all_traces.setdefault(cfg["name"], []).append(trace)

    write_config_json(directory, configs, vel_mm_s, acc_mm_s2)
    write_trial_summary_csv(directory, all_results)
    write_raw_traces(directory, all_traces)
    write_summary_json(directory, all_results)


def write_config_json(out_dir, configs, vel_mm_s, acc_mm_s2):
    config_dict = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "motion": {"velocity_mm_s": vel_mm_s, "acceleration_mm_s2": acc_mm_s2},
        "detection": {
            "noise_counts": NOISE_COUNTS,
            "engage_band_counts": ENGAGE_BAND_COUNTS,
            "settle_band_counts": SETTLE_BAND_COUNTS,
            "settle_hold_s": SETTLE_HOLD_S,
            "timeout_s": TIMEOUT_S,
        },
        "configurations": configs,
    }
    with open(out_dir / "config.json", "w") as f:
        json.dump(config_dict, f, indent=2)


def write_trial_summary_csv(out_dir, all_results):
    """Write one row per TrialResult."""
    import csv
    header = [
        "trial_id", "config", "direction", "target_counts", "home_counts",
        "t_cmd_ns", "t_react_ns", "t_engage_ns", "t_complete_ns",
        "reaction_us", "motion_us", "settle_us", "total_us",
        "final_pos_counts", "final_error_counts",
    ]
    with open(out_dir / "trial_summary.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for r in all_results:
            writer.writerow([
                r.trial_id, r.config, r.direction,
                r.target_counts, r.home_counts,
                r.t_cmd_ns, r.t_react_ns, r.t_engage_ns, r.t_complete_ns,
                r.reaction_us, r.motion_us, r.settle_us, r.total_us,
                r.final_pos_counts, r.final_error_counts,
            ])


def write_raw_traces(out_dir, all_traces):
    """Write one raw_trace_<config>.csv per config.

    Each file has columns: trial_id, t_ns_since_cmd, position_counts
    All trials for a config are interleaved by trial_id.
    """
    import csv
    for config_name, traces in all_traces.items():
        path = out_dir / f"raw_trace_{config_name}.csv"
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["trial_id", "t_ns_since_cmd", "position_counts"])
            for trial_id, trace in enumerate(traces):
                for t_ns, pos_counts in trace:
                    writer.writerow([trial_id, t_ns, pos_counts])


# data parsing method written with AI
def write_summary_json(out_dir, all_results):
    """Compute per-config aggregate stats and write summary.json."""
    from statistics import mean, median

    # group results by config name
    by_config = {}
    for r in all_results:
        by_config.setdefault(r.config, []).append(r)

    summary = {}
    for config_name, results in by_config.items():
        reaction = [r.reaction_us for r in results if r.reaction_us == r.reaction_us]
        motion   = [r.motion_us   for r in results if r.motion_us   == r.motion_us]
        settle   = [r.settle_us   for r in results if r.settle_us   == r.settle_us]
        total    = [r.total_us    for r in results if r.total_us    == r.total_us]

        def _stats(values):
            if not values:
                return {"median": None, "mean": None, "p95": None, "p99": None}
            s = sorted(values)
            n = len(s)
            pct = lambda p: s[max(0, min(n - 1, int(round(p / 100.0 * (n - 1)))))]
            return {
                "median": median(s),
                "mean": mean(s),
                "p95": pct(95),
                "p99": pct(99),
            }

        summary[config_name] = {
            "n": len(results),
            "reaction_us": _stats(reaction),
            "motion_us":   _stats(motion),
            "settle_us":   _stats(settle),
            "total_us":    _stats(total),
        }

    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)
        
     
def main():
    ap = argparse.ArgumentParser(
        description="DOF-5 latency benchmark — measures stage reaction time "
                    "across multiple move distances and directions.")
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--output-dir", default=None,
                    help="default: ./bench_<timestamp>")
    ap.add_argument("--distances", default="10,25,50,100,200",
                    help="comma-separated move distances in µm")
    ap.add_argument("--trials", type=int, default=100,
                    help="trials per configuration")
    ap.add_argument("--velocity", type=float, default=1.0,
                    help="move velocity in mm/s (default: 1.0, max: 125)")
    ap.add_argument("--acceleration", type=float, default=20.0,
                    help="move acceleration in mm/s² (default: 20, max: 6000)")
    ap.add_argument("--no-plot", action="store_true",
                    help="skip automatic plotting after benchmark")
    args = ap.parse_args()

    # Create output directory
    out_dir = Path(args.output_dir
                   or f"bench_{time.strftime('%Y%m%d_%H%M%S')}")
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[bench] output dir: {out_dir.resolve()}")

    # Connect CAN bus
    bus = can.interface.Bus(channel=args.channel, interface="socketcan",
                            bitrate=1_000_000)
    try:
        # Init drive
        dof_init.init_drive(bus)

        # Set motion parameters (required before any move)
        vel = args.velocity
        acc = args.acceleration
        dof_init.set_motion_params(bus, vel_mm_s=vel, acc_mm_s2=acc)
        print(f"[bench] velocity={vel} mm/s, acceleration={acc} mm/s²")

        # Warn if velocity is too high for smallest distance relative to polling rate
        min_dist = min(int(d.strip()) for d in args.distances.split(","))
        transit_ms = min_dist / vel  # ms
        if transit_ms < 0.5:
            print(f"[bench] ⚠  {min_dist}µm move at {vel} mm/s = ~{transit_ms:.2f}ms transit "
                  f"— less than one CAN poll (~0.37ms)."
                  f" Events may be missed for small distances.")

        # Dry-run pre-check: one small move to verify everything works
        print("[bench] dry-run pre-check...")
        home = dof_init.get_pos_counts(bus)
        test_target = home + int(10 * dof_init.COUNTS_PER_MM / 1000)  # 10 µm
        pre_result, _ = run_single_trial(bus, test_target, home)
        if pre_result.t_complete_ns is not None:
            print(f"  OK — trial completed in {pre_result.total_us:.0f} µs")
        else:
            print(f"  WARNING — trial did not complete "
                  f"(timeout or no reaction)")
        dof_init.sr(bus, dof_init.OP_SET_POSITION,
                    struct.pack(">i", home))
        dof_init.sr(bus, dof_init.OP_UPDATE)

        # Run full benchmark suite
        run_benchmark_suite(bus, out_dir, vel, acc, trial_overrides=None)
        print(f"[bench] done — results in {out_dir.resolve()}")

        # Auto-generate plots
        if not args.no_plot:
            plot_script = (Path(__file__).resolve().parent.parent
                           / "scripts" / "plot_latency_bench.py")
            if plot_script.exists():
                print(f"[bench] generating plots...")
                subprocess.run(
                    [sys.executable, str(plot_script), str(out_dir), "--open"],
                    check=False,
                )
            else:
                print(f"[bench] plot script not found at {plot_script}")

    except KeyboardInterrupt:
        print("\n[bench] interrupted by user")
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
    
    
    
    

        
        
        
        
        
    
    

    
    
    
    
