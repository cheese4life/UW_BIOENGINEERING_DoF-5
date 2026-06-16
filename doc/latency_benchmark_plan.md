# DOF-5 Latency Benchmark — Project Reference

## 1. Purpose

Build a barebones real-time latency benchmark for the DOF-5 stage (Juno chip, CAN bus). Gather ~100 samples per test configuration across multiple move distances and directions. Output raw CSVs that can later be turned into plots and graphs to visualize stage reaction speed.

**Goal:** answer *"how fast can the DOF stage react to new focus information?"* with statistical rigor — not just a few traces, but enough data to draw distributions.

**Early use case:** autofocus on button press. **Later:** continuous real-time autofocus (dependent on OCT hardware and other focus sensors).

**Constraint:** this is medical hardware. No simulations. All testing is live on the bench machine with the physical stage attached.

---

## 2. What We Already Know (from Smoke Tests)

Smoke test script: `scripts/dof_smoke_test.py`
Results: `smoketest_results/`
Telemetry reference: `smoketest_results/stage_telemetry_reference.txt`

### Verified Opcodes (all respond, 3/3)

| Opcode | Label | Resp bytes | Typical latency |
|--------|-------|------------|-----------------|
| `0x37` | GetActualPosition | 4 | ~370 µs |
| `0x35` | GetEventStatus | 2 | ~350 µs |
| `0xA6` | GetActivityStatus | 4 | ~340 µs |
| `0xA7` | GetSignalStatus | 6 | ~340 µs |
| `0xAD` | GetActualVelocity | 6 | ~400 µs |
| `0x1E` | GetTargetPosition | 6 | ~440 µs |
| `0x4A` | GetCommandedPosition | 6 | ~390 µs |
| `0xB6` | GetMotorCommand | 2 | ~315 µs |
| `0x4B` | GetIntegrationStep | 6 | ~380 µs |

### Latency Numbers

| Metric | Median | p95 | p99 |
|--------|--------|-----|-----|
| GetPosition round-trip | ~375 µs | ~420 µs | ~500 µs |
| SetPosition+Update round-trip | ~720 µs | ~765 µs | ~870 µs |
| 50 µm move-to-settle | ~145 ms | — | ~173 ms |

### Settle Criterion (proven)

Position within ±15 nm (±3 counts) of target for 5 ms continuous.

### Bus Constants

```
CAN bus:   can0, 1 Mbps
TX_ID:     0x600
RX_ID:     0x580
Axis:      0
Counts/mm: 200,000
Sample period: 51 µs
Soft limits: ±1.2 mm
```

---

## 3. Files to Write

```
scripts/
├── dof_smoke_test.py               [EXISTING — keep, do not modify]
├── dof_latency_bench.py            [NEW — benchmark data collector]
└── plot_latency_bench.py           [NEW — reads CSVs, generates plots]
```

### `dof_latency_bench.py`

The benchmark runner. Runs on the bench machine. Collects real-time data.

**Responsibilities:**
- Reuse CAN helpers and `init_drive()` from `dof_smoke_test.py`
- Define `TrialResult` dataclass with event-breakdown fields
- Implement `run_single_trial()` that issues one move and detects
  `t_cmd`, `t_react`, `t_engage`, `t_complete`
- Implement `run_benchmark_suite()` orchestrator that loops over
  all configurations × 100 trials each
- Write output: `config.json`, `trial_summary.csv`,
  `raw_trace_<config>.csv` per configuration, `summary.json`

**CLI:**
```bash
python3 scripts/dof_latency_bench.py                        # default test matrix
python3 scripts/dof_latency_bench.py --distances 10,50,100  # custom distances
python3 scripts/dof_latency_bench.py --trials 50             # fewer trials
python3 scripts/dof_latency_bench.py --output-dir ./bench_2026-06-11
```

### `plot_latency_bench.py`

The visualization script. Runs anywhere (including Mac dev machine). Reads CSVs produced by the benchmark.

**Responsibilities:**
- Parse `trial_summary.csv` and `raw_trace_*.csv`
- Generate the plots listed in Section 8
- Save as PNG/SVG to an output directory

**CLI:**
```bash
python3 scripts/plot_latency_bench.py ./bench_20260611_120000
python3 scripts/plot_latency_bench.py ./bench_20260611_120000 --format svg
```

---

## 4. Test Matrix

Run trials at multiple distances, both directions, ~100 each:

| Config | Distance | Direction | Trials | Rationale |
|--------|----------|-----------|--------|-----------|
| `010um_pos` | 10 µm | +1 | 100 | Tiny focus tweak |
| `010um_neg` | 10 µm | -1 | 100 | Tiny focus tweak |
| `025um_pos` | 25 µm | +1 | 100 | Small correction |
| `025um_neg` | 25 µm | -1 | 100 | Small correction |
| `050um_pos` | 50 µm | +1 | 100 | Baseline (matches smoke) |
| `050um_neg` | 50 µm | -1 | 100 | Baseline (matches smoke) |
| `100um_pos` | 100 µm | +1 | 100 | Moderate move |
| `100um_neg` | 100 µm | -1 | 100 | Moderate move |
| `200um_pos` | 200 µm | +1 | 100 | Large move |
| `200um_neg` | 200 µm | -1 | 100 | Large move |

**Total: 1000 trials × ~250 ms ≈ 4 minutes of testing.**

Motion parameters (fixed across all trials):
- Velocity: 1.0 mm/s
- Acceleration: 20.0 mm/s²

---

## 5. Event Definitions (What We're Measuring)

One core function issues a move, polls `GetActualPosition` as fast as
possible, and detects four key moments:

| Event | Variable | Definition |
|-------|----------|------------|
| **Command sent** | `t_cmd` | Timestamp when `SetPosition` byte hits the CAN bus. t=0 reference. |
| **Stage reacts** | `t_react` | Position first moves > noise threshold (±3 counts / ±15 nm) away from starting position. Measures *how long the stage takes to physically respond*. |
| **Stage engages** | `t_engage` | Position reaches within engage band (±50 counts / ±250 nm) of target. Measures *how long motion/transit takes*. |
| **Stage completes** | `t_complete` | Position stays within settle band (±3 counts / ±15 nm) of target for 5 ms continuous. Measures *full rest-to-rest settle*. Matches smoke test criterion. |

### Derived Columns (in output CSV)

```
reaction_us  = t_react    - t_cmd        "How long to start moving"
motion_us    = t_engage   - t_react      "How long to get near target"
settle_us    = t_complete - t_engage     "How long to lock in"
total_us     = t_complete - t_cmd        "Full rest-to-rest cycle"
```

---

## 6. Event Detection Algorithm

```python
def run_single_trial(bus, target_counts, home_counts):
    """Issue one move, poll position at max rate, detect 4 events."""

    # ----- 1. Send command, record t=0 -----
    t_cmd = time.perf_counter_ns()
    sr(bus, OP_SET_POSITION, struct.pack(">i", target_counts))
    sr(bus, OP_UPDATE)

    # ----- 2. Poll loop -----
    trace = []
    t_react = None
    t_engage = None
    t_complete = None
    in_settle_band_since = None

    while True:
        t_now = time.perf_counter_ns()
        pos = get_pos_counts(bus)
        trace.append((t_now - t_cmd, pos))

        # Detect REACT: first movement beyond noise
        if t_react is None and abs(pos - home_counts) > NOISE_COUNTS:
            t_react = t_now

        # Detect ENGAGE: within engage band of target (only after react)
        if t_engage is None and t_react is not None:
            if abs(pos - target_counts) <= ENGAGE_BAND_COUNTS:
                t_engage = t_now

        # Detect COMPLETE: within settle band for hold time continuous
        if abs(pos - target_counts) <= SETTLE_BAND_COUNTS:
            if in_settle_band_since is None:
                in_settle_band_since = t_now
            elif (t_now - in_settle_band_since) / 1e9 >= SETTLE_HOLD_S:
                t_complete = t_now
                break
        else:
            in_settle_band_since = None  # left band, reset timer

        # Safety timeout
        if (t_now - t_cmd) / 1e9 > TIMEOUT_S:
            break

    return TrialResult(...), trace
```

### Constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `NOISE_COUNTS` | 3 | Position change threshold for "reacted" (±15 nm) |
| `ENGAGE_BAND_COUNTS` | 50 | "Near target" threshold (±250 nm) |
| `SETTLE_BAND_COUNTS` | 3 | "At target" threshold (±15 nm) |
| `SETTLE_HOLD_S` | 0.005 | Must stay in band for 5 ms |
| `TIMEOUT_S` | 0.5 | Max 500 ms per trial (safety) |

---

## 7. Output Format

### Directory Structure (per run)

```
bench_20260611_120000/
├── config.json                  # All parameters used for this run
├── trial_summary.csv            # One row per trial (1000 rows)
├── summary.json                 # Aggregate stats per configuration
├── raw_trace_010um_pos.csv      # Raw position-vs-time, all 100 trials
├── raw_trace_010um_neg.csv
├── raw_trace_025um_pos.csv
├── raw_trace_025um_neg.csv
├── raw_trace_050um_pos.csv
├── raw_trace_050um_neg.csv
├── raw_trace_100um_pos.csv
├── raw_trace_100um_neg.csv
├── raw_trace_200um_pos.csv
└── raw_trace_200um_neg.csv      # 10 trace files total
```

### `config.json`

```json
{
  "can_channel": "can0",
  "bitrate": 1000000,
  "timestamp": "2026-06-11 12:00:00",
  "motion": {
    "velocity_mm_s": 1.0,
    "acceleration_mm_s2": 20.0
  },
  "detection": {
    "noise_counts": 3,
    "engage_band_counts": 50,
    "settle_band_counts": 3,
    "settle_hold_s": 0.005,
    "timeout_s": 0.5
  },
  "configurations": [
    {"distance_um": 10, "direction": 1, "trials": 100},
    {"distance_um": 10, "direction": -1, "trials": 100},
    ...
  ]
}
```

### `trial_summary.csv`

```csv
trial_id,config,direction,target_counts,home_counts,
t_cmd_ns,t_react_ns,t_engage_ns,t_complete_ns,
reaction_us,motion_us,settle_us,total_us,
final_pos_counts,final_error_counts
```

One row per trial. `NaN` for any event that was not detected (e.g., `t_engage`
if the trial timed out before engaging).

### `raw_trace_<config>.csv`

```csv
trial_id,t_ns_since_cmd,position_counts
0,0,-123
0,372000,-123
0,705000,-122
...
0,145200000,9876
1,0,451
1,368000,452
...
```

All 100 trials in a single file, interleaved by `trial_id`. Column
`t_ns_since_cmd` is nanoseconds since `t_cmd`. Position is raw encoder counts.

### `summary.json`

```json
{
  "010um_pos": {
    "n": 100,
    "reaction_us": { "median": ..., "mean": ..., "p95": ..., "p99": ... },
    "motion_us":    { "median": ..., "mean": ..., "p95": ..., "p99": ... },
    "settle_us":    { "median": ..., "mean": ..., "p95": ..., "p99": ... },
    "total_us":     { "median": ..., "mean": ..., "p95": ..., "p99": ... }
  },
  ...
}
```

---

## 8. Plotting Plan

Generated by `plot_latency_bench.py` from the benchmark output CSVs.

| # | Plot | X-axis | Y-axis | What it shows |
|---|------|--------|--------|---------------|
| 1 | Total cycle histogram | total_us (µs or ms) | frequency | Distribution of full move times, all trials pooled |
| 2 | Box plot by distance | move distance (µm) | total_us (ms) | How move duration scales with distance |
| 3 | Reaction time histogram | reaction_us (µs) | frequency | Is reaction time constant across distances? |
| 4 | Stacked breakdown bar | move distance (µm) | time (ms) | reaction + motion + settle broken out |
| 5 | Overlaid position traces | ms since command | position (µm) | All 100 traces per config overlaid, one subplot per distance |
| 6 | CDF (cumulative distribution) | total_us (µs or ms) | cumulative % | "95% of 50 µm moves complete within X ms" |
| 7 | Engage vs settle scatter | engage time (ms) | settle time (ms) | Correlation between crude arrival and fine lock-in |

**Plotting library:** `matplotlib` (likely already available on the bench machine).

---

## 9. Implementation Phases

### Phase A — Skeleton + Single Trial
- Create `dof_latency_bench.py` with imports and CAN helpers
- Define `TrialResult` dataclass
- Implement `run_single_trial()` with full event detection
- Test with one hardcoded 50 µm move and print to console

### Phase B — Benchmark Suite
- Implement `run_benchmark_suite()` orchestrator
- Loop over configurations × trials
- Write `config.json`, `trial_summary.csv`, `summary.json`
- Write raw trace files per configuration

### Phase C — Plotting Script
- Create `plot_latency_bench.py`
- Parse benchmark output
- Generate all 7 plot types
- Save to output directory

### Phase D — Run + Iterate
- Run full benchmark on the bench machine
- Generate plots
- Review results, tune detection thresholds if needed
- Re-run if detection isn't capturing events cleanly

---

## 10. Edge Cases & Safety

- **What if the stage never reacts?** Detectable — `t_react` stays `None`.
  Mark trial as failed, record timeout, move to next trial.
- **What if the move overshoots?** The engage and settle bands catch it —
  overshoot means leaving and re-entering the band, which resets the hold
  timer. Correctly measures full settle including overshoot.
- **What if the stage drifts during the trial?** The noise threshold
  prevents spurious react detection. The settle hold timer prevents
  declaring settle on a transient pass through the band.
- **What if the stage is already near the target?** Unlikely for the
  distances in the test matrix, but `run_single_trial` could issue a
  zero-move command to home position between trials.
- **Ctrl-C safety:** wrap the main loop in `try/finally` so the CAN bus
  shuts down cleanly and the servo stays in position.

---

## 11. Machine Dependency

| Where | What | Status |
|-------|------|--------|
| Bench machine (`dofcomputer`) | CAN bus, socketcan, IXXAT | Only place benchmark can run |
| Bench machine | `python-can` library | Installed via user site-packages |
| Mac dev machine | Cannot run benchmark (no stage) | Can run plotting script |
| Both | `matplotlib` | Needed for plots |

---

## 12. Reference Code

The proven helpers in `dof_smoke_test.py` to reuse:

- `sr(bus, op, payload, timeout)` — send CAN + receive response
- `sr_timed(bus, op, payload, timeout)` — same but returns elapsed ns
- `s32(data)` — decode signed 32-bit int from CAN response
- `get_pos_counts(bus)` — read current position in encoder counts
- `init_drive(bus)` — full init sequence (calibrate, enable servo)
- `set_motion_params(bus, vel_mm_s, acc_mm_s2)` — set velocity/accel
- `LatencyStats` dataclass + `summarize_ns()` — statistical aggregation

New things to build on top:

- `TrialResult` dataclass with broken-out event timestamps
- `run_single_trial()` with the 4-event detection loop
- `run_benchmark_suite()` orchestrator
- All output CSV/JSON writers
- All plotting in `plot_latency_bench.py`
