#!/usr/bin/env python3
"""DOF-5 latency-bench smoke test.

Purpose: figure out what telemetry the Juno chip is actually willing to give
us, and how fast, BEFORE writing the real latency benchmark. Output of this
script is what the latency-bench spec will be built around.

Run on the Linux bench machine (socketcan + python-can + IXXAT up on can0).
Mac dev machine cannot run this -- it has no stage attached.

Phases:
    1. Connect + init drive (reuses the known-good sequence from
       dof_oscillate_v1.py).
    2. Round-trip latency of GetActualPosition (op 0x37). 2000 reps.
       Tells us our polling-rate ceiling.
    3. Round-trip latency of the command path: SetPosition + Update
       (issuing the same position the stage is already at, so no motion).
       1000 reps. Tells us the command-send floor latency.
    4. Probe candidate read opcodes that are NOT yet used in the existing
       code. If they respond, we get richer telemetry for the real bench.
       Each opcode is tried a few times with a short timeout; failure is
       fine and just means "skip in the real bench".
    5. (Only with --allow-motion) Execute a few small moves and dump the
       raw position trace at max polling rate. This is the dataset we use
       to decide whether a position trace alone is enough to derive
       t_motion / t_settle, or whether we need the status opcodes from
       phase 4.

Outputs:
    - prints a human-readable summary to stdout
    - writes results to <output-dir>/ as:
        latency_get_pos.csv          (one row per round-trip)
        latency_cmd_path.csv         (one row per round-trip)
        opcode_probe.json            (which opcodes responded)
        move_trace_<n>.csv           (one per small-move trial)
        summary.json                 (aggregate stats)

SAFETY:
    - --allow-motion is required to issue any move command. Without it the
      script is read-only.
    - Motion phase uses small moves (default 50 um) well inside the soft
      limits from dof_oscillate_v1.py (+/- 1.2 mm).
    - Ctrl-C at any point cleanly shuts the bus and leaves the servo
      holding its last commanded position.

Usage:
    python3 dof_smoke_test.py                       # phases 1-4, no motion
    python3 dof_smoke_test.py --allow-motion        # all phases
    python3 dof_smoke_test.py --output-dir ./smoke_2026-05-22
"""
from __future__ import annotations

import argparse
import json
import os
import site
import struct
import sys
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from statistics import mean, median, pstdev

sys.path.insert(0, site.getusersitepackages())
import can  # type: ignore

# ---------------------------------------------------------------------------
# Juno CAN protocol constants (copied from dof_oscillate_v1.py -- known good)
# ---------------------------------------------------------------------------
TX_ID = 0x600
RX_ID = 0x580
AXIS = 0
COUNTS_PER_MM = 200_000
SAMPLE_S = 51e-6
SOFT_LIMIT_MM = 1.2  # match dof_oscillate_v1.py

# Verified opcodes (used in existing scripts)
OP_SET_POSITION = 0x10
OP_SET_VELOCITY = 0x11
OP_UPDATE = 0x1A
OP_RESET_EVENT = 0x34
OP_GET_ACT_POS = 0x37
OP_SET_OPMODE = 0x65
OP_SET_MOTOR_CMD = 0x77
OP_SET_ACC = 0x90
OP_SET_DEC = 0x91
OP_CAL_ANALOG = 0xF5
OPMODE_CAL = 0x06
OPMODE_FULL = 0x37

# Candidate opcodes to probe in phase 4. These are PLAUSIBLE per the Juno
# documentation family but are NOT yet verified on this stage. The smoke
# test will tell us which actually respond. Do not trust this list until
# phase 4 confirms.
CANDIDATE_READ_OPCODES = [
    # (opcode, label, payload, expected_resp_bytes_min)
    (0x35, "GetEventStatus",       b"", 2),
    (0xA6, "GetActivityStatus",    b"", 2),
    (0xA7, "GetSignalStatus",      b"", 2),
    (0xAD, "GetActualVelocity",    b"", 2),
    (0x1E, "GetTargetPosition",    b"", 2),
    (0x4A, "GetCommandedPosition", b"", 2),
    (0xB6, "GetMotorCommand",      b"", 2),
    (0x4B, "GetIntegrationStep",   b"", 2),
]


# ---------------------------------------------------------------------------
# Low-level CAN helpers (mirror dof_oscillate_v1.py so behavior is identical)
# ---------------------------------------------------------------------------
def sr(bus, op, p=b"", timeout=0.2):
    """Send + receive one CAN round-trip. Returns response bytes."""
    bus.send(can.Message(arbitration_id=TX_ID,
                         data=bytes([AXIS, op]) + p,
                         is_extended_id=False))
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        m = bus.recv(timeout=end - time.monotonic())
        if m and m.arbitration_id == RX_ID:
            return bytes(m.data)
    raise RuntimeError(f"timeout op=0x{op:02X}")


def sr_timed(bus, op, p=b"", timeout=0.2):
    """sr() but returns (response_bytes, elapsed_ns_send_to_recv)."""
    t0 = time.perf_counter_ns()
    resp = sr(bus, op, p, timeout)
    t1 = time.perf_counter_ns()
    return resp, t1 - t0


def s32(d):
    body = d[2:]
    if not body:
        return 0
    sign = b"\xff" if body[0] & 0x80 else b"\x00"
    return struct.unpack(">i", sign * (4 - len(body)) + body)[0]


def get_pos_counts(bus):
    return s32(sr(bus, OP_GET_ACT_POS))


def vel_reg(mm_s):
    return int(round(mm_s * COUNTS_PER_MM * SAMPLE_S * 65536))


def acc_reg(mm_s2):
    return int(round(mm_s2 * COUNTS_PER_MM * SAMPLE_S * SAMPLE_S * 65536))


def init_drive(bus):
    print("[init] resetting events, calibrating, enabling servo...")
    sr(bus, OP_RESET_EVENT, struct.pack(">H", 0xA000))
    sr(bus, OP_RESET_EVENT, struct.pack(">H", 0xEFFF))
    sr(bus, OP_SET_MOTOR_CMD, struct.pack(">h", 0))
    sr(bus, OP_SET_OPMODE, struct.pack(">H", OPMODE_CAL))
    time.sleep(0.05)
    sr(bus, OP_CAL_ANALOG, struct.pack(">H", 0))
    time.sleep(0.2)
    sr(bus, OP_RESET_EVENT, struct.pack(">H", 0xEFFF))
    sr(bus, OP_SET_OPMODE, struct.pack(">H", OPMODE_FULL))
    time.sleep(0.05)
    pos_mm = get_pos_counts(bus) / COUNTS_PER_MM
    print(f"[init] servo on at {pos_mm:+.4f} mm")
    return pos_mm


def set_motion_params(bus, vel_mm_s, acc_mm_s2):
    sr(bus, OP_SET_ACC, struct.pack(">i", acc_reg(acc_mm_s2)))
    sr(bus, OP_SET_DEC, struct.pack(">i", acc_reg(acc_mm_s2)))
    sr(bus, OP_SET_VELOCITY, struct.pack(">i", vel_reg(vel_mm_s)))


# ---------------------------------------------------------------------------
# Stats helpers
# ---------------------------------------------------------------------------
@dataclass
class LatencyStats:
    n: int
    min_us: float
    median_us: float
    mean_us: float
    p95_us: float
    p99_us: float
    max_us: float
    stdev_us: float


def summarize_ns(samples_ns: list[int]) -> LatencyStats:
    if not samples_ns:
        return LatencyStats(0, 0, 0, 0, 0, 0, 0, 0)
    s = sorted(samples_ns)
    n = len(s)

    def pct(p):
        k = max(0, min(n - 1, int(round(p / 100.0 * (n - 1)))))
        return s[k] / 1000.0

    return LatencyStats(
        n=n,
        min_us=s[0] / 1000.0,
        median_us=median(s) / 1000.0,
        mean_us=mean(s) / 1000.0,
        p95_us=pct(95),
        p99_us=pct(99),
        max_us=s[-1] / 1000.0,
        stdev_us=pstdev(s) / 1000.0 if n > 1 else 0.0,
    )


def print_stats(label: str, st: LatencyStats):
    print(f"  {label}: n={st.n}  "
          f"min={st.min_us:.0f}  med={st.median_us:.0f}  "
          f"mean={st.mean_us:.0f}  p95={st.p95_us:.0f}  "
          f"p99={st.p99_us:.0f}  max={st.max_us:.0f}  "
          f"sd={st.stdev_us:.0f}  (us)")


# ---------------------------------------------------------------------------
# Phases
# ---------------------------------------------------------------------------
def phase_get_pos_latency(bus, out_dir: Path, reps: int) -> LatencyStats:
    print(f"\n[phase 2] GetActualPosition round-trip latency x{reps}")
    samples = []
    with open(out_dir / "latency_get_pos.csv", "w") as f:
        f.write("idx,elapsed_ns,position_counts\n")
        for i in range(reps):
            try:
                resp, dt = sr_timed(bus, OP_GET_ACT_POS)
                pos = s32(resp)
                samples.append(dt)
                f.write(f"{i},{dt},{pos}\n")
            except RuntimeError as e:
                f.write(f"{i},TIMEOUT,{e}\n")
    st = summarize_ns(samples)
    print_stats("get_pos", st)
    return st


def phase_cmd_path_latency(bus, out_dir: Path, reps: int) -> LatencyStats:
    """Send SetPosition (current position, no motion) + Update, time both."""
    print(f"\n[phase 3] Command-path round-trip latency x{reps} "
          "(SetPosition+Update, target = current pos, no motion)")
    cur = get_pos_counts(bus)
    samples = []
    with open(out_dir / "latency_cmd_path.csv", "w") as f:
        f.write("idx,set_ns,update_ns,total_ns\n")
        for i in range(reps):
            try:
                _, t_set = sr_timed(
                    bus, OP_SET_POSITION, struct.pack(">i", cur))
                _, t_upd = sr_timed(bus, OP_UPDATE)
                total = t_set + t_upd
                samples.append(total)
                f.write(f"{i},{t_set},{t_upd},{total}\n")
            except RuntimeError as e:
                f.write(f"{i},TIMEOUT,TIMEOUT,{e}\n")
    st = summarize_ns(samples)
    print_stats("cmd_path", st)
    return st


def phase_probe_opcodes(bus, out_dir: Path) -> dict:
    """Try each candidate read opcode a few times; report response + timing."""
    print("\n[phase 4] Probing candidate read opcodes (3 tries each)")
    results = {}
    for op, label, payload, _min_resp in CANDIDATE_READ_OPCODES:
        attempts = []
        for _ in range(3):
            try:
                resp, dt = sr_timed(bus, op, payload, timeout=0.1)
                attempts.append({
                    "ok": True,
                    "elapsed_us": dt / 1000.0,
                    "resp_hex": resp.hex(),
                    "resp_len": len(resp),
                })
            except RuntimeError as e:
                attempts.append({"ok": False, "error": str(e)})
        ok_count = sum(1 for a in attempts if a.get("ok"))
        results[f"0x{op:02X}_{label}"] = {
            "opcode": op,
            "label": label,
            "ok_count": ok_count,
            "attempts": attempts,
        }
        status = "OK " if ok_count == 3 else ("?? " if ok_count else "-- ")
        first_ok = next((a for a in attempts if a.get("ok")), None)
        extra = (f"resp={first_ok['resp_hex']:>16}  "
                 f"~{first_ok['elapsed_us']:.0f}us"
                 if first_ok else "no response")
        print(f"  {status} 0x{op:02X} {label:<22} {extra}")
    with open(out_dir / "opcode_probe.json", "w") as f:
        json.dump(results, f, indent=2)
    return results


def phase_move_traces(bus, out_dir: Path,
                      move_um: float, reps: int,
                      vel_mm_s: float, acc_mm_s2: float,
                      extra_read_opcodes: list[int]) -> list[dict]:
    """For each rep: command +move_um from current pos, poll at max rate
    until pos has been within +-15 nm of target for 5 ms straight, OR 200 ms
    max. Then move back. One CSV per move.
    """
    print(f"\n[phase 5] Move traces: {reps} reps of +/-{move_um:.1f} um")
    set_motion_params(bus, vel_mm_s, acc_mm_s2)
    settle_band_counts = max(1, int(round(15e-6 * COUNTS_PER_MM)))  # +-15 nm
    settle_hold_s = 0.005
    move_timeout_s = 0.200

    home_counts = get_pos_counts(bus)
    move_counts = int(round(move_um * 1e-3 * COUNTS_PER_MM))
    summaries = []

    for rep in range(reps):
        for direction in (+1, -1):
            target = home_counts + direction * move_counts
            # Issue the move (commanded-out timestamp captured around send)
            t_cmd_start = time.perf_counter_ns()
            sr(bus, OP_SET_POSITION, struct.pack(">i", target))
            sr(bus, OP_UPDATE)
            t_cmd_done = time.perf_counter_ns()

            # Poll position as fast as possible until settle or timeout
            trace = []  # (t_ns_since_cmd_start, pos_counts, extra...)
            in_band_since = None
            t_start = time.perf_counter_ns()
            while True:
                try:
                    resp, _ = sr_timed(bus, OP_GET_ACT_POS, timeout=0.05)
                    pos = s32(resp)
                except RuntimeError:
                    break
                t_now = time.perf_counter_ns()
                row = [t_now - t_cmd_start, pos]
                # Optional extra opcodes that worked in phase 4
                for op in extra_read_opcodes:
                    try:
                        rx, _ = sr_timed(bus, op, b"", timeout=0.05)
                        row.append(rx.hex())
                    except RuntimeError:
                        row.append("")
                trace.append(row)

                if abs(pos - target) <= settle_band_counts:
                    if in_band_since is None:
                        in_band_since = t_now
                    elif (t_now - in_band_since) / 1e9 >= settle_hold_s:
                        break
                else:
                    in_band_since = None

                if (t_now - t_start) / 1e9 >= move_timeout_s:
                    break

            tag = f"{rep:02d}_{'pos' if direction > 0 else 'neg'}"
            fn = out_dir / f"move_trace_{tag}.csv"
            with open(fn, "w") as f:
                header = ["t_ns_since_cmd", "position_counts"]
                header += [f"op_0x{op:02X}_hex" for op in extra_read_opcodes]
                f.write(",".join(header) + "\n")
                for row in trace:
                    f.write(",".join(str(x) for x in row) + "\n")

            send_us = (t_cmd_done - t_cmd_start) / 1000.0
            settled = in_band_since is not None
            settle_us = ((in_band_since - t_cmd_start) / 1000.0
                         if settled else None)
            summaries.append({
                "rep": rep,
                "direction": direction,
                "target_counts": target,
                "send_us": send_us,
                "samples": len(trace),
                "settled": settled,
                "settle_us_from_cmd": settle_us,
                "trace_file": fn.name,
            })
            print(f"  rep {rep:02d} dir={direction:+d}  "
                  f"send={send_us:5.0f}us  samples={len(trace):4d}  "
                  f"settled={'Y' if settled else 'N'}  "
                  f"settle={settle_us:.0f}us"
                  if settled else
                  f"  rep {rep:02d} dir={direction:+d}  "
                  f"send={send_us:5.0f}us  samples={len(trace):4d}  "
                  f"settled=N  settle=?")

            # Return to home before next iteration
            sr(bus, OP_SET_POSITION, struct.pack(">i", home_counts))
            sr(bus, OP_UPDATE)
            time.sleep(0.05)

    return summaries


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--bitrate", type=int, default=1_000_000)
    ap.add_argument("--output-dir", default=None,
                    help="default: ./smoke_<timestamp>")
    ap.add_argument("--get-pos-reps", type=int, default=2000)
    ap.add_argument("--cmd-path-reps", type=int, default=1000)
    ap.add_argument("--allow-motion", action="store_true",
                    help="enable phase 5 (small moves)")
    ap.add_argument("--move-um", type=float, default=50.0,
                    help="phase 5 move magnitude in micrometers")
    ap.add_argument("--move-reps", type=int, default=5)
    ap.add_argument("--move-vel-mm-s", type=float, default=1.0)
    ap.add_argument("--move-acc-mm-s2", type=float, default=20.0)
    args = ap.parse_args()

    out_dir = Path(args.output_dir or
                   f"smoke_{time.strftime('%Y%m%d_%H%M%S')}")
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[smoke] output dir: {out_dir.resolve()}")

    bus = can.interface.Bus(channel=args.channel, interface="socketcan",
                            bitrate=args.bitrate)
    summary: dict = {
        "channel": args.channel,
        "bitrate": args.bitrate,
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "args": vars(args),
    }
    try:
        # Phase 1
        start_pos_mm = init_drive(bus)
        summary["start_pos_mm"] = start_pos_mm

        # Phase 2
        st_get = phase_get_pos_latency(bus, out_dir, args.get_pos_reps)
        summary["get_pos_latency_us"] = asdict(st_get)

        # Phase 3
        st_cmd = phase_cmd_path_latency(bus, out_dir, args.cmd_path_reps)
        summary["cmd_path_latency_us"] = asdict(st_cmd)

        # Phase 4
        probe = phase_probe_opcodes(bus, out_dir)
        summary["opcode_probe"] = {k: {"ok_count": v["ok_count"],
                                       "opcode": v["opcode"],
                                       "label": v["label"]}
                                   for k, v in probe.items()}
        working_extra = [v["opcode"] for v in probe.values()
                         if v["ok_count"] == 3]

        # Phase 5 (optional)
        if args.allow_motion:
            # Confirm we won't blow past soft limits
            margin_mm = args.move_um * 1e-3
            if abs(start_pos_mm) + margin_mm > SOFT_LIMIT_MM - 0.05:
                print(f"[abort] start pos {start_pos_mm:+.4f} mm + move "
                      f"{margin_mm:.4f} mm too close to soft limit "
                      f"+/-{SOFT_LIMIT_MM} mm. Re-home first.")
            else:
                moves = phase_move_traces(
                    bus, out_dir,
                    move_um=args.move_um,
                    reps=args.move_reps,
                    vel_mm_s=args.move_vel_mm_s,
                    acc_mm_s2=args.move_acc_mm_s2,
                    extra_read_opcodes=working_extra,
                )
                summary["move_traces"] = moves
        else:
            print("\n[phase 5] skipped (no --allow-motion)")

        with open(out_dir / "summary.json", "w") as f:
            json.dump(summary, f, indent=2)
        print(f"\n[smoke] wrote summary -> {out_dir / 'summary.json'}")
        print("[smoke] done.")

    except KeyboardInterrupt:
        print("\n[smoke] interrupted by user")
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
