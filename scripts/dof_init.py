from __future__ import annotations

import site
import struct
import sys
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from statistics import mean, median, pstdev

sys.path.insert(0, site.getusersitepackages())
import can 

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

# statistical helpers for later implementations
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


