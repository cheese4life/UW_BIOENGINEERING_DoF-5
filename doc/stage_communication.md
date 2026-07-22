# DOF-5 Stage Communication — Technical Reference

> **Scope:** CAN 2.0B protocol, Juno MCU opcode set, command execution model,
> telemetry polling, and the software layer that wraps it all.
> **Last updated:** 2026-07-20

---

## 1. System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│  HOST (Linux, socketcan)                                        │
│  ┌──────────────────────┐                                       │
│  │ dof_init.py           │  Python CAN wrapper                  │
│  │ autofocus_latency_    │                                       │
│  │ webapp.py             │                                       │
│  └────────┬─────────────┘                                       │
│           │ python-can socketcan                                │
│           ▼                                                     │
│  ┌──────────────────────┐                                       │
│  │ can0 (1 Mbps)         │  CAN 2.0B interface                  │
│  └────────┬─────────────┘                                       │
└───────────┼─────────────────────────────────────────────────────┘
            │ 4-wire CAN bus (CAN_H, CAN_L, GND, +24V)
            ▼
┌─────────────────────────────────────────────────────────────────┐
│  JUNO MCU (Dover / AllMotion)                                    │
│  ┌──────────────────────┐                                       │
│  │ Servo loop (~19.6 kHz)│  PID + trajectory planner            │
│  │ Encoder interface     │  incremental encoder                  │
│  │ Voice-coil driver     │  PWM → analog current                │
│  │ CAN transceiver       │  1 Mbps, 11-bit IDs                  │
│  └──────────────────────┘                                       │
└─────────────────────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────────┐
│  DOF-5 STAGE                                                     │
│  Voice-coil motor  |  200,000 counts/mm (5 nm/count)             │
│  Travel: ±1.2 mm (soft limit), ±3 mm (hard stop)                 │
│  Payload: 0–900 g                                                │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. CAN Bus Layer

### 2.1 Physical

| Parameter | Value |
|-----------|-------|
| Interface | `socketcan` (Linux kernel driver) |
| Channel | `can0` |
| Bitrate | 1,000,000 bps (1 Mbps) |
| Frame format | CAN 2.0B, **11-bit standard IDs** |
| Transmit ID | `0x600` |
| Receive ID | `0x580` |

### 2.2 Message Framing

Every command and response follows the same structure:

```
Byte 0:    Axis number (always 0x00 for a single-axis stage)
Byte 1:    Opcode (see §3)
Bytes 2-N: Payload (opcode-dependent, may be 0 bytes)
```

The Juno MCU processes one command at a time. Every TX message on `0x600`
produces exactly one RX message on `0x580`. The protocol is strictly
**command → response** — the MCU never initiates a message on its own.

### 2.3 Send-Receive Loop (`sr()`)

The foundational building block in `scripts/dof_init.py`:

```python
def sr(bus, op, p=b"", timeout=0.2):
    bus.send(can.Message(
        arbitration_id=0x600,
        data=bytes([0, op]) + p,
        is_extended_id=False,
    ))
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        m = bus.recv(timeout=end - time.monotonic())
        if m and m.arbitration_id == 0x580:
            return bytes(m.data)
    raise RuntimeError(f"timeout op=0x{op:02X}")
```

What happens:
1. Host sends a CAN frame on `0x600` with `[Axis, Opcode, ...payload]`
2. Host blocks, polling for a reply on `0x580`
3. Juno MCU receives the frame, executes the opcode, sends response
4. Host returns the response bytes

Typical round-trip latency: **~350–450 µs** for reads, **~720 µs** for writes
(measured in `smoketest_results/stage_telemetry_reference.txt`).

### 2.4 Signed 32-bit Decoding (`s32()`)

Position values span millions of encoder counts (±200,000 counts/mm ×
several mm). The Juno returns these as signed big-endian integers with
only as many bytes as needed:

```python
def s32(d):
    body = d[2:]          # skip [Axis, Opcode]
    if not body:
        return 0
    sign = b"\xff" if body[0] & 0x80 else b"\x00"
    return struct.unpack(">i", sign * (4 - len(body)) + body)[0]
```

This pads the response to 4 bytes with sign extension, then interprets as
a signed 32-bit big-endian integer (`>i`).

---

## 3. Opcode Reference

### 3.1 Write Commands (configure and move the stage)

| Opcode | Name | Payload | Description |
|--------|------|---------|-------------|
| `0x10` | SetPosition | 4 bytes (`>i`) | Write target position in encoder counts |
| `0x11` | SetVelocity | 4 bytes (`>i`) | Write velocity in counts/sample |
| `0x1A` | Update | none | Execute buffered position/velocity. *Must be called after SetPosition for motion to begin.* |
| `0x65` | SetOpMode | 1 byte | Switch operating mode (calibration, full, etc.) |
| `0x77` | SetMotorCmd | 2 bytes (`>h`) | Direct voice-coil force command (bypasses servo) |
| `0x90` | SetAcceleration | 4 bytes (`>i`) | Write acceleration in counts/sample² |
| `0x91` | SetDeceleration | 4 bytes (`>i`) | Write deceleration in counts/sample² |
| `0x34` | ResetEvent | none | Clear event status register |
| `0xF5` | CalAnalog | none | Trigger analog calibration routine |

### 3.2 Read Commands (poll position, velocity, status)

| Opcode | Name | Resp Bytes | Description |
|--------|------|------------|-------------|
| `0x37` | GetActualPosition | 4 | Current encoder position (counts) |
| `0x1E` | GetTargetPosition | 6 | Final destination (the value last sent via SetPosition) |
| `0x4A` | GetCommandedPosition | 6 | Trajectory planner's instantaneous desired position; difference from `0x37` = following error |
| `0xAD` | GetActualVelocity | 6 | Velocity in encoder counts per servo sample (51 µs) |
| `0xB6` | GetMotorCommand | 2 | Signed 16-bit voice-coil force proxy (higher = more force) |
| `0x4B` | GetIntegrationStep | 6 | Servo loop integrator accumulation |
| `0x35` | GetEventStatus | 2 | Event flags (motion complete, error, etc.) |
| `0xA6` | GetActivityStatus | 4 | Activity state (in-motion, settling, idle) |
| `0xA7` | GetSignalStatus | 6 | Signal quality / limit switch / fault flags |

### 3.3 Operating Modes

| Mode | Value | Description |
|------|-------|-------------|
| `OPMODE_CAL` | `0x06` | Calibration mode — used during analog calibration |
| `OPMODE_FULL` | `0x37` | Full servo mode — normal operation with PID + trajectory planner |

`init_drive()` in `dof_init.py` sequences: analog calibration → full servo → home.

---

## 4. Command Execution Model

### 4.1 The SetPosition + Update Two-Step

The Juno MCU buffers commands. `SetPosition` (0x10) writes a target into a
register — the stage does *not* move yet. `Update` (0x1A) tells the MCU to
execute the buffered target. This separation allows configuring position,
velocity, and acceleration in any order before triggering motion.

**Every motion command in the codebase follows this pattern:**

```python
dof_init.sr(bus, 0x10, struct.pack(">i", target_counts))   # SetPosition
dof_init.sr(bus, 0x1A)                                      # Update → MOVE NOW
```

Two CAN round-trips (~720 µs total). The stage begins the trajectory
immediately after receiving `Update`.

### 4.2 Trajectory Planner

After `Update`, the Juno MCU's on-board trajectory planner computes a
velocity profile:

```
Velocity
  ▲
  │      ╱‾‾‾‾‾‾‾‾‾╲
  │     ╱            ╲
  │    ╱              ╲
  │   ╱                ╲
  │  ╱                  ╲
  ──┴───────────────────────► Time
      acceleration   deceleration
      limited        limited
```

- Short moves (< ~40 µm): triangular profile — never reaches cruise velocity
- Longer moves: trapezoidal profile — acceleration → cruise → deceleration

The planner respects `SetVelocity`, `SetAcceleration`, and `SetDeceleration`
values. Defaults used in our system: **125 mm/s velocity, 6000 mm/s² acceleration**.

### 4.3 Servo Loop

The MCU runs a PID servo loop at approximately **19.6 kHz** (every 51 µs).
Each iteration:
1. Polls the incremental encoder
2. Computes position error vs the trajectory planner's commanded position
3. Adjusts the voice-coil current to minimize error

The `GetIntegrationStep` (0x4B) opcode reveals the integrator state —
useful for diagnosing steady-state load compensation.

---

## 5. Position Polling

### 5.1 The Simple Path

```python
pos = dof_init.get_pos_counts(bus)
# → s32(sr(bus, 0x37))
# → ~375 µs round-trip
```

One CAN read, ~375 µs. Returns the encoder's absolute position in counts.
Convert to mm: `pos / 200_000`.

### 5.2 The Latency-Benchmark Polling Loop

Found in `run_trial()` (autofocus_latency_webapp.py, dof_latency_bench.py):

```python
while True:
    curr_position = dof_init.get_pos_counts(bus)   # poll
    t_curr = time.perf_counter_ns()

    # detect first motion
    if t_react is None and abs(curr_position - home) > 3:
        t_react = t_curr

    # detect near-target arrival
    if t_engage is None and abs(curr_position - target) <= 50:
        t_engage = t_curr

    # detect settle (within ±3 counts for 5 ms)
    if abs(curr_position - target) <= 3:
        if settle_start is None:
            settle_start = t_curr
        elif (t_curr - settle_start) / 1e9 >= 0.005:
            t_complete = t_curr
            break
    else:
        settle_start = None

    # safety timeout
    if (t_curr - t_cmd) / 1e9 > 0.5:
        break
```

This tight loop polls as fast as CAN allows (~2.7 kHz), producing four
event timestamps:

| Event | Meaning | Typical value |
|-------|---------|---------------|
| `t_cmd` | Command sent | t=0 |
| `t_react` | Stage first moves (3 counts = 15 nm) | +2.5 ms |
| `t_engage` | Arrives within 50 counts of target | +distance-dependent |
| `t_complete` | Settled within 3 counts for 5 ms | +distance-dependent |

### 5.3 Velocity Polling

Two methods exist:

**Classical (Δpos/Δtime):** Compute velocity from subsequent position reads.
Used by default. Simple, no extra opcode.

**Polled (0xAD):** Use `GetActualVelocity` to read the MCU's internal
velocity register directly:
```python
vel_counts = s32(sr(bus, 0xAD))
vel_mm_s = vel_counts / 200_000 / 51e-6
```
Returns velocity in counts per 51 µs servo sample. ~400 µs round-trip.

---

## 6. Motion Parameters

### 6.1 Units

| Constant | Value | Notes |
|----------|-------|-------|
| `COUNTS_PER_MM` | 200,000 | Encoder resolution (5 nm/count) |
| `SAMPLE_S` | 51e-6 | Servo loop period (19.6 kHz) |
| `SOFT_LIMIT_MM` | ±1.2 | Stage won't exceed this via normal commands |
| `dz_mm_per_row` | 0.004593 | OCT image row spacing |

### 6.2 Setting Motion Parameters

```python
def set_motion_params(bus, vel_mm_s=125.0, acc_mm_s2=6000.0):
    vel_counts = int(vel_mm_s * COUNTS_PER_MM * SAMPLE_S)
    acc_counts = int(acc_mm_s2 * COUNTS_PER_MM * SAMPLE_S**2)

    sr(bus, 0x11, struct.pack(">i", vel_counts))    # SetVelocity
    sr(bus, 0x90, struct.pack(">i", acc_counts))    # SetAcceleration
    sr(bus, 0x91, struct.pack(">i", acc_counts))    # SetDeceleration
```

Three CAN round-trips. Parameters persist across `Update` calls until
changed.

### 6.3 Counts ↔ mm Conversion

```python
# mm → counts
target_counts = home_counts + direction * int(error_um * 200)

# counts → mm
position_mm = counts / 200_000
position_um = counts / 200          # same value, different unit
```

Key: 1 µm = 200 counts. The `* 200` in the mm→counts formula converts µm
to counts.

---

## 7. Detection Bands (from latency benchmarking)

Three concentric bands around the target define four motion phases:

```
                     ◄──────────────── total move distance ────────────────►
    home ───────────┼────────────────────────────────────────────┼────── target
                    │                                            │
         ┌──────────┼────────────────────────────────────────────┼──────┐
         │ NOISE    │              MOTION ZONE                    │SETTLE│
         │ BAND     │  > 3 counts from home AND > 50 from target │ BAND │
         │ ±3 cn    │                                            │±3 cn │
         └──────────┼────────────────────────────────────────────┼──────┘
                    │                                            │
                    └── ENGAGEMENT BAND ─────────────────────────┘
                       ≤ 50 counts from target

Events:
  t_cmd ──► t_react (leaves noise band) ──► t_engage (enters engagement band) ──► t_complete (settle)
```

| Band | Width (counts) | Width (nm) | Purpose |
|------|---------------|------------|---------|
| Noise | ±3 | ±15 | Dead zone — too small to distinguish from encoder noise |
| Engagement | ±50 | ±250 | Near-target arrival — "we're close enough to start counting settle time" |
| Settle | ±3 | ±15 | Final lock — must hold for 5 ms continuous |

---

## 8. Initialization Sequence

`init_drive(bus)` in `dof_init.py` performs the full boot sequence:

```
1. CAN bus open (bus already connected by main())
2. OPMODE_CAL (0x06)  →  switch to calibration mode
3. CAL_ANALOG (0xF5)  →  run analog calibration
4. OPMODE_FULL (0x37) →  switch to full servo mode
5. RESET_EVENT (0x34) →  clear any stale events
6. SET_VELOCITY (0x11) → 125 mm/s default
7. SET_ACCEL (0x90)    → 6000 mm/s² default
8. SET_DECEL (0x91)    → 6000 mm/s² default
9. Home command        →  move to zero position
```

The stage must be initialized once after power-on. Subsequent commands
don't require re-initialization.

---

## 9. Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| CAN over RS-232/485 | 1 Mbps vs 115k bps; multi-drop bus; Linux `socketcan` is native, stable, and handles real-time queuing |
| Big-endian (`>i`) encoding | Juno MCU uses network byte order; `struct.pack(">i", val)` matches its expectation |
| SetPosition/Update as separate opcodes | Allows atomic configuration of position + velocity + acceleration before motion triggers |
| poll position in tight loop (not interrupts) | Simpler, no event handling edge cases; 375 µs poll latency is fine for 0.5s timeout |
| 3-count noise band (15 nm) | Confirmed by smoke tests — real encoder noise floor is ~1-2 counts |

---

## 10. Files

| File | Role |
|------|------|
| `scripts/dof_init.py` | CAN opcodes, `sr()`, `s32()`, `get_pos_counts()`, `set_motion_params()`, `init_drive()` — source of truth for all stage communication |
| `scripts/autofocus_latency_webapp.py` | Flask server that receives OCT frames, runs `detect()`, commands the DOF stage, returns latency JSON |
| `smoketest_results/stage_telemetry_reference.txt` | Verified opcode → response size → latency table |
| `doc/latency_benchmark_plan.md` | Project brief for latency characterization |
| `doc/latency_report.md` | Full analysis of reaction/motion/settle phases |
| `config.yaml` | control parameters (focus line, deadband, max move, EMA alpha) |
| `README.md` | Hardware setup, wiring pinout, first-time initialization |
