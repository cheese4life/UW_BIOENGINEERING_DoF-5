# Live Simulation Engine — Architecture

## Purpose

A standalone frame server that plays back pre-generated synthetic OCT frames
at a configurable FPS. LabVIEW polls it for frames and feeds them into the
existing `autofocus_latency_webapp.py` `/focus_live` endpoint.

The simulation has **one job**: produce frames with realistic corneal motion
and serve them over HTTP. Nothing else changes in the existing pipeline.

## Architecture Diagram

```
┌─────────────────────────┐      ┌──────────────────────────────┐
│  Simulation Engine      │      │  autofocus_latency_webapp.py │
│  (NEW - standalone)     │      │  (UNCHANGED)                 │
│                         │      │                              │
│  Web UI for params      │      │  /focus_live  ← LabVIEW      │
│  GET /live_frame → PNG  │──┐   │  /focus       ← LabVIEW      │
│                         │  │   │                              │
└─────────────────────────┘  │   └──────────────────────────────┘
                             │
                             │   LabVIEW reads frame from
                             │   simulation, POSTs it to
                             └── webapp's /focus_live
```

## Data Flow (LabVIEW integration, unchanged)

```
1. LabVIEW polls sim_server: GET /live_frame → gets img_b64
2. LabVIEW POSTs to webapp: /focus_live {"img": img_b64, "velocity_mm_s": 10, ...}
3. Webapp runs detect(), commands DOF stage, returns latency JSON
4. LabVIEW records/reports the result
```

## Files

| File | Purpose |
|------|---------|
| `scripts/motion_model.py` | `PatientMotionModel` — drift + tremor + microsaccades + physio rhythms |
| `scripts/generate_patient_sim.py` | CLI: runs motion model against a cornea sample, writes frames to disk |
| `scripts/sim_server.py` | Flask app: mmap frame loader, timer thread, HTTP endpoints, web UI |

## HTTP Endpoints

```
GET /live_frame
  → {"img_b64": "...", "frame_idx": 1842, "time_s": 4.605}

GET /state
  → {"fps": 100, "sample": "cornea_1", "total_frames": 12000, "playing": true}

POST /set_params  {"fps": 100, "sample": "cornea_1"}
  → {"ok": true}

GET /
  → HTML page: OCT preview, FPS slider (10-400), play/pause, sample selector
```

## Frame Skipping Strategy

```
Master frames generated at 400 fps (ground truth)
Playback timer fires at target_fps Hz
On each tick: frame = master_frames[int(elapsed_sec * 400)]

→ at 400 fps: every frame shown (full richness, tremor visible)
→ at 100 fps: every 4th frame (75% of high-frequency detail lost)
→ at 10 fps:  every 40th frame (tremor + microsaccades invisible)
```

This directly models the clinical reality: lower-framerate scanners lose
the high-frequency micro-motion detail that a 400 fps scanner captures.

## Motion Model (PatientMotionModel)

### Components

| Component | Behavior | Parameters |
|-----------|----------|------------|
| **Drift** | Ornstein-Uhlenbeck process (mean-reverting random walk). Slow wandering away from center, gently pulled back toward zero. | σ (volatility), θ (reversion strength), bounds ±50 px |
| **Tremor** | High-frequency micro-oscillation at ~87 Hz. Amplitude ~0.15 px. Adds texture to motion that is invisible to naked eye but detectable by CV at high FPS. | frequency, amplitude |
| **Microsaccades** | Ballistic jumps 1-3 per second. Jump → exponential decay return. Dominant visible feature. | inter-event interval (exponential, mean ~0.5s), amplitude (truncated normal, μ=3 px, σ=3 px, clip 1-10 px), decay τ (0.05-0.15s) |
| **Physiological rhythms** | Breathing (~0.25 Hz, ~0.5 px) + heartbeat (~1.2 Hz, ~0.3 px). Gentle periodic baseline oscillation with slight random phase jitter. | frequency, amplitude per rhythm |

### Hard Constraints

| Constraint | Value | Reason |
|------------|-------|--------|
| Max shift | ±50 px (±230 µm) | Flat cap in both directions — cornea stays fully in frame with margin |
| Blinking | Out of scope | Edge case; implement core focusing pipeline first |

### Patient Profiles

Two presets that tune the motion model parameters for different clinical scenarios:

| Parameter | Calm Patient | Anxious Patient |
|-----------|-------------|-----------------|
| Drift σ | 5.0 | 12.0 |
| Drift θ | 0.03 (slow return) | 0.08 (faster tug back) |
| Tremor amplitude | 0.10 px | 0.20 px |
| µSaccade rate | 1.0 Hz (mean interval 1.0s) | 3.0 Hz (mean interval 0.33s) |
| µSaccade amplitude | μ=2, σ=2, clip 1-6 px | μ=5, σ=4, clip 2-10 px |
| µSaccade duration | 20-30 ms | 10-20 ms (faster, sharper) |
| Breathing amplitude | 0.4 px | 0.8 px |
| Heartbeat amplitude | 0.2 px | 0.4 px |
| **Expected behavior** | Gentle drift within ±15 px, occasional small saccades, smooth tracking easy | Wild excursions ±30-40 px, frequent large saccades, hard to lock focus |

### Out of Scope (explicitly deferred)

- Blinking / eye closure
- Tear film artifacts
- Instrument motion (camera shake)
- Multi-layer corneal reflections

### Sub-pixel Shifts

`cv2.warpAffine` with `INTER_CUBIC` and float-valued shift matrix handles
fractional shifts naturally. The background canvas fill (top/bottom noise
strips) only triggers when `abs(shift) >= 1.0` pixels — fractional shifts
are handled entirely by the interpolation kernel.

This allows modeling movements as fine as 0.15 px (tremor amplitude) at
400 fps without producing visual artifacts.

### Key Numbers (from project config)

| Constant | Value |
|----------|-------|
| `dz_mm_per_row` | 0.004593 mm/row (4.593 µm/row) |
| `focus_row` | 150 |
| Frame dimensions | ~256 rows × ~512 cols (float32) |
| Cornea samples | cornea_1 .. cornea_4 in `data/samples/` |
| Master FPS | 400 (ground truth generation rate) |

### Disk Budget

12,000 frames × 256 × 512 × 4 bytes ≈ 6.3 GB for 30 seconds at 400 fps.
Acceptable for local dev machine. Frames loaded via mmap — never all in RAM.

### On-the-Fly Mode

For longer sessions or constrained disk, the engine also supports generating
frames on-the-fly (no pre-generation). The motion model computes `shift_px(t)`
at the playback FPS, applies `warpAffine` to the reference image, and serves
the result directly. This avoids storing thousands of `.npy` files but adds
~1-3 ms of CPU per frame. Both modes (pre-generated + on-the-fly) are supported
and selectable via the web UI.

## What Does NOT Change

- `autofocus_latency_webapp.py` — untouched
- `scripts/dof_init.py` — untouched
- `cornea_focus/surface.py` — untouched
- `cornea_focus/control.py` — untouched
- `cornea_focus/dof_driver.py` — untouched
- `cornea_focus/generate_sim.py` — untouched
- `play_sim_with_dof.py` — untouched
