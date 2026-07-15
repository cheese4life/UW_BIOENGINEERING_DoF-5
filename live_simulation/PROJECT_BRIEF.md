# Live Simulation Engine — Project Brief

> **Status:** Design phase — motion model finalized, pending implementation.
> **Last updated:** 2026-07-14

---

## 1. Elevator Pitch

**One sentence:** A standalone OCT frame server that plays back synthetic
corneal scans with realistic patient motion, enabling LabVIEW autofocus
development against a repeatable, physics-grounded test harness.

---

## 2. Why This Exists

The DOF-5 autofocus pipeline needs a live OCT feed to test against. The real
OCT scanner isn't ready yet. Rather than block LabVIEW integration, we build
a simulator that:

- Produces frames indistinguishable (to the CV pipeline) from a real scanner
- Models the motion characteristics of different patient types
- Allows FPS to be dialed from 10–400 to test how frame rate affects focus lock
- Plugs into the **existing** `/focus_live` API — zero changes to LabVIEW wiring

When the real scanner arrives, we swap the simulation URL for the scanner URL.
Everything else stays the same.

---

## 3. Architecture

```
┌─────────────────────────┐      ┌──────────────────────────────┐
│  sim_server.py          │      │  autofocus_latency_webapp.py │
│  (NEW — this project)   │      │  (UNCHANGED)                 │
│                         │      │                              │
│  GET /live_frame → PNG  │──┐   │  /focus_live  ← LabVIEW      │
│  GET /state             │  │   │  /focus       ← LabVIEW      │
│  POST /set_params       │  │   │                              │
│  GET / (web UI)         │  │   └──────────────────────────────┘
└─────────────────────────┘  │
                             │   LabVIEW: GET frame from sim,
                             │   POST frame to webapp's /focus_live
                             └── webapp: runs detect(), commands stage,
                                          returns latency JSON
```

### 3.1 What We Build (3 files)

| File | Role | Lines (est.) |
|------|------|-------------|
| `scripts/motion_model.py` | `MotionModel` class — OU drift, tremor, microsaccades, physio rhythms. Pure numpy. | ~300 |
| `scripts/generate_patient_sim.py` | CLI that runs the motion model against a cornea sample, writes all frames + manifest to disk. | ~150 |
| `scripts/sim_server.py` | Flask app — mmap frame loader, timer thread, `/live_frame` endpoint, web UI with FPS slider and profile picker. | ~400 |

### 3.2 What We Don't Touch

| File | Status |
|------|--------|
| `autofocus_latency_webapp.py` | Untouched — LabVIEW is wired to it |
| `scripts/dof_init.py` | Untouched |
| `cornea_focus/surface.py` | Untouched |
| `cornea_focus/control.py` | Untouched |
| `cornea_focus/dof_driver.py` | Untouched |
| `cornea_focus/generate_sim.py` | Untouched |
| `play_sim_with_dof.py` | Untouched |

---

## 4. HTTP API (what LabVIEW talks to)

### `GET /live_frame`

The only endpoint LabVIEW needs. Returns the current frame as base64 PNG.

```json
{
  "img_b64": "iVBORw0KGgo...",
  "frame_idx": 1842,
  "time_s": 4.605,
  "shift_px": 12.34,
  "position_mm": 0.0567
}
```

LabVIEW polls this at its loop rate, gets the frame, POSTs it to the existing
webapp. The simulation doesn't need to know what LabVIEW does with it.

### `GET /state`

Diagnostic endpoint — current engine state.

```json
{
  "fps": 100,
  "sample": "cornea_1",
  "profile": "anxious",
  "total_frames": 12000,
  "current_frame": 1842,
  "playing": true,
  "mode": "pre-generated"
}
```

### `POST /set_params`

Adjust playback parameters at runtime.

```json
// Request
{"fps": 200, "profile": "calm", "sample": "cornea_2"}

// Response
{"ok": true}
```

### `GET /` — Web UI

Minimal HTML page for the operator:
- OCT preview (updates at ~30fps via `setInterval`)
- FPS slider: 10–400
- Profile selector: calm / anxious
- Sample selector: cornea_1–4
- Play / Pause / Reset buttons
- Current shift, position, frame index readout

---

## 5. Motion Model (see `MOTION_MODEL.md` for full details)

```
shift_px(t) = clamp(drift(t) + tremor(t) + microsaccades(t) + physio(t), -50, +50)
```

| Component | Model | Calm | Anxious |
|-----------|-------|------|---------|
| Drift | Ornstein-Uhlenbeck | σ=5, θ=0.03 (wanders ±20px) | σ=12, θ=0.08 (wanders ±35px) |
| Tremor | 87 Hz sinusoid | ±0.10 px | ±0.20 px |
| Microsaccades | Ballistic jump + exp decay | 1 Hz, 1–6 px jumps | 3 Hz, 2–10 px jumps |
| Physio | Breathing (0.25 Hz) + heartbeat (1.2 Hz) | ±0.6 px combined | ±1.2 px combined |

---

## 6. Frame Generation

### 6.1 Pre-Generated Mode (disk)

```
generate_patient_sim.py --profile anxious --sample cornea_1 --duration 30

Output:
  data/patient_sim/
    frame_000000.npy  ...  frame_011999.npy    (12,000 frames)
    manifest.csv       (frame_idx, shift_px, position_mm, time_s)
    motion_config.json (profile, seed, all params)
```

- 30s × 400fps = 12,000 frames
- ~6.3 GB disk (256×512 float32 × 12k)
- Loaded via `numpy.memmap` in playback — never all in RAM
- Same format as existing `data/sim/` directory

### 6.2 On-the-Fly Mode (no disk)

For longer sessions or constrained disk, the engine generates frames
in real-time: `shift_px = model.shift_at(t)` → `warpAffine` → serve PNG.
Adds ~2ms CPU per frame. Selectable via web UI.

### 6.3 Warping Technique

Reuses the proven approach from `cornea_focus/generate_sim.py`:
- `cv2.warpAffine` with `INTER_CUBIC` for sub-pixel precision
- Single oversized background canvas (H + 100 rows) built from sample noise stats
- Background window scrolls *with* the cornea — no texture sliding

---

## 7. Frame Rate & Richness Loss

The master frames are generated at 400 fps (the "ground truth"). The playback
timer fires at the configured target FPS. On each tick:

```
master_index = int(elapsed_seconds * 400)
current_frame = master_frames[master_index]
```

| Playback FPS | Stride | Detail retained | Clinical analog |
|-------------|--------|----------------|-----------------|
| 400 | 1× | 100% — tremor, µsaccades, all fully resolved | Research-grade scanner |
| 200 | 2× | Tremor partially aliased | High-end clinical |
| 100 | 4× | Tremor aliased to noise, µsaccades stepwise | Standard clinical |
| 30 | 13× | Only drift + physio clearly visible | Older systems |
| 10 | 40× | Cornea appears to "jump" between frames | Very old / low-cost |

This directly models the clinical reality: lower FPS loses high-frequency
micro-motion detail. The autofocus algorithm will perform differently at
each setting — exactly what we want to characterize.

---

## 8. Implementation Order

| # | Task | Risk | Dependencies |
|---|------|------|-------------|
| 1 | `motion_model.py` — `MotionModel` class with all 4 components | Medium | numpy only |
| 2 | Unit tests — trajectory validation, seed determinism, boundary checks | Low | #1 |
| 3 | `generate_patient_sim.py` — CLI generator script | Low | #1, cornea samples |
| 4 | Generate 30s datasets for both profiles × 4 samples | Low | #3 |
| 5 | `sim_server.py` — Flask app + mmap loader + timer thread | Low | #4 |
| 6 | Web UI — HTML page with preview, sliders, controls | Low | #5 |
| 7 | Integration test — sim → LabVIEW → webapp → stage (end-to-end) | Low | #6 |

---

## 9. Success Criteria

- [ ] `generate_patient_sim.py --profile calm --duration 30` produces 12,000 valid `.npy` frames
- [ ] `generate_patient_sim.py --profile anxious --duration 30` produces visually distinct, more aggressive motion
- [ ] `sim_server.py` serves frames at configurable 10–400 FPS via `/live_frame`
- [ ] At 400fps: tremor visible as surface detection jitter
- [ ] At 10fps: microsaccades appear as unexplained jumps between frames
- [ ] Same seed → same trajectory (reproducible testing)
- [ ] LabVIEW can poll `/live_frame` and POST to existing webapp without modification
- [ ] Zero changes to `autofocus_latency_webapp.py`, `dof_init.py`, or any `cornea_focus/` module

---

## 10. Key Decisions Log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-07-14 | ±50 px hard clamp on all motion | Keeps cornea fully in frame with margin |
| 2026-07-14 | Two profiles (calm / anxious) | Covers easy and hard tracking scenarios |
| 2026-07-14 | Blinking out of scope | Edge case; implement core focus pipeline first |
| 2026-07-14 | Master FPS fixed at 400 | Highest plausible clinical framerate; sub-multiples via stride |
| 2026-07-14 | Simulation is standalone server | Zero coupling to existing webapp; LabVIEW bridges them |
| 2026-07-14 | Pre-generated frames on disk + on-the-fly mode | Disk for perf, on-the-fly for flexibility |
| 2026-07-14 | `warpAffine` + scrolled background from `generate_sim.py` | Proven technique, no reason to reinvent |
