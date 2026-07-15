# Motion Model — Patient Corneal Dynamics

> **Purpose:** Generate realistic, physics-grounded shift trajectories that
> mimic human corneal micro-movements during an OCT scan. Fed into
> `cv2.warpAffine` to produce synthetic frames for the simulation engine.

---

## 1. Composite Shift Function

```
shift_px(t) = clamp(drift(t) + tremor(t) + microsaccades(t) + physio(t),  -50, +50)
```

Four independent components summed, then hard-clamped at ±50 px (±230 µm).
The clamp exists to guarantee the cornea never shifts off the image canvas.

---

## 2. Component 1 — Drift (Ornstein-Uhlenbeck Process)

### 2.1 Rationale

A mean-reverting random walk is the standard model for fixational eye drift.
The OU process produces slow, smooth wandering with a gentle pull back toward
center — exactly what an ophthalmologist observes in a patient trying (but
failing) to hold perfectly still.

### 2.2 Discrete-time Step

```
x(t+dt) = x(t) · e^(-θ·dt) + σ · √((1 - e^(-2θ·dt)) / (2θ)) · N(0,1)
```

This is the **exact** solution to the OU SDE (not Euler-Maruyama approximation),
so the step is unbiased regardless of `dt`.

| Symbol | Meaning | Units |
|--------|---------|-------|
| `x(t)` | Current drift offset | px |
| `θ` | Mean-reversion strength | s⁻¹ |
| `σ` | Volatility (noise intensity) | px·s⁻⁰·⁵ |
| `dt` | Time step (1/fps) | s |
| `N(0,1)` | Standard normal draw | — |

Steady-state standard deviation: **σ / √(2θ)** px

### 2.3 Profiles

| Parameter | Calm | Anxious |
|-----------|------|---------|
| `θ` | 0.03 | 0.08 |
| `σ` | 5.0 | 12.0 |
| Steady-state σ | ~20 px | ~30 px |
| Typical range | ±10 to ±20 px | ±25 to ±45 px |
| e^(-θ·dt) at 400fps | 0.999925 | 0.999800 |

### 2.4 Boundary Soft-Push

When `abs(x) > 48` px, apply an extra reversion per frame:
```
x *= 0.98
```
This prevents the process from getting "stuck" at the ±50 px clamp boundary
where the OU reversion alone is too weak to overcome the noise term.

### 2.5 State (on-the-fly mode)

```
drift_x: float          # current offset
drift_rng: Generator    # seeded numpy Generator
```

---

## 3. Component 2 — Tremor (87 Hz Micro-Oscillation)

### 3.1 Rationale

Human eyes exhibit a physiological micro-tremor at ~87 Hz with an amplitude
of ~0.15 photoreceptor widths. In our OCT coordinate system this translates
to ~0.15 px peak. It's invisible to the naked eye but introduces high-frequency
"texture" in the surface detection signal.

At lower playback FPS this component is undersampled and aliases into
pseudo-random noise — which is exactly what happens with a real low-FPS
OCT scanner. This is the primary source of **richness loss** at reduced FPS.

### 3.2 Formula

```
tremor(t) = A · sin(2π · 87 · t + φ)
```

| Parameter | Calm | Anxious |
|-----------|------|---------|
| `A` (amplitude) | 0.10 px | 0.20 px |
| Frequency | 87 Hz | 87 Hz |

### 3.3 Phase Jitter

Every 2-3 seconds (uniform draw), the phase `φ` is re-randomized. To maintain
**C⁰ continuity** (no visual glitch at the boundary):

```
At epoch boundary t = t_b:
  v = A · sin(2π·87·t_b + φ_old)
  φ_new = arcsin(v/A) - 2π·87·t_b
```

Both branches of arcsin are evaluated; the one whose derivative sign matches
the old derivative is selected. This ensures the value is identical at the
boundary — only the slope changes, producing a subtle, organic irregularity.

### 3.4 State

```
tremor_phase: float         # current φ
tremor_epoch_end: float     # time of next phase reset
tremor_rng: Generator
```

---

## 4. Component 3 — Microsaccades (Ballistic Jumps)

### 4.1 Rationale

Microsaccades are the dominant visible feature in corneal motion tracking.
They are involuntary, ballistic (no visual feedback during execution),
and occur 1-3 times per second. Each one is a rapid linear displacement
followed by a slower exponential return toward baseline.

### 4.2 Event Model

```
Inter-event interval:  I ~ Exponential(1 / rate)        [clamped ≥ 0.2s refractory]
Amplitude:             A ~ TruncatedNormal(μ, σ, a_min, a_max)
Direction sign:        D = sign(drift_x) with P=0.6, else -sign(drift_x)
Jump duration:         T_jump ~ Uniform(t_jump_min, t_jump_max)
Return time constant:  τ ~ Uniform(τ_min, τ_max)
```

| Parameter | Calm | Anxious |
|-----------|------|---------|
| `rate` (Hz) | 1.0 | 3.0 |
| `μ` (px) | 2 | 5 |
| `σ` (px) | 2 | 4 |
| `a_min` (px) | 1 | 2 |
| `a_max` (px) | 6 | 10 |
| `t_jump_min` (ms) | 20 | 10 |
| `t_jump_max` (ms) | 30 | 20 |
| `τ_min` (s) | 0.05 | 0.04 |
| `τ_max` (s) | 0.15 | 0.08 |

### 4.3 Shape of a Single Microsaccade

```
           ↑
    A ·|   ╱╲
       |  ╱  ╲___
       | ╱       ╲___  exponential decay
       |╱           ╲_____________
  ─────┼──────────────────────────→ time
       t0  t1      t1+τ   t1+3τ
       │← T_jump →│
```

**Jump phase** (t ∈ [t0, t1]):
```
sacc(t) = A · D · (t - t0) / T_jump
```

**Return phase** (t ∈ [t1, ∞)):
```
sacc(t) = A · D · exp(-(t - t1) / τ)
```

Truncated when `exp(-(t-t1)/τ) < 0.05` (≈ 3τ), at which point the event
is removed from the active list.

### 4.4 Overlapping Events

If a new microsaccade fires while a previous one is still in its return
phase, the new event **launches from the residual**:

```
At new event time t0_new:
  residual = Σ active_returns(t0_new)
  Jump:      sacc(t) = residual + A·D·(t - t0_new)/T_jump          for t ∈ [t0, t1]
  Return:    sacc(t) = (residual + A·D) · exp(-(t - t1)/τ)         for t > t1
```

This prevents visual discontinuities when saccades cluster.

### 4.5 Refractory Period

Minimum 200 ms between consecutive microsaccade initiations. If the
exponential draw yields `I < 0.2`, it is redrawn.

### 4.6 State

```
active_saccades: list[MicroSaccade]
  where MicroSaccade = (t0, t1, amplitude, direction, tau)

next_event_time: float
saccade_rng: Generator
```

---

## 5. Component 4 — Physiological Rhythms

### 5.1 Rationale

Head motion from breathing and heartbeat couples into the OCT image as a
slow, quasi-periodic baseline oscillation. These rhythms are slow enough
to be visible even at 10 fps.

### 5.2 Formula

```
physio(t) = A_breath · sin(2π · f_breath · t + φ_breath)
          + A_heart  · sin(2π · f_heart  · t + φ_heart)
```

| Parameter | Calm | Anxious | Units |
|-----------|------|---------|-------|
| `f_breath` | 0.25 | 0.25 | Hz |
| `A_breath` | 0.4 | 0.8 | px |
| `f_heart` | 1.2 | 1.2 | Hz |
| `A_heart` | 0.2 | 0.4 | px |

### 5.3 Phase Jitter

At each zero-crossing of either sinusoid, add `N(0, 0.05)` rad to its
phase. This makes breathing slightly irregular (as in real patients)
without breaking continuity.

### 5.4 State

```
breath_phase: float
heart_phase: float
breath_last_jitter_t: float
heart_last_jitter_t: float
physio_rng: Generator
```

---

## 6. Putting It All Together

### 6.1 Execution Order (per frame)

```
1. drift_x     = ou_step(drift_x, dt, θ, σ, drift_rng)
2. drift_x     = soft_push(drift_x)            # if near ±48
3. trem_val    = tremor(t, phase)
4. sacc_val    = sum_active_saccades(t)        # schedule new if t ≥ next_event
5. phys_val    = physio(t, breath_phase, heart_phase)
6. shift       = clamp(drift_x + trem_val + sacc_val + phys_val, -50, +50)
7. frame       = warpAffine(ref, shift)        # cv2 with INTER_CUBIC
```

### 6.2 Visual Summary (conceptual)

```
Time →
─────────────────────────────────────────────────────────────────
drift:         ~~~~────────~~~~────────~~~~────────────~~~~       (±20px calm, ±35px anxious)
tremor:        ∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿∿    (±0.15px @ 87Hz)
µsaccades:     |↑↓|        |↑|     |↓↓|          |↑↑↑|           (1-3/sec)
physio:        ~~~~~~~~~  ~~~~~~  ~~~~~~~~~  ~~~~~~             (±0.6px combined)
─────────────────────────────────────────────────────────────────
composite:     ~∿~|↑↓|∿~~∿~~~∿|↑|∿~∿∿|↓↓|∿~~~∿∿|↑↑↑|∿∿∿∿       (organic)
─────────────────────────────────────────────────────────────────

At 400fps:  full composite resolved — tremor visible, µsaccades continuous
At 100fps:  tremor aliased into noise, µsaccades stepwise but discernible
At 10fps:   only drift + physio visible, µsaccades as mysterious teleports
```

### 6.3 Validation Checks (to add in tests)

| Check | Expected |
|-------|----------|
| All shifts clamped to [-50, +50] | No output exceeds bounds |
| C⁰ continuity | No instantaneous jumps between frames (except saccade initiations) |
| Seed determinism | Same seed → same trajectory |
| Profile difference | Anxious has larger variance, more saccades than Calm |
| Steady-state drift | OU process variance converges to σ²/(2θ) over long horizon |
| Refractory period | No two saccades initiated within 200ms |

---

## 7. Frame Generation (Warping)

### 7.1 Algorithm (per frame)

```python
# From cornea_focus/generate_sim.py — the proven technique
M = np.float32([[1, 0, 0], [0, 1, shift]])            # float shift
shifted = cv2.warpAffine(
    ref, M, (W, H),
    borderMode=cv2.BORDER_CONSTANT,
    borderValue=0,
    flags=cv2.INTER_CUBIC                             # smooth sub-pixel
)

exposed = int(np.ceil(abs(shift)))
bg_top = max_shift - int(round(shift))                # scrolls with cornea
bg = big_background[bg_top : bg_top + H]

if shift > 0:
    shifted[:exposed] = bg[:exposed]
elif shift < 0:
    shifted[H - exposed:] = bg[H - exposed:]
```

### 7.2 Background Canvas

Built once per sample:
- Sample top/bottom `STRIP=20` rows for noise statistics (mean, std)
- Build `TOTAL_H = H + 2*max_shift = H + 100` rows
- Linear blend of top/bottom statistics across the canvas
- The **same canvas slice** follows the cornea shift — noise texture never
  slides against tissue (no "static on a moving TV" effect)

### 7.3 Sub-pixel Behavior

For `|shift| < 1.0`: `warpAffine` with `INTER_CUBIC` handles everything.
`exposed = 1` so one row gets background fill. At 512 columns wide, this
is imperceptible.

For `|shift| ≥ 1.0`: `exposed` rows filled with the correctly-positioned
background window. Transitions smoothly across integer boundaries.

---

## 8. API (`MotionModel` class)

```python
class MotionModel:
    PROFILES = {
        "calm":    {...},   # all parameter dicts
        "anxious": {...},
    }

    def __init__(self, profile: str = "calm", seed: int = 42):
        """Load profile params and initialize all RNGs and state."""

    def reset(self):
        """Reset all state to t=0, keeping the same seed base."""

    def shift_at(self, t_sec: float) -> float:
        """Return shift_px at time t, updating internal state.
        Thread-safe for single-producer use."""

    def generate_trajectory(self, n_frames: int,
                            fps: float = 400.0) -> np.ndarray:
        """Pre-generate a full shift array of shape (n_frames,).
        Used by the disk-generation script."""

    @property
    def state_dict(self) -> dict:
        """Serializable snapshot of all internal state (for debugging)."""
```
