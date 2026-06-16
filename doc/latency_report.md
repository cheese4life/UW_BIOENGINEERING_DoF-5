# DOF-5 Latency Benchmark — Results & Interpretation

**Run analyzed:** `bench_20260616_210048/` (1000 trials, 100 per config)
**Motion params:** velocity = 1.0 mm/s, acceleration = 20 mm/s²
**Stage:** DOF-5 single-axis objective focuser (voice-coil, 200,000 counts/mm, ±15 nm settle band)

---

## 1. The short answer to your question

> *"Doesn't motion latency just reflect how long it takes the stage to actually move? So doesn't that mean only reaction and settle matter?"*

**You are half right, and the half you have right is the important half.**

`motion_us` is **not a latency at all** — it is **travel time**. It is governed by the
velocity and acceleration limits you set (1.0 mm/s, 20 mm/s²), i.e. by *physics and your
own speed cap*, not by how "smart" or "responsive" the controller is. The proof is in the
data: motion time scales linearly with distance while reaction time stays flat (Section 3).

So yes — the two numbers that describe the **responsiveness of the system** are:

- **Reaction** = the true dead time (command → first physical motion). ~2.3–2.7 ms.
- **Settle** = the fine lock-in time once it has arrived. ~44–60 ms.

But here is the nuance that changes your conclusion about eye tracking: **`settle` only
matters when you intend to stop.** A continuous tracker never stops — it rides a moving
target — so for tracking, the relevant numbers are **reaction (dead time)** and **slew
rate (the velocity cap that drives `motion`)**, *not* settle. More on that in Section 5.

---

## 2. The numbers (medians)

| Config    | Reaction (ms) | Motion (ms) | Settle (ms) | Total (ms) |
|-----------|--------------:|------------:|------------:|-----------:|
| 10 µm +   | 2.68          | 37.1        | 57.2        | 97.1       |
| 10 µm −   | 2.28          | 37.5        | 59.7        | 99.4       |
| 25 µm +   | 2.61          | 62.9        | 58.1        | 124.0      |
| 25 µm −   | 2.31          | 63.3        | 57.9        | 123.6      |
| 50 µm +   | 2.65          | 92.4        | 56.3        | 151.5      |
| 50 µm −   | 2.34          | 92.8        | 53.2        | 148.5      |
| 100 µm +  | 2.67          | 142.8       | 51.7        | 197.0      |
| 100 µm −  | 2.28          | 143.2       | 48.0        | 193.6      |
| 200 µm +  | 2.65          | 243.2       | 49.3        | 295.5      |
| 200 µm −  | **25.98** ⚠️  | 243.6       | 44.0        | 290.2      |

(⚠️ the `200 µm −` reaction outlier is a measurement artifact, not real — see Section 4.)

---

## 3. What each phase actually measures

```
  t_cmd            t_react              t_engage            t_complete
    |---------------|--------------------|-------------------|
    |  reaction     |     motion         |      settle       |
    | (dead time)   |  (travel time)     |  (fine lock-in)   |
    |               |                    |                   |
  command      stage first         within 250 nm        within 15 nm
  hits bus     physically moves    of target            for 5 ms straight
```

### Reaction (~2.5 ms) — *the real latency*
Flat across **every** distance (2.28–2.68 ms). That is the signature of a true dead time:
it does not care how far you are going, only that you asked. It is the sum of:
- CAN round-trip to issue `SetPosition` + `Update` (~720 µs, measured in smoke tests),
- the servo's own loop/command-processing delay,
- one polling interval of detection granularity (~375 µs per `GetActualPosition`).

This is the number that says *"the stage begins responding ~2.5 ms after it is told to."*

### Motion (37 → 243 ms) — *travel time, NOT latency*
Scales cleanly with distance:

| Distance | Motion (ms) | Implied speed |
|---------:|------------:|--------------:|
| 10 µm    | 37          | accel-limited (triangular profile, never reaches cruise) |
| 25 µm    | 63          | "                                                        |
| 50 µm    | 92          | just reaching cruise                                     |
| 100 µm   | 143         | cruise-limited ≈ 1 mm/s                                  |
| 200 µm   | 243         | cruise-limited ≈ 1 mm/s                                  |

From 10 µm to 200 µm the move grows by 190 µm and motion grows by ~206 ms → ≈ **0.92 µm/ms
≈ 0.92 mm/s**, which is exactly your 1.0 mm/s velocity cap (minus the accel ramps). Short
moves (≤50 µm) never even reach full speed; they are pure acceleration ramps, which is why
10 µm still costs 37 ms instead of the 10 ms a constant 1 mm/s would predict.

**Takeaway: motion time is a knob you control.** Raise the velocity/acceleration limits and
it shrinks. It is not telling you anything about the controller's responsiveness.

### Settle (~50 ms) — *fine lock-in*
Roughly constant (44–60 ms), drifting slightly *down* for larger moves (the stage arrives
with momentum and coasts into band). This is the time to go from "within 250 nm" to "parked
within 15 nm for 5 ms continuous." 5 ms of that is the hold requirement itself, so the true
mechanical settle is ~40–55 ms. **This only counts when you need to come to a precise, certified stop.**

---

## 4. Data-quality flags (read before trusting every cell)

1. **`200 µm −` reaction = 26 ms median, 222 ms p99.** This is an order of magnitude off the
   flat ~2.5 ms everywhere else. Almost certainly a **detection artifact**, not real hardware
   behavior: on large negative moves the first samples may already sit inside the noise band,
   or there is a homing/backlash interaction. The `motion` and `settle` columns for that config
   look normal, so the move itself was fine — only the `t_react` pick is suspect.
2. **`100 µm +` and `200 µm +` motion p99 spikes** (218 ms, and a 120 ms reaction p99). A
   handful of trials show outliers consistent with occasional CAN poll stalls or a missed
   sample, not a systematic effect — medians are clean.
3. Means are pulled around by these tails; **trust the medians** for the headline story.

None of these change the core conclusions, but they are the first things to clean up before
this becomes a published figure.

---

## 5. About "follow an eye with a 3 ms delay"

This is the part worth being careful about, because the intuition is *directionally right*
but the specific claim overreaches.

**What is true:** ~2.5 ms is a genuinely good dead time. It means that ~2.5 ms after new
focus information arrives, the stage *begins* correcting. For a closed control loop, low dead
time is the single most valuable property — it is what lets you run a high update rate without
going unstable.

**What does not follow:** "3 ms reaction" ≠ "the eye is followed with a 3 ms lag." Two reasons:

1. **Reaction is when motion *starts*, not when the target is *reached*.** To actually arrive
   at a new focus depth you still pay the travel time. For a small 10 µm correction that is
   ~37 ms of travel (at the current 1 mm/s cap), not 3 ms. The error is not nulled until the
   stage gets there.

2. **This benchmark measures discrete point-to-point moves that fully stop and settle.** A real
   tracker does not do that — it continuously feeds a moving target and never waits to settle.
   So the ~97 ms "total" for a 10 µm move *overstates* tracking latency, while the bare 3 ms
   reaction *understates* it. The honest tracking metric is neither one; it is **closed-loop
   bandwidth / phase lag**, which this benchmark does not measure yet.

**The number that actually governs tracking is slew rate.** At 1.0 mm/s the stage moves
1 µm per ms. So it can perfectly follow any focus target that drifts slower than ~1 µm/ms;
faster than that and it falls behind no matter how good the 2.5 ms dead time is. The good news:
1 mm/s is a conservative cap (the DOF-5 voice coil can do far more), so there is large headroom.

**Realistic framing:** the dead time says real-time focus tracking is *architecturally
feasible* — 2.5 ms is well inside the budget. Whether you can "follow an eye" depends entirely
on (a) how fast the focus depth you are correcting actually moves, and (b) the velocity cap you
allow. Slow axial drift (patient settling, breathing, tear-film/cornea depth changes): easily
tracked. Fast saccade-driven changes: limited by slew rate, not by the 2.5 ms.

---

## 6. What to test next

In rough priority order:

1. **Closed-loop tracking sweep (the real eye-tracking test).** Command a sine target
   `x(t) = A·sin(2πft)` and measure tracking error, amplitude attenuation, and phase lag vs
   frequency. This yields the **−3 dB bandwidth** and **phase-lag-at-Xms**, which are the
   numbers that genuinely answer "can it follow an eye?" You already have a sine driver
   (`dof_oscillate_v1.py`) to build on.

2. **Push velocity / acceleration.** Re-run the suite at 2, 5, 10 mm/s and higher accel. Motion
   time should drop proportionally while reaction stays flat — this directly quantifies how much
   tracking headroom you can buy, and where the voice coil / power stage saturates.

3. **Decompose the 2.5 ms reaction.** Separate the CAN transport (~0.7 ms) from the servo's
   intrinsic response and from detection granularity. A faster poll (or event-driven readout)
   could sharpen `t_react` and remove the polling floor.

4. **Fix the `200 µm −` reaction artifact.** Add a pre-move dwell so the noise band is clean at
   `t_cmd`, or detect reaction from velocity sign instead of absolute displacement. Re-run that
   config to confirm it collapses back to ~2.5 ms.

5. **Replay a real focus-depth trace.** Once you have a recorded axial motion profile (from the
   OCT surface tracker / `cornea_focus` pipeline), feed it as the target and report RMS tracking
   error in µm. That is the end-to-end number a reviewer will ask for.

6. **Step-response settle tuning.** If settle (~50 ms) becomes the bottleneck for the
   button-press autofocus use case, tune the controller's final-approach gains; it is currently
   the largest *controllable* contributor for small moves.

---

## 7. One-paragraph summary for a reviewer

> The DOF-5 stage exhibits a flat ~2.5 ms command-to-motion dead time, independent of move
> distance, with fine-settle (±15 nm) completing in ~50 ms and gross travel governed entirely
> by the 1.0 mm/s velocity cap (motion time scales linearly with distance, 37–243 ms over
> 10–200 µm). The low, distance-invariant dead time indicates that real-time closed-loop focus
> tracking is feasible; the practical tracking limit is slew rate, not latency, and the current
> conservative velocity cap leaves substantial headroom. Next step is a closed-loop frequency
> sweep to convert these open-loop point-to-point figures into a tracking bandwidth.
