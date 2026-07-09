# Autofocus Latency Web-App — Design & Implementation Guide

> **Status:** Design doc / build guide
> **Owner:** Anton Bloch (developer-implemented backend; AI-assisted UI only)
> **Last updated:** 2026-06-17
> **Companion tutor agents:** `latency_tutor` (CAN/latency internals), `Cornea Focus Tutor` (pipeline + Python)

---

## 0. How to read this document

This file is two things at once:

1. **A design doc** — what we are building, why, and how the pieces fit together.
2. **A line-by-line build guide** — broken into numbered phases so a tutor agent
   (or you, solo) can walk through it one concept at a time.

Two reading conventions:

- **🎓 Concept** blocks explain *why* a piece exists and what Python/hardware idea
  it teaches. Read these when you are learning.
- **🛠 Tutor checkpoint** blocks mark natural pause points to hand control to the
  `latency_tutor` or `Cornea Focus Tutor` agent to verify code you just wrote.

Everything in this doc is grounded in code that already exists in this repo. When
a section says "reuse X", it means *import it*, not rewrite it. The single most
important principle of this feature is: **do not reinvent the CAN layer, the
latency event model, or the driver abstraction — they are already proven.**

---

## 1. Purpose & one-paragraph summary

We want a **standalone, terminal-launched web app** that lets a developer:

1. Generate a focus **error** (a distance the cornea is out of focus), either
   **randomly** or from a **pre-defined value** the developer types in.
2. Set the stage's **velocity** and **acceleration** in the UI.
3. Press a single **"Focus"** button.
4. Watch the DOF stage physically correct that error.
5. Receive a **final latency report** breaking the correction into three phases:
   - **receive** the command (reaction / dead time),
   - **execute** the command (motion / travel),
   - **finish** the command (settle / lock-in).
6. Optionally run a **DOF reading-accuracy** self-check that probes how
   trustworthy the position values reported by the on-board Juno chip are.

The feature is a **simulation in the cornea-sim sense** — it reuses the same
units, the same control goal (drive a focus error to zero), and the same DOF
stage as `cornea_focus/` — but it **short-circuits the vision pipeline**.
Instead of detecting the cornea from an OCT frame to *measure* the error, the
error is handed to the system directly (random or typed). That isolates the
**stage-correction half** of the autofocus loop so we can study its latency
without the confounding variable of detector noise. See §3 for the precise
relationship.

> **Why a web app and not another matplotlib window?** We already have
> `scripts/play_sim_with_dof.py` driving a live matplotlib figure. A button-driven
> web UI is the right tool when the user's action is *discrete* (click → one move
> → one report) rather than *continuous* (stream frames). It also gives us a clean
> place to render the final report as text/numbers, which matplotlib is bad at.

---

## 2. Non-goals (what this feature is NOT)

- **Not a closed-loop tracker.** It commands one discrete move per button press
  and waits for full settle. Continuous sinusoidal tracking is a separate future
  test (see `doc/latency_report.md` §6 item 1).
- **Not a replacement for the batch benchmark.** `scripts/dof_latency_bench.py`
  already gathers 1000 trials for statistical rigor. This web app is the
  *interactive, single-shot* counterpart — used for sanity checks, demos, and
  poking at one configuration at a time.
- **Not a vision system.** No OCT frames are processed here. The "error" is an
  input number, not a detected one. (See §3 for why this is deliberate.)
- **Not a hardware-in-the-loop *truth* measurement.** The DOF reading-accuracy
  panel (§11) is a **self-consistency** check, not an external ground-truth
  measurement. We are honest about that limit up front.

---

## 3. How this relates to the existing cornea simulation

The repo already has a complete closed loop:

```
OCT frame (data/sim/*.npy)
   │
   ▼
cornea_focus/surface.py :: detect()        ← measures where the cornea IS
   │   (returns median_y in pixel rows)
   ▼
cornea_focus/control.py :: Controller.step()← decides where the stage SHOULD go
   │   (error_mm = (median_y − focus_row) × dz_mm_per_row)
   ▼
cornea_focus/dof_driver.py :: CanDriver    ← moves the stage there
   │   (move_absolute(target_mm))
   ▼
DOF-5 stage physically moves
```

This new feature **keeps the back half and replaces the front half with a number box**:

```
UI: error_µm  (random button OR typed value)
   │
   ▼
[NEW] autofocus_latency_webapp.py :: run_trial()
   │   counts = get_pos_counts(bus); sr(bus, OP_SET_POSITION, ...)
   │   (mirrors control.py's sign convention)
   ▼
dof_init + can.Bus                     ← SAME CAN layer, reused
   │
   ▼
DOF-5 stage physically moves
   │
   ▼
[NEW] latency report (reaction / motion / settle)
```

**Two concrete things to internalize before writing code:**

1. **Units.** The whole repo speaks in mixed units; you must keep them straight:
   - The DOF encoder: `COUNTS_PER_MM = 200_000` → **1 µm = 200 counts**, **1 count = 5 nm**.
   - OCT image rows: `dz_mm_per_row = 0.004593` (from `config.yaml`).
   - The web app's "error" input is in **µm** (human-friendly). Convert to counts
     (`× 200`) before any band comparison in the event detector.
2. **Sign convention.** In `cornea_focus/control.py`, `delta_mm = -Kp * error_mm`:
     a *positive* error (cornea below focus line) means the stage moves
     *negative*. Our web app mirrors this: `target_counts = home_counts - error_um * 200`.
     Keep the sign consistent or the stage will move *away* from focus. 🛠 Tutor
     checkpoint: ask the tutor to explain why the sign is negative before you
     wire the button.

---

## 4. File layout & deliverable

One standalone file, runnable from the project root:

```bash
python scripts/autofocus_latency_webapp.py
```

```
scripts/
└── autofocus_latency_webapp.py     ← THE new file. Single file, clear sections.
```

**Why a single file?** Three reasons: (1) it matches the existing convention
(`play_sim.py`, `play_sim_with_dof.py`, `dof_latency_bench.py` are each one
terminal-runnable script); (2) a tutor can walk one file top-to-bottom without
jumping around; (3) the UI is small enough that an embedded HTML string keeps
the whole feature self-contained. If it grows past ~700 lines, *then* split.

The file is organized into clearly-labelled sections (see §8 for the skeleton):

```
SECTION 0 — imports
SECTION 1 — configuration constants (the "simple variables" contract)
SECTION 2 — driver factory + lifecycle  (REUSE cornea_focus.dof_driver)
SECTION 3 — error generation            (random + predefined)
SECTION 4 — single-trial latency runner (ADAPT latency/dof_latency_bench.run_single_trial)
SECTION 5 — accuracy self-check module  (NEW — §11)
SECTION 6 — latency report builder
SECTION 7 — Flask web app + endpoints   (UI is AI-written, endpoints are dev-written)
SECTION 8 — embedded HTML/JS template   (AI-written, talks to backend via §1 variables)
SECTION 9 — main() / argparse entrypoint
```

The hard rule from the user: **the UI (Section 8) may be written by AI, but
Sections 2–6 (button-press logic, DOF integration, latency report, accuracy
check) MUST be implemented by the developer.** The UI only ever talks to the
backend through the variables defined in Section 1 (see §6).

---

## 5. The "simple variables" contract (UI ↔ backend boundary)

This is the most important design decision in the whole feature. To keep the
developer "not lost with which code does what", the UI and backend communicate
**only** through one request shape and one response shape. Both are plain
dictionaries (JSON over HTTP). No shared mutable state, no global variables.

### 5.1 Request (UI → backend), `POST /focus`

```python
{
    "mode":         "random" | "predefined",   # where the error comes from
    "error_um":     float | None,              # required iff mode == "predefined"
    "velocity_mm_s":   float,                  # stage velocity cap
    "acceleration_mm_s2": float,               # stage accel cap
    "direction":    +1 | -1 | 0,               # 0 = let sign come from error; ±1 = force
}
```

### 5.2 Response (backend → UI), returned by `/focus`

```python
{
    "error_um":      100.0,        # the error actually used (so UI can show it)
    "direction":     -1,
    "target_mm":     -0.100,       # absolute target sent to the stage
    "home_mm":       0.000,        # position before the move
    "final_mm":      -0.10012,     # position after settle
    "final_error_nm": 12.0,        # |final - target| in nm
    "events": {
        "t_cmd_ns":      1234567890,
        "t_react_ns":    1234567890,
        "t_engage_ns":   1234567890,
        "t_complete_ns": 1234567890,
    },
    "phases_us": {                 # the three numbers the user asked for
        "receive_us":  2500.0,     # = t_react   - t_cmd     ("how long to RECEIVE")
        "execute_us":  143000.0,   # = t_engage  - t_react   ("how long to EXECUTE")
        "finish_us":   52000.0,    # = t_complete- t_engage  ("how long to FINISH")
        "total_us":    197500.0,   # = t_complete- t_cmd
    },
    "status":        "complete" | "timeout" | "error",
    "message":       "",           # human note if status != complete
    "trace": [(t_ns_since_cmd, pos_counts), ...]  # optional, for the live chart
}
```

> 🎓 **Concept — why a frozen contract matters.** When the UI and backend share
> *only* this dict, you can rewrite the UI (or the backend) without touching the
> other half. It also means the tutor can test the backend with `curl` and never
> open a browser. This is the same separation `cornea_focus/control.py` already
> uses: `Controller.step()` takes numbers in, returns a `ControlOutput`
> dataclass out — no UI coupling.

### 5.3 Mapping user's three questions to existing event model

The user's three latency phases map **exactly** onto the proven event model in
`latency/LATENCY_SPEC.md` §5 and `latency/dof_latency_bench.py`:

| User's words | This app's key | Spec's name | Definition |
|---|---|---|---|
| "how long to **receive** the command" | `receive_us` | `reaction_us` | `t_react − t_cmd` |
| "how long to **execute** the command" | `execute_us` | `motion_us` | `t_engage − t_react` |
| "how long to **finish** the command" | `finish_us` | `settle_us` | `t_complete − t_engage` |
| (bonus) full cycle | `total_us` | `total_us` | `t_complete − t_cmd` |

> 🎓 **Concept — what each phase physically means** (from
> `doc/latency_report.md` §3, which you should read in full):
> - **receive (reaction)** is true *dead time*: CAN round-trip (~0.7 ms) + servo
>   processing + one poll. It is flat (~2.5 ms) regardless of distance.
> - **execute (motion)** is *travel time*, governed by your velocity/accel caps.
>   It scales linearly with distance. **It is not a latency — it is physics.**
> - **finish (settle)** is *fine lock-in*: getting from ±250 nm to parked at
>   ±15 nm for 5 ms continuous. ~50 ms, fairly constant.

---

## 6. Backend module breakdown — what the developer implements

Each subsection below is one function the developer writes. The AI UI only ever
calls these through the contract in §5.

### 6.1 Bus lifecycle — open, home, shutdown  (reuse `dof_init`, ~10 lines)

Open a raw `can.interface.Bus` in `main()` and pass it everywhere. Call
`dof_init.init_drive(bus)` at startup to home the stage and enable the servo.
In the `finally` block, tear down the bus with `bus.shutdown()`.

This is the pure Option B path (§7.1) — no `MockDriver`, no `CanDriver` wrapper,
no `make_driver` factory. The web app only ever talks to the real physical stage.
Every function in Sections 3–6 receives the `bus` handle directly.

> 🛠 Tutor checkpoint: open `scripts/dof_init.py` with the tutor and walk
> `init_drive()`. Confirm you understand why it must run before any move (it
> calibrates analog offsets and enables the servo — see `dof_oscillate_v1.py`
> `init_drive()` comments).

### 6.2 `generate_error(mode, error_um, direction) -> (error_um, direction)`

Responsibilities:
- If `mode == "random"`: draw an error from a sensible distribution. Two good
  options — pick one and document it:
  - **Uniform over the benchmark matrix** `[10, 25, 50, 100, 200] µm` (matches
    `DISTANCES_UM` in `dof_latency_bench.py`, so results are comparable).
  - **Uniform continuous** in `[10, 200] µm`.
- If `mode == "predefined"`: validate `error_um` is within soft-limit-aware
  bounds (a 200 µm move from a stage already at +1.1 mm would exceed the +1.2 mm
  soft limit — reject with a clear message).
- Direction: if the caller passes `direction ∈ {+1, -1}`, honor it; if `0`, pick
  randomly. Always bias the move to keep the stage inside `[min_mm, max_mm]` —
  i.e., if the stage is near the positive soft limit, force negative.

> 🎓 **Concept — soft limits are a safety feature, not a suggestion.**
> The stage enforces `[-1.2, +1.2] mm` (see `dof_init.SOFT_LIMIT_MM`). The servo
> raises an error outside that range. Your generator must never emit a move
> that would violate it, or the trial will crash mid-button-press. Compute the
> *feasible* error from the *current* position before returning.

### 6.3 `run_trial(bus, error_um, direction, vel_mm_s, acc_mm_s2) -> TrialReport`

This is the heart of the feature. It is a **direct adaptation** of
`latency/dof_latency_bench.py :: run_single_trial()`. Do not rewrite the
algorithm — copy its structure and adapt the I/O. Concretely:

1. Read `home_counts = dof_init.get_pos_counts(bus)` (raw integer counts,
   same as the benchmark).
2. Compute `target_counts = home_counts + direction * int(error_um * 200)`.
   (Because `COUNTS_PER_MM = 200_000` → `200 counts/µm`.)
3. Set motion params: `dof_init.set_motion_params(bus, vel_mm_s, acc_mm_s2)` —
   the UX lets the user change velocity/acceleration per click, so re-send
   before every trial.
4. Record `t_cmd = time.perf_counter_ns()`.
5. Issue the move: `dof_init.sr(bus, dof_init.OP_SET_POSITION, struct.pack(">i", target_counts))`
   followed by `dof_init.sr(bus, dof_init.OP_UPDATE)` — verbatim from the benchmark.
6. **Poll loop with the four-event detector** — verbatim from
   `run_single_trial`, using the same constants:
   - `NOISE_COUNTS = 3`, `ENGAGE_BAND_COUNTS = 50`, `SETTLE_BAND_COUNTS = 3`,
     `SETTLE_HOLD_S = 0.005`, `TIMEOUT_S = 0.5`.
7. Build and return the report dict from §5.2.

> 🎓 **Concept — why we poll `GetActualPosition` instead of trusting the
> command ack.** `OP_SET_POSITION + OP_UPDATE` returns as soon as the command
> is *acknowledged*, not when motion *completes*. The only way to know the stage
> has actually arrived is to poll its position repeatedly and watch it settle.
> This is exactly what the benchmark does, and it is why the poll loop is the
> core of the latency measurement. Read `latency/LATENCY_SPEC.md` §6 aloud with
> the tutor before implementing this function.

> 🛠 Tutor checkpoint: this is the function the `latency_tutor` agent exists for.
> Ask it to walk you through `run_single_trial` line by line, then have it
> validate your adapted version. **Do not move on until the tutor confirms the
> event-detection logic is identical in spirit to the benchmark.**

### 6.4 `build_report(...) -> dict`

Pure function: takes the raw timestamps + positions, returns the §5.2 dict.
Keeps the messy unit conversions (`counts → nm`, `ns → µs`) in one place so they
can be unit-tested without a stage. 🎓 **Concept — pure functions are
testable functions.** This is why `cornea_focus/control.py`'s `Controller.step`
returns a dataclass instead of mutating globals.

### 6.5 `run_accuracy_check(bus) -> dict`  (the NEW feature — see §11)

Separate endpoint `POST /accuracy`. Returns a self-consistency report. Detailed
in §11.

---

## 7. Reuse map — exactly what to import, not reinvent

| You need | Import from | File |
|---|---|---|
| CAN bus handle | `can.interface.Bus` | `python-can` (stdlib-like) |
| Config loading | `config.load` | `cornea_focus/config.py` |
| CAN send/receive primitive | `dof_init.sr` | `scripts/dof_init.py` |
| Position read (counts) | `dof_init.get_pos_counts` | `scripts/dof_init.py` |
| Servo init sequence | `dof_init.init_drive` | `scripts/dof_init.py` |
| Set vel/acc registers | `dof_init.set_motion_params` | `scripts/dof_init.py` |
| All opcodes + bus constants | `dof_init.OP_*`, `TX_ID`, etc. | `scripts/dof_init.py` |
| The 4-event trial algorithm | `run_single_trial` (adapt) | `latency/dof_latency_bench.py` |
| The `TrialResult` dataclass shape | `TrialResult` (adapt) | `latency/dof_latency_bench.py` |
| Event-detection constants | `NOISE_COUNTS`, etc. | `latency/dof_latency_bench.py` |

> 🎓 **Concept — code reuse is a safety property.** Every line above has already
> been validated against real hardware (`smoketest_results/`,
> `bench_20260616_*`). Reusing it means reusing its validated behavior. New code
> is where new bugs live; minimize new code.

### 7.1 The `driver` vs `bus` tension — RESOLVED

**Decision: Option B — raw `bus` only.** The web app opens a `can.interface.Bus`
in `main()`, passes it to `dof_init.init_drive(bus)` at startup, and every
function receives the `bus` handle directly.

Rationale: `dof_init` already exposes everything the poll loop needs
(`get_pos_counts`, `sr`, `set_motion_params`), and the benchmark already proves
this exact pattern works. No `CanDriver`, no `MockDriver`, no `make_driver`
factory. The app only runs on the bench machine with the real physical stage.

---

## 8. Implementation skeleton (single file)

```python
#!/usr/bin/env python3
"""Autofocus latency web-app — interactive single-shot DOF correction tester.

Run:  python scripts/autofocus_latency_webapp.py [--host 0.0.0.0] [--port 5000]
"""
# SECTION 0 — imports (stdlib, then third-party, then project)
# SECTION 1 — config constants + the §5 contract dataclasses
# SECTION 2 — bus lifecycle (open can.Bus, init_drive, shutdown in finally)
# SECTION 3 — error generation (generate_error)
# SECTION 4 — run_trial (adapted from latency/dof_latency_bench.run_single_trial)
# SECTION 5 — run_accuracy_check (§11)
# SECTION 6 — build_report (pure, unit-testable)
# SECTION 7 — Flask app: GET /, POST /focus, POST /accuracy, GET /status
# SECTION 8 — HTML/JS template string (black background, the button, the report)
# SECTION 9 — argparse main()
```

Dependencies (add to a venv; none are exotic):
- `flask` (web framework — pick Flask over FastAPI here purely for pedagogical
  simplicity; one file, four routes, no async).
- `python-can` (already present on the bench machine — used by all existing scripts).
- `pyyaml` (already required by `cornea_focus.config`).

> 🛠 Tutor checkpoint: before writing Section 7, have the `Cornea Focus Tutor`
> agent explain Flask's request/response cycle using `play_sim_with_dof.py`'s
> WebAgg backend as a mental anchor. Both serve HTTP; Flask is just more
  explicit about routes.

---

## 9. The UI (Section 8 — AI-assisted, black background)

Requirements handed to the AI for the UI; the developer reviews but does **not**
author:

- Full-screen black background (`body { background:#000; color:#0f0; }`).
- A large centered **"FOCUS"** button.
- Three numeric inputs: **Velocity (mm/s)**, **Acceleration (mm/s²)**,
  **Error (µm)**.
- A **"Randomize error"** toggle (sets `mode: "random"`).
- A **direction** dropdown: `auto / + / −`.
- A **report panel** that appears after a trial, showing `receive_us`,
  `execute_us`, `finish_us`, `total_us`, plus `error_um`, `target_mm`,
  `final_error_nm`, and a status line.
- A live **position readout** polled from `GET /status` every ~200 ms (purely
  cosmetic — keeps the user oriented; do NOT let it interfere with a running
  trial).
- An optional line chart of the last trial's `trace` (time vs position).
- A second button: **"Run accuracy check"** that hits `POST /accuracy`.

The ONLY thing the JS is allowed to do is:

1. Read the inputs into the §5.1 request dict.
2. `fetch('/focus', {method:'POST', body: JSON.stringify(req)})`.
3. Render the §5.2 response dict into the report panel.

No stage logic in JS. Ever. If the JS needs a number the backend doesn't expose,
**add it to the contract** (§5), don't compute it in the browser. This is the
"simple variables" rule made enforceable.

> 🎓 **Concept — separation of concerns.** The browser is a *display*; the
> Python process is the *authority*. The same split already exists in
> `play_sim_with_dof.py`: matplotlib draws, the Python loop decides. Here, JS
> draws, Flask decides.

---

## 10. Safety, edge cases, and Ctrl-C behavior

Mirror the discipline of `dof_latency_bench.py` §10 and `play_sim_with_dof.py`'s
`finally` block:

- **One move at a time.** The `/focus` endpoint must refuse a second request
  while a trial is running (return `429` or `status: "busy"`). The stage cannot
  be commanded by two callers simultaneously — the second `SetPosition` would
  interrupt the first mid-settle and corrupt the measurement.
- **Ctrl-C / shutdown.** Wrap `main()` in `try/finally` that calls `bus.shutdown()`. Never leave the servo energized and unattended if the process dies.
- **Timeouts.** `TIMEOUT_S = 0.5` per trial (same as benchmark). If a trial
  times out, return `status: "timeout"` with whatever partial timestamps you
  have — do not hang the UI.
- **Soft-limit violations.** `generate_error` must prevent these (§6.2); if one
  slips through, the servo rejects the command — catch the CAN timeout/error,
  return `status: "error"`, do not crash the server.
- **Headless server.** This is a *web* app precisely so it runs on the remote
  bench box without a display. Bind to `0.0.0.0` and access it through VS Code's
  automatic port forwarding (same pattern as `play_sim_with_dof.py`'s WebAgg
  `address="0.0.0.0"`).

---

## 11. DOF reading-accuracy self-check (the NEW sub-feature)

### 11.1 Honest framing (read first)

The user is correct to be skeptical: **the only position information we have
comes from the Juno chip's own encoder.** There is no second, independent sensor
in this system. So we **cannot** measure absolute physical truth from software.
What we *can* do is measure **internal self-consistency** — does the encoder
agree with itself across repeated and reciprocal commands? That is genuinely
useful: it catches config errors (wrong `COUNTS_PER_MM`), backlash, drift, and
saturation. It does not catch a calibrated-but-wrong encoder.

State this limitation verbatim in the UI when the user opens the accuracy panel.
Honesty here is a hard requirement, not a nice-to-have.

### 11.2 Three self-consistency probes

Each probe issues real moves and reports numbers. All reuse `run_trial`'s polling
so they share the proven settle detection.

| # | Probe | What it asks | What it catches |
|---|---|---|---|
| 1 | **Repeatability** | Command the same 100 µm move N=20 times from the same home. Report stdev of `final_error_nm`. | Servo noise; inconsistent settle. |
| 2 | **Linearity / counts-per-mm** | Command steps of 10, 50, 100, 200 µm. For each, compare *reported displacement* (`final−home`) to *commanded*. Plot ratio vs distance. | Wrong `COUNTS_PER_MM`; gain error. |
| 3 | **Hysteresis / backlash** | Move +100 µm then −100 µm. Report residual `|final − home|`. Mean over N=10 cycles. | Mechanical backlash; stiction. |

### 11.3 Output shape (`POST /accuracy` response)

```python
{
    "limitation": "Encoder-only self-consistency; not absolute ground truth.",
    "repeatability": {
        "n": 20, "step_um": 100.0,
        "final_error_nm_mean": 8.2, "final_error_nm_stdev": 4.1,
        "verdict": "pass" | "marginal" | "fail",
    },
    "linearity": [
        {"commanded_um": 10.0,  "reported_um": 10.02, "ratio": 1.002, ...},
        {"commanded_um": 50.0,  "reported_um": 49.95, "ratio": 0.999, ...},
        ...
    ],
    "hysteresis": {
        "n_cycles": 10, "step_um": 100.0,
        "residual_nm_mean": 22.0, "residual_nm_stdev": 11.0,
        "verdict": "pass" | "marginal" | "fail",
    },
}
```

Verdict thresholds (document and tune with the tutor):
- repeatability stdev < 50 nm → pass; 50–200 nm → marginal; > 200 nm → fail.
- hysteresis residual < 100 nm → pass; 100–500 nm → marginal; > 500 nm → fail.
- linearity ratio within ±1% at every step → pass.

> 🛠 Tutor checkpoint: After writing probe 2, verify it on the real stage. Any
> deviation from a 1.000 commanded/reported ratio is a real encoder/calibration
> finding. The tutor should help you interpret the first real-hardware run.

---

## 12. Implementation phases (for the tutor to walk you through)

Each phase ends with something you can run and see. Do not start phase N+1 until
phase N's checkpoint passes.

### Phase 0 — Scaffolding (≈30 min)
- Create `scripts/autofocus_latency_webapp.py` with Section 0–1 only.
- Pin the §5 contract as `@dataclass` types in code.
- `python scripts/autofocus_latency_webapp.py --help` prints usage. Nothing else.
- 🛑 **Checkpoint:** contract dataclasses import cleanly; `--help` works.

### Phase 1 — Bus wiring, no UI (≈1 hr)
- Implement Section 2 (open `can.Bus`, `init_drive`, `finally` shutdown).
- Implement a CLI subcommand `--self-test` that homes, moves +50 µm, moves back,
  prints positions. This is your "can I talk to the stage?" gate.
- 🛑 **Checkpoint:** `--self-test` round-trips a 50 µm move on the real stage.

### Phase 2 — The trial runner (≈2 hr)  ← the core phase
- Implement Sections 3, 4, 6 (`generate_error`, `run_trial`, `build_report`).
- Add a CLI subcommand `--trial 100` that runs one 100 µm trial and prints the
  §5.2 dict to stdout — **no web server yet.**
- Compare your printed `phases_us` against `bench_20260616_210048/` for the
  `100um_pos` config: reaction ~2.6 ms, motion ~143 ms, settle ~52 ms,
  total ~197 ms. They won't match exactly (single trial), but should be the same
  *order of magnitude*. If reaction is 100 ms, you have a bug.
- 🛑 **Checkpoint:** single trial matches the historical numbers within ~2×.
- 🛠 **Hand off to `latency_tutor`** to validate the event-detection loop.

### Phase 3 — Flask shell + contract round-trip (≈1 hr)
- Implement Section 7 routes, but have `/focus` just `echo` the request back as
  the response (fake report). Implement Section 8 UI (AI-written) against the
  echoed contract.
- 🛑 **Checkpoint:** click "FOCUS" in the browser, see a fake report render with
  the values you typed. UI↔backend plumbing proven before real stage calls.

### Phase 4 — Wire the real runner behind the button (≈1 hr)
- Replace the `/focus` echo with a real `run_trial(...)` call. Add the busy-lock
  (§10).
- 🛑 **Checkpoint:** a real 100 µm click produces a real report on the bench
  stage. The three phase numbers display correctly.

### Phase 5 — Live position polling + trace chart (≈1 hr)
- Add `GET /status` and the 200 ms poll. Add the trace line chart from `trace`.
- 🛑 **Checkpoint:** watch the stage gauge move during a trial; chart shows the
  settle tail.

### Phase 6 — Accuracy self-check (≈2 hr)  ← the NEW feature
- Implement Section 5 + `/accuracy` route + UI panel.
- 🛑 **Checkpoint:** three probes run; real stage gives a verdict per probe.
- 🛠 **Hand off to `latency_tutor`** to interpret the first real-hardware run.

### Phase 7 — Polish & edge cases (≈1 hr)
- Soft-limit-aware error generation (§6.2).
- Timeout/busy handling (§10).
- `README.md` one-paragraph usage note.

---

## 13. Testing strategy

- **All testing is on the bench machine (`dofcomputer`).** There is no mock —
  the app only talks to the real physical stage. This keeps latency numbers real
  and avoids the false confidence of instant mock moves.
- **Unit tests:** add `tests/test_autofocus_webapp.py` covering *only* the pure
  functions: `generate_error` boundary logic, `build_report` unit conversions,
  soft-limit rejection. Do not unit-test `run_trial` (needs hardware) — test it
  via the Phase 2 CLI checkpoint instead. This mirrors the existing
  `tests/test_control.py` / `tests/test_surface.py` split (pure logic tested,
  hardware not).
- **Regression check:** the historical benchmark numbers in
  `bench_20260616_210048/` are your oracle. Any single-trial result within ~2×
  of the medians in `doc/latency_report.md` §2 means the wiring is correct.

---

## 14. Definition of done

The feature is complete when **all** of these hold:

1. `python scripts/autofocus_latency_webapp.py` launches a black-background web
   app reachable through VS Code port forwarding on the remote bench server.
2. Clicking **FOCUS** with a random or predefined error physically moves the
   real DOF stage and displays `receive_us`, `execute_us`, `finish_us`,
   `total_us`, `final_error_nm`, and `status`.
3. The velocity and acceleration inputs actually change the stage's behavior
   (verify: a 100 µm move at 5 mm/s has visibly shorter `execute_us` than at
   1 mm/s — this proves the params reach the registers via `dof_init.set_motion_params`).
4. The accuracy panel runs all three probes and prints the honesty limitation
   verbatim.
5. Ctrl-C shuts the servo down cleanly (verify in process list / stage hum).
6. No new code duplicates the CAN layer or the event detector — all imported per §7.
7. `tests/test_autofocus_webapp.py` passes for the pure functions.
8. The two tutor agents (`latency_tutor`, `Cornea Focus Tutor`) have each signed
   off on their respective phases (2 and 6).

---

## 15. Open questions to resolve with the tutor before coding

1. **Option A vs B for the `driver`/`bus` split (§7.1).** Resolved: Option B (raw `bus` only).
2. **Random-error distribution (§6.2):** discrete benchmark matrix vs continuous
   uniform. Recommended: discrete, for comparability with existing reports.
3. **Should `/status` polling pause during a trial?** Recommended: yes — a poll
   is a CAN round-trip (~375 µs) that could perturb the tight event-detection
   loop. Pause the cosmetic gauge while `run_trial` owns the bus.
4. **Accuracy probe 2 step list:** keep it identical to `DISTANCES_UM`
   `[10,25,50,100,200]`? Recommended: yes.
5. **Where to log trial results?** Recommended: append each `/focus` result to
   `autofocus_webapp_runs/<timestamp>.jsonl` so interactive sessions still leave
   an artifact, like the benchmark does.

---

## 16. Quick reference — constants you will type a lot

```
COUNTS_PER_MM      = 200_000        # dof_init.py
COUNTS_PER_UM      = 200            # derived: COUNTS_PER_MM / 1000
NM_PER_COUNT       = 5              # derived: 1e6 nm/mm / COUNTS_PER_MM
SOFT_LIMIT_MM      = 1.2            # stage min/max (±1.2, from dof_init.py)

NOISE_COUNTS       = 3              # = 15 nm   (react threshold)
ENGAGE_BAND_COUNTS = 50             # = 250 nm  (near-target)
SETTLE_BAND_COUNTS = 3              # = 15 nm   (locked)
SETTLE_HOLD_S      = 0.005          # 5 ms continuous in band
TIMEOUT_S          = 0.5            # trial safety cap

TX_ID = 0x600   RX_ID = 0x580   AXIS = 0   SAMPLE_S = 51e-6
```

All of these already live in `scripts/dof_init.py` and
`latency/dof_latency_bench.py`. Import them; do not redeclare magic numbers.
