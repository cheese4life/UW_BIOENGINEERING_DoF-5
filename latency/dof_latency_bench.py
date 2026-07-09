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

    # === Payload characterization fields (multi-channel Juno telemetry).
    # Collected during the trial's polling loop by reading GetActualVelocity
    # (0xAD), GetCommandedPosition (0x4A), and GetMotorCommand (0xB6) in
    # addition to the existing GetActualPosition (0x37).
    #
    # Peaks & following error are computed across the whole motion phase
    # (t_react → t_complete). Velocity milestones are sampled at 25/50/75%
    # of the motion interval — for a triangular move 50% = peak velocity,
    # for a trapezoidal move 50% = mid-cruise plateau.
    peak_velocity_mm_s: float = float('nan')           # max |v| during motion
    peak_velocity_t_frac: float = float('nan')         # when (0=motion start, 1=end)
    velocity_at_25pct_mm_s: float = float('nan')       # snapshot at 25% of motion
    velocity_at_50pct_mm_s: float = float('nan')       # snapshot at 50% (peak for triangular)
    velocity_at_75pct_mm_s: float = float('nan')       # snapshot at 75% of motion
    peak_motor_cmd: int = 0                            # max |force| register value
    peak_motor_cmd_t_frac: float = float('nan')
    peak_following_error_counts: int = 0               # max |commanded - actual|
    mean_following_error_counts: float = float('nan')
    # Calculated trapezoidal reference (what peak velocity *should* be
    # for a perfect physics-textbook move of this distance at this v/a).
    calc_peak_velocity_mm_s: float = float('nan')
    calc_motion_us: float = float('nan')               # ideal motion time from trapezoidal math
    # Rate-of-change slopes (jerk proxy). avg_accel_mm_s2 = peak_v / t_to_peak
    # if we have enough samples; otherwise NaN.
    avg_accel_mm_s2: float = float('nan')
    avg_decel_mm_s2: float = float('nan')


    

NOISE_COUNTS = 3
ENGAGE_BAND_COUNTS = 50
SETTLE_BAND_COUNTS = 3
SETTLE_HOLD_S = 0.005
TIMEOUT_S = 2.0              # per-move safety timeout (was 0.5; max-spec
                            # moves can take ~80ms+ and CAN reads stutter)
INTER_TRIAL_GUARD_RETRY_S = 0.001  # back-off between CAN reads on retry

# Stage travel limits — DEFAULTS ONLY. These are overwritten at runtime by
# probe_travel_envelope() + configure_runtime_envelope() because the
# encoder origin varies per power-cycle (observed +0.0033, +2.86, +5.87
# mm in different runs). Hardcoded values don't apply across runs.
SOFT_LIMIT_POS_MM = 1.5     # overwritten at startup by probe
SOFT_LIMIT_NEG_MM = -1.5    # overwritten at startup by probe
# Reference position the stage returns to between trials. Computed as the
# midpoint of the discovered soft-limit envelope. Symmetric moves up to
# ~half the usable travel fit centered at this home.
HOME_MM = 0.0               # overwritten at startup by configure_runtime_envelope
SAFETY_MARGIN_MM = 0.10      # never command right up to a soft limit;
                             # larger margin because long moves at max
                             # spec overshoot more
HOME_DRIFT_WARN_COUNTS = 50  # warn if post-home sample is >0.25 µm off HOME_MM

# Move distances in µm. The user wants to push all the way to 4.0 mm so
# they can see how latency scales with distance across the full travel
# envelope. 13 distances × 2 directions × 100 trials = 2600 trials.
DISTANCES_UM = [10, 25, 50, 100, 200, 500, 1000, 1500,
                2000, 2500, 3000, 3500, 4000]
DIRECTIONS = [("pos", +1), ("neg", -1)]


def _read_pos_retry(bus, max_retries=5):
    """Read actual position, retrying on CAN timeout (max-spec moves
    sometimes produce sparse/blocked reads). Returns (position, ok_bool)."""
    for attempt in range(max_retries):
        try:
            return dof_init.get_pos_counts(bus), True
        except RuntimeError as e:
            # sr() raises RuntimeError on CAN poll timeout
            time.sleep(INTER_TRIAL_GUARD_RETRY_S * (attempt + 1))
    return 0, False


def in_safe_travel(pos_counts, extra_margin_mm=0.0):
    """Return True if pos_counts is within the soft-limit window.

    `extra_margin_mm` can be applied to further back off the legal window
    before a move is even commanded (we want to make sure the *target* and
    the expected worst-case overshoot both fit).
    """
    pos_mm = pos_counts / dof_init.COUNTS_PER_MM
    lo = SOFT_LIMIT_NEG_MM + SAFETY_MARGIN_MM + extra_margin_mm
    hi = SOFT_LIMIT_POS_MM - SAFETY_MARGIN_MM - extra_margin_mm
    return lo <= pos_mm <= hi


def safe_target_for_move(home_counts, direction, distance_um):
    """Compute target counts for a move, enforcing a soft-limit check.

    Returns (target_counts, ok). If the move would command the stage outside
    [SOFT_LIMIT_NEG_MM + margin, SOFT_LIMIT_POS_MM - margin], returns
    (None, False) and the caller must abort or re-home.
    """
    delta = direction * int(distance_um * dof_init.COUNTS_PER_MM / 1000)
    target = home_counts + delta
    if not in_safe_travel(target, extra_margin_mm=0.0):
        return None, False
    return target, True


# Sentinel for "use module-level HOME_MM at call time". We can't use
# `target_mm=HOME_MM` as a default because Python binds default-argument
# values at function-definition time, before configure_runtime_envelope()
# mutates the global HOME_MM. So we use None and look it up at call time.
_HOME_SENTINEL = object()


def go_home(bus, target_mm=_HOME_SENTINEL, label="home", verbose=False):
    """Command an absolute move to `target_mm` and block until settled.

    Uses a moderate velocity (not the bench's max) so the positioning move
    itself is safe and well-controlled, regardless of what
    --velocity/--acceleration the user set for the test moves. The caller
    must re-apply the test motion params after this returns.

    For long-distance configs (≥ ~0.75 mm), `target_mm` is the opposite
    edge of the travel envelope — see safe_start_for_move().

    If target_mm is omitted, uses the current value of module-level HOME_MM
    (looked up at call time, not at function-definition time, so
    configure_runtime_envelope() mutations are respected).

    `verbose=True` prints the move (use for the initial home / dry-run).
    During the trial loop, verbose=False keeps the log quiet.
    """
    if target_mm is _HOME_SENTINEL:
        target_mm = HOME_MM
    target_counts = int(round(target_mm * dof_init.COUNTS_PER_MM))
    home = dof_init.get_pos_counts(bus)
    if abs(home - target_counts) <= SETTLE_BAND_COUNTS:
        if verbose:
            print(f"[home:{label}] already at {target_mm:+.3f} mm ({target_counts}); no move needed")
        return True

    if verbose:
        print(f"[home:{label}] moving to {target_mm:+.3f} mm ({target_counts} counts)")
    # use conservative motion params for the home move itself
    dof_init.set_motion_params(bus, vel_mm_s=20.0, acc_mm_s2=400.0)
    dof_init.sr(bus, dof_init.OP_SET_POSITION,
                struct.pack(">i", target_counts))
    dof_init.sr(bus, dof_init.OP_UPDATE)
    ok = settle_guard(bus, target_counts, label=label)
    if verbose:
        if ok:
            print(f"[home:{label}] settled at {target_mm:+.3f} mm")
        else:
            print(f"[home:{label}] FAILED to settle at {target_mm:+.3f} mm")
    elif not ok:
        # always surface failures even when quiet
        print(f"[home:{label}] FAILED to settle at {target_mm:+.3f} mm")
    return ok


def settle_guard(bus, target_counts, label="guard", verbose=False):
    """Ensure stage has actually settled at `target_counts` before we let
    the bench start a new trial. Mirrors the in-band-hold logic used in
    run_single_trial, but as an explicit inter-trial barrier.

    Returns True if settled within TIMEOUT_S, False otherwise.
    By default silent on success — prints a warning only on timeout/CAN fail.
    """
    in_band_since = None
    t_start = time.perf_counter_ns()

    while True:
        t_now = time.perf_counter_ns()
        position, ok = _read_pos_retry(bus)
        if not ok:
            print(f"[guard:{label}] CAN read failed repeatedly — aborting guard")
            return False

        if abs(position - target_counts) <= SETTLE_BAND_COUNTS:
            if in_band_since is None:
                in_band_since = t_now
            elif (t_now - in_band_since) / 1e9 >= SETTLE_HOLD_S:
                return True
        else:
            in_band_since = None

        if (t_now - t_start) / 1e9 > TIMEOUT_S:
            print(f"[guard:{label}] TIMEOUT — pos={position} target={target_counts} "
                  f"err={abs(position - target_counts)} counts")
            return False


def move_and_wait_for_stop(bus, target_counts, label="probe",
                           settle_hold_s=0.5, timeout_s=4.0,
                           move_grace_s=0.5):
    """Command a move and wait for the stage to *physically stop*, even if
    it never reaches the requested target. Used by probe_travel_envelope()
    to find where the controller clamps at its own internal limit.

    "Physical stop" = position hasn't changed by >NOISE_COUNTS in the last
    settle_hold_s seconds. Implementation: track `last_pos` (the previous
    sample) and `last_motion_t` (when we last saw >NOISE_COUNTS change
    vs. last_pos). If (now - last_motion_t) >= settle_hold_s, the stage
    has been still long enough → return.

    `move_grace_s` = sanity grace period at start of move where we don't
    bail out on "no motion" — the servo's response latency to a fresh
    SetPosition+Update can be ~100-300ms, and during that window the stage
    position legitimately hasn't started changing yet. Without this grace,
    early polling reads the pre-move position and the settle logic would
    falsely exit "stopped" before motion ever began.

    Returns the actual settled position in counts.
    """
    dof_init.sr(bus, dof_init.OP_SET_POSITION,
                struct.pack(">i", target_counts))
    dof_init.sr(bus, dof_init.OP_UPDATE)

    last_pos, _ = _read_pos_retry(bus)
    last_motion_t = time.perf_counter_ns()
    t_start = last_motion_t

    while True:
        t_now = time.perf_counter_ns()
        position, ok = _read_pos_retry(bus)
        if not ok:
            print(f"[probe:{label}] CAN read failed — returning last known pos")
            return last_pos if last_pos is not None else position

        # Track motion: any sample-to-sample change > NOISE_COUNTS means
        # the stage is currently moving. We compare to *last_pos* (not
        # start_pos) so once the stage settles, "moved" goes False even
        # though we are far from start.
        if abs(position - last_pos) > NOISE_COUNTS:
            last_motion_t = t_now
        last_pos = position

        # Allow settle-check only after the grace period (so we don't
        # declare "stopped" before the servo has even started moving).
        if (t_now - t_start) / 1e9 > move_grace_s:
            if (t_now - last_motion_t) / 1e9 >= settle_hold_s:
                return position

        if (t_now - t_start) / 1e9 > timeout_s:
            print(f"[probe:{label}] TIMEOUT after {timeout_s}s — "
                  f"pos={position}")
            return position


def _probe_one_direction(bus, start_counts, direction, step_mm=0.5,
                         max_steps=24, settle_hold_s=0.5, timeout_s=4.0):
    """Walk the stage in `direction` in `step_mm` increments until a step
    produces no net motion (we've hit a limit the controller won't pass).

    Returns the settled position (counts) at the limit. We use small steps
    because the Juno controller SILENTLY REJECTS (not clamps) commands
    that target far outside its software limits — issuing one big +10mm
    move just leaves the stage where it was. Walking in small steps keeps
    every target inside or just outside the controller's software limit,
    so we make progress until we hit the wall, then detect it as "no
    motion on this step".

    `direction` is +1 or -1.

    settle_hold_s is intentionally larger than SETTLE_HOLD_S (5ms) because
    at the slow probe velocity (3 mm/s) a 0.5mm step takes ~170ms of
    motion + ~200ms of servo final-approach damping — total ~0.4s before
    the stage is truly stopped. 0.5s of "no position change" is a robust
    stop marker.

    On detecting the limit we immediately back off one step in the
    reverse direction. Empirically, sustaining contact with the hard
    limit (holding the stage there for seconds while my "2 consecutive
    no-motion" check waited) causes the Juno to enter a deep fault state
    that SILENTLY REJECTS all subsequent move commands. Backing off
    immediately keeps the controller healthy for the next probe direction.
    The function returns the position BEFORE the backoff (= the limit).
    """
    step_counts = int(step_mm * dof_init.COUNTS_PER_MM)
    label = "+probe" if direction > 0 else "-probe"
    # Re-read the actual current position — the caller passes a "start"
    # but after probing the other direction (or after a timeout) the
    # stage may not actually be where we think it is.
    current, ok = _read_pos_retry(bus)
    if not ok:
        print(f"[probe:{label}] CAN read failed at start — aborting walk")
        return start_counts
    print(f"[probe:{label}] walk starts at "
          f"{current/dof_init.COUNTS_PER_MM:+.4f} mm")
    for i in range(max_steps):
        prev_pos = current
        target = current + direction * step_counts
        settled = move_and_wait_for_stop(
            bus, target, label=f"{label}#{i}",
            settle_hold_s=settle_hold_s, timeout_s=timeout_s)
        # Re-read to get the authoritative current position.
        settled_actual, ok = _read_pos_retry(bus)
        if ok:
            settled = settled_actual
        delta = (settled - current) * direction
        current = settled
        # If we made < 1/5 of the requested step, we've hit the limit.
        # Use a single-step detection (not 2 consecutive) to avoid holding
        # the stage against the hard limit — sustained contact puts the
        # Juno into a deep fault state we can't recover from.
        if delta < step_counts / 5:
            # The limit edge is approximately `prev_pos` (where we
            # successfully settled one step ago), not `settled` (where
            # the servo drooped back to after the failed step). Using
            # settled here causes a coordinate mismatch that locks the
            # controller out of subsequent moves.
            limit_pos = prev_pos
            print(f"[probe:{label}] step {i}: target {target/dof_init.COUNTS_PER_MM:+.3f}mm "
                  f"settled {settled/dof_init.COUNTS_PER_MM:+.3f}mm "
                  f"(delta {delta/dof_init.COUNTS_PER_MM:+.3f}mm) → HIT LIMIT "
                  f"(recording edge as {limit_pos/dof_init.COUNTS_PER_MM:+.3f}mm "
                  f"= last successful position)")
            return limit_pos
        print(f"[probe:{label}] step {i}: target {target/dof_init.COUNTS_PER_MM:+.3f}mm "
              f"settled {settled/dof_init.COUNTS_PER_MM:+.3f}mm "
              f"(delta {delta/dof_init.COUNTS_PER_MM:+.3f}mm) → continuing")
    print(f"[probe:{label}] reached max_steps ({max_steps}) without "
          f"hitting a limit — returning last position")
    return current


def _clear_limit_fault_state(bus, hold_counts):
    """Reset the controller's fault state after it's been pushed against a
    hard limit. After hitting the +edge or -edge the Juno enters a state
    that SILENTLY REJECTS subsequent move commands (including reverse
    ones). Empirically the ONLY reliable way to clear this is to perform
    a full re-init including OP_CAL_ANALOG. This slightly perturbs the
    encoder origin (by ~few counts), which is acceptable because the probe
    re-reads the actual position immediately after recovery.
    """
    print(f"[probe] clearing fault state (full re-init)...")
    # Match init_drive's sequence exactly: events, motor cmd zero,
    # OPMODE_CAL, CAL_ANALOG, events, OPMODE_FULL. This sequence was
    # proven to enable the servo from any state at startup; it works
    # for limit-fault recovery too.
    dof_init.sr(bus, dof_init.OP_RESET_EVENT, struct.pack(">H", 0xA000))
    dof_init.sr(bus, dof_init.OP_RESET_EVENT, struct.pack(">H", 0xEFFF))
    dof_init.sr(bus, dof_init.OP_SET_MOTOR_CMD, struct.pack(">h", 0))
    dof_init.sr(bus, dof_init.OP_SET_OPMODE, struct.pack(">H", dof_init.OPMODE_CAL))
    time.sleep(0.05)
    dof_init.sr(bus, dof_init.OP_CAL_ANALOG, struct.pack(">H", 0))
    time.sleep(0.2)
    dof_init.sr(bus, dof_init.OP_RESET_EVENT, struct.pack(">H", 0xEFFF))
    dof_init.sr(bus, dof_init.OP_SET_OPMODE, struct.pack(">H", dof_init.OPMODE_FULL))
    time.sleep(0.1)
    # Re-read position — CAL_ANALOG may have shifted the register by a
    # few counts. Use the actual current position as the new hold target
    # so we don't re-introduce a "far from current position" issue.
    actual, _ = _read_pos_retry(bus)
    dof_init.sr(bus, dof_init.OP_SET_POSITION, struct.pack(">i", actual))
    dof_init.sr(bus, dof_init.OP_UPDATE)
    # Generous settle time so the servo is fully ready for the next move
    time.sleep(1.0)
    return actual


def _nudge_off_limit(bus, direction, nudge_counts=3000, max_tries=5):
    """If the stage is stuck against a hard limit, apply brief direct
    motor torque commands to drag it back into safe travel. Uses
    OP_SET_MOTOR_CMD to bypass the trajectory planner (which rejects all
    commands when the stage reports being outside software limits).

    `direction` is the direction to nudge IN (away from the limit). So if
    the stage is stuck at the +limit, pass direction=-1; if at the -limit
    pass direction=+1.

    `nudge_counts` is the motor command level. Sign is determined by
    direction. After each nudge we read the position; if it changed in
    the requested direction, we got it loose. Cycle up to max_tries with
    escalating nudge amplitude.

    Returns the post-nudge position (counts) on success, or None if we
    couldn't move it.
    """
    print(f"[probe] nudging stage in direction {direction:+d} to escape "
          f"limit contact...")
    start_pos, _ = _read_pos_retry(bus)
    for attempt in range(max_tries):
        torque = direction * nudge_counts * (attempt + 1)
        print(f"[probe]   attempt {attempt+1}: motor_cmd={torque}")
        # Apply torque briefly (direct motor command, bypasses trajectory)
        dof_init.sr(bus, dof_init.OP_SET_MOTOR_CMD, struct.pack(">h", torque))
        time.sleep(0.1)
        # Release
        dof_init.sr(bus, dof_init.OP_SET_MOTOR_CMD, struct.pack(">h", 0))
        time.sleep(0.1)
        # Did we move in the desired direction?
        pos, _ = _read_pos_retry(bus)
        delta = (pos - start_pos) * direction
        print(f"[probe]   pos now {pos/dof_init.COUNTS_PER_MM:+.4f} mm "
              f"(delta {delta/dof_init.COUNTS_PER_MM:+.4f} mm)")
        if delta > 100:  # moved at least 100 counts in desired direction
            print(f"[probe]   ✗ stage moved — out of limit contact")
            return pos
    print(f"[probe]   ✗ could not nudge stage off limit after {max_tries} tries")
    return None


def probe_travel_envelope(bus):
    """Empirically discover the stage's actual travel envelope regardless
    of where the encoder zero happens to sit.

    After init_drive() the encoder origin varies per power-cycle (observed
    +5.87mm, +2.86mm, +0.0033mm, -0.0136mm in different runs). Hardcoded
    limits don't mean anything across runs because the Juno controller has
    its own internal software limits relative to its (floating) origin.

    Strategy: walk the stage in small 0.5mm steps in each direction until
    a step produces no net motion (we've hit the controller's limit).
    Small steps are required because the Juno SILENTLY REJECTS commands
    far outside its limits — a single big +10mm probe just leaves the
    stage where it was, producing a 0-width envelope.

    Returns (neg_limit_mm, pos_limit_mm).
    """
    print("[probe] discovering actual travel envelope by step-walk...")
    start_counts = dof_init.get_pos_counts(bus)
    start_mm = start_counts / dof_init.COUNTS_PER_MM
    print(f"[probe] starting position: {start_mm:+.4f} mm")

    # If the stage powered up at or near a hard stop (common: from prior
    # runs the stage often ends at +5.88mm), the Juno controller's internal
    # software-limit logic may silently REJECT every move command —
    # including the reverse-direction move that would bring it back into
    # safe travel. Re-running init_drive() clears events / faults and
    # re-enables the servo cleanly, so subsequent moves are accepted.
    if (start_mm > 4.5 or start_mm < -1.5):
        print(f"[probe] stage powered up near a hard stop "
              f"({start_mm:+.3f} mm); re-running init to clear limit state")
        dof_init.init_drive(bus)
        start_counts = dof_init.get_pos_counts(bus)
        start_mm = start_counts / dof_init.COUNTS_PER_MM
        print(f"[probe] post-re-init position: {start_mm:+.4f} mm")

    # Use conservative motion params so the probe itself can't damage
    # the stage even if a limit is misconfigured.
    dof_init.set_motion_params(bus, vel_mm_s=3.0, acc_mm_s2=80.0)

    # Clear any pending events / fault state from init_drive before probing.
    # init_drive() ends in OPMODE_FULL with the servo holding position, but
    # the controller often needs more settling time before accepting the
    # first move command. Without this, the first probe step often gets
    # silently rejected (no motion detected) and the probe falsely reports
    # 0mm travel.
    dof_init.sr(bus, dof_init.OP_RESET_EVENT, struct.pack(">H", 0xA000))
    dof_init.sr(bus, dof_init.OP_RESET_EVENT, struct.pack(">H", 0xEFFF))
    # Zero motor command (matches init_drive pattern; clears any residual
    # torque command from calibration).
    dof_init.sr(bus, dof_init.OP_SET_MOTOR_CMD, struct.pack(">h", 0))
    # Hold current position briefly to confirm servo is responsive
    dof_init.sr(bus, dof_init.OP_SET_POSITION, struct.pack(">i", start_counts))
    dof_init.sr(bus, dof_init.OP_UPDATE)
    # IMPORTANT: give the servo 1.0s to settle post-init_drive before
    # probing. The controller's trajectory planner can take 200-800ms
    # after OPMODE_FULL is enabled to accept move commands. Shorter breaks
    # the probe at start positions like -0.0136mm (no motion detected).
    time.sleep(1.0)
    pos_check = dof_init.get_pos_counts(bus)
    if abs(pos_check - start_counts) > 100:
        print(f"[probe] WARNING: stage drifted "
              f"{(pos_check-start_counts)/dof_init.COUNTS_PER_MM:+.4f} mm "
              f"after reset — servo may not be holding")
    else:
        print(f"[probe] events reset; servo holding at {start_mm:+.4f} mm "
              f"(1.0s settle waited)")

    # Walk ONE direction only: pick the direction that gets us AWAY from
    # the closer edge (estimated using the documented 5mm travel). After
    # hitting ANY hard limit, the Juno controller SILENTLY REJECTS all
    # subsequent move commands — a second direction walk would fail.
    # Instead we discover one edge empirically and derive the other from
    # the documented Total Travel spec.
    if start_mm > 1.0:
        # Closer to + edge: walk negative to discover the -edge.
        print(f"[probe] walking negative first (stage closer to +edge)")
        walk_direction = -1
    else:
        # Closer to - edge: walk positive to discover the +edge.
        print(f"[probe] walking positive first (stage closer to -edge)")
        walk_direction = +1

    discovered_edge_counts = _probe_one_direction(bus, start_counts, walk_direction)
    discovered_edge_mm = discovered_edge_counts / dof_init.COUNTS_PER_MM
    edge_label = "+" if walk_direction > 0 else "-"
    print(f"[probe] discovered {edge_label} edge: {discovered_edge_mm:+.4f} mm")

    # Derive the UNdiscovered edge from documented Total Travel. DOF-5
    # datasheet specs 5mm usable travel / 6mm hard-stop travel. Measured
    # value from earlier successful probes = ~5.99mm; use that.
    DOCUMENTED_TRAVEL_MM = 5.99
    if walk_direction > 0:
        pos_limit_mm = discovered_edge_mm
        neg_limit_mm = pos_limit_mm - DOCUMENTED_TRAVEL_MM
    else:
        neg_limit_mm = discovered_edge_mm
        pos_limit_mm = neg_limit_mm + DOCUMENTED_TRAVEL_MM

    travel_mm = pos_limit_mm - neg_limit_mm
    print(f"[probe] derived envelope: [{neg_limit_mm:+.4f}, "
          f"{pos_limit_mm:+.4f}] mm  (travel = {travel_mm:.4f} mm)")
    print(f"[probe] (one edge discovered empirically, other derived from "
          f"DOF-5 datasheet spec of {DOCUMENTED_TRAVEL_MM}mm total travel)")

    # The probe's walk ended by hitting a hard limit, which leaves the
    # Juno controller SILENTLY REJECTING all subsequent move commands.
    # Re-init the controller to clear the post-limit lockout. Empirically
    # verified: init_drive() does NOT shift the encoder origin (drift is
    # typically <1µm), so the discovered envelope coordinates remain valid.
    print(f"[probe] re-initializing drive to clear post-limit lockout...")
    pre_init_pos = dof_init.get_pos_counts(bus)
    dof_init.init_drive(bus)
    post_init_pos = dof_init.get_pos_counts(bus)
    drift_mm = (post_init_pos - pre_init_pos) / dof_init.COUNTS_PER_MM
    print(f"[probe] re-init drift: {drift_mm:+.4f} mm "
          f"({pre_init_pos} → {post_init_pos} counts)")
    if abs(drift_mm) > 0.001:
        print(f"[probe] WARNING: encoder origin shifted by {drift_mm:+.4f} mm "
              f"during re-init — discovered envelope coordinates may be "
              f"slightly off")
    return neg_limit_mm, pos_limit_mm


def configure_runtime_envelope(neg_limit_mm, pos_limit_mm, max_move_mm):
    """Configure the bench's SOFT_LIMIT and HOME constants based on the
    empirically discovered envelope. Returns True if max_move fits.

    Plan:
      - soft-limit window = discovered envelope minus SAFETY_MARGIN at
        each end (so we never command right up to the physically-clamped
        edge).
      - HOME = midpoint of the soft-limit window, so symmetric moves
        up to ~half the travel fit centered.
    """
    global SOFT_LIMIT_NEG_MM, SOFT_LIMIT_POS_MM, HOME_MM
    SOFT_LIMIT_NEG_MM = neg_limit_mm + SAFETY_MARGIN_MM
    SOFT_LIMIT_POS_MM = pos_limit_mm - SAFETY_MARGIN_MM
    HOME_MM = (SOFT_LIMIT_NEG_MM + SOFT_LIMIT_POS_MM) / 2.0
    travel_mm = SOFT_LIMIT_POS_MM - SOFT_LIMIT_NEG_MM
    print(f"[envelope] configured: soft=[{SOFT_LIMIT_NEG_MM:+.3f}, "
          f"{SOFT_LIMIT_POS_MM:+.3f}] mm  home={HOME_MM:+.3f} mm  "
          f"usable travel={travel_mm:.3f} mm")
    if max_move_mm > travel_mm:
        print(f"[envelope] ERROR: max requested move {max_move_mm:.3f} mm "
              f"exceeds usable travel {travel_mm:.3f} mm")
        return False
    if max_move_mm > travel_mm - 2 * SAFETY_MARGIN_MM:
        print(f"[envelope] note: largest move ({max_move_mm:.2f}mm) uses "
              f"reduced safety margin (runs close to limits)")
    return True


def run_single_trial(bus, target_counts, home_counts, vel_mm_s=125.0,
                     acc_mm_s2=6000.0, distance_um=0):
    """Run one trial with multi-channel Juno telemetry.
    
    In addition to the existing position polling, each sample reads:
      - GetActualVelocity (0xAD) → real stage velocity from encoder differentiation
      - GetCommandedPosition (0x4A) → trajectory planner's intended position
                                       (difference from actual = following error)
      - GetMotorCommand (0xB6) → voice-coil current ≈ force on payload
    
    After the move, snapshot statistics are computed: peak velocity and
    where in the motion it occurred, velocity at 25/50/75% of motion,
    peak motor command (force), peak and mean following error. The
    ideal trapezoidal-motion reference is also computed from v/a/distance
    so the plots can overlay measured vs calculated.
    
    vel_mm_s, acc_mm_s2, distance_um are needed for the calculated
    trapezoidal reference (peak velocity, ideal motion time). They don't
    affect the move itself — motion params are already set by the caller.
    """
    t_cmd = time.perf_counter_ns()
    dof_init.sr(bus, dof_init.OP_SET_POSITION, struct.pack(">i", target_counts))
    dof_init.sr(bus, dof_init.OP_UPDATE)
    
    # Rich multi-channel trace: each sample is
    # (t_ns_since_cmd, pos_counts, vel_mm_s, cmd_counts, motor_cmd).
    # vel_mm_s computed at sample time from GetActualVelocity register.
    trace = []
    
    t_react = None
    t_engage = None
    t_complete = None
    in_settle_band_since = None
    
    # Per-trial peak/snapshot trackers (motion phase only)
    peak_v_mm_s = 0.0
    peak_v_t_ns = None       # t_ns_since_cmd at peak velocity
    peak_motor = 0
    peak_motor_t_ns = None
    peak_following_err = 0
    following_err_sum = 0
    following_err_n = 0
    # Velocity samples during motion (t, v) for later 25/50/75% interpolation
    motion_v_samples = []    # list of (t_ns_since_cmd, vel_mm_s)
    
    while True:
        
        t_now = time.perf_counter_ns()
        # === Multi-channel poll: position + velocity + commanded pos + motor cmd.
        # Each sr() round-trip is ~370-430 µs (per smoke test), so the full
        # 4-read poll takes ~1.5 ms per sample. That's ~4x slower than the
        # position-only poll, but it captures real payload dynamics.
        try:
            position = dof_init.get_pos_counts(bus)
            velocity_mm_s = dof_init.get_velocity_mm_s(bus)
            commanded = dof_init.get_commanded_pos_counts(bus)
            motor_cmd = dof_init.get_motor_cmd(bus)
        except RuntimeError:
            # CAN read failed — fall back to position-only for this sample
            # so we don't lose the trial entirely. Velocity/cmd/motor
            # values from last successful read will be reused.
            try:
                position = dof_init.get_pos_counts(bus)
            except RuntimeError:
                break  # CAN is fully dead, abort trial
            velocity_mm_s = 0.0
            commanded = position
            motor_cmd = 0
        
        # === SAFETY: abort trial immediately if position leaves the
        # soft-limit window. Commands can glitch or the servo can run away
        # under backlog; we must never let the stage reach a hard stop.
        if not in_safe_travel(position):
            pos_mm = position / dof_init.COUNTS_PER_MM
            print(f"[trial] ABORT: position {pos_mm:+.4f} mm outside safe "
                  f"travel [{SOFT_LIMIT_NEG_MM:+.2f}, {SOFT_LIMIT_POS_MM:+.2f}] "
                  f"— breaking to avoid edge collision")
            break

        # engage
        if t_engage is None and t_react is not None and abs(position - target_counts) <= ENGAGE_BAND_COUNTS:
            t_engage = t_now
        
        if(t_react is None and abs(position - home_counts) > NOISE_COUNTS):
            t_react = t_now

        # === Multi-channel telemetry tracking (only during motion phase,
        # i.e. once we've started moving). Skip the pre-move dead-time
        # samples (the stage is stationary) so peaks/snapshots are
        # meaningful.
        if t_react is not None and t_complete is None:
            abs_v = abs(velocity_mm_s)
            if abs_v > peak_v_mm_s:
                peak_v_mm_s = abs_v
                peak_v_t_ns = t_now - t_cmd
            abs_m = abs(motor_cmd)
            if abs_m > peak_motor:
                peak_motor = abs_m
                peak_motor_t_ns = t_now - t_cmd
            following_err = abs(commanded - position)
            if following_err > peak_following_err:
                peak_following_err = following_err
            following_err_sum += following_err
            following_err_n += 1
            motion_v_samples.append((t_now - t_cmd, velocity_mm_s))
        
        trace.append((t_now - t_cmd, position, velocity_mm_s, commanded, motor_cmd))
        
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

    # === Payload characterization statistics
    result.peak_velocity_mm_s = peak_v_mm_s
    result.peak_motor_cmd = peak_motor
    result.peak_following_error_counts = peak_following_err
    result.mean_following_error_counts = (
        following_err_sum / following_err_n if following_err_n > 0 else float('nan')
    )
    # Fractional position of peak velocity within the motion interval
    if (peak_v_t_ns is not None and t_react is not None
            and t_complete is not None and t_complete > t_react):
        motion_dur_ns = t_complete - t_react
        result.peak_velocity_t_frac = (peak_v_t_ns - (t_react - t_cmd)) / motion_dur_ns
    if (peak_motor_t_ns is not None and t_react is not None
            and t_complete is not None and t_complete > t_react):
        motion_dur_ns = t_complete - t_react
        result.peak_motor_cmd_t_frac = (peak_motor_t_ns - (t_react - t_cmd)) / motion_dur_ns

    # === Velocity milestones at 25/50/75% of motion interval
    # (Only meaningful if we have a clean t_react → t_complete interval
    # AND collected velocity samples during that window.)
    if (t_react is not None and t_complete is not None
            and t_complete > t_react and len(motion_v_samples) > 4):
        # Time of motion start relative to t_cmd (= t_react_ns - t_cmd_ns)
        motion_start_ns = t_react - t_cmd
        motion_dur_ns = t_complete - t_react
        for frac, attr in [(0.25, 'velocity_at_25pct_mm_s'),
                           (0.50, 'velocity_at_50pct_mm_s'),
                           (0.75, 'velocity_at_75pct_mm_s')]:
            target_t = motion_start_ns + frac * motion_dur_ns
            # Linear interp between the two nearest samples
            setattr(result, attr,
                    _interp_velocity_at(motion_v_samples, target_t))

        # === Calculated trapezoidal reference for this move
        calc_peak, calc_motion_us = _calc_trapezoidal_reference(
            distance_um, vel_mm_s, acc_mm_s2)
        result.calc_peak_velocity_mm_s = calc_peak
        result.calc_motion_us = calc_motion_us

        # === Rate-of-change slopes (avg accel/decel). For a triangular
        # move peak velocity = a*t_accel, so avg_accel = peak_v / t_to_peak.
        # We measure t_to_peak from peak_v_t_ns - motion_start_ns.
        if (peak_v_t_ns is not None and peak_v_mm_s > 0.1
                and t_react is not None and t_complete is not None):
            t_to_peak_s = (peak_v_t_ns - motion_start_ns) / 1e9
            if t_to_peak_s > 0.001:
                result.avg_accel_mm_s2 = peak_v_mm_s / t_to_peak_s
            # Decel: from peak to end of motion.
            t_from_peak_s = (motion_dur_ns - (peak_v_t_ns - motion_start_ns)) / 1e9
            if t_from_peak_s > 0.001:
                result.avg_decel_mm_s2 = peak_v_mm_s / t_from_peak_s

    return result, trace


def _interp_velocity_at(samples, target_t_ns):
    """Linear-interpolate velocity at target_t_ns given list of
    (t_ns, vel_mm_s) samples. Returns 0.0 if no samples bracket target."""
    if not samples:
        return 0.0
    # Find bracketing pair
    for i in range(len(samples) - 1):
        t0, v0 = samples[i]
        t1, v1 = samples[i + 1]
        if t0 <= target_t_ns <= t1:
            if t1 == t0:
                return v0
            frac = (target_t_ns - t0) / (t1 - t0)
            return v0 + frac * (v1 - v0)
    # Out of range — return nearest endpoint
    if target_t_ns <= samples[0][0]:
        return samples[0][1]
    return samples[-1][1]


def _calc_trapezoidal_reference(distance_um, vel_mm_s, acc_mm_s2):
    """Calculate the IDEAL trapezoidal-motion peak velocity and total
    motion time for a move of `distance_um` at velocity `vel_mm_s` and
    acceleration `acc_mm_s2`. Used as the reference curve to overlay on
    measured velocity traces.

    Returns (peak_velocity_mm_s, motion_time_us).

    For a triangular move (distance < 2*d_accel): peak v is set by the
    geometry, not by the velocity cap.
    For a trapezoidal move (distance >= 2*d_accel): peak v = vel_mm_s.

    d_accel = v_max^2 / (2*a) is the distance required to reach v_max.
    For v=125, a=6000: d_accel = 1.30 mm; 2*d_accel = 2.60 mm.
    """
    d_mm = distance_um / 1000.0
    if d_mm <= 0 or vel_mm_s <= 0 or acc_mm_s2 <= 0:
        return float('nan'), float('nan')
    # Distance to accelerate from 0 to v_max (and back to 0)
    d_accel_mm = (vel_mm_s ** 2) / (2 * acc_mm_s2)
    if d_mm >= 2 * d_accel_mm:
        # Trapezoidal: accel + cruise + decel
        peak_v = vel_mm_s
        t_accel_s = vel_mm_s / acc_mm_s2
        t_cruise_s = (d_mm - 2 * d_accel_mm) / vel_mm_s
        t_total_s = 2 * t_accel_s + t_cruise_s
    else:
        # Triangular: never reaches v_max. peak_v = sqrt(a*d/2).
        # Time to peak = sqrt(d/a). Total time = 2 * t_peak.
        peak_v = (acc_mm_s2 * d_mm / 2.0) ** 0.5
        t_total_s = 2 * ((d_mm / acc_mm_s2) ** 0.5)
    return peak_v, t_total_s * 1e6  # s → µs


def safe_start_for_move(direction, distance_um):
    """Compute a safe starting position (in mm) for a move of given
    direction and magnitude, plus the resulting target.

    For small moves we start near 0 (servo auto-cal origin). For moves that
    won't fit centered on 0, we start at the opposite edge of the travel
    envelope so the stage has room in the move direction.
    e.g. +4 mm move → start at -2.4 mm edge, target at +1.6 mm edge
         (4.0 mm of travel — exactly the full envelope).

    The full travel envelope is 1.6 - (-2.4) = 4.0 mm. For moves ≤ ~3.8 mm
    we keep a SAFETY_MARGIN_MM buffer on each end. For the very largest
    move (4.0 mm) we shrink the margin adaptively so it still fits —
    verified safe by prior `autofocus_latency_webapp --latency-test` runs.

    Returns (start_mm, target_mm). Caller commands the start as the
    "home" for that trial (after settling there).
    """
    move_mm = distance_um / 1000.0
    full_travel_mm = SOFT_LIMIT_POS_MM - SOFT_LIMIT_NEG_MM  # 4.0 mm

    if move_mm > full_travel_mm:
        # Sanity: this should never happen with our default distance list
        raise ValueError(
            f"Move of {move_mm:.3f} mm exceeds full stage travel "
            f"({full_travel_mm:.3f} mm) — would hit hard stops")

    # Adaptive margin: keep SAFETY_MARGIN_MM on each end if the move fits;
    # otherwise shrink margin so the move just fits end-to-end. At max
    # (4.0 mm move) the margin goes to 0, meaning start and target land
    # exactly on the soft-limit edges.
    margin = min(SAFETY_MARGIN_MM, (full_travel_mm - move_mm) / 2.0)
    if margin < 0:
        margin = 0.0

    # Try a symmetric layout first: start at HOME_MM, target at HOME_MM ± move.
    symmetric_target_mm = HOME_MM + direction * move_mm
    # Does that target fit within bounds with margin on the move-direction
    # side, AND is HOME_MM itself inside the buffered envelope?
    if direction > 0:
        symmetric_fits = (symmetric_target_mm <= SOFT_LIMIT_POS_MM - margin
                          and HOME_MM >= SOFT_LIMIT_NEG_MM + margin)
    else:
        symmetric_fits = (symmetric_target_mm >= SOFT_LIMIT_NEG_MM + margin
                          and HOME_MM <= SOFT_LIMIT_POS_MM - margin)

    if symmetric_fits:
        start_mm = HOME_MM
        target_mm = symmetric_target_mm
    else:
        # Asymmetric: start at the edge opposite to the move direction so
        # the stage has the full travel envelope available for the move.
        if direction > 0:
            start_mm = SOFT_LIMIT_NEG_MM + margin
            target_mm = start_mm + move_mm
        else:
            start_mm = SOFT_LIMIT_POS_MM - margin
            target_mm = start_mm - move_mm
    return start_mm, target_mm


def verify_travel_envelope(bus, distances_um):
    """Physically pre-flight every position the bench will command.

    For each (distance, direction) the bench would run, compute the
    start_mm and target_mm via safe_start_for_move(), then physically
    move the stage to each one and confirm it can actually get there
    and settle. Abort BEFORE the bench starts collecting data if any
    position is unreachable. This is the "known good spot" safety net:
    fail fast on hardware/calibration problems instead of having trials
    silently skipped mid-run.

    Returns True if all positions verified, False otherwise.
    """
    print(f"[preflight] verifying reachability of all positions the bench "
          f"will command...")
    positions_to_test = set()
    for dist in distances_um:
        for _, direction in DIRECTIONS:
            start_mm, target_mm = safe_start_for_move(direction, dist)
            positions_to_test.add(round(start_mm, 4))
            positions_to_test.add(round(target_mm, 4))
    # Always include HOME_MM
    positions_to_test.add(round(HOME_MM, 4))
    positions_to_test = sorted(positions_to_test)

    print(f"[preflight] {len(positions_to_test)} unique positions to verify "
          f"across {len(distances_um)*len(DIRECTIONS)} configs")
    print(f"[preflight]   range: {positions_to_test[0]:+.3f} mm → "
          f"{positions_to_test[-1]:+.3f} mm")

    failures = []
    # Allow a small floating-point tolerance on the boundary check so that
    # positions like SOFT_LIMIT_POS_MM - SAFETY_MARGIN_MM (which is exactly
    # where the asymmetric-move start lands) don't get falsely rejected
    # as "outside buffered soft-limit window".
    BOUNDARY_EPS_MM = 0.0001  # 0.1 µm tolerance
    lo_buffered = SOFT_LIMIT_NEG_MM + SAFETY_MARGIN_MM - BOUNDARY_EPS_MM
    hi_buffered = SOFT_LIMIT_POS_MM - SAFETY_MARGIN_MM + BOUNDARY_EPS_MM
    for pos_mm in positions_to_test:
        # Check soft-limit math first (should never fail given safe_start_for_move,
        # but defends against future config drift)
        if not (lo_buffered <= pos_mm <= hi_buffered):
            print(f"[preflight]   ✗ {pos_mm:+.3f} mm — outside buffered "
                  f"soft-limit window [{lo_buffered+BOUNDARY_EPS_MM:+.3f}, "
                  f"{hi_buffered-BOUNDARY_EPS_MM:+.3f}] mm")
            failures.append(pos_mm)
            continue
        # Physically move there and confirm settle
        ok = go_home(bus, target_mm=pos_mm,
                     label=f"preflight {pos_mm:+.3f}", verbose=False)
        if not ok:
            print(f"[preflight]   ✗ {pos_mm:+.3f} mm — UNREACHABLE "
                  f"(settle failed)")
            failures.append(pos_mm)
        else:
            print(f"[preflight]   ✓ {pos_mm:+.3f} mm")
    if failures:
        print(f"[preflight] FAILED — {len(failures)}/{len(positions_to_test)} "
              f"positions unreachable: {failures}")
        return False
    print(f"[preflight] PASSED — all {len(positions_to_test)} positions "
          f"reachable and settle-confirmed")
    # Return to HOME_MM as the known-good starting position
    go_home(bus, target_mm=HOME_MM, label="preflight-return", verbose=False)
    return True


def run_benchmark_suite(bus, directory, vel_mm_s, acc_mm_s2, trial_overrides,
                        distances_um=None):
    configs = []

    # CLI override takes precedence; otherwise use module default
    if distances_um is None:
        distances_um = DISTANCES_UM

    for dist in distances_um:
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
        # === Determine the per-config starting position. For small moves we
        # start at HOME_MM=0 (symmetric, plenty of margin). For moves whose
        # magnitude is larger than ~half the smaller of the two travel
        # envelopes, we start at the opposite edge so the stage has room.
        # Example: +4 mm move starts at -2.4 mm edge, target +1.6 mm edge.
        start_mm, target_mm = safe_start_for_move(
            cfg["direction"], cfg["distance_um"])
        start_counts = int(round(start_mm * dof_init.COUNTS_PER_MM))
        target_counts_expected = int(round(target_mm * dof_init.COUNTS_PER_MM))
        move_mm = cfg["direction"] * cfg["distance_um"] / 1000.0
        print(f"\n[bench] config: {cfg['name']}  "
              f"({cfg['distance_um']}µm, dir={cfg['direction']:+d})  "
              f"{cfg['trials']} trials  "
              f"start={start_mm:+.3f}mm → target={target_mm:+.3f}mm "
              f"(move {move_mm:+.3f}mm)")
        config_results = []
        for trial_i in range(cfg["trials"]):
            # === SAFETY: re-position to this config's start before every
            # trial. Without this, 100 consecutive same-direction moves
            # walk the stage and it collides with the soft limits.
            if not go_home(bus, target_mm=start_mm,
                           label=f"{cfg['name']}#{trial_i}", verbose=False):
                print(f"[bench] ABORT: re-positioning failed before trial "
                      f"{global_trial_id} ({cfg['name']}); stopping bench "
                      f"to avoid edge collision")
                break
            # Re-apply test motion params (go_home sets conservative ones)
            dof_init.set_motion_params(bus, vel_mm_s=vel_mm_s,
                                       acc_mm_s2=acc_mm_s2)

            # Sample home AFTER settling at the start position.
            home = dof_init.get_pos_counts(bus)
            home_mm = home / dof_init.COUNTS_PER_MM
            if abs(home - start_counts) > HOME_DRIFT_WARN_COUNTS:
                print(f"[bench] WARNING: home sample {home_mm:+.4f} mm "
                      f"differs from start {start_mm:+.3f} mm; drift > "
                      f"{HOME_DRIFT_WARN_COUNTS} counts")

            # === SAFETY: pre-flight check that the expected target is
            # inside the safe travel window. If not, skip the trial.
            if not in_safe_travel(target_counts_expected, extra_margin_mm=0.0):
                print(f"[bench] SKIP trial {global_trial_id} "
                      f"({cfg['name']}): expected target "
                      f"{target_mm:+.3f} mm outside safe travel")
                result = TrialResult()
                result.trial_id = global_trial_id
                result.config = cfg["name"]
                result.direction = cfg["direction"]
                result.target_counts = -1
                result.home_counts = home
                result.final_error_counts = -1
                all_results.append(result)
                config_results.append(result)
                global_trial_id += 1
                continue

            # Compute the actual target relative to the sampled home (so
            # small drift in the start sample doesn't bias the move size).
            # But also verify the actual target is still in safe travel.
            target = home + cfg["direction"] * int(
                cfg["distance_um"] * dof_init.COUNTS_PER_MM / 1000)
            if not in_safe_travel(target, extra_margin_mm=0.0):
                print(f"[bench] SKIP trial {global_trial_id} "
                      f"({cfg['name']}): sampled target "
                      f"{target/dof_init.COUNTS_PER_MM:+.3f} mm "
                      f"outside safe travel")
                result = TrialResult()
                result.trial_id = global_trial_id
                result.config = cfg["name"]
                result.direction = cfg["direction"]
                result.target_counts = -1
                result.home_counts = home
                result.final_error_counts = -1
                all_results.append(result)
                config_results.append(result)
                global_trial_id += 1
                continue

            result, trace = run_single_trial(bus, target, home,
                                             vel_mm_s=vel_mm_s,
                                             acc_mm_s2=acc_mm_s2,
                                             distance_um=cfg["distance_um"])

            result.trial_id = global_trial_id
            result.config = cfg["name"]
            result.direction = cfg["direction"]

            # One-line per-trial status. Show total latency if the trial
            # completed, else mark it so it's obvious from the log.
            if result.t_complete_ns is not None:
                status = (f"total={result.total_us/1000:6.2f}ms "
                          f"(react={result.reaction_us/1000:.2f} "
                          f"motion={result.motion_us/1000:.2f} "
                          f"settle={result.settle_us/1000:.2f})  "
                          f"v_pk={result.peak_velocity_mm_s:.1f}mm/s "
                          f"F_pk={result.peak_motor_cmd} "
                          f"err_pk={result.peak_following_error_counts}cts")
            else:
                status = "TIMEOUT/INCOMPLETE"
            print(f"  {cfg['name']} #{trial_i:3d}: {status}")

            global_trial_id = global_trial_id + 1

            all_results.append(result)
            config_results.append(result)
            all_traces.setdefault(cfg["name"], []).append(trace)
        else:
            # inner loop completed without break — print config summary
            _print_config_summary(cfg["name"], config_results)
            continue
        # inner loop broke (e.g. homing failure) — abort outer loop too
        break

    write_config_json(directory, configs, vel_mm_s, acc_mm_s2)
    write_trial_summary_csv(directory, all_results)
    write_raw_traces(directory, all_traces)
    write_summary_json(directory, all_results)


def _print_config_summary(config_name, results):
    """Print a one-line median/p95 summary after each config block."""
    from statistics import median
    totals = [r.total_us for r in results
              if r.total_us == r.total_us and r.t_complete_ns is not None]
    reacts = [r.reaction_us for r in results
              if r.reaction_us == r.reaction_us]
    motions = [r.motion_us for r in results
               if r.motion_us == r.motion_us]
    settles = [r.settle_us for r in results
               if r.settle_us == r.settle_us]
    n_complete = len(totals)
    n_total = len(results)
    if not totals:
        print(f"[bench] {config_name}: no completed trials "
              f"({n_total} attempted)")
        return
    s = sorted(totals)
    p95 = s[max(0, min(n_complete - 1, int(round(0.95 * (n_complete - 1)))))]
    print(f"[bench] {config_name}: n={n_complete}/{n_total}  "
          f"total med={median(totals)/1000:.2f}ms  "
          f"p95={p95/1000:.2f}ms  "
          f"(react {median(reacts)/1000:.2f} / "
          f"motion {median(motions)/1000:.2f} / "
          f"settle {median(settles)/1000:.2f} ms)")


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
        "safety": {
            "soft_limit_pos_mm": SOFT_LIMIT_POS_MM,
            "soft_limit_neg_mm": SOFT_LIMIT_NEG_MM,
            "travel_source": "empirically discovered at startup via probe_travel_envelope()",
            "home_mm": HOME_MM,
            "safety_margin_mm": SAFETY_MARGIN_MM,
            "home_drift_warn_counts": HOME_DRIFT_WARN_COUNTS,
            "re_home_between_trials": True,
            "abort_on_travel_violation": True,
            "preflight_verify_positions": True,
        },
        "configurations": configs,
    }
    with open(out_dir / "config.json", "w") as f:
        json.dump(config_dict, f, indent=2)


def write_trial_summary_csv(out_dir, all_results):
    """Write one row per TrialResult, including the new payload-
    characterization columns (peak velocity, motor command, following
    error, calculated trapezoidal reference, accel/decel slopes)."""
    import csv
    header = [
        # existing latency columns
        "trial_id", "config", "direction", "target_counts", "home_counts",
        "t_cmd_ns", "t_react_ns", "t_engage_ns", "t_complete_ns",
        "reaction_us", "motion_us", "settle_us", "total_us",
        "final_pos_counts", "final_error_counts",
        # new payload-characterization columns
        "peak_velocity_mm_s", "peak_velocity_t_frac",
        "velocity_at_25pct_mm_s", "velocity_at_50pct_mm_s",
        "velocity_at_75pct_mm_s",
        "peak_motor_cmd", "peak_motor_cmd_t_frac",
        "peak_following_error_counts", "mean_following_error_counts",
        "calc_peak_velocity_mm_s", "calc_motion_us",
        "avg_accel_mm_s2", "avg_decel_mm_s2",
    ]
    def _fmt_num(v):
        """Format a number for CSV; NaN/None → empty string."""
        if v is None:
            return ""
        try:
            fv = float(v)
            if fv != fv:  # NaN
                return ""
            if fv == int(fv) and abs(fv) < 1e15:
                return str(int(fv))
            return f"{fv:.6g}"
        except (TypeError, ValueError):
            return str(v)
    with open(out_dir / "trial_summary.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for r in all_results:
            writer.writerow([
                r.trial_id, r.config, r.direction,
                r.target_counts, r.home_counts,
                r.t_cmd_ns, r.t_react_ns, r.t_engage_ns, r.t_complete_ns,
                _fmt_num(r.reaction_us), _fmt_num(r.motion_us),
                _fmt_num(r.settle_us), _fmt_num(r.total_us),
                r.final_pos_counts, r.final_error_counts,
                _fmt_num(r.peak_velocity_mm_s),
                _fmt_num(r.peak_velocity_t_frac),
                _fmt_num(r.velocity_at_25pct_mm_s),
                _fmt_num(r.velocity_at_50pct_mm_s),
                _fmt_num(r.velocity_at_75pct_mm_s),
                r.peak_motor_cmd,
                _fmt_num(r.peak_motor_cmd_t_frac),
                r.peak_following_error_counts,
                _fmt_num(r.mean_following_error_counts),
                _fmt_num(r.calc_peak_velocity_mm_s),
                _fmt_num(r.calc_motion_us),
                _fmt_num(r.avg_accel_mm_s2),
                _fmt_num(r.avg_decel_mm_s2),
            ])


def write_raw_traces(out_dir, all_traces):
    """Write one raw_trace_<config>.csv per config.

    Each file has columns:
        trial_id, t_ns_since_cmd, position_counts,
        velocity_mm_s, commanded_counts, motor_cmd
    All trials for a config are interleaved by trial_id. The new columns
    are populated by the multi-channel poll in run_single_trial; traces
    recorded before this update have empty values for the new columns
    (still loadable by the plot script via flexible parsing).
    """
    import csv
    for config_name, traces in all_traces.items():
        path = out_dir / f"raw_trace_{config_name}.csv"
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["trial_id", "t_ns_since_cmd", "position_counts",
                             "velocity_mm_s", "commanded_counts", "motor_cmd"])
            for trial_id, trace in enumerate(traces):
                for sample in trace:
                    # Backward-compat: sample may be 2-tuple (old) or
                    # 5-tuple (new). Always emit 6 columns, empty for old.
                    if len(sample) == 5:
                        t_ns, pos, vel, cmd, mot = sample
                        writer.writerow([trial_id, t_ns, pos, vel, cmd, mot])
                    elif len(sample) == 2:
                        t_ns, pos = sample
                        writer.writerow([trial_id, t_ns, pos, "", "", ""])
                    else:
                        writer.writerow([trial_id] + list(sample))


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
            # Payload-characterization stats (peaks are typically the
            # interesting per-config quantity; we report median + p95
            # across the 100 trials per config)
            "peak_velocity_mm_s":           _stats([r.peak_velocity_mm_s for r in results
                                                    if r.peak_velocity_mm_s == r.peak_velocity_mm_s]),
            "velocity_at_50pct_mm_s":       _stats([r.velocity_at_50pct_mm_s for r in results
                                                    if r.velocity_at_50pct_mm_s == r.velocity_at_50pct_mm_s]),
            "peak_motor_cmd":               _stats([float(r.peak_motor_cmd) for r in results
                                                    if r.peak_motor_cmd > 0]),
            "peak_following_error_counts":  _stats([float(r.peak_following_error_counts) for r in results
                                                    if r.peak_following_error_counts > 0]),
            "mean_following_error_counts":  _stats([r.mean_following_error_counts for r in results
                                                    if r.mean_following_error_counts == r.mean_following_error_counts]),
            "calc_peak_velocity_mm_s":      _stats([r.calc_peak_velocity_mm_s for r in results
                                                    if r.calc_peak_velocity_mm_s == r.calc_peak_velocity_mm_s]),
            "avg_accel_mm_s2":              _stats([r.avg_accel_mm_s2 for r in results
                                                    if r.avg_accel_mm_s2 == r.avg_accel_mm_s2]),
            "avg_decel_mm_s2":              _stats([r.avg_decel_mm_s2 for r in results
                                                    if r.avg_decel_mm_s2 == r.avg_decel_mm_s2]),
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
    ap.add_argument("--distances",
                    default="10,25,50,100,200,500,1000,1500,2000,2500,3000,3500,4000",
                    help="comma-separated move distances in µm "
                         "(default spans 10µm to 4mm)")
    ap.add_argument("--trials", type=int, default=100,
                    help="trials per configuration")
    ap.add_argument("--velocity", type=float, default=1.0,
                    help="move velocity in mm/s (default: 1.0, max: 125)")
    ap.add_argument("--acceleration", type=float, default=20.0,
                    help="move acceleration in mm/s² (default: 20, max: 6000)")
    ap.add_argument("--no-plot", action="store_true",
                    help="skip automatic plotting after benchmark")
    ap.add_argument("--probe-only", action="store_true",
                    help="run only the travel-envelope probe and exit "
                         "(useful for testing the probe in isolation)")
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

        # --probe-only: just run the travel-envelope probe and exit. Useful
        # for testing the probe in isolation without running 2600 trials.
        if args.probe_only:
            neg_mm, pos_mm = probe_travel_envelope(bus)
            travel_mm = pos_mm - neg_mm
            print()
            print(f"=== PROBE-ONLY RESULT ===")
            print(f"  envelope: [{neg_mm:+.4f}, {pos_mm:+.4f}] mm")
            print(f"  travel:   {travel_mm:.4f} mm")
            if configure_runtime_envelope(neg_mm, pos_mm, max_move_mm=4.0):
                print(f"  configured home: {HOME_MM:+.4f} mm")
                print(f"  configured soft limits: "
                      f"[{SOFT_LIMIT_NEG_MM:+.4f}, {SOFT_LIMIT_POS_MM:+.4f}] mm")
            return

        # Set motion parameters (required before any move)
        vel = args.velocity
        acc = args.acceleration
        dof_init.set_motion_params(bus, vel_mm_s=vel, acc_mm_s2=acc)
        print(f"[bench] velocity={vel} mm/s, acceleration={acc} mm/s²")

        # Parse distance list from CLI now so we know the max move before
        # probing the envelope.
        distances_um = [int(d.strip()) for d in args.distances.split(",")]
        max_move_mm = max(distances_um) / 1000.0
        print(f"[bench] distances: {distances_um} µm  "
              f"({len(distances_um)*2*args.trials} total trials planned, "
              f"max move = {max_move_mm:.2f} mm)")

        # Warn if velocity is too high for smallest distance relative to polling rate
        min_dist = min(distances_um)
        transit_ms = min_dist / vel  # ms
        if transit_ms < 0.5:
            print(f"[bench] ⚠  {min_dist}µm move at {vel} mm/s = ~{transit_ms:.2f}ms transit "
                  f"— less than one CAN poll (~0.37ms)."
                  f" Events may be missed for small distances.")

        # === CRITICAL SAFETY: empirically discover the stage's actual
        # travel envelope. The encoder origin varies per power-cycle
        # (observed +5.87mm, +2.86mm, +0.0033mm in different runs) so
        # hardcoded [-1.5, +3.5] mm limits are meaningless across runs.
        # The probe commands large moves and observes where the controller
        # silently clamps at its own internal limit, giving us the actual
        # reachable envelope for this specific power-up.
        neg_limit_mm, pos_limit_mm = probe_travel_envelope(bus)

        # Configure runtime soft limits based on discovered envelope
        if not configure_runtime_envelope(neg_limit_mm, pos_limit_mm,
                                          max_move_mm):
            raise RuntimeError(
                f"Discovered travel envelope ({pos_limit_mm - neg_limit_mm:.3f} mm) "
                f"is too small for the largest requested move "
                f"({max_move_mm:.3f} mm). Reduce --distances max or "
                f"check for stage obstruction.")

        # === SAFETY: move the stage to the discovered HOME position
        # (midpoint of usable envelope). This is the "known good spot" —
        # the only position guaranteed to be safe regardless of where the
        # encoder zero landed.
        print(f"[bench] homing to discovered home {HOME_MM:+.3f} mm "
              f"(soft limits [{SOFT_LIMIT_NEG_MM:+.2f}, "
              f"{SOFT_LIMIT_POS_MM:+.2f}] mm)")
        if not go_home(bus, label="initial", verbose=True):
            raise RuntimeError(
                "Initial homing to discovered home failed — refusing to "
                "start bench. Stage may be obstructed.")
        # Re-apply test motion params (go_home uses conservative ones)
        dof_init.set_motion_params(bus, vel_mm_s=vel, acc_mm_s2=acc)

        # === SAFETY: physically verify every position the bench will
        # command is reachable, now against the discovered envelope.
        if not verify_travel_envelope(bus, distances_um):
            raise RuntimeError(
                "Pre-flight travel envelope check FAILED — one or more "
                "planned positions are unreachable even within the "
                "discovered envelope. Check stage calibration / power / "
                "physical obstruction and re-run.")
        # Re-apply motion params after pre-flight
        dof_init.set_motion_params(bus, vel_mm_s=vel, acc_mm_s2=acc)

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
        # Return to the known home reference and confirm settle
        home_counts_ref = int(round(HOME_MM * dof_init.COUNTS_PER_MM))
        dof_init.sr(bus, dof_init.OP_SET_POSITION,
                    struct.pack(">i", home_counts_ref))
        dof_init.sr(bus, dof_init.OP_UPDATE)
        # Make sure the dry-run's return move has fully settled so the first
        # real trial's `home` sample isn't tainted by servo oscillation.
        if not settle_guard(bus, home_counts_ref, label="dry-run-return",
                            verbose=True):
            print(f"[bench] WARNING: dry-run return move did not settle cleanly")
        # Re-apply test motion params (dry-run-return used conservative)
        dof_init.set_motion_params(bus, vel_mm_s=vel, acc_mm_s2=acc)

        # Run full benchmark suite
        run_benchmark_suite(bus, out_dir, vel, acc, trial_overrides=None,
                            distances_um=distances_um)
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
    
    
    
    

        
        
        
        
        
    
    

    
    
    
    
