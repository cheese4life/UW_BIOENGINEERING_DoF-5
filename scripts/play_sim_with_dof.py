"""Live cornea-focus demo: simulated OCT video -> surface detection ->
controller -> DOF driver, with a real-time visualization of what the stage
is being commanded to do.

Driver type is read from config.yaml:
  driver.type: mock   -> in-process MockDriver (works on macOS / any host)
  driver.type: can    -> real Dover DOF-5 over socketcan (Linux only)

When detection is invalid (cornea apex clipped off-screen), the controller
is bypassed and the stage is HELD at its last commanded position.

Layout
======
  Left panel  : OCT image w/ surface trace, top/bot/median markers, focus
                line, INVALID banner.
  Right panel : Stage gauge (vertical bar showing actual position vs soft
                limits) + time-history line plot of target vs actual mm.

Controls
========
  Space  - pause/resume
  q      - quit
  r      - restart
"""
import csv
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from cornea_focus import config as cf_config            # noqa: E402
from cornea_focus.surface import detect                 # noqa: E402
from cornea_focus.control import Controller             # noqa: E402
from cornea_focus.dof_driver import make_driver         # noqa: E402


FPS = 30
SIM_DIR = ROOT / "data" / "sim"
CONFIG_PATH = ROOT / "config.yaml"
HISTORY_S = 6.0   # seconds of target/actual history to plot


def load_manifest(path: Path) -> dict[int, dict]:
    rows: dict[int, dict] = {}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            i = int(row["frame_idx"])
            rows[i] = {
                "shift_px": int(row["shift_px"]),
                "position_mm": float(row["position_mm"]),
            }
    return rows


def normalize_to_uint8(frame: np.ndarray) -> np.ndarray:
    fmin, fmax = float(frame.min()), float(frame.max())
    if fmax - fmin < 1e-9:
        return np.zeros_like(frame, dtype=np.uint8)
    return ((frame - fmin) / (fmax - fmin) * 255.0).astype(np.uint8)


def main() -> None:
    cfg = cf_config.load(str(CONFIG_PATH))
    det_cfg = cfg.detector
    dz_mm_per_row = cfg.units.dz_mm_per_row
    focus_row = cfg.control.focus_line_row

    frame_paths = sorted(SIM_DIR.glob("frame_*.npy"))
    if not frame_paths:
        raise SystemExit(f"No frames in {SIM_DIR}. Run cornea_focus/generate_sim.py first.")
    manifest = load_manifest(SIM_DIR / "manifest.csv")
    n_frames = len(frame_paths)

    # --- driver + controller --------------------------------------------------
    driver = make_driver(cfg.driver)
    print(f"[demo] driver={driver.__class__.__name__}; homing...")
    driver.home()
    controller = Controller(cfg.control, cfg.units)

    # Soft-limit display range. MockDriver=±3mm, CanDriver=±1.2mm.
    stage_min = float(getattr(driver, "_min_mm", -3.0))
    stage_max = float(getattr(driver, "_max_mm", +3.0))

    first_raw = np.load(frame_paths[0])
    h, w = first_raw.shape
    DX_MM_PER_COL = 10.04 / w
    extent_mm = [0.0, w * DX_MM_PER_COL, h * dz_mm_per_row, 0.0]

    # --- figure layout --------------------------------------------------------
    # Image panel dominates; gauge is narrow; history is medium width.
    fig = plt.figure(figsize=(17, 7))
    gs = fig.add_gridspec(1, 3, width_ratios=[10, 1.2, 5], wspace=0.35)
    ax_img = fig.add_subplot(gs[0, 0])
    ax_gauge = fig.add_subplot(gs[0, 1])
    ax_hist = fig.add_subplot(gs[0, 2])
    fig.canvas.manager.set_window_title("Cornea Focus Demo — Sim + DOF")

    # --- left: OCT image + overlays ------------------------------------------
    # aspect="auto" lets the image fill its panel. Axes are still labelled in
    # real mm via extent_mm, but pixels are stretched to fit the panel so the
    # image isn't cramped. (The tracked SURFACE points are still placed in
    # mm coordinates so they line up with the cornea correctly.)
    img_artist = ax_img.imshow(
        normalize_to_uint8(first_raw),
        cmap="gray", vmin=0, vmax=255,
        extent=extent_mm, aspect="auto", interpolation="nearest",
    )
    x_mm = (np.arange(w) + 0.5) * DX_MM_PER_COL
    init_res = detect(first_raw, det_cfg)
    (surface_line,) = ax_img.plot(
        x_mm, init_res.surface_y * dz_mm_per_row,
        color="red", linewidth=1.2, label="surface",
    )
    top_line = ax_img.axhline(init_res.top_y * dz_mm_per_row,
                              color="cyan", linewidth=0.8, label="top_y")
    bot_line = ax_img.axhline(init_res.bottom_y * dz_mm_per_row,
                              color="magenta", linewidth=0.8, label="bottom_y")
    median_line = ax_img.axhline(init_res.median_y * dz_mm_per_row,
                                 color="lime", linewidth=1.2,
                                 label="median_y (tracked)")
    ax_img.axhline(focus_row * dz_mm_per_row,
                   color="white", linewidth=1.0, linestyle="--", label="focus row")
    ax_img.set_xlabel("Lateral (mm)")
    ax_img.set_ylabel("Axial depth (mm)")
    ax_img.legend(loc="upper right", fontsize=7, framealpha=0.6)
    ax_img.set_title("OCT view + surface detection", fontsize=10)
    # All live numbers go here, OUT of the title so they can never collide
    # with adjacent panels.
    hud_text = ax_img.text(
        0.005, 0.995, "", transform=ax_img.transAxes,
        fontsize=9, family="monospace",
        ha="left", va="top",
        bbox=dict(facecolor="black", alpha=0.7, edgecolor="none", pad=4),
        color="white",
    )
    invalid_banner = ax_img.text(
        0.5, 0.5,
        "⚠  CORNEA OUT OF FRAME\nSTAGE FROZEN! HOLDING LAST POSITION",
        transform=ax_img.transAxes,
        ha="center", va="center", fontsize=16, color="white", weight="bold",
        bbox=dict(facecolor="red", alpha=0.85, edgecolor="white", pad=10),
        visible=False,
    )

    # --- middle: stage position gauge ----------------------------------------
    # Thermometer style: vertical track spanning [stage_min, stage_max], with
    # red caution bands at the extremes, a horizontal pointer for ACTUAL
    # position, and a dashed marker for TARGET. No bar that can overflow.
    GAUGE_HALF = 0.5
    ax_gauge.set_xlim(-GAUGE_HALF, GAUGE_HALF)
    ax_gauge.set_ylim(stage_min, stage_max)
    ax_gauge.set_xticks([])
    ax_gauge.set_ylabel("Stage position (mm)")
    ax_gauge.set_title("DOF stage", fontsize=10)
    ax_gauge.axhline(0, color="grey", linewidth=0.5, linestyle=":")
    # Soft-limit caution bands (top and bottom 5% of travel)
    span = stage_max - stage_min
    ax_gauge.axhspan(stage_min, stage_min + 0.05 * span, color="red", alpha=0.15)
    ax_gauge.axhspan(stage_max - 0.05 * span, stage_max, color="red", alpha=0.15)
    # Filled "ribbon" from 0 to current actual position. Use a single
    # Rectangle patch so we can just resize it each frame instead of
    # tearing down and rebuilding a fill_between polygon (which kills blit).
    actual_fill = Rectangle(
        (-GAUGE_HALF * 0.5, 0.0), GAUGE_HALF, 0.0,
        facecolor="lime", alpha=0.55, edgecolor="none", animated=True,
    )
    ax_gauge.add_patch(actual_fill)
    # ACTUAL pointer (horizontal line)
    actual_marker = ax_gauge.axhline(0.0, color="lime", linewidth=2.5, label="actual")
    # TARGET pointer (dashed)
    target_marker = ax_gauge.axhline(0.0, color="orange", linewidth=1.5,
                                     linestyle="--", label="target")
    ax_gauge.legend(loc="upper right", fontsize=7)

    # --- right: time history -------------------------------------------------
    history_n = int(HISTORY_S * FPS)
    t_hist = deque(maxlen=history_n)
    target_hist = deque(maxlen=history_n)
    actual_hist = deque(maxlen=history_n)
    error_hist = deque(maxlen=history_n)

    (target_line_h,) = ax_hist.plot([], [], color="orange", linewidth=1.5, label="target")
    (actual_line_h,) = ax_hist.plot([], [], color="lime", linewidth=1.5, label="actual")
    (error_line_h,) = ax_hist.plot([], [], color="red", linewidth=1.0, alpha=0.6, label="error (focus)")
    ax_hist.set_xlabel("Time (s)")
    ax_hist.set_ylabel("Position (mm)")
    ax_hist.set_title("Stage history vs commanded", fontsize=10)
    ax_hist.axhline(0, color="grey", linewidth=0.5, linestyle=":")
    ax_hist.legend(loc="upper right", fontsize=7)
    ax_hist.set_ylim(stage_min * 0.6, stage_max * 0.6)
    ax_hist.set_xlim(-HISTORY_S, 0)

    # --- run state -----------------------------------------------------------
    state = {
        "paused": False,
        "idx": 0,
        "t0": time.monotonic(),
        "last_target_mm": 0.0,
    }

    # Mark every dynamic artist as animated so blit only repaints these.
    for art in (img_artist, surface_line, top_line, bot_line, median_line,
                actual_marker, target_marker,
                target_line_h, actual_line_h, error_line_h, hud_text,
                invalid_banner):
        art.set_animated(True)
    DYNAMIC_ARTISTS = (
        img_artist, surface_line, top_line, bot_line, median_line,
        actual_fill, actual_marker, target_marker,
        target_line_h, actual_line_h, error_line_h, hud_text, invalid_banner,
    )

    def update(_):
        if state["paused"]:
            return DYNAMIC_ARTISTS

        i = state["idx"]
        raw = np.load(frame_paths[i])
        img_artist.set_data(normalize_to_uint8(raw))

        res = detect(raw, det_cfg)
        surface_line.set_ydata(res.surface_y * dz_mm_per_row)
        top_line.set_ydata([res.top_y * dz_mm_per_row])
        bot_line.set_ydata([res.bottom_y * dz_mm_per_row])

        # --- decide what to command -----------------------------------------
        # Direct absolute mapping: cornea offset (mm) -> stage offset (mm).
        # The Controller's relative integrator is the right thing in a real
        # closed loop where moving the stage moves the cornea in the image,
        # but the SIMULATION has no such feedback, so the integrator would
        # saturate at +/- soft limit. An absolute map keeps the stage
        # oscillating exactly with the cornea, which is what we want to
        # visualize.
        error_mm_raw = (res.median_y - focus_row) * dz_mm_per_row
        if res.valid:
            target_mm_raw = -error_mm_raw  # move stage opposite the offset
            target_mm = float(np.clip(target_mm_raw, stage_min, stage_max))
            error_mm = error_mm_raw
            try:
                driver.move_absolute(target_mm)
            except Exception as e:
                print(f"[demo] driver error: {e}")
            state["last_target_mm"] = target_mm
            invalid_banner.set_visible(False)
            median_line.set_color("lime")
            median_line.set_ydata([res.median_y * dz_mm_per_row])
            track_label = "TRACK "
        else:
            # FREEZE: do not call move_absolute. Hold last commanded target.
            target_mm = state["last_target_mm"]
            error_mm = 0.0
            invalid_banner.set_visible(True)
            median_line.set_color("orange")
            track_label = "FROZEN"

        actual_mm = driver.get_position()
        # Hard-clamp the displayed values to the gauge's visible range so a
        # runaway never draws outside the panel.
        actual_disp = float(np.clip(actual_mm, stage_min, stage_max))
        target_disp = float(np.clip(target_mm, stage_min, stage_max))

        # --- gauge ----------------------------------------------------------
        # Resize the ribbon rectangle in-place (cheap; preserves blit).
        actual_fill.set_height(actual_disp)
        actual_fill.set_facecolor("lime" if res.valid else "orange")

        actual_marker.set_ydata([actual_disp])
        actual_marker.set_color("lime" if res.valid else "orange")

        target_marker.set_ydata([target_disp])

        # --- history --------------------------------------------------------
        t_now = time.monotonic() - state["t0"]
        t_hist.append(t_now)
        target_hist.append(target_mm)
        actual_hist.append(actual_mm)
        error_hist.append(error_mm)
        # plot relative to "now=0" so the trace scrolls
        rel_t = np.array(t_hist) - t_now
        target_line_h.set_data(rel_t, list(target_hist))
        actual_line_h.set_data(rel_t, list(actual_hist))
        error_line_h.set_data(rel_t, list(error_hist))

        # --- HUD ------------------------------------------------------------
        meta = manifest.get(i, {})
        shift = meta.get("shift_px", 0)
        sim_pos = meta.get("position_mm", 0.0)
        hud_text.set_text(
            f"[{track_label}]  Frame {i:04d}/{n_frames - 1}   "
            f"shift={shift:+d}px   sim={sim_pos:+.3f}mm   "
            f"err={error_mm:+.3f}mm   "
            f"target={target_mm:+.3f}mm   actual={actual_mm:+.3f}mm"
        )

        state["idx"] = (i + 1) % n_frames
        return DYNAMIC_ARTISTS

    def on_key(event):
        if event.key == " ":
            state["paused"] = not state["paused"]
        elif event.key == "q":
            plt.close(fig)
        elif event.key == "r":
            state["idx"] = 0

    fig.canvas.mpl_connect("key_press_event", on_key)

    ani = animation.FuncAnimation(
        fig, update, interval=1000.0 / FPS, blit=True, cache_frame_data=False,
    )
    fig._ani = ani  # type: ignore[attr-defined]
    # Use constrained_layout-style spacing without the wonky tight_layout call.
    fig.subplots_adjust(left=0.06, right=0.97, top=0.93, bottom=0.10, wspace=0.45)
    try:
        plt.show()
    finally:
        try:
            driver.stop()
        except Exception:
            pass
        try:
            driver.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
