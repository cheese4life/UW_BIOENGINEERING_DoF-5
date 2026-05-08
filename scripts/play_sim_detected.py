"""Playback simulated cornea frames with live surface detection overlay.

Runs cornea_focus.surface.detect() on every frame and overlays:
  - the detected per-column surface trace (red line)
  - top_y / bottom_y horizontal markers (cyan / magenta)
  - center_y horizontal marker (yellow)
  - focus_line_row reference (white dashed)
  - HUD readout: frame, sim shift, center_y (px + mm), focus error (mm)

Controls:
  Space  - pause/resume
  q      - quit
  r      - restart
"""
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Allow `python scripts/play_sim_detected.py` from project root
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from cornea_focus import config as cf_config  # noqa: E402
from cornea_focus.surface import detect       # noqa: E402


FPS = 30
SIM_DIR = ROOT / "data" / "sim"
CONFIG_PATH = ROOT / "config.yaml"


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
        raise SystemExit(f"No frames found in {SIM_DIR}. Run generate_sim.py first.")

    manifest = load_manifest(SIM_DIR / "manifest.csv")
    n_frames = len(frame_paths)

    first_raw = np.load(frame_paths[0])
    h, w = first_raw.shape

    # Lateral calibration: cornea_1 annotated metadata gives ~10.04 mm full width.
    DX_MM_PER_COL = 10.04 / w
    extent_mm = [0.0, w * DX_MM_PER_COL, h * dz_mm_per_row, 0.0]

    fig, ax = plt.subplots(figsize=(12, 5))
    fig.canvas.manager.set_window_title("Cornea Sim + Surface Detection")

    img_artist = ax.imshow(
        normalize_to_uint8(first_raw),
        cmap="gray", vmin=0, vmax=255,
        extent=extent_mm, aspect="equal", interpolation="nearest",
    )

    # X coordinate for every column, in mm, used to plot surface trace
    x_mm = (np.arange(w) + 0.5) * DX_MM_PER_COL

    # Initial detection just to size artists
    init_res = detect(first_raw, det_cfg)

    (surface_line,) = ax.plot(
        x_mm, init_res.surface_y * dz_mm_per_row,
        color="red", linewidth=1.2, label="detected surface",
    )

    top_line = ax.axhline(init_res.top_y * dz_mm_per_row,
                          color="cyan", linewidth=0.8, linestyle="-", label="top_y")
    bot_line = ax.axhline(init_res.bottom_y * dz_mm_per_row,
                          color="magenta", linewidth=0.8, linestyle="-", label="bottom_y")
    median_line = ax.axhline(init_res.median_y * dz_mm_per_row,
                             color="lime", linewidth=1.2, linestyle="-",
                             label="median_y (tracked)")
    focus_line = ax.axhline(focus_row * dz_mm_per_row,
                            color="white", linewidth=1.0, linestyle="--", label="focus row")

    ax.set_xlabel("Lateral position (mm)")
    ax.set_ylabel("Axial depth (mm)")
    ax.legend(loc="upper right", fontsize=8, framealpha=0.6)

    title = ax.set_title("")
    # Big "INVALID" banner painted only when detection is untrustworthy
    invalid_banner = ax.text(
        0.5, 0.5,
        "⚠  CORNEA OUT OF FRAME\nSTAGE FROZEN! HOLDING LAST POSITION",
        transform=ax.transAxes,
        ha="center", va="center", fontsize=18, color="white", weight="bold",
        bbox=dict(facecolor="red", alpha=0.85, edgecolor="white", pad=12),
        visible=False,
    )

    # Last-known-good tracked center (mm). Used when current frame is invalid.
    # Trip immediately on the first bad frame -- when the cornea is clipping
    # we want to STOP commanding the stage right away, not wait.
    state = {
        "paused": False,
        "idx": 0,
        "last_good_mm": init_res.median_y * dz_mm_per_row,
    }

    def update(_):
        if state["paused"]:
            return (img_artist, title, surface_line, top_line, bot_line, median_line, invalid_banner)

        i = state["idx"]
        raw = np.load(frame_paths[i])
        img_artist.set_data(normalize_to_uint8(raw))

        res = detect(raw, det_cfg)
        surface_line.set_ydata(res.surface_y * dz_mm_per_row)
        top_line.set_ydata([res.top_y * dz_mm_per_row])
        bot_line.set_ydata([res.bottom_y * dz_mm_per_row])

        if res.valid:
            tracked_mm = res.median_y * dz_mm_per_row
            state["last_good_mm"] = tracked_mm
            invalid_banner.set_visible(False)
            # show median tracker normally
            median_line.set_color("lime")
            median_line.set_ydata([tracked_mm])
        else:
            # FREEZE: hold last good tracked position, do NOT update median
            # line to the (untrustworthy) live median. Recolor it orange to
            # indicate "held".
            tracked_mm = state["last_good_mm"]
            invalid_banner.set_visible(True)
            median_line.set_color("orange")
            median_line.set_ydata([tracked_mm])

        meta = manifest.get(i, {})
        shift = meta.get("shift_px", 0)
        sim_pos = meta.get("position_mm", 0.0)
        error_mm = tracked_mm - focus_row * dz_mm_per_row
        flag = "TRACK " if res.valid else "FROZEN"

        title.set_text(
            f"[{flag}] Frame {i:04d}/{n_frames - 1}   "
            f"shift={shift:+d}px   sim={sim_pos:+.3f}mm   "
            f"apex={res.apex_y:5.1f}px   tracked={tracked_mm:+.3f}mm   "
            f"err={error_mm:+.3f}mm   edge={res.edge_fraction*100:4.1f}%"
        )

        state["idx"] = (i + 1) % n_frames
        return (img_artist, title, surface_line, top_line, bot_line, median_line, invalid_banner)

    def on_key(event):
        if event.key == " ":
            state["paused"] = not state["paused"]
        elif event.key == "q":
            plt.close(fig)
        elif event.key == "r":
            state["idx"] = 0

    fig.canvas.mpl_connect("key_press_event", on_key)

    ani = animation.FuncAnimation(
        fig, update, interval=1000.0 / FPS, blit=False, cache_frame_data=False,
    )
    fig._ani = ani  # type: ignore[attr-defined]

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
