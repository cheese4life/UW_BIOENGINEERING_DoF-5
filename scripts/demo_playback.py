#!/usr/bin/env python3
"""Demo playback with scrubbing — load pre-generated patient-sim frames
and display them with a timeline slider for seeking.

Usage:
  python3 scripts/demo_playback.py data/patient_sim/20260714_120000/
  python3 scripts/demo_playback.py data/patient_sim/20260714_120000/ --fps 30
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button

# Fall back to WebAgg on headless hosts
import os as _os
_forced = _os.environ.get("DOF_BACKEND") or _os.environ.get("MPLBACKEND")
if _forced:
    matplotlib.use(_forced, force=True)
elif not _os.environ.get("DISPLAY") and sys.platform.startswith("linux"):
    matplotlib.use("WebAgg", force=True)
    matplotlib.rcParams["webagg.open_in_browser"] = False
    matplotlib.rcParams["webagg.address"] = "0.0.0.0"
    matplotlib.rcParams["webagg.port"] = int(_os.environ.get("DOF_WEBAGG_PORT", 8989))

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))


def load_manifest(manifest_path: Path) -> list[dict]:
    with open(manifest_path, newline="") as f:
        return list(csv.DictReader(f))


def normalize_uint8(frame: np.ndarray) -> np.ndarray:
    fmin, fmax = float(frame.min()), float(frame.max())
    if fmax - fmin < 1e-9:
        return np.zeros_like(frame, dtype=np.uint8)
    return ((frame - fmin) / (fmax - fmin) * 255).astype(np.uint8)


def main() -> None:
    parser = argparse.ArgumentParser(description="Demo playback of patient-sim frames")
    parser.add_argument("sim_dir", help="Path to patient_sim output directory")
    parser.add_argument("--fps", type=float, default=30.0,
                        help="Playback FPS (default: 30)")
    parser.add_argument("--stride", type=int, default=0,
                        help="Frame stride (0 = auto from master FPS / playback FPS)")
    parser.add_argument("--no-detection", action="store_true",
                        help="Skip surface detection (faster)")
    args = parser.parse_args()

    sim_dir = Path(args.sim_dir)
    if not sim_dir.is_dir():
        raise SystemExit(f"Directory not found: {sim_dir}")

    # --- load config & manifest ---
    config_path = sim_dir / "config.json"
    if config_path.exists():
        with open(config_path) as f:
            config = json.load(f)
        master_fps = config.get("master_fps", 400)
        dz = config.get("dz_mm_per_row", 0.004593)
        print(f"Config:  profile={config.get('profile')}  sample={config.get('sample')}  "
              f"duration={config.get('duration_s')}s  master_fps={master_fps}")
    else:
        master_fps = 400
        dz = 0.004593
        print("No config.json found, using defaults.")

    manifest = load_manifest(sim_dir / "manifest.csv")
    n_frames = len(manifest)
    all_shifts = np.array([float(r["shift_px"]) for r in manifest], dtype=np.float32)

    frame_files = sorted(sim_dir.glob("frame_*.npy"))
    if len(frame_files) != n_frames:
        print(f"Warning: manifest has {n_frames} rows but found {len(frame_files)} .npy files")

    duration_s = n_frames / master_fps

    stride = args.stride if args.stride > 0 else max(1, int(round(master_fps / args.fps)))
    effective_fps = master_fps / stride
    playback_indices = list(range(0, n_frames, stride))

    print(f"Playback: {args.fps:.0f} fps target  →  stride={stride}  "
          f"effective={effective_fps:.1f}fps  →  {len(playback_indices)} frames shown")

    # --- detection (optional) ---
    try:
        from cornea_focus.surface import detect
        from cornea_focus.config import DetectorConfig
        det_cfg = DetectorConfig(mask_top_rows=10, blur_sigma=3,
                                 peak_prominence=10, smoothing_window=11)
        has_detection = not args.no_detection
    except Exception:
        has_detection = False
        print("Surface detection not available (cornea_focus module missing?)")

    # --- load first frame ---
    first = np.load(str(frame_files[0]))
    H, W = first.shape
    focus_row = 150

    # --- figure ---
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 2, height_ratios=[20, 1], width_ratios=[3, 1],
                          hspace=0.35, wspace=0.3)
    ax_img = fig.add_subplot(gs[0, 0])
    ax_shift = fig.add_subplot(gs[0, 1])
    ax_slider = fig.add_subplot(gs[1, :])
    # button panels — positioned manually below slider
    ax_btn_back = fig.add_axes([0.25, 0.005, 0.10, 0.04])
    ax_btn_play = fig.add_axes([0.38, 0.005, 0.10, 0.04])
    ax_btn_fwd  = fig.add_axes([0.51, 0.005, 0.10, 0.04])
    try:
        fig.canvas.manager.set_window_title("Patient Sim — Demo Playback")
    except Exception:
        pass

    extent_mm = [0, W * 10.04 / W, H * dz, 0]
    img = ax_img.imshow(normalize_uint8(first), cmap="gray", aspect="auto",
                        extent=extent_mm, vmin=0, vmax=255, interpolation="nearest")
    focus_line = ax_img.axhline(focus_row * dz, color="white", linestyle="--", linewidth=1)
    if has_detection:
        res = detect(first, det_cfg)
        (surface_line,) = ax_img.plot([], [], color="lime", linewidth=1.2, label="surface")
        median_line = ax_img.axhline(0, color="lime", linewidth=1.2, label="median")
        ax_img.legend(loc="upper right", fontsize=7, framealpha=0.6)
    ax_img.set_title("OCT frame + surface detection")
    ax_img.set_xlabel("Lateral (mm)")
    ax_img.set_ylabel("Axial depth (mm)")

    # --- shift history plot ---
    all_times = np.array([float(r["time_s"]) for r in manifest])
    (shift_trace,) = ax_shift.plot(all_times, all_shifts, color="cyan", linewidth=1.0, alpha=0.5)
    (cursor_dot,) = ax_shift.plot([0], [0], "o", color="yellow", markersize=8, zorder=5)
    ax_shift.set_ylim(-55, 55)
    ax_shift.axhline(0, color="grey", linewidth=0.5, linestyle=":")
    ax_shift.axhline(50, color="red", linewidth=0.5, linestyle="--", alpha=0.4)
    ax_shift.axhline(-50, color="red", linewidth=0.5, linestyle="--", alpha=0.4)
    ax_shift.set_title("Shift vs time (drag slider or click plot)")
    ax_shift.set_xlabel("Time (s)")
    ax_shift.set_ylabel("Shift (px)")

    hud = fig.text(0.01, 0.01, "", fontsize=10, family="monospace",
                   ha="left", va="bottom", color="white",
                   bbox=dict(facecolor="black", alpha=0.75, edgecolor="none", pad=6))

    # --- scrubber slider ---
    slider = Slider(
        ax=ax_slider, label="", valmin=0, valmax=duration_s,
        valinit=0, valfmt="%d frames", valstep=1.0 / master_fps,
    )
    # Re-style the slider label to show time
    ax_slider.set_xlabel("Drag to scrub  |  ← → arrow keys frame-step  |  Space = play/pause  |  R = restart",
                         fontsize=9)

    # --- playback state ---
    state = {
        "paused": True,
        "master_idx": 0,
        "t_playback_start": time.monotonic(),
        "t_at_pause": 0.0,    # simulation time when paused
    }

    def load_and_show(master_idx: int):
        """Load frame at master_idx and update all artists."""
        idx = int(np.clip(master_idx, 0, n_frames - 1))
        frame = np.load(str(sim_dir / f"frame_{idx:06d}.npy"))
        img.set_data(normalize_uint8(frame))

        shift_px = all_shifts[idx]
        t_sim = idx / master_fps
        slider.eventson = False
        slider.set_val(t_sim)
        slider.eventson = True
        cursor_dot.set_data([t_sim], [shift_px])
        # update play button label
        btn_play.label.set_text("⏸" if not state["paused"] else "▶")

        err = 0.0
        status = ""
        if has_detection:
            try:
                res = detect(frame, det_cfg)
                x_vals = (np.arange(W) + 0.5) * (10.04 / W)
                surface_line.set_data(x_vals, res.surface_y * dz)
                median_line.set_ydata([res.median_y * dz])
                err = (res.median_y - focus_row) * dz * 1000
                status = "VALID" if res.valid else "INVALID"
                valid_color = "lime" if res.valid else "red"
            except Exception:
                status = "detect err"
                valid_color = "orange"
        else:
            status = "off"
            valid_color = "grey"

        hud.set_text(
            f"Master {idx:06d}/{n_frames-1}  |  "
            f"t={t_sim:.3f}s  |  "
            f"shift={shift_px:+.2f}px ({shift_px*dz*1000:+.1f}µm)  |  "
            f"err={err:.1f}µm [{status}]"
        )
        hud.set_color(valid_color if has_detection else "white")
        fig.canvas.draw_idle()
        return idx

    def on_slider(val):
        """Called when user drags the slider."""
        t_sim = float(val)
        idx = int(round(t_sim * master_fps))
        state["paused"] = True
        state["master_idx"] = idx
        state["t_at_pause"] = t_sim
        load_and_show(idx)

    slider.on_changed(on_slider)

    # --- button widgets ---
    def _step_frame(delta: int):
        """Step forward or backward by one master frame."""
        state["paused"] = True
        new_val = max(0.0, min(duration_s, slider.val + delta / master_fps))
        slider.set_val(new_val)

    def _toggle_play(_event):
        state["paused"] = not state["paused"]
        if not state["paused"]:
            state["t_at_pause"] = state["master_idx"] / master_fps
        state["t_playback_start"] = time.monotonic()
        load_and_show(state["master_idx"])

    btn_back = Button(ax_btn_back, "◀◀", color="#2a2a2a", hovercolor="#4a4a4a")
    btn_back.label.set_color("white")
    btn_back.label.set_fontsize(14)
    btn_back.on_clicked(lambda _e: _step_frame(-1))

    btn_play = Button(ax_btn_play, "▶", color="#2a2a2a", hovercolor="#4a4a4a")
    btn_play.label.set_color("lime")
    btn_play.label.set_fontsize(16)
    btn_play.on_clicked(_toggle_play)

    btn_fwd = Button(ax_btn_fwd, "▶▶", color="#2a2a2a", hovercolor="#4a4a4a")
    btn_fwd.label.set_color("white")
    btn_fwd.label.set_fontsize(14)
    btn_fwd.on_clicked(lambda _e: _step_frame(1))

    def on_click_shift(event):
        """Click on the shift plot to jump to that time."""
        if event.inaxes != ax_shift:
            return
        t_click = event.xdata
        if t_click is None:
            return
        t_sim = float(np.clip(t_click, 0, duration_s))
        state["paused"] = True
        slider.set_val(t_sim)  # triggers on_slider → load_and_show

    fig.canvas.mpl_connect("button_press_event", on_click_shift)

    def on_key(event):
        key = event.key.lower() if event.key else ""
        if key in (" ", "space"):
            _toggle_play(None)
        elif key in ("q", "escape"):
            plt.close(fig)
        elif key == "r":
            slider.set_val(0.0)
            state["paused"] = True
            state["t_at_pause"] = 0.0
            state["t_playback_start"] = time.monotonic()
        elif key in ("right", "arrowright", "n"):
            _step_frame(1)
        elif key in ("left", "arrowleft", "p"):
            _step_frame(-1)

    fig.canvas.mpl_connect("key_press_event", on_key)

    # --- FuncAnimation-based playback loop (works with all backends) ---
    interval_ms = 1000.0 / args.fps

    def anim_update(_frame_num):
        if state["paused"]:
            return ()
        elapsed_real = time.monotonic() - state["t_playback_start"]
        t_sim = state["t_at_pause"] + elapsed_real
        if t_sim > duration_s:
            state["paused"] = True
            return ()
        state["master_idx"] = int(round(t_sim * master_fps))
        load_and_show(state["master_idx"])
        return ()

    ani = animation.FuncAnimation(fig, anim_update, interval=interval_ms,
                                  blit=False, cache_frame_data=False)
    fig._ani = ani  # prevent garbage collection

    # --- show initial frame ---
    load_and_show(0)
    fig.subplots_adjust(left=0.05, right=0.97, top=0.95, bottom=0.08, wspace=0.3)

    plt.show()


if __name__ == "__main__":
    main()
