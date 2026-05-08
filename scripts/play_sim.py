"""Playback simulated cornea frames as a 30 FPS video.

Renders frames from data/sim/ with proper physical aspect ratio (mm axes),
a frame counter, and a position readout from manifest.csv.

Controls:
  Space  - pause/resume
  q      - quit
  r      - restart from frame 0
"""
import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# physical calibration (must match config.yaml)
DZ_MM_PER_ROW = 0.004593       # axial, vertical
DX_MM_PER_COL = 10.04 / 256    # lateral, horizontal (from cornea_1 annotated metadata)

FPS = 30
SIM_DIR = Path(__file__).resolve().parent.parent / "data" / "sim"


def load_manifest(path: Path) -> dict[int, dict]:
    """frame_idx -> {shift_px, position_mm}."""
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
    """Rescale arbitrary float range to 0-255 uint8 for display."""
    fmin, fmax = float(frame.min()), float(frame.max())
    if fmax - fmin < 1e-9:
        return np.zeros_like(frame, dtype=np.uint8)
    scaled = (frame - fmin) / (fmax - fmin) * 255.0
    return scaled.astype(np.uint8)


def main() -> None:
    frame_paths = sorted(SIM_DIR.glob("frame_*.npy"))
    if not frame_paths:
        raise SystemExit(f"No frames found in {SIM_DIR}. Run generate_sim.py first.")

    manifest_path = SIM_DIR / "manifest.csv"
    manifest = load_manifest(manifest_path) if manifest_path.exists() else {}

    n_frames = len(frame_paths)
    first = normalize_to_uint8(np.load(frame_paths[0]))
    h, w = first.shape

    # extent in mm: [xmin, xmax, ymax, ymin] (ymin/ymax flipped so row 0 is at top)
    extent_mm = [0.0, w * DX_MM_PER_COL, h * DZ_MM_PER_ROW, 0.0]

    fig, ax = plt.subplots(figsize=(12, 4))
    fig.canvas.manager.set_window_title("Cornea Sim Playback")

    img_artist = ax.imshow(
        first,
        cmap="gray",
        vmin=0,
        vmax=255,
        extent=extent_mm,
        aspect="equal",   # equal mm/mm => physically accurate stretch
        interpolation="nearest",
    )
    ax.set_xlabel("Lateral position (mm)")
    ax.set_ylabel("Axial depth (mm)")

    title = ax.set_title("")

    state = {"paused": False, "idx": 0}

    def update(_):
        if state["paused"]:
            return (img_artist, title)

        i = state["idx"]
        frame = normalize_to_uint8(np.load(frame_paths[i]))
        img_artist.set_data(frame)

        meta = manifest.get(i, {})
        shift = meta.get("shift_px", 0)
        pos_mm = meta.get("position_mm", 0.0)
        title.set_text(
            f"Frame {i:04d} / {n_frames - 1}    "
            f"shift = {shift:+d} px    "
            f"sim position = {pos_mm:+.4f} mm"
        )

        state["idx"] = (i + 1) % n_frames
        return (img_artist, title)

    def on_key(event):
        if event.key == " ":
            state["paused"] = not state["paused"]
        elif event.key == "q":
            plt.close(fig)
        elif event.key == "r":
            state["idx"] = 0

    fig.canvas.mpl_connect("key_press_event", on_key)

    interval_ms = 1000.0 / FPS
    ani = animation.FuncAnimation(
        fig,
        update,
        interval=interval_ms,
        blit=False,
        cache_frame_data=False,
    )

    # keep a reference so the animation isn't garbage collected
    fig._ani = ani  # type: ignore[attr-defined]

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
