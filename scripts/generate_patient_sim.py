#!/usr/bin/env python3
"""Generate patient-simulation frames from a MotionModel trajectory.

Reuses the proven warpAffine + scrolled-background technique from
cornea_focus/generate_sim.py, but feeds it shifts from the MotionModel
(OU drift + tremor + microsaccades + physio rhythms).

Usage:
  python3 scripts/generate_patient_sim.py --profile calm --duration 30
  python3 scripts/generate_patient_sim.py --profile anxious --sample cornea_2 --duration 10 --seed 123
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from live_simulation.motion_model import MotionModel  # noqa: E402

DZ_MM_PER_ROW = 0.004593
MASTER_FPS = 400
MAX_SHIFT = 50
DC_ROWS = 4
SAMPLE_BAND = (5, 25)
STRIP = 20


def _load_and_clean_sample(path: Path) -> np.ndarray:
    """Load a cornea .npy, apply DC artifact fix, return float32 reference."""
    ref = np.load(path).astype(np.float32)
    clean = ref[SAMPLE_BAND[0]:SAMPLE_BAND[1]]
    cm, cs = float(clean.mean()), float(clean.std())
    rng = np.random.default_rng(1)
    ref[:DC_ROWS] = np.clip(
        rng.normal(cm, cs, size=(DC_ROWS, ref.shape[1])), 0.0, None
    ).astype(ref.dtype)
    return ref


def _build_background(ref: np.ndarray) -> np.ndarray:
    """Build an oversized noise canvas (H + 2*MAX_SHIFT rows)."""
    H, W = ref.shape
    top_strip = ref[:STRIP]
    bot_strip = ref[-STRIP:]
    top_mean, top_std = float(top_strip.mean()), float(top_strip.std())
    bot_mean, bot_std = float(bot_strip.mean()), float(bot_strip.std())
    rng = np.random.default_rng(0)
    total_h = H + 2 * MAX_SHIFT
    blend = np.linspace(0.0, 1.0, total_h)[:, None]
    mean_profile = (1.0 - blend) * top_mean + blend * bot_mean
    std_profile = (1.0 - blend) * top_std + blend * bot_std
    bg = rng.normal(0.0, 1.0, size=(total_h, W)) * std_profile + mean_profile
    return np.clip(bg, 0.0, None).astype(ref.dtype)


def _warp_frame(ref: np.ndarray, big_bg: np.ndarray, shift_px: float) -> np.ndarray:
    """Apply warpAffine + scrolled background fill for a float shift."""
    H, W = ref.shape
    M = np.float32([[1, 0, 0], [0, 1, shift_px]])
    shifted = cv2.warpAffine(
        ref, M, (W, H),
        borderMode=cv2.BORDER_CONSTANT, borderValue=0,
        flags=cv2.INTER_CUBIC,
    )
    exposed = int(np.ceil(abs(shift_px)))
    bg_top = MAX_SHIFT - int(round(shift_px))
    bg = big_bg[bg_top:bg_top + H]
    if shift_px > 0:
        shifted[:exposed] = bg[:exposed]
    elif shift_px < 0:
        shifted[H - exposed:] = bg[H - exposed:]
    return shifted


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate patient-simulation frames")
    parser.add_argument("--profile", default="calm", choices=["calm", "anxious"],
                        help="Patient motion profile")
    parser.add_argument("--sample", default="cornea_1",
                        help="Cornea sample name (cornea_1..4) or path to .npy")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Duration in seconds")
    parser.add_argument("--seed", type=int, default=42,
                        help="RNG seed for reproducibility")
    parser.add_argument("--output", default=None,
                        help="Output directory (default: data/patient_sim/<timestamp>/)")
    args = parser.parse_args()

    # --- resolve sample path ---
    sample_path: Path
    if args.sample.endswith(".npy"):
        sample_path = Path(args.sample)
    else:
        sample_path = ROOT / "data" / "samples" / f"{args.sample}.npy"
    if not sample_path.exists():
        raise SystemExit(f"Sample not found: {sample_path}")

    # --- output directory ---
    if args.output:
        out_dir = Path(args.output)
    else:
        ts = time.strftime("%Y%m%d_%H%M%S")
        out_dir = ROOT / "data" / "patient_sim" / ts
    out_dir.mkdir(parents=True, exist_ok=True)

    n_frames = int(args.duration * MASTER_FPS)

    print(f"Profile:  {args.profile}")
    print(f"Sample:   {sample_path}")
    print(f"Duration: {args.duration}s  →  {n_frames} frames @ {MASTER_FPS}fps")
    print(f"Output:   {out_dir}")

    # --- load & prep ---
    ref = _load_and_clean_sample(sample_path)
    H, W = ref.shape
    big_bg = _build_background(ref)

    # --- motion model ---
    model = MotionModel(args.profile, seed=args.seed)

    # --- config dump ---
    config = {
        "profile": args.profile,
        "sample": args.sample,
        "duration_s": args.duration,
        "master_fps": MASTER_FPS,
        "n_frames": n_frames,
        "seed": args.seed,
        "max_shift_px": MAX_SHIFT,
        "dz_mm_per_row": DZ_MM_PER_ROW,
        "frame_shape": list(ref.shape),
        "params": {f: getattr(model.params, f) for f in model.params.__dataclass_fields__},
    }
    with open(out_dir / "config.json", "w") as f:
        json.dump(config, f, indent=2)

    # --- generate ---
    print("Generating frames...", end="", flush=True)
    t_start = time.perf_counter()

    manifest_rows = []
    for i in range(n_frames):
        t_sec = i / MASTER_FPS
        shift = model.shift_at(t_sec)
        frame = _warp_frame(ref, big_bg, shift)
        np.save(out_dir / f"frame_{i:06d}.npy", frame)
        manifest_rows.append({
            "frame_idx": i,
            "time_s": round(t_sec, 6),
            "shift_px": round(shift, 4),
            "position_mm": round(shift * DZ_MM_PER_ROW, 6),
        })

        if i > 0 and i % 500 == 0:
            pct = 100.0 * i / n_frames
            elapsed = time.perf_counter() - t_start
            eta = elapsed / i * n_frames - elapsed
            print(f"\rGenerating frames... {pct:.0f}%  ({i}/{n_frames})  ETA {eta:.0f}s", end="", flush=True)

    elapsed = time.perf_counter() - t_start
    print(f"\rGenerated {n_frames} frames in {elapsed:.1f}s  ({n_frames/elapsed:.0f} fps)")

    # --- manifest.csv ---
    manifest_path = out_dir / "manifest.csv"
    with open(manifest_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["frame_idx", "time_s", "shift_px", "position_mm"])
        writer.writeheader()
        writer.writerows(manifest_rows)

    # --- summary ---
    all_shifts = np.array([r["shift_px"] for r in manifest_rows], dtype=np.float32)
    summary = {
        "n_frames": n_frames,
        "shift_mean": round(float(all_shifts.mean()), 4),
        "shift_std": round(float(all_shifts.std()), 4),
        "shift_min": round(float(all_shifts.min()), 4),
        "shift_max": round(float(all_shifts.max()), 4),
        "position_range_mm": f"{round(float(all_shifts.min()) * DZ_MM_PER_ROW, 4)} .. "
                              f"{round(float(all_shifts.max()) * DZ_MM_PER_ROW, 4)}",
        "disk_mb": round(sum(
            f.stat().st_size for f in out_dir.glob("frame_*.npy")
        ) / 1e6, 1),
    }
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\nSummary:\n  {json.dumps(summary, indent=2)}")
    print(f"\nDone → {out_dir}")


if __name__ == "__main__":
    main()
