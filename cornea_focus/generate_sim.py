import numpy as np
import cv2
from pathlib import Path
import csv


ROOT = Path(__file__).parent.parent
Path(ROOT / "data" / "sim").mkdir(parents=True, exist_ok=True)
reference_image = np.load(ROOT / "data" / "samples" / "cornea_1.npy").astype(np.float32)

# Suppress the OCT DC artifact: the top few rows (around y=0) have an
# elevated mean that would otherwise be misdetected as a surface. Replace
# them with samples drawn from the clean background just below.
DC_ROWS = 4           # rows to overwrite at the top
SAMPLE_BAND = (5, 25) # rows used to estimate clean background statistics
_clean_band = reference_image[SAMPLE_BAND[0]:SAMPLE_BAND[1]]
_clean_mean = float(_clean_band.mean())
_clean_std = float(_clean_band.std())
_dc_rng = np.random.default_rng(1)
_dc_fill = _dc_rng.normal(_clean_mean, _clean_std, size=(DC_ROWS, reference_image.shape[1]))
reference_image[:DC_ROWS] = np.clip(_dc_fill, 0.0, None).astype(reference_image.dtype)

# sin wave behavior for image manipulation
n_frames = 300
t = np.linspace(0, 2 * np.pi, n_frames)
shifts = (50 * np.sin(t)).astype(int)

# Sample background statistics from the top and bottom strips of the
# reference image so synthetic fill rows match the surrounding noise floor.
H, W = reference_image.shape
STRIP = 20  # rows to sample for background stats
top_strip = reference_image[:STRIP]
bot_strip = reference_image[-STRIP:]
top_mean, top_std = float(top_strip.mean()), float(top_strip.std())
bot_mean, bot_std = float(bot_strip.mean()), float(bot_strip.std())
rng = np.random.default_rng(0)

# Build ONE oversized fixed background that extends past the frame on both
# sides by max_shift rows. Each frame slices a window of this background
# OFFSET BY THE SAME SHIFT as the cornea, so the fill texture moves with
# the cornea (no perceived sliding of static noise against moving tissue).
max_shift = int(np.abs(shifts).max())
TOTAL_H = H + 2 * max_shift
# vertical blend across the oversized canvas
blend = np.linspace(0.0, 1.0, TOTAL_H)[:, None]
mean_profile = (1.0 - blend) * top_mean + blend * bot_mean
std_profile = (1.0 - blend) * top_std + blend * bot_std
big_background = rng.normal(0.0, 1.0, size=(TOTAL_H, W)) * std_profile + mean_profile
big_background = np.clip(big_background, 0.0, None).astype(reference_image.dtype)


for i in range(n_frames):
    s = int(shifts[i])
    # Background window scrolls with the cornea: row 0 of frame == row
    # (max_shift - s) of the big background. So when cornea moves down (+s),
    # background also moves down by the same amount.
    bg_top = max_shift - s
    bg = big_background[bg_top : bg_top + H]

    M = np.float32([[1, 0, 0], [0, 1, s]])
    shifted = cv2.warpAffine(
        reference_image, M, (W, H), borderMode=cv2.BORDER_CONSTANT, borderValue=0
    )
    if s > 0:
        shifted[:s] = bg[:s]
    elif s < 0:
        shifted[H + s :] = bg[H + s :]
    np.save(ROOT / "data" / "sim" / f"frame_{i:04d}.npy", shifted)

manifest_path = ROOT / "data" / "sim" / "manifest.csv"
with open(manifest_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["frame_idx", "shift_px", "position_mm"])
    for i in range(n_frames):
        position_mm = shifts[i] * 0.004593
        writer.writerow([i, shifts[i], round(position_mm, 6)])

print(f"Generated {n_frames} frames -> {ROOT / 'data' / 'sim'}")
    
    
    
    

