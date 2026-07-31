# Abstract overview:
# CV (version 2) works by splitting pixels into a 2D data structure that
# represents every pixel as a scored float value from 0.0 to 1.0.  
#
# "0" being no surface detected, "1" being most probable surface detected.
#
# The score is determined by two main tests:
# 
# 1. does a given location have an aggressive jump from dark to bright pixels?
#       (And vice versa. This algo is optimal for bright and dark cornea scans)
# 2. does the given location maintain a valid trend of dark to bright pixel jumps?
#
#
# Remaining scores are then compared against factors like how deep the pixels are 
# relative to the scans size. 
#
# (The depths of OCT scans are unlikely to have a surface, priority reserved for top)
#
# Surface drawing also follows most continuous path direction, meaning surface detection
# will not suddenly drop off.


import numpy as np
import cv2
from scipy.ndimage import median_filter
from dataclasses import dataclass
from cornea_focus.config import DetectorConfig


# normalize pixels
# OCT samples are not uniform in terms of pixel brightness
# Some surfaces are brighter/have brighter mass than others
#
# This is a problem because all surfaces need to be detected even if
# brightness drops off. So we introduce a contrast floor, which highlights
# the minimum that a detected bright spot should look like.
#
# Thus, the pixel scoring setup works like this:
# 1. take the 75th percentile of all pixel scores
# 2. multiply by contrast floor (0.35)
#
#
# ex)
#
# compute the global scale: the 75th percentile of all column maxes. 
# If pixel scores are 9.5, 0.8, 0.2, 75th percentile = 0.8.
#
# Next, apply contrast floor
#
# Col A:  max = 9.5  --> divisor = 9.5   (9.5 > 0.28, use 9.5)
# Col B:  max = 0.8  --> divisor = 0.8   (0.8 > 0.28, use 0.8)
# Col C:  max = 0.2  --> divisor = 0.28  (0.2 < 0.28, USE THE FLOOR!)
#
# Basically, for every detected surface per column, the contrast floor
# applies a minimum score so that dim areas aren't excluded from the 
# overall surface detection
#
#
_CONTRAST_FLOOR = 0.35


# Depth penalty. So as pixels are split into columns and each pixel
# is checked, eventially a bright spot is detected. However, a column
# can have multiple bright spots that can appear as surface canidates.
# 
# BELOW_EDGE_GAIN says that for every bright spot detected after the first
# detection, mulitply it by 0.35 to keep the score at a reasonable level
# relative to it's depth.
#
# Rationale: Zhaoudong provided a great scan of eye, but retna blobs
# under the cornea confused old CV script. BELOW_EDGE_GAIN lowers the 
# score of these bright spots under the cornea, meaning they can be ignored.
# (This is also good for corneal scans with anomolies that might similarly
# trip the algorithm)
_BELOW_EDGE_GAIN = 0.35


# Optimization
# Checks to make sure rows contain pixels with scores of at least 0.25
# If not, then ignore those rows. 
# This speeds up the surface path selection by ignoring redundant rows.
# Rationale: a good path will only exist with scores above the threshold
# No need to spend resources iterating through rows with poor scores when 
# determining and drawing a selected path
_BAND_THRESHOLD = 0.25


@dataclass
class SurfaceResult:
    # array of 256 numbers (full surface profile)
    surface_y: np.ndarray
    
    # highest point of cornea (min row index)
    top_y: float
    
    # lowest point of cornea (max row index)
    bottom_y: float
    
    # midpoint between top and bottom (legacy)
    center_y: float

    # robust tracking point: median of the per-column surface trace.
    # Insensitive to a few clipped or noisy columns near the edges.
    median_y: float

    # apex (highest point of cornea) on the SMOOTHED trace
    apex_y: float

    # True if the detection looks trustworthy. False when too many columns
    # report a value at the top mask boundary or at the bottom of the image,
    # which means the cornea has clipped off-screen and top/bottom are unreliable.
    valid: bool

    # fraction of columns that landed against the top or bottom boundary.
    # Useful for diagnostics and for the overlay HUD.
    edge_fraction: float
    
    
    
    
# In raw form, each pixel is scored in a non uniform format.
# The best pixel in a bright center column might have a raw gradient of 200
# while the best pixel in a dim edge column might only reach 8.
#
# If both columns are left as is, the edge column gets ignored. The
# center column is 25x louder and wins every time, so the surface trace
# drops off at the edges.
#
# This function fixes that. It divides each column by its own best score
# so every column's best pixel becomes 1.0. Now dim edges compete equally
# with the bright center.
#
# But there is a trap. A noise column with no real tissue might have a
# best pixel of 0.02. Dividing by 0.02 amplifies that noise all the way to
# 1.0, which looks exactly like a real surface. The tracer would draw a
# path through empty air.
#
# That is what CONTRAST_FLOOR prevents. The function takes the 75th
# percentile of all column bests as the global scale, a number that
# represents what a typical real column looks like. If a column's best is
# below 35 percent of that global scale, the column is probably noise.
# Its divisor gets forced up to the floor value instead of its own tiny
# best, so the noise stays crushed near zero and cannot fake a surface.
def _normalize_columns(x: np.ndarray) -> np.ndarray:
    col_scale = x.max(axis=0)
    global_scale = float(np.percentile(col_scale, 75)) or 1.0
    np.maximum(col_scale, _CONTRAST_FLOOR * global_scale, out=col_scale)
    x /= (col_scale + 1e-9)
    np.clip(x, 0.0, 1.0, out=x)
    return x

# function checks to make sure that selected pixels stay bright after 
# being discovered
def _step_strength(blurred: np.ndarray, m: int) -> np.ndarray:
    h, w = blurred.shape
    step = np.zeros((h - 1, w), dtype=np.float32)
    if h < 2 * m + 3:
        return step

    # C[k] = sum of rows [0, k): any window sum is one subtraction
    C = np.zeros((h + 1, w), dtype=np.float32)
    np.cumsum(blurred, axis=0, out=C[1:])

    # boundary row r sits between image rows r and r+1; only rows with a full
    # window on both sides can be judged, the rest stay at 0
    r0, r1 = m, h - 1 - m
    mid = C[r0 + 1:r1 + 1]          # sum of rows [0, r]
    lo = C[r0 + 1 - m:r1 + 1 - m]   # sum of rows [0, r-m]
    hi = C[r0 + 1 + m:r1 + 1 + m]   # sum of rows [0, r+m]
    step[r0:r1] = (hi + lo - 2.0 * mid) / m
    return step

# This function is the scoring logic, scoring likelyhood of surface detection 
# as a float from 0 to 1.
# 
# Set up with two tests (as detailed in slides):
# Test A: is there a sharp dark to bright jump at this row?
# Test B: Do "m" rows below stay bright?
# Current setup assumes dark is air and light is tissue. 
# (Which can be adjusted as needs demand)
def _edge_response(blurred: np.ndarray, cfg: DetectorConfig) -> np.ndarray:
    # sharp term: locates the boundary to the exact row
    grad = np.diff(blurred, axis=0)
    np.clip(grad, 0.0, None, out=grad)

    # robust term: confirms there is real tissue underneath
    step = _step_strength(blurred, max(1, int(cfg.step_rows)))
    np.clip(step, 0.0, None, out=step)

    # geometric mean: a row must satisfy both to score highly
    resp = np.sqrt(_normalize_columns(grad) * _normalize_columns(step))

    # make sure not to start at top of scan
    resp[:cfg.mask_top_rows] = 0.0

    h = resp.shape[0]

    # attenuate everything below the first strong edge of each column
    strong = resp >= float(cfg.first_edge_ratio)
    below = np.empty_like(strong)
    below[0] = False
    np.logical_or.accumulate(strong[:-1], axis=0, out=below[1:])
    resp[below] *= _BELOW_EDGE_GAIN

    depth = (np.arange(h, dtype=np.float32) / max(h - 1, 1))[:, None]
    resp -= float(cfg.depth_bias) * depth

    return resp



# This function processes the 2D array scoring matrix. Right now, every position of a pixel
# is represented as a float score from 0 to 1. 
# This function is for path selection, selecting the highest probability for a smooth
# surface trace. Works by determinng the next high score in a column, and applying
# a depth penalty based on how many pixels away that pixel is
def _trace_surface(resp: np.ndarray, cfg: DetectorConfig) -> np.ndarray:
    h, w = resp.shape
    max_jump = max(1, int(cfg.max_col_jump))
    penalty = float(cfg.smooth_penalty)
    n_off = 2 * max_jump + 1

    # only search the rows that actually contain candidate edges (plus a
    # max_jump margin). On a typical scan that is a third of the image, and
    # the DP cost is linear in the number of rows searched. The band is data
    # driven, so it follows the cornea as it moves through the frame.
    hot = np.flatnonzero(resp.max(axis=1) >= _BAND_THRESHOLD)
    if hot.size:
        r0 = max(0, int(hot[0]) - max_jump)
        r1 = min(h, int(hot[-1]) + max_jump + 1)
    else:
        r0, r1 = 0, h
    band = resp[r0:r1]
    hb = band.shape[0]

    # cost of moving |d| rows between two columns, for d = +max_jump .. -max_jump
    move_cost = (penalty * np.abs(np.arange(max_jump, -max_jump - 1, -1))
                 ).astype(np.float32)
    rows = np.arange(hb)

    score = band[:, 0].astype(np.float32).copy()
    # back[x][y] = window slot the trace came from to reach row y of column x
    back = np.empty((w, hb), dtype=np.int16)
    # scratch buffer: padded[max_jump : max_jump + hb] is the live score column,
    # the -inf margins make out-of-band predecessors unreachable
    padded = np.full(hb + 2 * max_jump, -np.inf, dtype=np.float32)
    # win[y, j] == score[y + j - max_jump]: every legal predecessor of row y,
    # built as a stride view so no data is copied per column
    win = np.lib.stride_tricks.sliding_window_view(padded, n_off)[:hb]
    cand = np.empty((hb, n_off), dtype=np.float32)

    for x in range(1, w):
        padded[max_jump:max_jump + hb] = score
        np.subtract(win, move_cost, out=cand)
        best = cand.argmax(axis=1)
        back[x] = best
        score = band[:, x] + cand[rows, best]

    # walk the best endpoint back to column 0
    path = np.empty(w, dtype=np.int32)
    path[w - 1] = int(np.argmax(score))
    for x in range(w - 1, 0, -1):
        path[x - 1] = path[x] + int(back[x, path[x]]) - max_jump

    return (path + r0).astype(np.float32)




# Function that ties everything together (applies blur, etc)
def detect(frame: np.ndarray, cfg: DetectorConfig) -> SurfaceResult:
    img = frame.copy() # saving original image

    # img[:cfg.mask_top_rows, :] = 0 # slicing based on mask
    # ^ this is to avoid the noise found in upper sections of scans
    # can be changed in config


    # blur strength adjusted in config settings
    blurred = cv2.GaussianBlur(img, (0, 0), sigmaX = cfg.blur_sigma)
                # ^ 0,0 is blur window. 0,0 means "auto"
    blurred = blurred.astype(np.float32, copy=False)

    # measured in rows, where top of cornea is row0 and bottom is row 510.
    #
    # We no longer take the biggest ABSOLUTE jump per column. That only worked
    # while every column had the same brightness: on real scans the surface
    # fades toward the periphery, and the strongest jump in those columns is
    # some deeper structure, so the trace dropped off the cornea. Instead we
    # score every row by how much it looks like an air/tissue boundary in
    # RELATIVE terms, then pick the single most continuous path through those
    # scores instead of deciding each column on its own.
    resp = _edge_response(blurred, cfg)
    surface_y = _trace_surface(resp, cfg)

    # grad rows = image rows - 1; kept for the boundary checks below
    h_grad = resp.shape[0]

    # median filter smooth our jagged surface
    # manages aggressive surface_y detections from col to col
    smoothed_y = median_filter(surface_y, size=cfg.smoothing_window)

    # Robust tracking point: median is immune to a handful of bad columns.
    median_y = float(np.median(smoothed_y))

    # Bounding-box top/bottom (legacy, sensitive to outliers).
    top_y = float(np.min(smoothed_y))
    bottom_y = float(np.max(smoothed_y))
    center_y = (bottom_y + top_y) / 2.0

    # Apex (highest cornea point) on the smoothed trace.
    apex_y = top_y

    # Validity check: the only reliable signal of "cornea off-screen" is the
    # apex (smallest y) sitting against the detection-window boundary. When
    # that happens the columns that USED to find the apex now snap onto the
    # next-strongest edge (often the bottom of the cornea), which makes the
    # median jump wildly -- so we cannot trust ANY tracking statistic. Freeze.
    #
    #   apex_at_top:    apex pinned at top mask -> cornea clipped above
    #   apex_at_bottom: apex pinned at bottom of image -> cornea clipped below
    #                     (rare, only happens if the whole cornea has fallen out)
    EDGE_TOL = 2  # rows of slack
    apex_at_top = apex_y <= (cfg.mask_top_rows + EDGE_TOL)
    apex_at_bottom = apex_y >= (h_grad - 1 - EDGE_TOL)

    # Diagnostic only: fraction of columns pinned to either edge.
    at_top = smoothed_y <= (cfg.mask_top_rows + EDGE_TOL)
    at_bot = smoothed_y >= (h_grad - 1 - EDGE_TOL)
    edge_fraction = float((at_top | at_bot).mean())

    valid = not (apex_at_top or apex_at_bottom)

    return SurfaceResult(
        surface_y=smoothed_y,
        top_y=top_y,
        bottom_y=bottom_y,
        center_y=center_y,
        median_y=median_y,
        apex_y=apex_y,
        valid=valid,
        edge_fraction=edge_fraction,
    )
    
    