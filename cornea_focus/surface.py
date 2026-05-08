import numpy as np
import cv2 # openCV used for gaus blur
from scipy.ndimage import median_filter
from dataclasses import dataclass
from cornea_focus.config import DetectorConfig

# container for output
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
    

def detect(frame: np.ndarray, cfg: DetectorConfig) -> SurfaceResult:
    img = frame.copy() # saving original image
    
    # img[:cfg.mask_top_rows, :] = 0 # slicing based on mask
    # ^ this is to avoid the noise found in upper sections of scans
    # can be changed in config
    
    
    # blur strength adjusted in config settings
    blurred = cv2.GaussianBlur(img, (0, 0), sigmaX = cfg.blur_sigma)
                # ^ 0,0 is blur window. 0,0 means "auto"
                
    # blurred[:cfg.mask_top_rows, :] = 0
                
    # np.diff is used to compute difference between elements
    grad = np.diff(blurred.astype(np.float32), axis=0)
    
    # ignore the ignored area
    grad[:cfg.mask_top_rows, :] = 0
    # axis = 0 means "subtract each row from the next row"
    
    # measured in rows, where top of cornea is row0 and bottom is row 510
    # looking for biggest jump
    
    # computing difference going downward, looking for biggest shift
    
    
    
    surface_y = np.argmax(grad, axis=0).astype(np.float32)
    
    
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
    h_grad = grad.shape[0]
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
    
    