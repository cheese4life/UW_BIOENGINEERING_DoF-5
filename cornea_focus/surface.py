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
    
    # midpoint between top and bottom
    center_y: float
    

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
    
    top_y = float(np.min(smoothed_y))
    bottom_y = float(np.max(smoothed_y))
    center_y = (bottom_y + top_y) / 2.0
    
    return SurfaceResult(surface_y = smoothed_y, top_y = top_y,
                        bottom_y = bottom_y, center_y = center_y)
    
    