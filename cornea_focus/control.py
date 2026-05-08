# take a measurement, return a result

import numpy as np
from dataclasses import dataclass
from cornea_focus.config import ControlConfig, UnitsConfig

@dataclass
class ControlOutput:
    error_mm: float
    target_mm: float
    clipped: bool
    in_deadband: bool

class Controller:
    def __init__(self, cfg: ControlConfig, units: UnitsConfig):
        self._filtered_y = 0.0
        self._cfg = cfg
        self._units = units
    
    # need to show psuedo code of this to team
    def step(self, center_y: float, current_pos_mm: float) -> ControlOutput:

        Kp = 0.7

        # config holds thresholds in micrometers; convert once.
        deadband_mm = self._cfg.deadband_um * 1e-3
        max_move_mm = self._cfg.max_move_um * 1e-3

        self._filtered_y = self._cfg.ema_alpha * center_y + (1 - self._cfg.ema_alpha) * self._filtered_y
        error_mm = (self._filtered_y - self._cfg.focus_line_row) * self._units.dz_mm_per_row
        if abs(error_mm) < deadband_mm:
            return ControlOutput(error_mm=error_mm, target_mm=current_pos_mm, clipped=False, in_deadband=True)
        delta_mm_raw = -Kp * error_mm
        delta_mm = float(np.clip(delta_mm_raw, -max_move_mm, max_move_mm))
        clipped = delta_mm != delta_mm_raw
        target_mm = current_pos_mm + delta_mm
        return ControlOutput(error_mm=error_mm, target_mm=target_mm, clipped=clipped, in_deadband=False)
    
    
        