from dataclasses import dataclass
import yaml
from pathlib import Path



@dataclass
class DetectorConfig:
    mask_top_rows: int
    blur_sigma: int
    peak_prominence: float
    smoothing_window: int

@dataclass
class UnitsConfig:
    pixel_to_dof_units: float
    dz_mm_per_row: float
    
@dataclass
class ControlConfig:
    focus_line_row: int
    deadband_um: float
    max_move_um: float
    ema_alpha: float

@dataclass
class SourceConfig:
    type: str
    path: str

@dataclass
class DriverConfig:
    type: str
    can_channel: str
    can_bitrate: int
    
    
@dataclass
class Config:
    detector: DetectorConfig
    control: ControlConfig
    units: UnitsConfig
    source: SourceConfig
    driver: DriverConfig
    
    
    # add load here
    
    

def load(path: str = "config.yaml") -> Config:
    raw = yaml.safe_load(Path(path).read_text())
    return Config(
        detector=DetectorConfig(**raw["detector"]),
        control=ControlConfig(**raw["control"]),
        units=UnitsConfig(**raw["units"]),
        source=SourceConfig(**raw["source"]),
        driver=DriverConfig(**raw["driver"])
    )