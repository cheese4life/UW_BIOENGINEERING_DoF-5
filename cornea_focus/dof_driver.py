from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class DriverStatus:
    position_mm: float
    homed: bool
    moving: bool
    error: str | None

class DOFDriver(ABC):
    @abstractmethod
    def home(self):
        pass
    
    @abstractmethod
    def move_absolute(self, mm: float):
        pass
    
    @abstractmethod
    def move_relative(self, mm: float):
        pass
    
    @abstractmethod
    def get_position(self) -> float:
        pass
    
    @abstractmethod
    def is_homed(self) -> bool:
        pass
    
    @abstractmethod
    def stop(self):
        pass
    
    @abstractmethod
    def close(self):
        pass
    
    @abstractmethod
    def get_status(self) -> DriverStatus:
        pass
    
class MockDriver(DOFDriver):
        # refer to DOF docs for these constraints
    def __init__(self, min_mm: float = -3.0, max_mm: float = 3.0):
        self._position_mm = 0.0
        self._homed = False
        self._min_mm = min_mm
        self._max_mm = max_mm
        
    def home(self):
        self._position_mm = 0.0
        self._homed = True
        print("[MockDriver] Homed. Position reset to 0.0 mm.")
        
    def move_absolute(self, mm: float):
        if not self._homed:
            raise RuntimeError("Cannot move stage: Stage is not homed!")
        if mm < self._min_mm or mm > self._max_mm:
            raise ValueError(f"Target {mm:.4f} mm is outside limits [{self._min_mm}, {self._max_mm}].")
        self._position_mm = mm
        print(f"[MockDriver] Moved to {mm:.4f} mm")
        
    def move_relative(self, mm: float):
        target = self._position_mm + mm
        self.move_absolute(target)
        
    def get_position(self) -> float:
        return self._position_mm
    
    def is_homed(self) -> bool:
        return self._homed
    
    def stop(self):
        print("Stopped DOF stage")
        
    def close(self):
        print("Ending session...")
        
    def get_status(self) -> DriverStatus:
        return DriverStatus(
            position_mm = self._position_mm,
            homed = self._homed,
            moving = False,
            error = None
        )

        