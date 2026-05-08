from abc import ABC, abstractmethod
from dataclasses import dataclass
import struct
import time


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
    def __init__(self, min_mm: float = -3.0, max_mm: float = 3.0, verbose: bool = False):
        self._position_mm = 0.0
        self._homed = False
        self._min_mm = min_mm
        self._max_mm = max_mm
        self._verbose = verbose
        
    def home(self):
        self._position_mm = 0.0
        self._homed = True
        if self._verbose:
            print("[MockDriver] Homed. Position reset to 0.0 mm.")
        
    def move_absolute(self, mm: float):
        if not self._homed:
            raise RuntimeError("Cannot move stage: Stage is not homed!")
        if mm < self._min_mm or mm > self._max_mm:
            raise ValueError(f"Target {mm:.4f} mm is outside limits [{self._min_mm}, {self._max_mm}].")
        self._position_mm = mm
        if self._verbose:
            print(f"[MockDriver] Moved to {mm:.4f} mm")
        
    def move_relative(self, mm: float):
        target = self._position_mm + mm
        self.move_absolute(target)
        
    def get_position(self) -> float:
        return self._position_mm
    
    def is_homed(self) -> bool:
        return self._homed
    
    def stop(self):
        if self._verbose:
            print("Stopped DOF stage")
        
    def close(self):
        if self._verbose:
            print("Ending session...")
        
    def get_status(self) -> DriverStatus:
        return DriverStatus(
            position_mm = self._position_mm,
            homed = self._homed,
            moving = False,
            error = None
        )

class RemoteDriver(DOFDriver):
    def __init__(self, base_url: str = "http://localhost:8000"):
        self._base_url = base_url

    def home(self):
        raise NotImplementedError("RemoteDriver not yet implemented. Set up the Linux host first.")

    def move_absolute(self, mm: float):
        raise NotImplementedError("RemoteDriver not yet implemented. Set up the Linux host first.")

    def move_relative(self, mm: float):
        raise NotImplementedError("RemoteDriver not yet implemented. Set up the Linux host first.")

    def get_position(self) -> float:
        raise NotImplementedError("RemoteDriver not yet implemented. Set up the Linux host first.")

    def is_homed(self) -> bool:
        raise NotImplementedError("RemoteDriver not yet implemented. Set up the Linux host first.")

    def stop(self):
        raise NotImplementedError("RemoteDriver not yet implemented. Set up the Linux host first.")

    def close(self):
        raise NotImplementedError("RemoteDriver not yet implemented. Set up the Linux host first.")

    def get_status(self) -> DriverStatus:
        raise NotImplementedError("RemoteDriver not yet implemented. Set up the Linux host first.")


# ============================================================================
# CanDriver — drives the real Dover Motion DOF-5 stage over CAN via the
# PMD Juno chip protocol. Mirrors the known-good patterns in
# dof_oscillate_v1.py. Linux + python-can + socketcan only.
# ============================================================================

# PMD Juno protocol constants (see Juno chip datasheet / dof_oscillate_v1.py)
_TX_ID = 0x600
_RX_ID = 0x580
_AXIS = 0
_COUNTS_PER_MM = 200_000
_SAMPLE_S = 51e-6
_OP_SET_POSITION = 0x10
_OP_SET_VELOCITY = 0x11
_OP_UPDATE = 0x1A
_OP_RESET_EVENT = 0x34
_OP_GET_ACT_POS = 0x37
_OP_SET_OPMODE = 0x65
_OP_SET_MOTOR_CMD = 0x77
_OP_SET_ACC = 0x90
_OP_SET_DEC = 0x91
_OP_CAL_ANALOG = 0xF5
_OPMODE_CAL = 0x06
_OPMODE_FULL = 0x37


def _vel_reg(mm_s: float) -> int:
    return int(round(mm_s * _COUNTS_PER_MM * _SAMPLE_S * 65536))


def _acc_reg(mm_s2: float) -> int:
    return int(round(mm_s2 * _COUNTS_PER_MM * _SAMPLE_S * _SAMPLE_S * 65536))


def _decode_s32(d: bytes) -> int:
    body = d[2:]
    if not body:
        return 0
    sign = b"\xff" if body[0] & 0x80 else b"\x00"
    return struct.unpack(">i", sign * (4 - len(body)) + body)[0]


class CanDriver(DOFDriver):
    """Real Dover DOF-5 stage over socketcan + PMD Juno chip protocol.

    Linux only — needs python-can + a running socketcan interface (e.g. can0).
    Defaults match the working dof_oscillate_v1.py: 1 mm/s velocity cap,
    20 mm/s^2 accel, ±1.2 mm soft limits.
    """

    def __init__(
        self,
        channel: str = "can0",
        bitrate: int = 1_000_000,
        min_mm: float = -1.2,
        max_mm: float = 1.2,
        vel_mm_s: float = 1.0,
        acc_mm_s2: float = 20.0,
    ):
        try:
            import can  # local import so non-Linux machines can still import this module
        except ImportError as e:
            raise RuntimeError(
                "python-can is required for CanDriver. `pip install python-can` "
                "and ensure a socketcan interface is up."
            ) from e
        self._can = can
        self._bus = can.interface.Bus(channel=channel, interface="socketcan", bitrate=bitrate)
        self._min_mm = min_mm
        self._max_mm = max_mm
        self._vel_mm_s = vel_mm_s
        self._acc_mm_s2 = acc_mm_s2
        self._homed = False

    # --- low-level CAN helpers ---------------------------------------------
    def _send(self, op: int, payload: bytes = b"", timeout: float = 0.2) -> bytes:
        msg = self._can.Message(
            arbitration_id=_TX_ID,
            data=bytes([_AXIS, op]) + payload,
            is_extended_id=False,
        )
        self._bus.send(msg)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            m = self._bus.recv(timeout=end - time.monotonic())
            if m and m.arbitration_id == _RX_ID:
                return bytes(m.data)
        raise RuntimeError(f"CAN timeout op=0x{op:02X}")

    def _set_motion_params(self):
        self._send(_OP_SET_ACC, struct.pack(">i", _acc_reg(self._acc_mm_s2)))
        self._send(_OP_SET_DEC, struct.pack(">i", _acc_reg(self._acc_mm_s2)))
        self._send(_OP_SET_VELOCITY, struct.pack(">i", _vel_reg(self._vel_mm_s)))

    # --- DOFDriver API -----------------------------------------------------
    def home(self):
        # Initialize the drive: clear events, calibrate analog, enable servo.
        self._send(_OP_RESET_EVENT, struct.pack(">H", 0xA000))
        self._send(_OP_RESET_EVENT, struct.pack(">H", 0xEFFF))
        self._send(_OP_SET_MOTOR_CMD, struct.pack(">h", 0))
        self._send(_OP_SET_OPMODE, struct.pack(">H", _OPMODE_CAL))
        time.sleep(0.05)
        self._send(_OP_CAL_ANALOG, struct.pack(">H", 0))
        time.sleep(0.2)
        self._send(_OP_RESET_EVENT, struct.pack(">H", 0xEFFF))
        self._send(_OP_SET_OPMODE, struct.pack(">H", _OPMODE_FULL))
        time.sleep(0.05)
        # Move to physical zero so all subsequent commands are referenced from 0.
        self._homed = True
        self._set_motion_params()
        self._send(_OP_SET_POSITION, struct.pack(">i", 0))
        self._send(_OP_UPDATE)
        print(f"[CanDriver] homed; servo on at {self.get_position():+.4f} mm")

    def move_absolute(self, mm: float):
        if not self._homed:
            raise RuntimeError("CanDriver: stage not homed")
        if mm < self._min_mm or mm > self._max_mm:
            raise ValueError(f"target {mm:+.4f} mm outside soft limits [{self._min_mm}, {self._max_mm}]")
        # Motion params (vel/accel) are set once at home() and rarely change;
        # re-sending every frame would triple CAN traffic at 30 Hz.
        target_counts = int(round(mm * _COUNTS_PER_MM))
        self._send(_OP_SET_POSITION, struct.pack(">i", target_counts))
        self._send(_OP_UPDATE)

    def move_relative(self, mm: float):
        self.move_absolute(self.get_position() + mm)

    def get_position(self) -> float:
        return _decode_s32(self._send(_OP_GET_ACT_POS)) / _COUNTS_PER_MM

    def is_homed(self) -> bool:
        return self._homed

    def stop(self):
        # Re-command current position at zero velocity update -> halts motion.
        try:
            pos = self.get_position()
            self.move_absolute(pos)
        except Exception as e:
            print(f"[CanDriver] stop failed: {e}")

    def close(self):
        try:
            self._bus.shutdown()
        except Exception:
            pass

    def get_status(self) -> DriverStatus:
        try:
            pos = self.get_position()
            return DriverStatus(position_mm=pos, homed=self._homed, moving=False, error=None)
        except Exception as e:
            return DriverStatus(position_mm=float("nan"), homed=self._homed, moving=False, error=str(e))


# ============================================================================
# Factory
# ============================================================================

def make_driver(driver_cfg) -> DOFDriver:
    """Construct the driver named by config.driver.type ('mock' or 'can')."""
    t = driver_cfg.type.lower()
    if t == "mock":
        return MockDriver()
    if t == "can":
        return CanDriver(
            channel=driver_cfg.can_channel,
            bitrate=driver_cfg.can_bitrate,
        )
    raise ValueError(f"Unknown driver type: {driver_cfg.type!r}")
    
    
    