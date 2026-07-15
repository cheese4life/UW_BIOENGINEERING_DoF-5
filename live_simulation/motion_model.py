import numpy as np
from dataclasses import dataclass

@dataclass
class ShiftComponents:
    drift_theta: float
    drift_sigma: float
    tremor_amplitude: float
    saccade_rate: float
    saccade_amp_mu: float
    saccade_amp_sigma: float
    saccade_amp_min: float
    saccade_amp_max: float
    saccade_jump_min: float
    saccade_jump_max: float
    saccade_tau_min: float
    saccade_tau_max: float
    breath_amplitude: float
    heart_amplitude: float
    
CALM = ShiftComponents(
    drift_theta = 0.03,
    drift_sigma = 5.0,
    tremor_amplitude = 0.10,
    saccade_rate = 1.0,
    saccade_amp_mu = 2.0,
    saccade_amp_sigma = 2.0,
    saccade_amp_min = 1.0,
    saccade_amp_max = 6.0,
    saccade_jump_min = 20.0,
    saccade_jump_max = 30.0,
    saccade_tau_min = 0.05,
    saccade_tau_max = 0.15,
    breath_amplitude = 0.4,
    heart_amplitude = 0.2
)

ANXIOUS = ShiftComponents(
    drift_theta = 0.08,
    drift_sigma = 12.0,
    tremor_amplitude = 0.20,
    saccade_rate = 3.0,
    saccade_amp_mu = 5.0,
    saccade_amp_sigma = 4.0,
    saccade_amp_min = 2.0,
    saccade_amp_max = 10.0,
    saccade_jump_min = 10.0,
    saccade_jump_max = 20.0,
    saccade_tau_min = 0.04,
    saccade_tau_max = 0.08,
    breath_amplitude = 0.8,
    heart_amplitude = 0.4
)

PROFILES = {
    "calm" : CALM,
    "anxious" : ANXIOUS
}

class MotionModel:
    def __init__(self, profile: str = "calm", seed: int = 42):
        self.params = PROFILES[profile]
        
        rngs = np.random.default_rng(seed).spawn(4)
        self.drift_rng = rngs[0]
        self.tremor_rng = rngs[1]
        self.saccade_rng = rngs[2]
        self.physio_rng = rngs[3]
        
        # start params
        self.drift_x = 0.0
        self.tremor_phase = self.tremor_rng.uniform(0, 2 * np.pi)
        self.tremor_epoch_end = self.tremor_rng.uniform(2.0, 3.0)

        self.active_saccades: list[dict] = []
        self.next_event_time = self.saccade_rng.exponential(
            1.0 / self.params.saccade_rate)

        self.breath_phase = self.physio_rng.uniform(0.0, 2 * np.pi)
        self.heart_phase = self.physio_rng.uniform(0.0, 2 * np.pi)
        self.breath_last_jitter_t = 0.0
        self.heart_last_jitter_t = 0.0
        self._last_t = 0.0
        
    
    def shift_at(self, t_sec: float) -> float:
        dt = t_sec - self._last_t

        theta = self.params.drift_theta
        sigma = self.params.drift_sigma
        decay = np.exp(-theta * dt)
        noise_coef = sigma * np.sqrt((1.0 - np.exp(-2 * theta * dt)) / (2 * theta))
        self.drift_x = self.drift_x * decay + noise_coef * self.drift_rng.normal()
        
        if(abs(self.drift_x) > 48):
            self.drift_x = self.drift_x * 0.98
            
        # --- tremor ---
        TREMOR_FREQ = 87.0
        A = self.params.tremor_amplitude

        if t_sec >= self.tremor_epoch_end:
            # C⁰-continuous phase reset via arcsin
            old_val = A * np.sin(2 * np.pi * TREMOR_FREQ * t_sec + self.tremor_phase)
            dv = A * 2 * np.pi * TREMOR_FREQ * np.cos(
                2 * np.pi * TREMOR_FREQ * t_sec + self.tremor_phase)
            arg = np.clip(old_val / A, -1.0, 1.0)
            angle = np.arcsin(arg)
            if dv < 0:
                angle = np.pi - angle
            self.tremor_phase = angle - 2 * np.pi * TREMOR_FREQ * t_sec
            self.tremor_epoch_end = t_sec + self.tremor_rng.uniform(2.0, 3.0)

        tremor_val = A * np.sin(2 * np.pi * TREMOR_FREQ * t_sec + self.tremor_phase)

        # --- microsaccades ---
        # Schedule any due events (guard against huge time jumps)
        for _ in range(100):
            if t_sec < self.next_event_time:
                break
            self._schedule_saccade(t_sec)

        saccade_val = 0.0
        surviving = []
        for sacc in self.active_saccades:
            if t_sec < sacc["t0"]:
                surviving.append(sacc)
                continue
            # jump phase: linear ramp
            if t_sec < sacc["t1"]:
                saccade_val += sacc["residual"] + sacc["amplitude"] * sacc["direction"] * (t_sec - sacc["t0"]) / sacc["duration"]
                surviving.append(sacc)
                continue
            # return phase: exponential decay
            elapsed = t_sec - sacc["t1"]
            contrib = (sacc["residual"] + sacc["amplitude"] * sacc["direction"]) * np.exp(-elapsed / sacc["tau"])
            if abs(contrib) > 0.05 * abs(sacc["amplitude"]):
                saccade_val += contrib
                surviving.append(sacc)
        self.active_saccades = surviving

        # --- physio ---
        BREATH_FREQ = 0.25
        HEART_FREQ = 1.2

        # phase jitter at each zero-crossing
        old_b = np.sin(2 * np.pi * BREATH_FREQ * (t_sec - dt) + self.breath_phase)
        new_b = np.sin(2 * np.pi * BREATH_FREQ * t_sec + self.breath_phase)
        if old_b * new_b <= 0 and t_sec - self.breath_last_jitter_t > 1.0:
            self.breath_phase += self.physio_rng.normal(0.0, 0.05)
            self.breath_last_jitter_t = t_sec

        old_h = np.sin(2 * np.pi * HEART_FREQ * (t_sec - dt) + self.heart_phase)
        new_h = np.sin(2 * np.pi * HEART_FREQ * t_sec + self.heart_phase)
        if old_h * new_h <= 0 and t_sec - self.heart_last_jitter_t > 0.3:
            self.heart_phase += self.physio_rng.normal(0.0, 0.05)
            self.heart_last_jitter_t = t_sec

        physio_val = (
            self.params.breath_amplitude * np.sin(2 * np.pi * BREATH_FREQ * t_sec + self.breath_phase)
            + self.params.heart_amplitude * np.sin(2 * np.pi * HEART_FREQ * t_sec + self.heart_phase)
        )

        # --- composite ---
        self._last_t = t_sec
        return float(np.clip(self.drift_x + tremor_val + saccade_val + physio_val, -50.0, 50.0))

    def _schedule_saccade(self, t_sec: float) -> None:
        """Generate a new microsaccade event and schedule the next one."""
        p = self.params
        # amplitude: truncated normal (with sanity guard)
        for _ in range(50):
            amp = self.saccade_rng.normal(p.saccade_amp_mu, p.saccade_amp_sigma)
            if p.saccade_amp_min <= amp <= p.saccade_amp_max:
                break
        else:
            amp = float(np.clip(amp, p.saccade_amp_min, p.saccade_amp_max))
        # direction: biased toward drift sign (60% probability)
        drift_sign = 1 if self.drift_x >= 0 else -1
        direction = drift_sign if self.saccade_rng.random() < 0.6 else -drift_sign
        # jump duration (ms → s)
        t_jump = self.saccade_rng.uniform(p.saccade_jump_min, p.saccade_jump_max) / 1000.0
        # decay tau
        tau = self.saccade_rng.uniform(p.saccade_tau_min, p.saccade_tau_max)
        # residual from currently decaying saccades
        residual = 0.0
        for sacc in self.active_saccades:
            if sacc["t0"] <= t_sec < sacc["t1"]:
                residual += sacc["residual"] + sacc["amplitude"] * sacc["direction"] * (t_sec - sacc["t0"]) / sacc["duration"]
            elif t_sec >= sacc["t1"]:
                elapsed = t_sec - sacc["t1"]
                r = (sacc["residual"] + sacc["amplitude"] * sacc["direction"]) * np.exp(-elapsed / sacc["tau"])
                if abs(r) > 0.05 * abs(sacc["amplitude"]):
                    residual += r

        self.active_saccades.append({
            "t0": t_sec,
            "t1": t_sec + t_jump,
            "amplitude": amp,
            "direction": direction,
            "duration": t_jump,
            "tau": tau,
            "residual": residual,
        })
        # schedule next event (refractory >= 0.2s)
        while True:
            interval = self.saccade_rng.exponential(1.0 / p.saccade_rate)
            if interval >= 0.2:
                break
        self.next_event_time = t_sec + interval

    def reset(self) -> None:
        """Reset all state to t=0, preserving the same RNGs (trajectory repeats)."""
        self.drift_x = 0.0
        self.tremor_phase = self.tremor_rng.uniform(0, 2 * np.pi)
        self.tremor_epoch_end = self.tremor_rng.uniform(2.0, 3.0)
        self.active_saccades = []
        self.next_event_time = self.saccade_rng.exponential(1.0 / self.params.saccade_rate)
        self.breath_phase = self.physio_rng.uniform(0.0, 2 * np.pi)
        self.heart_phase = self.physio_rng.uniform(0.0, 2 * np.pi)
        self.breath_last_jitter_t = 0.0
        self.heart_last_jitter_t = 0.0
        self._last_t = 0.0

    def generate_trajectory(self, n_frames: int, fps: float = 400.0) -> np.ndarray:
        """Pre-generate a full shift array of shape (n_frames,)."""
        shifts = np.empty(n_frames, dtype=np.float32)
        for i in range(n_frames):
            shifts[i] = self.shift_at(i / fps)
        return shifts

    @property
    def state_dict(self) -> dict:
        """Serializable snapshot of all internal state (for debugging)."""
        return {
            "drift_x": self.drift_x,
            "tremor_phase": self.tremor_phase,
            "tremor_epoch_end": self.tremor_epoch_end,
            "active_saccades": len(self.active_saccades),
            "next_event_time": self.next_event_time,
            "breath_phase": self.breath_phase,
            "heart_phase": self.heart_phase,
            "breath_last_jitter_t": self.breath_last_jitter_t,
            "heart_last_jitter_t": self.heart_last_jitter_t,
            "_last_t": self._last_t,
        }
        
        
        
    
    

