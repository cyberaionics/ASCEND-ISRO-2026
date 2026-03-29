"""
ASCEND Phase 2 — VIO Hover Stabilizer (PID + EMA)
Daemon thread that converts ESP32-CAM optical-flow data into
roll/pitch RC corrections for X-Y plane hover stabilization.

Uses a full PID controller per axis with EMA-filtered velocity
for much more stable hover than a pure P-controller:
  - P-term: immediate response to current drift velocity
  - I-term: eliminates steady-state drift (wind, CG offset)
  - D-term: damps oscillations / overshoots
  - EMA:    smooths noisy optical-flow readings

Algorithm (runs at 20 Hz):
  1. Read (dx, dy, quality) from ESP32CamReader
  2. Skip if quality < VIO_MIN_QUALITY (insufficient features)
  3. Skip if ESP32-CAM data age > VIO_DATA_TIMEOUT (stale)
  4. Skip if TF-02 altitude < VIO_MIN_ALT_M (too low)
  5. Apply deadzone: zero out flow below VIO_DEADZONE_PX
  6. Scale to velocity: vel = (flow_px * alt_m) / (focal_len * dt)
  7. Apply EMA filter to smooth velocity
  8. Apply PID controller: correction = -(Kp*vel + Ki*∫vel + Kd*d(vel)/dt)
  9. Clamp to ±VIO_MAX_CORRECTION_PWM
  10. Expose roll_correction, pitch_correction for scheduler

Sign conventions (drone body frame → RC override):
  - ESP32-CAM dx positive = drone drifting RIGHT  → roll LEFT  (−correction)
  - ESP32-CAM dy positive = drone drifting FORWARD → pitch BACK (−correction)
  - Roll  RC: 1500 + correction (positive = right roll)
  - Pitch RC: 1500 + correction (positive = forward pitch)
"""

import threading
import time
from typing import Optional

from ..config import Config
from ..logger import Logger
from ..hardware.esp32_cam import ESP32CamReader
from ..hardware.tf02 import TF02Reader


class VIOStabilizer(threading.Thread):
    """Converts optical-flow into roll/pitch corrections for hover hold.

    Uses a PID controller per axis with EMA-filtered velocity for
    stable position hold. The corrections are exposed as thread-safe
    properties that the main state machine reads when composing
    ``send_rc_override()`` calls.

    Args:
        esp32_cam: Running ESP32CamReader instance.
        tf02:      Running TF02Reader instance (for altitude scaling).
    """

    def __init__(self, esp32_cam: ESP32CamReader, tf02: TF02Reader) -> None:
        super().__init__(daemon=True, name="VIOStabilizer")
        self._cam = esp32_cam
        self._tf02 = tf02
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        # Output corrections (PWM offset from 1500, positive = right/forward)
        self._roll_correction: int = 0
        self._pitch_correction: int = 0

        # Internal state for velocity estimation
        self._prev_time: float = 0.0

        # EMA-filtered velocities (m/s)
        self._ema_vel_x: float = 0.0
        self._ema_vel_y: float = 0.0

        # PID integral accumulators (PWM units)
        self._integral_x: float = 0.0
        self._integral_y: float = 0.0

        # PID derivative: previous filtered velocity
        self._prev_vel_x: float = 0.0
        self._prev_vel_y: float = 0.0

        # Diagnostics
        self._active: bool = False  # True when corrections are being applied

    # ── Thread-safe Properties ─────────────────────────────────────────

    @property
    def roll_correction(self) -> int:
        """Roll PWM offset from neutral 1500. Positive = tilt right."""
        with self._lock:
            return self._roll_correction

    @property
    def pitch_correction(self) -> int:
        """Pitch PWM offset from neutral 1500. Positive = tilt forward."""
        with self._lock:
            return self._pitch_correction

    @property
    def active(self) -> bool:
        """``True`` when VIO corrections are being actively computed."""
        with self._lock:
            return self._active

    def get_corrections(self) -> tuple:
        """Return ``(roll_correction, pitch_correction, active)`` atomically.

        Returns:
            Tuple of (roll_pwm_offset, pitch_pwm_offset, is_active).
        """
        with self._lock:
            return (self._roll_correction, self._pitch_correction, self._active)

    # ── Public API ─────────────────────────────────────────────────────

    def stop(self) -> None:
        """Signal the stabilizer thread to stop."""
        self._running.clear()

    # ── Core Algorithm ─────────────────────────────────────────────────

    def _compute_corrections(self) -> None:
        """Run one iteration of the VIO PID stabilization loop."""
        now = time.time()

        # ── Gate 1: ESP32-CAM data freshness ──────────────────────────
        if self._cam.data_age > Config.VIO_DATA_TIMEOUT:
            self._zero_output()
            return

        # ── Gate 2: Read flow data ────────────────────────────────────
        dx, dy, quality = self._cam.get_flow()

        # ── Gate 3: Quality check (minimum tracked features) ──────────
        if quality < Config.VIO_MIN_QUALITY:
            self._zero_output()
            return

        # ── Gate 4: Altitude check (TF-02) ────────────────────────────
        alt_m = self._tf02.distance_m
        if alt_m is None or alt_m < Config.VIO_MIN_ALT_M:
            self._zero_output()
            return

        # ── Gate 5: Time delta ────────────────────────────────────────
        if self._prev_time == 0.0:
            # First iteration — just initialise and skip
            self._prev_time = now
            self._zero_output()
            return

        dt = now - self._prev_time
        if dt <= 0.001 or dt > 0.5:
            # Invalid or too-large time gap — reset
            self._prev_time = now
            self._reset_pid()
            self._zero_output()
            return

        # ── Compute flow with deadzone ────────────────────────────────
        flow_x = dx if abs(dx) >= Config.VIO_DEADZONE_PX else 0
        flow_y = dy if abs(dy) >= Config.VIO_DEADZONE_PX else 0

        # ── Scale to approximate velocity (m/s) ──────────────────────
        #   velocity ≈ (pixel_displacement × altitude) / (focal_length × dt)
        scale = alt_m / (Config.VIO_FOCAL_LENGTH_PX * dt)
        raw_vel_x = flow_x * scale  # lateral velocity, positive = right
        raw_vel_y = flow_y * scale  # longitudinal velocity, positive = forward

        # ── EMA filter to smooth noisy optical flow ───────────────────
        alpha = Config.VIO_EMA_ALPHA
        self._ema_vel_x = alpha * raw_vel_x + (1.0 - alpha) * self._ema_vel_x
        self._ema_vel_y = alpha * raw_vel_y + (1.0 - alpha) * self._ema_vel_y
        vel_x = self._ema_vel_x
        vel_y = self._ema_vel_y

        # ── PID controller (per-axis) ─────────────────────────────────
        # P-term: proportional to current velocity
        p_x = Config.VIO_KP * vel_x
        p_y = Config.VIO_KP * vel_y

        # I-term: integral of velocity (≈ position drift) with anti-windup
        self._integral_x += vel_x * dt
        self._integral_y += vel_y * dt
        int_max = Config.VIO_INTEGRAL_MAX / max(Config.VIO_KI, 0.001)  # scale to vel units
        self._integral_x = max(-int_max, min(int_max, self._integral_x))
        self._integral_y = max(-int_max, min(int_max, self._integral_y))
        i_x = Config.VIO_KI * self._integral_x
        i_y = Config.VIO_KI * self._integral_y

        # D-term: derivative of filtered velocity (rate of acceleration)
        d_vel_x = (vel_x - self._prev_vel_x) / dt
        d_vel_y = (vel_y - self._prev_vel_y) / dt
        d_x = Config.VIO_KD * d_vel_x
        d_y = Config.VIO_KD * d_vel_y

        # ── Combine PID output ────────────────────────────────────────
        # Negative feedback: drift RIGHT (vel_x > 0) → roll LEFT (−correction)
        raw_roll = -(p_x + i_x + d_x)
        raw_pitch = -(p_y + i_y + d_y)

        # ── Clamp to safe range ───────────────────────────────────────
        max_corr = Config.VIO_MAX_CORRECTION_PWM
        roll_corr = int(max(-max_corr, min(max_corr, raw_roll)))
        pitch_corr = int(max(-max_corr, min(max_corr, raw_pitch)))

        # ── Store results ─────────────────────────────────────────────
        with self._lock:
            self._roll_correction = roll_corr
            self._pitch_correction = pitch_corr
            self._active = True

        # Update previous state for derivative
        self._prev_vel_x = vel_x
        self._prev_vel_y = vel_y
        self._prev_time = now

    def _reset_pid(self) -> None:
        """Reset PID state (integral and derivative memory)."""
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._prev_vel_x = 0.0
        self._prev_vel_y = 0.0
        self._ema_vel_x = 0.0
        self._ema_vel_y = 0.0

    def _zero_output(self) -> None:
        """Set both corrections to zero (no VIO input)."""
        with self._lock:
            self._roll_correction = 0
            self._pitch_correction = 0
            self._active = False

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        """Run the VIO stabilization loop at VIO_RATE_HZ."""
        Logger.info(
            f"VIOStabilizer running at {Config.VIO_RATE_HZ} Hz "
            f"(Kp={Config.VIO_KP}, Ki={Config.VIO_KI}, Kd={Config.VIO_KD}, "
            f"EMA={Config.VIO_EMA_ALPHA}, max_corr=±{Config.VIO_MAX_CORRECTION_PWM} PWM)"
        )
        while self._running.is_set():
            try:
                self._compute_corrections()
            except Exception as exc:
                Logger.warn(f"VIOStabilizer error: {exc}")
                self._zero_output()
            time.sleep(Config.VIO_INTERVAL)

        self._zero_output()
        Logger.info("VIOStabilizer stopped")
