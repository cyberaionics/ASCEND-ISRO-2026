"""
ASCEND Phase 2 — VIO Hover Stabilizer
Daemon thread that converts ESP32-CAM optical-flow data into
roll/pitch RC corrections for X-Y plane hover stabilization.

The stabilizer reads flow displacements from ESP32CamReader,
scales them to approximate velocities using TF-02 altitude,
and outputs P-controller corrections clamped to safe limits.

Algorithm (runs at 20 Hz):
  1. Read (dx, dy, quality) from ESP32CamReader
  2. Skip if quality < VIO_MIN_QUALITY (insufficient features)
  3. Skip if ESP32-CAM data age > VIO_DATA_TIMEOUT (stale)
  4. Skip if TF-02 altitude < VIO_MIN_ALT_M (too low)
  5. Apply deadzone: zero out flow below VIO_DEADZONE_PX
  6. Scale to velocity: vel = (flow_px * alt_m) / (focal_len * dt)
  7. Apply P-gain: correction = -VIO_KP * velocity
  8. Clamp to ±VIO_MAX_CORRECTION_PWM
  9. Expose roll_correction, pitch_correction for scheduler

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

    The corrections are exposed as thread-safe properties that the main
    state machine reads when composing ``send_rc_override()`` calls.

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
        self._prev_dx: int = 0
        self._prev_dy: int = 0
        self._prev_time: float = 0.0

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
        """Run one iteration of the VIO stabilization loop."""
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
            # First iteration — just store and skip
            self._prev_dx = dx
            self._prev_dy = dy
            self._prev_time = now
            self._zero_output()
            return

        dt = now - self._prev_time
        if dt <= 0.0 or dt > 0.5:
            # Invalid or too-large time gap — reset
            self._prev_dx = dx
            self._prev_dy = dy
            self._prev_time = now
            self._zero_output()
            return

        # ── Compute flow deltas ───────────────────────────────────────
        # The ESP32-CAM already sends per-frame displacements, so dx/dy
        # are the raw pixel flow since the previous camera frame.
        flow_x = dx
        flow_y = dy

        # ── Deadzone filter ───────────────────────────────────────────
        if abs(flow_x) < Config.VIO_DEADZONE_PX:
            flow_x = 0
        if abs(flow_y) < Config.VIO_DEADZONE_PX:
            flow_y = 0

        # ── Scale to approximate velocity (m/s) ──────────────────────
        #   velocity ≈ (pixel_displacement × altitude) / (focal_length × dt)
        # This is the standard optical-flow to ground-velocity conversion.
        scale = alt_m / (Config.VIO_FOCAL_LENGTH_PX * dt)
        vel_x = flow_x * scale  # lateral velocity (m/s), positive = right
        vel_y = flow_y * scale  # longitudinal velocity (m/s), positive = forward

        # ── P-controller ──────────────────────────────────────────────
        # Negative feedback: if drone drifts RIGHT (vel_x > 0),
        # we need to roll LEFT (negative correction).
        raw_roll = -Config.VIO_KP * vel_x
        raw_pitch = -Config.VIO_KP * vel_y

        # ── Clamp to safe range ───────────────────────────────────────
        max_corr = Config.VIO_MAX_CORRECTION_PWM
        roll_corr = int(max(-max_corr, min(max_corr, raw_roll)))
        pitch_corr = int(max(-max_corr, min(max_corr, raw_pitch)))

        # ── Store results ─────────────────────────────────────────────
        with self._lock:
            self._roll_correction = roll_corr
            self._pitch_correction = pitch_corr
            self._active = True

        # Update previous state
        self._prev_dx = dx
        self._prev_dy = dy
        self._prev_time = now

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
            f"(Kp={Config.VIO_KP}, max_corr=±{Config.VIO_MAX_CORRECTION_PWM} PWM)"
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
