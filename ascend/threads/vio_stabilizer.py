"""
ASCEND Phase 2 — VIO Hover Stabilizer (CV Position Hold + PID Velocity)
Daemon thread that converts ESP32-CAM optical-flow data into
roll/pitch RC corrections for X-Y plane hover stabilization.

Architecture: Cascaded Position → Velocity controller
═══════════════════════════════════════════════════════

  Outer loop (CV — Position Hold):
    • Integrates optical-flow velocity into X-Y position estimate (metres)
    • Tracks displacement from the hover origin (0, 0)
    • Position P-controller: desired_vel = -POS_KP × position_error
    • This is what pulls the drone BACK to where it started hovering

  Inner loop (PID — Velocity Control):
    • Error = measured_velocity − desired_velocity
    • PID controller converts velocity error into roll/pitch PWM
    • EMA-filtered velocity for noise rejection

  Why this works:
    Pure velocity PID only reacts to drift speed, so if the drone drifts
    1 metre and then stops (velocity = 0), the PID outputs zero — no
    force pulls it back. With position integration, the outer loop sees
    "we are 1m off" and generates a return velocity command.

Algorithm (runs at 20 Hz):
  1. Read (dx, dy, quality) from ESP32CamReader
  2. Safety gates: quality, data age, altitude
  3. Scale flow → velocity (m/s) using TF-02 altitude
  4. EMA filter on velocity
  5. INTEGRATE velocity → position displacement (metres from hover origin)
  6. Outer P-loop: pos_error → desired velocity to return
  7. Inner PID loop: vel_error → roll/pitch PWM correction
  8. Clamp to ±VIO_MAX_CORRECTION_PWM
"""

import threading
import time
from typing import Optional

from ..config import Config
from ..logger import Logger
from ..hardware.esp32_cam import ESP32CamReader
from ..hardware.tf02 import TF02Reader


class VIOStabilizer(threading.Thread):
    """Cascaded position + velocity controller for optical-flow hover hold.

    Outer loop integrates flow into position estimate (CV), inner loop
    runs PID on velocity error. Corrections are exposed as thread-safe
    properties that the state machine reads for ``send_rc_override()``.

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

        # ── CV Position Estimate (outer loop) ─────────────────────────
        # Accumulated displacement from hover origin in metres
        self._pos_x: float = 0.0   # positive = right of origin
        self._pos_y: float = 0.0   # positive = forward of origin

        # ── Velocity state (inner loop) ───────────────────────────────
        self._prev_time: float = 0.0

        # EMA-filtered velocities (m/s)
        self._ema_vel_x: float = 0.0
        self._ema_vel_y: float = 0.0

        # PID integral accumulators (velocity-error units × time)
        self._integral_x: float = 0.0
        self._integral_y: float = 0.0

        # PID derivative: previous velocity error
        self._prev_vel_err_x: float = 0.0
        self._prev_vel_err_y: float = 0.0

        # Diagnostics
        self._active: bool = False

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
        """Return ``(roll_correction, pitch_correction, active)`` atomically."""
        with self._lock:
            return (self._roll_correction, self._pitch_correction, self._active)

    def get_position(self) -> tuple:
        """Return ``(pos_x, pos_y)`` — current estimated position in metres."""
        with self._lock:
            return (self._pos_x, self._pos_y)

    # ── Public API ─────────────────────────────────────────────────────

    def stop(self) -> None:
        """Signal the stabilizer thread to stop."""
        self._running.clear()

    def reset_position(self) -> None:
        """Reset the CV position origin to current location.

        Called when entering HOVER state so the drone holds from
        wherever it stabilised, not from where it took off.
        """
        with self._lock:
            self._pos_x = 0.0
            self._pos_y = 0.0
            self._integral_x = 0.0
            self._integral_y = 0.0
            self._prev_vel_err_x = 0.0
            self._prev_vel_err_y = 0.0
            self._ema_vel_x = 0.0
            self._ema_vel_y = 0.0
        Logger.info("VIO: position origin reset to (0, 0)")

    # ── Core Algorithm ─────────────────────────────────────────────────

    def _compute_corrections(self) -> None:
        """One iteration of the cascaded position → velocity controller."""
        now = time.time()

        # ── Gate 1: ESP32-CAM data freshness ──────────────────────────
        if self._cam.data_age > Config.VIO_DATA_TIMEOUT:
            self._zero_output()
            return

        # ── Gate 2: Read flow data ────────────────────────────────────
        dx, dy, quality = self._cam.get_flow()

        # ── Gate 3: Quality check ─────────────────────────────────────
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
            self._prev_time = now
            self._zero_output()
            return

        dt = now - self._prev_time
        if dt <= 0.001 or dt > 0.5:
            self._prev_time = now
            self._reset_state()
            self._zero_output()
            return

        # ══════════════════════════════════════════════════════════════
        # STEP 1: Flow → Velocity (with deadzone + EMA)
        # ══════════════════════════════════════════════════════════════
        flow_x = dx if abs(dx) >= Config.VIO_DEADZONE_PX else 0
        flow_y = dy if abs(dy) >= Config.VIO_DEADZONE_PX else 0

        # Standard optical-flow to ground-velocity conversion
        scale = alt_m / (Config.VIO_FOCAL_LENGTH_PX * dt)
        raw_vel_x = flow_x * scale
        raw_vel_y = flow_y * scale

        # EMA filter
        alpha = Config.VIO_EMA_ALPHA
        self._ema_vel_x = alpha * raw_vel_x + (1.0 - alpha) * self._ema_vel_x
        self._ema_vel_y = alpha * raw_vel_y + (1.0 - alpha) * self._ema_vel_y
        vel_x = self._ema_vel_x
        vel_y = self._ema_vel_y

        # ══════════════════════════════════════════════════════════════
        # STEP 2: CV — Integrate velocity → position (outer loop)
        # ══════════════════════════════════════════════════════════════
        # Accumulate displacement from hover origin
        self._pos_x += vel_x * dt
        self._pos_y += vel_y * dt

        # Slow exponential decay to handle optical flow integration drift
        # over long hover durations. Without this, small systematic OF
        # bias would accumulate into a large phantom position offset.
        self._pos_x *= Config.VIO_POS_DECAY
        self._pos_y *= Config.VIO_POS_DECAY

        # Clamp position to sane range
        pos_max = Config.VIO_POS_MAX_M
        self._pos_x = max(-pos_max, min(pos_max, self._pos_x))
        self._pos_y = max(-pos_max, min(pos_max, self._pos_y))

        # ══════════════════════════════════════════════════════════════
        # STEP 3: Outer P-loop — position error → desired velocity
        # ══════════════════════════════════════════════════════════════
        # If drone is 0.5m RIGHT of origin (pos_x=+0.5), we want it to
        # move LEFT → desired velocity is negative (towards origin).
        desired_vel_x = -Config.VIO_POS_KP * self._pos_x
        desired_vel_y = -Config.VIO_POS_KP * self._pos_y

        # ══════════════════════════════════════════════════════════════
        # STEP 4: Inner PID loop — velocity error → PWM correction
        # ══════════════════════════════════════════════════════════════
        vel_err_x = vel_x - desired_vel_x
        vel_err_y = vel_y - desired_vel_y

        # P-term
        p_x = Config.VIO_KP * vel_err_x
        p_y = Config.VIO_KP * vel_err_y

        # I-term with anti-windup
        self._integral_x += vel_err_x * dt
        self._integral_y += vel_err_y * dt
        int_max = Config.VIO_INTEGRAL_MAX / max(Config.VIO_KI, 0.001)
        self._integral_x = max(-int_max, min(int_max, self._integral_x))
        self._integral_y = max(-int_max, min(int_max, self._integral_y))
        i_x = Config.VIO_KI * self._integral_x
        i_y = Config.VIO_KI * self._integral_y

        # D-term (derivative of velocity error)
        d_err_x = (vel_err_x - self._prev_vel_err_x) / dt
        d_err_y = (vel_err_y - self._prev_vel_err_y) / dt
        d_x = Config.VIO_KD * d_err_x
        d_y = Config.VIO_KD * d_err_y

        # ══════════════════════════════════════════════════════════════
        # STEP 5: Combine + clamp
        # ══════════════════════════════════════════════════════════════
        # Negative feedback: positive vel_err (drifting right/forward)
        # → negative roll/pitch (tilt left/back to counteract)
        raw_roll = -(p_x + i_x + d_x)
        raw_pitch = -(p_y + i_y + d_y)

        max_corr = Config.VIO_MAX_CORRECTION_PWM
        roll_corr = int(max(-max_corr, min(max_corr, raw_roll)))
        pitch_corr = int(max(-max_corr, min(max_corr, raw_pitch)))

        # ── Store results ─────────────────────────────────────────────
        with self._lock:
            self._roll_correction = roll_corr
            self._pitch_correction = pitch_corr
            self._active = True

        # Update state
        self._prev_vel_err_x = vel_err_x
        self._prev_vel_err_y = vel_err_y
        self._prev_time = now

    def _reset_state(self) -> None:
        """Reset velocity/PID state (not position — that persists)."""
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._prev_vel_err_x = 0.0
        self._prev_vel_err_y = 0.0
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
            f"VIOStabilizer running at {Config.VIO_RATE_HZ} Hz — "
            f"Cascaded POS(Kp={Config.VIO_POS_KP}) → "
            f"VEL(Kp={Config.VIO_KP}, Ki={Config.VIO_KI}, Kd={Config.VIO_KD}), "
            f"EMA={Config.VIO_EMA_ALPHA}, max=±{Config.VIO_MAX_CORRECTION_PWM} PWM"
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
