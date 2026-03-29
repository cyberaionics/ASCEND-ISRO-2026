"""
ASCEND Phase 2 — VIO Hover Stabilizer (CV Fusion + Cascaded PID)
Daemon thread that fuses two optical-flow sources for X-Y hover hold:

  1. ESP32-CAM  — 30 Hz UART flow (fast, low quality, 96×96 simplified LK)
  2. RPi5 CV    — 30 Hz OpenCV flow (slower, HIGH quality, Shi-Tomasi + LK)

Sensor Fusion Strategy:
  • When BOTH sources have fresh data → weighted average
    (CV_FLOW_WEIGHT for CV, 1-CV_FLOW_WEIGHT for ESP32)
  • When only ONE source has data → use that source at full weight
  • When NEITHER has data → zero output (graceful degradation)

Architecture: Cascaded Position → Velocity Controller
  Outer loop: Integrates fused velocity → position estimate → P-controller
  Inner loop: PID on velocity error → roll/pitch PWM correction
"""

import threading
import time
from typing import Optional

from ..config import Config
from ..logger import Logger
from ..hardware.esp32_cam import ESP32CamReader
from ..hardware.tf02 import TF02Reader


class VIOStabilizer(threading.Thread):
    """Fused CV + ESP32-CAM optical-flow → cascaded position+velocity hold.

    Accepts two optional flow sources. Uses whichever has fresh data,
    preferring the CV source (higher quality) via weighted fusion.

    Args:
        esp32_cam: Running ESP32CamReader instance (UART flow, optional).
        tf02:      Running TF02Reader instance (altitude scaling).
        cv_flow:   Running CVFlowProcessor instance (OpenCV flow, optional).
    """

    def __init__(self, esp32_cam: Optional[ESP32CamReader],
                 tf02: TF02Reader,
                 cv_flow=None) -> None:
        super().__init__(daemon=True, name="VIOStabilizer")
        self._cam = esp32_cam
        self._cv = cv_flow
        self._tf02 = tf02
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        # Output corrections (PWM offset from 1500)
        self._roll_correction: int = 0
        self._pitch_correction: int = 0

        # ── CV Position Estimate (outer loop) ─────────────────────────
        self._pos_x: float = 0.0   # metres right of hover origin
        self._pos_y: float = 0.0   # metres forward of hover origin

        # ── Velocity state (inner loop) ───────────────────────────────
        self._prev_time: float = 0.0
        self._ema_vel_x: float = 0.0
        self._ema_vel_y: float = 0.0

        # PID state
        self._integral_x: float = 0.0
        self._integral_y: float = 0.0
        self._prev_vel_err_x: float = 0.0
        self._prev_vel_err_y: float = 0.0

        # Diagnostics
        self._active: bool = False
        self._source: str = "none"  # "cv", "esp32", "fused", "none"

    # ── Thread-safe Properties ─────────────────────────────────────────

    @property
    def roll_correction(self) -> int:
        with self._lock:
            return self._roll_correction

    @property
    def pitch_correction(self) -> int:
        with self._lock:
            return self._pitch_correction

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    @property
    def source(self) -> str:
        """Which flow source is currently active: 'cv', 'esp32', 'fused', 'none'."""
        with self._lock:
            return self._source

    def get_corrections(self) -> tuple:
        """Return ``(roll_correction, pitch_correction, active)`` atomically."""
        with self._lock:
            return (self._roll_correction, self._pitch_correction, self._active)

    def get_position(self) -> tuple:
        """Return ``(pos_x, pos_y)`` in metres from hover origin."""
        with self._lock:
            return (self._pos_x, self._pos_y)

    # ── Public API ─────────────────────────────────────────────────────

    def stop(self) -> None:
        self._running.clear()

    def reset_position(self) -> None:
        """Reset to current location as hover origin."""
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

    # ── Sensor Fusion ──────────────────────────────────────────────────

    def _get_fused_flow(self) -> tuple:
        """Read and fuse flow from available sources.

        Returns:
            (dx, dy, quality, source_name) or None if no valid data.
        """
        cv_valid = False
        esp_valid = False
        cv_dx, cv_dy, cv_q = 0.0, 0.0, 0
        esp_dx, esp_dy, esp_q = 0, 0, 0
        timeout = Config.VIO_DATA_TIMEOUT

        # ── Check CV flow (RPi5 OpenCV) ───────────────────────────────
        if self._cv is not None:
            if self._cv.data_age < timeout:
                cv_dx, cv_dy, cv_q = self._cv.get_flow()
                if cv_q >= Config.VIO_MIN_QUALITY:
                    cv_valid = True

        # ── Check ESP32-CAM flow (UART) ───────────────────────────────
        if self._cam is not None:
            if self._cam.data_age < timeout:
                esp_dx, esp_dy, esp_q = self._cam.get_flow()
                if esp_q >= Config.VIO_MIN_QUALITY:
                    esp_valid = True

        # ── Fuse ──────────────────────────────────────────────────────
        if cv_valid and esp_valid:
            # Weighted average — CV gets higher weight (more accurate)
            w_cv = Config.CV_FLOW_WEIGHT
            w_esp = 1.0 - w_cv
            dx = w_cv * cv_dx + w_esp * esp_dx
            dy = w_cv * cv_dy + w_esp * esp_dy
            q = max(cv_q, esp_q)
            return (dx, dy, q, "fused")

        elif cv_valid:
            return (cv_dx, cv_dy, cv_q, "cv")

        elif esp_valid:
            return (float(esp_dx), float(esp_dy), esp_q, "esp32")

        return None  # No valid source

    # ── Core Algorithm ─────────────────────────────────────────────────

    def _compute_corrections(self) -> None:
        """One iteration of the fused cascaded controller."""
        now = time.time()

        # ── Get fused flow ────────────────────────────────────────────
        fused = self._get_fused_flow()
        if fused is None:
            self._zero_output()
            return

        flow_dx, flow_dy, quality, src = fused

        # ── Altitude check ────────────────────────────────────────────
        alt_m = self._tf02.distance_m
        if alt_m is None or alt_m < Config.VIO_MIN_ALT_M:
            self._zero_output()
            return

        # ── Time delta ────────────────────────────────────────────────
        if self._prev_time == 0.0:
            self._prev_time = now
            with self._lock:
                self._source = src
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
        fx = flow_dx if abs(flow_dx) >= Config.VIO_DEADZONE_PX else 0.0
        fy = flow_dy if abs(flow_dy) >= Config.VIO_DEADZONE_PX else 0.0

        scale = alt_m / (Config.VIO_FOCAL_LENGTH_PX * dt)
        raw_vel_x = fx * scale
        raw_vel_y = fy * scale

        alpha = Config.VIO_EMA_ALPHA
        self._ema_vel_x = alpha * raw_vel_x + (1.0 - alpha) * self._ema_vel_x
        self._ema_vel_y = alpha * raw_vel_y + (1.0 - alpha) * self._ema_vel_y
        vel_x = self._ema_vel_x
        vel_y = self._ema_vel_y

        # ══════════════════════════════════════════════════════════════
        # STEP 2: Integrate velocity → position (CV position hold)
        # ══════════════════════════════════════════════════════════════
        self._pos_x += vel_x * dt
        self._pos_y += vel_y * dt

        self._pos_x *= Config.VIO_POS_DECAY
        self._pos_y *= Config.VIO_POS_DECAY

        pos_max = Config.VIO_POS_MAX_M
        self._pos_x = max(-pos_max, min(pos_max, self._pos_x))
        self._pos_y = max(-pos_max, min(pos_max, self._pos_y))

        # ══════════════════════════════════════════════════════════════
        # STEP 3: Outer P-loop — position → desired velocity
        # ══════════════════════════════════════════════════════════════
        desired_vel_x = -Config.VIO_POS_KP * self._pos_x
        desired_vel_y = -Config.VIO_POS_KP * self._pos_y

        # ══════════════════════════════════════════════════════════════
        # STEP 4: Inner PID — velocity error → PWM correction
        # ══════════════════════════════════════════════════════════════
        vel_err_x = vel_x - desired_vel_x
        vel_err_y = vel_y - desired_vel_y

        # P
        p_x = Config.VIO_KP * vel_err_x
        p_y = Config.VIO_KP * vel_err_y

        # I with anti-windup
        self._integral_x += vel_err_x * dt
        self._integral_y += vel_err_y * dt
        int_max = Config.VIO_INTEGRAL_MAX / max(Config.VIO_KI, 0.001)
        self._integral_x = max(-int_max, min(int_max, self._integral_x))
        self._integral_y = max(-int_max, min(int_max, self._integral_y))
        i_x = Config.VIO_KI * self._integral_x
        i_y = Config.VIO_KI * self._integral_y

        # D
        d_err_x = (vel_err_x - self._prev_vel_err_x) / dt
        d_err_y = (vel_err_y - self._prev_vel_err_y) / dt
        d_x = Config.VIO_KD * d_err_x
        d_y = Config.VIO_KD * d_err_y

        # ══════════════════════════════════════════════════════════════
        # STEP 5: Combine + clamp
        # ══════════════════════════════════════════════════════════════
        raw_roll = -(p_x + i_x + d_x)
        raw_pitch = -(p_y + i_y + d_y)

        max_corr = Config.VIO_MAX_CORRECTION_PWM
        roll_corr = int(max(-max_corr, min(max_corr, raw_roll)))
        pitch_corr = int(max(-max_corr, min(max_corr, raw_pitch)))

        with self._lock:
            self._roll_correction = roll_corr
            self._pitch_correction = pitch_corr
            self._active = True
            self._source = src

        self._prev_vel_err_x = vel_err_x
        self._prev_vel_err_y = vel_err_y
        self._prev_time = now

    def _reset_state(self) -> None:
        """Reset velocity/PID state (not position)."""
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._prev_vel_err_x = 0.0
        self._prev_vel_err_y = 0.0
        self._ema_vel_x = 0.0
        self._ema_vel_y = 0.0

    def _zero_output(self) -> None:
        with self._lock:
            self._roll_correction = 0
            self._pitch_correction = 0
            self._active = False
            self._source = "none"

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        sources = []
        if self._cam is not None:
            sources.append("ESP32-CAM")
        if self._cv is not None:
            sources.append("RPi5-CV")
        Logger.info(
            f"VIOStabilizer running at {Config.VIO_RATE_HZ} Hz — "
            f"Sources: {', '.join(sources) or 'NONE'} | "
            f"Fusion weight: CV={Config.CV_FLOW_WEIGHT:.0%} ESP32={1-Config.CV_FLOW_WEIGHT:.0%} | "
            f"POS(Kp={Config.VIO_POS_KP}) → "
            f"VEL(Kp={Config.VIO_KP}, Ki={Config.VIO_KI}, Kd={Config.VIO_KD})"
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
