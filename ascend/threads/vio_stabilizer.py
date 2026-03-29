"""
ASCEND — VIO Hover Stabilizer (Dual-Pipeline PID Fusion)
Daemon thread that fuses two independent optical-flow pipelines for
X-Y hover hold in STABILIZE mode:

  Pipeline 1 — ESP32-CAM:  block-matching flow → physics → PID → weight 0.4
  Pipeline 2 — ORB (RPi):  ORB keypoints + homography → physics → PID → weight 0.6

Fusion strategy:
  • Both active → renormalized weighted sum of corrections
  • Only one active → that pipeline at full weight (graceful degradation)
  • Neither active → zero output, I-terms reset
  • I-term reset on gate failure per-pipeline independently

Output: .roll_pwm and .pitch_pwm (1500 ± correction) for direct use
in PixhawkLink.send_rc_override().
"""

import threading
import time
from typing import Optional

from ..config import Config
from ..logger import Logger
from ..hardware.esp32_cam import ESP32CamReader
from ..hardware.tf02 import TF02Reader


class VIOStabilizer(threading.Thread):
    """Dual-pipeline VIO optical-flow → PID → fused roll/pitch correction.

    Each pipeline runs an independent PID controller on velocity error.
    The two corrections are fused at the output level using configurable
    weights.  When a pipeline's safety gates fail, its I-term is reset
    and it drops out of the fusion.

    Args:
        esp32_cam: Running ESP32CamReader instance (UART flow, required).
        tf02:      Running TF02Reader instance (altitude scaling, required).
        cv_flow:   Running flow processor with same interface as ESP32CamReader
                   (ORBFlowProcessor or CVFlowProcessor, optional).
    """

    def __init__(self, esp32_cam: Optional[ESP32CamReader],
                 tf02: TF02Reader,
                 cv_flow: object = None) -> None:
        super().__init__(daemon=True, name="VIOStabilizer")
        self._cam = esp32_cam
        self._cv = cv_flow
        self._tf02 = tf02
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        # Output (PWM values ready for RC override)
        self._roll_pwm: int = 1500
        self._pitch_pwm: int = 1500
        self._is_active: bool = False

        # ── ESP32 pipeline PID state ──────────────────────────────────
        self._esp_prev_time: float = 0.0
        self._esp_integral_x: float = 0.0
        self._esp_integral_y: float = 0.0
        self._esp_prev_vx: float = 0.0
        self._esp_prev_vy: float = 0.0

        # ── ORB pipeline PID state ────────────────────────────────────
        self._orb_prev_time: float = 0.0
        self._orb_integral_x: float = 0.0
        self._orb_integral_y: float = 0.0
        self._orb_prev_vx: float = 0.0
        self._orb_prev_vy: float = 0.0

    # ── Thread-safe Properties ─────────────────────────────────────────

    @property
    def roll_pwm(self) -> int:
        """Roll PWM value (1500 ± correction) for RC override."""
        with self._lock:
            return self._roll_pwm

    @property
    def pitch_pwm(self) -> int:
        """Pitch PWM value (1500 ± correction) for RC override."""
        with self._lock:
            return self._pitch_pwm

    @property
    def is_active(self) -> bool:
        """True if at least one pipeline is producing corrections."""
        with self._lock:
            return self._is_active

    @property
    def active(self) -> bool:
        """Alias for is_active (backward compat)."""
        with self._lock:
            return self._is_active

    def get_corrections(self) -> tuple:
        """Return ``(roll_pwm, pitch_pwm, is_active)`` atomically."""
        with self._lock:
            return (self._roll_pwm, self._pitch_pwm, self._is_active)

    # ── Public API ─────────────────────────────────────────────────────

    def stop(self) -> None:
        """Signal the stabilizer thread to stop."""
        self._running.clear()

    def reset(self) -> None:
        """Reset both pipeline I-terms and velocity state."""
        self._esp_integral_x = 0.0
        self._esp_integral_y = 0.0
        self._esp_prev_vx = 0.0
        self._esp_prev_vy = 0.0
        self._orb_integral_x = 0.0
        self._orb_integral_y = 0.0
        self._orb_prev_vx = 0.0
        self._orb_prev_vy = 0.0
        Logger.info("VIO: PID state reset")

    # ── Per-Pipeline PID ───────────────────────────────────────────────

    def _run_pipeline_esp(self, alt_m: float, now: float) -> Optional[tuple]:
        """Run the ESP32-CAM pipeline.  Returns (roll_corr, pitch_corr) or None.

        Flow values from ESP32CamReader are int16 × 100.
        """
        if self._cam is None:
            return None

        # Gate: data freshness
        if self._cam.data_age > Config.VIO_DATA_TIMEOUT:
            self._esp_reset_iterm()
            return None

        dx, dy, quality = self._cam.get_flow()

        # Gate: quality
        if quality < Config.VIO_MIN_QUALITY:
            self._esp_reset_iterm()
            return None

        # Time delta
        if self._esp_prev_time == 0.0:
            self._esp_prev_time = now
            return None

        dt = now - self._esp_prev_time
        if dt <= 0.001 or dt > 0.5:
            self._esp_prev_time = now
            self._esp_reset_iterm()
            return None
        self._esp_prev_time = now

        # Deadzone
        fdx = dx / 100.0 if abs(dx) >= Config.VIO_DEADZONE_PX * 100 else 0.0
        fdy = dy / 100.0 if abs(dy) >= Config.VIO_DEADZONE_PX * 100 else 0.0

        # Flow → velocity (m/s)
        vx = (fdx * alt_m) / (Config.VIO_FOCAL_LENGTH_PX * dt)
        vy = (fdy * alt_m) / (Config.VIO_FOCAL_LENGTH_PX * dt)

        # PID on velocity error (target = 0)
        return self._pid_compute(
            vx, vy, dt,
            Config.ESP_VIO_KP, Config.ESP_VIO_KI, Config.ESP_VIO_KD,
            "esp"
        )

    def _run_pipeline_orb(self, alt_m: float, now: float) -> Optional[tuple]:
        """Run the ORB pipeline.  Returns (roll_corr, pitch_corr) or None.

        Flow values from ORBFlowProcessor / CVFlowProcessor are actual pixels.
        """
        if self._cv is None:
            return None

        # Gate: data freshness
        if self._cv.data_age > Config.VIO_DATA_TIMEOUT:
            self._orb_reset_iterm()
            return None

        dx, dy, quality = self._cv.get_flow()

        # Gate: quality
        if quality < Config.VIO_MIN_QUALITY:
            self._orb_reset_iterm()
            return None

        # Time delta
        if self._orb_prev_time == 0.0:
            self._orb_prev_time = now
            return None

        dt = now - self._orb_prev_time
        if dt <= 0.001 or dt > 0.5:
            self._orb_prev_time = now
            self._orb_reset_iterm()
            return None
        self._orb_prev_time = now

        # Deadzone
        fdx = float(dx) if abs(dx) >= Config.VIO_DEADZONE_PX else 0.0
        fdy = float(dy) if abs(dy) >= Config.VIO_DEADZONE_PX else 0.0

        # Flow → velocity (m/s)
        vx = (fdx * alt_m) / (Config.VIO_FOCAL_LENGTH_PX * dt)
        vy = (fdy * alt_m) / (Config.VIO_FOCAL_LENGTH_PX * dt)

        # PID on velocity error (target = 0)
        return self._pid_compute(
            vx, vy, dt,
            Config.ORB_VIO_KP, Config.ORB_VIO_KI, Config.ORB_VIO_KD,
            "orb"
        )

    def _pid_compute(self, vx: float, vy: float, dt: float,
                     kp: float, ki: float, kd: float,
                     pipeline: str) -> tuple:
        """Generic PID computation on velocity.  Returns (roll_corr, pitch_corr).

        Args:
            vx, vy: Measured velocity in m/s.
            dt: Time step in seconds.
            kp, ki, kd: PID gains for this pipeline.
            pipeline: "esp" or "orb" — selects which state variables to use.
        """
        # Proportional (negative: rightward flow → left roll to correct)
        p_x = -kp * vx
        p_y = -kp * vy

        # Integral with anti-windup
        int_max = Config.VIO_INTEGRAL_MAX / max(ki, 0.001)

        if pipeline == "esp":
            self._esp_integral_x += (-vx) * dt
            self._esp_integral_y += (-vy) * dt
            self._esp_integral_x = max(-int_max, min(int_max, self._esp_integral_x))
            self._esp_integral_y = max(-int_max, min(int_max, self._esp_integral_y))
            i_x = ki * self._esp_integral_x
            i_y = ki * self._esp_integral_y
            # Derivative (on velocity change, not error)
            d_x = -kd * (vx - self._esp_prev_vx) / dt
            d_y = -kd * (vy - self._esp_prev_vy) / dt
            self._esp_prev_vx = vx
            self._esp_prev_vy = vy
        else:
            self._orb_integral_x += (-vx) * dt
            self._orb_integral_y += (-vy) * dt
            self._orb_integral_x = max(-int_max, min(int_max, self._orb_integral_x))
            self._orb_integral_y = max(-int_max, min(int_max, self._orb_integral_y))
            i_x = ki * self._orb_integral_x
            i_y = ki * self._orb_integral_y
            d_x = -kd * (vx - self._orb_prev_vx) / dt
            d_y = -kd * (vy - self._orb_prev_vy) / dt
            self._orb_prev_vx = vx
            self._orb_prev_vy = vy

        roll_corr = p_x + i_x + d_x
        pitch_corr = p_y + i_y + d_y

        return (roll_corr, pitch_corr)

    # ── I-term Reset ───────────────────────────────────────────────────

    def _esp_reset_iterm(self) -> None:
        """Reset ESP32 pipeline integrator and derivative state."""
        self._esp_integral_x = 0.0
        self._esp_integral_y = 0.0
        self._esp_prev_vx = 0.0
        self._esp_prev_vy = 0.0

    def _orb_reset_iterm(self) -> None:
        """Reset ORB pipeline integrator and derivative state."""
        self._orb_integral_x = 0.0
        self._orb_integral_y = 0.0
        self._orb_prev_vx = 0.0
        self._orb_prev_vy = 0.0

    # ── Fusion ─────────────────────────────────────────────────────────

    def _fuse_and_store(self, esp_result: Optional[tuple],
                        orb_result: Optional[tuple]) -> None:
        """Fuse pipeline outputs and store as thread-safe PWM values.

        Renormalized weighted sum when both active; full weight when
        only one active; zero when neither active.
        """
        esp_ok = esp_result is not None
        orb_ok = orb_result is not None

        if esp_ok and orb_ok:
            w_esp = Config.ESP_VIO_WEIGHT
            w_orb = Config.ORB_VIO_WEIGHT
            total = w_esp + w_orb  # should be 1.0, renormalize anyway
            roll_corr = (w_esp * esp_result[0] + w_orb * orb_result[0]) / total
            pitch_corr = (w_esp * esp_result[1] + w_orb * orb_result[1]) / total

        elif esp_ok:
            roll_corr = esp_result[0]
            pitch_corr = esp_result[1]

        elif orb_ok:
            roll_corr = orb_result[0]
            pitch_corr = orb_result[1]

        else:
            # Neither pipeline active
            with self._lock:
                self._roll_pwm = 1500
                self._pitch_pwm = 1500
                self._is_active = False
            return

        # Clamp corrections
        max_corr = Config.VIO_MAX_CORRECTION_PWM
        roll_corr = max(-max_corr, min(max_corr, roll_corr))
        pitch_corr = max(-max_corr, min(max_corr, pitch_corr))

        with self._lock:
            self._roll_pwm = 1500 + int(roll_corr)
            self._pitch_pwm = 1500 + int(pitch_corr)
            self._is_active = True

    # ── Main Loop ──────────────────────────────────────────────────────

    def _compute_corrections(self) -> None:
        """One iteration: run both pipelines, fuse, store."""
        now = time.time()

        # Altitude gate (common to both pipelines)
        alt_m = self._tf02.distance_m
        if alt_m is None or alt_m < Config.VIO_MIN_ALT_M:
            self._esp_reset_iterm()
            self._orb_reset_iterm()
            with self._lock:
                self._roll_pwm = 1500
                self._pitch_pwm = 1500
                self._is_active = False
            return

        # Run each pipeline independently
        esp_result = self._run_pipeline_esp(alt_m, now)
        orb_result = self._run_pipeline_orb(alt_m, now)

        # Fuse and store
        self._fuse_and_store(esp_result, orb_result)

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        """Run the VIO stabilizer loop at VIO_RATE_HZ."""
        sources = []
        if self._cam is not None:
            sources.append("ESP32-CAM")
        if self._cv is not None:
            sources.append("ORB-CV")
        Logger.info(
            f"VIOStabilizer running at {Config.VIO_RATE_HZ} Hz — "
            f"Sources: {', '.join(sources) or 'NONE'} | "
            f"ESP PID(Kp={Config.ESP_VIO_KP}, Ki={Config.ESP_VIO_KI}, "
            f"Kd={Config.ESP_VIO_KD}) w={Config.ESP_VIO_WEIGHT} | "
            f"ORB PID(Kp={Config.ORB_VIO_KP}, Ki={Config.ORB_VIO_KI}, "
            f"Kd={Config.ORB_VIO_KD}) w={Config.ORB_VIO_WEIGHT}"
        )
        while self._running.is_set():
            try:
                self._compute_corrections()
            except Exception as exc:
                Logger.warn(f"VIOStabilizer error: {exc}")
                with self._lock:
                    self._roll_pwm = 1500
                    self._pitch_pwm = 1500
                    self._is_active = False
            time.sleep(Config.VIO_INTERVAL)

        with self._lock:
            self._roll_pwm = 1500
            self._pitch_pwm = 1500
            self._is_active = False
        Logger.info("VIOStabilizer stopped")
