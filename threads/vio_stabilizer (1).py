"""
ASCEND — VIO Hover Stabilizer (Dual-Pipeline PID Fusion)
Daemon thread that fuses two independent optical-flow pipelines for
X-Y hover hold in STABILIZE mode:

  Pipeline 1 — LK (WiFi):  Shi-Tomasi + LK flow from ESP32-CAM WiFi
                            → physics → PID → weight 0.4
  Pipeline 2 — ORB (WiFi): ORB keypoints + homography from same WiFi stream
                            → physics → PID → weight 0.6

Both pipelines read from a shared WiFiFrameReader so only ONE HTTP
connection is maintained to the ESP32-CAM.

Migration from UART:
  The old ESP32CamReader encoded dx/dy as int16 × 100 (i.e. 1 pixel =
  value of 100).  WiFiLKProcessor and WiFiORBProcessor return ACTUAL
  pixel values directly.  The ×100 scaling and integer conversion in
  _run_pipeline_lk() have been removed.

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
from ..hardware.tf02 import TF02Reader


class VIOStabilizer(threading.Thread):
    """Dual-pipeline VIO optical-flow → PID → fused roll/pitch correction.

    Each pipeline runs an independent PID controller on velocity error.
    The two corrections are fused at the output level using configurable
    weights.  When a pipeline's safety gates fail, its I-term is reset
    and it drops out of the fusion.

    Args:
        lk_flow:   Running WiFiLKProcessor instance (LK flow, required).
        tf02:      Running TF02Reader instance (altitude scaling, required).
        orb_flow:  Running WiFiORBProcessor instance (optional).

    Note:
        The parameter name ``esp32_cam`` is kept as a deprecated alias so
        existing call-sites that pass ``esp32_cam=<reader>`` still work.
        Pass either ``lk_flow=<proc>`` or ``esp32_cam=<proc>``.
    """

    def __init__(self,
                 tf02: TF02Reader,
                 lk_flow: object = None,
                 orb_flow: object = None,
                 # Legacy keyword aliases for backward compatibility
                 esp32_cam: object = None,
                 cv_flow: object = None) -> None:
        super().__init__(daemon=True, name="VIOStabilizer")

        # Accept legacy keyword names so old call sites don't break
        self._lk  = lk_flow  or esp32_cam  # Pipeline 1: LK (WiFi)
        self._orb = orb_flow or cv_flow     # Pipeline 2: ORB (WiFi)
        self._tf02 = tf02

        self._lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        # Output (PWM values ready for RC override)
        self._roll_pwm: int = 1500
        self._pitch_pwm: int = 1500
        self._is_active: bool = False

        # ── LK pipeline PID state ─────────────────────────────────────
        self._lk_prev_time: float = 0.0
        self._lk_integral_x: float = 0.0
        self._lk_integral_y: float = 0.0
        self._lk_prev_vx: float = 0.0
        self._lk_prev_vy: float = 0.0

        # ── ORB pipeline PID state ────────────────────────────────────
        self._orb_prev_time: float = 0.0
        self._orb_integral_x: float = 0.0
        self._orb_integral_y: float = 0.0
        self._orb_prev_vx: float = 0.0
        self._orb_prev_vy: float = 0.0

        # Diagnostic
        self._diag_reason: str = ""
        self._diag_last_log: float = 0.0

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
        return self.is_active

    def get_corrections(self) -> tuple:
        """Return ``(roll_pwm, pitch_pwm, is_active)`` atomically."""
        with self._lock:
            return (self._roll_pwm, self._pitch_pwm, self._is_active)

    # ── Public API ─────────────────────────────────────────────────────

    def stop(self) -> None:
        """Signal the stabilizer thread to stop."""
        self._running.clear()

    def reset(self) -> None:
        """Reset both pipeline I-terms, velocity state, AND prev_time.

        Called on state transitions (e.g. TAKEOFF → HOVER) to ensure
        the first pipeline call after reset doesn't see a stale dt.
        """
        self._lk_integral_x = 0.0
        self._lk_integral_y = 0.0
        self._lk_prev_vx = 0.0
        self._lk_prev_vy = 0.0
        self._lk_prev_time = 0.0
        self._orb_integral_x = 0.0
        self._orb_integral_y = 0.0
        self._orb_prev_vx = 0.0
        self._orb_prev_vy = 0.0
        self._orb_prev_time = 0.0
        Logger.info("VIO: PID state reset (I-terms + timing cleared)")

    # ── Per-Pipeline PID ───────────────────────────────────────────────

    def _run_pipeline_lk(self, alt_m: float, now: float) -> Optional[tuple]:
        """Run the LK pipeline (WiFiLKProcessor).

        Returns (roll_corr, pitch_corr) in PWM units, or None if gated.

        KEY CHANGE from old UART code:
          Old ESP32CamReader: dx/dy were int16 × 100 — needed /100.0
          New WiFiLKProcessor: dx/dy are actual pixels — no scaling needed
        """
        if self._lk is None:
            return None

        if self._lk.data_age > Config.VIO_DATA_TIMEOUT:
            self._lk_reset_iterm()
            return None

        dx, dy, quality = self._lk.get_flow()

        if quality < Config.VIO_MIN_QUALITY:
            self._lk_reset_iterm()
            return None

        if self._lk_prev_time == 0.0:
            self._lk_prev_time = now
            return None

        dt = now - self._lk_prev_time
        if dt <= 0.001 or dt > 0.5:
            self._lk_prev_time = now
            self._lk_reset_iterm()
            return None
        self._lk_prev_time = now

        # Deadzone — actual pixels (not ×100)
        fdx = float(dx) if abs(dx) >= Config.VIO_DEADZONE_PX else 0.0
        fdy = float(dy) if abs(dy) >= Config.VIO_DEADZONE_PX else 0.0

        # Optical flow → velocity (m/s)
        # v = (flow_pixels × altitude_m) / (focal_length_px × dt_s)
        vx = (fdx * alt_m) / (Config.VIO_FOCAL_LENGTH_PX * dt)
        vy = (fdy * alt_m) / (Config.VIO_FOCAL_LENGTH_PX * dt)

        return self._pid_compute(
            vx, vy, dt,
            Config.ESP_VIO_KP, Config.ESP_VIO_KI, Config.ESP_VIO_KD,
            "lk"
        )

    def _run_pipeline_orb(self, alt_m: float, now: float) -> Optional[tuple]:
        """Run the ORB pipeline (WiFiORBProcessor).

        Returns (roll_corr, pitch_corr) in PWM units, or None if gated.
        """
        if self._orb is None:
            return None

        if self._orb.data_age > Config.VIO_DATA_TIMEOUT:
            self._orb_reset_iterm()
            return None

        dx, dy, quality = self._orb.get_flow()

        if quality < Config.VIO_MIN_QUALITY:
            self._orb_reset_iterm()
            return None

        if self._orb_prev_time == 0.0:
            self._orb_prev_time = now
            return None

        dt = now - self._orb_prev_time
        if dt <= 0.001 or dt > 0.5:
            self._orb_prev_time = now
            self._orb_reset_iterm()
            return None
        self._orb_prev_time = now

        # Deadzone — actual pixels
        fdx = float(dx) if abs(dx) >= Config.VIO_DEADZONE_PX else 0.0
        fdy = float(dy) if abs(dy) >= Config.VIO_DEADZONE_PX else 0.0

        # Flow → velocity
        vx = (fdx * alt_m) / (Config.VIO_FOCAL_LENGTH_PX * dt)
        vy = (fdy * alt_m) / (Config.VIO_FOCAL_LENGTH_PX * dt)

        return self._pid_compute(
            vx, vy, dt,
            Config.ORB_VIO_KP, Config.ORB_VIO_KI, Config.ORB_VIO_KD,
            "orb"
        )

    def _pid_compute(self,
                     vx: float, vy: float, dt: float,
                     kp: float, ki: float, kd: float,
                     pipeline: str) -> tuple:
        """Generic PID on velocity error.  Returns (roll_corr, pitch_corr).

        Args:
            vx, vy:   Measured velocity in m/s.
            dt:       Time step in seconds.
            kp,ki,kd: PID gains for this pipeline.
            pipeline: "lk" or "orb" — selects which state variables.
        """
        p_x = -kp * vx
        p_y = -kp * vy

        int_max = Config.VIO_INTEGRAL_MAX / max(ki, 0.001)

        if pipeline == "lk":
            self._lk_integral_x += (-vx) * dt
            self._lk_integral_y += (-vy) * dt
            self._lk_integral_x = max(-int_max, min(int_max, self._lk_integral_x))
            self._lk_integral_y = max(-int_max, min(int_max, self._lk_integral_y))
            i_x = ki * self._lk_integral_x
            i_y = ki * self._lk_integral_y
            d_x = -kd * (vx - self._lk_prev_vx) / dt
            d_y = -kd * (vy - self._lk_prev_vy) / dt
            self._lk_prev_vx = vx
            self._lk_prev_vy = vy
        else:  # orb
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

        return (p_x + i_x + d_x, p_y + i_y + d_y)

    # ── I-term Reset ───────────────────────────────────────────────────

    def _lk_reset_iterm(self) -> None:
        self._lk_integral_x = 0.0
        self._lk_integral_y = 0.0
        self._lk_prev_vx = 0.0
        self._lk_prev_vy = 0.0

    def _orb_reset_iterm(self) -> None:
        self._orb_integral_x = 0.0
        self._orb_integral_y = 0.0
        self._orb_prev_vx = 0.0
        self._orb_prev_vy = 0.0

    # ── Fusion ─────────────────────────────────────────────────────────

    def _fuse_and_store(self,
                        lk_result: Optional[tuple],
                        orb_result: Optional[tuple]) -> None:
        """Renormalized weighted sum when both active; single when one."""
        lk_ok  = lk_result  is not None
        orb_ok = orb_result is not None

        if lk_ok and orb_ok:
            w_lk  = Config.ESP_VIO_WEIGHT
            w_orb = Config.ORB_VIO_WEIGHT
            total = w_lk + w_orb
            roll_corr  = (w_lk * lk_result[0]  + w_orb * orb_result[0])  / total
            pitch_corr = (w_lk * lk_result[1]  + w_orb * orb_result[1])  / total

        elif lk_ok:
            roll_corr, pitch_corr = lk_result

        elif orb_ok:
            roll_corr, pitch_corr = orb_result

        else:
            with self._lock:
                self._roll_pwm  = 1500
                self._pitch_pwm = 1500
                self._is_active = False
            return

        max_corr   = Config.VIO_MAX_CORRECTION_PWM
        roll_corr  = max(-max_corr, min(max_corr, roll_corr))
        pitch_corr = max(-max_corr, min(max_corr, pitch_corr))

        with self._lock:
            self._roll_pwm  = 1500 + int(roll_corr)
            self._pitch_pwm = 1500 + int(pitch_corr)
            self._is_active = True

    # ── Main Loop ──────────────────────────────────────────────────────

    def _compute_corrections(self) -> None:
        """One iteration: altitude gate → both pipelines → fuse → store."""
        now = time.time()

        alt_m = self._tf02.distance_m
        if alt_m is None:
            self._diag_reason = "TF02: no data"
            self._lk_reset_iterm()
            self._orb_reset_iterm()
            with self._lock:
                self._roll_pwm  = 1500
                self._pitch_pwm = 1500
                self._is_active = False
            return

        if alt_m < Config.VIO_MIN_ALT_M:
            self._diag_reason = f"alt {alt_m:.2f}m < {Config.VIO_MIN_ALT_M}m"
            self._lk_reset_iterm()
            self._orb_reset_iterm()
            with self._lock:
                self._roll_pwm  = 1500
                self._pitch_pwm = 1500
                self._is_active = False
            return

        lk_result  = self._run_pipeline_lk(alt_m, now)
        orb_result = self._run_pipeline_orb(alt_m, now)

        # Build diagnostic reason string
        if lk_result is None and orb_result is None:
            reasons = []
            if self._lk is None:
                reasons.append("LK: no source")
            elif self._lk.data_age > Config.VIO_DATA_TIMEOUT:
                reasons.append(f"LK: stale ({self._lk.data_age:.1f}s) — "
                               "check ESP32-CAM WiFi")
            elif self._lk.quality < Config.VIO_MIN_QUALITY:
                reasons.append(f"LK: low quality ({self._lk.quality})")
            else:
                reasons.append("LK: init/dt")

            if self._orb is None:
                reasons.append("ORB: no source")
            elif self._orb.data_age > Config.VIO_DATA_TIMEOUT:
                reasons.append(f"ORB: stale ({self._orb.data_age:.1f}s)")
            elif self._orb.quality < Config.VIO_MIN_QUALITY:
                reasons.append(f"ORB: low quality ({self._orb.quality})")
            else:
                reasons.append("ORB: init/dt")

            self._diag_reason = "; ".join(reasons)
        else:
            self._diag_reason = ""

        self._fuse_and_store(lk_result, orb_result)

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        """Run the VIO stabilizer loop at VIO_RATE_HZ."""
        sources = []
        if self._lk  is not None: sources.append("WiFi-LK")
        if self._orb is not None: sources.append("WiFi-ORB")
        Logger.info(
            f"VIOStabilizer running at {Config.VIO_RATE_HZ} Hz — "
            f"Sources: {', '.join(sources) or 'NONE'} | "
            f"LK  PID(Kp={Config.ESP_VIO_KP}, Ki={Config.ESP_VIO_KI}, "
            f"Kd={Config.ESP_VIO_KD}) w={Config.ESP_VIO_WEIGHT} | "
            f"ORB PID(Kp={Config.ORB_VIO_KP}, Ki={Config.ORB_VIO_KI}, "
            f"Kd={Config.ORB_VIO_KD}) w={Config.ORB_VIO_WEIGHT}"
        )

        while self._running.is_set():
            try:
                self._compute_corrections()
            except Exception as exc:
                Logger.warn(f"VIOStabilizer error: {exc}")
                self._diag_reason = f"exception: {exc}"
                with self._lock:
                    self._roll_pwm  = 1500
                    self._pitch_pwm = 1500
                    self._is_active = False

            now = time.time()
            if now - self._diag_last_log >= 1.0:
                self._diag_last_log = now
                with self._lock:
                    active = self._is_active
                    rpwm   = self._roll_pwm
                    ppwm   = self._pitch_pwm
                if active:
                    Logger.info(f"VIO ACTIVE: roll={rpwm} pitch={ppwm}")
                elif self._diag_reason:
                    Logger.warn(f"VIO INACTIVE: {self._diag_reason}")

            time.sleep(Config.VIO_INTERVAL)

        with self._lock:
            self._roll_pwm  = 1500
            self._pitch_pwm = 1500
            self._is_active = False
        Logger.info("VIOStabilizer stopped")
