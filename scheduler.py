"""
ASCEND — Scheduler & State Machine
Entry point for all CLI modes.  Run as: python3 -m ascend --mode <mode>

State machine for the qualification round (Tasks 1–3):
    IDLE → PREFLIGHT → ARM → TAKEOFF → HOVER → LAND → DISARM → DONE

All flight is in STABILIZE mode using RC override for throttle, roll, pitch.
VIO corrections are applied from the dual-pipeline VIOStabilizer during
TAKEOFF and HOVER.  SafetyMonitor.emergency_flag triggers LAND from any state.
Only the StateMachine writes to PixhawkLink (no concurrent writers aside
from HeartbeatSender and RangefinderBridge which use the send_lock).

WiFi migration:
  Removed: ESP32CamReader (UART ttyAMA2), ESP32FrameReader (UART ttyAMA3),
           ORBFlowProcessor (raw UART frames).
  Added:   WiFiFrameReader  — single MJPEG connection to ESP32-CAM
           WiFiLKProcessor  — Shi-Tomasi + LK flow (replaces ESP32CamReader)
           WiFiORBProcessor — ORB + homography flow (replaces ORBFlowProcessor)
"""

import argparse
import enum
import signal
import sys
import threading
import time
from typing import Optional

from pymavlink import mavutil

from .config import Config
from .logger import Logger
from .hardware.tf02 import TF02Reader
from .hardware.pixhawk import PixhawkLink
from .hardware.wifi_flow import WiFiFrameReader, WiFiLKProcessor, WiFiORBProcessor, WhiteDetector
from .threads.bridge import (
    HeartbeatSender,
    RangefinderBridge,
    SafetyMonitor,
    TelemetryStreamer,
)
from .threads.vio_stabilizer import VIOStabilizer
from .checks.health import HealthChecker
from .checks.setup import AutoTuneSetup
from .checks.monitor import AutoTuneMonitor


# ═══════════════════════════════════════════════════════════════════════════
# Flight State Enum
# ═══════════════════════════════════════════════════════════════════════════

class State(enum.Enum):
    """Flight state machine states (qualification round)."""
    IDLE      = "IDLE"
    PREFLIGHT = "PREFLIGHT"
    ARM       = "ARM"
    TAKEOFF   = "TAKEOFF"
    HOVER     = "HOVER"
    LAND      = "LAND"
    DISARM    = "DISARM"
    DONE      = "DONE"


# ═══════════════════════════════════════════════════════════════════════════
# State Machine  (unchanged from original — only imports changed)
# ═══════════════════════════════════════════════════════════════════════════

class StateMachine:
    """Central flight sequencer for the ASCEND qualification round.

    Manages state transitions:
        IDLE → PREFLIGHT → ARM → TAKEOFF → HOVER → LAND → DISARM → DONE

    Safety:
      - SafetyMonitor.emergency_flag polled every loop tick
      - Ctrl+C → immediate LAND + DISARM

    VIO:
      - VIOStabilizer provides roll_pwm / pitch_pwm corrections
      - Applied during TAKEOFF and HOVER via RC override

    Args:
        pixhawk: Connected PixhawkLink instance.
        tf02:    Running TF02Reader instance.
        safety:  Running SafetyMonitor instance.
        vio:     Running VIOStabilizer instance (optional, for X-Y hold).
    """

    def __init__(self, pixhawk: PixhawkLink, tf02: TF02Reader,
                 safety: SafetyMonitor,
                 vio: Optional[VIOStabilizer] = None) -> None:
        self._px = pixhawk
        self._tf02 = tf02
        self._safety = safety
        self._vio = vio
        self._state = State.IDLE
        self._running: bool = True

        self._state_enter_time: float = 0.0
        self._hover_start: Optional[float] = None
        self._land_start: Optional[float] = None

        self._armed: bool = False
        self._mode: int = 0
        self._mode_name: str = "UNKNOWN"
        self._battery_volt: float = 0.0
        self._battery_pct: int = 0

        self._flight_start: float = 0.0
        self._min_alt: float = float("inf")
        self._max_alt: float = 0.0
        self._vio_active_ticks: int = 0
        self._total_ticks: int = 0

        self._ramp_throttle: int = Config.TAKEOFF_START_PWM
        self._current_throttle: int = Config.MIN_THROTTLE_PWM

        # VIO drift tracking — accumulated X-Y displacement from takeoff
        self._drift_x: float = 0.0
        self._drift_y: float = 0.0
        self._drift_last_time: float = 0.0

        # White-pad detector
        self._white_detector: Optional[WhiteDetector] = None
        self._white_off_count: int = 0  # consecutive off-pad readings

    def get_telemetry(self) -> dict:
        """Build a telemetry dict for the TelemetryStreamer."""
        alt_m = self._tf02.distance_m
        telem = {
            "state":        self._state.value,
            "altitude_m":   alt_m if alt_m is not None else 0.0,
            "battery_pct":  self._battery_pct,
            "battery_volt": self._battery_volt,
            "armed":        self._armed,
            "mode":         self._mode_name,
            "tf02_dist_m":  alt_m,
        }
        if self._vio is not None:
            roll, pitch, active = self._vio.get_corrections()
            telem["vio_active"]    = active
            telem["vio_roll_pwm"]  = roll
            telem["vio_pitch_pwm"] = pitch
        else:
            telem["vio_active"]    = False
            telem["vio_roll_pwm"]  = 1500
            telem["vio_pitch_pwm"] = 1500
        return telem

    def run(self) -> None:
        """Execute the state machine main loop."""
        Logger.header("ASCEND STATE MACHINE — FLY MODE")
        Logger.info(f"Target altitude  : {Config.HOVER_TARGET_ALT_M} m")
        Logger.info(f"Hover duration   : {Config.HOVER_DURATION_S} s")
        Logger.info(f"Takeoff threshold: {Config.TAKEOFF_ALT_THRESHOLD} m")
        Logger.info("Press Ctrl+C to abort → emergency LAND\n")

        self._px.request_data_streams(rate_hz=10)
        self._transition(State.IDLE)

        try:
            while self._running:
                self._poll_mavlink()

                if self._state not in (State.DONE, State.LAND, State.DISARM):
                    if self._safety.emergency_flag:
                        Logger.error(
                            f"Safety trigger: {self._safety.emergency_reason}"
                        )
                        self._transition(State.LAND)

                # Drift kill — immediate DISARM if drone drifts off board
                if self._state in (State.TAKEOFF, State.HOVER):
                    self._check_drift_kill()
                    self._check_white_pad()

                handler = {
                    State.IDLE:      self._state_idle,
                    State.PREFLIGHT: self._state_preflight,
                    State.ARM:       self._state_arm,
                    State.TAKEOFF:   self._state_takeoff,
                    State.HOVER:     self._state_hover,
                    State.LAND:      self._state_land,
                    State.DISARM:    self._state_disarm,
                    State.DONE:      self._state_done,
                }.get(self._state)

                if handler:
                    handler()

                time.sleep(Config.MAIN_LOOP_INTERVAL)

        except KeyboardInterrupt:
            Logger.warn("Ctrl+C — sending LAND + DISARM")
            self._px.clear_rc_override()
            self._px.set_mode("LAND")
            time.sleep(0.5)
            self._px.disarm()
        finally:
            self._running = False

    def stop(self) -> None:
        self._running = False

    def _poll_mavlink(self) -> None:
        for _ in range(20):
            msg = self._px.recv_any(timeout=0.005)
            if msg is None:
                break
            mtype = msg.get_type()

            if mtype == "HEARTBEAT" and msg.get_srcSystem() == Config.SYSTEM_ID:
                self._armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                if msg.custom_mode != self._mode:
                    self._mode = msg.custom_mode
                    self._mode_name = self._resolve_mode(msg.custom_mode)
                    Logger.info(f"FC mode → {self._mode_name}")

            elif mtype == "SYS_STATUS":
                self._battery_volt = msg.voltage_battery / 1000.0
                self._battery_pct  = max(0, min(100, msg.battery_remaining))

            elif mtype == "LOCAL_POSITION_NED":
                self._safety.update_position(msg.x, msg.y)

            elif mtype == "STATUSTEXT":
                Logger.warn(f"FC: {msg.text}")

    def _transition(self, new_state: State) -> None:
        old = self._state
        self._state = new_state
        self._state_enter_time = time.time()
        Logger.info(f"State: {old.value} → {new_state.value}")

        if new_state == State.HOVER and self._vio is not None:
            self._vio.reset()

    def _resolve_mode(self, custom_mode: int) -> str:
        for name, num in Config.MODE_MAP.items():
            if num == custom_mode:
                return name
        return f"MODE_{custom_mode}"

    # ── Drift Kill ─────────────────────────────────────────────────────

    def _accumulate_drift(self) -> None:
        """Integrate VIO velocity to track total X-Y drift from takeoff."""
        if self._vio is None:
            return
        vx, vy = self._vio.get_velocity()
        now = time.time()
        if self._drift_last_time > 0:
            dt = now - self._drift_last_time
            if 0 < dt < 0.5:  # sanity check
                self._drift_x += vx * dt
                self._drift_y += vy * dt
        self._drift_last_time = now

    def _check_drift_kill(self) -> None:
        """EMERGENCY DISARM if drone drifts beyond DRIFT_KILL_M."""
        if not Config.DRIFT_KILL_ENABLED:
            return
        self._accumulate_drift()
        drift_total = (self._drift_x ** 2 + self._drift_y ** 2) ** 0.5
        if drift_total > Config.DRIFT_KILL_M:
            Logger.error(
                f"⚠ DRIFT KILL — drift={drift_total:.2f}m "
                f"(x={self._drift_x:.2f}, y={self._drift_y:.2f}) "
                f"> limit {Config.DRIFT_KILL_M}m — KILLING MOTORS"
            )
            self._px.clear_rc_override()
            self._px.disarm()
            self._running = False

    def _check_white_pad(self) -> None:
        """LAND if camera no longer sees white board underneath."""
        if not Config.WHITE_CHECK_ENABLED:
            return
        if self._white_detector is None:
            return
        # Grace period: skip check right after takeoff
        elapsed = time.time() - self._flight_start
        if elapsed < Config.WHITE_GRACE_S:
            return
        # Skip if white detector data is stale
        if self._white_detector.data_age > 2.0:
            return

        if not self._white_detector.is_over_white:
            self._white_off_count += 1
            if self._white_off_count >= 3:  # 3 consecutive readings (~0.6s)
                Logger.error(
                    f"⚠ WHITE PAD LOST — white_ratio="
                    f"{self._white_detector.white_ratio:.2f} "
                    f"< {Config.WHITE_MIN_RATIO} — LANDING NOW"
                )
                self._transition(State.LAND)
        else:
            self._white_off_count = 0

    # ── State Handlers (unchanged logic) ──────────────────────────────

    def _state_idle(self) -> None:
        self._transition(State.PREFLIGHT)

    def _state_preflight(self) -> None:
        # Check TF-02 data available
        if self._tf02.distance_m is not None:
            Logger.ok(f"Preflight: TF-02 = {self._tf02.distance_m:.2f} m")
            self._transition(State.ARM)
        elif time.time() - self._state_enter_time > Config.PREFLIGHT_TIMEOUT:
            Logger.error("Preflight timeout — TF-02 no data")
            self._transition(State.DONE)

    def _state_arm(self) -> None:
        elapsed = time.time() - self._state_enter_time

        # Send arm command only ONCE at the start — not every loop tick
        if elapsed < 0.1:
            Logger.info("Setting STABILIZE mode …")
            self._px.set_mode("STABILIZE")

        if elapsed > 0.5 and not self._armed and not getattr(self, '_arm_sent', False):
            Logger.info("Sending ARM command …")
            self._px.arm()
            self._arm_sent = True
            Logger.info("Waiting for FC to confirm armed …")

        if self._armed:
            Logger.ok("Armed successfully ✓")
            self._arm_sent = False
            self._flight_start = time.time()
            self._safety.set_flight_active(True)
            self._transition(State.TAKEOFF)
        elif elapsed > Config.ARM_TIMEOUT_S:
            Logger.error(f"Arm timeout after {elapsed:.1f}s — check arming switches")
            self._arm_sent = False
            self._transition(State.DONE)

    def _state_takeoff(self) -> None:
        now     = time.time()
        elapsed = now - self._state_enter_time

        # ── Throttle ramp: +10 PWM every 100 ms ──────────────────────
        # On first entry, initialise ramp tracking variables.
        if not hasattr(self, '_last_ramp_time'):
            self._last_ramp_time = now
            self._current_throttle = Config.TAKEOFF_START_PWM

        # Increase throttle by TAKEOFF_RAMP_PWM_STEP every TAKEOFF_RAMP_INTERVAL
        if now - self._last_ramp_time >= Config.TAKEOFF_RAMP_INTERVAL:
            self._current_throttle += Config.TAKEOFF_RAMP_PWM_STEP
            self._last_ramp_time = now

        # Clamp to maximum allowed throttle
        self._current_throttle = min(
            self._current_throttle, Config.TAKEOFF_MAX_PWM
        )

        roll_pwm  = self._vio.roll_pwm  if self._vio else 1500
        pitch_pwm = self._vio.pitch_pwm if self._vio else 1500

        # Apply trim offsets to compensate for mechanical drift
        roll_pwm  += Config.TRIM_ROLL_PWM
        pitch_pwm += Config.TRIM_PITCH_PWM

        self._px.send_rc_override(
            throttle=self._current_throttle,
            roll=roll_pwm,
            pitch=pitch_pwm,
        )

        alt_m = self._tf02.distance_m
        alt_str = f"{alt_m:.2f}m" if alt_m is not None else "---"

        # Log throttle + altitude + VIO + velocity every second
        if not hasattr(self, '_last_takeoff_log') or now - self._last_takeoff_log >= 1.0:
            self._last_takeoff_log = now
            vio_ok = self._vio.is_active if self._vio else False
            vx, vy = self._vio.get_velocity() if self._vio else (0.0, 0.0)
            Logger.info(
                f"TAKEOFF | t={elapsed:.1f}s | "
                f"throttle={self._current_throttle} PWM | "
                f"alt={alt_str} | "
                f"roll={roll_pwm} pitch={pitch_pwm} | "
                f"VIO={'ON' if vio_ok else 'OFF'} | "
                f"vel=({vx:+.3f}, {vy:+.3f}) m/s"
            )

        # Check if we reached target altitude
        if alt_m is not None:
            target = Config.HOVER_TARGET_ALT_M * Config.TAKEOFF_ALT_THRESHOLD
            if alt_m >= target:
                Logger.ok(
                    f"Takeoff complete — alt={alt_m:.2f}m "
                    f"throttle={self._current_throttle} PWM → HOVER"
                )
                self._transition(State.HOVER)

        # Timeout
        if elapsed > Config.TAKEOFF_TIMEOUT_S:
            Logger.error(
                f"Takeoff timeout after {elapsed:.0f}s — "
                f"max throttle was {self._current_throttle} PWM, "
                f"alt={alt_str}. Increase TAKEOFF_MAX_PWM in config."
            )
            self._transition(State.LAND)

    def _state_hover(self) -> None:
        now = time.time()
        if self._hover_start is None:
            self._hover_start = now

        elapsed  = now - self._hover_start
        remaining = Config.HOVER_DURATION_S - elapsed

        alt_m = self._tf02.distance_m
        if alt_m is None:
            throttle = Config.BASE_THROTTLE_PWM
            alt_err  = 0.0
        else:
            alt_err  = Config.HOVER_TARGET_ALT_M - alt_m
            throttle = int(Config.BASE_THROTTLE_PWM + Config.HOVER_KP_ALT * alt_err)
            throttle = max(Config.MIN_THROTTLE_PWM,
                           min(Config.MAX_THROTTLE_PWM, throttle))

        roll_pwm  = self._vio.roll_pwm  if self._vio else 1500
        pitch_pwm = self._vio.pitch_pwm if self._vio else 1500

        # Apply trim offsets to compensate for mechanical drift
        roll_pwm  += Config.TRIM_ROLL_PWM
        pitch_pwm += Config.TRIM_PITCH_PWM

        self._px.send_rc_override(
            throttle=throttle,
            roll=roll_pwm,
            pitch=pitch_pwm,
        )

        # Log hover status every second
        if not hasattr(self, '_last_hover_log') or now - self._last_hover_log >= 1.0:
            self._last_hover_log = now
            alt_str = f"{alt_m:.2f}m" if alt_m is not None else "---"
            vio_ok  = self._vio.is_active if self._vio else False
            Logger.info(
                f"HOVER  | t={elapsed:.0f}s | remain={remaining:.0f}s | "
                f"alt={alt_str} (err={alt_err:+.2f}m) | "
                f"throttle={throttle} PWM | "
                f"roll={roll_pwm} pitch={pitch_pwm} | "
                f"VIO={'ON' if vio_ok else 'OFF'}"
            )

        if elapsed >= Config.HOVER_DURATION_S:
            Logger.info("Hover complete → LAND")
            self._transition(State.LAND)

    def _state_land(self) -> None:
        now = time.time()
        if self._land_start is None:
            self._land_start = now
            self._current_throttle = Config.BASE_THROTTLE_PWM
            Logger.info("LAND sequence started")

        elapsed = now - self._land_start
        self._current_throttle = int(
            Config.BASE_THROTTLE_PWM
            - Config.LAND_THROTTLE_DROP_PER_S * elapsed
        )
        self._current_throttle = max(
            Config.MIN_THROTTLE_PWM, self._current_throttle
        )

        self._px.send_rc_override(
            throttle=self._current_throttle,
            roll=1500, pitch=1500,
        )

        alt_m = self._tf02.distance_m
        alt_str = f"{alt_m:.2f}m" if alt_m is not None else "---"

        # Log every second during landing
        if not hasattr(self, '_last_land_log') or now - self._last_land_log >= 1.0:
            self._last_land_log = now
            Logger.info(
                f"LAND   | t={elapsed:.1f}s | "
                f"throttle={self._current_throttle} PWM | alt={alt_str}"
            )

        if alt_m is not None and alt_m < Config.TOUCHDOWN_ALT_M:
            if elapsed > Config.LAND_DURATION_S * 0.5:
                Logger.ok(f"Touchdown detected at {alt_m:.2f}m → DISARM")
                self._transition(State.DISARM)
        elif elapsed > Config.LAND_DURATION_S:
            Logger.info("Land timer expired → DISARM")
            self._transition(State.DISARM)

    def _state_disarm(self) -> None:
        self._px.clear_rc_override()
        self._safety.set_flight_active(False)
        if self._armed:
            self._px.disarm()
        if not self._armed:
            Logger.ok("Disarmed")
            self._transition(State.DONE)
        elif time.time() - self._state_enter_time > Config.DISARM_TIMEOUT_S:
            Logger.warn("Disarm timeout")
            self._transition(State.DONE)

    def _state_done(self) -> None:
        Logger.ok("Mission complete")
        self._running = False


# ═══════════════════════════════════════════════════════════════════════════
# Scheduler
# ═══════════════════════════════════════════════════════════════════════════

class Scheduler:
    """Top-level launcher.  Creates hardware objects, starts threads, runs SM."""

    def __init__(self, mode: str) -> None:
        self._mode    = mode
        self._threads = []
        self._pixhawk: Optional[PixhawkLink] = None
        self._tf02:    Optional[TF02Reader]   = None

        # WiFi camera pipeline (replaces all UART ESP32 objects)
        self._wifi_frame:   Optional[WiFiFrameReader]   = None
        self._wifi_lk:      Optional[WiFiLKProcessor]   = None
        self._wifi_orb:     Optional[WiFiORBProcessor]  = None

        signal.signal(signal.SIGTERM, self._shutdown_handler)

    def run(self) -> int:
        try:
            return {
                "check":    self._run_check,
                "setup":    self._run_setup,
                "all":      self._run_all,
                "autotune": self._run_autotune,
                "fly":      self._run_fly,
            }[self._mode]()
        except Exception as exc:
            Logger.error(f"Scheduler fatal: {exc}")
            return 1
        finally:
            self._cleanup()

    # ── Mode Runners ───────────────────────────────────────────────────

    def _run_check(self) -> int:
        if not self._connect_pixhawk():
            return 1
        checker = HealthChecker(self._pixhawk)
        return 0 if checker.run_all() else 1

    def _run_setup(self) -> int:
        if not self._connect_pixhawk():
            return 1
        setup = AutoTuneSetup(self._pixhawk)
        return 0 if setup.run_all() else 1

    def _run_all(self) -> int:
        if not self._connect_pixhawk():
            return 1
        checker = HealthChecker(self._pixhawk)
        if not checker.run_all():
            Logger.error("Health check failed — skipping setup")
            return 1
        setup = AutoTuneSetup(self._pixhawk)
        return 0 if setup.run_all() else 1

    def _run_autotune(self) -> int:
        if not self._connect_pixhawk():
            return 1
        self._start_tf02()
        self._start_bridge()
        self._start_heartbeat()
        monitor = AutoTuneMonitor(self._pixhawk)
        monitor.run()
        return 0

    def _run_fly(self) -> int:
        """Start all threads and run the flight state machine."""
        if not self._connect_pixhawk():
            return 1

        # Start hardware readers
        self._start_tf02()
        self._start_wifi_cam()

        # Let sensors stabilise
        Logger.info("Waiting 5s for sensor + WiFi stream stabilisation …")
        time.sleep(5.0)

        # ── Sensor verification ────────────────────────────────────────
        Logger.info("Verifying sensor data streams …")

        # TF-02
        if self._tf02 and self._tf02.is_alive():
            tf02_m = self._tf02.distance_m
            if tf02_m is not None:
                Logger.ok(f"TF-02: alive, reading {tf02_m:.2f} m")
            else:
                Logger.warn("TF-02: thread alive but NO DATA — check /dev/ttyAMA0")
        else:
            Logger.error("TF-02: thread NOT alive — check wiring")

        # WiFi frame reader
        if self._wifi_frame and self._wifi_frame.is_alive():
            age = self._wifi_frame.data_age
            if age < 1.0:
                Logger.ok(
                    f"WiFi camera: alive, {self._wifi_frame.frame_count} frames, "
                    f"age={age:.2f}s"
                )
            else:
                Logger.warn(
                    f"WiFi camera: thread alive but NO FRAMES (age={age:.1f}s). "
                    f"Check ESP32-CAM power and URL: {Config.ESP32_WIFI_STREAM_URL}"
                )
        else:
            Logger.error("WiFi camera: thread NOT alive")

        # LK flow
        if self._wifi_lk and self._wifi_lk.is_alive():
            age = self._wifi_lk.data_age
            if age < 1.0:
                dx, dy, q = self._wifi_lk.get_flow()
                Logger.ok(
                    f"LK flow: alive, quality={q}, "
                    f"flow=({dx:.1f}, {dy:.1f}), age={age:.2f}s"
                )
            else:
                Logger.warn(f"LK flow: thread alive but stale (age={age:.1f}s)")
        else:
            Logger.warn("LK flow: not running — VIO will be disabled")

        # ORB flow
        if self._wifi_orb and self._wifi_orb.is_alive():
            age = self._wifi_orb.data_age
            if age < 2.0:
                Logger.ok(
                    f"ORB flow: alive, quality={self._wifi_orb.quality}, "
                    f"age={age:.2f}s"
                )
            else:
                Logger.warn(f"ORB flow: thread alive but stale (age={age:.1f}s)")
        else:
            Logger.warn("ORB flow: not running — VIO will use LK-only")

        # Safety monitor
        safety = SafetyMonitor(self._tf02, self._pixhawk)
        safety.start()
        self._threads.append(safety)

        # VIO stabilizer (dual-pipeline: LK + ORB, both over WiFi)
        vio = VIOStabilizer(
            tf02=self._tf02,
            lk_flow=self._wifi_lk,
            orb_flow=self._wifi_orb,
        )
        vio.start()
        self._threads.append(vio)
        Logger.ok("VIO stabilizer started — WiFi dual-pipeline fusion active")

        # Bridge and heartbeat
        self._start_bridge()
        self._start_heartbeat()

        # State machine
        sm = StateMachine(self._pixhawk, self._tf02, safety, vio=vio)

        # Telemetry streamer
        telem = TelemetryStreamer(sm.get_telemetry)
        telem.start()
        self._threads.append(telem)

        sm.run()
        return 0

    # ── Thread Starters ────────────────────────────────────────────────

    def _connect_pixhawk(self) -> bool:
        self._pixhawk = PixhawkLink()
        return self._pixhawk.connect()

    def _start_tf02(self) -> None:
        self._tf02 = TF02Reader()
        self._tf02.start()
        self._threads.append(self._tf02)

    def _start_bridge(self) -> None:
        bridge = RangefinderBridge(self._tf02, self._pixhawk)
        bridge.start()
        self._threads.append(bridge)

    def _start_heartbeat(self) -> None:
        hb = HeartbeatSender(self._pixhawk)
        hb.start()
        self._threads.append(hb)

    def _start_wifi_cam(self) -> None:
        """Start WiFi frame reader + LK + ORB processors.

        WiFiFrameReader auto-discovers the ESP32-CAM on each connect
        attempt (tries candidate IPs, then scans subnets).  All three
        processors share a single HTTP connection.
        """
        Logger.info("Starting WiFi camera pipeline — auto-discovery enabled")
        try:
            # One connection — auto-discovers ESP32-CAM IP
            self._wifi_frame = WiFiFrameReader()  # no URL = auto-discover
            self._wifi_frame.start()
            self._threads.append(self._wifi_frame)
            Logger.ok("WiFiFrameReader started")

            # LK optical flow (weight 0.4)
            self._wifi_lk = WiFiLKProcessor(self._wifi_frame)
            self._wifi_lk.start()
            self._threads.append(self._wifi_lk)
            Logger.ok("WiFiLKProcessor started")

            # ORB optical flow (weight 0.6)
            self._wifi_orb = WiFiORBProcessor(self._wifi_frame)
            self._wifi_orb.start()
            self._threads.append(self._wifi_orb)
            Logger.ok("WiFiORBProcessor started")

            # White board detector (colour-based pad detection)
            self._white_detector = WhiteDetector(self._wifi_frame)
            self._white_detector.start()
            self._threads.append(self._white_detector)
            Logger.ok("WhiteDetector started")

        except Exception as exc:
            Logger.error(f"WiFi camera pipeline failed: {exc}")
            Logger.warn("VIO will be INACTIVE — drone will hover without X-Y hold")
            self._wifi_frame = None
            self._wifi_lk    = None
            self._wifi_orb   = None

    # ── Cleanup ────────────────────────────────────────────────────────

    def _cleanup(self) -> None:
        Logger.info("Cleaning up …")
        if self._pixhawk and self._pixhawk.connected:
            try:
                self._pixhawk.clear_rc_override()
                self._pixhawk.disarm()
                Logger.warn("Safety disarm sent during cleanup")
            except Exception:
                pass
        for t in self._threads:
            if hasattr(t, "stop"):
                t.stop()
        for t in self._threads:
            t.join(timeout=2.0)
        if self._pixhawk:
            self._pixhawk.close()
        Logger.ok("Shutdown complete")

    def _shutdown_handler(self, signum: int, frame: object) -> None:
        if self._pixhawk and self._pixhawk.connected:
            try:
                self._pixhawk.clear_rc_override()
                self._pixhawk.disarm()
            except Exception:
                pass
        raise KeyboardInterrupt


def main() -> None:
    """CLI entry point: python3 -m ascend --mode <mode>"""
    parser = argparse.ArgumentParser(
        prog="ascend",
        description="ASCEND — Autonomous drone flight system (IRoC-U 2026)",
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=["check", "setup", "all", "autotune", "fly"],
        help="Operating mode: check | setup | all | autotune | fly",
    )
    args = parser.parse_args()
    scheduler = Scheduler(mode=args.mode)
    sys.exit(scheduler.run())


if __name__ == "__main__":
    main()
