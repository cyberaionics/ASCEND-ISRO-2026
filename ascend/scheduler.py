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
from .hardware.esp32_cam import ESP32CamReader
from .hardware.esp32_cam_frame import ESP32FrameReader, ORBFlowProcessor
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
# State Machine
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

        # Timers
        self._state_enter_time: float = 0.0
        self._hover_start: Optional[float] = None
        self._land_start: Optional[float] = None

        # Cached MAVLink telemetry
        self._armed: bool = False
        self._mode: int = 0
        self._mode_name: str = "UNKNOWN"
        self._battery_volt: float = 0.0
        self._battery_pct: int = 0

        # Flight record for mission summary
        self._flight_start: float = 0.0
        self._min_alt: float = float("inf")
        self._max_alt: float = 0.0
        self._vio_active_ticks: int = 0
        self._total_ticks: int = 0

        # Takeoff ramp state
        self._ramp_throttle: int = Config.TAKEOFF_START_PWM
        self._current_throttle: int = Config.MIN_THROTTLE_PWM

    # ── Public API ─────────────────────────────────────────────────────

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
            telem["vio_active"] = active
            telem["vio_roll_pwm"] = roll
            telem["vio_pitch_pwm"] = pitch
        else:
            telem["vio_active"] = False
            telem["vio_roll_pwm"] = 1500
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

                # Safety monitor check (any state except DONE)
                if self._state not in (State.DONE, State.LAND, State.DISARM):
                    if self._safety.emergency_flag:
                        Logger.error(
                            f"Safety trigger: {self._safety.emergency_reason}"
                        )
                        self._transition(State.LAND)

                # State dispatch
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
        """Signal the state machine to stop."""
        self._running = False

    # ── MAVLink Polling ────────────────────────────────────────────────

    def _poll_mavlink(self) -> None:
        """Read all pending MAVLink messages and cache relevant data."""
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
                self._battery_pct = max(0, min(100, msg.battery_remaining))

            elif mtype == "LOCAL_POSITION_NED":
                self._safety.update_position(msg.x, msg.y)

            elif mtype == "STATUSTEXT":
                Logger.warn(f"FC: {msg.text}")

    # ── State Transition ───────────────────────────────────────────────

    def _transition(self, new_state: State) -> None:
        """Perform a state transition with logging."""
        old = self._state
        self._state = new_state
        self._state_enter_time = time.time()

        # Activate safety monitoring during flight phases
        if new_state in (State.TAKEOFF, State.HOVER):
            self._safety.set_flight_active(True)
        elif new_state in (State.IDLE, State.LAND, State.DISARM, State.DONE):
            self._safety.set_flight_active(False)

        Logger.ok(f"STATE: {old.value} → {new_state.value}")

    # ── VIO Helper ─────────────────────────────────────────────────────

    def _get_vio_roll_pitch(self) -> tuple:
        """Return (roll_pwm, pitch_pwm) from VIO or neutral."""
        if self._vio is not None:
            roll, pitch, active = self._vio.get_corrections()
            if active:
                self._vio_active_ticks += 1
            self._total_ticks += 1
            return (
                max(Config.RC_PWM_MIN, min(Config.RC_PWM_MAX, roll)),
                max(Config.RC_PWM_MIN, min(Config.RC_PWM_MAX, pitch)),
            )
        self._total_ticks += 1
        return (1500, 1500)

    # ── State Handlers ─────────────────────────────────────────────────

    def _state_idle(self) -> None:
        """IDLE — verify threads alive, GPS-denied confirmed, auto-advance."""
        Logger.info("IDLE: verifying system ready …")
        # Quick sanity: TF02 thread alive
        if self._tf02 is not None and self._tf02.is_alive():
            Logger.ok("TF02Reader thread alive")
        else:
            Logger.warn("TF02Reader thread NOT alive — proceeding anyway")
        self._transition(State.PREFLIGHT)

    def _state_preflight(self) -> None:
        """PREFLIGHT — run health checks.  On pass → ARM.  On fail → abort."""
        Logger.info("PREFLIGHT: running health checks …")

        # Check 1: Pixhawk heartbeat present
        self._poll_mavlink()
        if not self._px.connected:
            Logger.error("No Pixhawk heartbeat — cannot fly")
            self._running = False
            return

        # Check 2: TF-02 reading
        tf02_m = self._tf02.distance_m
        if tf02_m is None:
            Logger.warn("TF-02 no reading yet — waiting 2s …")
            time.sleep(2.0)
            tf02_m = self._tf02.distance_m
            if tf02_m is None:
                Logger.error("TF-02 still no data — abort")
                self._running = False
                return
        Logger.ok(f"TF-02 reading: {tf02_m:.2f} m")

        # Check 3: Battery voltage
        if 0 < self._battery_volt < Config.BATT_CRITICAL_VOLT:
            Logger.error(
                f"Battery CRITICAL: {self._battery_volt:.2f}V — abort"
            )
            self._running = False
            return

        Logger.ok("Preflight checks passed")
        self._transition(State.ARM)

    def _state_arm(self) -> None:
        """ARM — set STABILIZE mode, send arm command, wait for confirmation.

        Retries the arm command every 2 seconds because ArduCopter may
        reject the first attempt if the mode switch hasn't settled or
        pre-arm checks are still running.
        """
        elapsed = time.time() - self._state_enter_time

        # Phase 1 (first 0.1s): send STABILIZE mode command
        if elapsed < 0.1:
            Logger.info("ARM: setting STABILIZE mode …")
            self._px.set_mode("STABILIZE")
            return

        # Phase 2 (0.1–1.0s): wait for mode to settle, poll for confirmation
        if elapsed < 1.0:
            self._poll_mavlink()
            return

        # Phase 3: verify mode is STABILIZE before arming
        self._poll_mavlink()

        if self._armed:
            Logger.ok("Drone ARMED — STABILIZE mode confirmed")
            self._flight_start = time.time()
            self._transition(State.TAKEOFF)
            return

        # Retry arm command every 2 seconds
        time_since_entry = elapsed
        retry_interval = 2.0
        should_send_arm = (
            time_since_entry < 1.0 + 0.1  # first arm at ~1s
            or int(time_since_entry / retry_interval) != int(
                (time_since_entry - Config.MAIN_LOOP_INTERVAL) / retry_interval
            )
        )
        if should_send_arm:
            if self._mode_name != "STABILIZE":
                Logger.warn(
                    f"ARM: mode is {self._mode_name}, resending STABILIZE …"
                )
                self._px.set_mode("STABILIZE")
                time.sleep(0.3)
            Logger.info(f"ARM: sending arm command (attempt at {elapsed:.1f}s) …")
            self._px.arm()

        # Timeout
        if elapsed > Config.ARM_TIMEOUT_S:
            Logger.error(
                f"Arming FAILED within {Config.ARM_TIMEOUT_S:.0f}s — "
                "check Mission Planner HUD for pre-arm failures. "
                "Transitioning to DISARM"
            )
            self._transition(State.DISARM)

    def _state_takeoff(self) -> None:
        """TAKEOFF — 2-phase throttle strategy in STABILIZE mode.

        Phase 1 (ramp): Increase throttle from TAKEOFF_START_PWM at
            TAKEOFF_RAMP_PWM_PER_S until the drone lifts off (alt > 0.15m).
        Phase 2 (climb): Switch to altitude P-controller to climb smoothly
            to HOVER_TARGET_ALT_M.

        Transition to HOVER when altitude >= TAKEOFF_ALT_THRESHOLD.
        """
        elapsed = time.time() - self._state_enter_time
        alt_m = self._tf02.distance_m

        if alt_m is None:
            Logger.warn("TAKEOFF: TF02 no data — holding throttle")
            roll, pitch = self._get_vio_roll_pitch()
            self._px.send_rc_override(
                roll=roll, pitch=pitch,
                throttle=self._ramp_throttle
            )
            return

        # Track altitude stats
        self._min_alt = min(self._min_alt, alt_m)
        self._max_alt = max(self._max_alt, alt_m)

        # Timeout check
        if elapsed > Config.TAKEOFF_TIMEOUT_S:
            Logger.error(
                f"TAKEOFF timeout ({Config.TAKEOFF_TIMEOUT_S:.0f}s) — "
                f"max alt reached: {self._max_alt:.2f}m — LAND"
            )
            self._transition(State.LAND)
            return

        # Altitude reached → HOVER
        if alt_m >= Config.TAKEOFF_ALT_THRESHOLD:
            Logger.ok(
                f"Altitude {alt_m:.2f}m >= "
                f"{Config.TAKEOFF_ALT_THRESHOLD}m → HOVER"
            )
            self._current_throttle = self._ramp_throttle
            self._transition(State.HOVER)
            return

        # ── Phase 1: Ramp until airborne (alt > 0.15m) ────────────────
        if alt_m < 0.15:
            self._ramp_throttle = min(
                Config.TAKEOFF_START_PWM + int(
                    elapsed * Config.TAKEOFF_RAMP_PWM_PER_S
                ),
                Config.MAX_THROTTLE_PWM,
            )

        # ── Phase 2: Altitude P-controller for climb ──────────────────
        else:
            error = Config.HOVER_TARGET_ALT_M - alt_m
            target_thr = int(
                Config.BASE_THROTTLE_PWM + Config.HOVER_KP_ALT * error
            )
            target_thr = max(
                Config.MIN_THROTTLE_PWM,
                min(Config.MAX_THROTTLE_PWM, target_thr)
            )

            # Smooth transition: ramp toward target at limited rate
            step = max(1, int(Config.TAKEOFF_RAMP_PWM_PER_S
                              * Config.MAIN_LOOP_INTERVAL))
            if self._ramp_throttle < target_thr:
                self._ramp_throttle = min(
                    self._ramp_throttle + step, target_thr
                )
            elif self._ramp_throttle > target_thr:
                self._ramp_throttle = max(
                    self._ramp_throttle - step, target_thr
                )

        roll, pitch = self._get_vio_roll_pitch()
        self._px.send_rc_override(
            roll=roll, pitch=pitch,
            throttle=self._ramp_throttle
        )

        # 1 Hz logging (reduce log spam)
        if int(elapsed * 2) != int((elapsed - Config.MAIN_LOOP_INTERVAL) * 2):
            Logger.info(
                f"TAKEOFF alt={alt_m:.2f}m throttle={self._ramp_throttle} "
                f"phase={'ramp' if alt_m < 0.15 else 'climb'} "
                f"elapsed={elapsed:.1f}s"
            )

    def _state_hover(self) -> None:
        """HOVER — altitude P-controller + VIO roll/pitch corrections.

        Throttle = BASE_THROTTLE + Kp_alt × (target_alt - current_alt)
        Duration: HOVER_DURATION_S seconds.
        """
        if self._hover_start is None:
            self._hover_start = time.time()
            if self._vio is not None:
                self._vio.reset()
            Logger.info(f"Hovering for {Config.HOVER_DURATION_S:.0f}s …")

        alt_m = self._tf02.distance_m
        if alt_m is None:
            roll, pitch = self._get_vio_roll_pitch()
            self._px.send_rc_override(roll=roll, pitch=pitch, throttle=1500)
            return

        # Track altitude stats
        self._min_alt = min(self._min_alt, alt_m)
        self._max_alt = max(self._max_alt, alt_m)

        # Altitude P-controller
        error = Config.HOVER_TARGET_ALT_M - alt_m
        throttle = int(Config.BASE_THROTTLE_PWM + Config.HOVER_KP_ALT * error)
        throttle = max(Config.MIN_THROTTLE_PWM, min(Config.MAX_THROTTLE_PWM, throttle))

        # VIO corrections
        roll, pitch = self._get_vio_roll_pitch()

        # Apply RC override
        self._px.send_rc_override(
            roll=roll, pitch=pitch, throttle=throttle
        )

        elapsed = time.time() - self._hover_start

        # 1 Hz logging
        if int(elapsed) != int(elapsed - Config.MAIN_LOOP_INTERVAL):
            vio_tag = ""
            if self._vio is not None and self._vio.is_active:
                vio_r, vio_p, _ = self._vio.get_corrections()
                vio_tag = f" vio_r={vio_r} vio_p={vio_p}"
            Logger.info(
                f"HOVER alt={alt_m:.2f}m err={error:+.2f}m "
                f"thr={throttle}{vio_tag} t={elapsed:.0f}s"
            )

        # Battery check
        if 0 < self._battery_volt < Config.BATT_LOW_VOLT:
            Logger.warn(
                f"Low battery ({self._battery_volt:.2f}V) → LAND"
            )
            self._hover_start = None
            self._transition(State.LAND)
            return

        # Duration complete
        if elapsed >= Config.HOVER_DURATION_S:
            Logger.ok(f"Hover complete ({elapsed:.0f}s) → LAND")
            self._hover_start = None
            self._transition(State.LAND)

    def _state_land(self) -> None:
        """LAND — controlled throttle reduction from current throttle.

        Decreases throttle at LAND_THROTTLE_DROP_PER_S until touchdown
        (alt < 0.10m) or throttle reaches MIN.  Does NOT clear RC override
        on entry — maintains control throughout descent.
        """
        if self._land_start is None:
            self._land_start = time.time()
            # Use actual current throttle, not hardcoded BASE
            self._land_start_thr = max(
                self._ramp_throttle, Config.BASE_THROTTLE_PWM
            )
            Logger.info(
                f"LAND: descending from throttle={self._land_start_thr}, "
                f"drop rate={Config.LAND_THROTTLE_DROP_PER_S} PWM/s"
            )

        elapsed = time.time() - self._land_start
        alt_m = self._tf02.distance_m

        # Linear throttle reduction from actual current throttle
        drop = int(elapsed * Config.LAND_THROTTLE_DROP_PER_S)
        throttle = max(Config.MIN_THROTTLE_PWM, self._land_start_thr - drop)

        # Track altitude stats
        if alt_m is not None:
            self._min_alt = min(self._min_alt, alt_m)
            self._max_alt = max(self._max_alt, alt_m)

        self._px.send_rc_override(throttle=throttle)

        # Check touchdown conditions
        touchdown = False
        if alt_m is not None and alt_m < Config.TOUCHDOWN_ALT_M:
            Logger.ok(f"Touchdown detected (alt={alt_m:.2f}m) — cutting motors")
            touchdown = True
        elif throttle <= Config.MIN_THROTTLE_PWM:
            Logger.ok("Throttle at minimum — cutting motors")
            touchdown = True

        if touchdown:
            self._px.clear_rc_override()
            self._land_start = None
            self._transition(State.DISARM)
        else:
            # 2 Hz logging
            if int(elapsed * 2) != int(
                (elapsed - Config.MAIN_LOOP_INTERVAL) * 2
            ):
                Logger.info(
                    f"LAND alt={alt_m:.2f}m thr={throttle} "
                    f"elapsed={elapsed:.1f}s"
                ) if alt_m is not None else None

    def _state_disarm(self) -> None:
        """DISARM — send disarm command, wait for confirmation."""
        elapsed = time.time() - self._state_enter_time

        if elapsed < 0.1:
            Logger.info("DISARM: sending disarm command …")
            self._px.clear_rc_override()
            self._px.disarm()
            return

        self._poll_mavlink()
        if not self._armed:
            Logger.ok("Drone DISARMED")
            self._transition(State.DONE)
            return

        if elapsed > Config.DISARM_TIMEOUT_S:
            Logger.warn("Disarm timeout — forcing disarm")
            self._px.disarm()
            self._transition(State.DONE)

    def _state_done(self) -> None:
        """DONE — log mission summary, stop state machine."""
        flight_time = time.time() - self._flight_start if self._flight_start else 0
        vio_pct = (
            (self._vio_active_ticks / self._total_ticks * 100)
            if self._total_ticks > 0 else 0
        )

        Logger.header("MISSION SUMMARY")
        Logger.kv("Flight time", f"{flight_time:.1f} s")
        Logger.kv("Min altitude", f"{self._min_alt:.2f} m"
                   if self._min_alt != float("inf") else "N/A")
        Logger.kv("Max altitude", f"{self._max_alt:.2f} m")
        Logger.kv("VIO active", f"{vio_pct:.1f}%")
        Logger.kv("Final battery", f"{self._battery_volt:.2f} V")
        Logger.rule()

        self._running = False

    # ── Utility ────────────────────────────────────────────────────────

    @staticmethod
    def _resolve_mode(mode_id: int) -> str:
        """Return ArduCopter mode name for a mode number."""
        for name, mid in Config.MODE_MAP.items():
            if mid == mode_id:
                return name
        return f"MODE_{mode_id}"


# ═══════════════════════════════════════════════════════════════════════════
# Scheduler (CLI Entry Point)
# ═══════════════════════════════════════════════════════════════════════════

class Scheduler:
    """CLI entry point — instantiates shared objects and routes to modes.

    Modes:
        check    — run pre-flight health checks
        setup    — write Pixhawk parameters
        autotune — passive AutoTune monitor
        fly      — full autonomous flight (state machine)
    """

    def __init__(self, mode: str) -> None:
        self._mode = mode.lower()
        self._threads: list = []
        self._tf02: Optional[TF02Reader] = None
        self._esp32_cam: Optional[ESP32CamReader] = None
        self._frame_reader: Optional[ESP32FrameReader] = None
        self._orb_flow: Optional[ORBFlowProcessor] = None
        self._pixhawk: Optional[PixhawkLink] = None

    def run(self) -> int:
        """Execute the selected mode.  Returns exit code."""
        Logger.banner()
        Logger.info(f"Mode: {self._mode}")
        Logger.rule()

        signal.signal(signal.SIGINT, self._shutdown_handler)
        signal.signal(signal.SIGTERM, self._shutdown_handler)

        try:
            if self._mode == "check":
                return self._run_check()
            elif self._mode == "setup":
                return self._run_setup()
            elif self._mode == "all":
                return self._run_all()
            elif self._mode == "autotune":
                return self._run_autotune()
            elif self._mode == "fly":
                return self._run_fly()
            else:
                Logger.error(f"Unknown mode: {self._mode}")
                return 1
        except KeyboardInterrupt:
            Logger.warn("Interrupted — shutting down")
            return 0
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
        self._start_esp32_cam()
        self._start_frame_reader()

        # Let sensors stabilise
        Logger.info("Waiting 3s for sensor stabilisation …")
        time.sleep(3.0)

        # ── Sensor data verification ──────────────────────────────────
        Logger.info("Verifying sensor data streams …")

        # TF-02
        if self._tf02.is_alive():
            tf02_m = self._tf02.distance_m
            if tf02_m is not None:
                Logger.ok(f"TF-02: alive, reading {tf02_m:.2f} m")
            else:
                Logger.warn("TF-02: thread alive but NO DATA yet")
        else:
            Logger.error("TF-02: thread NOT alive — check /dev/ttyAMA0")

        # ESP32-CAM flow
        if self._esp32_cam is not None:
            if self._esp32_cam.is_alive():
                if self._esp32_cam.data_age < 1.0:
                    dx, dy, q = self._esp32_cam.get_flow()
                    Logger.ok(f"ESP32-CAM: alive, quality={q}, "
                              f"flow=({dx},{dy}), age={self._esp32_cam.data_age:.2f}s")
                else:
                    Logger.warn(
                        f"ESP32-CAM: thread alive but data stale "
                        f"(age={self._esp32_cam.data_age:.1f}s) — "
                        "check UART2 wiring and ESP32 firmware"
                    )
            else:
                Logger.error(
                    "ESP32-CAM: thread DIED — failed to open /dev/ttyAMA2. "
                    "VIO ESP32 pipeline will be inactive."
                )
        else:
            Logger.warn("ESP32-CAM: not started (init failed)")

        # ORB flow
        if self._orb_flow is not None:
            if self._frame_reader is not None and self._frame_reader.is_alive():
                if self._orb_flow.is_alive():
                    if self._orb_flow.data_age < 2.0:
                        Logger.ok(f"ORB flow: alive, quality={self._orb_flow.quality}")
                    else:
                        Logger.warn(
                            "ORB flow: thread alive but data stale — "
                            "check UART3 wiring (ttyAMA3 @ 921600)"
                        )
                else:
                    Logger.warn("ORB flow: thread DIED")
            else:
                Logger.warn("ORB flow: frame reader not alive — no raw frames")
        else:
            Logger.warn("ORB flow: not started — VIO will run ESP32-only")

        # Safety monitor
        safety = SafetyMonitor(self._tf02, self._pixhawk)
        safety.start()
        self._threads.append(safety)

        # VIO stabilizer with dual pipelines
        vio = VIOStabilizer(
            esp32_cam=self._esp32_cam,
            tf02=self._tf02,
            cv_flow=self._orb_flow,
        )
        vio.start()
        self._threads.append(vio)
        Logger.ok("VIO stabilizer started — dual-pipeline fusion active")

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

    def _start_esp32_cam(self) -> None:
        """Start the ESP32-CAM optical-flow reader (UART0, ttyAMA2)."""
        try:
            self._esp32_cam = ESP32CamReader()
            self._esp32_cam.start()
            self._threads.append(self._esp32_cam)
            Logger.ok("ESP32-CAM flow reader started (ttyAMA2)")
        except Exception as exc:
            Logger.warn(f"ESP32-CAM flow reader failed: {exc}")
            self._esp32_cam = None

    def _start_frame_reader(self) -> None:
        """Start ESP32 frame reader + ORB flow processor (UART2, ttyAMA3)."""
        try:
            self._frame_reader = ESP32FrameReader()
            self._frame_reader.start()
            self._threads.append(self._frame_reader)
            Logger.ok("ESP32 frame reader started (ttyAMA3)")

            self._orb_flow = ORBFlowProcessor(self._frame_reader)
            self._orb_flow.start()
            self._threads.append(self._orb_flow)
            Logger.ok("ORB flow processor started")
        except Exception as exc:
            Logger.warn(
                f"ESP32 frame/ORB pipeline failed: {exc} — "
                "VIO will run ESP32-only"
            )
            self._frame_reader = None
            self._orb_flow = None

    # ── Cleanup ────────────────────────────────────────────────────────

    def _cleanup(self) -> None:
        """Stop all threads and close connections."""
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
        """Best-effort disarm before raising KeyboardInterrupt."""
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
    code = scheduler.run()
    sys.exit(code)


if __name__ == "__main__":
    main()
