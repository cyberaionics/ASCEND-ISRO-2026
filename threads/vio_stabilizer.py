"""
ASCEND — Mission Controller (GUIDED_NOGPS State Machine)
Deterministic state machine for high-precision indoor flight:

  WAIT_FOR_EKF → ARM → TAKEOFF → HOVER_60S → LAND

Architecture:
  The SensorFusionBridge feeds ODOMETRY → EKF3.  This module commands
  the flight controller using SET_POSITION_TARGET_LOCAL_NED and
  MAV_CMD_NAV_LAND.  A Python-side PI loop monitors drift during hover
  and sends velocity corrections.

Coordinate frames:
  - Position targets: MAV_FRAME_LOCAL_NED (Z negative = up)
  - Velocity feedback: MAV_FRAME_BODY_FRD

Supersedes the old VIOStabilizer (dual-pipeline PID → attitude target).
"""

import enum
import threading
import time
from typing import Optional

from pymavlink import mavutil

from ..config import Config
from ..logger import Logger
from ..hardware.tf02 import TF02Reader
from ..hardware.pixhawk import PixhawkLink


# ═══════════════════════════════════════════════════════════════════════════
# Flight State Enum
# ═══════════════════════════════════════════════════════════════════════════

class MissionState(enum.Enum):
    """Mission controller states."""
    WAIT_FOR_EKF = "WAIT_FOR_EKF"
    ARM          = "ARM"
    TAKEOFF      = "TAKEOFF"
    HOVER_60S    = "HOVER_60S"
    LAND         = "LAND"
    DONE         = "DONE"


# ═══════════════════════════════════════════════════════════════════════════
# Mission Controller
# ═══════════════════════════════════════════════════════════════════════════

class MissionController:
    """Deterministic state machine for indoor GUIDED_NOGPS flight.

    State transitions:
        WAIT_FOR_EKF → ARM → TAKEOFF → HOVER_60S → LAND → DONE

    During HOVER_60S, a Python-side PI loop monitors position drift
    and sends SET_POSITION_TARGET_LOCAL_NED commands with velocity
    corrections targeting (X:0, Y:0, Z:-1.0) in NED coordinates.

    After exactly 60 seconds, commands MAV_CMD_NAV_LAND at current X, Y.

    Args:
        pixhawk:       Connected PixhawkLink instance.
        tf02:          Running TF02Reader instance.
        safety_flag:   Callable returning True if safety emergency detected.
    """

    def __init__(self, pixhawk: PixhawkLink, tf02: TF02Reader,
                 safety_flag=None) -> None:
        self._px = pixhawk
        self._tf02 = tf02
        self._safety_flag = safety_flag  # callable → bool

        self._state = MissionState.WAIT_FOR_EKF
        self._state_enter_time: float = 0.0
        self._running: bool = True

        # ── ARM state ─────────────────────────────────────────────────
        self._arm_sent: bool = False
        self._armed: bool = False
        self._mode: int = 0
        self._mode_name: str = "UNKNOWN"

        # ── HOVER PI controller state ─────────────────────────────────
        self._hover_start: Optional[float] = None
        self._integral_x: float = 0.0
        self._integral_y: float = 0.0
        self._last_pi_time: float = 0.0

        # ── Position tracking from LOCAL_POSITION_NED ─────────────────
        self._local_pos_x: float = 0.0
        self._local_pos_y: float = 0.0
        self._local_pos_z: float = 0.0
        self._pos_lock = threading.Lock()

        # ── Battery ───────────────────────────────────────────────────
        self._battery_volt: float = 0.0
        self._battery_pct: int = 0

        # ── State Timers ──────────────────────────────────────────────
        self._ground_touch_start: Optional[float] = None

        # ── Diagnostics ───────────────────────────────────────────────
        self._last_log: float = 0.0

    # ── Public API ─────────────────────────────────────────────────────

    @property
    def state(self) -> MissionState:
        """Current state of the mission controller."""
        return self._state

    def stop(self) -> None:
        """Signal the controller to stop."""
        self._running = False

    def get_local_position(self) -> tuple:
        """Return ``(x, y, z)`` from latest LOCAL_POSITION_NED."""
        with self._pos_lock:
            return (self._local_pos_x, self._local_pos_y, self._local_pos_z)

    # ── State Transition ───────────────────────────────────────────────

    def _transition(self, new_state: MissionState) -> None:
        old = self._state
        self._state = new_state
        self._state_enter_time = time.time()
        Logger.info(f"Mission: {old.value} → {new_state.value}")

    # ── MAVLink Polling ────────────────────────────────────────────────

    def _poll_mavlink(self) -> None:
        """Drain inbound MAVLink messages, update internal state."""
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
                with self._pos_lock:
                    self._local_pos_x = msg.x
                    self._local_pos_y = msg.y
                    self._local_pos_z = msg.z

            elif mtype == "STATUSTEXT":
                text = msg.text if isinstance(msg.text, str) else msg.text.decode("utf-8", errors="replace")
                Logger.warn(f"FC: {text}")

    def _resolve_mode(self, custom_mode: int) -> str:
        for name, num in Config.MODE_MAP.items():
            if num == custom_mode:
                return name
        return f"MODE_{custom_mode}"

    # ── Main Loop ──────────────────────────────────────────────────────

    def run(self) -> None:
        """Execute the mission state machine."""
        Logger.header("ASCEND MISSION CONTROLLER — GUIDED_NOGPS")
        Logger.info(f"Target position : ({Config.HOVER_TARGET_X}, "
                     f"{Config.HOVER_TARGET_Y}, {Config.HOVER_TARGET_Z}) NED")
        Logger.info(f"Hover duration  : {Config.HOVER_DURATION_S}s")
        Logger.info(f"PI gains        : P={Config.POS_HOLD_KP}, "
                     f"I={Config.POS_HOLD_KI}")
        Logger.info("Press Ctrl+C to abort → emergency LAND\n")

        self._px.request_data_streams(rate_hz=10)
        self._transition(MissionState.WAIT_FOR_EKF)

        try:
            while self._running:
                self._poll_mavlink()

                # Safety check — trigger LAND from any active state
                if self._state not in (MissionState.LAND, MissionState.DONE):
                    if self._safety_flag and self._safety_flag():
                        Logger.error("⚠ Safety trigger — entering LAND")
                        self._transition(MissionState.LAND)

                handler = {
                    MissionState.WAIT_FOR_EKF: self._state_wait_for_ekf,
                    MissionState.ARM:          self._state_arm,
                    MissionState.TAKEOFF:      self._state_takeoff,
                    MissionState.HOVER_60S:    self._state_hover_60s,
                    MissionState.LAND:         self._state_land,
                    MissionState.DONE:         self._state_done,
                }.get(self._state)

                if handler:
                    handler()

                time.sleep(Config.MAIN_LOOP_INTERVAL)

        except KeyboardInterrupt:
            Logger.warn("Ctrl+C — emergency LAND + DISARM")
            self._px.set_mode("LAND")
            time.sleep(0.5)
            self._px.disarm()
        finally:
            self._running = False
            Logger.info("MissionController loop exited")

    # ── WAIT_FOR_EKF ───────────────────────────────────────────────────

    def _state_wait_for_ekf(self) -> None:
        """Wait until EKF3 has converged after receiving ODOMETRY data.

        Checks EKF_STATUS_REPORT flags for velocity and horizontal
        position estimate quality.  Times out after EKF_INIT_TIMEOUT.
        """
        elapsed = time.time() - self._state_enter_time

        # Poll for EKF status
        msg = self._px.recv("EKF_STATUS_REPORT", timeout=0.5)
        if msg is not None:
            # flags is a bitmask — check velocity and horizontal position
            # EKF_ATTITUDE=1, EKF_VELOCITY_HORIZ=2, EKF_VELOCITY_VERT=4,
            # EKF_POS_HORIZ_REL=8, EKF_POS_HORIZ_ABS=16, EKF_POS_VERT_ABS=32
            vel_ok = bool(msg.flags & 0x02)    # VELOCITY_HORIZ
            pos_ok = bool(msg.flags & 0x08)    # POS_HORIZ_REL
            att_ok = bool(msg.flags & 0x01)    # ATTITUDE

            if elapsed > 3.0 and att_ok:
                # After 3 seconds with attitude, try to proceed even
                # without full EKF convergence (indoor may not converge
                # pos until motion starts)
                if vel_ok or pos_ok:
                    Logger.ok(
                        f"EKF converged (flags=0x{msg.flags:04X}) "
                        f"after {elapsed:.1f}s → ARM"
                    )
                    self._transition(MissionState.ARM)
                    return
                elif elapsed > 8.0:
                    # Proceed with attitude-only after 8 seconds
                    Logger.warn(
                        f"EKF partial convergence (flags=0x{msg.flags:04X}) "
                        f"at {elapsed:.1f}s — proceeding with attitude-only"
                    )
                    self._transition(MissionState.ARM)
                    return

        # Periodic log
        now = time.time()
        if now - self._last_log >= 2.0:
            self._last_log = now
            Logger.info(f"WAIT_FOR_EKF | t={elapsed:.0f}s | "
                         f"waiting for EKF3 convergence …")

        # Timeout
        if elapsed > Config.EKF_INIT_TIMEOUT:
            Logger.warn(
                f"EKF convergence timeout after {elapsed:.0f}s — "
                f"proceeding anyway (indoor EKF may converge after motion)"
            )
            self._transition(MissionState.ARM)

    # ── ARM ────────────────────────────────────────────────────────────

    def _state_arm(self) -> None:
        """Set GUIDED_NOGPS mode and arm the motors."""
        elapsed = time.time() - self._state_enter_time

        # Step 1: Set mode (once)
        if elapsed < 0.1:
            Logger.info("Setting GUIDED_NOGPS mode …")
            self._px.set_mode("GUIDED_NOGPS")

        # Step 2: Send arm command (once after mode is set)
        if elapsed > 0.5 and not self._arm_sent:
            Logger.info("Sending ARM command …")
            self._px.arm()
            self._arm_sent = True

        # Step 3: Check for armed confirmation
        if self._armed:
            Logger.ok("Armed successfully ✓")
            self._arm_sent = False
            self._transition(MissionState.TAKEOFF)
        elif elapsed > Config.ARM_TIMEOUT_S:
            Logger.error(
                f"ARM timeout after {elapsed:.1f}s — "
                f"check pre-arm conditions on FC"
            )
            self._arm_sent = False
            self._transition(MissionState.DONE)

    # ── TAKEOFF ────────────────────────────────────────────────────────

    def _state_takeoff(self) -> None:
        """Command position target at Z=-1.0m (1m up in NED).

        Sends SET_POSITION_TARGET_LOCAL_NED continuously until the
        TF02 lidar reads ≥ 0.9m altitude.
        """
        elapsed = time.time() - self._state_enter_time

        # Command: go to (0, 0, -1.0) in NED
        self._px.send_position_target_local_ned_full(
            x=Config.HOVER_TARGET_X,
            y=Config.HOVER_TARGET_Y,
            z=Config.HOVER_TARGET_Z,
        )

        alt_m = self._tf02.distance_m
        alt_str = f"{alt_m:.2f}m" if alt_m is not None else "---"

        # Log every second
        now = time.time()
        if now - self._last_log >= 1.0:
            self._last_log = now
            Logger.info(
                f"TAKEOFF | t={elapsed:.1f}s | alt={alt_str} | "
                f"target Z={Config.HOVER_TARGET_Z}m (NED)"
            )

        # Check altitude threshold (1.0m target, pass at 0.9m)
        if alt_m is not None and alt_m >= abs(Config.HOVER_TARGET_Z) * 0.9:
            Logger.ok(f"Takeoff complete — alt={alt_m:.2f}m → HOVER_60S")
            self._transition(MissionState.HOVER_60S)

        # Timeout
        if elapsed > 30.0:
            Logger.error(
                f"Takeoff timeout after {elapsed:.0f}s — alt={alt_str}"
            )
            self._transition(MissionState.LAND)

    # ── HOVER_60S ──────────────────────────────────────────────────────

    def _state_hover_60s(self) -> None:
        """Precision hover at (0, 0, -1.0) for exactly 60 seconds.

        Implements a Python-side PI loop:
          err_x = HOVER_TARGET_X - local_pos_x
          err_y = HOVER_TARGET_Y - local_pos_y
          integral += err * dt
          cmd_v = P * err + I * integral

        Sends SET_POSITION_TARGET_LOCAL_NED with both position AND
        velocity correction at 20 Hz (main loop rate).
        """
        now = time.time()

        # Initialize hover timer
        if self._hover_start is None:
            self._hover_start = now
            self._integral_x = 0.0
            self._integral_y = 0.0
            self._last_pi_time = now
            Logger.info("HOVER_60S started — PI position hold active")

        elapsed = now - self._hover_start
        remaining = Config.HOVER_DURATION_S - elapsed

        # ── PI computation ────────────────────────────────────────────
        with self._pos_lock:
            pos_x = self._local_pos_x
            pos_y = self._local_pos_y

        alt_m = self._tf02.distance_m
        cmd_vx = 0.0
        cmd_vy = 0.0

        altitude_achieved = alt_m is not None and alt_m >= abs(Config.HOVER_TARGET_Z) * 0.95

        if altitude_achieved:
            # Position error
            err_x = Config.HOVER_TARGET_X - pos_x
            err_y = Config.HOVER_TARGET_Y - pos_y

            # Time step for integral
            dt = now - self._last_pi_time
            self._last_pi_time = now
            if dt <= 0 or dt > 0.5:
                dt = Config.MAIN_LOOP_INTERVAL

            # Integrate
            self._integral_x += err_x * dt
            self._integral_y += err_y * dt

            # Anti-windup clamp
            max_i = Config.POS_HOLD_INTEGRAL_MAX
            self._integral_x = max(-max_i, min(max_i, self._integral_x))
            self._integral_y = max(-max_i, min(max_i, self._integral_y))

            # PI output → velocity command
            cmd_vx = Config.POS_HOLD_KP * err_x + Config.POS_HOLD_KI * self._integral_x
            cmd_vy = Config.POS_HOLD_KP * err_y + Config.POS_HOLD_KI * self._integral_y
        else:
            self._last_pi_time = now

        # ── Send position + velocity target ───────────────────────────
        self._px.send_position_target_local_ned_full(
            x=Config.HOVER_TARGET_X,
            y=Config.HOVER_TARGET_Y,
            z=Config.HOVER_TARGET_Z,
            vx=cmd_vx,
            vy=cmd_vy,
            vz=0.0,
        )

        # ── Diagnostics every second ──────────────────────────────────
        if now - self._last_log >= 1.0:
            self._last_log = now
            alt_m = self._tf02.distance_m
            alt_str = f"{alt_m:.2f}m" if alt_m is not None else "---"
            drift = (pos_x ** 2 + pos_y ** 2) ** 0.5
            Logger.info(
                f"HOVER  | t={elapsed:.0f}s | remain={remaining:.0f}s | "
                f"pos=({pos_x:+.2f}, {pos_y:+.2f}) | "
                f"drift={drift:.3f}m | "
                f"cmd_v=({cmd_vx:+.3f}, {cmd_vy:+.3f}) | "
                f"alt={alt_str}"
            )

        # ── 60-second timeout → LAND ──────────────────────────────────
        if elapsed >= Config.HOVER_DURATION_S:
            Logger.ok(f"Hover complete after {elapsed:.1f}s → LAND")
            self._transition(MissionState.LAND)

    # ── LAND ───────────────────────────────────────────────────────────

    def _state_land(self) -> None:
        """Command MAV_CMD_NAV_LAND at current X, Y position.

        Monitors altitude and disarms upon touchdown.
        """
        elapsed = time.time() - self._state_enter_time

        # Send LAND command once on entry
        if elapsed < 0.1:
            with self._pos_lock:
                land_x = self._local_pos_x
                land_y = self._local_pos_y
            Logger.info(f"LAND: Commanding NAV_LAND at ({land_x:.2f}, {land_y:.2f})")
            self._px.send_land_command(x=land_x, y=land_y)
            # Also set LAND mode as backup
            self._px.set_mode("LAND")

        # Monitor altitude
        alt_m = self._tf02.distance_m
        alt_str = f"{alt_m:.2f}m" if alt_m is not None else "---"

        # Log every second
        now = time.time()
        if now - self._last_log >= 1.0:
            self._last_log = now
            Logger.info(f"LAND   | t={elapsed:.1f}s | alt={alt_str}")

        # Touchdown detection
        if alt_m is not None and alt_m < Config.TOUCHDOWN_ALT_M:
            if self._ground_touch_start is None:
                self._ground_touch_start = now
            elif now - self._ground_touch_start >= 1.0:
                Logger.ok(f"Ground Touch confirmed for 1s at {alt_m:.2f}m — disarming")
                self._px.disarm()
                self._transition(MissionState.DONE)
        else:
            self._ground_touch_start = None

        if elapsed > Config.LAND_DURATION_S + 5.0 and self._state != MissionState.DONE:
            Logger.warn("Land timeout — force disarm")
            self._px.disarm()
            self._transition(MissionState.DONE)

    # ── DONE ───────────────────────────────────────────────────────────

    def _state_done(self) -> None:
        """Mission complete — stop the controller."""
        Logger.ok("Mission COMPLETE")
        self._running = False


# ═══════════════════════════════════════════════════════════════════════════
# Backward-compatible alias
# ═══════════════════════════════════════════════════════════════════════════

# The old VIOStabilizer interface is no longer used.  Import MissionController
# instead.  Kept as alias so old code referencing VIOStabilizer doesn't crash.
VIOStabilizer = MissionController
