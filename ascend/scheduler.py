"""
ASCEND Phase 2 — Scheduler & State Machine
Entry point for all CLI modes. Run as: python3 -m ascend.scheduler --mode <mode>
Includes VIO-based X-Y hover stabilization via ESP32-CAM optical flow.
"""

import argparse
import enum
import signal
import sys
import time
from typing import Optional

from pymavlink import mavutil

from .config import Config
from .logger import Logger
from .hardware.tf02 import TF02Reader
from .hardware.pixhawk import PixhawkLink
from .hardware.esp32_cam import ESP32CamReader
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
    """Flight state machine states."""
    IDLE       = "IDLE"
    PREFLIGHT  = "PREFLIGHT"
    TAKEOFF    = "TAKEOFF"
    HOVER      = "HOVER"
    RETURN     = "RETURN"
    LAND       = "LAND"
    EMERGENCY  = "EMERGENCY"


# ═══════════════════════════════════════════════════════════════════════════
# State Machine
# ═══════════════════════════════════════════════════════════════════════════

class StateMachine:
    """Central flight sequencer that runs the Phase 2 hover mission.

    Manages state transitions according to the ASCEND flight-state spec:
    IDLE → PREFLIGHT → TAKEOFF → HOVER → RETURN → LAND → IDLE.

    Safety is enforced by:
      - SafetyMonitor.emergency_flag polled every loop tick
      - RC8 switch: slide DOWN = autonomous (onboard computer)
                    slide UP   = manual RC control
      - FS_GCS_ENABLE at the Pixhawk firmware level (if RPi5 crashes)

    VIO stabilization:
      - ESP32-CAM optical flow → VIOStabilizer → roll/pitch corrections
      - Applied during TAKEOFF and HOVER states via RC override
      - Corrections are zero when VIO data is invalid (graceful degradation)

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
        self._manual_mode: bool = False
        self._running: bool = True

        # Timers
        self._alt_stable_since: Optional[float] = None
        self._hover_start: Optional[float] = None
        self._touchdown_since: Optional[float] = None

        # Cached telemetry
        self._armed: bool = False
        self._mode: int = 0
        self._mode_name: str = "UNKNOWN"
        self._altitude_m: float = 0.0
        self._battery_pct: int = 0
        self._battery_volt: float = 0.0
        self._vib_x: float = 0.0
        self._vib_y: float = 0.0
        self._vib_z: float = 0.0
        self._landed_state: int = 0
        self._rc_ch8: int = 0  # latest RC channel 8 PWM value

        # Takeoff ramp state
        self._ramp_throttle: int = 1100
        self._ground_alt: float = 0.0
        self._lifted: bool = False
        self._hover_throttle: int = 1500
        self._emergency_land_sent: bool = False
        self._takeoff_sent: bool = False

        # Crash safety: force-disarm watchdog
        self._crash_land_since: Optional[float] = None

    # ── Public API ─────────────────────────────────────────────────────

    def get_telemetry(self) -> dict:
        """Build a telemetry dict for the TelemetryStreamer."""
        telem = {
            "state":        self._state.value,
            "altitude_m":   self._altitude_m,
            "battery_pct":  self._battery_pct,
            "battery_volt": self._battery_volt,
            "armed":        self._armed,
            "mode":         self._mode_name,
            "manual_mode":  self._manual_mode,
            "tf02_dist_m":  self._tf02.distance_m,
            "vib_x":        self._vib_x,
            "vib_y":        self._vib_y,
            "vib_z":        self._vib_z,
            "rc_ch8":       self._rc_ch8,
        }
        # ── VIO telemetry ─────────────────────────────────────────
        if self._vio is not None:
            roll_c, pitch_c, vio_active = self._vio.get_corrections()
            pos_x, pos_y = self._vio.get_position()
            telem["vio_active"] = vio_active
            telem["vio_roll_corr"] = roll_c
            telem["vio_pitch_corr"] = pitch_c
            telem["vio_pos_x"] = round(pos_x, 3)
            telem["vio_pos_y"] = round(pos_y, 3)
        else:
            telem["vio_active"] = False
            telem["vio_roll_corr"] = 0
            telem["vio_pitch_corr"] = 0
            telem["vio_pos_x"] = 0.0
            telem["vio_pos_y"] = 0.0
        return telem

    def run(self) -> None:
        """Execute the state machine main loop."""
        Logger.header("ASCEND STATE MACHINE — FLY MODE")
        Logger.info(f"Target altitude : {Config.TARGET_ALT_M} m")
        Logger.info(f"Hover duration  : {Config.HOVER_DURATION} s")
        Logger.info("RC8 DOWN (<1300) = Autonomous | RC8 UP (>1700) = Manual RC")
        Logger.info("Press Ctrl+C to abort → emergency LAND\n")

        self._px.request_data_streams(rate_hz=10)
        self._transition(State.IDLE)

        try:
            while self._running:
                # ── Ingest MAVLink messages ────────────────────────
                self._poll_mavlink()

                # ── RC8 switch check (highest priority) ───────────
                self._check_rc8_switch()

                if self._manual_mode:
                    time.sleep(Config.TX_POLL_INTERVAL)
                    continue

                # ── Safety monitor check ──────────────────────────
                if self._safety.emergency_flag and self._state != State.EMERGENCY:
                    Logger.error(f"Safety trigger: {self._safety.emergency_reason}")
                    self._transition(State.EMERGENCY)

                # ── State handlers ────────────────────────────────
                handler = {
                    State.IDLE:       self._state_idle,
                    State.PREFLIGHT:  self._state_preflight,
                    State.TAKEOFF:    self._state_takeoff,
                    State.HOVER:      self._state_hover,
                    State.RETURN:     self._state_return,
                    State.LAND:       self._state_land,
                    State.EMERGENCY:  self._state_emergency,
                }.get(self._state)

                if handler:
                    handler()

                time.sleep(Config.MAIN_LOOP_INTERVAL)

        except KeyboardInterrupt:
            Logger.warn("Ctrl+C — sending LAND + DISARM")
            self._px.clear_rc_override()
            self._px.set_mode("LAND")
            time.sleep(0.5)
            self._force_disarm()
        finally:
            self._running = False

    def stop(self) -> None:
        """Signal the state machine to stop."""
        self._running = False

    # ── MAVLink Polling ────────────────────────────────────────────────

    def _poll_mavlink(self) -> None:
        """Read all pending MAVLink messages and cache relevant data."""
        for _ in range(20):  # drain up to 20 messages per tick
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

            elif mtype == "GLOBAL_POSITION_INT":
                self._altitude_m = msg.relative_alt / 1000.0

            elif mtype == "LOCAL_POSITION_NED":
                self._safety.update_position(msg.x, msg.y)

            elif mtype == "VIBRATION":
                self._vib_x = msg.vibration_x
                self._vib_y = msg.vibration_y
                self._vib_z = msg.vibration_z

            elif mtype == "EXTENDED_SYS_STATE":
                self._landed_state = msg.landed_state

            elif mtype == "STATUSTEXT":
                Logger.warn(f"FC: {msg.text}")

            elif mtype == "RC_CHANNELS":
                self._rc_ch8 = msg.chan8_raw  # cache ch8 for switch logic

    # ── RC8 Switch ─────────────────────────────────────────────────────

    def _check_rc8_switch(self) -> None:
        """Check RC channel 8 to toggle autonomous / manual mode."""
        ch8 = self._rc_ch8
        if ch8 == 0 or ch8 == 65535:
            return

        autonomous_requested = ch8 < 1300   # switch DOWN = autonomous
        manual_requested = ch8 > 2000       # switch UP   = manual

        if manual_requested and not self._manual_mode:
            Logger.warn(f"RC8 MANUAL MODE (ch8={ch8}) → STABILIZE")
            self._manual_mode = True
            self._px.set_mode("STABILIZE")
            self._px.clear_rc_override()

        elif autonomous_requested and self._manual_mode:
            Logger.ok(f"RC8 AUTONOMOUS MODE (ch8={ch8}) → resuming mission")
            self._manual_mode = False
            self._safety.reset()
            self._transition(State.IDLE)

    # ── State Transition ───────────────────────────────────────────────

    def _transition(self, new_state: State) -> None:
        old = self._state
        self._state = new_state
        self._emergency_land_sent = False
        self._takeoff_sent = False
        self._ramp_throttle = 1100
        self._lifted = False

        if new_state in (State.TAKEOFF, State.HOVER):
            self._safety.set_flight_active(True)
        elif new_state in (State.IDLE, State.LAND):
            self._safety.set_flight_active(False)

        Logger.ok(f"STATE: {old.value} → {new_state.value}")

    # ── State Handlers ─────────────────────────────────────────────────

    def _state_idle(self) -> None:
        """IDLE — waiting on the ground. Auto-transition to PREFLIGHT."""
        self._transition(State.PREFLIGHT)

    def _state_preflight(self) -> None:
        """PREFLIGHT — arm the drone and validate sensors."""
        Logger.info("Preflight: arming in STABILIZE mode …")
        self._px.set_mode("STABILIZE")

        deadline = time.time() + Config.PREFLIGHT_TIMEOUT
        while time.time() < deadline and self._running:
            self._poll_mavlink()
            if self._mode_name == "STABILIZE":
                Logger.ok("Mode confirmed STABILIZE — sending ARM")
                break
            time.sleep(0.2)
        else:
            Logger.error("Could not set STABILIZE mode — returning to IDLE")
            self._transition(State.IDLE)
            return

        time.sleep(0.5)
        self._px.arm()

        deadline = time.time() + Config.PREFLIGHT_TIMEOUT
        while time.time() < deadline and self._running:
            self._poll_mavlink()
            if self._armed:
                Logger.ok("Drone ARMED")
                self._px.set_mode("STABILIZE")
                self._transition(State.TAKEOFF)
                return
            time.sleep(0.2)

        Logger.error("Arming FAILED within timeout — returning to IDLE")
        self._transition(State.IDLE)

    def _get_vio_roll_pitch(self) -> tuple:
        """Return (roll_pwm, pitch_pwm) with VIO corrections applied.

        If VIO is unavailable or inactive, returns neutral (1500, 1500).
        """
        roll = 1500
        pitch = 1500
        if self._vio is not None:
            roll_c, pitch_c, _ = self._vio.get_corrections()
            roll = max(1000, min(2000, 1500 + roll_c))
            pitch = max(1000, min(2000, 1500 + pitch_c))
        return roll, pitch

    def _state_takeoff(self) -> None:
        """TAKEOFF — gradual throttle ramp until liftoff then climb to target."""
        tf02 = self._tf02.distance_m
        if tf02 is None:
            Logger.warn("TF02 no data — holding")
            self._px.send_rc_override(throttle=1100)
            return

        alt = tf02
        target = Config.TARGET_ALT_M

        if self._ground_alt == 0.0:
            self._ground_alt = alt
            Logger.info(f"Starting throttle ramp from 1100. Ground={alt:.2f}m")

        # Get VIO-corrected roll/pitch for X-Y stabilization
        roll, pitch = self._get_vio_roll_pitch()

        if not self._lifted:
            if alt > self._ground_alt + 0.15:
                self._lifted = True
                Logger.ok(f"LIFTOFF detected at throttle={self._ramp_throttle} alt={alt:.2f}m")
            else:
                self._ramp_throttle += 1
                self._ramp_throttle = min(self._ramp_throttle, 2000)
                Logger.info(f"Ramping... alt={alt:.2f}m throttle={self._ramp_throttle}")
                self._px.send_rc_override(
                    roll=roll, pitch=pitch, throttle=self._ramp_throttle
                )
                return

        error = target - alt
        KP = 30.0
        throttle = int(self._ramp_throttle + KP * error)
        throttle = max(1100, min(1900, throttle))

        Logger.info(f"CLIMBING alt={alt:.2f}m target={target:.2f}m error={error:.2f}m throttle={throttle}")
        self._px.send_rc_override(roll=roll, pitch=pitch, throttle=throttle)

        if abs(error) <= Config.ALT_TOLERANCE_M:
            if self._alt_stable_since is None:
                self._alt_stable_since = time.time()
            elif time.time() - self._alt_stable_since >= Config.ALT_STABLE_TIME:
                Logger.ok(f"Altitude stable at {alt:.2f}m")
                self._alt_stable_since = None
                self._hover_throttle = throttle
                self._ramp_throttle = 1100
                self._lifted = False
                self._transition(State.HOVER)
        else:
            self._alt_stable_since = None

    def _state_hover(self) -> None:
        """HOVER — hold altitude (Z) + CV position hold (X-Y)."""
        if self._hover_start is None:
            self._hover_start = time.time()
            # Reset VIO position origin — hold from HERE
            if self._vio is not None:
                self._vio.reset_position()
            Logger.info(f"Hovering for {Config.HOVER_DURATION:.0f}s …")

        tf02 = self._tf02.distance_m
        if tf02 is None:
            self._px.send_rc_override(throttle=1500)
            return

        alt = tf02
        target = Config.TARGET_ALT_M
        error = target - alt
        base = getattr(self, '_hover_throttle', 1500)

        KP = 30.0
        throttle = int(base + KP * error)
        throttle = max(1100, min(1900, throttle))

        # Get VIO-corrected roll/pitch for X-Y stabilization
        roll, pitch = self._get_vio_roll_pitch()

        elapsed = time.time() - self._hover_start
        vio_tag = ""
        if self._vio is not None and self._vio.active:
            rc, pc, _ = self._vio.get_corrections()
            px, py = self._vio.get_position()
            vio_tag = f" vio_r={rc:+d} vio_p={pc:+d} pos=({px:+.2f},{py:+.2f})m"
        Logger.info(
            f"HOVER alt={alt:.2f}m error={error:.2f}m throttle={throttle}"
            f" roll={roll} pitch={pitch}{vio_tag} elapsed={elapsed:.0f}s"
        )

        self._px.send_rc_override(
            roll=roll, pitch=pitch, throttle=throttle
        )

        if 0 < self._battery_volt < Config.LOW_BATTERY_VOLT:
            Logger.warn(f"Low battery ({self._battery_volt:.2f}V) → RETURN")
            self._hover_start = None
            self._transition(State.RETURN)
            return

        if elapsed >= Config.HOVER_DURATION:
            Logger.ok(f"Hover complete ({elapsed:.0f}s) → RETURN")
            self._hover_start = None
            self._transition(State.RETURN)

    def _state_return(self) -> None:
        """RETURN — descend slowly back to ground."""
        self._transition(State.LAND)

    def _state_land(self) -> None:
        """LAND — controlled descent, wait for touchdown."""
        self._px.set_mode("LAND")
        self._check_touchdown()
        self._check_crash_disarm()

    def _state_emergency(self) -> None:
        """EMERGENCY — immediate LAND from any state."""
        if not self._emergency_land_sent:
            self._px.set_mode("LAND")
            self._px.clear_rc_override()
            self._emergency_land_sent = True
        self._check_touchdown()
        self._check_crash_disarm()

    # ── Force Disarm (Crash Safety) ────────────────────────────────────

    def _force_disarm(self) -> None:
        """Force-disarm the drone — stops ALL motors immediately.

        Uses the existing PixhawkLink.disarm() method. Called when:
          - Ctrl+C abort
          - Crash watchdog triggers (stuck on ground with props spinning)
          - Cleanup / shutdown
        """
        Logger.warn("FORCE DISARM — stopping motors")
        self._px.clear_rc_override()
        self._px.disarm()

    def _check_crash_disarm(self) -> None:
        """Watchdog: if in LAND/EMERGENCY and stuck near ground, force-disarm.

        Prevents props from spinning after a crash/tip-over. If TF-02 reads
        below CRASH_DISARM_ALT_M for longer than CRASH_DISARM_TIMEOUT, the
        drone is force-disarmed regardless of FC landed_state.
        """
        if self._state not in (State.LAND, State.EMERGENCY):
            self._crash_land_since = None
            return

        tf02_m = self._tf02.distance_m
        if tf02_m is not None and tf02_m < Config.CRASH_DISARM_ALT_M:
            if self._crash_land_since is None:
                self._crash_land_since = time.time()
            elif time.time() - self._crash_land_since >= Config.CRASH_DISARM_TIMEOUT:
                Logger.warn(
                    f"Crash watchdog: alt={tf02_m:.2f}m for "
                    f">{Config.CRASH_DISARM_TIMEOUT:.0f}s — force disarming"
                )
                self._force_disarm()
                self._crash_land_since = None
                self._safety.reset()
                self._transition(State.IDLE)
                self._running = False
        else:
            self._crash_land_since = None

    # ── Touchdown Detection ────────────────────────────────────────────

    def _check_touchdown(self) -> None:
        """Detect touchdown using three criteria."""
        tf02_m = self._tf02.distance_m
        tf02_low = tf02_m is not None and tf02_m < Config.TOUCHDOWN_ALT_M
        on_ground = self._landed_state == 1

        if tf02_low:
            if self._touchdown_since is None:
                self._touchdown_since = time.time()
        else:
            self._touchdown_since = None

        tf02_stable = (self._touchdown_since is not None and
                       time.time() - self._touchdown_since >= Config.TOUCHDOWN_TIME)

        if on_ground and tf02_stable and not self._armed:
            Logger.ok("Touchdown confirmed — motors disarmed")
            self._touchdown_since = None
            self._safety.reset()
            self._transition(State.IDLE)
            self._running = False

    @staticmethod
    def _resolve_mode(mode_id: int) -> str:
        """Return ArduCopter mode name for a mode number."""
        for name, mid in Config.MODE_MAP.items():
            if mid == mode_id:
                return name
        return f"MODE_{mode_id}"


# ═══════════════════════════════════════════════════════════════════════════
# Scheduler (Entry Point)
# ═══════════════════════════════════════════════════════════════════════════

class Scheduler:
    """CLI entry point — instantiates shared objects and routes to the correct mode."""
    def __init__(self, mode: str) -> None:
        self._mode = mode.lower()
        self._threads: list = []
        self._tf02: Optional[TF02Reader] = None
        self._esp32_cam: Optional[ESP32CamReader] = None
        self._pixhawk: Optional[PixhawkLink] = None

    def run(self) -> int:
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
        if not self._connect_pixhawk():
            return 1
        self._start_tf02()
        self._start_esp32_cam()
        time.sleep(5.0)

        safety = SafetyMonitor(self._tf02, self._pixhawk)
        safety.start()
        self._threads.append(safety)

        # Start VIO stabilizer (X-Y hover hold via optical flow)
        vio = VIOStabilizer(self._esp32_cam, self._tf02)
        vio.start()
        self._threads.append(vio)
        Logger.ok("VIO stabilizer started — X-Y hover hold active")

        self._start_bridge()
        self._start_heartbeat()

        sm = StateMachine(self._pixhawk, self._tf02, safety, vio=vio)
        telem = TelemetryStreamer(sm.get_telemetry)
        telem.start()
        self._threads.append(telem)

        sm.run()
        return 0

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
        """Start the ESP32-CAM optical-flow reader thread."""
        self._esp32_cam = ESP32CamReader()
        self._esp32_cam.start()
        self._threads.append(self._esp32_cam)

    def _cleanup(self) -> None:
        Logger.info("Cleaning up …")
        # Force-disarm as safety net — ensure motors stop
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
        # Best-effort disarm before raising KeyboardInterrupt
        if self._pixhawk and self._pixhawk.connected:
            try:
                self._pixhawk.clear_rc_override()
                self._pixhawk.disarm()
            except Exception:
                pass
        raise KeyboardInterrupt


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="ascend",
        description="ASCEND Phase 1 — Autonomous drone flight system",
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
