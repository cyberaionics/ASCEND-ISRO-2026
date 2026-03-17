"""
ASCEND Phase 1 — Scheduler & State Machine
Entry point for all CLI modes.  Run as:  python3 -m ascend.scheduler --mode <mode>
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
from .threads.bridge import (
    HeartbeatSender,
    RangefinderBridge,
    SafetyMonitor,
    TelemetryStreamer,
)
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
    """Central flight sequencer that runs the Phase 1 hover mission.

    Manages state transitions according to the ASCEND flight-state spec:
    IDLE → PREFLIGHT → TAKEOFF → HOVER → RETURN → LAND → IDLE.

    Safety is enforced by:
      - ``SafetyMonitor.emergency_flag`` polled every loop tick
      - TX override via ``RC_CHANNELS`` polled every 100 ms
      - ``FS_GCS_ENABLE`` at the Pixhawk firmware level (if RPi5 crashes)

    Args:
        pixhawk: Connected PixhawkLink instance.
        tf02:    Running TF02Reader instance.
        safety:  Running SafetyMonitor instance.
    """

    def __init__(self, pixhawk: PixhawkLink, tf02: TF02Reader,
                 safety: SafetyMonitor) -> None:
        self._px = pixhawk
        self._tf02 = tf02
        self._safety = safety
        self._state = State.IDLE
        self._manual_mode: bool = False
        self._running: bool = True

        # Timers
        self._alt_stable_since: Optional[float] = None
        self._hover_start: Optional[float] = None
        self._touchdown_since: Optional[float] = None

        # Cached telemetry values (updated from MAVLink stream)
        self._armed: bool = False
        self._mode: int = 0
        self._mode_name: str = "UNKNOWN"
        self._altitude_m: float = 0.0
        self._battery_pct: int = 0
        self._battery_volt: float = 0.0
        self._vib_x: float = 0.0
        self._vib_y: float = 0.0
        self._vib_z: float = 0.0
        self._landed_state: int = 0  # from EXTENDED_SYS_STATE

    # ── Public API ─────────────────────────────────────────────────────

    def get_telemetry(self) -> dict:
        """Build a telemetry dict for the TelemetryStreamer."""
        return {
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
        }

    def run(self) -> None:
        """Execute the state machine main loop.

        Blocks the calling thread until the mission completes or is
        interrupted by Ctrl+C.
        """
        Logger.header("ASCEND STATE MACHINE — FLY MODE")
        Logger.info(f"Target altitude : {Config.TARGET_ALT_M} m")
        Logger.info(f"Hover duration  : {Config.HOVER_DURATION} s")
        Logger.info("Press Ctrl+C to abort → emergency LAND\n")

        self._px.request_data_streams(rate_hz=10)
        self._transition(State.IDLE)

        try:
            while self._running:
                # ── Ingest MAVLink messages ────────────────────────
                self._poll_mavlink()

                # ── TX override check (highest priority) ──────────
                self._check_tx_override()

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
            Logger.warn("Ctrl+C — sending LAND and stopping")
            self._px.set_mode("LAND")
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

            elif mtype == "RC_CHANNELS":
                pass  # handled by _check_tx_override directly

    # ── TX Override ────────────────────────────────────────────────────

    def _check_tx_override(self) -> None:
        """Poll RC_CHANNELS to detect transmitter presence.

        If valid PWM (1000–2000) is detected on channel 1, immediately
        switch to STABILIZE and set ``manual_mode = True``.
        """
        msg = self._px.recv("RC_CHANNELS", timeout=0.05)
        if msg is None:
            return

        ch1 = msg.chan1_raw
        tx_present = Config.RC_PWM_MIN <= ch1 <= Config.RC_PWM_MAX

        if tx_present and not self._manual_mode:
            # TX just appeared — hand off to pilot
            Logger.warn(f"TX OVERRIDE DETECTED (ch1={ch1}) → STABILIZE")
            self._manual_mode = True
            self._px.set_mode("STABILIZE")

        elif not tx_present and self._manual_mode:
            # TX disappeared — return to IDLE (do NOT resume previous state)
            Logger.info("TX lost → returning to IDLE (autonomous)")
            self._manual_mode = False
            self._safety.reset()
            self._transition(State.IDLE)

    # ── State Transition ───────────────────────────────────────────────

    def _transition(self, new_state: State) -> None:
        """Transition to a new state with logging."""
        old = self._state
        self._state = new_state

        if new_state in (State.TAKEOFF, State.HOVER):
            self._safety.set_flight_active(True)
        elif new_state in (State.IDLE, State.LAND):
            self._safety.set_flight_active(False)

        Logger.ok(f"STATE: {old.value} → {new_state.value}")

    # ── State Handlers ─────────────────────────────────────────────────

    def _state_idle(self) -> None:
        """IDLE — waiting on the ground. Auto-transition to PREFLIGHT."""
        # In --mode fly, we start immediately
        self._transition(State.PREFLIGHT)

    def _state_preflight(self) -> None:
        """PREFLIGHT — arm the drone and validate sensors."""
        Logger.info("Preflight: arming in GUIDED mode …")
        self._px.set_mode("GUIDED")
        time.sleep(1.0)
        self._px.arm()

        # Wait for arming confirmation
        deadline = time.time() + Config.PREFLIGHT_TIMEOUT
        while time.time() < deadline and self._running:
            self._poll_mavlink()
            if self._armed:
                Logger.ok("Drone ARMED")
                self._transition(State.TAKEOFF)
                return
            time.sleep(0.2)

        Logger.error("Arming FAILED within timeout — returning to IDLE")
        self._transition(State.IDLE)

    def _state_takeoff(self) -> None:
        """TAKEOFF — climb to target altitude."""
        Logger.info(f"Sending TAKEOFF to {Config.TARGET_ALT_M} m")
        self._px.takeoff(Config.TARGET_ALT_M)

        # Wait until at altitude and stable for ALT_STABLE_TIME
        tf02_alt = self._tf02.distance_m
        if tf02_alt is not None:
            alt = tf02_alt
        else:
            alt = self._altitude_m

        target = Config.TARGET_ALT_M
        if abs(alt - target) <= Config.ALT_TOLERANCE_M:
            if self._alt_stable_since is None:
                self._alt_stable_since = time.time()
            elif time.time() - self._alt_stable_since >= Config.ALT_STABLE_TIME:
                Logger.ok(f"Altitude stable at {alt:.2f} m for "
                           f"{Config.ALT_STABLE_TIME}s")
                self._alt_stable_since = None
                self._transition(State.HOVER)
        else:
            self._alt_stable_since = None

    def _state_hover(self) -> None:
        """HOVER — hold position for configured duration."""
        if self._hover_start is None:
            self._hover_start = time.time()
            self._px.set_mode("LOITER")
            Logger.info(f"Hovering for {Config.HOVER_DURATION:.0f}s …")

        elapsed = time.time() - self._hover_start

        # Battery warning check
        if self._battery_volt > 0 and self._battery_volt < Config.LOW_BATTERY_VOLT:
            Logger.warn(f"Low battery ({self._battery_volt:.2f}V) → RETURN")
            self._hover_start = None
            self._transition(State.RETURN)
            return

        if elapsed >= Config.HOVER_DURATION:
            Logger.ok(f"Hover complete ({elapsed:.0f}s) → RETURN")
            self._hover_start = None
            self._transition(State.RETURN)

    def _state_return(self) -> None:
        """RETURN — fly back to home using RTL mode."""
        self._px.set_mode("RTL")

        # Check if we're close to home (from LOCAL_POSITION_NED)
        msg = self._px.recv("LOCAL_POSITION_NED", timeout=0.2)
        if msg is not None:
            dist_home = (msg.x ** 2 + msg.y ** 2) ** 0.5
            if dist_home <= Config.HOME_RADIUS_M:
                Logger.ok(f"Home reached (dist={dist_home:.1f}m) → LAND")
                self._transition(State.LAND)

    def _state_land(self) -> None:
        """LAND — controlled descent, wait for touchdown."""
        self._px.set_mode("LAND")
        self._check_touchdown()

    def _state_emergency(self) -> None:
        """EMERGENCY — immediate LAND from any state."""
        self._px.set_mode("LAND")
        self._check_touchdown()

    # ── Touchdown Detection ────────────────────────────────────────────

    def _check_touchdown(self) -> None:
        """Detect touchdown using three criteria:
        1. landed_state == ON_GROUND  (EXTENDED_SYS_STATE)
        2. TF-02 altitude < 0.10 m for > 1.0 s
        3. Motors disarmed
        """
        tf02_m = self._tf02.distance_m
        tf02_low = tf02_m is not None and tf02_m < Config.TOUCHDOWN_ALT_M
        on_ground = self._landed_state == 1  # MAV_LANDED_STATE_ON_GROUND

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
            self._running = False  # mission complete

    # ── Utility ────────────────────────────────────────────────────────

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
    """CLI entry point — instantiates shared objects and routes to the
    correct mode.

    Supported modes:
      ``check``    — run HealthChecker only (bench)
      ``setup``    — run AutoTuneSetup only (bench)
      ``all``      — HealthChecker → AutoTuneSetup in sequence (bench)
      ``autotune`` — run AutoTuneMonitor (field, pilot flies)
      ``fly``      — run full StateMachine hover mission (field)
    """

    def __init__(self, mode: str) -> None:
        self._mode = mode.lower()
        self._threads: list = []
        self._tf02: Optional[TF02Reader] = None
        self._pixhawk: Optional[PixhawkLink] = None

    def run(self) -> int:
        """Execute the selected mode.

        Returns:
            Exit code (0 = success, 1 = failure).
        """
        Logger.banner()
        Logger.info(f"Mode: {self._mode}")
        Logger.rule()

        # Register signal handler for graceful shutdown
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
        """Run HealthChecker only."""
        if not self._connect_pixhawk():
            return 1
        checker = HealthChecker(self._pixhawk)
        return 0 if checker.run_all() else 1

    def _run_setup(self) -> int:
        """Run AutoTuneSetup only."""
        if not self._connect_pixhawk():
            return 1
        setup = AutoTuneSetup(self._pixhawk)
        return 0 if setup.run_all() else 1

    def _run_all(self) -> int:
        """HealthChecker → AutoTuneSetup in sequence."""
        if not self._connect_pixhawk():
            return 1

        checker = HealthChecker(self._pixhawk)
        if not checker.run_all():
            Logger.error("Health check failed — skipping setup")
            return 1

        setup = AutoTuneSetup(self._pixhawk)
        return 0 if setup.run_all() else 1

    def _run_autotune(self) -> int:
        """Run AutoTuneMonitor with background bridge + heartbeat."""
        if not self._connect_pixhawk():
            return 1
        self._start_tf02()
        self._start_bridge()
        self._start_heartbeat()

        monitor = AutoTuneMonitor(self._pixhawk)
        monitor.run()  # blocks until Ctrl+C
        return 0

    def _run_fly(self) -> int:
        """Run the full StateMachine hover mission."""
        if not self._connect_pixhawk():
            return 1
        self._start_tf02()
        time.sleep(0.5)  # let TF02 start reading

        # Create safety monitor
        safety = SafetyMonitor(self._tf02, self._pixhawk)
        safety.start()
        self._threads.append(safety)

        # Start background threads
        self._start_bridge()
        self._start_heartbeat()

        # State machine
        sm = StateMachine(self._pixhawk, self._tf02, safety)

        # Telemetry streamer (reads from state machine)
        telem = TelemetryStreamer(sm.get_telemetry)
        telem.start()
        self._threads.append(telem)

        sm.run()  # blocks until mission complete or Ctrl+C
        return 0

    # ── Helpers ────────────────────────────────────────────────────────

    def _connect_pixhawk(self) -> bool:
        """Create and connect the PixhawkLink instance."""
        self._pixhawk = PixhawkLink()
        if not self._pixhawk.connect():
            return False
        return True

    def _start_tf02(self) -> None:
        """Start the TF-02 reader thread."""
        self._tf02 = TF02Reader()
        self._tf02.start()
        self._threads.append(self._tf02)

    def _start_bridge(self) -> None:
        """Start the RangefinderBridge thread."""
        bridge = RangefinderBridge(self._tf02, self._pixhawk)
        bridge.start()
        self._threads.append(bridge)

    def _start_heartbeat(self) -> None:
        """Start the HeartbeatSender thread."""
        hb = HeartbeatSender(self._pixhawk)
        hb.start()
        self._threads.append(hb)

    def _cleanup(self) -> None:
        """Stop all daemon threads and close connections."""
        Logger.info("Cleaning up …")
        for t in self._threads:
            if hasattr(t, "stop"):
                t.stop()
        for t in self._threads:
            t.join(timeout=2.0)
        if self._pixhawk:
            self._pixhawk.close()
        Logger.ok("Shutdown complete")

    def _shutdown_handler(self, signum: int, frame: object) -> None:
        """Handle SIGINT/SIGTERM for graceful shutdown."""
        raise KeyboardInterrupt


# ═══════════════════════════════════════════════════════════════════════════
# Entry Point — python3 -m ascend.scheduler --mode <mode>
# ═══════════════════════════════════════════════════════════════════════════

def main() -> None:
    """Parse CLI arguments and run the scheduler."""
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
