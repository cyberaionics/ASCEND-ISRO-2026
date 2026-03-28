"""
ASCEND Phase 2 — State Machine
All seven flight-state handlers live here. Runs on the main thread.
"""

import math
import threading
import time
from typing import Optional

from pymavlink import mavutil

from .config import Config
from .logger import Logger
from .drone_state import DroneState, State
from .hardware.pixhawk import PixhawkLink
from .hardware.tf02 import TF02Reader
from .threads.commands import CommandReceiver


class StateMachine:
    """Central flight sequencer for the Phase 2 autonomous hover mission.

    Runs on the main thread at 20 Hz. Each iteration:
        1. Polls MAVLink messages and updates DroneState.
        2. Checks ``emergency_flag`` → EMERGENCY if set.
        3. Checks ``manual_mode`` → pauses commanding if True.
        4. Dispatches to the current state handler.

    State handlers return the **next** :class:`State`; the main loop
    performs the transition and logs it via :meth:`Logger.state`.

    Args:
        pixhawk: Connected PixhawkLink instance.
        tf02: Running TF02Reader instance.
        drone_state: Shared DroneState instance.
        emergency_flag: ``threading.Event`` set by SafetyMonitor.
        cmd_rx: CommandReceiver instance (for start_flag).
    """

    def __init__(
        self,
        pixhawk: PixhawkLink,
        tf02: TF02Reader,
        drone_state: DroneState,
        emergency_flag: threading.Event,
        cmd_rx: CommandReceiver,
    ) -> None:
        self._px = pixhawk
        self._tf02 = tf02
        self._ds = drone_state
        self._eflag = emergency_flag
        self._cmd = cmd_rx
        self._running: bool = True

        # Per-state timers (reset on state entry)
        self._alt_stable_since: Optional[float] = None
        self._hover_start: Optional[float] = None
        self._hover_last_log: float = 0.0
        self._drift_start: Optional[float] = None
        self._touchdown_since: Optional[float] = None
        self._last_alt_log: float = 0.0
        self._last_dist_log: float = 0.0

        # Command-sent-once flags (avoid resending every tick)
        self._takeoff_sent: bool = False
        self._land_sent: bool = False
        self._rtl_sent: bool = False
        self._loiter_sent: bool = False

    # ── Public API ─────────────────────────────────────────────────────

    def run(self) -> None:
        """Execute the main state-machine loop (blocks until complete)."""
        Logger.header("ASCEND PHASE 2 — AUTONOMOUS FLIGHT")
        Logger.info(f"Target altitude : {Config.TARGET_ALT_M} m")
        Logger.info(f"Hover duration  : {Config.HOVER_DURATION:.0f} s")
        Logger.info("Awaiting start command from laptop …\n")

        self._px.request_data_streams(rate_hz=10)
        self._ds.set_current_state(State.IDLE)

        try:
            while self._running:
                # 1. Ingest MAVLink telemetry
                self._poll_mavlink()

                # 2. Emergency check (highest priority after TX)
                if (self._eflag.is_set()
                        and self._ds.current_state != State.EMERGENCY):
                    reason = self._ds.emergency_reason or "unknown"
                    self._transition(State.EMERGENCY, reason)

                # 3. Abort command check
                if self._cmd.abort_flag.is_set():
                    self._cmd.abort_flag.clear()
                    if self._ds.current_state != State.EMERGENCY:
                        self._transition(State.EMERGENCY, "abort_mission command")

                # 4. Manual mode — pause commanding
                if self._ds.manual_mode:
                    time.sleep(Config.TX_POLL_INTERVAL)
                    continue

                # 5. Dispatch to current state handler
                current = self._ds.current_state
                handler = {
                    State.IDLE:       self._handle_idle,
                    State.PREFLIGHT:  self._handle_preflight,
                    State.TAKEOFF:    self._handle_takeoff,
                    State.HOVER:      self._handle_hover,
                    State.RETURN:     self._handle_return,
                    State.LAND:       self._handle_land,
                    State.EMERGENCY:  self._handle_emergency,
                }.get(current)

                if handler:
                    next_state = handler()
                    if next_state is not None and next_state != current:
                        self._transition(next_state)

                time.sleep(Config.MAIN_LOOP_INTERVAL)

        except KeyboardInterrupt:
            Logger.warn("Ctrl+C — sending LAND")
            self._px.set_mode("LAND")
        finally:
            self._running = False

    def stop(self) -> None:
        """Stop the state machine loop."""
        self._running = False

    def on_tx_lost(self) -> None:
        """Called by TXOverrideWatcher when TX signal disappears."""
        self._transition(State.IDLE, "TX lost — returning to IDLE")

    # ── State Transition ───────────────────────────────────────────────

    def _transition(self, new_state: State, reason: str = "") -> None:
        """Perform a state transition: update DroneState, reset timers, log."""
        old = self._ds.current_state
        self._ds.set_current_state(new_state)
        Logger.state(old.value, new_state.value, reason)

        # Reset all per-state variables
        self._alt_stable_since = None
        self._hover_start = None
        self._hover_last_log = 0.0
        self._drift_start = None
        self._touchdown_since = None
        self._last_alt_log = 0.0
        self._last_dist_log = 0.0
        self._takeoff_sent = False
        self._land_sent = False
        self._rtl_sent = False
        self._loiter_sent = False

    # ═══════════════════════════════════════════════════════════════════
    # STATE HANDLERS — each returns Optional[State]
    # Return None to stay in current state, or a new State to transition.
    # ═══════════════════════════════════════════════════════════════════

    def _handle_idle(self) -> Optional[State]:
        """IDLE — on the ground, awaiting start command from laptop."""
        if not self._cmd.start_flag.is_set():
            return None

        # Start command received — validate conditions
        self._cmd.start_flag.clear()

        if self._ds.manual_mode:
            Logger.warn("start_mission ignored — TX override active")
            return None

        volt = self._ds.battery_volt
        if volt > 0 and volt < Config.BATT_MIN_START_V:
            Logger.warn(f"Battery too low to start: {volt:.2f}V "
                        f"(need ≥ {Config.BATT_MIN_START_V}V)")
            return None

        Logger.ok("Start command accepted — entering PREFLIGHT")
        return State.PREFLIGHT

    def _handle_preflight(self) -> Optional[State]:
        """PREFLIGHT — run 4 sequential gates, arm if all pass."""

        # Gate 1: Battery voltage
        volt = self._ds.battery_volt
        if volt > 0 and volt < Config.BATT_MIN_START_V:
            Logger.error(f"Gate 1 FAIL: battery {volt:.2f}V "
                         f"< {Config.BATT_MIN_START_V}V")
            return State.IDLE
        Logger.ok(f"Gate 1 PASS: battery {volt:.2f}V")

        # Gate 2: TF-02 producing valid readings
        if self._tf02.distance_cm is None:
            Logger.error("Gate 2 FAIL: TF-02 not producing readings")
            return State.IDLE
        Logger.ok(f"Gate 2 PASS: TF-02 reading {self._tf02.distance_cm} cm")

        # Gate 3: EKF healthy (pos_horiz_abs, vel_horiz, pred_pos_horiz_abs)
        ekf = self._px.recv("EKF_STATUS_REPORT", timeout=2.0)
        if ekf is None:
            Logger.error("Gate 3 FAIL: no EKF_STATUS_REPORT received")
            return State.IDLE
        flags = ekf.flags
        ekf_ok = bool(flags & 0x01) and bool(flags & 0x02) and bool(flags & 0x08)
        if not ekf_ok:
            Logger.error(f"Gate 3 FAIL: EKF flags unhealthy (0x{flags:04x})")
            return State.IDLE
        Logger.ok(f"Gate 3 PASS: EKF healthy (0x{flags:04x})")

        # Gate 4: No emergency flag set
        if self._eflag.is_set():
            Logger.error("Gate 4 FAIL: emergency_flag is set")
            return State.IDLE
        Logger.ok("Gate 4 PASS: no emergency")

        # All gates passed — set GUIDED mode and arm
        Logger.info("All gates passed — arming in GUIDED mode")
        self._px.set_mode("GUIDED")
        time.sleep(0.5)  # allow mode change to propagate
        self._px.arm()

        # Wait up to ARMING_TIMEOUT for armed confirmation
        deadline = time.time() + Config.ARMING_TIMEOUT
        while time.time() < deadline:
            self._poll_mavlink()
            if self._ds.armed:
                Logger.ok("Pixhawk ARMED — ready for takeoff")
                return State.TAKEOFF
            time.sleep(0.1)

        Logger.error("Arming not confirmed within timeout")
        return State.IDLE

    def _handle_takeoff(self) -> Optional[State]:
        """TAKEOFF — climb to target altitude, wait for stability."""
        if not self._takeoff_sent:
            self._px.takeoff(Config.TARGET_ALT_M)
            self._takeoff_sent = True

        alt = self._ds.altitude_m

        # Log altitude every 0.5 s
        now = time.time()
        if now - self._last_alt_log >= 0.5:
            Logger.info(f"  climbing … alt={alt:.2f} m "
                        f"(target={Config.TARGET_ALT_M})")
            self._last_alt_log = now

        # Check altitude stability: within tolerance for ALT_STABLE_TIME
        if abs(alt - Config.TARGET_ALT_M) <= Config.ALT_TOLERANCE_M:
            if self._alt_stable_since is None:
                self._alt_stable_since = time.time()
            elif time.time() - self._alt_stable_since >= Config.ALT_STABLE_TIME:
                Logger.ok(f"Altitude stable at {alt:.2f} m for "
                          f"{Config.ALT_STABLE_TIME}s")
                return State.HOVER
        else:
            # Reset stability timer if altitude drifts out of tolerance
            self._alt_stable_since = None

        return None

    def _handle_hover(self) -> Optional[State]:
        """HOVER — hold position in LOITER for 5 minutes."""
        if not self._loiter_sent:
            self._px.set_mode("LOITER")
            self._loiter_sent = True
            self._hover_start = time.time()
            Logger.info(f"Hovering for {Config.HOVER_DURATION:.0f}s …")

        elapsed = time.time() - self._hover_start
        remaining = max(0.0, Config.HOVER_DURATION - elapsed)
        self._ds.set_hover_remaining_s(remaining)

        # Log remaining time every 60 s
        now = time.time()
        if now - self._hover_last_log >= 60.0:
            Logger.info(f"  hover remaining: {remaining:.0f}s "
                        f"({remaining / 60:.1f} min)")
            self._hover_last_log = now

        # Battery check — emergency if below BATT_LOW_V
        volt = self._ds.battery_volt
        if volt > 0 and volt < Config.BATT_LOW_V:
            Logger.error(f"Low battery in hover: {volt:.2f}V")
            return State.EMERGENCY

        # Altitude drift warning (informational only, does not exit state)
        alt = self._ds.altitude_m
        if abs(alt - Config.TARGET_ALT_M) > Config.HOVER_DRIFT_LIMIT_M:
            if self._drift_start is None:
                self._drift_start = time.time()
            elif time.time() - self._drift_start > Config.HOVER_DRIFT_TIME:
                Logger.warn(f"Altitude drift: {alt:.2f} m "
                            f"(target {Config.TARGET_ALT_M})")
                # Reset to avoid log spam
                self._drift_start = time.time()
        else:
            self._drift_start = None

        # Timer expired → RETURN
        if remaining <= 0:
            Logger.ok("Hover complete → RETURN")
            self._ds.set_hover_remaining_s(None)
            return State.RETURN

        return None

    def _handle_return(self) -> Optional[State]:
        """RETURN — fly back to home via RTL."""
        if not self._rtl_sent:
            self._px.set_mode("RTL")
            self._rtl_sent = True

        dist = self._ds.dist_to_home_m

        # Log distance every 2 s
        now = time.time()
        if now - self._last_dist_log >= 2.0 and dist is not None:
            Logger.info(f"  dist to home: {dist:.1f} m")
            self._last_dist_log = now

        # Transition to LAND when close to home
        if dist is not None and dist < Config.HOME_RADIUS_M:
            Logger.ok(f"Home reached (dist={dist:.1f}m) → LAND")
            return State.LAND

        return None

    def _handle_land(self) -> Optional[State]:
        """LAND — controlled descent, wait for touchdown confirmation."""
        if not self._land_sent:
            self._px.set_mode("LAND")
            self._land_sent = True

        # Log altitude every 0.5 s
        now = time.time()
        if now - self._last_alt_log >= 0.5:
            alt = self._ds.altitude_m
            Logger.info(f"  landing … alt={alt:.2f} m")
            self._last_alt_log = now

        if self._check_touchdown():
            Logger.ok("LANDED — mission complete")
            self._ds.set_hover_remaining_s(None)
            self._running = False
            return State.IDLE

        return None

    def _handle_emergency(self) -> Optional[State]:
        """EMERGENCY — immediate LAND, wait for touchdown."""
        if not self._land_sent:
            self._px.set_mode("LAND")
            self._land_sent = True
            Logger.error(f"EMERGENCY LAND — reason: "
                         f"{self._ds.emergency_reason or 'unknown'}")

        if self._check_touchdown():
            Logger.ok("Emergency landing complete")
            self._eflag.clear()
            self._ds.set_emergency_reason(None)
            return State.IDLE

        return None

    # ── Touchdown Detection ────────────────────────────────────────────

    def _check_touchdown(self) -> bool:
        """Check three simultaneous touchdown conditions.

        Cond 1: EXTENDED_SYS_STATE.landed_state == MAV_LANDED_STATE_ON_GROUND
        Cond 2: TF-02 < TOUCHDOWN_ALT_CM for > TOUCHDOWN_TIME continuously
        Cond 3: HEARTBEAT armed bit == 0 (disarmed)

        Returns:
            ``True`` when all three are satisfied simultaneously.
        """
        # Cond 1: landed_state
        ext = self._px.recv("EXTENDED_SYS_STATE", timeout=0.05)
        on_ground = False
        if ext is not None:
            on_ground = (ext.landed_state == 1)  # MAV_LANDED_STATE_ON_GROUND

        # Cond 2: TF-02 low for sustained period
        dist_cm = self._tf02.distance_cm
        tf02_low = dist_cm is not None and dist_cm < Config.TOUCHDOWN_ALT_CM

        if tf02_low:
            if self._touchdown_since is None:
                self._touchdown_since = time.time()
        else:
            self._touchdown_since = None

        tf02_stable = (self._touchdown_since is not None and
                       time.time() - self._touchdown_since >= Config.TOUCHDOWN_TIME)

        # Cond 3: Motors disarmed
        disarmed = not self._ds.armed

        return on_ground and tf02_stable and disarmed

    # ── MAVLink Polling ────────────────────────────────────────────────

    def _poll_mavlink(self) -> None:
        """Read pending MAVLink messages and update DroneState."""
        for _ in range(20):  # drain up to 20 messages per tick
            msg = self._px.recv_any(timeout=0.005)
            if msg is None:
                break
            mtype = msg.get_type()

            if mtype == "HEARTBEAT" and msg.get_srcSystem() == Config.SYSTEM_ID:
                armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self._ds.set_armed(armed)
                mode_name = self._resolve_mode(msg.custom_mode)
                self._ds.set_fc_mode(mode_name)

            elif mtype == "SYS_STATUS":
                self._ds.set_battery_volt(msg.voltage_battery / 1000.0)
                self._ds.set_battery_pct(msg.battery_remaining)

            elif mtype == "LOCAL_POSITION_NED":
                # NED: z is negative upward
                alt = -msg.z
                self._ds.set_altitude_m(alt)
                self._ds.set_position(msg.x, msg.y)
                dist = math.sqrt(msg.x ** 2 + msg.y ** 2)
                self._ds.set_dist_to_home_m(dist)

            elif mtype == "VIBRATION":
                self._ds.set_vibration(
                    msg.vibration_x, msg.vibration_y, msg.vibration_z
                )

        # Update TF-02 distance in shared state
        tf02_m = self._tf02.distance_m
        self._ds.set_tf02_dist_m(tf02_m)

    # ── Utility ────────────────────────────────────────────────────────

    @staticmethod
    def _resolve_mode(mode_id: int) -> str:
        """Convert a numeric ArduCopter mode ID back to a name."""
        for name, mid in Config.MODE_MAP.items():
            if mid == mode_id:
                return name
        return f"MODE_{mode_id}"
