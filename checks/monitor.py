"""
ASCEND Phase 1 — AutoTune Monitor
Passive MAVLink monitor during pilot-flown AutoTune flights.
Does NOT control the drone — only observes and reports.
"""

import datetime
import json
import os
import time
from typing import Optional

from pymavlink import mavutil

from ..config import Config
from ..logger import Logger
from ..hardware.pixhawk import PixhawkLink


# ArduCopter mode number for AutoTune
_MODE_AUTOTUNE = Config.MODE_MAP["AUTOTUNE"]


class AutoTuneMonitor:
    """Real-time monitor for pilot-flown AutoTune sessions.

    Watches the MAVLink stream and reports:
      - Armed/disarmed transitions
      - Flight mode changes (especially AutoTune mode 22)
      - Elapsed AutoTune time
      - Live altitude and battery percentage
      - STATUSTEXT messages containing "AutoTune" / "AUTOTUNE"
      - Battery voltage warnings

    On disarm after an AutoTune session the tuned PID values are read
    from the Pixhawk and saved to a timestamped JSON file.

    Args:
        pixhawk: Connected PixhawkLink instance.
    """

    # PID parameters to read after AutoTune completes
    _PID_PARAMS = [
        "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D",
        "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D",
        "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D",
    ]

    def __init__(self, pixhawk: PixhawkLink) -> None:
        self._px = pixhawk
        self._was_armed: bool = False
        self._current_mode: int = -1
        self._autotune_start: Optional[float] = None
        self._autotune_completed: bool = False
        self._running: bool = True

    # ── Public API ─────────────────────────────────────────────────────

    def run(self) -> None:
        """Run the monitor loop until interrupted.

        Blocks the calling thread. Press Ctrl+C to stop.
        """
        Logger.header("AUTOTUNE MONITOR")
        Logger.info("Waiting for MAVLink messages — fly manually, then "
                     "switch to AutoTune mode via RC switch.")
        Logger.info("Press Ctrl+C to stop monitoring.\n")

        self._px.request_data_streams(rate_hz=4)

        try:
            while self._running:
                msg = self._px.recv_any(timeout=0.25)
                if msg is None:
                    continue
                self._process(msg)
        except KeyboardInterrupt:
            Logger.info("Monitor interrupted by user")
        finally:
            Logger.info("AutoTune monitor stopped")

    def stop(self) -> None:
        """Signal the monitor to stop."""
        self._running = False

    # ── Message Dispatch ───────────────────────────────────────────────

    def _process(self, msg: object) -> None:
        """Route a MAVLink message to the appropriate handler."""
        mtype = msg.get_type()
        if mtype == "HEARTBEAT":
            self._handle_heartbeat(msg)
        elif mtype == "SYS_STATUS":
            self._handle_sys_status(msg)
        elif mtype == "GLOBAL_POSITION_INT":
            self._handle_position(msg)
        elif mtype == "STATUSTEXT":
            self._handle_statustext(msg)
        elif mtype == "VFR_HUD":
            self._handle_vfr_hud(msg)

    # ── Heartbeat (armed state + mode tracking) ───────────────────────

    def _handle_heartbeat(self, msg: object) -> None:
        """Detect arming transitions and mode changes."""
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = msg.custom_mode

        # Mode change
        if mode != self._current_mode:
            mode_name = self._mode_name(mode)
            Logger.info(f"Mode changed → {mode_name} (id={mode})")
            if mode == _MODE_AUTOTUNE:
                self._autotune_start = time.time()
                Logger.ok("━━ AutoTune ACTIVE ━━")
            elif self._current_mode == _MODE_AUTOTUNE:
                elapsed = self._autotune_elapsed_str()
                Logger.info(f"Left AutoTune mode after {elapsed}")
            self._current_mode = mode

        # ArduPilot signals armed/disarmed via base_mode
        if armed and not self._was_armed:
            Logger.ok("✦ ARMED")
        elif not armed and self._was_armed:
            Logger.warn("✦ DISARMED")
            self._on_disarm()

        self._was_armed = armed

    # ── SYS_STATUS (battery) ──────────────────────────────────────────

    def _handle_sys_status(self, msg: object) -> None:
        """Print battery information and voltage warnings."""
        voltage = msg.voltage_battery / 1000.0  # mV → V
        remaining = msg.battery_remaining        # %

        if voltage < Config.CRITICAL_BATTERY_VOLT:
            Logger.error(f"CRITICAL BATTERY: {voltage:.2f} V  ({remaining}%)")
        elif voltage < Config.LOW_BATTERY_VOLT:
            Logger.warn(f"LOW BATTERY: {voltage:.2f} V  ({remaining}%)")

        # Print periodic battery info during AutoTune
        if self._autotune_start is not None and self._current_mode == _MODE_AUTOTUNE:
            elapsed = self._autotune_elapsed_str()
            Logger.info(f"  AutoTune {elapsed}  |  "
                         f"batt={voltage:.2f}V ({remaining}%)")

    # ── Position (altitude) ───────────────────────────────────────────

    def _handle_position(self, msg: object) -> None:
        """Report altitude during AutoTune."""
        if self._current_mode == _MODE_AUTOTUNE:
            alt = msg.relative_alt / 1000.0  # mm → m
            Logger.info(f"  alt={alt:.1f} m")

    # ── VFR_HUD (secondary altitude source) ───────────────────────────

    def _handle_vfr_hud(self, msg: object) -> None:
        """Placeholder — could log airspeed, groundspeed, heading."""
        pass  # altitude already covered by GLOBAL_POSITION_INT

    # ── STATUSTEXT ────────────────────────────────────────────────────

    def _handle_statustext(self, msg: object) -> None:
        """Print STATUSTEXT messages, highlight AutoTune-related ones."""
        text = msg.text if isinstance(msg.text, str) else msg.text.decode("utf-8", errors="replace")
        text_upper = text.upper()

        if "AUTOTUNE" in text_upper:
            Logger.ok(f"FC: {text}")
            if "SUCCESS" in text_upper or "COMPLETE" in text_upper:
                self._autotune_completed = True
                Logger.header("AUTOTUNE COMPLETE")
                Logger.ok("Land in AltHold mode to save gains.")
                Logger.warn("Do NOT re-enter AutoTune before landing!")
        else:
            Logger.info(f"FC: {text}")

    # ── Disarm Handler ─────────────────────────────────────────────────

    def _on_disarm(self) -> None:
        """Read and save tuned PID values after disarming."""
        Logger.section("Post-Disarm PID Readout")

        pid_values = {}
        for name in self._PID_PARAMS:
            val = self._px.read_param(name)
            if val is not None:
                pid_values[name] = val
                Logger.kv(f"  {name}", f"{val:.6f}")
            else:
                Logger.warn(f"  {name}: could not read")

        # Save to JSON
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"autotune_results_{ts}.json"
        result = {
            "timestamp": datetime.datetime.now().isoformat(),
            "autotune_completed": self._autotune_completed,
            "pid_values": pid_values,
        }
        try:
            with open(filename, "w") as f:
                json.dump(result, f, indent=2)
            Logger.ok(f"Results saved → {os.path.abspath(filename)}")
        except OSError as exc:
            Logger.error(f"Cannot save results: {exc}")

        # Verdict
        if self._autotune_completed:
            Logger.ok("Verdict: AutoTune COMPLETED successfully ✓")
        else:
            Logger.warn("Verdict: AutoTune NOT completed (no SUCCESS message "
                         "received)")

        # Reset for next session
        self._autotune_start = None
        self._autotune_completed = False

    # ── Utility ────────────────────────────────────────────────────────

    def _autotune_elapsed_str(self) -> str:
        """Return a ``MM:SS`` string for AutoTune elapsed time."""
        if self._autotune_start is None:
            return "00:00"
        elapsed = time.time() - self._autotune_start
        mins, secs = divmod(int(elapsed), 60)
        return f"{mins:02d}:{secs:02d}"

    @staticmethod
    def _mode_name(mode_id: int) -> str:
        """Return ArduCopter mode name for a given mode number."""
        for name, mid in Config.MODE_MAP.items():
            if mid == mode_id:
                return name
        return f"UNKNOWN({mode_id})"
