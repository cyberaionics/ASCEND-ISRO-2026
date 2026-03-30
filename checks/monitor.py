"""
ASCEND — Flight Safety Monitor
Real-time daemon that monitors sensor health and triggers BRAKE mode
on failure.

Monitored conditions:
  1. ESP32-CAM stream latency:  > 250 ms → BRAKE
  2. TF-02 Lidar data age:     > 250 ms → BRAKE
  3. TF-02 health check:       signal strength failure → BRAKE

The monitor reads timestamps from the SensorFusionBridge to determine
if data streams are stale.  On any failure, it sends MAV_CMD_DO_SET_MODE
with BRAKE mode to immediately stop the drone.

Runs at 20 Hz in its own daemon thread.

Note: The original AutoTuneMonitor class is preserved at the bottom of
this file for backward compatibility.
"""

import datetime
import json
import os
import threading
import time
from typing import Optional

from pymavlink import mavutil

from ..config import Config
from ..logger import Logger
from ..hardware.pixhawk import PixhawkLink
from ..hardware.tf02 import TF02Reader


# ═══════════════════════════════════════════════════════════════════════════
# Flight Safety Monitor — replaces passive AutoTune monitor
# ═══════════════════════════════════════════════════════════════════════════

class FlightSafetyMonitor(threading.Thread):
    """Active safety daemon that monitors sensor heartbeats at 20 Hz.

    Checks:
      1. ESP32-CAM stream latency (from SensorFusionBridge.last_esp32_time)
      2. TF-02 LiDAR data age (from TF02Reader.data_age)
      3. TF-02 signal strength health

    If any stream latency exceeds 250 ms OR lidar health fails,
    immediately sends MAV_CMD_DO_SET_MODE → BRAKE mode and sets
    the emergency flag.

    Args:
        pixhawk:       Connected PixhawkLink instance.
        tf02:          Running TF02Reader instance.
        fusion_bridge: Running SensorFusionBridge instance (provides timestamps).
    """

    def __init__(self, pixhawk: PixhawkLink, tf02: TF02Reader,
                 fusion_bridge: object) -> None:
        super().__init__(daemon=True, name="FlightSafetyMonitor")
        self._px = pixhawk
        self._tf02 = tf02
        self._fusion = fusion_bridge

        self._running = threading.Event()
        self._running.set()
        self._lock = threading.Lock()

        self._emergency_flag: bool = False
        self._emergency_reason: str = ""
        self._flight_active: bool = False

        # Prevent duplicate triggers
        self._triggered: bool = False

        # Diagnostic
        self._last_log: float = 0.0
        self._check_count: int = 0

    # ── Thread-safe Properties ─────────────────────────────────────────

    @property
    def emergency_flag(self) -> bool:
        """``True`` if an emergency condition has been detected."""
        with self._lock:
            return self._emergency_flag

    @property
    def emergency_reason(self) -> str:
        """Human-readable reason for the emergency."""
        with self._lock:
            return self._emergency_reason

    def get_emergency(self) -> bool:
        """Callable interface for MissionController safety check."""
        return self.emergency_flag

    def set_flight_active(self, active: bool) -> None:
        """Enable or disable in-flight safety checks.

        Args:
            active: ``True`` when the drone is armed and flying.
        """
        with self._lock:
            self._flight_active = active
            if not active:
                self._triggered = False

    def reset(self) -> None:
        """Clear the emergency flag (e.g. after landing)."""
        with self._lock:
            self._emergency_flag = False
            self._emergency_reason = ""
            self._triggered = False

    def stop(self) -> None:
        """Signal the monitor thread to stop."""
        self._running.clear()

    # ── Emergency Trigger ──────────────────────────────────────────────

    def _trigger_brake(self, reason: str) -> None:
        """Send BRAKE mode and set emergency flag.

        Only triggers once per flight — avoids flooding the FC with
        mode change commands.
        """
        with self._lock:
            if self._triggered:
                return
            self._triggered = True
            self._emergency_flag = True
            self._emergency_reason = reason

        Logger.error(f"⚠ SAFETY BRAKE: {reason}")
        Logger.error("Sending MAV_CMD_DO_SET_MODE → BRAKE")

        # Send BRAKE mode via set_mode (which uses MAV_CMD_DO_SET_MODE)
        try:
            self._px.set_mode("BRAKE")
        except Exception as exc:
            Logger.error(f"BRAKE mode failed: {exc} — trying LAND")
            try:
                self._px.set_mode("LAND")
            except Exception:
                Logger.error("LAND mode also failed — critical failure")

    # ── Health Checks ──────────────────────────────────────────────────

    def _check_esp32_heartbeat(self) -> None:
        """Check ESP32-CAM stream latency.

        Triggers BRAKE if no data received for > 250 ms.
        """
        if self._fusion is None:
            return

        last_time = self._fusion.last_esp32_time
        if last_time == 0.0:
            return  # not yet initialized

        latency = time.time() - last_time
        if latency > Config.STREAM_LATENCY_MAX_S:
            self._trigger_brake(
                f"ESP32-CAM stream stale — latency={latency*1000:.0f}ms "
                f"> {Config.STREAM_LATENCY_MAX_S*1000:.0f}ms limit"
            )

    def _check_lidar_heartbeat(self) -> None:
        """Check TF-02 LiDAR data age.

        Triggers BRAKE if data_age > 250 ms.
        """
        data_age = self._tf02.data_age
        if data_age == float("inf"):
            return  # not yet initialized

        if data_age > Config.STREAM_LATENCY_MAX_S:
            self._trigger_brake(
                f"TF-02 LiDAR stale — data_age={data_age*1000:.0f}ms "
                f"> {Config.STREAM_LATENCY_MAX_S*1000:.0f}ms limit"
            )

    def _check_lidar_health(self) -> None:
        """Check TF-02 signal strength and range validity.

        Triggers BRAKE if readings are consistently out of range.
        """
        dist_cm = self._tf02.distance_cm
        strength = self._tf02.strength

        if dist_cm is None:
            return  # no data yet

        # Signal strength too low (sensor obstruction or malfunction)
        if strength < 50 and dist_cm > Config.TF02_MAX_CM:
            self._trigger_brake(
                f"TF-02 health check FAILED — "
                f"dist={dist_cm}cm, strength={strength} "
                f"(out of range or obstructed)"
            )

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        """Poll safety conditions at 20 Hz."""
        Logger.info(
            f"FlightSafetyMonitor running at {Config.SAFETY_MONITOR_HZ} Hz | "
            f"latency limit: {Config.STREAM_LATENCY_MAX_S*1000:.0f}ms"
        )
        interval = 1.0 / Config.SAFETY_MONITOR_HZ

        while self._running.is_set():
            with self._lock:
                flight_active = self._flight_active

            if flight_active and not self._triggered:
                self._check_esp32_heartbeat()
                self._check_lidar_heartbeat()
                self._check_lidar_health()
                self._check_count += 1

            # Periodic diagnostic log
            now = time.time()
            if now - self._last_log >= 5.0 and flight_active:
                self._last_log = now
                esp32_age = (time.time() - self._fusion.last_esp32_time) * 1000 \
                    if self._fusion and self._fusion.last_esp32_time > 0 else -1
                lidar_age = self._tf02.data_age * 1000
                Logger.info(
                    f"SAFETY | checks={self._check_count} | "
                    f"esp32_latency={esp32_age:.0f}ms | "
                    f"lidar_latency={lidar_age:.0f}ms | "
                    f"status={'ARMED' if not self._triggered else 'TRIGGERED'}"
                )

            time.sleep(interval)

        Logger.info("FlightSafetyMonitor stopped")


# ═══════════════════════════════════════════════════════════════════════════
# AutoTune Monitor (kept for backward compatibility)
# ═══════════════════════════════════════════════════════════════════════════

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
