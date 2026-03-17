"""
ASCEND Phase 1 — Daemon Thread Workers
Provides four background daemons that run alongside the main state machine:
  • RangefinderBridge — TF-02 → Pixhawk at 20 Hz
  • HeartbeatSender  — companion heartbeat at 1 Hz
  • TelemetryStreamer — JSON telemetry to laptop over UDP at 1 Hz
  • SafetyMonitor    — continuous fault detection
"""

import datetime
import json
import socket
import threading
import time
from typing import Any, Callable, Optional

from ..config import Config
from ..logger import Logger
from ..hardware.tf02 import TF02Reader
from ..hardware.pixhawk import PixhawkLink


# ═══════════════════════════════════════════════════════════════════════════
# Rangefinder Bridge
# ═══════════════════════════════════════════════════════════════════════════

class RangefinderBridge(threading.Thread):
    """Forwards TF-02 distance readings to the Pixhawk EKF at 20 Hz.

    Reads ``TF02Reader.distance_cm`` and calls
    ``PixhawkLink.send_distance_sensor()`` for every valid reading.

    Args:
        tf02: Shared TF02Reader instance.
        pixhawk: Shared PixhawkLink instance.
    """

    def __init__(self, tf02: TF02Reader, pixhawk: PixhawkLink) -> None:
        super().__init__(daemon=True, name="RangefinderBridge")
        self._tf02 = tf02
        self._pixhawk = pixhawk
        self._running = threading.Event()
        self._running.set()

    def stop(self) -> None:
        """Signal the bridge thread to stop."""
        self._running.clear()

    def run(self) -> None:
        """Continuously forward distance readings."""
        Logger.info(f"RangefinderBridge running at {Config.BRIDGE_HZ} Hz")
        while self._running.is_set():
            dist = self._tf02.distance_cm
            if dist is not None:
                try:
                    self._pixhawk.send_distance_sensor(dist)
                except Exception as exc:
                    Logger.warn(f"Bridge send error: {exc}")
            time.sleep(Config.BRIDGE_INTERVAL)
        Logger.info("RangefinderBridge stopped")


# ═══════════════════════════════════════════════════════════════════════════
# Heartbeat Sender
# ═══════════════════════════════════════════════════════════════════════════

class HeartbeatSender(threading.Thread):
    """Sends a companion-computer MAVLink heartbeat at 1 Hz.

    Must run during ALL flight phases to prevent the Pixhawk
    ``FS_GCS_ENABLE`` failsafe from triggering.

    Args:
        pixhawk: Shared PixhawkLink instance.
    """

    def __init__(self, pixhawk: PixhawkLink) -> None:
        super().__init__(daemon=True, name="HeartbeatSender")
        self._pixhawk = pixhawk
        self._running = threading.Event()
        self._running.set()

    def stop(self) -> None:
        """Signal the heartbeat thread to stop."""
        self._running.clear()

    def run(self) -> None:
        """Continuously send heartbeats."""
        Logger.info("HeartbeatSender running at 1 Hz")
        while self._running.is_set():
            try:
                self._pixhawk.send_heartbeat()
            except Exception as exc:
                Logger.warn(f"Heartbeat send error: {exc}")
            time.sleep(Config.HEARTBEAT_INTERVAL)
        Logger.info("HeartbeatSender stopped")


# ═══════════════════════════════════════════════════════════════════════════
# Telemetry Streamer
# ═══════════════════════════════════════════════════════════════════════════

class TelemetryStreamer(threading.Thread):
    """Streams live telemetry as JSON over UDP to the laptop at 1 Hz.

    The laptop runs as a Wi-Fi hotspot; RPi5 is a client. Telemetry
    is sent as one JSON packet per second on the configured UDP port.

    Args:
        state_getter: Callable returning a dict with all current telemetry
                      fields (populated by the state machine or scheduler).
    """

    def __init__(self, state_getter: Callable[[], dict]) -> None:
        super().__init__(daemon=True, name="TelemetryStreamer")
        self._get_state = state_getter
        self._running = threading.Event()
        self._running.set()
        self._sock: Optional[socket.socket] = None

    def stop(self) -> None:
        """Signal the streamer thread to stop."""
        self._running.clear()

    def run(self) -> None:
        """Open a UDP socket and stream telemetry."""
        Logger.info(f"TelemetryStreamer → {Config.LAPTOP_IP}:{Config.TELEMETRY_PORT}")
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except OSError as exc:
            Logger.error(f"TelemetryStreamer cannot create socket: {exc}")
            return

        dest = (Config.LAPTOP_IP, Config.TELEMETRY_PORT)
        while self._running.is_set():
            try:
                state = self._get_state()
                state["timestamp"] = datetime.datetime.now().isoformat()
                packet = json.dumps(state).encode("utf-8")
                self._sock.sendto(packet, dest)
            except Exception:
                # Laptop may not be reachable — silently continue
                pass
            time.sleep(Config.TELEMETRY_INTERVAL)

        self._sock.close()
        Logger.info("TelemetryStreamer stopped")


# ═══════════════════════════════════════════════════════════════════════════
# Safety Monitor
# ═══════════════════════════════════════════════════════════════════════════

class SafetyMonitor(threading.Thread):
    """Continuously monitors three failure conditions and sets an emergency flag.

    Monitored conditions:
        1. **TF-02 data age** — no valid reading for > 2 s (sensor failure).
        2. **Wi-Fi heartbeat age** — no laptop packet for > 3 s (link loss).
        3. **Geofence breach** — position exceeds ``FENCE_X_M`` / ``FENCE_Y_M``.

    The main ``StateMachine`` polls ``emergency_flag`` on every iteration.

    Args:
        tf02: Shared TF02Reader instance.
        pixhawk: Shared PixhawkLink instance.
    """

    def __init__(self, tf02: TF02Reader, pixhawk: PixhawkLink) -> None:
        super().__init__(daemon=True, name="SafetyMonitor")
        self._tf02 = tf02
        self._pixhawk = pixhawk
        self._running = threading.Event()
        self._running.set()

        # Thread-safe shared state
        self._lock = threading.Lock()
        self._emergency_flag: bool = False
        self._emergency_reason: str = ""

        # Wi-Fi heartbeat tracking — updated externally
        self._last_wifi_hb: float = time.time()

        # Position tracking — updated from main loop
        self._pos_x: float = 0.0
        self._pos_y: float = 0.0

        # Flight-active flag — safety checks only run when armed & flying
        self._flight_active: bool = False

    # ── Public Properties ──────────────────────────────────────────────

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

    def set_flight_active(self, active: bool) -> None:
        """Enable or disable in-flight safety checks.

        Args:
            active: ``True`` when the drone is armed and flying.
        """
        with self._lock:
            self._flight_active = active

    def update_wifi_heartbeat(self) -> None:
        """Mark receipt of a Wi-Fi heartbeat from the laptop."""
        with self._lock:
            self._last_wifi_hb = time.time()

    def update_position(self, x: float, y: float) -> None:
        """Update the latest LOCAL_POSITION_NED values.

        Args:
            x: North position in metres from home.
            y: East position in metres from home.
        """
        with self._lock:
            self._pos_x = x
            self._pos_y = y

    def reset(self) -> None:
        """Clear the emergency flag (e.g. after landing)."""
        with self._lock:
            self._emergency_flag = False
            self._emergency_reason = ""

    def stop(self) -> None:
        """Signal the monitor thread to stop."""
        self._running.clear()

    # ── Trigger ────────────────────────────────────────────────────────

    def _trigger(self, reason: str) -> None:
        """Set the emergency flag with the given reason (once only)."""
        with self._lock:
            if not self._emergency_flag:
                self._emergency_flag = True
                self._emergency_reason = reason
                Logger.error(f"⚠ EMERGENCY: {reason}")

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        """Poll safety conditions at 10 Hz."""
        Logger.info("SafetyMonitor running")
        while self._running.is_set():
            with self._lock:
                flight_active = self._flight_active

            if flight_active:
                self._check_tf02()
                self._check_wifi()
                self._check_geofence()

            time.sleep(0.1)
        Logger.info("SafetyMonitor stopped")

    def _check_tf02(self) -> None:
        """Trigger if TF-02 data is stale (> 2 s without valid reading)."""
        if self._tf02.data_age > Config.TF02_DATA_TIMEOUT:
            self._trigger("TF-02 sensor failure — no data for "
                          f"{self._tf02.data_age:.1f}s")

    def _check_wifi(self) -> None:
        """Trigger if laptop Wi-Fi heartbeat is stale (> 3 s)."""
        with self._lock:
            age = time.time() - self._last_wifi_hb
        if age > Config.WIFI_HB_TIMEOUT:
            self._trigger(f"Wi-Fi link loss — no heartbeat for {age:.1f}s")

    def _check_geofence(self) -> None:
        """Trigger if LOCAL_POSITION_NED exceeds the virtual fence."""
        with self._lock:
            x, y = self._pos_x, self._pos_y
        if abs(x) > Config.FENCE_X_M or abs(y) > Config.FENCE_Y_M:
            self._trigger(f"Geofence breach — pos=({x:.1f}, {y:.1f}) "
                          f"limits=±({Config.FENCE_X_M}, {Config.FENCE_Y_M})")
