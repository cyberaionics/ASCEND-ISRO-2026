"""
ASCEND Phase 2 — Pixhawk MAVLink Interface
Thread-safe pymavlink wrapper for all flight-controller communication.
"""

import threading
import time
from typing import Any, Optional

from pymavlink import mavutil

from ..config import Config
from ..logger import Logger


class PixhawkLink:
    """Thread-safe MAVLink connection to a Pixhawk flight controller.

    All outbound messages are serialised through a send-lock so multiple
    threads (heartbeat, bridge, state machine, TX watcher) can share one
    connection safely.
    """

    def __init__(self) -> None:
        self._conn: Optional[Any] = None
        self._send_lock = threading.Lock()
        self._connected = False

    # ── Connection ─────────────────────────────────────────────────────

    def connect(self, timeout: float = Config.CONNECT_TIMEOUT) -> bool:
        """Open serial link and wait for a Pixhawk heartbeat.

        Returns:
            ``True`` if connected and heartbeat received.
        """
        Logger.info(f"Connecting to Pixhawk on {Config.PIXHAWK_PORT} "
                     f"@ {Config.PIXHAWK_BAUD} …")
        try:
            self._conn = mavutil.mavlink_connection(
                Config.PIXHAWK_PORT,
                baud=Config.PIXHAWK_BAUD,
                source_system=Config.COMPANION_SYSID,
                source_component=Config.COMPANION_COMPID,
            )
        except Exception as exc:
            Logger.error(f"Cannot open Pixhawk port: {exc}")
            return False

        Logger.info(f"Waiting up to {timeout:.0f}s for heartbeat …")
        hb = self._conn.wait_heartbeat(timeout=timeout)
        if hb is None:
            Logger.error("No heartbeat — check USB cable and power.")
            return False

        self._connected = True
        Logger.ok(f"Heartbeat from system {self._conn.target_system}")
        return True

    @property
    def connected(self) -> bool:
        return self._connected

    def close(self) -> None:
        """Close the MAVLink connection."""
        if self._conn is not None:
            self._conn.close()
            self._connected = False
            Logger.info("Pixhawk connection closed")

    # ── Heartbeat ──────────────────────────────────────────────────────

    def send_heartbeat(self) -> None:
        """Send a companion-computer heartbeat (prevents FS_GCS_ENABLE)."""
        if self._conn is None:
            return
        with self._send_lock:
            self._conn.mav.heartbeat_send(
                type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=mavutil.mavlink.MAV_STATE_ACTIVE,
            )

    # ── Rangefinder ────────────────────────────────────────────────────

    def send_distance_sensor(self, distance_cm: int) -> None:
        """Forward a TF-02 reading to the Pixhawk EKF.

        Args:
            distance_cm: Distance in centimetres.
        """
        if self._conn is None:
            return
        dist = max(Config.TF02_MIN_CM, min(distance_cm, Config.TF02_MAX_CM))
        with self._send_lock:
            self._conn.mav.distance_sensor_send(
                time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
                min_distance=Config.TF02_MIN_CM,
                max_distance=Config.TF02_MAX_CM,
                current_distance=dist,
                type=mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
                id=0,
                orientation=mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270,
                covariance=0,
            )

    # ── Parameters ─────────────────────────────────────────────────────

    def read_param(self, name: str,
                   timeout: float = Config.PARAM_TIMEOUT) -> Optional[float]:
        """Read a single parameter from the Pixhawk.

        Returns:
            Parameter value as float, or ``None`` on failure.
        """
        if self._conn is None:
            return None
        with self._send_lock:
            self._conn.mav.param_request_read_send(
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                param_id=name.encode("utf-8"),
                param_index=-1,
            )
        msg = self.recv("PARAM_VALUE", timeout=timeout)
        if msg is not None and msg.param_id.rstrip("\x00") == name:
            return msg.param_value
        return None

    def write_param(self, name: str, value: float,
                    timeout: float = Config.PARAM_TIMEOUT) -> bool:
        """Write a parameter and verify with ACK.

        Returns:
            ``True`` if confirmed.
        """
        if self._conn is None:
            return False
        with self._send_lock:
            self._conn.mav.param_set_send(
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                param_id=name.encode("utf-8"),
                param_value=float(value),
                param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.recv("PARAM_VALUE", timeout=0.5)
            if msg and msg.param_id.rstrip("\x00") == name:
                if abs(msg.param_value - float(value)) < 0.01:
                    return True
                return False
        return False

    # ── Flight Commands ────────────────────────────────────────────────

    def set_mode(self, mode_name: str) -> bool:
        """Set an ArduCopter flight mode by name.

        Returns:
            ``True`` if the command was sent.
        """
        mode_id = Config.MODE_MAP.get(mode_name.upper())
        if mode_id is None:
            Logger.error(f"Unknown mode: {mode_name}")
            return False
        if self._conn is None:
            return False
        with self._send_lock:
            self._conn.mav.set_mode_send(
                target_system=self._conn.target_system,
                base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode=mode_id,
            )
        Logger.info(f"SET_MODE → {mode_name.upper()}")
        return True

    def arm(self) -> bool:
        """Send arm command."""
        if self._conn is None:
            return False
        with self._send_lock:
            self._conn.mav.command_long_send(
                self._conn.target_system, self._conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0,
            )
        Logger.info("ARM command sent")
        return True

    def disarm(self) -> bool:
        """Send disarm command."""
        if self._conn is None:
            return False
        with self._send_lock:
            self._conn.mav.command_long_send(
                self._conn.target_system, self._conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 0, 0, 0, 0, 0, 0,
            )
        Logger.info("DISARM command sent")
        return True

    def takeoff(self, alt_m: float) -> bool:
        """Send MAV_CMD_NAV_TAKEOFF.

        Args:
            alt_m: Target altitude in metres AGL.
        """
        if self._conn is None:
            return False
        with self._send_lock:
            self._conn.mav.command_long_send(
                self._conn.target_system, self._conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, alt_m,
            )
        Logger.info(f"TAKEOFF → {alt_m:.1f} m")
        return True

    # ── MAVLink Receive ────────────────────────────────────────────────

    def recv(self, msg_type: str, timeout: float = 1.0) -> Optional[Any]:
        """Wait for a specific MAVLink message type.

        Returns:
            Message object, or ``None`` on timeout.
        """
        if self._conn is None:
            return None
        try:
            return self._conn.recv_match(type=msg_type, blocking=True,
                                          timeout=timeout)
        except Exception:
            return None

    def recv_any(self, timeout: float = 0.05) -> Optional[Any]:
        """Receive the next MAVLink message of any type.

        Returns:
            Message object, or ``None``.
        """
        if self._conn is None:
            return None
        try:
            return self._conn.recv_match(blocking=True, timeout=timeout)
        except Exception:
            return None

    # ── Data Streams ───────────────────────────────────────────────────

    def request_data_streams(self, rate_hz: int = 10) -> None:
        """Request all standard telemetry streams from Pixhawk."""
        if self._conn is None:
            return
        with self._send_lock:
            self._conn.mav.request_data_stream_send(
                self._conn.target_system, self._conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                rate_hz, 1,
            )
        Logger.info(f"Requested data streams at {rate_hz} Hz")
