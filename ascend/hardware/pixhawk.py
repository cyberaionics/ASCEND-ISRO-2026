"""
ASCEND Phase 1 — Pixhawk MAVLink Interface
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

    All outbound MAVLink messages are serialised through a send-lock so
    multiple threads (heartbeat, bridge, state machine) can share one
    connection safely.

    Typical lifecycle::

        px = PixhawkLink()
        px.connect()
        px.send_heartbeat()
        px.send_distance_sensor(350)
        px.set_mode("GUIDED")
        msg = px.recv("HEARTBEAT", timeout=3.0)
        px.close()
    """

    def __init__(self) -> None:
        self._conn: Optional[Any] = None
        self._send_lock = threading.Lock()
        self._connected = False

    # ── Connection ─────────────────────────────────────────────────────

    def connect(self, timeout: float = Config.CONNECT_TIMEOUT) -> bool:
        """Open the serial link and wait for a Pixhawk heartbeat.

        Args:
            timeout: Maximum seconds to wait for the first heartbeat.

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
            Logger.error("No heartbeat received — check USB cable and power.")
            return False

        self._connected = True
        Logger.ok(f"Heartbeat from system {self._conn.target_system}, "
                   f"component {self._conn.target_component}")
        return True

    @property
    def connected(self) -> bool:
        """``True`` if a heartbeat has been received."""
        return self._connected

    def close(self) -> None:
        """Close the MAVLink connection."""
        if self._conn is not None:
            self._conn.close()
            self._connected = False
            Logger.info("Pixhawk connection closed")

    # ── Heartbeat ──────────────────────────────────────────────────────

    def send_heartbeat(self) -> None:
        """Send a companion-computer heartbeat to prevent FS_GCS_ENABLE."""
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

    # ── Rangefinder Bridge ─────────────────────────────────────────────

    def send_distance_sensor(self, distance_cm: int) -> None:
        """Forward a TF-02 reading to the Pixhawk EKF as DISTANCE_SENSOR.

        Args:
            distance_cm: Distance in centimetres (clamped to valid range).
        """
        if self._conn is None:
            return
        # Clamp to sensor limits
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

    # ── Parameter Read / Write ─────────────────────────────────────────

    def read_param(self, name: str, timeout: float = Config.PARAM_TIMEOUT) -> Optional[float]:
        """Read a single parameter value from the Pixhawk.

        Args:
            name: Parameter name string (e.g. ``"RNGFND1_TYPE"``).
            timeout: Maximum seconds to wait for response.

        Returns:
            Parameter value as float, or ``None`` on failure.
        """
        if self._conn is None:
            return None
        name_bytes = name.encode("utf-8")
        with self._send_lock:
            self._conn.mav.param_request_read_send(
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                param_id=name_bytes,
                param_index=-1,
            )
        msg = self.recv("PARAM_VALUE", timeout=timeout)
        if msg is not None and msg.param_id.rstrip("\x00") == name:
            return msg.param_value
        return None

    def write_param(self, name: str, value: float,
                    timeout: float = Config.PARAM_TIMEOUT) -> bool:
        """Write a parameter to the Pixhawk and verify with PARAM_VALUE ACK.

        Args:
            name: Parameter name string.
            value: Desired value.
            timeout: Maximum seconds to wait for ACK.

        Returns:
            ``True`` if the parameter was successfully written and confirmed.
        """
        if self._conn is None:
            return False
        name_bytes = name.encode("utf-8")
        # Determine param type — integer params use MAV_PARAM_TYPE_INT32
        if isinstance(value, int) or (isinstance(value, float) and value == int(value)
                                       and "VOLT" not in name
                                       and "SPIN" not in name
                                       and "HOVER" not in name
                                       and "AGGR" not in name
                                       and "MIN_D" not in name):
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        else:
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32

        with self._send_lock:
            self._conn.mav.param_set_send(
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                param_id=name_bytes,
                param_value=float(value),
                param_type=param_type,
            )

        # Wait for ACK (PARAM_VALUE echo)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.recv("PARAM_VALUE", timeout=0.5)
            if msg is not None and msg.param_id.rstrip("\x00") == name:
                # Verify value (allow small float tolerance)
                if abs(msg.param_value - float(value)) < 0.01:
                    return True
                Logger.warn(f"Param {name}: wrote {value}, read back {msg.param_value}")
                return False
        return False

    # ── Flight Commands ────────────────────────────────────────────────

    def set_mode(self, mode_name: str) -> bool:
        """Command Pixhawk to switch to an ArduCopter flight mode.

        Args:
            mode_name: One of the keys in ``Config.MODE_MAP``
                       (e.g. ``"GUIDED"``, ``"LAND"``).

        Returns:
            ``True`` if the command was sent (no ACK verification).
        """
        mode_id = Config.MODE_MAP.get(mode_name.upper())
        if mode_id is None:
            Logger.error(f"Unknown flight mode: {mode_name}")
            return False
        if self._conn is None:
            return False

        with self._send_lock:
            self._conn.mav.set_mode_send(
                target_system=self._conn.target_system,
                base_mode=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode=mode_id,
            )
        Logger.info(f"SET_MODE → {mode_name.upper()} (id={mode_id})")
        return True

    def arm(self) -> bool:
        """Send arm command to the Pixhawk.

        Returns:
            ``True`` if the command was sent.
        """
        if self._conn is None:
            return False
        with self._send_lock:
            self._conn.mav.command_long_send(
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                confirmation=0,
                param1=1,   # 1 = arm
                param2=0,
                param3=0, param4=0, param5=0, param6=0, param7=0,
            )
        Logger.info("ARM command sent")
        return True

    def disarm(self) -> bool:
        """Send disarm command to the Pixhawk.

        Returns:
            ``True`` if the command was sent.
        """
        if self._conn is None:
            return False
        with self._send_lock:
            self._conn.mav.command_long_send(
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                confirmation=0,
                param1=0,   # 0 = disarm
                param2=0,
                param3=0, param4=0, param5=0, param6=0, param7=0,
            )
        Logger.info("DISARM command sent")
        return True

    def takeoff(self, alt_m: float) -> bool:
        """Send MAV_CMD_NAV_TAKEOFF to the Pixhawk.

        Args:
            alt_m: Target altitude in metres above ground.

        Returns:
            ``True`` if the command was sent.
        """
        if self._conn is None:
            return False
        with self._send_lock:
            self._conn.mav.command_long_send(
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                confirmation=0,
                param1=0,              # min pitch (ignored for copter)
                param2=0, param3=0, param4=0, param5=0, param6=0,
                param7=alt_m,          # target altitude MSL (copter uses AGL)
            )
        Logger.info(f"TAKEOFF command sent → {alt_m:.1f} m")
        return True

    # ── MAVLink Receive ────────────────────────────────────────────────

    def recv(self, msg_type: str, timeout: float = 1.0) -> Optional[Any]:
        """Wait for a MAVLink message of the given type.

        Args:
            msg_type: MAVLink message type string (e.g. ``"HEARTBEAT"``).
            timeout: Maximum seconds to wait.

        Returns:
            The MAVLink message object, or ``None`` on timeout.
        """
        if self._conn is None:
            return None
        try:
            msg = self._conn.recv_match(type=msg_type, blocking=True,
                                         timeout=timeout)
            return msg
        except Exception:
            return None

    def recv_any(self, timeout: float = 0.1) -> Optional[Any]:
        """Receive the next MAVLink message of any type.

        Args:
            timeout: Maximum seconds to wait.

        Returns:
            A MAVLink message, or ``None``.
        """
        if self._conn is None:
            return None
        try:
            msg = self._conn.recv_match(blocking=True, timeout=timeout)
            return msg
        except Exception:
            return None

    # ── Request Data Streams ───────────────────────────────────────────

    def request_data_streams(self, rate_hz: int = 4) -> None:
        """Ask Pixhawk to send common telemetry streams at the given rate.

        Args:
            rate_hz: Messages per second for each stream.
        """
        if self._conn is None:
            return
        streams = [
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
        ]
        for stream_id in streams:
            with self._send_lock:
                self._conn.mav.request_data_stream_send(
                    target_system=self._conn.target_system,
                    target_component=self._conn.target_component,
                    req_stream_id=stream_id,
                    req_message_rate=rate_hz,
                    start_stop=1,
                )
        Logger.info(f"Requested data streams at {rate_hz} Hz")
    def send_velocity(self, vx: float, vy: float, vz: float) -> None:
        """Send body-frame velocity command for GUIDED_NOGPS mode.

        Args:
            vx: Forward velocity in m/s (NED frame).
            vy: Right velocity in m/s (NED frame).
            vz: Down velocity in m/s (negative = up in NED).
        """
        if self._conn is None:
            return
        with self._send_lock:
            self._conn.mav.set_position_target_local_ned_send(
                time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                type_mask=0b0000111111000111,  # ignore position & accel, use velocity only
                x=0, y=0, z=0,
                vx=vx, vy=vy, vz=vz,
                afx=0, afy=0, afz=0,
                yaw=0, yaw_rate=0,
            )
    def send_rc_override(self, roll: int = 1500, pitch: int = 1500,
                          throttle: int = 1500, yaw: int = 1500) -> None:
        """Send fake RC PWM override to Pixhawk.

        Args:
            roll:     PWM 1000-2000 (1500 = neutral)
            pitch:    PWM 1000-2000 (1500 = neutral)
            throttle: PWM 1000-2000 (1500 = mid, 1600 = climb)
            yaw:      PWM 1000-2000 (1500 = neutral)
        """
        if self._conn is None:
            return
        with self._send_lock:
            self._conn.mav.rc_channels_override_send(
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                chan1_raw=roll,
                chan2_raw=pitch,
                chan3_raw=throttle,
                chan4_raw=yaw,
                chan5_raw=0,
                chan6_raw=0,
                chan7_raw=0,
                chan8_raw=0,
            )

    def clear_rc_override(self) -> None:
        """Release all RC overrides — hands full control back to the RC transmitter.

        Sends 0 on all channels which tells ArduCopter to stop using
        companion-computer RC values and use the real TX instead.
        """
        if self._conn is None:
            return
        with self._send_lock:
            self._conn.mav.rc_channels_override_send(
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                chan1_raw=0,
                chan2_raw=0,
                chan3_raw=0,
                chan4_raw=0,
                chan5_raw=0,
                chan6_raw=0,
                chan7_raw=0,
                chan8_raw=0,
            )
        Logger.info("RC override cleared — TX has full control")
