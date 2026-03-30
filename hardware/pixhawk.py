"""
ASCEND Phase 1 — Pixhawk MAVLink Interface
Thread-safe pymavlink wrapper for all flight-controller communication.
"""

import math
import queue
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
        self._telemetry_queue: Optional[queue.Queue] = None

    def set_telemetry_queue(self, q: queue.Queue) -> None:
        """Register a queue to receive raw MAVLink message bytes."""
        self._telemetry_queue = q

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

    # ── GUIDED_NOGPS Attitude / Position Targets ───────────────────────

    def send_attitude_target(self, roll_rad: float, pitch_rad: float,
                              yaw_rad: float, thrust: float) -> None:
        """Send SET_ATTITUDE_TARGET for GUIDED_NOGPS body-frame control.

        Commands the flight controller to hold the given attitude angles
        and thrust level.  This is the primary control method for indoor
        non-GPS flight — replaces RC channel overrides.

        Args:
            roll_rad:  Desired roll angle in radians (positive = right).
            pitch_rad: Desired pitch angle in radians (positive = nose down).
            yaw_rad:   Desired yaw angle in radians (0 = north, positive = CW).
            thrust:    Normalised collective thrust (0.0 … 1.0).
        """
        if self._conn is None:
            return

        # Clamp thrust to [0, 1]
        thrust = max(0.0, min(1.0, thrust))

        # Convert Euler angles (ZYX convention) → quaternion [w, x, y, z]
        cr = math.cos(roll_rad  * 0.5)
        sr = math.sin(roll_rad  * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cy = math.cos(yaw_rad   * 0.5)
        sy = math.sin(yaw_rad   * 0.5)

        q = [
            cr * cp * cy + sr * sp * sy,   # w
            sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy,   # z
        ]

        # type_mask: ignore body roll/pitch/yaw RATES (bits 0,1,2)
        # We command attitude quaternion + thrust only.
        type_mask = 0b00000111  # ignore body_roll_rate, body_pitch_rate, body_yaw_rate

        with self._send_lock:
            self._conn.mav.set_attitude_target_send(
                time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                type_mask=type_mask,
                q=q,
                body_roll_rate=0.0,
                body_pitch_rate=0.0,
                body_yaw_rate=0.0,
                thrust=thrust,
            )

    def send_position_target_local_ned(self, vx: float = 0.0,
                                        vy: float = 0.0,
                                        vz: float = 0.0) -> None:
        """Send SET_POSITION_TARGET_LOCAL_NED in velocity-only mode.

        Used during landing to command a steady descent rate.  Only
        velocity fields are set; position and acceleration are ignored.

        Args:
            vx: North velocity in m/s.
            vy: East velocity in m/s.
            vz: Down velocity in m/s (positive = descend).
        """
        if self._conn is None:
            return

        # type_mask: ignore pos (bits 0,1,2), accel (bits 6,7,8),
        #            yaw (bit 10), yaw_rate (bit 11)
        # Only use velocity (bits 3,4,5 = 0 → use them)
        type_mask = 0b0000110111000111

        with self._send_lock:
            self._conn.mav.set_position_target_local_ned_send(
                time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_NED,
                type_mask=type_mask,
                x=0.0, y=0.0, z=0.0,
                vx=vx, vy=vy, vz=vz,
                afx=0.0, afy=0.0, afz=0.0,
                yaw=0.0,
                yaw_rate=0.0,
            )

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
            if msg is not None and self._telemetry_queue is not None:
                try:
                    self._telemetry_queue.put_nowait(msg.get_msgbuf())
                except Exception:
                    pass
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
            if msg is not None and self._telemetry_queue is not None:
                try:
                    self._telemetry_queue.put_nowait(msg.get_msgbuf())
                except Exception:
                    pass
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

    # ── EKF3 Initialization (GPS-denied indoor flight) ─────────────────

    def send_gps_global_origin(self, lat: int = 0, lon: int = 0,
                                alt: int = 0) -> None:
        """Send SET_GPS_GLOBAL_ORIGIN to bootstrap EKF3 without GPS.

        This is **mandatory** for EKF3 alignment in GPS-denied mode.
        Must be sent before any ODOMETRY messages.

        Args:
            lat: Latitude in degrees × 1e7 (default 0).
            lon: Longitude in degrees × 1e7 (default 0).
            alt: Altitude in mm above MSL (default 0).
        """
        if self._conn is None:
            return
        with self._send_lock:
            self._conn.mav.set_gps_global_origin_send(
                target_system=self._conn.target_system,
                latitude=lat,
                longitude=lon,
                altitude=alt,
                time_usec=int(time.time() * 1e6),
            )
        Logger.info(f"SET_GPS_GLOBAL_ORIGIN → ({lat}, {lon}, {alt})")

    def send_home_position(self, lat: int = 0, lon: int = 0,
                            alt: int = 0) -> None:
        """Send SET_HOME_POSITION to complete EKF3 origin handshake.

        Sent immediately after SET_GPS_GLOBAL_ORIGIN.

        Args:
            lat: Latitude in degrees × 1e7 (default 0).
            lon: Longitude in degrees × 1e7 (default 0).
            alt: Altitude in mm above MSL (default 0).
        """
        if self._conn is None:
            return
        # Quaternion [w, x, y, z] = identity (no rotation)
        q = [1.0, 0.0, 0.0, 0.0]
        with self._send_lock:
            self._conn.mav.set_home_position_send(
                target_system=self._conn.target_system,
                latitude=lat,
                longitude=lon,
                altitude=alt,
                x=0.0, y=0.0, z=0.0,           # local NED position
                q=q,
                approach_x=0.0,
                approach_y=0.0,
                approach_z=0.0,
                time_usec=int(time.time() * 1e6),
            )
        Logger.info(f"SET_HOME_POSITION → ({lat}, {lon}, {alt})")

    # ── ODOMETRY (Vision → EKF3 Fusion) ────────────────────────────────

    def send_odometry(self, x: float, y: float, z: float,
                       vx: float, vy: float, vz: float) -> None:
        """Send ODOMETRY message (msg ID 331) to feed vision data to EKF3.

        Frame conventions:
          - Position (x, y, z): MAV_FRAME_LOCAL_NED
          - Velocity (vx, vy, vz): MAV_FRAME_BODY_FRD
          - estimator_type: MAV_ESTIMATOR_TYPE_VISION (6)

        Args:
            x, y, z:    Position in local NED frame (metres).
            vx, vy, vz: Velocity in body FRD frame (m/s).
        """
        if self._conn is None:
            return

        # Identity quaternion — we don't estimate orientation
        q = [1.0, 0.0, 0.0, 0.0]

        # Covariance: 21-element upper-triangular (row-major).
        # Use NaN to indicate "not available" — EKF will use its own.
        pose_cov = [float('nan')] * 21
        vel_cov = [float('nan')] * 21

        with self._send_lock:
            self._conn.mav.odometry_send(
                time_usec=int(time.time() * 1e6),
                frame_id=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                child_frame_id=mavutil.mavlink.MAV_FRAME_BODY_FRD,
                x=x, y=y, z=z,
                q=q,
                vx=vx, vy=vy, vz=vz,
                rollspeed=0.0,
                pitchspeed=0.0,
                yawspeed=0.0,
                pose_covariance=pose_cov,
                velocity_covariance=vel_cov,
                reset_counter=0,
                estimator_type=Config.ODOMETRY_ESTIMATOR_TYPE,
                quality=0,
            )

    # ── Position + Velocity Target (Local NED) ─────────────────────────

    def send_position_target_local_ned_full(
        self,
        x: float = 0.0, y: float = 0.0, z: float = -1.0,
        vx: float = 0.0, vy: float = 0.0, vz: float = 0.0,
    ) -> None:
        """Send SET_POSITION_TARGET_LOCAL_NED with both position AND velocity.

        Used during HOVER for PI drift correction: commands the desired
        hold position plus a PI-computed velocity nudge.

        Coordinate frame: MAV_FRAME_LOCAL_NED (Z negative = up).

        Args:
            x, y, z:    Target position (m) in local NED.
            vx, vy, vz: Target velocity (m/s) in local NED.
        """
        if self._conn is None:
            return

        # type_mask: use position (bits 0,1,2) AND velocity (bits 3,4,5)
        # Ignore acceleration (bits 6,7,8), yaw (bit 10), yaw_rate (bit 11)
        type_mask = 0b0000110111000000

        with self._send_lock:
            self._conn.mav.set_position_target_local_ned_send(
                time_boot_ms=int(time.time() * 1000) & 0xFFFFFFFF,
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                coordinate_frame=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask=type_mask,
                x=x, y=y, z=z,
                vx=vx, vy=vy, vz=vz,
                afx=0.0, afy=0.0, afz=0.0,
                yaw=0.0,
                yaw_rate=0.0,
            )

    # ── NAV_LAND Command ───────────────────────────────────────────────

    def send_land_command(self, x: float = 0.0, y: float = 0.0) -> bool:
        """Send MAV_CMD_NAV_LAND at the specified local position.

        Args:
            x: North position in local NED (m).
            y: East position in local NED (m).

        Returns:
            ``True`` if the command was sent.
        """
        if self._conn is None:
            return False
        with self._send_lock:
            self._conn.mav.command_long_send(
                target_system=self._conn.target_system,
                target_component=self._conn.target_component,
                command=mavutil.mavlink.MAV_CMD_NAV_LAND,
                confirmation=0,
                param1=0,       # abort alt (0 = use default)
                param2=0,       # land mode (0 = default)
                param3=0,       # empty
                param4=float('nan'),  # yaw (NaN = use current)
                param5=x,       # latitude / local X
                param6=y,       # longitude / local Y
                param7=0,       # altitude (0 = ground)
            )
        Logger.info(f"MAV_CMD_NAV_LAND sent at ({x:.2f}, {y:.2f})")
        return True

