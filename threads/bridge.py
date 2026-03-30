"""
ASCEND Phase 1 — Daemon Thread Workers
Provides five background daemons that run alongside the main state machine:
  • RangefinderBridge   — TF-02 → Pixhawk at 20 Hz
  • HeartbeatSender     — companion heartbeat at 1 Hz
  • TelemetryStreamer   — raw MAVLink pass-through to laptop UDP
  • SafetyMonitor       — continuous fault detection
  • SensorFusionBridge  — ESP32-CAM + TF-02 → ODOMETRY at 20 Hz (EKF3 fusion)
"""

import socket
import threading
import time
from typing import Any, Optional

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
    """Transparent MAVLink bridge: Pixhawk USB RX → laptop UDP:14550.

    This enables Mission Planner telemetry over Wi-Fi while Python code
    continues controlling the same flight controller over USB.
    """

    def __init__(self, pixhawk: PixhawkLink) -> None:
        super().__init__(daemon=True, name="TelemetryStreamer")
        self._pixhawk = pixhawk
        self._running = threading.Event()
        self._running.set()
        self._sock: Optional[socket.socket] = None
        self._dest = (Config.LAPTOP_IP, Config.TELEMETRY_PORT)

    def _forward_packet(self, packet: bytes) -> None:
        """Forward one raw MAVLink packet to laptop UDP endpoint."""
        if not packet or self._sock is None:
            return
        try:
            self._sock.sendto(packet, self._dest)
        except Exception:
            # Packet loss on Wi-Fi is acceptable for telemetry.
            pass

    def stop(self) -> None:
        """Signal the streamer thread to stop."""
        self._running.clear()

    def run(self) -> None:
        """Open UDP socket and relay all inbound MAVLink packets."""
        Logger.info(f"TelemetryStreamer → {Config.LAPTOP_IP}:{Config.TELEMETRY_PORT}")
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except OSError as exc:
            Logger.error(f"TelemetryStreamer cannot create socket: {exc}")
            return

        self._pixhawk.add_rx_listener(self._forward_packet)
        while self._running.is_set():
            time.sleep(0.1)

        self._pixhawk.remove_rx_listener(self._forward_packet)
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
        self._hard_kill_sent: bool = False

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
            self._hard_kill_sent = False

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
        interval = 1.0 / max(1, Config.SAFETY_MONITOR_HZ)
        while self._running.is_set():
            with self._lock:
                flight_active = self._flight_active

            if flight_active:
                self._check_tf02()
                self._check_wifi()
                self._check_drift_kill()
                self._check_geofence()

            time.sleep(interval)
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

    def _check_drift_kill(self) -> None:
        """Hard-kill if drift exceeds strict indoor limit."""
        with self._lock:
            x, y = self._pos_x, self._pos_y
            already_sent = self._hard_kill_sent

        if already_sent:
            return

        if abs(x) > Config.DRIFT_KILL_M or abs(y) > Config.DRIFT_KILL_M:
            self._trigger(
                f"Drift kill breach — pos=({x:.2f}, {y:.2f}) "
                f"limit=±{Config.DRIFT_KILL_M:.2f}m"
            )
            Logger.error("⚠ HARD-KILL: Forcing LAND + DISARM")
            self._pixhawk.set_mode("LAND")
            time.sleep(0.15)
            self._pixhawk.disarm()
            with self._lock:
                self._hard_kill_sent = True


# ═══════════════════════════════════════════════════════════════════════════
# Sensor Fusion Bridge — ESP32-CAM + TF-02 → ODOMETRY → EKF3
# ═══════════════════════════════════════════════════════════════════════════

class SensorFusionBridge(threading.Thread):
    """20 Hz sensor fusion daemon: optical flow + lidar → ODOMETRY messages.

    At startup, sends SET_GPS_GLOBAL_ORIGIN and SET_HOME_POSITION to
    bootstrap EKF3 alignment (mandatory for GPS-denied indoor flight).

    Each iteration at 20 Hz:
      1. Read pixel displacement from WiFiLKProcessor
      2. Read altitude from TF02Reader
      3. Compute real velocity:  V = (pixel_flow × altitude) / focal_length
      4. Apply low-pass filter:  V_filt = α × V_raw + (1-α) × V_prev
      5. Integrate velocity → position estimate
      6. Package into ODOMETRY message (MAV_ESTIMATOR_TYPE_VISION)
      7. Send to Pixhawk EKF3

    Frame conventions:
      - Position: MAV_FRAME_LOCAL_NED
      - Velocity: MAV_FRAME_BODY_FRD

    Args:
        lk_flow: Running WiFiLKProcessor instance.
        tf02:    Running TF02Reader instance.
        pixhawk: Connected PixhawkLink instance.
    """

    def __init__(self, lk_flow: object, tf02: TF02Reader,
                 pixhawk: PixhawkLink) -> None:
        super().__init__(daemon=True, name="SensorFusionBridge")
        self._lk = lk_flow
        self._tf02 = tf02
        self._pixhawk = pixhawk

        self._running = threading.Event()
        self._running.set()
        self._lock = threading.Lock()

        # ── Fusion state ──────────────────────────────────────────────
        self._filtered_vx: float = 0.0
        self._filtered_vy: float = 0.0
        self._pos_x: float = 0.0
        self._pos_y: float = 0.0
        self._prev_time: float = 0.0

        # ── Heartbeat timestamps (for safety monitor) ─────────────────
        self._last_esp32_time: float = 0.0
        self._last_lidar_time: float = 0.0
        self._diag_last_log: float = 0.0

    # ── Thread-safe Properties ─────────────────────────────────────────

    @property
    def filtered_vx(self) -> float:
        """Latest filtered X velocity (m/s, body-forward)."""
        with self._lock:
            return self._filtered_vx

    @property
    def filtered_vy(self) -> float:
        """Latest filtered Y velocity (m/s, body-right)."""
        with self._lock:
            return self._filtered_vy

    @property
    def last_esp32_time(self) -> float:
        """``time.time()`` of most recent valid ESP32-CAM data."""
        with self._lock:
            return self._last_esp32_time

    @property
    def last_lidar_time(self) -> float:
        """``time.time()`` of most recent valid TF02 data."""
        with self._lock:
            return self._last_lidar_time

    @property
    def position(self) -> tuple:
        """Return ``(x, y)`` integrated position estimate in metres."""
        with self._lock:
            return (self._pos_x, self._pos_y)

    def stop(self) -> None:
        """Signal the fusion thread to stop."""
        self._running.clear()

    # ── EKF3 Initialization ────────────────────────────────────────────

    def _init_ekf(self) -> None:
        """Send SET_GPS_GLOBAL_ORIGIN + SET_HOME_POSITION at (0,0,0).

        Mandatory for EKF3 alignment in GPS-denied mode.  Sent once at
        thread startup before the fusion loop begins.
        """
        lat = Config.EKF_ORIGIN_LAT
        lon = Config.EKF_ORIGIN_LON
        alt = Config.EKF_ORIGIN_ALT

        Logger.info("EKF_INIT: Sending GPS origin + home position handshake …")
        for _ in range(5):
            self._pixhawk.send_gps_global_origin(lat, lon, alt)
            time.sleep(0.2)
            self._pixhawk.send_home_position(lat, lon, alt)
            time.sleep(0.2)
        Logger.ok("EKF_INIT: GPS origin and home position set at (0, 0, 0) — "
                   "EKF3 alignment should begin")

    # ── Fusion Step ────────────────────────────────────────────────────

    def _fusion_step(self) -> None:
        """One iteration of the sensor fusion loop."""
        now = time.time()

        # ── Read sensors ──────────────────────────────────────────────
        # ESP32-CAM optical flow
        if self._lk is None:
            return

        dx, dy, quality = self._lk.get_flow()
        alt_m = self._tf02.distance_m

        # Gate: need both valid flow and altitude
        if quality < 1 or alt_m is None or alt_m < Config.VIO_MIN_ALT_M:
            # Keep sending last-known data — don't zero out
            if self._prev_time > 0:
                with self._lock:
                    px, py = self._pos_x, self._pos_y
                    fvx, fvy = self._filtered_vx, self._filtered_vy
                self._pixhawk.send_odometry(
                    x=px, y=py, z=-alt_m if alt_m is not None else 0.0,
                    vx=fvx, vy=fvy, vz=0.0,
                )
            return

        # ── Update heartbeat timestamps ───────────────────────────────
        with self._lock:
            if self._lk.data_age < 1.0:
                self._last_esp32_time = now
            if self._tf02.data_age < 1.0:
                self._last_lidar_time = now

        # ── Compute dt ────────────────────────────────────────────────
        if self._prev_time == 0.0:
            self._prev_time = now
            return
        dt = now - self._prev_time
        if dt <= 0.001 or dt > 0.5:
            self._prev_time = now
            return
        self._prev_time = now

        # ── MATH: Real velocity (m/s) ─────────────────────────────────
        # V_true = (Pixel_Flow × Lidar_Altitude) / Focal_Length_Pixels
        raw_vx = (dx * alt_m) / Config.VIO_FOCAL_LENGTH_PX
        raw_vy = (dy * alt_m) / Config.VIO_FOCAL_LENGTH_PX

        # ── Low-Pass Filter (α = 0.2) ─────────────────────────────────
        # Eliminates high-frequency motor noise from flow measurements.
        alpha = Config.LPF_ALPHA
        with self._lock:
            self._filtered_vx = alpha * raw_vx + (1.0 - alpha) * self._filtered_vx
            self._filtered_vy = alpha * raw_vy + (1.0 - alpha) * self._filtered_vy
            fvx = self._filtered_vx
            fvy = self._filtered_vy

        # ── Integrate velocity → position ─────────────────────────────
        with self._lock:
            self._pos_x += fvx * dt
            self._pos_y += fvy * dt
            px, py = self._pos_x, self._pos_y

        # ── Send ODOMETRY to EKF3 ─────────────────────────────────────
        # Position: MAV_FRAME_LOCAL_NED (z negative = up)
        # Velocity: MAV_FRAME_BODY_FRD
        self._pixhawk.send_odometry(
            x=px, y=py, z=-alt_m,
            vx=fvx, vy=fvy, vz=0.0,
        )

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        """Run EKF init, then 20 Hz sensor fusion loop."""
        Logger.info(f"SensorFusionBridge starting at {Config.FUSION_RATE_HZ} Hz")

        # ── Phase 1: EKF3 initialization ──────────────────────────────
        self._init_ekf()

        # ── Phase 2: Fusion loop at 20 Hz ─────────────────────────────
        Logger.info(
            f"Fusion loop active — "
            f"LPF α={Config.LPF_ALPHA}, "
            f"focal={Config.VIO_FOCAL_LENGTH_PX}px, "
            f"estimator=VISION({Config.ODOMETRY_ESTIMATOR_TYPE})"
        )

        next_tick = time.perf_counter()
        while self._running.is_set():
            try:
                self._fusion_step()
            except Exception as exc:
                Logger.warn(f"SensorFusionBridge error: {exc}")

            # Diagnostic log every 2 seconds
            now = time.time()
            if now - self._diag_last_log >= 2.0:
                self._diag_last_log = now
                with self._lock:
                    Logger.info(
                        f"FUSION | vx={self._filtered_vx:+.3f} "
                        f"vy={self._filtered_vy:+.3f} m/s | "
                        f"pos=({self._pos_x:+.2f}, {self._pos_y:+.2f}) m | "
                        f"alt={self._tf02.distance_m or 0:.2f}m"
                    )

            next_tick += Config.FUSION_INTERVAL
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()

        Logger.info("SensorFusionBridge stopped")

