"""
ASCEND Phase 2 — Safety Monitor
Daemon thread: continuously monitors 4 failure conditions and sets
an emergency_flag threading.Event when any triggers.
"""

import threading
import time

from ..config import Config
from ..logger import Logger
from ..drone_state import DroneState, State
from ..hardware.tf02 import TF02Reader
from ..hardware.pixhawk import PixhawkLink


# Flight states where safety checks are active
_IN_FLIGHT = {State.TAKEOFF, State.HOVER, State.RETURN}


class SafetyMonitor(threading.Thread):
    """Monitors failure conditions and sets emergency_flag on trigger.

    Monitored conditions (only active during TAKEOFF / HOVER / RETURN):
        1. TF-02 data age > 2.0 s  → sensor failure
        2. EKF health flags bad for > 1.0 s  → EKF degradation
        3. Laptop Wi-Fi heartbeat absent > 3.0 s  → link loss
        4. LOCAL_POSITION_NED outside geofence  → boundary breach

    Condition 5 (RPi5 crash) is handled at firmware level by Pixhawk
    ``FS_GCS_ENABLE`` parameter — no code needed here.

    Args:
        tf02: Shared TF02Reader instance.
        pixhawk: Shared PixhawkLink instance (for EKF polling).
        drone_state: Shared DroneState.
        emergency_flag: ``threading.Event`` set when any condition fires.
    """

    def __init__(
        self,
        tf02: TF02Reader,
        pixhawk: PixhawkLink,
        drone_state: DroneState,
        emergency_flag: threading.Event,
    ) -> None:
        super().__init__(daemon=True, name="SafetyMonitor")
        self._tf02 = tf02
        self._px = pixhawk
        self._ds = drone_state
        self._eflag = emergency_flag
        self._running = threading.Event()
        self._running.set()

        # EKF degradation tracking
        self._ekf_bad_since: float = 0.0

    def stop(self) -> None:
        self._running.clear()

    def clear(self) -> None:
        """Reset the emergency flag (after landing)."""
        self._eflag.clear()
        self._ds.set_emergency_reason(None)

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        Logger.info("SafetyMonitor running")
        while self._running.is_set():
            state = self._ds.current_state
            if state in _IN_FLIGHT and not self._eflag.is_set():
                self._check_tf02()
                self._check_ekf()
                self._check_wifi()
                self._check_geofence()
            else:
                # Reset EKF timer when not in flight
                self._ekf_bad_since = 0.0
            time.sleep(0.1)  # 10 Hz check rate
        Logger.info("SafetyMonitor stopped")

    # ── Trigger Helper ─────────────────────────────────────────────────

    def _trigger(self, reason: str) -> None:
        if not self._eflag.is_set():
            Logger.error(f"⚠ SAFETY: {reason}")
            self._ds.set_emergency_reason(reason)
            self._eflag.set()

    # ── Check 1: TF-02 Sensor ─────────────────────────────────────────

    def _check_tf02(self) -> None:
        if self._tf02.data_age > Config.TF02_DATA_TIMEOUT:
            self._trigger(
                f"TF-02 sensor failure — no data for "
                f"{self._tf02.data_age:.1f}s"
            )

    # ── Check 2: EKF Health ────────────────────────────────────────────

    def _check_ekf(self) -> None:
        msg = self._px.recv("EKF_STATUS_REPORT", timeout=0.05)
        if msg is None:
            return

        # flags is a bitmask; bits 0,1 = velocity, 3,4 = position
        flags = msg.flags
        pos_ok = bool(flags & 0x01)     # EKF_ATTITUDE
        vel_ok = bool(flags & 0x02)     # EKF_VELOCITY_HORIZ
        pos_horiz = bool(flags & 0x08)  # EKF_POS_HORIZ_ABS

        if not (pos_ok and vel_ok and pos_horiz):
            if self._ekf_bad_since == 0.0:
                self._ekf_bad_since = time.time()
            elif time.time() - self._ekf_bad_since > Config.EKF_FAIL_TIMEOUT:
                bad_flags = []
                if not pos_ok:
                    bad_flags.append("attitude")
                if not vel_ok:
                    bad_flags.append("vel_horiz")
                if not pos_horiz:
                    bad_flags.append("pos_horiz_abs")
                self._trigger(
                    f"EKF health failure: {', '.join(bad_flags)}"
                )
        else:
            self._ekf_bad_since = 0.0

    # ── Check 3: Wi-Fi Link ───────────────────────────────────────────

    def _check_wifi(self) -> None:
        age = time.time() - self._ds.last_heartbeat_rx
        if age > Config.WIFI_HB_TIMEOUT:
            self._trigger(f"Wi-Fi link lost — no packet for {age:.1f}s")

    # ── Check 4: Geofence ─────────────────────────────────────────────

    def _check_geofence(self) -> None:
        x = self._ds.pos_x
        y = self._ds.pos_y
        if abs(x) > Config.FENCE_X_M or abs(y) > Config.FENCE_Y_M:
            self._trigger(
                f"Boundary breach at x={x:.1f} y={y:.1f} "
                f"(limits ±{Config.FENCE_X_M}/{Config.FENCE_Y_M})"
            )
