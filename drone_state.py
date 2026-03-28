"""
ASCEND Phase 2 — Drone State & Flight State Enum
Thread-safe shared state object used by all threads and the state machine.
"""

import enum
import threading
import time
from typing import Any, Dict, Optional


class State(enum.Enum):
    """Flight state machine states."""
    IDLE       = "IDLE"
    PREFLIGHT  = "PREFLIGHT"
    TAKEOFF    = "TAKEOFF"
    HOVER      = "HOVER"
    RETURN     = "RETURN"
    LAND       = "LAND"
    EMERGENCY  = "EMERGENCY"


class DroneState:
    """Thread-safe shared state object for the ASCEND drone system.

    All threads read and write drone state exclusively through this
    object's getters, setters, and :meth:`update` method, which
    acquire an internal lock to prevent data races.

    Typical usage::

        ds = DroneState()
        ds.set_altitude_m(4.2)
        alt = ds.altitude_m
        ds.update(battery_pct=78, battery_volt=15.1)
        snap = ds.snapshot()  # dict copy for telemetry
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()

        # Flight state
        self._current_state: State = State.IDLE
        self._manual_mode: bool = False

        # Pixhawk telemetry
        self._armed: bool = False
        self._altitude_m: float = 0.0
        self._battery_pct: int = 0
        self._battery_volt: float = 0.0
        self._fc_mode: str = "UNKNOWN"

        # Sensors
        self._tf02_dist_m: Optional[float] = None
        self._vib_x: float = 0.0
        self._vib_y: float = 0.0
        self._vib_z: float = 0.0

        # Navigation
        self._dist_to_home_m: Optional[float] = None
        self._pos_x: float = 0.0
        self._pos_y: float = 0.0

        # Mission timers
        self._hover_remaining_s: Optional[float] = None

        # Safety
        self._emergency_reason: Optional[str] = None

        # Connectivity
        self._last_heartbeat_rx: float = time.time()

    # ── Generic Batch Update ───────────────────────────────────────────

    def update(self, **kwargs: Any) -> None:
        """Update multiple fields atomically under a single lock.

        Args:
            **kwargs: Field names (without leading underscore) mapped to
                      new values.  Unknown keys are silently ignored.
        """
        with self._lock:
            for key, val in kwargs.items():
                attr = f"_{key}"
                if hasattr(self, attr):
                    setattr(self, attr, val)

    # ── current_state ──────────────────────────────────────────────────

    @property
    def current_state(self) -> State:
        with self._lock:
            return self._current_state

    def set_current_state(self, state: State) -> None:
        with self._lock:
            self._current_state = state

    # ── manual_mode ────────────────────────────────────────────────────

    @property
    def manual_mode(self) -> bool:
        with self._lock:
            return self._manual_mode

    def set_manual_mode(self, val: bool) -> None:
        with self._lock:
            self._manual_mode = val

    # ── armed ──────────────────────────────────────────────────────────

    @property
    def armed(self) -> bool:
        with self._lock:
            return self._armed

    def set_armed(self, val: bool) -> None:
        with self._lock:
            self._armed = val

    # ── altitude_m ─────────────────────────────────────────────────────

    @property
    def altitude_m(self) -> float:
        with self._lock:
            return self._altitude_m

    def set_altitude_m(self, val: float) -> None:
        with self._lock:
            self._altitude_m = val

    # ── battery_pct ────────────────────────────────────────────────────

    @property
    def battery_pct(self) -> int:
        with self._lock:
            return self._battery_pct

    def set_battery_pct(self, val: int) -> None:
        with self._lock:
            self._battery_pct = max(0, min(100, val))

    # ── battery_volt ───────────────────────────────────────────────────

    @property
    def battery_volt(self) -> float:
        with self._lock:
            return self._battery_volt

    def set_battery_volt(self, val: float) -> None:
        with self._lock:
            self._battery_volt = val

    # ── fc_mode ────────────────────────────────────────────────────────

    @property
    def fc_mode(self) -> str:
        with self._lock:
            return self._fc_mode

    def set_fc_mode(self, val: str) -> None:
        with self._lock:
            self._fc_mode = val

    # ── tf02_dist_m ────────────────────────────────────────────────────

    @property
    def tf02_dist_m(self) -> Optional[float]:
        with self._lock:
            return self._tf02_dist_m

    def set_tf02_dist_m(self, val: Optional[float]) -> None:
        with self._lock:
            self._tf02_dist_m = val

    # ── vibration ──────────────────────────────────────────────────────

    @property
    def vib_x(self) -> float:
        with self._lock:
            return self._vib_x

    @property
    def vib_y(self) -> float:
        with self._lock:
            return self._vib_y

    @property
    def vib_z(self) -> float:
        with self._lock:
            return self._vib_z

    def set_vibration(self, x: float, y: float, z: float) -> None:
        with self._lock:
            self._vib_x = x
            self._vib_y = y
            self._vib_z = z

    # ── dist_to_home_m ─────────────────────────────────────────────────

    @property
    def dist_to_home_m(self) -> Optional[float]:
        with self._lock:
            return self._dist_to_home_m

    def set_dist_to_home_m(self, val: Optional[float]) -> None:
        with self._lock:
            self._dist_to_home_m = val

    # ── position (for geofence) ────────────────────────────────────────

    @property
    def pos_x(self) -> float:
        with self._lock:
            return self._pos_x

    @property
    def pos_y(self) -> float:
        with self._lock:
            return self._pos_y

    def set_position(self, x: float, y: float) -> None:
        with self._lock:
            self._pos_x = x
            self._pos_y = y

    # ── hover_remaining_s ──────────────────────────────────────────────

    @property
    def hover_remaining_s(self) -> Optional[float]:
        with self._lock:
            return self._hover_remaining_s

    def set_hover_remaining_s(self, val: Optional[float]) -> None:
        with self._lock:
            self._hover_remaining_s = val

    # ── emergency_reason ───────────────────────────────────────────────

    @property
    def emergency_reason(self) -> Optional[str]:
        with self._lock:
            return self._emergency_reason

    def set_emergency_reason(self, val: Optional[str]) -> None:
        with self._lock:
            self._emergency_reason = val

    # ── last_heartbeat_rx ──────────────────────────────────────────────

    @property
    def last_heartbeat_rx(self) -> float:
        with self._lock:
            return self._last_heartbeat_rx

    def set_last_heartbeat_rx(self, val: float) -> None:
        with self._lock:
            self._last_heartbeat_rx = val

    # ── Snapshot for Telemetry ─────────────────────────────────────────

    def snapshot(self) -> Dict[str, Any]:
        """Return a dict copy of all fields (for TelemetryStreamer).

        The snapshot is taken atomically under a single lock acquisition.
        """
        with self._lock:
            return {
                "state":            self._current_state.value,
                "altitude_m":       self._altitude_m,
                "target_alt_m":     4.0,
                "hover_remaining":  self._hover_remaining_s,
                "battery_pct":      self._battery_pct,
                "battery_volt":     self._battery_volt,
                "armed":            self._armed,
                "fc_mode":          self._fc_mode,
                "manual_mode":      self._manual_mode,
                "tf02_dist_m":      self._tf02_dist_m,
                "dist_to_home_m":   self._dist_to_home_m,
                "emergency_reason": self._emergency_reason,
                "vib_x":            self._vib_x,
                "vib_y":            self._vib_y,
                "vib_z":            self._vib_z,
            }
