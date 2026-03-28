"""
ASCEND Phase 2 — Configuration
All system constants in one place. No magic numbers anywhere else.
"""


class Config:
    """Centralised configuration for the ASCEND Phase 2 drone system.

    Every tunable constant — serial ports, baud rates, timing values,
    thresholds, geofence limits, UDP ports — lives here so nothing is
    hard-coded elsewhere in the codebase.
    """

    # ── Serial Ports ───────────────────────────────────────────────────
    PIXHAWK_PORT: str = "/dev/ttyACM0"
    PIXHAWK_BAUD: int = 921600

    TF02_PORT: str = "/dev/ttyAMA0"
    TF02_BAUD: int = 115200

    # ── TF-02 Frame Constants ──────────────────────────────────────────
    TF02_HEADER: int = 0x59
    TF02_FRAME_LEN: int = 9
    TF02_MIN_CM: int = 30
    TF02_MAX_CM: int = 800

    # ── MAVLink System IDs ─────────────────────────────────────────────
    SYSTEM_ID: int = 1           # Pixhawk system ID
    COMPANION_SYSID: int = 255   # RPi5 companion computer
    COMPANION_COMPID: int = 190  # MAV_COMP_ID_ONBOARD_COMPUTER4

    # ── Timing (seconds) ──────────────────────────────────────────────
    HEARTBEAT_INTERVAL: float = 1.0
    TELEMETRY_INTERVAL: float = 1.0
    BRIDGE_HZ: int = 20
    BRIDGE_INTERVAL: float = 1.0 / 20
    TX_POLL_INTERVAL: float = 0.1          # RC override poll (100 ms)
    MAIN_LOOP_HZ: int = 20
    MAIN_LOOP_INTERVAL: float = 1.0 / 20   # 50 ms state-machine tick
    CONNECT_TIMEOUT: float = 30.0
    PARAM_TIMEOUT: float = 5.0
    ARMING_TIMEOUT: float = 5.0

    # ── TX Override Debounce ───────────────────────────────────────────
    TX_ON_DEBOUNCE: int = 5                # 5 consecutive ON  → 500 ms
    TX_OFF_DEBOUNCE: int = 3               # 3 consecutive OFF → 300 ms

    # ── Flight Parameters ──────────────────────────────────────────────
    TARGET_ALT_M: float = 1.0
    ALT_TOLERANCE_M: float = 0.2           # ±0.2 m → "at altitude"
    ALT_STABLE_TIME: float = 2.0           # stable 2 s → HOVER
    HOVER_DURATION: float = 300.0          # 5 minutes
    HOVER_DRIFT_LIMIT_M: float = 0.5       # warn if drift > 0.5 m
    HOVER_DRIFT_TIME: float = 3.0          # for 3 s continuously
    TOUCHDOWN_ALT_CM: int = 10             # TF-02 < 10 cm → ground
    TOUCHDOWN_TIME: float = 1.0            # must be below for 1 s
    HOME_RADIUS_M: float = 1.5             # RTL → LAND radius

    # ── Geofence ───────────────────────────────────────────────────────
    FENCE_X_M: float = 10.0
    FENCE_Y_M: float = 10.0

    # ── Battery Thresholds ─────────────────────────────────────────────
    BATT_MIN_START_V: float = 9.5          # min voltage to start mission
    BATT_LOW_V: float = 9.0                # emergency in hover
    BATT_CRITICAL_V: float = 8.0           # 3.3 V / cell

    # ── Safety Timeouts ───────────────────────────────────────────────
    TF02_DATA_TIMEOUT: float = 2.0         # no reading → emergency
    EKF_FAIL_TIMEOUT: float = 1.0          # EKF unhealthy → emergency
    WIFI_HB_TIMEOUT: float = 4.0           # no laptop packet → emergency

    # ── RC Channel Limits ──────────────────────────────────────────────
    RC_PWM_MIN: int = 1000
    RC_PWM_MAX: int = 2000

    # ── Networking ─────────────────────────────────────────────────────
    LAPTOP_IP: str = "10.100.149.41"
    TELEMETRY_PORT: int = 14550
    CMD_PORT: int = 14551

    # ── ArduCopter Flight-Mode Numbers ─────────────────────────────────
    MODE_MAP: dict = {
        "STABILIZE":  0,
        "ALT_HOLD":   2,
        "LOITER":     5,
        "RTL":        6,
        "LAND":       9,
        "GUIDED":    15,
        "AUTOTUNE":  22,
    }
