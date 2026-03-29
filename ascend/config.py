"""
ASCEND Phase 1 — Configuration
All system constants in one place. No magic numbers anywhere else.
"""


class Config:
    """Centralised configuration for the ASCEND drone system.

    Every tunable constant — serial ports, baud rates, timing values,
    thresholds, Pixhawk parameters, geofence limits — lives here so
    nothing is hard-coded elsewhere in the codebase.
    """

    # ── Serial Ports ───────────────────────────────────────────────────
    PIXHAWK_PORT: str = "/dev/ttyACM0"
    PIXHAWK_BAUD: int = 921600

    TF02_PORT: str = "/dev/ttyAMA2"
    TF02_BAUD: int = 115200

    # ── TF-02 Frame Constants ──────────────────────────────────────────
    TF02_HEADER: int = 0x59
    TF02_FRAME_LEN: int = 9
    TF02_MIN_CM: int = 30
    TF02_MAX_CM: int = 800

    # ── ESP32-CAM (VIO Optical Flow) ──────────────────────────────────
    ESP32_CAM_PORT: str = "/dev/serial0"
    ESP32_CAM_BAUD: int = 921600
    ESP32_CAM_HEADER_1: int = 0xAA
    ESP32_CAM_HEADER_2: int = 0x55
    ESP32_CAM_FRAME_LEN: int = 8

    # ── VIO Stabilization ─────────────────────────────────────────────
    VIO_RATE_HZ: int = 20                    # stabilizer loop rate
    VIO_INTERVAL: float = 1.0 / 20           # 50 ms
    VIO_KP: float = 0.5                      # P-gain (PWM per m/s drift)
    VIO_KI: float = 0.15                     # I-gain (eliminates steady drift)
    VIO_KD: float = 0.3                      # D-gain (damps oscillation)
    VIO_EMA_ALPHA: float = 0.4               # EMA smoothing (0–1, lower=smoother)
    VIO_INTEGRAL_MAX: float = 50.0           # anti-windup clamp for I-term (PWM)
    VIO_DEADZONE_PX: int = 2                 # ignore flow below this
    VIO_MIN_QUALITY: int = 5                 # minimum tracked features
    VIO_MAX_CORRECTION_PWM: int = 100        # max ±100 PWM from neutral
    VIO_FOCAL_LENGTH_PX: float = 60.0        # OV2640 @ 96×96 approx
    VIO_DATA_TIMEOUT: float = 0.5            # stale ESP32-CAM threshold
    VIO_MIN_ALT_M: float = 0.3              # disable VIO below 30 cm

    # ── Crash Safety ──────────────────────────────────────────────────
    CRASH_DISARM_ALT_M: float = 0.15         # force-disarm if below this alt
    CRASH_DISARM_TIMEOUT: float = 3.0        # and stuck for this long (seconds)

    # ── MAVLink System IDs ─────────────────────────────────────────────
    SYSTEM_ID: int = 1           # Pixhawk system ID
    COMPANION_SYSID: int = 255   # RPi5 companion computer
    COMPANION_COMPID: int = 190  # MAV_COMP_ID_ONBOARD_COMPUTER4

    # ── Timing (seconds) ──────────────────────────────────────────────
    HEARTBEAT_INTERVAL: float = 1.0
    TELEMETRY_INTERVAL: float = 1.0
    BRIDGE_HZ: int = 20                    # rangefinder bridge rate
    BRIDGE_INTERVAL: float = 1.0 / BRIDGE_HZ
    TX_POLL_INTERVAL: float = 0.1          # RC override check (100 ms)
    MAIN_LOOP_INTERVAL: float = 0.05       # 20 Hz state-machine tick
    CONNECT_TIMEOUT: float = 30.0          # wait for Pixhawk heartbeat
    PARAM_TIMEOUT: float = 5.0             # param read/write ACK timeout
    PREFLIGHT_TIMEOUT: float = 30.0        # arming attempt window

    # ── Flight Parameters ──────────────────────────────────────────────
    TARGET_ALT_M: float = 1.0              # hover altitude (metres)
    ALT_TOLERANCE_M: float = 0.2           # ±0.2 m considered "at altitude"
    ALT_STABLE_TIME: float = 2.0           # stable for 2 s → transition
    HOVER_DURATION: float = 60.0           # 1 minute
    TOUCHDOWN_ALT_M: float = 0.10          # TF-02 reading below this = ground
    TOUCHDOWN_TIME: float = 1.0            # must be below for 1 s
    HOME_RADIUS_M: float = 1.5             # RTL → LAND radius

    # ── Geofence ───────────────────────────────────────────────────────
    FENCE_X_M: float = 15.0
    FENCE_Y_M: float = 15.0

    # ── Safety Thresholds ──────────────────────────────────────────────
    TF02_DATA_TIMEOUT: float = 2.0         # no reading → emergency
    WIFI_HB_TIMEOUT: float = 3.0           # no laptop heartbeat → emergency
    LOW_BATTERY_VOLT: float = 14.0         # 3.5 V / cell
    CRITICAL_BATTERY_VOLT: float = 9.0     # updated threshold

    # ── Vibration Thresholds (m/s²) ────────────────────────────────────
    VIB_EXCELLENT: float = 15.0
    VIB_GOOD: float = 30.0
    VIB_HIGH: float = 60.0

    # ── RC Channel Limits ──────────────────────────────────────────────
    RC_PWM_MIN: int = 1000
    RC_PWM_MAX: int = 2000
    RC_INVALID_VALS: tuple = (0, 65535)
    RC8_OPTION: int = 8  # <--- ADD THIS LINE

    # ── Telemetry Streaming ────────────────────────────────────────────
    LAPTOP_IP: str = "255.255.255.255"         # laptop hotspot gateway
    TELEMETRY_PORT: int = 14550

    # ── Health-Check Durations (seconds) ───────────────────────────────
    VIB_SAMPLE_TIME: float = 5.0
    TF02_SAMPLE_TIME: float = 5.0

    # ── Default PID Values (ArduCopter defaults) ──────────────────────
    DEFAULT_PIDS: dict = {
        "ATC_RAT_PIT_P": 0.135,
        "ATC_RAT_PIT_I": 0.135,
        "ATC_RAT_PIT_D": 0.0036,
        "ATC_RAT_RLL_P": 0.135,
        "ATC_RAT_RLL_I": 0.135,
        "ATC_RAT_RLL_D": 0.0036,
    }

    # ── Pixhawk Parameters — Rangefinder ──────────────────────────────
    PARAMS_RANGEFINDER: dict = {
        "RNGFND1_TYPE":    10,
        "RNGFND1_ORIENT":  25,
        "RNGFND1_MIN_CM":  30,
        "RNGFND1_MAX_CM":  800,
        "RNGFND1_GNDCLR":  15,
        "EK3_RNG_USE_HGT": 70,
    }

    # ── Pixhawk Parameters — AutoTune ─────────────────────────────────
    PARAMS_AUTOTUNE: dict = {
        "ATC_RAT_PIT_P":  0.135,
        "ATC_RAT_PIT_I":  0.135,
        "ATC_RAT_PIT_D":  0.0036,
        "ATC_RAT_RLL_P":  0.135,
        "ATC_RAT_RLL_I":  0.135,
        "ATC_RAT_RLL_D":  0.0036,
        "AUTOTUNE_AGGR":  0.1,
        "AUTOTUNE_AXES":  3,
        "AUTOTUNE_MIN_D": 0.001,
    }

    # ── Pixhawk Parameters — Failsafe ─────────────────────────────────
    PARAMS_FAILSAFE: dict = {
        "FS_GCS_ENABLE":   1,
        "FS_GCS_TIMEOUT":  3,
        "FS_THR_ENABLE":   0,
        "BATT_FS_LOW_ACT": 1,
        "BATT_LOW_VOLT":   10.4,
        "BATT_CRT_VOLT":   10,
    }

    # ── Pixhawk Parameters — Flight ───────────────────────────────────
    PARAMS_FLIGHT: dict = {
        "LOIT_SPEED":     500,
        "LOIT_ACC_MAX":   250,
        "RTL_ALT":        400,
        "RTL_LOIT_TIME":  3000,
        "ARMING_CHECK":   1,
        "DISARM_DELAY":   10,
        "MOT_SPIN_ARM":   0.10,
        "MOT_SPIN_MIN":   0.15,
        "MOT_THST_HOVER": 0.35,
    }

    # ── ArduCopter Flight-Mode Numbers ─────────────────────────────────
    MODE_MAP = {
        "STABILIZE":    0,
        "ACRO":         1,
        "ALT_HOLD":     2,
        "AUTO":         3,
        "GUIDED":       4,
        "LOITER":       5,
        "RTL":          6,
        "CIRCLE":       7,
        "LAND":         9,
        "DRIFT":        11,
        "SPORT":        13,
        "GUIDED_NOGPS": 20,  # ← add this
        "AUTOTUNE":     17,
    }
