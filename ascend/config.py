"""
ASCEND — Configuration
All system constants in one place. No magic numbers anywhere else.
Hardware: Pixhawk 2.4.8 (ArduCopter), RPi4, TF-02 LiDAR, ESP32-CAM (OV2640).
"""


class Config:
    """Centralised configuration for the ASCEND drone system.

    Every tunable constant — serial ports, baud rates, timing values,
    thresholds, PID gains, Pixhawk parameters — lives here so nothing
    is hard-coded elsewhere in the codebase.
    """

    # ── Serial Ports ───────────────────────────────────────────────────
    PIXHAWK_PORT: str = "/dev/ttyACM0"
    PIXHAWK_BAUD: int = 115200

    TF02_PORT: str = "/dev/ttyAMA0"
    TF02_BAUD: int = 115200

    # ── TF-02 Frame Constants ──────────────────────────────────────────
    TF02_HEADER: int = 0x59
    TF02_FRAME_LEN: int = 9
    TF02_MIN_CM: int = 30
    TF02_MAX_CM: int = 800

    # ── ESP32-CAM Flow Packets (UART0 → ttyAMA2 @ 115200) ─────────────
    ESP32_CAM_PORT: str = "/dev/ttyAMA2"
    ESP32_CAM_BAUD: int = 115200
    ESP32_CAM_HEADER_1: int = 0xAB
    ESP32_CAM_HEADER_2: int = 0xCD
    ESP32_CAM_FRAME_LEN: int = 8

    # ── ESP32-CAM Raw Frames (UART2 → ttyAMA3 @ 921600) ───────────────
    ESP32_FRAME_PORT: str = "/dev/ttyAMA3"
    ESP32_FRAME_BAUD: int = 921600
    ESP32_FRAME_WIDTH: int = 160
    ESP32_FRAME_HEIGHT: int = 120
    ESP32_FRAME_SYNC_1: int = 0xAA
    ESP32_FRAME_SYNC_2: int = 0x55
    ESP32_FRAME_PIXEL_COUNT: int = 160 * 120  # 19200

    # ── VIO Stabilization (common) ────────────────────────────────────
    VIO_RATE_HZ: int = 20
    VIO_INTERVAL: float = 1.0 / 20
    VIO_DEADZONE_PX: int = 2
    VIO_MIN_QUALITY: int = 5
    VIO_MAX_CORRECTION_PWM: int = 100
    VIO_FOCAL_LENGTH_PX: float = 100.0   # OV2640 @ QQVGA 160×120
    VIO_DATA_TIMEOUT: float = 0.5
    VIO_MIN_ALT_M: float = 0.3
    VIO_INTEGRAL_MAX: float = 50.0       # anti-windup clamp (PWM)

    # ── ESP32 Pipeline PID ────────────────────────────────────────────
    ESP_VIO_KP: float = 0.4
    ESP_VIO_KI: float = 0.05
    ESP_VIO_KD: float = 0.02
    ESP_VIO_WEIGHT: float = 0.4

    # ── ORB Pipeline PID ──────────────────────────────────────────────
    ORB_VIO_KP: float = 0.5
    ORB_VIO_KI: float = 0.08
    ORB_VIO_KD: float = 0.03
    ORB_VIO_WEIGHT: float = 0.6

    # ── Legacy single-source VIO (kept for cv_flow.py compat) ─────────
    VIO_KP: float = 0.5
    VIO_KI: float = 0.15
    VIO_KD: float = 0.3
    VIO_EMA_ALPHA: float = 0.4

    # ── VIO Position Hold (kept for backward compat) ──────────────────
    VIO_POS_KP: float = 0.8
    VIO_POS_MAX_M: float = 3.0
    VIO_POS_DECAY: float = 0.998

    # ── OpenCV Optical Flow (RPi camera, kept for cv_flow.py) ─────────
    CV_CAMERA_ID: int = 0
    CV_FRAME_WIDTH: int = 160
    CV_FRAME_HEIGHT: int = 120
    CV_FPS: int = 30
    CV_MAX_CORNERS: int = 80
    CV_QUALITY_LEVEL: float = 0.05
    CV_MIN_DISTANCE: int = 7
    CV_BLOCK_SIZE: int = 7
    CV_LK_WIN_SIZE: int = 15
    CV_LK_MAX_LEVEL: int = 3
    CV_FB_THRESHOLD: float = 1.0
    CV_FLOW_WEIGHT: float = 0.7

    # ── ORB Feature Detector ──────────────────────────────────────────
    ORB_N_FEATURES: int = 200
    ORB_RANSAC_THRESH: float = 5.0
    ORB_MIN_MATCHES: int = 8

    # ── Crash Safety ──────────────────────────────────────────────────
    CRASH_DISARM_ALT_M: float = 0.15
    CRASH_DISARM_TIMEOUT: float = 3.0

    # ── MAVLink System IDs ─────────────────────────────────────────────
    SYSTEM_ID: int = 1
    COMPANION_SYSID: int = 255
    COMPANION_COMPID: int = 190

    # ── Timing (seconds) ──────────────────────────────────────────────
    HEARTBEAT_INTERVAL: float = 1.0
    HEARTBEAT_HZ: int = 1
    TELEMETRY_INTERVAL: float = 1.0
    BRIDGE_HZ: int = 20
    BRIDGE_INTERVAL: float = 1.0 / 20
    TX_POLL_INTERVAL: float = 0.1
    MAIN_LOOP_INTERVAL: float = 0.05
    CONNECT_TIMEOUT: float = 30.0
    PARAM_TIMEOUT: float = 5.0
    PREFLIGHT_TIMEOUT: float = 30.0

    # ── Flight Parameters ──────────────────────────────────────────────
    TARGET_ALT_M: float = 1.0
    HOVER_TARGET_ALT_M: float = 1.0
    ALT_TOLERANCE_M: float = 0.2
    ALT_STABLE_TIME: float = 2.0
    HOVER_DURATION: float = 300.0       # 5 minutes (Task 2)
    HOVER_DURATION_S: float = 300.0
    HOVER_KP_ALT: float = 200.0        # PWM per metre error
    BASE_THROTTLE_PWM: int = 1550
    MIN_THROTTLE_PWM: int = 1100
    MAX_THROTTLE_PWM: int = 1800
    TAKEOFF_START_PWM: int = 1400       # Start ramp near spool-up, not 1100
    TAKEOFF_RAMP_PWM_PER_S: int = 50    # PWM increase per second during ramp
    TAKEOFF_TIMEOUT_S: float = 30.0     # 30s — enough time to climb
    TAKEOFF_ALT_THRESHOLD: float = 0.70 # 70% of target alt to enter HOVER
    LAND_DURATION_S: float = 8.0        # 8s linear descent
    LAND_THROTTLE_DROP_PER_S: int = 50  # PWM decrease per second during land
    ARM_TIMEOUT_S: float = 10.0
    DISARM_TIMEOUT_S: float = 5.0
    TOUCHDOWN_ALT_M: float = 0.10
    TOUCHDOWN_TIME: float = 1.0
    HOME_RADIUS_M: float = 1.5

    # ── Geofence ───────────────────────────────────────────────────────
    FENCE_X_M: float = 15.0
    FENCE_Y_M: float = 15.0

    # ── Safety Thresholds ──────────────────────────────────────────────
    TF02_DATA_TIMEOUT: float = 2.0
    WIFI_HB_TIMEOUT: float = 3.0
    LOW_BATTERY_VOLT: float = 14.0
    CRITICAL_BATTERY_VOLT: float = 13.2
    BATT_LOW_VOLT: float = 14.0
    BATT_CRITICAL_VOLT: float = 13.2

    # ── Vibration Thresholds (m/s²) ────────────────────────────────────
    VIB_EXCELLENT: float = 15.0
    VIB_GOOD: float = 30.0
    VIB_HIGH: float = 60.0

    # ── RC Channel Limits ──────────────────────────────────────────────
    RC_PWM_MIN: int = 1000
    RC_PWM_MAX: int = 2000
    RC_INVALID_VALS: tuple = (0, 65535)
    RC8_OPTION: int = 8

    # ── Telemetry Streaming ────────────────────────────────────────────
    TELEMETRY_HOST: str = "0.0.0.0"
    LAPTOP_IP: str = "255.255.255.255"
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
        "BATT_LOW_VOLT":   14.0,
        "BATT_CRT_VOLT":   13.2,
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
        "GUIDED_NOGPS": 20,
        "AUTOTUNE":     17,
    }
