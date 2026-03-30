"""
ASCEND — Configuration
All system constants in one place. No magic numbers anywhere else.
Hardware: Pixhawk 2.4.8 (ArduCopter), RPi4, TF-02 LiDAR, ESP32-CAM (OV2640).

WiFi Architecture (replaces UART):
  ESP32-CAM → WiFi MJPEG stream → RPi4 OpenCV optical flow
  ESP32 operates in STA mode, joining the RPi4's WiFi hotspot.

  To create the RPi4 hotspot:
    sudo nmcli device wifi hotspot ifname wlan0 ssid ASCEND-AP password ascend123
  The ESP32-CAM connects and gets static IP 10.42.0.200 (set in .ino).
  Stream URL: http://10.42.0.200:81/stream

  If the ESP32 is in AP mode instead (fallback):
    Connect RPi4 to "ASCEND-CAM" network.
    Stream URL: http://192.168.4.1:81/stream
    Update ESP32_WIFI_STREAM_URL below accordingly.
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

    TF02_PORT: str = "/dev/serial0"
    TF02_BAUD: int = 115200

    # ── TF-02 Frame Constants ──────────────────────────────────────────
    TF02_HEADER: int = 0x59
    TF02_FRAME_LEN: int = 9
    TF02_MIN_CM: int = 30
    TF02_MAX_CM: int = 800

    # ── ESP32-CAM WiFi Stream ──────────────────────────────────────────
    # The ESP32-CAM streams MJPEG over WiFi instead of UART.
    # The system will AUTO-DISCOVER the ESP32-CAM by trying candidate IPs
    # in order, then falling back to a subnet scan if none respond.
    #
    # Priority order:
    #   1. RPi hotspot (most reliable — no phone AP isolation issues)
    #   2. Phone hotspot (works only if phone allows device-to-device)
    #   3. ESP32 AP mode (fallback — connect RPi to ASCEND-CAM network)

    # Ordered list of (stream_url, still_url, status_url) to try.
    # WiFiFrameReader will test each in order and use the first reachable one.
    ESP32_WIFI_CANDIDATES: list = [
        {   # RPi hotspot — ESP32 static IP set in .ino
            "stream": "http://10.42.0.200:81/stream",
            "still":  "http://10.42.0.200:80/capture",
            "status": "http://10.42.0.200:80/status",
            "label":  "RPi-hotspot (10.42.0.200)",
        },
        {   # Phone hotspot — current ESP32-CAM IP
            "stream": "http://10.27.96.106:81/stream",
            "still":  "http://10.27.96.106:80/capture",
            "status": "http://10.27.96.106:80/status",
            "label":  "Phone-hotspot (10.27.96.106)",
        },
        {   # ESP32 AP mode — default IP when ESP32 acts as access point
            "stream": "http://192.168.4.1:81/stream",
            "still":  "http://192.168.4.1:80/capture",
            "status": "http://192.168.4.1:80/status",
            "label":  "ESP32-AP (192.168.4.1)",
        },
    ]

    # Fallback: if none of the candidates respond, scan these subnets
    # looking for port 81 (the ESP32-CAM MJPEG port).
    ESP32_WIFI_SCAN_SUBNETS: list = [
        "10.42.0",      # RPi hotspot subnet
        "10.27.96",     # Phone hotspot subnet (your phone)
        "192.168.43",   # Android default hotspot subnet
        "192.168.4",    # ESP32 AP subnet
        "192.168.1",    # Common home router subnet
    ]
    ESP32_WIFI_SCAN_PORT: int = 81
    ESP32_WIFI_SCAN_TIMEOUT: float = 0.3   # per-host TCP timeout (seconds)

    # Default URLs (set at runtime after discovery — these are the initial
    # fallback if discovery is skipped or hasn't run yet).
    ESP32_WIFI_STREAM_URL: str = "http://10.27.96.106:81/stream"
    ESP32_WIFI_STILL_URL:  str = "http://10.27.96.106:80/capture"
    ESP32_WIFI_STATUS_URL: str = "http://10.27.96.106:80/status"

    # How long to wait for the first WiFi frame before warning
    ESP32_WIFI_CONNECT_TIMEOUT: float = 15.0

    # ── ESP32-CAM UART (LEGACY — no longer used, kept for reference) ───
    # These ports are now free for other peripherals.
    # ESP32_CAM_PORT: str = "/dev/ttyAMA2"   # was UART0 flow packets
    # ESP32_CAM_BAUD: int = 115200
    # ESP32_FRAME_PORT: str = "/dev/ttyAMA3" # was UART2 raw frames
    # ESP32_FRAME_BAUD: int = 921600

    # ── VIO Stabilization (common) ────────────────────────────────────
    VIO_RATE_HZ: int = 20
    VIO_INTERVAL: float = 1.0 / 20
    VIO_DEADZONE_PX: int = 1
    VIO_MIN_QUALITY: int = 2          # lowered from 5 — camera is low quality
    VIO_MAX_CORRECTION_PWM: int = 150
    VIO_FOCAL_LENGTH_PX: float = 200.0   # OV2640 @ QVGA 320×240 (scaled from QQVGA)
    VIO_DATA_TIMEOUT: float = 8.0    # allow 8s gap during ESP32 restart after motor arm
    VIO_MIN_ALT_M: float = 0.02
    VIO_INTEGRAL_MAX: float = 50.0       # anti-windup clamp (PWM)

    # ── LK Pipeline PID (was ESP32 UART pipeline) ────────────────────
    # Now drives WiFiLKProcessor output (actual pixels, not ×100).
    ESP_VIO_KP: float = 200.0
    ESP_VIO_KI: float = 25.0
    ESP_VIO_KD: float = 10.0
    ESP_VIO_WEIGHT: float = 0.4

    # ── ORB Pipeline PID (WiFiORBProcessor) ──────────────────────────
    ORB_VIO_KP: float = 250.0
    ORB_VIO_KI: float = 35.0
    ORB_VIO_KD: float = 15.0
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

    # ── OpenCV Optical Flow (shared by LK and CVFlowProcessor) ────────
    # WiFiLKProcessor and CVFlowProcessor both use these parameters.
    CV_CAMERA_ID: int = 0
    CV_FRAME_WIDTH: int = 320    # QVGA — must match CAM_FRAMESIZE in .ino
    CV_FRAME_HEIGHT: int = 240   # QVGA — must match CAM_FRAMESIZE in .ino
    CV_FPS: int = 15             # QVGA runs ~15fps over WiFi
    CV_MAX_CORNERS: int = 150    # more corners for bigger frame
    CV_QUALITY_LEVEL: float = 0.03    # lowered — detect more features on poor camera
    CV_MIN_DISTANCE: int = 10    # scaled up for larger frame
    CV_BLOCK_SIZE: int = 7
    CV_LK_WIN_SIZE: int = 21     # larger window for QVGA
    CV_LK_MAX_LEVEL: int = 3
    CV_FB_THRESHOLD: float = 1.0
    CV_FLOW_WEIGHT: float = 0.7

    # ── ORB Feature Detector ──────────────────────────────────────────
    ORB_N_FEATURES: int = 200
    ORB_RANSAC_THRESH: float = 5.0
    ORB_MIN_MATCHES: int = 4          # lowered — poor camera produces fewer matches

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
    TARGET_ALT_M: float = 0.5           # lowered from 1.0 — indoor net above
    HOVER_TARGET_ALT_M: float = 0.5     # lowered from 1.0 — indoor net above
    ALT_TOLERANCE_M: float = 0.15
    ALT_STABLE_TIME: float = 2.0
    HOVER_DURATION: float = 15.0        # shorter hover for indoor testing
    HOVER_DURATION_S: float = 15.0      # shorter hover for indoor testing
    HOVER_KP_ALT: float = 200.0        # PWM per metre error
    BASE_THROTTLE_PWM: int = 1550
    MIN_THROTTLE_PWM: int = 1100
    MAX_THROTTLE_PWM: int = 1750       # lowered from 1900 — indoor safety
    TAKEOFF_START_PWM: int = 1100
    TAKEOFF_HOVER_PWM: int = 1500
    TAKEOFF_MAX_PWM: int = 1650        # lowered from 1850 — prevent hitting net
    TAKEOFF_RAMP_PWM_STEP: int = 2     # SUPER SLOW: was 5, now 2 PWM per step
    TAKEOFF_RAMP_INTERVAL: float = 0.15 # slower: was 0.1, now 0.15s per step
    TAKEOFF_RAMP_PWM_PER_S: int = 13   # effective: 2/0.15 ≈ 13 PWM/s (was 50)
    TAKEOFF_TIMEOUT_S: float = 60.0    # longer timeout since ramp is slower
    TAKEOFF_ALT_THRESHOLD: float = 0.70
    LAND_DURATION_S: float = 8.0
    LAND_THROTTLE_DROP_PER_S: int = 30  # slower descent: was 50, now 30
    ARM_TIMEOUT_S: float = 10.0
    DISARM_TIMEOUT_S: float = 5.0
    TOUCHDOWN_ALT_M: float = 0.10
    TOUCHDOWN_TIME: float = 1.0
    HOME_RADIUS_M: float = 1.0

    # ── Roll / Pitch Trim (anti-drift) ────────────────────────────────
    # Adjust these to counter mechanical drift.
    # +roll  → tilt RIGHT (correct leftward drift)
    # -roll  → tilt LEFT  (correct rightward drift)
    # +pitch → tilt FORWARD / nose DOWN (correct backward drift)
    # -pitch → tilt BACKWARD / nose UP (correct forward drift)
    # Start with ±10, test, adjust. Typical range: -30 to +30.
    TRIM_ROLL_PWM: int = 0
    TRIM_PITCH_PWM: int = 0

    # ── Geofence ───────────────────────────────────────────────────────
    FENCE_X_M: float = 1.5             # tightened from 15 — indoor white board
    FENCE_Y_M: float = 1.5             # tightened from 15 — indoor white board

    # ── VIO Drift Kill (emergency motor kill if drone drifts off board) ─
    # If accumulated X-Y displacement exceeds this, IMMEDIATE DISARM.
    # The white board is small — drone must not drift beyond it.
    DRIFT_KILL_M: float = 1.0          # max allowed drift from takeoff position
    DRIFT_KILL_ENABLED: bool = True

    # ── White Board Detection (camera-based landing pad check) ────────
    # If <WHITE_MIN_RATIO of pixels are "white", drone is off the pad → LAND.
    # White = HSV saturation < WHITE_S_MAX AND value > WHITE_V_MIN
    WHITE_MIN_RATIO: float = 0.3       # 30% of frame must be white to be "on pad"
    WHITE_S_MAX: int = 60              # max saturation for a "white" pixel (0-255)
    WHITE_V_MIN: int = 160             # min brightness for a "white" pixel (0-255)
    WHITE_CHECK_ENABLED: bool = True
    WHITE_GRACE_S: float = 8.0         # ignore white check for first 8s after takeoff

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
