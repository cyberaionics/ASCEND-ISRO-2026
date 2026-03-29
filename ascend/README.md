# ASCEND — Autonomous Drone System for IRoC-U 2026

**Autonomous Surveyor Challenge for Exploration, Navigation and Dynamics**
Team Bayes Frontier · IIT Dharwad · IRoC-U 2026 Qualification Round

Qualification tasks T1–T3:
stable takeoff, 5-minute autonomous hover at 1 m, controlled landing — all GPS-denied.

X-Y position hold via dual-pipeline VIO (ESP32-CAM block-matching + RPi ORB homography).
Z-axis altitude hold via TF-02 LiDAR + P-controller.
All flight in STABILIZE mode using RC override.

---

## 1. Hardware Components

| Component            | Model / Spec                                       |
|----------------------|----------------------------------------------------|
| Frame                | Holybro S500 V2, AUW ~1.8 kg                      |
| Flight Controller    | Pixhawk 2.4.8 — ArduCopter (latest stable)        |
| Companion Computer   | Raspberry Pi 4, Python 3.11                        |
| Rangefinder          | Benewake TF-02 LiDAR (0–8 m, 100 Hz)              |
| Optical Flow Camera  | AI Thinker ESP32-CAM (OV2640, 160×120 QQVGA)      |
| RC Transmitter       | Radiomaster (ELRS protocol)                        |
| RC Receiver          | Radiomaster R88 (ELRS, SBUS → Pixhawk RC IN)      |
| Battery              | 4S LiPo 14.8 V, low 14.0 V, critical 13.2 V      |

### ESP32-CAM Configuration
- **Mounted**: downward-facing, perpendicular to ground
- **UART0** (GPIO1/3) → RPi `/dev/ttyAMA2` @ 115200 — 8-byte flow packets
- **UART2** (GPIO16/17) → RPi `/dev/ttyAMA3` @ 921600 — raw grayscale frames
- **Resolution**: 160×120 QQVGA grayscale
- **Algorithm**: Block-matching SAD, 6×5 grid of 16×16 blocks, ±8 px search

---

## 2. Hardware Wiring

```
 ┌──────────────────────────────────────────────────────────────────┐
 │                       WIRING DIAGRAM                             │
 ├──────────────────────────────────────────────────────────────────┤
 │                                                                  │
 │   TF-02 LiDAR                  Raspberry Pi 4                    │
 │   ┌─────────┐                ┌──────────────────┐                │
 │   │ VCC (5V)├────────────────┤ 5V  (Pin 2)      │                │
 │   │ GND     ├────────────────┤ GND (Pin 6)      │                │
 │   │ TXD     ├────────────────┤ RXD0 (GPIO15)    │                │
 │   │ RXD     ├────────────────┤ TXD0 (GPIO14)    │                │
 │   └─────────┘                │  /dev/ttyAMA0    │                │
 │                              │  @ 115200 baud   │                │
 │   ESP32-CAM                  │                  │                │
 │   ┌─────────┐                │                  │                │
 │   │ VCC (5V)├────────────────┤ 5V  (Pin 4)      │                │
 │   │ GND     ├────────────────┤ GND (Pin 9)      │                │
 │   │ UART0 TX├────────────────┤ RXD2 (GPIO0)     │                │
 │   │ (GPIO1) │                │  /dev/ttyAMA2    │                │
 │   │         │                │  @ 115200 (flow) │                │
 │   │ UART2 TX├────────────────┤ RXD3 (GPIO4)     │                │
 │   │ (GPIO17)│                │  /dev/ttyAMA3    │                │
 │   └─────────┘                │  @ 921600 (frame)│                │
 │     (downward-facing)        └─────────┬────────┘                │
 │                                        │ USB                     │
 │   Pixhawk 2.4.8              ┌─────────┴────────┐                │
 │   ┌──────────┐               │ USB Port         │                │
 │   │ USB      ├───────────────┤ /dev/ttyACM0     │                │
 │   │          │               │ @ 115200 baud    │                │
 │   │ RC IN    ├────┐          └──────────────────┘                │
 │   └──────────┘    │                                              │
 │                   │   R88 ELRS Receiver                          │
 │                   │   ┌─────────────┐                            │
 │                   └───┤ SBUS OUT    │                            │
 │                       └─────────────┘                            │
 │                                                                  │
 │   NOTES:                                                         │
 │   • TF-02 on UART0 (/dev/ttyAMA0 @ 115200)                       │
 │   • ESP32-CAM flow on UART2 (/dev/ttyAMA2 @ 115200)              │
 │   • ESP32-CAM frames on UART3 (/dev/ttyAMA3 @ 921600)            │
 │   • Pixhawk via USB (/dev/ttyACM0 @ 115200)                      │
 │   • ESP32-CAM faces DOWNWARD for optical flow                    │
 │   • RPi bridges TF-02 → Pixhawk via MAVLink DISTANCE_SENSOR      │
 └──────────────────────────────────────────────────────────────────┘
```

### ESP32-CAM Module Pinout (AI Thinker)

```
              ┌─────────────────────┐
              │    ┌───────────┐    │
              │    │  OV2640   │    │
              │    │  CAMERA   │    │
              │    │ (face ↓)  │    │
              │    └───────────┘    │
              │                     │
              │   ┌─────────────┐   │
              │   │   ESP32     │   │
              │   │   Module    │   │
              │   └─────────────┘   │
              │                     │
         5V  ─┤ 1               16  ├─ 3V3
        GND  ─┤ 2               15  ├─ IO16 ←── UART2 RX (PSRAM CS*)
       IO12  ─┤ 3               14  ├─ IO0  ←── BOOT (pull LOW to flash)
       IO13  ─┤ 4               13  ├─ GND
       IO15  ─┤ 5               12  ├─ VCC (5V)
       IO14  ─┤ 6               11  ├─ U0R  ←── GPIO3 (UART0 RX)
        IO2  ─┤ 7               10  ├─ U0T  ←── GPIO1 (UART0 TX) ──→ RPi RXD2
        IO4  ─┤ 8  [Flash LED]   9  ├─ GND
              └─────────────────────┘
                    │         │
                    │  GPIO17 │ ← UART2 TX (NOT on header — solder or remap)
                    │         │
                    └─────────┘

  * IO16 is shared with PSRAM CS. Disable PSRAM in Arduino IDE
    (QQVGA 160×120 fits in internal DRAM — PSRAM not needed).

  * GPIO17 is NOT broken out on the AI Thinker header.
    Options: (a) solder wire to ESP32 pad, (b) remap to IO12/IO13,
    (c) use a variant board with GPIO17 exposed.
```

### ESP32-CAM → RPi4 Connection Table

```
 ┌───────────────────────────────────────────────────────────────────┐
 │  ESP32-CAM Pin    Signal       RPi4 Pin         Wire Color        │
 ├───────────────────────────────────────────────────────────────────┤
 │  Pin 1  (5V)      VCC          Pin 4  (5V)      🔴 Red            │
 │  Pin 9  (GND)     Ground       Pin 6  (GND)     ⚫ Black          │
 │                                                                   │
 │──── UART0: Flow Packets (115200 baud) ────────────────────────────│
 │  Pin 10 (U0T)     TX →         Pin 27 (RXD2)    🟢 Green          │
 │  GPIO1             UART0 TX     GPIO0             /dev/ttyAMA2    │
 │                                                                   │
 │──── UART2: Raw Frames (921600 baud) ──────────────────────────────│
 │  GPIO17 *          TX →         Pin 7  (RXD3)    🔵 Blue          │
 │                     UART2 TX     GPIO4             /dev/ttyAMA3   │
 │                                                                   │
 │  * GPIO17 requires solder mod or pin remap (see pinout above)     │
 │  * Alternative: remap UART2 TX to IO12 (pin 3) or IO13 (pin 4)    │
 │    and wire to RPi Pin 7 instead                                  │
 └───────────────────────────────────────────────────────────────────┘
```

### TF-02 → RPi4 Connection Table

```
 ┌───────────────────────────────────────────────────────────────────┐
 │  TF-02 Pin     Signal       RPi4 Pin         Wire Color           │
 ├───────────────────────────────────────────────────────────────────┤
 │  VCC (Red)      5V           Pin 2  (5V)      🔴 Red              │
 │  GND (Black)    Ground       Pin 14 (GND)     ⚫ Black            │
 │  TXD (Green)    TX →         Pin 10 (RXD0)    🟢 Green            │
 │                               GPIO15           /dev/ttyAMA0       │
 │  RXD (White)    RX ←         Pin 8  (TXD0)    ⚪ White            │
 │                               GPIO14           /dev/ttyAMA0       │
 └───────────────────────────────────────────────────────────────────┘
```

### Pixhawk → RPi4

```
 ┌───────────────────────────────────────────────────────────────────┐
 │  Pixhawk 2.4.8    Signal     RPi4                                 │
 ├───────────────────────────────────────────────────────────────────┤
 │  Micro-USB         USB        USB-A port                          │
 │                                Appears as /dev/ttyACM0 @ 115200   │
 └───────────────────────────────────────────────────────────────────┘
```

### ESP32-CAM Firmware

The `.ino` firmware is located at:
```
firmware/esp32_cam/esp32_cam.ino
```

**Arduino IDE settings:**
| Setting | Value |
|---------|-------|
| Board | AI Thinker ESP32-CAM |
| PSRAM | Disabled |
| CPU Frequency | 240 MHz |
| Partition Scheme | Huge APP (3MB No OTA) |
| Upload Speed | 115200 |

Flash via FTDI adapter: connect IO0 to GND during upload, then disconnect for normal boot.

---

## 3. One-Time RPi Setup

### Enable UARTs
```bash
sudo raspi-config
# → Interface Options → Serial Port
#   Login shell over serial? → NO
#   Serial port hardware?    → YES

# Enable UART2 and UART3 in /boot/config.txt:
echo "dtoverlay=uart2" | sudo tee -a /boot/config.txt
echo "dtoverlay=uart3" | sudo tee -a /boot/config.txt
sudo reboot
```

### Add User to dialout Group
```bash
sudo usermod -aG dialout $USER
# Log out and log back in
```

### Install Dependencies
```bash
sudo apt update && sudo apt install -y python3-pip
pip3 install pymavlink pyserial opencv-python-headless numpy
```

### Clone ASCEND
```bash
git clone https://github.com/cyberaionics/ASCEND-ISRO-2026.git
cd ASCEND-ISRO-2026/ascend
```

---

## 4. CLI Usage

All modes are run from the project root (parent of `ascend/`):

```bash
# Health check only
python3 -m ascend.scheduler --mode check

# Write Pixhawk parameters only
python3 -m ascend.scheduler --mode setup

# Health check → parameter setup
python3 -m ascend.scheduler --mode all

# AutoTune monitor (pilot flies manually)
python3 -m ascend.scheduler --mode autotune

# Full autonomous hover mission
python3 -m ascend.scheduler --mode fly
```

---

## 5. State Machine (--mode fly)

```
IDLE → PREFLIGHT → ARM → TAKEOFF → HOVER → LAND → DISARM → DONE
```

| State | Behavior | Transition |
|-------|----------|------------|
| **IDLE** | Verify threads alive, GPS-denied | → PREFLIGHT always |
| **PREFLIGHT** | Heartbeat, TF-02, battery check | → ARM on pass, abort on fail |
| **ARM** | Set STABILIZE, send arm command | → TAKEOFF on armed (10s timeout) |
| **TAKEOFF** | Throttle ramp + VIO corrections | → HOVER at ≥ 0.90 m (15s timeout) |
| **HOVER** | Alt P-ctrl + VIO roll/pitch, 300s | → LAND after duration or abort |
| **LAND** | Linear throttle reduction, 5s | → DISARM at < 0.08 m or min throttle |
| **DISARM** | Send disarm, wait confirmation | → DONE (5s timeout) |
| **DONE** | Log mission summary, stop all | Terminal state |

**Safety**: `SafetyMonitor.emergency_flag` → LAND from any flight state.
**Abort**: Ctrl+C → immediate LAND + DISARM.

### Altitude Controller (Z)
```
throttle = BASE_THROTTLE (1550) + Kp_alt (200) × (target - current)
clamped to [1100, 1800] PWM
```

### VIO Controller (X-Y)
Dual-pipeline PID fusion — see Section 7.

---

## 6. Parameter Reference

All parameters written by `--mode setup`:

### Rangefinder
| Parameter          | Value | Purpose                           |
|--------------------|-------|-----------------------------------|
| `RNGFND1_TYPE`     | 10    | MAVLink rangefinder (from RPi)    |
| `RNGFND1_ORIENT`   | 25    | Downward facing                   |
| `RNGFND1_MIN_CM`   | 30    | Minimum range 0.3 m               |
| `RNGFND1_MAX_CM`   | 800   | Maximum range 8.0 m               |
| `RNGFND1_GNDCLR`   | 15    | Ground clearance offset 15 cm     |
| `EK3_RNG_USE_HGT`  | 70    | Rangefinder below 70% max range   |

### Failsafe
| Parameter          | Value | Purpose                           |
|--------------------|-------|-----------------------------------|
| `FS_GCS_ENABLE`    | 1     | Land if RPi heartbeat lost        |
| `FS_GCS_TIMEOUT`   | 3     | 3-second timeout                  |
| `FS_THR_ENABLE`    | 0     | Disable RC-loss failsafe          |
| `BATT_FS_LOW_ACT`  | 1     | Land on low battery               |
| `BATT_LOW_VOLT`    | 14.0  | 3.5 V/cell threshold             |
| `BATT_CRT_VOLT`    | 13.2  | 3.3 V/cell critical              |

### Flight
| Parameter          | Value | Purpose                           |
|--------------------|-------|-----------------------------------|
| `MOT_SPIN_ARM`     | 0.10  | Minimum spin when armed           |
| `MOT_SPIN_MIN`     | 0.15  | Minimum spin in flight            |
| `MOT_THST_HOVER`   | 0.35  | Hover throttle estimate           |
| `ARMING_CHECK`     | 1     | Enable all arming checks          |
| `DISARM_DELAY`     | 10    | 10 s disarm delay after landing   |

---

## 7. VIO Architecture — Dual-Pipeline Fusion

```
ESP32-CAM (OV2640, 160×120 QQVGA, downward-facing)
    │                                │
    │ UART0 (ttyAMA2 @ 115200)       │ UART2 (ttyAMA3 @ 921600)
    │ 8-byte flow packets @ 30 Hz    │ Raw grayscale frames @ ~5 fps
    ▼                                ▼
ESP32CamReader                   ESP32FrameReader
    │ flow_dx, flow_dy (×100)        │ latest_frame (120×160 numpy)
    │ quality (0–30)                 ▼
    │                            ORBFlowProcessor
    │                                │ ORB keypoints + homography
    │                                │ flow_dx, flow_dy, quality
    ▼                                ▼
┌────────────────────────────────────────────────────────┐
│                   VIOStabilizer (20 Hz)                │
│                                                        │
│  ESP32 Pipeline                ORB Pipeline            │
│  ┌──────────────┐              ┌──────────────┐        │
│  │ Safety gates │              │ Safety gates │        │
│  │ Deadzone     │              │ Deadzone     │        │
│  │ Flow → vel   │              │ Flow → vel   │        │
│  │ PID:         │              │ PID:         │        │
│  │  Kp = 0.4    │              │  Kp = 0.5    │        │
│  │  Ki = 0.05   │              │  Ki = 0.08   │        │
│  │  Kd = 0.02   │              │  Kd = 0.03   │        │
│  └──────┬───────┘              └──────┬───────┘        │
│         │ weight 0.4                  │ weight 0.6     │
│         └───────────┬─────────────────┘                │
│                     ▼                                  │
│            Renormalized Weighted Sum                   │
│            I-term reset on gate failure                │
│                     │                                  │
│         ┌───────────┴────────────┐                     │
│         │ roll_pwm = 1500 ± corr │                     │
│         │ pitch_pwm= 1500 ± corr │                     │
│         │ clamped ±100 PWM       │                     │
│         └────────────────────────┘                     │
└────────────────────────────────────────────────────────┘
                     │
                     ▼
            StateMachine (TAKEOFF / HOVER)
                     │
         send_rc_override(roll, pitch, throttle)
                     │
                     ▼
            Pixhawk (STABILIZE mode)
```

### VIO Constants
| Parameter              | Value  | Purpose                            |
|------------------------|--------|------------------------------------|
| `VIO_FOCAL_LENGTH_PX`  | 100.0  | OV2640 focal length @ QQVGA       |
| `VIO_DEADZONE_PX`      | 2      | Ignore flow below 2 px            |
| `VIO_MIN_QUALITY`      | 5      | Minimum valid features/blocks      |
| `VIO_MAX_CORRECTION_PWM`| 100   | Max ±100 PWM from neutral         |
| `VIO_DATA_TIMEOUT`     | 0.5 s  | Stale data threshold              |
| `VIO_MIN_ALT_M`        | 0.3 m  | VIO disabled below 30 cm          |

### ESP32 Flow Packet Protocol
```
Byte 0:   0xAB  (header 1)
Byte 1:   0xCD  (header 2)
Byte 2-3: flow_x (int16, little-endian, actual_pixels × 100)
Byte 4-5: flow_y (int16, little-endian, actual_pixels × 100)
Byte 6:   quality (uint8, valid SAD blocks, 0–30)
Byte 7:   checksum = XOR of bytes 2,3,4,5,6
```

### ESP32 Raw Frame Protocol
```
Byte 0:         0xAA  (sync 1)
Byte 1:         0x55  (sync 2)
Bytes 2–19201:  pixel data (160×120 = 19200 bytes, row-major grayscale)
Byte 19202:     XOR checksum of all pixel data bytes
Total: 19203 bytes per frame → ~4.8 fps at 921600 baud
```

### Fusion Rules
- **Both pipelines active**: correction = 0.4 × ESP32 + 0.6 × ORB (renormalized)
- **Only one active**: that pipeline at full weight (graceful degradation)
- **Neither active**: zero correction (neutral 1500)
- **Gate failure**: I-term reset on the failing pipeline independently

---

## 8. Thread Model

| # | Thread | File | Rate | Purpose |
|---|--------|------|------|---------|
| 1 | TF02Reader | `hardware/tf02.py` | 100 Hz | `.distance_m` from LiDAR |
| 2 | ESP32CamReader | `hardware/esp32_cam.py` | 30 Hz | `.flow_dx`, `.flow_dy`, `.quality` |
| 3 | ESP32FrameReader | `hardware/esp32_cam_frame.py` | ~5 Hz | `.latest_frame` (numpy array) |
| 4 | ORBFlowProcessor | `hardware/esp32_cam_frame.py` | ~5 Hz | ORB flow from frames |
| 5 | RangefinderBridge | `threads/bridge.py` | 20 Hz | TF02 → Pixhawk DISTANCE_SENSOR |
| 6 | HeartbeatSender | `threads/bridge.py` | 1 Hz | Companion heartbeat |
| 7 | TelemetryStreamer | `threads/bridge.py` | 1 Hz | UDP JSON to laptop |
| 8 | SafetyMonitor | `threads/bridge.py` | 10 Hz | Battery, altitude, geofence |
| 9 | VIOStabilizer | `threads/vio_stabilizer.py` | 20 Hz | Dual-pipeline PID fusion |
| — | StateMachine | `scheduler.py` | 20 Hz | Main mission loop (not a thread) |

All threads: `daemon=True`, `threading.Lock` properties, `_running` flag, `.stop()` method.

---

## 9. File Structure

```
ASCEND-ISRO-2026/
├── firmware/
│   └── esp32_cam/
│       └── esp32_cam.ino       ESP32-CAM optical flow + frame firmware
│
└── ascend/                      Python package (RPi4)
    ├── __init__.py              Package init
    ├── __main__.py              Entry: python3 -m ascend.scheduler
    ├── config.py                All constants — no magic numbers elsewhere
    ├── logger.py                ANSI colour-coded timestamped logger
    ├── hardware/
    │   ├── __init__.py
    │   ├── tf02.py              TF-02 LiDAR frame parser (daemon thread)
    │   ├── pixhawk.py           pymavlink wrapper (thread-safe)
    │   ├── esp32_cam.py         ESP32 flow packet reader (0xAB/0xCD, daemon)
    │   ├── esp32_cam_frame.py   ESP32 raw frame reader + ORB flow processor
    │   └── cv_flow.py           RPi camera Shi-Tomasi + LK flow (legacy)
    ├── threads/
    │   ├── __init__.py
    │   ├── bridge.py            RangefinderBridge, HeartbeatSender,
    │   │                         TelemetryStreamer, SafetyMonitor
    │   └── vio_stabilizer.py    Dual-pipeline VIO PID fusion (daemon)
    ├── checks/
    │   ├── __init__.py
    │   ├── health.py            Pre-flight diagnostic checks
    │   ├── setup.py             Pixhawk parameter writer
    │   └── monitor.py           Passive AutoTune session monitor
    ├── scheduler.py             StateMachine (8 states) + Scheduler CLI
    ├── requirements.txt         Python dependencies
    ├── Changes.md               Changelog
    └── README.md                This file
```

---

## 10. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `Cannot open /dev/ttyACM0` | Pixhawk not connected / powered | Check USB cable; `ls /dev/ttyACM*` |
| `No heartbeat received` | Wrong baud or port | Verify `PIXHAWK_BAUD = 115200` |
| `No TF-02 readings` | UART not enabled | `raspi-config` → enable serial hardware |
| TF-02 reads 0 cm | TXD/RXD swapped | Swap wires on TF-02 |
| `Cannot open /dev/ttyAMA2` | UART2 not enabled | Add `dtoverlay=uart2` to `/boot/config.txt` |
| `Cannot open /dev/ttyAMA3` | UART3 not enabled | Add `dtoverlay=uart3` to `/boot/config.txt` |
| `Arming FAILED within timeout` | Pre-arm check failure | Connect Mission Planner, read HUD |
| VIO corrections always zero | Quality too low / camera blocked | Ensure camera faces down with clear view; check lighting |
| Drone drifts with VIO active | PID gains too low / poor surface | Use textured ground; increase `ESP_VIO_KP` cautiously |
| ORB flow has low quality | Few features in frame | Ensure ground has visual texture (not plain surface) |
| High vibration (> 60 m/s²) | Poor motor balance / loose screws | Balance props, tighten fasteners, add foam |
| IMU clipping detected | Extreme vibration | **DO NOT FLY** — isolate FC with foam damping |
| `FS_GCS` failsafe triggers | RPi stopped sending heartbeats | Check RPi process running; check USB link |
| Battery critical (< 13.2 V) | Battery depleted | Land immediately, charge battery |

---

## License

Internal use — Bayes Frontier, IIT Dharwad. IRoC-U 2026 competition entry.
