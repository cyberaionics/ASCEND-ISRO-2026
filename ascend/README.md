# ASCEND Phase 2 — Flight Readiness + VIO Hover Stabilization

**Autonomous Surveyor Challenge for Exploration, Navigation and Dynamics**
Team Bayes Frontier · IIT Dharwad · IRoC-U 2026

Phase 1 covers qualification tasks T1–T4:
stable takeoff, 5-minute hover, controlled landing, and low-battery failsafe.

Phase 2 adds VIO-based X-Y hover stabilization using ESP32-CAM optical flow,
while Z-axis altitude hold continues via TF-02 LiDAR.

---

## 1. Hardware Components

| Component            | Model / Spec                                   |
|----------------------|------------------------------------------------|
| Frame                | Holybro S500 V2, carbon fiber, X-config, 280mm |
| Motors               | T-Motor F-Series 2306, 950KV × 4               |
| ESCs                 | SpeedyBee 35A BLHeli_S 4-in-1                  |
| Propellers           | 6010 (6″ 4.5-pitch) 2-blade polycarbonate      |
| Battery              | 14.8V 4S 4500mAh 40C LiPo                      |
| Flight Controller    | Pixhawk 2.4.8 — ArduCopter (latest stable)     |
| Companion Computer   | Raspberry Pi 5, 4GB RAM                        |
| Rangefinder          | Benewake TF-02 LiDAR (0.3–8.0m, 100Hz)         |
| Optical Flow Camera  | ESP32-CAM (OV2640, 96×96 grayscale, 30Hz)      |
| RC Transmitter       | Radiomaster (ELRS protocol)                    |
| RC Receiver          | Radiomaster R88 (ELRS, SBUS output)            |
| AUW                  | ~1.8 kg                                        |

---

## 2. Hardware Wiring

```
 ┌──────────────────────────────────────────────────────────────┐
 │                      WIRING DIAGRAM                          │
 ├──────────────────────────────────────────────────────────────┤
 │                                                              │
 │   TF-02 LiDAR               Raspberry Pi 5                   │
 │   ┌─────────┐               ┌──────────────────┐             │
 │   │ VCC (5V)├───────────────┤ 5V  (Pin 2)      │             │
 │   │ GND     ├───────────────┤ GND (Pin 6)      │             │
 │   │ TXD     ├───────────────┤ RXD0 (GPIO15)    │             │
 │   │ RXD     ├───────────────┤ TXD0 (GPIO14)    │             │
 │   └─────────┘               │                  │             │
 │                             │  /dev/ttyAMA0    │             │
 │   ESP32-CAM                 │                  │             │
 │   ┌─────────┐               │                  │             │
 │   │ VCC (5V)├───────────────┤ 5V  (Pin 4)      │             │
 │   │ GND     ├───────────────┤ GND (Pin 9)      │             │
 │   │ U1 TXD  ├───────────────┤ RXD2 (GPIO0)     │             │
 │   │ U1 RXD  ├───────────────┤ TXD2 (GPIO1)     │             │
 │   └─────────┘               │                  │             │
 │     (downward-facing)       │  /dev/ttyAMA2    │             │
 │                             └─────────┬────────┘             │
 │                                       │ USB                  │
 │                                       │                      │
 │   Pixhawk 2.4.8              ┌────────┴─────────┐            │
 │   ┌──────────┐               │ USB-C Port       │            │
 │   │ USB      ├───────────────┤ RPi5             │            │
 │   │          │               └──────────────────┘            │
 │   │ RC IN    ├───┐                                           │
 │   └──────────┘   │                                           │
 │                  │   R88 ELRS Receiver                       │
 │                  │   ┌─────────────┐                         │
 │                  └───┤ SBUS OUT    │                         │
 │                      │ VCC → 5V    │                         │
 │                      │ GND → GND   │                         │
 │                      └─────────────┘                         │
 │                                                              │
 │   NOTES:                                                     │
 │   • TF-02 is on RPi5 UART0 (/dev/ttyAMA0)                    │
 │   • ESP32-CAM is on RPi5 UART2 (/dev/ttyAMA2)                │
 │   • ESP32-CAM faces DOWNWARD for optical flow                │
 │   • RPi5 bridges TF-02 → Pixhawk via MAVLink                 │
 │   • Pixhawk USB appears as /dev/ttyACM0 on RPi5              │
 └──────────────────────────────────────────────────────────────┘
```

---

## 3. One-Time RPi5 Setup

### Enable UART
```bash
sudo raspi-config
# → Interface Options → Serial Port
#   Login shell over serial? → NO
#   Serial port hardware?    → YES
sudo reboot
```

### Add User to dialout Group
```bash
sudo usermod -aG dialout $USER
# Log out and log back in
```

### Install Dependencies
```bash
sudo apt update
sudo apt install -y python3-pip
pip3 install pymavlink pyserial
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
# Day 1 — Bench: health check only
python3 -m ascend.scheduler --mode check

# Day 1 — Bench: write Pixhawk parameters only
python3 -m ascend.scheduler --mode setup

# Day 1 — Bench: full health check → parameter setup
python3 -m ascend.scheduler --mode all

# Day 2 — Field: AutoTune monitor (pilot flies manually)
python3 -m ascend.scheduler --mode autotune

# Day 3 — Field: full autonomous hover mission
python3 -m ascend.scheduler --mode fly
```

---


## 5. Parameter Reference

All parameters written by `--mode setup`:

### Rangefinder (MAVLink bridge)
| Parameter          | Value | Purpose                           |
|--------------------|-------|-----------------------------------|
| `RNGFND1_TYPE`     | 10    | MAVLink rangefinder (from RPi5)   |
| `RNGFND1_ORIENT`   | 25    | Downward facing                   |
| `RNGFND1_MIN_CM`   | 30    | Minimum range 0.3 m               |
| `RNGFND1_MAX_CM`   | 800   | Maximum range 8.0 m               |
| `RNGFND1_GNDCLR`   | 15    | Ground clearance offset 15 cm     |
| `EK3_RNG_USE_HGT`  | 70    | Use rangefinder below 70% range   |

### AutoTune (PIDs reset to default before tuning)
| Parameter          | Value  | Purpose                          |
|--------------------|--------|----------------------------------|
| `ATC_RAT_PIT_P`    | 0.135  | Pitch rate P (default)           |
| `ATC_RAT_PIT_I`    | 0.135  | Pitch rate I (default)           |
| `ATC_RAT_PIT_D`    | 0.0036 | Pitch rate D (default)           |
| `ATC_RAT_RLL_P`    | 0.135  | Roll rate P (default)            |
| `ATC_RAT_RLL_I`    | 0.135  | Roll rate I (default)            |
| `ATC_RAT_RLL_D`    | 0.0036 | Roll rate D (default)            |
| `AUTOTUNE_AGGR`    | 0.1    | Conservative for heavy payload   |
| `AUTOTUNE_AXES`    | 3      | Tune pitch + roll                |
| `AUTOTUNE_MIN_D`   | 0.001  | Minimum derivative gain          |

### Failsafe
| Parameter          | Value | Purpose                           |
|--------------------|-------|-----------------------------------|
| `FS_GCS_ENABLE`    | 1     | Land if RPi5 heartbeat lost       |
| `FS_GCS_TIMEOUT`   | 3     | 3-second timeout                  |
| `FS_THR_ENABLE`    | 0     | Disable RC-loss failsafe          |
| `BATT_FS_LOW_ACT`  | 1     | Land on low battery               |
| `BATT_LOW_VOLT`    | 14.0  | 3.5 V/cell threshold              |
| `BATT_CRT_VOLT`    | 13.2  | 3.3 V/cell critical threshold     |

### Flight
| Parameter          | Value | Purpose                           |
|--------------------|-------|-----------------------------------|
| `LOIT_SPEED`       | 500   | Max 5 m/s in Loiter               |
| `LOIT_ACC_MAX`     | 250   | Max Loiter acceleration           |
| `RTL_ALT`          | 400   | Climb to 4 m before RTL           |
| `RTL_LOIT_TIME`    | 3000  | Hover 3 s at home before landing  |
| `ARMING_CHECK`     | 1     | Enable all arming checks          |
| `DISARM_DELAY`     | 10    | 10 s disarm delay after landing   |
| `MOT_SPIN_ARM`     | 0.10  | Minimum spin when armed           |
| `MOT_SPIN_MIN`     | 0.15  | Minimum spin in flight            |
| `MOT_THST_HOVER`   | 0.35  | Hover throttle estimate           |

### VIO Stabilization (ESP32-CAM optical flow)
| Parameter              | Value  | Purpose                            |
|------------------------|--------|------------------------------------|
| `ESP32_CAM_PORT`       | /dev/ttyAMA2 | ESP32-CAM UART port          |
| `ESP32_CAM_BAUD`       | 115200 | ESP32-CAM baud rate                |
| `VIO_KP`               | 0.5    | P-gain (PWM per m/s drift)         |
| `VIO_DEADZONE_PX`      | 2      | Ignore flow below 2 px             |
| `VIO_MIN_QUALITY`      | 5      | Minimum tracked features           |
| `VIO_MAX_CORRECTION_PWM`| 100   | Max ±100 PWM from neutral          |
| `VIO_FOCAL_LENGTH_PX`  | 60.0   | OV2640 focal length @ 96×96        |
| `VIO_DATA_TIMEOUT`     | 0.5    | Stale data threshold (seconds)     |
| `VIO_MIN_ALT_M`        | 0.3    | VIO disabled below 30 cm           |

---

## 6. Troubleshooting

| Symptom                                    | Likely Cause                       | Fix                                            |
|--------------------------------------------|------------------------------------|-------------------------------------------------|
| `Cannot open /dev/ttyACM0`                 | Pixhawk not connected / powered    | Check USB cable; `ls /dev/ttyACM*`              |
| `No heartbeat received`                    | Wrong baud rate or port            | Verify `PIXHAWK_PORT` and `PIXHAWK_BAUD`       |
| `No TF-02 readings received`              | UART not enabled on RPi5           | Run `raspi-config`, enable serial hardware      |
| TF-02 reads 0 cm constantly               | Wiring TXD/RXD swapped            | Swap GPIO14 ↔ GPIO15 wires on TF-02            |
| `Arming FAILED within timeout`            | Pre-arm check failure              | Connect via Mission Planner, read HUD messages  |
| High vibration (> 60 m/s²)                | Poor motor balance or loose screws | Balance props, tighten all fasteners, add foam   |
| IMU clipping detected                      | Extreme vibration                  | DO NOT FLY — isolate FC with foam damping       |
| `WRITE FAILED` on parameter               | Parameter name typo or read-only   | Check ArduCopter param list for exact name      |
| Drone enters STABILIZE unexpectedly        | TX override — RC transmitter is on | Power off the transmitter for autonomous mode   |
| Wi-Fi link loss → emergency LAND           | RPi5 lost Wi-Fi to laptop hotspot  | Ensure stable Wi-Fi; reduce distance            |
| `FS_GCS` failsafe triggers on Pixhawk     | RPi5 stopped sending heartbeats    | Check RPi5 process is running; check USB link   |
| AutoTune never completes                   | Wind too strong / battery too low  | Fly in calm conditions, use full battery        |
| `Cannot open /dev/ttyAMA2`                 | ESP32-CAM not connected / UART2 disabled | Check wiring; enable UART2 in `/boot/config.txt` with `dtoverlay=uart2` |
| VIO corrections always zero                | ESP32-CAM quality too low / camera obstructed | Ensure camera faces down with clear view of ground; check lighting |
| Drone still drifts with VIO active         | `VIO_KP` too low or poor optical flow surface | Increase `VIO_KP` cautiously; use textured ground surface |

---

## 7. Architecture Overview

```
┌──────────────────────────────────────────────────────────────┐
│                       RPi5 (Python)                         │
│                                                              │
│  ┌──────────┐  ┌───────────────┐  ┌──────────────┐           │
│  │ TF02     │  │ Rangefinder   │  │ Heartbeat    │           │
│  │ Reader   ├──┤ Bridge (20Hz) ├──┤ Sender (1Hz) │           │
│  │ (Thread) │  │ (Thread)      │  │ (Thread)     │           │
│  └────┬─────┘  └───────┬───────┘  └──────┬───────┘           │
│       │                │                 │                   │
│  ┌────┴─────┐  ┌───────┴─────────────────┴─────────────────┐ │
│  │ ESP32-CAM│  │              PixhawkLink (MAVLink)        │ │
│  │ Reader   │  └───────────────────────┬───────────────────┘ │
│  │ (Thread) │                          │                     │
│  └────┬─────┘                          │                     │
│       │                                │                     │
│  ┌────┴──────────┐                     │                     │
│  │ VIO           │                     │                     │
│  │ Stabilizer    ├─── roll/pitch ──┐   │                     │
│  │ (Thread, 20Hz)│   corrections   │   │                     │
│  └───────────────┘                 │   │                     │
│                                    ▼   │                     │
│  ┌────────────┐  ┌─────────────────┴┐  ┌────────────┐        │
│  │ Safety     │  │ State Machine    │  │ Telemetry  │        │
│  │ Monitor    │  │ (Main Loop)      │  │ Streamer   │        │
│  │ (Thread)   │  │ Z=TF02, XY=VIO   │  │ (Thread)   │        │
│  └────────────┘  └──────────────────┘  └──────┬─────┘        │
│                                               │ UDP          │
└───────────────────────────────────────────────┼──────────────┘
                                                │
               USB (/dev/ttyACM0)               │ Wi-Fi
                      │                         ▼
            ┌─────────┴─────────┐        ┌──────────────┐
            │  Pixhawk 2.4.8    │        │   Laptop     │
            │  (ArduCopter)     │        │  (Telemetry) │
            └───────────────────┘        └──────────────┘
```

---

## 8. File Structure

```
ascend/
├── __init__.py          Package init
├── __main__.py          Entry: python3 -m ascend.scheduler
├── config.py            All constants — no magic numbers elsewhere
├── logger.py            ANSI colour-coded timestamped logger
├── hardware/
│   ├── __init__.py
│   ├── tf02.py          TF-02 LiDAR frame parser (daemon thread)
│   ├── pixhawk.py       pymavlink wrapper (thread-safe)
│   └── esp32_cam.py     ESP32-CAM optical flow reader (daemon thread)
├── threads/
│   ├── __init__.py
│   ├── bridge.py        RangefinderBridge, HeartbeatSender,
│   │                     TelemetryStreamer, SafetyMonitor
│   └── vio_stabilizer.py VIO X-Y hover stabilizer (daemon thread)
├── checks/
│   ├── __init__.py
│   ├── health.py        3 pre-flight diagnostic checks
│   ├── setup.py         Pixhawk parameter writer
│   └── monitor.py       Passive AutoTune session monitor
├── scheduler.py         StateMachine + Scheduler CLI entry point
├── requirements.txt     Python dependencies
├── Changes.md           Changelog
└── README.md            This file
```

---

## License

Internal use — Bayes Frontier, IIT Dharwad. IRoC-U 2026 competition entry.
