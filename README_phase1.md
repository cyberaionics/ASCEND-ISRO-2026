# ASCEND Phase 1 — Flight Readiness

**Autonomous Surveyor Challenge for Exploration, Navigation and Dynamics**
Team Bayes Frontier · IIT Dharwad · IRoC-U 2026

Phase 1 covers qualification tasks T1–T4:
stable takeoff, 5-minute hover, controlled landing, and low-battery failsafe.

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
| Rangefinder          | Benewake TF-02 LiDAR (0.3–8.0m, 100Hz)        |
| RC Transmitter       | Radiomaster (ELRS protocol)                     |
| RC Receiver          | Radiomaster R88 (ELRS, SBUS output)             |
| AUW                  | ~1.8 kg                                         |

---

## 2. Hardware Wiring

```
 ┌────────────────────────────────────────────────────────┐
 │                  WIRING DIAGRAM                        │
 ├────────────────────────────────────────────────────────┤
 │                                                        │
 │   TF-02 LiDAR               Raspberry Pi 5            │
 │   ┌─────────┐               ┌──────────────┐          │
 │   │ VCC (5V)├───────────────┤ 5V  (Pin 2)  │          │
 │   │ GND     ├───────────────┤ GND (Pin 6)  │          │
 │   │ TXD     ├───────────────┤ RXD (GPIO15) │          │
 │   │ RXD     ├───────────────┤ TXD (GPIO14) │          │
 │   └─────────┘               └──────┬───────┘          │
 │                                     │ USB              │
 │                                     │                  │
 │   Pixhawk 2.4.8             ┌──────┴───────┐          │
 │   ┌──────────┐              │ USB-C Port   │          │
 │   │ USB      ├──────────────┤ RPi5         │          │
 │   │          │              └──────────────┘          │
 │   │ RC IN    ├──┐                                     │
 │   └──────────┘  │                                     │
 │                  │   R88 ELRS Receiver                 │
 │                  │   ┌─────────────┐                  │
 │                  └───┤ SBUS OUT    │                  │
 │                      │ VCC → 5V    │                  │
 │                      │ GND → GND   │                  │
 │                      └─────────────┘                  │
 │                                                        │
 │   NOTES:                                               │
 │   • TF-02 is on RPi5 UART, NOT Pixhawk SERIAL4        │
 │   • RPi5 bridges TF-02 → Pixhawk via MAVLink          │
 │   • Pixhawk USB appears as /dev/ttyACM0 on RPi5       │
 │   • TF-02 UART appears as /dev/serial0 on RPi5        │
 └────────────────────────────────────────────────────────┘
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
git clone <repo-url>
cd <repo>/ascend
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

## 5. Day-by-Day Procedure

### Day 1 — Bench Work (indoors, props OFF)

1. Install battery, power Pixhawk + RPi5.
2. SSH into RPi5 or connect a monitor/keyboard.
3. Run `--mode check`:
   - Verify PID values are read (all default → AutoTune needed).
   - Confirm vibration levels are EXCELLENT / GOOD.
   - Confirm TF-02 reads valid distances (point at ceiling/floor).
4. Run `--mode setup`:
   - Writes all parameters (rangefinder, failsafe, autotune, flight).
   - Verify every line shows `[ OK ]`.
5. Power-cycle the Pixhawk to load new parameters.
6. Re-run `--mode check` to confirm parameters took effect.

### Day 2 — AutoTune (outdoors, open field)

1. Props ON, fly area clear, battery fully charged.
2. Start `--mode autotune` on RPi5 (SSH from laptop over Wi-Fi).
3. Arm and take off manually in AltHold mode (use TX).
4. Hover at 4–5 m altitude in calm air.
5. Switch to AutoTune via RC switch (the monitor will report it).
6. Wait for "AutoTune COMPLETE" message in the terminal.
7. Switch back to AltHold, land, disarm.
8. Check `autotune_results_*.json` for saved PID values.
9. If not completed → charge battery, repeat from step 3.

### Day 3 — Autonomous Hover Flight

1. Ensure no TX is powered on (TX override will engage STABILIZE).
2. Start `--mode fly` on RPi5.
3. The drone will:
   - Arm in GUIDED mode
   - Take off to 4 m
   - Switch to LOITER and hover for 5 minutes
   - RTL to home position
   - Land and disarm
4. Monitor telemetry on the laptop (UDP port 14550).
5. If anything goes wrong: power on the TX → instant STABILIZE override.

---

## 6. Parameter Reference

All parameters written by `--mode setup`:

### Rangefinder (MAVLink bridge)
| Parameter          | Value | Purpose                           |
|--------------------|-------|-----------------------------------|
| `RNGFND1_TYPE`     | 10    | MAVLink rangefinder (from RPi5)   |
| `RNGFND1_ORIENT`   | 25    | Downward facing                   |
| `RNGFND1_MIN_CM`   | 30    | Minimum range 0.3 m              |
| `RNGFND1_MAX_CM`   | 800   | Maximum range 8.0 m              |
| `RNGFND1_GNDCLR`   | 15    | Ground clearance offset 15 cm    |
| `EK3_RNG_USE_HGT`  | 70    | Use rangefinder below 70% range  |

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
| `LOIT_ACC_MAX`     | 250   | Max Loiter acceleration            |
| `RTL_ALT`          | 400   | Climb to 4 m before RTL           |
| `RTL_LOIT_TIME`    | 3000  | Hover 3 s at home before landing  |
| `ARMING_CHECK`     | 1     | Enable all arming checks          |
| `DISARM_DELAY`     | 10    | 10 s disarm delay after landing   |
| `MOT_SPIN_ARM`     | 0.10  | Minimum spin when armed            |
| `MOT_SPIN_MIN`     | 0.15  | Minimum spin in flight             |
| `MOT_THST_HOVER`   | 0.35  | Hover throttle estimate            |

---

## 7. Troubleshooting

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

---

## 8. Architecture Overview

```
┌─────────────────────────────────────────────────────┐
│                    RPi5 (Python)                     │
│                                                      │
│  ┌──────────┐  ┌───────────────┐  ┌──────────────┐ │
│  │ TF02     │  │ Rangefinder   │  │ Heartbeat    │ │
│  │ Reader   ├──┤ Bridge (20Hz) ├──┤ Sender (1Hz) │ │
│  │ (Thread) │  │ (Thread)      │  │ (Thread)     │ │
│  └──────────┘  └───────┬───────┘  └──────┬───────┘ │
│                         │                  │         │
│  ┌──────────────────────┴──────────────────┴──────┐ │
│  │              PixhawkLink (MAVLink)              │ │
│  └───────────────────────┬────────────────────────┘ │
│                           │                          │
│  ┌────────────┐  ┌───────┴──────┐  ┌────────────┐  │
│  │ Safety     │  │ State        │  │ Telemetry  │  │
│  │ Monitor    │  │ Machine      │  │ Streamer   │  │
│  │ (Thread)   │  │ (Main Loop)  │  │ (Thread)   │  │
│  └────────────┘  └──────────────┘  └──────┬─────┘  │
│                                            │ UDP     │
└────────────────────────────────────────────┼────────┘
                                              │
                   USB (/dev/ttyACM0)         │ Wi-Fi
                      │                        ▼
            ┌─────────┴──────────┐    ┌──────────────┐
            │  Pixhawk 2.4.8    │    │   Laptop     │
            │  (ArduCopter)     │    │  (Telemetry) │
            └───────────────────┘    └──────────────┘
```

---

## 9. File Structure

```
ascend/
├── __init__.py          Package init
├── __main__.py          Entry: python3 -m ascend.scheduler
├── config.py            All constants — no magic numbers elsewhere
├── logger.py            ANSI colour-coded timestamped logger
├── hardware/
│   ├── __init__.py
│   ├── tf02.py          TF-02 LiDAR frame parser (daemon thread)
│   └── pixhawk.py       pymavlink wrapper (thread-safe)
├── threads/
│   ├── __init__.py
│   └── bridge.py        RangefinderBridge, HeartbeatSender,
│                         TelemetryStreamer, SafetyMonitor
├── checks/
│   ├── __init__.py
│   ├── health.py        3 pre-flight diagnostic checks
│   ├── setup.py         Pixhawk parameter writer
│   └── monitor.py       Passive AutoTune session monitor
├── scheduler.py         StateMachine + Scheduler CLI entry point
└── README_phase1.md     This file
```

---

## License

Internal use — Bayes Frontier, IIT Dharwad. IRoC-U 2026 competition entry.
