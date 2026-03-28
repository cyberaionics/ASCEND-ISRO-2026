# ASCEND Phase 2 — Autonomous Flight

**Autonomous Surveyor Challenge for Exploration, Navigation and Dynamics**
Team Bayes Frontier · IIT Dharwad · IRoC-U 2026

Phase 2 runs the complete autonomous hover mission:
takeoff → 5-minute hover at 4 m → RTL → landing.

---

## 1. Prerequisites

Phase 1 must be fully complete before running Phase 2:

- [x] Pixhawk calibrated (accel, compass, RC, ESC)
- [x] AutoTune completed — custom PIDs saved
- [x] Pixhawk parameters written (`--mode setup` from Phase 1)
- [x] TF-02 wired to RPi5 UART, producing valid readings
- [x] `pymavlink` and `pyserial` installed on RPi5
- [x] RPi5 connected to laptop Wi-Fi hotspot

---

## 2. Starting the Mission

### On RPi5 (SSH from laptop):

```bash
cd /path/to/project
python3 -m ascend.scheduler --mode fly
```

The system starts in **IDLE** state, waiting for a start command.

### From Laptop (send start command):

```python
import json, socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
RPi5_IP = "10.100.149.41"  # RPi5's IP on the hotspot network
CMD_PORT = 14551

# Send start command
cmd = json.dumps({"cmd": "start_mission"}).encode()
sock.sendto(cmd, (RPi5_IP, CMD_PORT))
```

Or use `netcat`:
```bash
echo '{"cmd": "start_mission"}' | nc -u 10.100.149.41 14551
```

---

## 3. Laptop Requirements

The laptop must send two types of UDP packets to RPi5:

### Heartbeat (every 1.0 second)
```json
{"cmd": "heartbeat", "timestamp": "2026-03-18T10:00:00"}
```
If the laptop stops sending heartbeats for > 3 seconds during flight,
the SafetyMonitor triggers an emergency landing.

### Commands (on demand)
```json
{"cmd": "start_mission"}
{"cmd": "abort_mission"}
```

**Example heartbeat loop (Python):**
```python
import json, socket, time, datetime

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
dest = ("10.100.149.41", 14551)

while True:
    pkt = json.dumps({
        "cmd": "heartbeat",
        "timestamp": datetime.datetime.now().isoformat()
    }).encode()
    sock.sendto(pkt, dest)
    time.sleep(1.0)
```

---

## 4. Reading Telemetry

RPi5 streams JSON telemetry to the laptop at 1 Hz on UDP port **14550**.

**Listener example:**
```python
import json, socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 14550))

while True:
    data, addr = sock.recvfrom(4096)
    pkt = json.loads(data)
    print(f"[{pkt['timestamp']}] State={pkt['state']} "
          f"Alt={pkt['altitude_m']:.1f}m Batt={pkt['battery_pct']}%")
```

**Telemetry packet fields:**

| Field              | Type          | Description                         |
|--------------------|---------------|-------------------------------------|
| `timestamp`        | ISO8601       | RPi5 local time                     |
| `state`            | string        | Current flight state                |
| `altitude_m`       | float         | Altitude from LOCAL_POSITION_NED    |
| `target_alt_m`     | float         | Config target (4.0 m)               |
| `hover_remaining`  | float / null  | Seconds left in hover timer         |
| `battery_pct`      | int           | Battery percentage 0–100            |
| `battery_volt`     | float         | Battery voltage                     |
| `armed`            | bool          | Pixhawk armed state                 |
| `fc_mode`          | string        | ArduCopter mode name                |
| `manual_mode`      | bool          | TX override active                  |
| `tf02_dist_m`      | float / null  | TF-02 distance reading              |
| `dist_to_home_m`   | float / null  | Distance to home (NED)              |
| `emergency_reason` | string / null | Set when in EMERGENCY state         |
| `vib_x/y/z`        | float         | Vibration levels                    |

---

## 5. State Transition Table

| From        | To          | Trigger                                  | RPi5 Action                     |
|-------------|-------------|------------------------------------------|---------------------------------|
| IDLE        | PREFLIGHT   | start_mission + no TX + batt ≥ 9.5 V     | Begin 4-gate preflight          |
| PREFLIGHT   | TAKEOFF     | All gates pass + armed confirmed         | Send TAKEOFF command            |
| PREFLIGHT   | IDLE        | Any gate fails or arming timeout         | Log failure reason              |
| TAKEOFF     | HOVER       | Alt stable ±0.2m for 2.0s                | Set LOITER, start timer         |
| HOVER       | RETURN      | 300s timer expires                       | Set RTL                         |
| HOVER       | EMERGENCY   | Batt < 9.0V or safety trigger            | Set LAND                        |
| RETURN      | LAND        | Distance to home < 1.5m                  | Set LAND                        |
| LAND        | IDLE        | Touchdown confirmed (3 conditions)       | Mission complete                |
| EMERGENCY   | IDLE        | Touchdown confirmed                      | Reset flags                     |
| **Any**     | EMERGENCY   | SafetyMonitor trigger or abort_mission   | Set LAND immediately            |
| **Any**     | STABILIZE   | TX detected (debounced 500ms)            | Pause commands, hand to pilot   |
| STABILIZE   | IDLE        | TX lost (debounced 300ms)                | Return to IDLE                  |

### Preflight Gates
1. Battery voltage ≥ 9.5V
2. TF-02 producing valid readings
3. EKF healthy (position + velocity flags)
4. No emergency flag set

### Touchdown Conditions (all three must be true)
1. `EXTENDED_SYS_STATE.landed_state == ON_GROUND`
2. TF-02 < 10 cm for > 1.0 second continuously
3. Pixhawk disarmed (HEARTBEAT armed bit = 0)

---

## 6. TX Override Procedure

The RC transmitter acts as a **hardware kill-switch** for autonomous mode.

### When to use it:
- Drone drifting dangerously
- Any unexpected behaviour
- Need to take manual control for any reason

### What happens:
1. Power ON the TX (Radiomaster ELRS)
2. TXOverrideWatcher detects valid RC signal (5 consecutive readings, 500ms)
3. RPi5 sends `SET_MODE: STABILIZE` to Pixhawk
4. RPi5 **stops all autonomous commands**
5. Pilot has full manual control via TX

### Returning to autonomous:
1. Power OFF the TX
2. After 3 consecutive OFF readings (300ms), RPi5 returns to IDLE
3. Send a new `start_mission` command from the laptop to restart

> **Note:** The drone does NOT resume its previous mission. It always
> returns to IDLE and waits for a fresh start command. This is a
> safety design decision.

---

## 7. Emergency Procedure

### What triggers an emergency:
| Condition             | Timeout | Description                        |
|-----------------------|---------|------------------------------------|
| TF-02 sensor failure  | 2.0s    | No valid reading from LiDAR        |
| EKF health failure    | 1.0s    | Position/velocity flags unhealthy  |
| Wi-Fi link loss       | 3.0s    | No UDP packet from laptop          |
| Geofence breach       | instant | Position > 10m from home           |
| abort_mission command | instant | Manual abort from laptop           |

### What RPi5 does:
1. Sets flight mode to **LAND** (vertical descent only)
2. Logs the emergency reason
3. Monitors touchdown conditions
4. Returns to IDLE after confirmed landing

### Why LAND instead of RTL:
RTL requires a valid position estimate to navigate home. If the emergency
was triggered by an EKF failure or sensor fault, that estimate is corrupt.
LAND descends vertically using only barometer + TF-02, which is always safe.

### Hardware-level backup (no code needed):
If the RPi5 crashes entirely, the Pixhawk `FS_GCS_ENABLE` parameter
detects the lost companion heartbeat after 3 seconds and forces a landing
autonomously at the firmware level.

---

## 8. Troubleshooting

| Symptom                               | Cause                              | Fix                                     |
|----------------------------------------|------------------------------------|-----------------------------------------|
| `No heartbeat` on start               | Pixhawk not powered / USB loose    | Check USB cable, power cycle             |
| Stuck in IDLE, no response to start    | Laptop not sending to correct IP   | Verify RPi5 IP and CMD_PORT (14551)     |
| `Gate 1 FAIL: battery`                | Battery below 14.4V               | Charge battery before flight             |
| `Gate 2 FAIL: TF-02`                  | TF-02 not connected / UART off    | Check wiring, run `raspi-config`         |
| `Gate 3 FAIL: EKF`                    | GPS lock not acquired              | Wait for GPS lock (outdoors)             |
| Arming timeout                         | Pre-arm check failing              | Check Mission Planner HUD messages       |
| TX override activates unexpectedly     | TX powered on or RF interference   | Power off TX, move away from RF sources  |
| Wi-Fi link loss in flight              | RPi5 lost Wi-Fi connection         | Improve Wi-Fi signal, reduce distance    |
| Emergency LAND during hover            | Battery dropped below 14.0V       | Use full battery, reduce hover duration  |
| Geofence breach                        | Wind drift in LOITER mode          | Increase FENCE_X_M / FENCE_Y_M in Config|

---

## 9. File Structure

```
ascend/
├── __init__.py           Package init
├── __main__.py           Entry: python3 -m ascend.scheduler
├── config.py             All constants — no magic numbers
├── logger.py             ANSI colour-coded timestamped logger
├── drone_state.py        Thread-safe shared state + State enum
├── hardware/
│   ├── __init__.py
│   ├── tf02.py           TF-02 LiDAR frame parser (daemon)
│   └── pixhawk.py        pymavlink wrapper (thread-safe)
├── threads/
│   ├── __init__.py
│   ├── bridge.py         RangefinderBridge (20 Hz)
│   ├── heartbeat.py      HeartbeatSender (1 Hz)
│   ├── tx_override.py    TXOverrideWatcher (debounced)
│   ├── safety.py         SafetyMonitor (4 checks)
│   ├── telemetry.py      TelemetryStreamer (1 Hz UDP)
│   └── commands.py       CommandReceiver (UDP server)
├── state_machine.py      StateMachine (7 state handlers)
├── scheduler.py          Scheduler + CLI entry point
└── README_phase2.md      This file
```

---

## 10. Thread Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     RPi5 (Python)                            │
│                                                              │
│  ┌────────────┐  ┌─────────────┐   ┌────────────────┐        │
│  │ TF02Reader │  │ Rangefinder │   │ Heartbeat      │        │
│  │ (100 Hz)   ├──┤ Bridge      │   │ Sender (1 Hz)  │        │
│  └────────────┘  │ (20 Hz)     │   └────────┬───────┘        │
│                  └──────┬──────┘            │                │
│                         │                   │                │
│  ┌──────────────────────┴───────────────────┴──────────────┐ │
│  │                PixhawkLink (MAVLink)                    │ │
│  └──────────────────────┬──────────────────────────────────┘ │
│                         │                                    │
│  ┌────────────┐  ┌──────┴──────┐   ┌─────────────────────┐   │
│  │ TX Override│  │ State       │   │ Telemetry Streamer  │   │
│  │ Watcher    │  │ Machine     │   │ (1 Hz → laptop)     │   │
│  │ (100 ms)   │  │ (main loop) │   └─────────┬───────────┘   │
│  └────────────┘  └──────┬──────┘             │ UDP:14550     │
│                         │                    │               │
│  ┌────────────┐  ┌──────┴──────┐             ▼               │
│  │ Safety     │  │ Command     │     ┌──────────────┐        │
│  │ Monitor    │  │ Receiver    │     │   Laptop     │        │
│  │ (10 Hz)    │  │ (UDP:14551) │◄────┤  (hotspot)   │        │
│  └────────────┘  └─────────────┘     └──────────────┘        │
│                                                              │
└───────────────────────────┬──────────────────────────────────┘
                            │ USB
                   ┌────────┴─────────┐
                   │  Pixhawk 2.4.8   │
                   │  (ArduCopter)    │
                   └──────────────────┘
```
