# ASCEND — Changelog

## Qualification Round Rewrite (2026-03-30)

### Overview

Complete rewrite of 5 core files for the IRoC-U 2026 qualification round.
Corrected ESP32-CAM protocol to match actual .ino firmware, added raw frame
capture + ORB optical flow pipeline, implemented dual-pipeline VIO with
independent PID controllers and weighted fusion, and rebuilt the state machine
with explicit ARM/DISARM/DONE states.

### What Changed

---

#### config.py — REWRITTEN

**Corrected values** (were wrong in previous version):

| Constant | Old Value | New Value | Reason |
|----------|-----------|-----------|--------|
| `PIXHAWK_BAUD` | 921600 | **115200** | Matches hardware spec |
| `TF02_PORT` | `/dev/serial0` | **`/dev/ttyAMA0`** | Explicit UART0 path |
| `ESP32_CAM_BAUD` | 921600 | **115200** | Matches .ino firmware |
| `ESP32_CAM_HEADER_1` | 0xAA | **0xAB** | Matches .ino firmware |
| `ESP32_CAM_HEADER_2` | 0x55 | **0xCD** | Matches .ino firmware |
| `VIO_FOCAL_LENGTH_PX` | 60.0 | **100.0** | Corrected for QQVGA 160×120 (was 96×96) |
| `VIO_MIN_ALT_M` | 0.0 | **0.3** | Safety: VIO off below 30 cm |
| `CRITICAL_BATTERY_VOLT` | 9.0 | **13.2** | Correct for 4S LiPo (3.3V/cell) |
| `HOVER_DURATION` | 60 | **300** | 5 minutes for Task 2 |

**New constants added**:

- **Dual-pipeline PID**: `ESP_VIO_KP=0.4`, `ESP_VIO_KI=0.05`, `ESP_VIO_KD=0.02`, `ESP_VIO_WEIGHT=0.4`
- **ORB pipeline PID**: `ORB_VIO_KP=0.5`, `ORB_VIO_KI=0.08`, `ORB_VIO_KD=0.03`, `ORB_VIO_WEIGHT=0.6`
- **ESP32 frame reader**: `ESP32_FRAME_PORT=/dev/ttyAMA3`, `ESP32_FRAME_BAUD=921600`, sync bytes, pixel count
- **ORB detector**: `ORB_N_FEATURES=200`, `ORB_RANSAC_THRESH=5.0`, `ORB_MIN_MATCHES=8`
- **Flight params**: `HOVER_KP_ALT=200.0`, `BASE_THROTTLE_PWM=1550`, `MIN_THROTTLE_PWM=1100`, `MAX_THROTTLE_PWM=1800`, `TAKEOFF_TIMEOUT_S=15`, `TAKEOFF_ALT_THRESHOLD=0.90`, `LAND_DURATION_S=5`, `ARM_TIMEOUT_S=10`, `DISARM_TIMEOUT_S=5`
- **Aliases**: `BATT_LOW_VOLT`, `BATT_CRITICAL_VOLT`, `HOVER_DURATION_S`, `HOVER_TARGET_ALT_M`, `TELEMETRY_HOST`, `HEARTBEAT_HZ`

> All existing constants used by unchanged modules (tf02.py, pixhawk.py, bridge.py, cv_flow.py, checks/) are preserved.

---

#### hardware/esp32_cam.py — REWRITTEN

**Protocol corrections** (3 bugs fixed from previous version):

1. **Headers**: `0xAA/0x55` → `0xAB/0xCD` — matches actual .ino firmware
2. **Byte order**: big-endian `>h` → **little-endian `<h`** — .ino sends LE
3. **Checksum**: `sum(bytes 0–6) & 0xFF` → **XOR of bytes 2,3,4,5,6** — matches .ino

**Interface changes**:
- Added `.last_update` property (time.time() of last valid packet)
- Documented that flow values are `actual_pixels × 100` (consumers divide by 100)
- No other interface changes — backward compatible

---

#### hardware/esp32_cam_frame.py — NEW FILE

Two classes for the high-bandwidth UART2 channel from ESP32-CAM:

**`ESP32FrameReader`** — daemon thread on `/dev/ttyAMA3` @ 921600 baud
- Captures raw 160×120 grayscale frames (19200 bytes per frame)
- Frame protocol: `[0xAA][0x55][19200 pixel bytes][XOR checksum]` = 19203 bytes
- Throughput: ~4.8 fps at 921600 baud
- Exposes `.latest_frame` as numpy ndarray (120, 160), uint8
- Thread-safe via `threading.Lock`

**`ORBFlowProcessor`** — daemon thread consuming ESP32FrameReader frames
- ORB keypoint detection + BFMatcher (Hamming) + RANSAC homography
- Extracts translation (dx, dy) from homography matrix H[0,2], H[1,2]
- Quality = number of RANSAC inliers
- Same interface as `CVFlowProcessor` (`.flow_dx`, `.flow_dy`, `.quality`, `.get_flow()`, `.data_age`)
- Drop-in replacement for `CVFlowProcessor` in VIOStabilizer

---

#### threads/vio_stabilizer.py — REWRITTEN

**Architecture change**: single cascaded P-controller → **dual independent PID controllers with weighted fusion**.

Previous architecture:
```
Single flow source → EMA → position integration → outer P-loop → inner PID → correction
```

New architecture:
```
ESP32 flow → physics → PID (Kp=0.4, Ki=0.05, Kd=0.02) → weight 0.4 ─┐
                                                                        ├→ fused correction
ORB flow   → physics → PID (Kp=0.5, Ki=0.08, Kd=0.03) → weight 0.6 ─┘
```

**Key behaviors**:
- Each pipeline runs an independent PID on velocity error (target = 0)
- Fusion: renormalized weighted sum when both active
- Graceful degradation: single pipeline at full weight when other drops
- I-term reset on per-pipeline gate failure (quality, freshness, altitude)
- ESP32 flow divided by 100.0 (values are actual_pixels × 100)
- ORB flow used directly (actual pixel values)

**Interface changes**:
- `.roll_pwm` / `.pitch_pwm` — full PWM values (1500 ± correction), ready for RC override
- `.is_active` — True if at least one pipeline active
- `.get_corrections()` → `(roll_pwm, pitch_pwm, is_active)` — convenience method
- `.reset()` — resets both pipeline I-terms and velocity state
- Removed: `.roll_correction`, `.pitch_correction` (use `roll_pwm - 1500` if needed)
- Removed: `.get_position()`, `.reset_position()`, `.source` (position integration removed)

---

#### scheduler.py — REWRITTEN

**State machine changes**:

Previous states (7):
```
IDLE → PREFLIGHT → TAKEOFF → HOVER → RETURN → LAND → EMERGENCY
```

New states (8):
```
IDLE → PREFLIGHT → ARM → TAKEOFF → HOVER → LAND → DISARM → DONE
```

| Change | Detail |
|--------|--------|
| Added `ARM` state | Explicit arming with 10s timeout, failure → DISARM |
| Added `DISARM` state | Explicit disarm with 5s timeout and force-disarm fallback |
| Added `DONE` state | Mission summary logging (flight time, alt range, VIO %) |
| Removed `RETURN` state | Simplified: HOVER goes directly to LAND |
| Removed `EMERGENCY` state | Safety triggers direct transition to LAND |

**TAKEOFF changes**:
- Throttle ramp: +5 PWM/tick at 20 Hz (reaches hover in ~5s)
- Altitude P-controller: `throttle = BASE(1550) + Kp(200) × error`
- Transition at `TAKEOFF_ALT_THRESHOLD` (0.90 m)
- Timeout: 15s → LAND

**HOVER changes**:
- Altitude P-controller: `Kp_alt = 200 PWM/m`, clamped [1100, 1800]
- VIO corrections from dual-pipeline VIOStabilizer
- Duration: `HOVER_DURATION_S` (300s = 5 min for Task 2)
- 1 Hz console logging with altitude, error, throttle, VIO

**LAND changes**:
- Linear throttle reduction from BASE to MIN over 5 seconds
- Touchdown: alt < 0.08 m or throttle at minimum → DISARM

**Scheduler changes**:
- Wires ESP32CamReader + ESP32FrameReader + ORBFlowProcessor → VIOStabilizer
- 3s sensor stabilisation wait before starting state machine
- Removed `CVFlowProcessor` from fly mode (replaced by ORBFlowProcessor)
- Graceful fallback: if frame reader/ORB fails, VIO runs ESP32-only

---

### Safety Guarantees

1. **Graceful degradation** — If either VIO pipeline fails, the other continues at full weight. If both fail, corrections are zero (neutral 1500).

2. **Conservative limits** — Maximum correction ±100 PWM (1400–1600 range).

3. **Multiple safety gates** — Quality threshold, data freshness timeout, minimum altitude — evaluated independently per pipeline.

4. **I-term reset** — Integrator terms are cleared when a pipeline's gates fail, preventing windup during dropout periods.

5. **Single writer guarantee** — Only StateMachine writes to PixhawkLink for flight commands. HeartbeatSender and RangefinderBridge use the existing send_lock.

6. **Existing modules unchanged** — `tf02.py`, `pixhawk.py`, `bridge.py`, `cv_flow.py`, all `checks/` files retain their original interfaces.

---

## Phase 2 — VIO Hover Stabilization (2026-03-29)

### Overview

Added Visual Inertial Odometry (VIO) based hover stabilization using an
ESP32-CAM running on-board Lucas-Kanade optical flow. The system now holds
position in the X-Y plane during hover (and takeoff/climb) in addition to
the existing Z-axis altitude hold via TF-02 LiDAR.

### What Changed

---

#### New Files

| File | Purpose |
|------|---------|
| `hardware/esp32_cam.py` | Daemon thread that reads 8-byte optical-flow packets from the ESP32-CAM over UART (`/dev/ttyAMA2`). Validates checksums, exposes `flow_dx`, `flow_dy`, `quality` as thread-safe properties. Mirrors the `TF02Reader` architecture. |
| `threads/vio_stabilizer.py` | Daemon thread that converts raw optical-flow pixels into roll/pitch PWM corrections. Uses altitude-scaled velocity estimation and a P-controller with multiple safety gates (quality, staleness, altitude, clamping). |
| `Changes.md` | This file — documents all modifications. |
| `requirements.txt` | Complete Python dependency list for the entire ASCEND codebase. |

---

#### Modified Files

##### `config.py`

**Added** two new configuration sections (no existing values were changed):

- **ESP32-CAM constants**: `ESP32_CAM_PORT` (`/dev/ttyAMA2`), `ESP32_CAM_BAUD` (115200), header bytes (`0xAA`, `0x55`), frame length (8).
- **VIO stabilization constants**: `VIO_RATE_HZ` (20), `VIO_KP` (0.5), `VIO_DEADZONE_PX` (2), `VIO_MIN_QUALITY` (5), `VIO_MAX_CORRECTION_PWM` (100), `VIO_FOCAL_LENGTH_PX` (60.0), `VIO_DATA_TIMEOUT` (0.5s), `VIO_MIN_ALT_M` (0.3m).

---

##### `scheduler.py`

**Imports added:**
- `from .hardware.esp32_cam import ESP32CamReader`
- `from .threads.vio_stabilizer import VIOStabilizer`

**`StateMachine` class changes:**

1. **Constructor** now accepts an optional `vio: VIOStabilizer` parameter.
2. **New method `_get_vio_roll_pitch()`** — returns VIO-corrected roll/pitch PWM.
3. **`_state_takeoff()`** and **`_state_hover()`** — RC override now includes VIO corrections.
4. **`get_telemetry()`** — Added VIO telemetry fields.

**`Scheduler` class changes:**

1. **New `_start_esp32_cam()`** — starts ESP32-CAM reader thread.
2. **`_run_fly()`** — starts ESP32CamReader and VIOStabilizer before state machine.
