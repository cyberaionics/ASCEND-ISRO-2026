# ASCEND — Changelog

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

> **No PID coefficients, flight parameters, or any other existing config values were modified.**

---

##### `scheduler.py`

**Imports added:**
- `from .hardware.esp32_cam import ESP32CamReader`
- `from .threads.vio_stabilizer import VIOStabilizer`

**`StateMachine` class changes:**

1. **Constructor** now accepts an optional `vio: VIOStabilizer` parameter (backward-compatible — defaults to `None`).

2. **New method `_get_vio_roll_pitch()`** — returns `(roll_pwm, pitch_pwm)` with VIO corrections applied, or `(1500, 1500)` if VIO is unavailable. Clamped to [1000, 2000] range.

3. **`_state_takeoff()`** — All `send_rc_override()` calls now pass VIO-corrected `roll` and `pitch` values instead of the previous implicit defaults of 1500. This enables X-Y drift correction during the climb phase.

4. **`_state_hover()`** — Same as takeoff: `send_rc_override()` calls now include VIO-corrected roll/pitch. Log line enriched with VIO correction values when active.

5. **`get_telemetry()`** — Telemetry dict now includes `vio_active`, `vio_roll_corr`, and `vio_pitch_corr` fields for live monitoring.

**`Scheduler` class changes:**

1. **New `_esp32_cam` attribute** and **`_start_esp32_cam()`** method — starts the ESP32-CAM reader daemon thread.

2. **`_run_fly()`** — Now starts `ESP32CamReader` and `VIOStabilizer` threads before the state machine. VIO is passed to `StateMachine` constructor. Both threads are included in the cleanup list (automatic shutdown).

> **The VIO stabilization is always active during `--mode fly`** — no extra command or flag is needed.

---

##### `README.md`

- Added ESP32-CAM to the hardware components table.
- Added ESP32-CAM wiring to the wiring diagram.
- Updated architecture diagram to show the VIO pipeline.
- Added VIO parameter reference table.
- Updated file structure listing with new files.
- Added ESP32-CAM troubleshooting entries.

---

### How It Works

```
ESP32-CAM (96×96 OV2640)
    │  Lucas-Kanade optical flow @ 30 Hz
    │  8-byte UART packets (dx, dy, quality)
    ▼
ESP32CamReader (/dev/ttyAMA2)
    │  Daemon thread, validates checksums
    │  Exposes flow_dx, flow_dy, quality
    ▼
VIOStabilizer (20 Hz loop)
    │  1. Quality gate (min 5 tracked features)
    │  2. Data freshness gate (< 500 ms)
    │  3. Altitude gate (> 30 cm via TF-02)
    │  4. Deadzone filter (±2 px)
    │  5. Velocity = (flow × altitude) / (focal_len × dt)
    │  6. P-controller: correction = -Kp × velocity
    │  7. Clamp to ±100 PWM
    ▼
StateMachine (TAKEOFF / HOVER)
    │  roll  = 1500 + roll_correction
    │  pitch = 1500 + pitch_correction
    │  throttle = existing altitude P-controller
    ▼
send_rc_override(roll, pitch, throttle)
    │
    ▼
Pixhawk (STABILIZE mode)
```

### Safety Guarantees

1. **Graceful degradation** — If the ESP32-CAM fails, corrections default to zero (neutral 1500). The drone reverts to altitude-hold-only behavior, same as Phase 1.

2. **Conservative limits** — Maximum correction is ±100 PWM (1400–1600 range), preventing any aggressive roll/pitch maneuvers.

3. **Multiple safety gates** — Quality threshold, data timeout, altitude minimum, and deadzone filtering all must pass before any correction is applied.

4. **No PID changes** — Zero modification to any existing PID coefficients, flight parameters, or thresholds.

5. **Existing behavior preserved** — All modes except `fly` are completely unchanged. Emergency, landing, and RC override behavior remain identical to Phase 1.
