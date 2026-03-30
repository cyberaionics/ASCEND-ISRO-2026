"""
ASCEND — Scheduler & State Machine
Entry point for all CLI modes.  Run as: python3 -m ascend --mode <mode>

GUIDED_NOGPS Architecture:
  1. TF02Reader         → lidar altitude at 100 Hz
  2. WiFiFrameReader    → MJPEG frames from ESP32-CAM
  3. WiFiLKProcessor    → optical flow from frames
  4. WiFiORBProcessor   → ORB flow (optional, for redundancy)
  5. SensorFusionBridge → fuses flow + lidar → ODOMETRY at 20 Hz → EKF3
  6. FlightSafetyMonitor → monitors sensor latency, triggers BRAKE
  7. MissionController  → state machine: WAIT_FOR_EKF→ARM→TAKEOFF→HOVER_60S→LAND
  8. HeartbeatSender    → companion heartbeat at 1 Hz
  9. RangefinderBridge   → TF-02 → Pixhawk DISTANCE_SENSOR at 20 Hz
"""

import argparse
import enum
import math
import signal
import sys
import threading
import time
from typing import Optional

from pymavlink import mavutil

from .config import Config
from .logger import Logger
from .hardware.tf02 import TF02Reader
from .hardware.pixhawk import PixhawkLink
from .hardware.wifi_flow import WiFiFrameReader, WiFiLKProcessor, WiFiORBProcessor, WhiteDetector
from .threads.bridge import (
    HeartbeatSender,
    RangefinderBridge,
    SafetyMonitor,
    SensorFusionBridge,
    TelemetryStreamer,
)
from .threads.vio_stabilizer import MissionController
from .checks.health import HealthChecker
from .checks.setup import AutoTuneSetup
from .checks.monitor import AutoTuneMonitor, FlightSafetyMonitor


# ═══════════════════════════════════════════════════════════════════════════
# Legacy State Enum (kept for backward compat, no longer used in fly mode)
# ═══════════════════════════════════════════════════════════════════════════

class State(enum.Enum):
    """Flight state machine states (legacy — qualification round)."""
    IDLE      = "IDLE"
    PREFLIGHT = "PREFLIGHT"
    ARM       = "ARM"
    TAKEOFF   = "TAKEOFF"
    HOVER     = "HOVER"
    LAND      = "LAND"
    DISARM    = "DISARM"
    DONE      = "DONE"


# ═══════════════════════════════════════════════════════════════════════════
# Scheduler
# ═══════════════════════════════════════════════════════════════════════════

class Scheduler:
    """Top-level launcher.  Creates hardware objects, starts threads, runs SM.

    The 'fly' mode has been refactored to use the new GUIDED_NOGPS
    architecture:  SensorFusionBridge → ODOMETRY → EKF3 + MissionController.
    """

    def __init__(self, mode: str) -> None:
        self._mode    = mode
        self._threads = []
        self._pixhawk: Optional[PixhawkLink] = None
        self._tf02:    Optional[TF02Reader]   = None

        # WiFi camera pipeline
        self._wifi_frame:   Optional[WiFiFrameReader]   = None
        self._wifi_lk:      Optional[WiFiLKProcessor]   = None
        self._wifi_orb:     Optional[WiFiORBProcessor]  = None

        signal.signal(signal.SIGTERM, self._shutdown_handler)

    def run(self) -> int:
        try:
            return {
                "check":    self._run_check,
                "setup":    self._run_setup,
                "all":      self._run_all,
                "autotune": self._run_autotune,
                "fly":      self._run_fly,
            }[self._mode]()
        except Exception as exc:
            Logger.error(f"Scheduler fatal: {exc}")
            return 1
        finally:
            self._cleanup()

    # ── Mode Runners ───────────────────────────────────────────────────

    def _run_check(self) -> int:
        if not self._connect_pixhawk():
            return 1
        checker = HealthChecker(self._pixhawk)
        return 0 if checker.run_all() else 1

    def _run_setup(self) -> int:
        if not self._connect_pixhawk():
            return 1
        setup = AutoTuneSetup(self._pixhawk)
        return 0 if setup.run_all() else 1

    def _run_all(self) -> int:
        if not self._connect_pixhawk():
            return 1
        checker = HealthChecker(self._pixhawk)
        if not checker.run_all():
            Logger.error("Health check failed — skipping setup")
            return 1
        setup = AutoTuneSetup(self._pixhawk)
        return 0 if setup.run_all() else 1

    def _run_autotune(self) -> int:
        if not self._connect_pixhawk():
            return 1
        self._start_tf02()
        self._start_bridge()
        self._start_heartbeat()
        monitor = AutoTuneMonitor(self._pixhawk)
        monitor.run()
        return 0

    def _run_fly(self) -> int:
        """Start all threads and run the GUIDED_NOGPS flight controller.

        Thread startup order:
          1. Pixhawk connection (921600 baud)
          2. TF02Reader (lidar altitude)
          3. WiFi camera pipeline (frame reader → LK + ORB flow)
          4. Sensor stabilisation wait
          5. SensorFusionBridge (flow + lidar → ODOMETRY → EKF3)
          6. FlightSafetyMonitor (sensor latency watchdog)
          7. HeartbeatSender + RangefinderBridge
          8. MissionController state machine (blocking)
        """
        # ── 1. Connect Pixhawk ────────────────────────────────────────
        if not self._connect_pixhawk():
            return 1
        Logger.ok(f"Pixhawk connected at {Config.PIXHAWK_BAUD} baud")

        # ── 2. Start TF-02 LiDAR ─────────────────────────────────────
        self._start_tf02()

        # ── 3. Start WiFi camera pipeline ─────────────────────────────
        self._start_wifi_cam()

        # ── 4. Sensor stabilisation ───────────────────────────────────
        Logger.info("Waiting 5s for sensor + WiFi stream stabilisation …")
        time.sleep(5.0)
        self._verify_sensors()

        # ── 5. Sensor Fusion Bridge (EKF_INIT + ODOMETRY at 20 Hz) ────
        fusion_bridge = SensorFusionBridge(
            lk_flow=self._wifi_lk,
            tf02=self._tf02,
            pixhawk=self._pixhawk,
        )
        fusion_bridge.start()
        self._threads.append(fusion_bridge)
        Logger.ok("SensorFusionBridge started — ODOMETRY + EKF_INIT active")

        # Allow EKF to start processing ODOMETRY
        Logger.info("Waiting 3s for EKF3 to begin processing ODOMETRY …")
        time.sleep(3.0)

        # ── 6. Flight Safety Monitor ──────────────────────────────────
        safety = FlightSafetyMonitor(
            pixhawk=self._pixhawk,
            tf02=self._tf02,
            fusion_bridge=fusion_bridge,
        )
        safety.start()
        self._threads.append(safety)
        Logger.ok(
            f"FlightSafetyMonitor started — latency limit: "
            f"{Config.STREAM_LATENCY_MAX_S * 1000:.0f}ms"
        )

        # ── 7. HeartbeatSender + RangefinderBridge + TelemetryStreamer ────────────────────
        self._start_heartbeat()
        self._start_bridge()
        self._start_telemetry_streamer()

        # ── 8. MissionController (blocking state machine) ─────────────
        Logger.header("LAUNCHING MISSION CONTROLLER")
        mission = MissionController(
            pixhawk=self._pixhawk,
            tf02=self._tf02,
            safety_flag=safety.get_emergency,
        )

        # Activate safety monitoring now that we're about to fly
        safety.set_flight_active(True)

        mission.run()  # blocks until DONE

        # Post-flight
        safety.set_flight_active(False)
        Logger.ok("Flight complete — shutting down")
        return 0

    # ── Sensor Verification ────────────────────────────────────────────

    def _verify_sensors(self) -> None:
        """Log sensor status after stabilisation delay."""
        Logger.info("Verifying sensor data streams …")

        # TF-02
        if self._tf02 and self._tf02.is_alive():
            tf02_m = self._tf02.distance_m
            if tf02_m is not None:
                Logger.ok(f"TF-02: alive, reading {tf02_m:.2f} m")
            else:
                Logger.warn("TF-02: thread alive but NO DATA — check /dev/serial0")
        else:
            Logger.error("TF-02: thread NOT alive — check wiring")

        # WiFi frame reader
        if self._wifi_frame and self._wifi_frame.is_alive():
            age = self._wifi_frame.data_age
            if age < 1.0:
                Logger.ok(
                    f"WiFi camera: alive, {self._wifi_frame.frame_count} frames, "
                    f"age={age:.2f}s"
                )
            else:
                Logger.warn(
                    f"WiFi camera: thread alive but NO FRAMES (age={age:.1f}s). "
                    f"Check ESP32-CAM power and WiFi network."
                )
        else:
            Logger.error("WiFi camera: thread NOT alive")

        # LK flow
        if self._wifi_lk and self._wifi_lk.is_alive():
            age = self._wifi_lk.data_age
            if age < 1.0:
                dx, dy, q = self._wifi_lk.get_flow()
                Logger.ok(
                    f"LK flow: alive, quality={q}, "
                    f"flow=({dx:.1f}, {dy:.1f}), age={age:.2f}s"
                )
            else:
                Logger.warn(f"LK flow: thread alive but stale (age={age:.1f}s)")
        else:
            Logger.warn("LK flow: not running — fusion will be impaired")

        # ORB flow (optional)
        if self._wifi_orb and self._wifi_orb.is_alive():
            age = self._wifi_orb.data_age
            if age < 2.0:
                Logger.ok(
                    f"ORB flow: alive, quality={self._wifi_orb.quality}, "
                    f"age={age:.2f}s"
                )
            else:
                Logger.warn(f"ORB flow: thread alive but stale (age={age:.1f}s)")
        else:
            Logger.info("ORB flow: not running (optional — LK is primary)")

    # ── Thread Starters ────────────────────────────────────────────────

    def _connect_pixhawk(self) -> bool:
        self._pixhawk = PixhawkLink()
        return self._pixhawk.connect()

    def _start_tf02(self) -> None:
        self._tf02 = TF02Reader()
        self._tf02.start()
        self._threads.append(self._tf02)

    def _start_bridge(self) -> None:
        bridge = RangefinderBridge(self._tf02, self._pixhawk)
        bridge.start()
        self._threads.append(bridge)

    def _start_telemetry_streamer(self) -> None:
        import queue
        q = queue.Queue(maxsize=200)
        if self._pixhawk:
            self._pixhawk.set_telemetry_queue(q)
        streamer = TelemetryStreamer(message_queue=q)
        streamer.start()
        self._threads.append(streamer)

    def _start_heartbeat(self) -> None:
        hb = HeartbeatSender(self._pixhawk)
        hb.start()
        self._threads.append(hb)

    def _start_wifi_cam(self) -> None:
        """Start WiFi frame reader + LK + ORB processors.

        WiFiFrameReader auto-discovers the ESP32-CAM on each connect
        attempt (tries candidate IPs, then scans subnets).  All
        processors share a single HTTP connection.
        """
        Logger.info("Starting WiFi camera pipeline — auto-discovery enabled")
        try:
            # One connection — auto-discovers ESP32-CAM IP
            self._wifi_frame = WiFiFrameReader()  # no URL = auto-discover
            self._wifi_frame.start()
            self._threads.append(self._wifi_frame)
            Logger.ok("WiFiFrameReader started")

            # LK optical flow (primary — used by SensorFusionBridge)
            self._wifi_lk = WiFiLKProcessor(self._wifi_frame)
            self._wifi_lk.start()
            self._threads.append(self._wifi_lk)
            Logger.ok("WiFiLKProcessor started (primary flow source)")

            # ORB optical flow (redundancy — not consumed by fusion bridge)
            self._wifi_orb = WiFiORBProcessor(self._wifi_frame)
            self._wifi_orb.start()
            self._threads.append(self._wifi_orb)
            Logger.ok("WiFiORBProcessor started (redundant)")

        except Exception as exc:
            Logger.error(f"WiFi camera pipeline failed: {exc}")
            Logger.warn("Sensor fusion will be impaired — ODOMETRY will lack flow data")
            self._wifi_frame = None
            self._wifi_lk    = None
            self._wifi_orb   = None

    # ── Cleanup ────────────────────────────────────────────────────────

    def _cleanup(self) -> None:
        Logger.info("Cleaning up …")
        if self._pixhawk and self._pixhawk.connected:
            try:
                self._pixhawk.set_mode("LAND")
                time.sleep(0.3)
                self._pixhawk.disarm()
                Logger.warn("Safety LAND + disarm sent during cleanup")
            except Exception:
                pass
        for t in self._threads:
            if hasattr(t, "stop"):
                t.stop()
        for t in self._threads:
            t.join(timeout=2.0)
        if self._pixhawk:
            self._pixhawk.close()
        Logger.ok("Shutdown complete")

    def _shutdown_handler(self, signum: int, frame: object) -> None:
        if self._pixhawk and self._pixhawk.connected:
            try:
                self._pixhawk.set_mode("LAND")
                self._pixhawk.disarm()
            except Exception:
                pass
        raise KeyboardInterrupt


def main() -> None:
    """CLI entry point: python3 -m ascend --mode <mode>"""
    parser = argparse.ArgumentParser(
        prog="ascend",
        description="ASCEND — Autonomous drone flight system (IRoC-U 2026)",
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=["check", "setup", "all", "autotune", "fly"],
        help="Operating mode: check | setup | all | autotune | fly",
    )
    args = parser.parse_args()
    scheduler = Scheduler(mode=args.mode)
    sys.exit(scheduler.run())


if __name__ == "__main__":
    main()
