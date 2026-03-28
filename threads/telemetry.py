"""
ASCEND Phase 2 — Telemetry Streamer
Daemon thread: streams JSON telemetry to laptop over UDP at 1 Hz.
"""

import datetime
import json
import socket
import threading
import time
from typing import Optional

from ..config import Config
from ..logger import Logger
from ..drone_state import DroneState


class TelemetryStreamer(threading.Thread):
    """Streams live telemetry as JSON over UDP to the laptop at 1 Hz.

    Reads a snapshot from ``DroneState``, adds a timestamp, and sends
    the packet to ``Config.LAPTOP_IP:Config.TELEMETRY_PORT``.
    Silently handles UDP errors (laptop may not always be reachable).

    Args:
        drone_state: Shared DroneState instance.
    """

    def __init__(self, drone_state: DroneState) -> None:
        super().__init__(daemon=True, name="TelemetryStreamer")
        self._ds = drone_state
        self._running = threading.Event()
        self._running.set()
        self._sock: Optional[socket.socket] = None

    def stop(self) -> None:
        self._running.clear()

    def run(self) -> None:
        Logger.info(
            f"TelemetryStreamer → {Config.LAPTOP_IP}:{Config.TELEMETRY_PORT}"
        )
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except OSError as exc:
            Logger.error(f"TelemetryStreamer socket error: {exc}")
            return

        dest = (Config.LAPTOP_IP, Config.TELEMETRY_PORT)
        while self._running.is_set():
            try:
                snap = self._ds.snapshot()
                snap["timestamp"] = datetime.datetime.now().isoformat()
                packet = json.dumps(snap).encode("utf-8")
                self._sock.sendto(packet, dest)
            except Exception:
                pass  # laptop may be unavailable — silently continue
            time.sleep(Config.TELEMETRY_INTERVAL)

        self._sock.close()
        Logger.info("TelemetryStreamer stopped")
