"""
ASCEND Phase 2 — Rangefinder Bridge
Daemon thread: TF-02 → Pixhawk DISTANCE_SENSOR at 20 Hz.
"""

import threading
import time

from ..config import Config
from ..logger import Logger
from ..hardware.tf02 import TF02Reader
from ..hardware.pixhawk import PixhawkLink


class RangefinderBridge(threading.Thread):
    """Forwards TF-02 distance readings to the Pixhawk EKF at 20 Hz.

    Args:
        tf02: Shared TF02Reader instance.
        pixhawk: Shared PixhawkLink instance.
    """

    def __init__(self, tf02: TF02Reader, pixhawk: PixhawkLink) -> None:
        super().__init__(daemon=True, name="RangefinderBridge")
        self._tf02 = tf02
        self._px = pixhawk
        self._running = threading.Event()
        self._running.set()

    def stop(self) -> None:
        """Signal the thread to stop."""
        self._running.clear()

    def run(self) -> None:
        Logger.info(f"RangefinderBridge running at {Config.BRIDGE_HZ} Hz")
        while self._running.is_set():
            dist = self._tf02.distance_cm
            if dist is not None:
                try:
                    self._px.send_distance_sensor(dist)
                except Exception as exc:
                    Logger.warn(f"Bridge send error: {exc}")
            time.sleep(Config.BRIDGE_INTERVAL)
        Logger.info("RangefinderBridge stopped")
