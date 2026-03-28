"""
ASCEND Phase 2 — Heartbeat Sender
Daemon thread: companion heartbeat to Pixhawk every 1.0 s.
"""

import threading
import time

from ..config import Config
from ..logger import Logger
from ..hardware.pixhawk import PixhawkLink


class HeartbeatSender(threading.Thread):
    """Sends a companion-computer MAVLink heartbeat at 1 Hz.

    Must run during ALL flight phases to prevent the Pixhawk
    ``FS_GCS_ENABLE`` failsafe from triggering a forced landing.

    Args:
        pixhawk: Shared PixhawkLink instance.
    """

    def __init__(self, pixhawk: PixhawkLink) -> None:
        super().__init__(daemon=True, name="HeartbeatSender")
        self._px = pixhawk
        self._running = threading.Event()
        self._running.set()

    def stop(self) -> None:
        self._running.clear()

    def run(self) -> None:
        Logger.info("HeartbeatSender running at 1 Hz")
        while self._running.is_set():
            try:
                self._px.send_heartbeat()
            except Exception as exc:
                Logger.warn(f"Heartbeat error: {exc}")
            time.sleep(Config.HEARTBEAT_INTERVAL)
        Logger.info("HeartbeatSender stopped")
