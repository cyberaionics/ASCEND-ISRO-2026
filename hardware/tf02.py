"""
ASCEND Phase 2 — TF-02 LiDAR Reader
Daemon thread that continuously parses 9-byte Benewake TF-02 frames
from the RPi5 hardware UART.
"""

import threading
import time
from typing import Optional

import serial  # pyserial

from ..config import Config
from ..logger import Logger


class TF02Reader(threading.Thread):
    """Continuous TF-02 LiDAR frame reader running in a daemon thread.

    The TF-02 outputs 9-byte frames at 100 Hz on ``/dev/serial0``.
    This thread validates checksums, discards bad frames, and exposes
    the latest reading through thread-safe properties.

    Frame format (9 bytes):
        [0]=0x59  [1]=0x59  — header
        [2]=dist_low  [3]=dist_high  — distance in cm
        [4]=str_low   [5]=str_high   — signal strength
        [6],[7]=reserved
        [8]=checksum  — (sum of bytes 0–7) & 0xFF
    """

    def __init__(self) -> None:
        super().__init__(daemon=True, name="TF02Reader")
        self._lock = threading.Lock()
        self._distance_cm: Optional[int] = None
        self._strength: int = 0
        self._error_count: int = 0
        self._last_read_time: float = 0.0
        self._running = threading.Event()
        self._running.set()

    # ── Thread-safe Properties ─────────────────────────────────────────

    @property
    def distance_cm(self) -> Optional[int]:
        """Latest valid distance in centimetres, or ``None``."""
        with self._lock:
            return self._distance_cm

    @property
    def distance_m(self) -> Optional[float]:
        """Latest valid distance in metres, or ``None``."""
        with self._lock:
            if self._distance_cm is not None:
                return self._distance_cm / 100.0
            return None

    @property
    def strength(self) -> int:
        """Latest signal-strength value."""
        with self._lock:
            return self._strength

    @property
    def error_count(self) -> int:
        """Total invalid / discarded frames."""
        with self._lock:
            return self._error_count

    @property
    def last_read_time(self) -> float:
        """``time.time()`` of the most recent valid frame."""
        with self._lock:
            return self._last_read_time

    @property
    def data_age(self) -> float:
        """Seconds since the last valid reading (inf if none yet)."""
        with self._lock:
            if self._last_read_time == 0.0:
                return float("inf")
            return time.time() - self._last_read_time

    # ── Public API ─────────────────────────────────────────────────────

    def read_single(self, timeout: float = 2.0) -> Optional[int]:
        """Block until one new valid frame is parsed, or timeout.

        Args:
            timeout: Maximum wait in seconds.

        Returns:
            Distance in centimetres, or ``None`` on timeout.
        """
        deadline = time.time() + timeout
        snapshot = self.distance_cm
        while time.time() < deadline:
            current = self.distance_cm
            if current is not None and current != snapshot:
                return current
            time.sleep(0.005)
        return self.distance_cm

    def stop(self) -> None:
        """Signal the reader thread to stop."""
        self._running.clear()

    # ── Frame Parsing ──────────────────────────────────────────────────

    @staticmethod
    def _valid(frame: bytes) -> bool:
        """Return True if the 9-byte frame passes header + checksum."""
        if len(frame) != Config.TF02_FRAME_LEN:
            return False
        if frame[0] != Config.TF02_HEADER or frame[1] != Config.TF02_HEADER:
            return False
        return (sum(frame[:8]) & 0xFF) == frame[8]

    def _parse(self, frame: bytes) -> None:
        """Extract distance and strength from a validated frame."""
        dist = frame[2] | (frame[3] << 8)
        strength = frame[4] | (frame[5] << 8)
        with self._lock:
            self._distance_cm = dist
            self._strength = strength
            self._last_read_time = time.time()

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        """Open the UART port and read TF-02 frames continuously."""
        Logger.info(f"TF02Reader starting on {Config.TF02_PORT}")
        try:
            ser = serial.Serial(
                port=Config.TF02_PORT,
                baudrate=Config.TF02_BAUD,
                timeout=0.1,
            )
        except serial.SerialException as exc:
            Logger.error(f"TF02Reader: cannot open port: {exc}")
            return

        try:
            while self._running.is_set():
                # Synchronise to double-0x59 header
                b1 = ser.read(1)
                if not b1 or b1[0] != Config.TF02_HEADER:
                    continue
                b2 = ser.read(1)
                if not b2 or b2[0] != Config.TF02_HEADER:
                    continue
                rest = ser.read(7)
                if len(rest) != 7:
                    with self._lock:
                        self._error_count += 1
                    continue
                frame = bytes([Config.TF02_HEADER, Config.TF02_HEADER]) + rest
                if self._valid(frame):
                    self._parse(frame)
                else:
                    with self._lock:
                        self._error_count += 1
        except serial.SerialException as exc:
            Logger.error(f"TF02Reader serial error: {exc}")
        finally:
            ser.close()
            Logger.info("TF02Reader stopped")
