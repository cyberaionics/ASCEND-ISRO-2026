"""
ASCEND Phase 2 — ESP32-CAM Optical Flow Reader
Daemon thread that continuously parses 8-byte optical-flow packets
from the ESP32-CAM over UART (/dev/ttyAMA2).

The ESP32-CAM runs a Lucas-Kanade optical-flow algorithm on 96×96
grayscale frames at 30 Hz and transmits compact 8-byte packets:

    Byte 0:    0xAA         — sync header byte 1
    Byte 1:    0x55         — sync header byte 2
    Byte 2-3:  dx (int16)   — X displacement in pixels (big-endian)
    Byte 4-5:  dy (int16)   — Y displacement in pixels (big-endian)
    Byte 6:    quality      — tracked feature count (0–255)
    Byte 7:    checksum     — (sum of bytes 0–6) & 0xFF
"""

import struct
import threading
import time
from typing import Optional

import serial  # pyserial

from ..config import Config
from ..logger import Logger


class ESP32CamReader(threading.Thread):
    """Continuous ESP32-CAM optical-flow reader running in a daemon thread.

    Validates checksums, discards bad frames, and exposes the latest
    optical-flow readings through thread-safe properties.

    Attributes:
        flow_dx:     Latest X displacement in pixels (positive = rightward).
        flow_dy:     Latest Y displacement in pixels (positive = forward).
        quality:     Tracked feature count from the current frame (0–255).
        data_age:    Seconds since the last valid reading.
        error_count: Running count of discarded / invalid frames.
    """

    def __init__(self) -> None:
        super().__init__(daemon=True, name="ESP32CamReader")
        self._lock = threading.Lock()
        self._flow_dx: int = 0
        self._flow_dy: int = 0
        self._quality: int = 0
        self._error_count: int = 0
        self._last_read_time: float = 0.0
        self._running = threading.Event()
        self._running.set()

    # ── Thread-safe Properties ─────────────────────────────────────────

    @property
    def flow_dx(self) -> int:
        """Latest X-axis optical-flow displacement in pixels."""
        with self._lock:
            return self._flow_dx

    @property
    def flow_dy(self) -> int:
        """Latest Y-axis optical-flow displacement in pixels."""
        with self._lock:
            return self._flow_dy

    @property
    def quality(self) -> int:
        """Tracked feature count from the latest frame (0–255)."""
        with self._lock:
            return self._quality

    @property
    def error_count(self) -> int:
        """Total count of discarded / invalid frames."""
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

    def get_flow(self) -> tuple:
        """Return ``(dx, dy, quality)`` atomically.

        Returns:
            Tuple of (dx_pixels, dy_pixels, quality_score).
        """
        with self._lock:
            return (self._flow_dx, self._flow_dy, self._quality)

    def stop(self) -> None:
        """Signal the reader thread to stop."""
        self._running.clear()

    # ── Frame Validation ───────────────────────────────────────────────

    @staticmethod
    def _validate_frame(frame: bytes) -> bool:
        """Return ``True`` if the 8-byte frame has valid headers and checksum.

        Args:
            frame: Exactly 8 bytes starting with the header pair.

        Returns:
            ``True`` if headers match and checksum is correct.
        """
        if len(frame) != Config.ESP32_CAM_FRAME_LEN:
            return False
        if frame[0] != Config.ESP32_CAM_HEADER_1 or frame[1] != Config.ESP32_CAM_HEADER_2:
            return False
        checksum = sum(frame[:7]) & 0xFF
        return checksum == frame[7]

    def _parse_frame(self, frame: bytes) -> None:
        """Extract dx, dy, and quality from a validated frame.

        Args:
            frame: Validated 8-byte optical-flow packet.
        """
        # Bytes 2-3: dx as signed int16 big-endian
        # Bytes 4-5: dy as signed int16 big-endian
        dx = struct.unpack(">h", frame[2:4])[0]
        dy = struct.unpack(">h", frame[4:6])[0]
        quality = frame[6]

        with self._lock:
            self._flow_dx = dx
            self._flow_dy = dy
            self._quality = quality
            self._last_read_time = time.time()

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        """Open the UART port and continuously read ESP32-CAM flow packets."""
        Logger.info(
            f"ESP32CamReader starting on {Config.ESP32_CAM_PORT} "
            f"@ {Config.ESP32_CAM_BAUD}"
        )
        try:
            ser = serial.Serial(
                port=Config.ESP32_CAM_PORT,
                baudrate=Config.ESP32_CAM_BAUD,
                timeout=0.1,
            )
        except serial.SerialException as exc:
            Logger.error(
                f"ESP32CamReader: cannot open {Config.ESP32_CAM_PORT}: {exc}"
            )
            return

        try:
            while self._running.is_set():
                # Synchronise to 0xAA 0x55 header pair
                byte = ser.read(1)
                if not byte or byte[0] != Config.ESP32_CAM_HEADER_1:
                    continue
                byte2 = ser.read(1)
                if not byte2 or byte2[0] != Config.ESP32_CAM_HEADER_2:
                    continue

                # Read remaining 6 bytes (dx_hi, dx_lo, dy_hi, dy_lo, quality, checksum)
                rest = ser.read(6)
                if len(rest) != 6:
                    with self._lock:
                        self._error_count += 1
                    continue

                frame = bytes([Config.ESP32_CAM_HEADER_1, Config.ESP32_CAM_HEADER_2]) + rest

                if self._validate_frame(frame):
                    self._parse_frame(frame)
                else:
                    with self._lock:
                        self._error_count += 1
        except serial.SerialException as exc:
            Logger.error(f"ESP32CamReader serial error: {exc}")
        finally:
            ser.close()
            Logger.info("ESP32CamReader stopped")
