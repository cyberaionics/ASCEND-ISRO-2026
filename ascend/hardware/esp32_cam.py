"""
ASCEND — ESP32-CAM Optical Flow Reader
Daemon thread that continuously parses 8-byte optical-flow packets
from the ESP32-CAM over UART (ttyAMA2 @ 115200).

The ESP32-CAM runs block-matching SAD on a 6×5 grid of 16×16 blocks,
±8 pixel search window, on 160×120 grayscale frames at 30 Hz and
transmits compact 8-byte packets:

    Byte 0:    0xAB         — sync header byte 1
    Byte 1:    0xCD         — sync header byte 2
    Byte 2:    flow_x low   — int16 little-endian (actual_pixels × 100)
    Byte 3:    flow_x high
    Byte 4:    flow_y low   — int16 little-endian (actual_pixels × 100)
    Byte 5:    flow_y high
    Byte 6:    quality      — uint8, valid SAD blocks count (0–30)
    Byte 7:    checksum     — XOR of bytes 2, 3, 4, 5, 6
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

    Validates XOR checksums, discards bad frames, and exposes the latest
    optical-flow readings through thread-safe properties.

    Flow values are stored as raw int16 from the packet (actual_pixels × 100).
    Consumers must divide by 100.0 to get actual pixel displacement.

    Attributes:
        flow_dx:     Latest X displacement (int16, ×100 of actual pixels).
        flow_dy:     Latest Y displacement (int16, ×100 of actual pixels).
        quality:     Number of valid SAD blocks from the current frame (0–30).
        last_update: time.time() of the most recent valid packet.
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
        """Latest X-axis flow (int16, actual_pixels × 100)."""
        with self._lock:
            return self._flow_dx

    @property
    def flow_dy(self) -> int:
        """Latest Y-axis flow (int16, actual_pixels × 100)."""
        with self._lock:
            return self._flow_dy

    @property
    def quality(self) -> int:
        """Valid SAD block count from the latest frame (0–30)."""
        with self._lock:
            return self._quality

    @property
    def last_update(self) -> float:
        """``time.time()`` of the most recent valid packet."""
        with self._lock:
            return self._last_read_time

    @property
    def last_read_time(self) -> float:
        """Alias for last_update (backward compat)."""
        with self._lock:
            return self._last_read_time

    @property
    def data_age(self) -> float:
        """Seconds since the last valid reading (inf if none yet)."""
        with self._lock:
            if self._last_read_time == 0.0:
                return float("inf")
            return time.time() - self._last_read_time

    @property
    def error_count(self) -> int:
        """Total count of discarded / invalid frames."""
        with self._lock:
            return self._error_count

    # ── Public API ─────────────────────────────────────────────────────

    def get_flow(self) -> tuple:
        """Return ``(dx, dy, quality)`` atomically.

        Returns:
            Tuple of (dx_raw, dy_raw, quality). dx/dy are ×100.
        """
        with self._lock:
            return (self._flow_dx, self._flow_dy, self._quality)

    def stop(self) -> None:
        """Signal the reader thread to stop."""
        self._running.clear()

    # ── Frame Validation ───────────────────────────────────────────────

    @staticmethod
    def _validate_frame(frame: bytes) -> bool:
        """Return ``True`` if the 8-byte frame has valid headers and XOR checksum.

        Checksum = XOR of bytes 2, 3, 4, 5, 6. Must equal byte 7.

        Args:
            frame: Exactly 8 bytes starting with the header pair.

        Returns:
            ``True`` if headers match and checksum is correct.
        """
        if len(frame) != Config.ESP32_CAM_FRAME_LEN:
            return False
        if frame[0] != Config.ESP32_CAM_HEADER_1 or frame[1] != Config.ESP32_CAM_HEADER_2:
            return False
        checksum = frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ frame[6]
        return checksum == frame[7]

    def _parse_frame(self, frame: bytes) -> None:
        """Extract dx, dy, and quality from a validated frame.

        Bytes 2-3: dx as signed int16 little-endian (actual_pixels × 100)
        Bytes 4-5: dy as signed int16 little-endian (actual_pixels × 100)
        Byte 6:    quality (uint8, 0–30)

        Args:
            frame: Validated 8-byte optical-flow packet.
        """
        dx = struct.unpack("<h", frame[2:4])[0]
        dy = struct.unpack("<h", frame[4:6])[0]
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
                # Synchronise to 0xAB 0xCD header pair
                byte = ser.read(1)
                if not byte or byte[0] != Config.ESP32_CAM_HEADER_1:
                    continue
                byte2 = ser.read(1)
                if not byte2 or byte2[0] != Config.ESP32_CAM_HEADER_2:
                    continue

                # Read remaining 6 bytes (dx_lo, dx_hi, dy_lo, dy_hi, quality, checksum)
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
