"""
ASCEND — ESP32-CAM Raw Frame Reader + ORB Optical Flow Processor
Two classes for the high-bandwidth UART2 channel from the ESP32-CAM:

1. ESP32FrameReader — daemon thread on ttyAMA3 @ 921600 that receives
   raw 160×120 grayscale frames and exposes the latest frame as a
   numpy array via a thread-safe property.

2. ORBFlowProcessor — daemon thread that consumes frames from
   ESP32FrameReader, runs ORB keypoint detection + homography-based
   optical flow, and exposes (flow_dx, flow_dy, quality) with the
   same interface as CVFlowProcessor / ESP32CamReader.

Frame protocol from ESP32 .ino (UART2, GPIO16/17):
    Byte 0:         0xAA  (sync 1)
    Byte 1:         0x55  (sync 2)
    Bytes 2–19201:  pixel data (160×120 = 19200 bytes, row-major grayscale)
    Byte 19202:     XOR checksum of all 19200 pixel bytes

    Total: 19203 bytes per frame → ~4.8 fps at 921600 baud
"""

import threading
import time
from typing import Optional

import cv2
import numpy as np
import serial  # pyserial

from ..config import Config
from ..logger import Logger


# ═══════════════════════════════════════════════════════════════════════════
# ESP32 Frame Reader — raw UART frame capture
# ═══════════════════════════════════════════════════════════════════════════

class ESP32FrameReader(threading.Thread):
    """Daemon thread that captures raw grayscale frames from ESP32-CAM UART2.

    Synchronises to the 0xAA/0x55 header pair, reads 19200 pixel bytes,
    validates the XOR checksum, and stores the latest frame as a numpy
    array accessible via a thread-safe property.

    Attributes:
        latest_frame:  Latest valid frame as numpy ndarray (120, 160), uint8.
        frame_count:   Total number of valid frames received.
        error_count:   Total number of invalid/dropped frames.
        data_age:      Seconds since the last valid frame.
    """

    _FRAME_SIZE: int = Config.ESP32_FRAME_PIXEL_COUNT  # 19200
    _SYNC_1: int = Config.ESP32_FRAME_SYNC_1           # 0xAA
    _SYNC_2: int = Config.ESP32_FRAME_SYNC_2           # 0x55

    def __init__(self) -> None:
        super().__init__(daemon=True, name="ESP32FrameReader")
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        self._latest_frame: Optional[np.ndarray] = None
        self._frame_count: int = 0
        self._error_count: int = 0
        self._last_read_time: float = 0.0

    # ── Thread-safe Properties ─────────────────────────────────────────

    @property
    def latest_frame(self) -> Optional[np.ndarray]:
        """Latest valid frame as (120, 160) uint8 ndarray, or None."""
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
            return None

    @property
    def frame_count(self) -> int:
        """Total valid frames received."""
        with self._lock:
            return self._frame_count

    @property
    def error_count(self) -> int:
        """Total invalid/dropped frames."""
        with self._lock:
            return self._error_count

    @property
    def last_read_time(self) -> float:
        """time.time() of the last valid frame."""
        with self._lock:
            return self._last_read_time

    @property
    def data_age(self) -> float:
        """Seconds since the last valid frame (inf if none yet)."""
        with self._lock:
            if self._last_read_time == 0.0:
                return float("inf")
            return time.time() - self._last_read_time

    # ── Public API ─────────────────────────────────────────────────────

    def stop(self) -> None:
        """Signal the reader thread to stop."""
        self._running.clear()

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        """Open the UART port and continuously read raw frames."""
        Logger.info(
            f"ESP32FrameReader starting on {Config.ESP32_FRAME_PORT} "
            f"@ {Config.ESP32_FRAME_BAUD}"
        )
        try:
            ser = serial.Serial(
                port=Config.ESP32_FRAME_PORT,
                baudrate=Config.ESP32_FRAME_BAUD,
                timeout=0.5,
            )
        except serial.SerialException as exc:
            Logger.error(
                f"ESP32FrameReader: cannot open {Config.ESP32_FRAME_PORT}: {exc}"
            )
            return

        try:
            while self._running.is_set():
                # Synchronise to 0xAA 0x55 header pair
                byte = ser.read(1)
                if not byte or byte[0] != self._SYNC_1:
                    continue
                byte2 = ser.read(1)
                if not byte2 or byte2[0] != self._SYNC_2:
                    continue

                # Read pixel data + checksum byte
                payload = ser.read(self._FRAME_SIZE + 1)
                if len(payload) != self._FRAME_SIZE + 1:
                    with self._lock:
                        self._error_count += 1
                    continue

                pixel_data = payload[:self._FRAME_SIZE]
                received_checksum = payload[self._FRAME_SIZE]

                # Validate XOR checksum of all pixel bytes
                computed = 0
                for b in pixel_data:
                    computed ^= b

                if computed != received_checksum:
                    with self._lock:
                        self._error_count += 1
                    continue

                # Store as numpy array (120 rows × 160 cols)
                frame = np.frombuffer(pixel_data, dtype=np.uint8).reshape(
                    Config.ESP32_FRAME_HEIGHT, Config.ESP32_FRAME_WIDTH
                )

                with self._lock:
                    self._latest_frame = frame
                    self._frame_count += 1
                    self._last_read_time = time.time()

        except serial.SerialException as exc:
            Logger.error(f"ESP32FrameReader serial error: {exc}")
        finally:
            ser.close()
            Logger.info("ESP32FrameReader stopped")


# ═══════════════════════════════════════════════════════════════════════════
# ORB Flow Processor — ORB keypoints + homography optical flow
# ═══════════════════════════════════════════════════════════════════════════

class ORBFlowProcessor(threading.Thread):
    """Daemon thread that runs ORB feature detection + homography-based
    optical flow on frames from ESP32FrameReader.

    Implements the same public interface as CVFlowProcessor and
    ESP32CamReader so VIOStabilizer can consume it transparently.

    Algorithm:
        1. Detect ORB keypoints + descriptors in current and previous frame
        2. Match descriptors with BFMatcher (Hamming distance)
        3. Compute homography via RANSAC from matched points
        4. Extract translation (dx, dy) from homography matrix H
        5. Quality = number of RANSAC inliers

    Args:
        frame_reader: Running ESP32FrameReader instance.
    """

    def __init__(self, frame_reader: ESP32FrameReader) -> None:
        super().__init__(daemon=True, name="ORBFlowProcessor")
        self._reader = frame_reader
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        # Output (same interface as ESP32CamReader / CVFlowProcessor)
        self._flow_dx: float = 0.0
        self._flow_dy: float = 0.0
        self._quality: int = 0
        self._last_read_time: float = 0.0
        self._error_count: int = 0

        # Internal state
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_kp = None
        self._prev_des = None

        # ORB detector (created once, reused)
        self._orb = cv2.ORB_create(nfeatures=Config.ORB_N_FEATURES)
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # ── Thread-safe Properties ─────────────────────────────────────────

    @property
    def flow_dx(self) -> float:
        """Latest X-axis displacement in pixels (actual, not ×100)."""
        with self._lock:
            return self._flow_dx

    @property
    def flow_dy(self) -> float:
        """Latest Y-axis displacement in pixels (actual, not ×100)."""
        with self._lock:
            return self._flow_dy

    @property
    def quality(self) -> int:
        """Number of RANSAC inliers from homography estimation."""
        with self._lock:
            return self._quality

    @property
    def last_update(self) -> float:
        """time.time() of the last valid flow computation."""
        with self._lock:
            return self._last_read_time

    @property
    def data_age(self) -> float:
        """Seconds since the last valid flow computation (inf if none)."""
        with self._lock:
            if self._last_read_time == 0.0:
                return float("inf")
            return time.time() - self._last_read_time

    @property
    def error_count(self) -> int:
        """Total failed flow computations."""
        with self._lock:
            return self._error_count

    def get_flow(self) -> tuple:
        """Return ``(dx, dy, quality)`` atomically."""
        with self._lock:
            return (self._flow_dx, self._flow_dy, self._quality)

    # ── Public API ─────────────────────────────────────────────────────

    def stop(self) -> None:
        """Signal the processor to stop."""
        self._running.clear()

    # ── Core ORB Algorithm ─────────────────────────────────────────────

    def _process_frame(self, gray: np.ndarray) -> None:
        """Run one iteration of ORB keypoint + homography flow.

        Args:
            gray: Current grayscale frame (uint8, 120×160).
        """
        # Detect ORB keypoints and descriptors
        kp, des = self._orb.detectAndCompute(gray, None)

        if self._prev_gray is None or self._prev_des is None:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_des = des
            return

        if des is None or len(kp) < Config.ORB_MIN_MATCHES:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_des = des
            with self._lock:
                self._error_count += 1
            return

        # Match descriptors
        try:
            matches = self._bf.match(self._prev_des, des)
        except cv2.error:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_des = des
            with self._lock:
                self._error_count += 1
            return

        if len(matches) < Config.ORB_MIN_MATCHES:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_des = des
            return

        # Extract matched point coordinates
        src_pts = np.float32(
            [self._prev_kp[m.queryIdx].pt for m in matches]
        ).reshape(-1, 1, 2)
        dst_pts = np.float32(
            [kp[m.trainIdx].pt for m in matches]
        ).reshape(-1, 1, 2)

        # Compute homography with RANSAC
        h_mat, mask = cv2.findHomography(
            src_pts, dst_pts, cv2.RANSAC, Config.ORB_RANSAC_THRESH
        )

        if h_mat is None or mask is None:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_des = des
            with self._lock:
                self._error_count += 1
            return

        n_inliers = int(mask.sum())

        if n_inliers < Config.ORB_MIN_MATCHES:
            self._prev_gray = gray
            self._prev_kp = kp
            self._prev_des = des
            return

        # Extract translation from homography
        # For downward-facing camera over planar ground:
        # H[0,2] = x translation, H[1,2] = y translation (in pixels)
        dx = float(h_mat[0, 2])
        dy = float(h_mat[1, 2])

        with self._lock:
            self._flow_dx = dx
            self._flow_dy = dy
            self._quality = n_inliers
            self._last_read_time = time.time()

        # Update state for next frame
        self._prev_gray = gray
        self._prev_kp = kp
        self._prev_des = des

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        """Consume frames from ESP32FrameReader and compute ORB flow."""
        Logger.info(
            f"ORBFlowProcessor starting — "
            f"features={Config.ORB_N_FEATURES}, "
            f"RANSAC_thresh={Config.ORB_RANSAC_THRESH}"
        )

        last_frame_count = 0

        while self._running.is_set():
            # Only process new frames (skip if no new frame available)
            current_count = self._reader.frame_count

            if current_count == last_frame_count:
                time.sleep(0.01)  # 100 Hz poll, no busy-wait
                continue

            last_frame_count = current_count
            frame = self._reader.latest_frame

            if frame is None:
                time.sleep(0.01)
                continue

            try:
                self._process_frame(frame)
            except Exception as exc:
                Logger.warn(f"ORBFlowProcessor frame error: {exc}")
                with self._lock:
                    self._error_count += 1

        Logger.info("ORBFlowProcessor stopped")
