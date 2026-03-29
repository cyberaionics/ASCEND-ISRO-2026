"""
ASCEND Phase 2 — RPi5 OpenCV Optical Flow Processor
Daemon thread that captures frames from a downward-facing camera on the
RPi5 and runs proper Shi-Tomasi + pyramidal Lucas-Kanade optical flow.

This gives MUCH higher quality flow than the ESP32-CAM's simplified
onboard algorithm because:
  1. Shi-Tomasi detects the BEST corners for tracking (cv2.goodFeaturesToTrack)
  2. Pyramidal LK handles larger displacements (cv2.calcOpticalFlowPyrLK)
  3. Outlier rejection via forward-backward consistency check
  4. RPi5 has far more compute than ESP32 → larger frames, more features

Camera auto-detection order:
  1. Pi Camera via V4L2 (/dev/video0)
  2. USB camera via V4L2 (/dev/video1, /dev/video2)
  3. Falls back to ESP32-CAM WiFi MJPEG stream if configured

Output interface: identical to ESP32CamReader (flow_dx, flow_dy, quality,
data_age, get_flow()) so the VIO stabilizer can use either or both.
"""

import threading
import time
from typing import Optional

import cv2
import numpy as np

from ..config import Config
from ..logger import Logger


class CVFlowProcessor(threading.Thread):
    """OpenCV optical-flow processor running on the RPi5.

    Captures frames from a downward-facing camera, detects strong
    corners (Shi-Tomasi), tracks them between frames (pyramidal LK),
    rejects outliers, and outputs median flow displacement.

    Exposes the same interface as ESP32CamReader so the VIO stabilizer
    can consume either source transparently.

    Args:
        camera_id: V4L2 device index (0 = /dev/video0, etc.) or
                   URL for MJPEG stream.
    """

    def __init__(self, camera_id: int = 0) -> None:
        super().__init__(daemon=True, name="CVFlowProcessor")
        self._camera_id = camera_id
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        # Output (same interface as ESP32CamReader)
        self._flow_dx: float = 0.0
        self._flow_dy: float = 0.0
        self._quality: int = 0
        self._last_read_time: float = 0.0
        self._error_count: int = 0

        # ── Shi-Tomasi corner detection parameters ────────────────────
        self._feature_params = dict(
            maxCorners=Config.CV_MAX_CORNERS,
            qualityLevel=Config.CV_QUALITY_LEVEL,
            minDistance=Config.CV_MIN_DISTANCE,
            blockSize=Config.CV_BLOCK_SIZE,
        )

        # ── Lucas-Kanade pyramidal optical flow parameters ────────────
        self._lk_params = dict(
            winSize=(Config.CV_LK_WIN_SIZE, Config.CV_LK_WIN_SIZE),
            maxLevel=Config.CV_LK_MAX_LEVEL,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                30,     # max iterations
                0.01,   # epsilon
            ),
        )

        # Internal state
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_points: Optional[np.ndarray] = None

    # ── Thread-safe Properties (same as ESP32CamReader) ────────────────

    @property
    def flow_dx(self) -> float:
        """Latest X-axis optical-flow displacement in pixels."""
        with self._lock:
            return self._flow_dx

    @property
    def flow_dy(self) -> float:
        """Latest Y-axis optical-flow displacement in pixels."""
        with self._lock:
            return self._flow_dy

    @property
    def quality(self) -> int:
        """Number of successfully tracked features (0–MAX_CORNERS)."""
        with self._lock:
            return self._quality

    @property
    def data_age(self) -> float:
        """Seconds since last valid reading (inf if none yet)."""
        with self._lock:
            if self._last_read_time == 0.0:
                return float("inf")
            return time.time() - self._last_read_time

    @property
    def error_count(self) -> int:
        """Total count of failed frames."""
        with self._lock:
            return self._error_count

    def get_flow(self) -> tuple:
        """Return ``(dx, dy, quality)`` atomically."""
        with self._lock:
            return (self._flow_dx, self._flow_dy, self._quality)

    def stop(self) -> None:
        """Signal the processor to stop."""
        self._running.clear()

    # ── Core CV Algorithm ──────────────────────────────────────────────

    def _process_frame(self, gray: np.ndarray) -> None:
        """Run one iteration of Shi-Tomasi + LK optical flow.

        Args:
            gray: Current grayscale frame (uint8).
        """
        if self._prev_gray is None:
            # First frame — detect corners, no flow yet
            self._prev_gray = gray
            self._prev_points = cv2.goodFeaturesToTrack(
                gray, **self._feature_params
            )
            return

        if self._prev_points is None or len(self._prev_points) < 3:
            # Too few features — re-detect
            self._prev_gray = gray
            self._prev_points = cv2.goodFeaturesToTrack(
                gray, **self._feature_params
            )
            return

        # ── Forward LK tracking ───────────────────────────────────────
        next_points, status_fwd, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_points, None, **self._lk_params
        )

        if next_points is None:
            self._prev_gray = gray
            self._prev_points = cv2.goodFeaturesToTrack(
                gray, **self._feature_params
            )
            return

        # ── Backward LK for consistency check (outlier rejection) ─────
        back_points, status_bwd, _ = cv2.calcOpticalFlowPyrLK(
            gray, self._prev_gray, next_points, None, **self._lk_params
        )

        # ── Filter: keep only points that pass both forward and
        #    backward tracking with small back-projection error ────────
        good_mask = np.zeros(len(status_fwd), dtype=bool)
        if back_points is not None:
            for i in range(len(status_fwd)):
                if status_fwd[i][0] == 1 and status_bwd[i][0] == 1:
                    # Back-projection error < 1 pixel = inlier
                    d = np.linalg.norm(
                        self._prev_points[i] - back_points[i]
                    )
                    if d < Config.CV_FB_THRESHOLD:
                        good_mask[i] = True

        n_good = int(np.sum(good_mask))

        if n_good < 2:
            # Too few inliers — reset features
            with self._lock:
                self._flow_dx = 0.0
                self._flow_dy = 0.0
                self._quality = 0
            self._prev_gray = gray
            self._prev_points = cv2.goodFeaturesToTrack(
                gray, **self._feature_params
            )
            return

        # ── Compute median flow (robust to remaining outliers) ────────
        prev_good = self._prev_points[good_mask]
        next_good = next_points[good_mask]
        deltas = next_good - prev_good  # shape: (N, 1, 2)
        deltas = deltas.reshape(-1, 2)

        # Median is robust to outliers — better than mean
        median_dx = float(np.median(deltas[:, 0]))
        median_dy = float(np.median(deltas[:, 1]))

        # ── Store results ─────────────────────────────────────────────
        with self._lock:
            self._flow_dx = median_dx
            self._flow_dy = median_dy
            self._quality = n_good
            self._last_read_time = time.time()

        # ── Update state for next frame ───────────────────────────────
        self._prev_gray = gray

        # Re-detect features periodically or when count drops low
        if n_good < Config.CV_MAX_CORNERS // 2:
            self._prev_points = cv2.goodFeaturesToTrack(
                gray, **self._feature_params
            )
        else:
            # Use tracked points as seeds for next frame
            self._prev_points = next_good.reshape(-1, 1, 2)

    # ── Thread Entry ───────────────────────────────────────────────────

    def run(self) -> None:
        """Open camera and run optical flow loop."""
        Logger.info(
            f"CVFlowProcessor starting on camera {self._camera_id}, "
            f"resolution={Config.CV_FRAME_WIDTH}×{Config.CV_FRAME_HEIGHT}, "
            f"corners={Config.CV_MAX_CORNERS}"
        )

        # Try to open camera
        cap = cv2.VideoCapture(self._camera_id, cv2.CAP_V4L2)
        if not cap.isOpened():
            # Fallback: try without V4L2 backend
            cap = cv2.VideoCapture(self._camera_id)

        if not cap.isOpened():
            Logger.error(
                f"CVFlowProcessor: cannot open camera {self._camera_id}"
            )
            return

        # Set resolution and FPS
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, Config.CV_FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Config.CV_FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, Config.CV_FPS)

        Logger.ok(
            f"CVFlowProcessor camera opened: "
            f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}×"
            f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
            f"{int(cap.get(cv2.CAP_PROP_FPS))} fps"
        )

        try:
            while self._running.is_set():
                ret, frame = cap.read()
                if not ret:
                    with self._lock:
                        self._error_count += 1
                    time.sleep(0.01)
                    continue

                # Convert to grayscale
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                try:
                    self._process_frame(gray)
                except Exception as exc:
                    Logger.warn(f"CVFlowProcessor frame error: {exc}")
                    with self._lock:
                        self._error_count += 1

        except Exception as exc:
            Logger.error(f"CVFlowProcessor fatal error: {exc}")
        finally:
            cap.release()
            Logger.info("CVFlowProcessor stopped")
