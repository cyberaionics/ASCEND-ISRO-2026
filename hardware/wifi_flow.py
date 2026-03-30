"""
ASCEND — WiFi Camera Flow Processors
======================================
Three classes for optical flow from ESP32-CAM over WiFi MJPEG:

  WiFiFrameReader   — One thread opens the MJPEG HTTP stream and stores
                      the latest frame in a shared buffer.  Only ONE
                      connection to the ESP32-CAM is ever opened.

  WiFiLKProcessor   — Reads from WiFiFrameReader, runs Shi-Tomasi
                      corner detection + pyramidal LK optical flow.
                      Drop-in replacement for ESP32CamReader (UART).
                      NOTE: flow_dx / flow_dy are ACTUAL pixels (not ×100).

  WiFiORBProcessor  — Reads from WiFiFrameReader, runs ORB keypoint
                      detection + BFMatcher + RANSAC homography flow.
                      Drop-in replacement for ORBFlowProcessor.

Both flow processors expose the identical public interface:
    .flow_dx   (float, pixels)
    .flow_dy   (float, pixels)
    .quality   (int)
    .data_age  (float, seconds)
    .get_flow() → (dx, dy, quality)

so VIOStabilizer can consume them without any other changes
(except removing the ×100 scaling that was specific to the old UART protocol).
"""

import threading
import time
from typing import Optional

import cv2
import numpy as np

from ..config import Config
from ..logger import Logger


# ═══════════════════════════════════════════════════════════════════════════
#  Shared Frame Buffer — single MJPEG connection, multiple readers
# ═══════════════════════════════════════════════════════════════════════════

class WiFiFrameReader(threading.Thread):
    """Daemon thread that opens the ESP32-CAM MJPEG stream over WiFi
    and continuously stores the latest frame in a shared buffer.

    Only one instance should be created so only one HTTP connection
    is maintained to the ESP32-CAM (which has limited RAM / TCP sockets).

    Consumers (WiFiLKProcessor, WiFiORBProcessor) call ``get_frame()``
    to retrieve the latest frame without opening their own connections.

    Args:
        url: Full MJPEG stream URL, e.g. ``http://10.42.0.200:81/stream``.
    """

    def __init__(self, url: str = "") -> None:
        super().__init__(daemon=True, name="WiFiFrameReader")
        self._url = url or Config.ESP32_WIFI_STREAM_URL
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        self._latest_frame: Optional[np.ndarray] = None
        self._frame_count: int = 0
        self._error_count: int = 0
        self._last_read_time: float = 0.0

    # ── Thread-safe Properties ─────────────────────────────────────

    @property
    def frame_count(self) -> int:
        """Total valid frames received since start."""
        with self._lock:
            return self._frame_count

    @property
    def error_count(self) -> int:
        """Total failed/dropped frames."""
        with self._lock:
            return self._error_count

    @property
    def data_age(self) -> float:
        """Seconds since the last valid frame (inf if none yet)."""
        with self._lock:
            if self._last_read_time == 0.0:
                return float("inf")
            return time.time() - self._last_read_time

    def get_frame(self) -> Optional[np.ndarray]:
        """Return a copy of the latest valid frame, or ``None``."""
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy()
            return None

    def stop(self) -> None:
        """Signal the reader thread to stop."""
        self._running.clear()

    # ── Thread Entry ───────────────────────────────────────────────

    def run(self) -> None:
        """Open the MJPEG stream and fill the shared frame buffer."""
        Logger.info(f"WiFiFrameReader starting — URL: {self._url}")

        reconnect_delay = 2.0

        while self._running.is_set():
            Logger.info(f"WiFiFrameReader connecting to {self._url} …")
            cap = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)

            if not cap.isOpened():
                Logger.warn(
                    f"WiFiFrameReader: cannot open stream {self._url}. "
                    f"Retry in {reconnect_delay:.0f}s. "
                    "Check ESP32-CAM power and WiFi connection."
                )
                time.sleep(reconnect_delay)
                reconnect_delay = min(reconnect_delay * 1.5, 15.0)
                continue

            Logger.ok("WiFiFrameReader: stream connected")
            reconnect_delay = 2.0  # reset backoff on success

            try:
                while self._running.is_set():
                    ret, frame = cap.read()
                    if not ret:
                        with self._lock:
                            self._error_count += 1
                        Logger.warn("WiFiFrameReader: frame read failed — reconnecting")
                        break  # outer loop will reconnect

                    # Convert BGR to greyscale — all flow algorithms use grey
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                    with self._lock:
                        self._latest_frame = gray
                        self._frame_count += 1
                        self._last_read_time = time.time()

            except Exception as exc:
                Logger.error(f"WiFiFrameReader exception: {exc}")
            finally:
                cap.release()
                if self._running.is_set():
                    Logger.info(
                        f"WiFiFrameReader: reconnecting in {reconnect_delay:.0f}s"
                    )
                    time.sleep(reconnect_delay)

        Logger.info("WiFiFrameReader stopped")


# ═══════════════════════════════════════════════════════════════════════════
#  Shi-Tomasi + Pyramidal Lucas-Kanade Flow  (replaces ESP32CamReader)
# ═══════════════════════════════════════════════════════════════════════════

class WiFiLKProcessor(threading.Thread):
    """Shi-Tomasi + pyramidal LK optical flow from the WiFi MJPEG stream.

    Reads frames from a shared ``WiFiFrameReader``, detects corners with
    Shi-Tomasi (``cv2.goodFeaturesToTrack``), tracks them between frames
    with pyramidal LK (``cv2.calcOpticalFlowPyrLK``), and rejects outliers
    with a forward-backward consistency check.  Outputs the median
    displacement of inlier tracks.

    This is a *direct drop-in replacement* for ``ESP32CamReader`` except:
      - flow_dx / flow_dy are actual pixels (not ×100 int16).
      - No UART, no baud rate, no UART wiring required.

    Args:
        frame_reader: A running ``WiFiFrameReader`` instance.
    """

    def __init__(self, frame_reader: WiFiFrameReader) -> None:
        super().__init__(daemon=True, name="WiFiLKProcessor")
        self._reader = frame_reader
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        # Output — same interface as ESP32CamReader
        self._flow_dx: float = 0.0
        self._flow_dy: float = 0.0
        self._quality: int = 0
        self._last_read_time: float = 0.0
        self._error_count: int = 0

        # ── Shi-Tomasi parameters ────────────────────────────────
        self._feature_params = dict(
            maxCorners=Config.CV_MAX_CORNERS,
            qualityLevel=Config.CV_QUALITY_LEVEL,
            minDistance=Config.CV_MIN_DISTANCE,
            blockSize=Config.CV_BLOCK_SIZE,
        )

        # ── Lucas-Kanade parameters ──────────────────────────────
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
        self._last_processed_frame: int = 0

    # ── Thread-safe Properties ─────────────────────────────────────

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
        """Number of successfully tracked inlier features."""
        with self._lock:
            return self._quality

    @property
    def data_age(self) -> float:
        """Seconds since the last valid computation (inf if none)."""
        with self._lock:
            if self._last_read_time == 0.0:
                return float("inf")
            return time.time() - self._last_read_time

    @property
    def error_count(self) -> int:
        """Total failed frame computations."""
        with self._lock:
            return self._error_count

    def get_flow(self) -> tuple:
        """Return ``(dx, dy, quality)`` atomically."""
        with self._lock:
            return (self._flow_dx, self._flow_dy, self._quality)

    def stop(self) -> None:
        """Signal the processor to stop."""
        self._running.clear()

    # ── Core LK Algorithm ──────────────────────────────────────────

    def _process_frame(self, gray: np.ndarray) -> None:
        """Run one iteration of Shi-Tomasi + LK flow.

        Args:
            gray: Current greyscale frame (uint8).
        """
        # ── First frame: detect corners only ─────────────────────
        if self._prev_gray is None:
            self._prev_gray = gray
            self._prev_points = cv2.goodFeaturesToTrack(
                gray, **self._feature_params
            )
            return

        # ── Too few features: re-detect ───────────────────────────
        if self._prev_points is None or len(self._prev_points) < 3:
            self._prev_gray = gray
            self._prev_points = cv2.goodFeaturesToTrack(
                gray, **self._feature_params
            )
            return

        # ── Forward LK tracking ───────────────────────────────────
        next_pts, status_fwd, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_points, None, **self._lk_params
        )
        if next_pts is None:
            self._prev_gray = gray
            self._prev_points = cv2.goodFeaturesToTrack(
                gray, **self._feature_params
            )
            return

        # ── Backward LK (outlier rejection) ──────────────────────
        back_pts, status_bwd, _ = cv2.calcOpticalFlowPyrLK(
            gray, self._prev_gray, next_pts, None, **self._lk_params
        )

        good_mask = np.zeros(len(status_fwd), dtype=bool)
        if back_pts is not None:
            for i in range(len(status_fwd)):
                if status_fwd[i][0] == 1 and status_bwd[i][0] == 1:
                    d = np.linalg.norm(self._prev_points[i] - back_pts[i])
                    if d < Config.CV_FB_THRESHOLD:
                        good_mask[i] = True

        n_good = int(np.sum(good_mask))

        if n_good < 2:
            with self._lock:
                self._flow_dx = 0.0
                self._flow_dy = 0.0
                self._quality = 0
            self._prev_gray = gray
            self._prev_points = cv2.goodFeaturesToTrack(
                gray, **self._feature_params
            )
            return

        # ── Median flow (robust to remaining outliers) ────────────
        prev_good = self._prev_points[good_mask]
        next_good = next_pts[good_mask]
        deltas = (next_good - prev_good).reshape(-1, 2)
        median_dx = float(np.median(deltas[:, 0]))
        median_dy = float(np.median(deltas[:, 1]))

        with self._lock:
            self._flow_dx = median_dx
            self._flow_dy = median_dy
            self._quality = n_good
            self._last_read_time = time.time()

        # ── Update for next frame ─────────────────────────────────
        self._prev_gray = gray
        if n_good < Config.CV_MAX_CORNERS // 2:
            self._prev_points = cv2.goodFeaturesToTrack(
                gray, **self._feature_params
            )
        else:
            self._prev_points = next_good.reshape(-1, 1, 2)

    # ── Thread Entry ───────────────────────────────────────────────

    def run(self) -> None:
        """Poll the shared frame buffer and compute LK flow."""
        Logger.info(
            f"WiFiLKProcessor starting — "
            f"maxCorners={Config.CV_MAX_CORNERS}, "
            f"winSize={Config.CV_LK_WIN_SIZE}"
        )

        while self._running.is_set():
            # Only process each new frame once
            current_count = self._reader.frame_count
            if current_count == self._last_processed_frame:
                time.sleep(0.005)
                continue

            self._last_processed_frame = current_count
            frame = self._reader.get_frame()

            if frame is None:
                time.sleep(0.005)
                continue

            try:
                self._process_frame(frame)
            except Exception as exc:
                Logger.warn(f"WiFiLKProcessor frame error: {exc}")
                with self._lock:
                    self._error_count += 1
                # Reset state so next frame starts fresh
                self._prev_gray = None
                self._prev_points = None

        Logger.info("WiFiLKProcessor stopped")


# ═══════════════════════════════════════════════════════════════════════════
#  ORB Keypoints + RANSAC Homography Flow  (replaces ORBFlowProcessor)
# ═══════════════════════════════════════════════════════════════════════════

class WiFiORBProcessor(threading.Thread):
    """ORB keypoint + BFMatcher + RANSAC homography optical flow
    from the shared WiFi frame buffer.

    Shares frames with ``WiFiLKProcessor`` via the same ``WiFiFrameReader``
    without opening a second HTTP connection to the ESP32-CAM.

    Algorithm:
        1. Detect ORB keypoints in current and previous frame.
        2. Match descriptors with BFMatcher (Hamming distance).
        3. Sort matches by distance; keep the best N.
        4. Compute homography via RANSAC from matched points.
        5. Extract translation (dx, dy) from H[0,2] and H[1,2].
        6. Quality = RANSAC inlier count.

    Args:
        frame_reader: Running ``WiFiFrameReader`` instance (shared).
    """

    def __init__(self, frame_reader: WiFiFrameReader) -> None:
        super().__init__(daemon=True, name="WiFiORBProcessor")
        self._reader = frame_reader
        self._lock = threading.Lock()
        self._running = threading.Event()
        self._running.set()

        # Output — same interface as WiFiLKProcessor
        self._flow_dx: float = 0.0
        self._flow_dy: float = 0.0
        self._quality: int = 0
        self._last_read_time: float = 0.0
        self._error_count: int = 0

        # Internal state
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_kp = None
        self._prev_des = None
        self._last_processed_frame: int = 0

        # ORB detector (created once; not thread-safe, used only in this thread)
        self._orb = cv2.ORB_create(nfeatures=Config.ORB_N_FEATURES)
        self._bf  = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # ── Thread-safe Properties ─────────────────────────────────────

    @property
    def flow_dx(self) -> float:
        with self._lock:
            return self._flow_dx

    @property
    def flow_dy(self) -> float:
        with self._lock:
            return self._flow_dy

    @property
    def quality(self) -> int:
        with self._lock:
            return self._quality

    @property
    def data_age(self) -> float:
        with self._lock:
            if self._last_read_time == 0.0:
                return float("inf")
            return time.time() - self._last_read_time

    @property
    def error_count(self) -> int:
        with self._lock:
            return self._error_count

    def get_flow(self) -> tuple:
        """Return ``(dx, dy, quality)`` atomically."""
        with self._lock:
            return (self._flow_dx, self._flow_dy, self._quality)

    def stop(self) -> None:
        """Signal the processor to stop."""
        self._running.clear()

    # ── Core ORB Algorithm ─────────────────────────────────────────

    def _process_frame(self, gray: np.ndarray) -> None:
        """Run one iteration of ORB + homography flow.

        Args:
            gray: Current greyscale frame (uint8).
        """
        kp, des = self._orb.detectAndCompute(gray, None)

        # First frame or no descriptors
        if self._prev_des is None or des is None:
            self._prev_gray = gray
            self._prev_kp   = kp
            self._prev_des  = des
            return

        if len(kp) < Config.ORB_MIN_MATCHES:
            self._prev_gray = gray
            self._prev_kp   = kp
            self._prev_des  = des
            with self._lock:
                self._error_count += 1
            return

        # Match descriptors
        try:
            matches = self._bf.match(self._prev_des, des)
        except cv2.error:
            self._prev_gray = gray
            self._prev_kp   = kp
            self._prev_des  = des
            with self._lock:
                self._error_count += 1
            return

        # Sort by distance; keep best matches only
        matches = sorted(matches, key=lambda m: m.distance)
        matches = matches[:min(len(matches), Config.ORB_N_FEATURES)]

        if len(matches) < Config.ORB_MIN_MATCHES:
            self._prev_gray = gray
            self._prev_kp   = kp
            self._prev_des  = des
            return

        # Build point arrays for homography
        src_pts = np.float32(
            [self._prev_kp[m.queryIdx].pt for m in matches]
        ).reshape(-1, 1, 2)
        dst_pts = np.float32(
            [kp[m.trainIdx].pt for m in matches]
        ).reshape(-1, 1, 2)

        # RANSAC homography
        h_mat, mask = cv2.findHomography(
            src_pts, dst_pts, cv2.RANSAC, Config.ORB_RANSAC_THRESH
        )

        if h_mat is None or mask is None:
            self._prev_gray = gray
            self._prev_kp   = kp
            self._prev_des  = des
            with self._lock:
                self._error_count += 1
            return

        n_inliers = int(mask.sum())
        if n_inliers < Config.ORB_MIN_MATCHES:
            self._prev_gray = gray
            self._prev_kp   = kp
            self._prev_des  = des
            return

        # Extract translation from homography
        # H[0,2] = dx, H[1,2] = dy (pixels)
        dx = float(h_mat[0, 2])
        dy = float(h_mat[1, 2])

        with self._lock:
            self._flow_dx = dx
            self._flow_dy = dy
            self._quality = n_inliers
            self._last_read_time = time.time()

        self._prev_gray = gray
        self._prev_kp   = kp
        self._prev_des  = des

    # ── Thread Entry ───────────────────────────────────────────────

    def run(self) -> None:
        """Poll the shared frame buffer and compute ORB flow."""
        Logger.info(
            f"WiFiORBProcessor starting — "
            f"nfeatures={Config.ORB_N_FEATURES}, "
            f"RANSAC_thresh={Config.ORB_RANSAC_THRESH}, "
            f"min_matches={Config.ORB_MIN_MATCHES}"
        )

        while self._running.is_set():
            current_count = self._reader.frame_count
            if current_count == self._last_processed_frame:
                time.sleep(0.01)
                continue

            self._last_processed_frame = current_count
            frame = self._reader.get_frame()

            if frame is None:
                time.sleep(0.01)
                continue

            try:
                self._process_frame(frame)
            except Exception as exc:
                Logger.warn(f"WiFiORBProcessor frame error: {exc}")
                with self._lock:
                    self._error_count += 1
                self._prev_gray = None
                self._prev_kp   = None
                self._prev_des  = None

        Logger.info("WiFiORBProcessor stopped")
