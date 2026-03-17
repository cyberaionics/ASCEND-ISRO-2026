"""
ASCEND Phase 1 — Health Checker
Pre-flight diagnostic: PID values, vibration levels, TF-02 rangefinder.
"""

import time
from typing import Optional

from ..config import Config
from ..logger import Logger
from ..hardware.pixhawk import PixhawkLink
from ..hardware.tf02 import TF02Reader


class HealthChecker:
    """Run three pre-flight health checks and report pass/fail.

    Check 1 — PID values: read current PIDs, compare to defaults.
    Check 2 — Vibration levels: sample 5 s, grade X/Y/Z, detect clipping.
    Check 3 — TF-02 rangefinder: sample 5 s, validate range and strength.

    Returns ``True`` from :meth:`run_all` only if every check passes.

    Args:
        pixhawk: Connected PixhawkLink instance.
        tf02:    TF02Reader instance (thread may or may not be started).
    """

    def __init__(self, pixhawk: PixhawkLink,
                 tf02: Optional[TF02Reader] = None) -> None:
        self._px = pixhawk
        self._tf02 = tf02

    # ── Public API ─────────────────────────────────────────────────────

    def run_all(self) -> bool:
        """Run all three health checks sequentially.

        Returns:
            ``True`` if all checks passed.
        """
        Logger.header("ASCEND HEALTH CHECK")
        r1 = self.check_pids()
        r2 = self.check_vibration()
        r3 = self.check_tf02()

        Logger.section("Summary")
        Logger.kv("PID check", "PASS" if r1 else "FAIL")
        Logger.kv("Vibration check", "PASS" if r2 else "FAIL")
        Logger.kv("TF-02 check", "PASS" if r3 else "FAIL")

        if r1 and r2 and r3:
            Logger.ok("All health checks PASSED ✓")
            return True
        else:
            Logger.error("One or more health checks FAILED ✗")
            return False

    # ── Check 1: PID Values ────────────────────────────────────────────

    def check_pids(self) -> bool:
        """Read pitch/roll PID parameters and flag default vs custom values.

        Returns:
            ``True`` (informational — never blocks flight by itself).
        """
        Logger.section("Check 1 — PID Values")
        all_default = True

        for name, default_val in Config.DEFAULT_PIDS.items():
            val = self._px.read_param(name)
            if val is None:
                Logger.warn(f"  {name}: COULD NOT READ")
                all_default = False
                continue

            if abs(val - default_val) < 0.0001:
                tag = f"\033[93m[DEFAULT]\033[0m"
            else:
                tag = f"\033[92m[CUSTOM]\033[0m"
                all_default = False

            Logger.kv(f"  {name}", f"{val:.4f}  {tag}")

        if all_default:
            Logger.warn("All PIDs at ArduCopter defaults — AutoTune recommended")
        else:
            Logger.info("Custom PIDs detected — will be reset before AutoTune")

        return True  # informational check

    # ── Check 2: Vibration Levels ──────────────────────────────────────

    def check_vibration(self) -> bool:
        """Sample VIBRATION message for 5 seconds and grade vibration.

        Returns:
            ``True`` if vibration is below threshold and no IMU clipping.
        """
        Logger.section("Check 2 — Vibration Levels")
        Logger.info(f"Sampling vibration for {Config.VIB_SAMPLE_TIME:.0f}s "
                     "(keep props OFF) …")

        samples: list = []
        clip_total = [0, 0, 0]
        deadline = time.time() + Config.VIB_SAMPLE_TIME

        while time.time() < deadline:
            msg = self._px.recv("VIBRATION", timeout=1.0)
            if msg is not None:
                samples.append((msg.vibration_x, msg.vibration_y, msg.vibration_z))
                clip_total[0] += msg.clipping_0
                clip_total[1] += msg.clipping_1
                clip_total[2] += msg.clipping_2

        if not samples:
            Logger.error("No VIBRATION messages received — check connection")
            return False

        # Compute averages
        n = len(samples)
        avg_x = sum(s[0] for s in samples) / n
        avg_y = sum(s[1] for s in samples) / n
        avg_z = sum(s[2] for s in samples) / n
        Logger.kv("Samples", n)
        Logger.kv("Avg X", f"{avg_x:.2f} m/s²")
        Logger.kv("Avg Y", f"{avg_y:.2f} m/s²")
        Logger.kv("Avg Z", f"{avg_z:.2f} m/s²")

        # Grade each axis
        passed = True
        for axis, val in [("X", avg_x), ("Y", avg_y), ("Z", avg_z)]:
            grade = self._vib_grade(val)
            Logger.kv(f"  {axis} grade", grade)
            if val >= Config.VIB_HIGH:
                passed = False

        # Clipping check — critical
        if any(c > 0 for c in clip_total):
            Logger.error(f"IMU CLIPPING detected: {clip_total} — DO NOT FLY")
            return False

        Logger.ok("No IMU clipping detected") if passed else None
        return passed

    @staticmethod
    def _vib_grade(val: float) -> str:
        """Return a colour-coded vibration grade string."""
        if val < Config.VIB_EXCELLENT:
            return f"\033[92mEXCELLENT (< {Config.VIB_EXCELLENT})\033[0m"
        if val < Config.VIB_GOOD:
            return f"\033[92mGOOD (< {Config.VIB_GOOD})\033[0m"
        if val < Config.VIB_HIGH:
            return f"\033[93mHIGH — add foam damping\033[0m"
        return f"\033[91mDANGEROUS (≥ {Config.VIB_HIGH}) — DO NOT FLY\033[0m"

    # ── Check 3: TF-02 Rangefinder ─────────────────────────────────────

    def check_tf02(self) -> bool:
        """Read 5 seconds of TF-02 distance samples and validate.

        Returns:
            ``True`` if valid readings were received in range 30–800 cm.
        """
        Logger.section("Check 3 — TF-02 Rangefinder")

        if self._tf02 is None:
            Logger.info("Starting temporary TF02Reader for health check …")
            tf02 = TF02Reader()
            tf02.start()
            time.sleep(0.5)  # let the thread init
        else:
            tf02 = self._tf02

        Logger.info(f"Sampling TF-02 for {Config.TF02_SAMPLE_TIME:.0f}s …")
        samples: list = []
        deadline = time.time() + Config.TF02_SAMPLE_TIME
        prev_dist: Optional[int] = None

        while time.time() < deadline:
            dist = tf02.distance_cm
            strength = tf02.strength
            if dist is not None and dist != prev_dist:
                samples.append(dist)
                prev_dist = dist
                # Live print every ~0.5s worth of samples
                if len(samples) % 10 == 1:
                    Logger.info(f"  dist={dist} cm  strength={strength}")
            time.sleep(0.01)

        # Stop temp reader if we created one
        if self._tf02 is None:
            tf02.stop()
            tf02.join(timeout=2.0)

        if not samples:
            Logger.error("No TF-02 readings received — check wiring")
            return False

        avg = sum(samples) / len(samples)
        in_range = sum(1 for s in samples
                       if Config.TF02_MIN_CM <= s <= Config.TF02_MAX_CM)
        Logger.kv("Total samples", len(samples))
        Logger.kv("Average dist", f"{avg:.1f} cm")
        Logger.kv("Valid range", f"{in_range}/{len(samples)}")
        Logger.kv("Errors", tf02.error_count)

        if in_range == 0:
            Logger.error("No readings within valid range (30–800 cm)")
            return False

        Logger.ok(f"TF-02 healthy — {len(samples)} samples, avg {avg:.1f} cm")
        return True
