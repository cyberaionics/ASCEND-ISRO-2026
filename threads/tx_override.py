"""
ASCEND Phase 2 — TX Override Watcher
Daemon thread: polls RC_CHANNELS every 100 ms with debounce logic.
"""

import threading
import time
from typing import Callable, Optional

from ..config import Config
from ..logger import Logger
from ..drone_state import DroneState
from ..hardware.pixhawk import PixhawkLink


class TXOverrideWatcher(threading.Thread):
    """Debounced RC transmitter presence detector.

    Polls ``RC_CHANNELS.chan1_raw`` from the Pixhawk every 100 ms.
    Uses **5-reading ON debounce** (500 ms) and **3-reading OFF
    debounce** (300 ms) to prevent RF glitches from causing
    accidental mode switches.

    On TX detected (manual_mode False → True):
        1. Sets ``DroneState.manual_mode = True``
        2. Sends ``SET_MODE: STABILIZE``
        3. Calls ``on_tx_detected()`` callback

    On TX lost (manual_mode True → False):
        1. Sets ``DroneState.manual_mode = False``
        2. Calls ``on_tx_lost()`` callback

    Args:
        pixhawk: Shared PixhawkLink instance.
        drone_state: Shared DroneState instance.
        on_tx_detected: Callback invoked when TX override engages.
        on_tx_lost:     Callback invoked when TX override disengages.
    """

    def __init__(
        self,
        pixhawk: PixhawkLink,
        drone_state: DroneState,
        on_tx_detected: Optional[Callable[[], None]] = None,
        on_tx_lost: Optional[Callable[[], None]] = None,
    ) -> None:
        super().__init__(daemon=True, name="TXOverrideWatcher")
        self._px = pixhawk
        self._ds = drone_state
        self._on_tx_detected = on_tx_detected
        self._on_tx_lost = on_tx_lost
        self._running = threading.Event()
        self._running.set()

        # Debounce counters
        self._on_count: int = 0
        self._off_count: int = 0

    def stop(self) -> None:
        self._running.clear()

    def run(self) -> None:
        Logger.info("TXOverrideWatcher running (100 ms poll, debounced)")
        while self._running.is_set():
            try:
                self._poll()
            except Exception as exc:
                Logger.warn(f"TX watcher error: {exc}")
            time.sleep(Config.TX_POLL_INTERVAL)
        Logger.info("TXOverrideWatcher stopped")

    # ── Poll Logic ─────────────────────────────────────────────────────

    def _poll(self) -> None:
        """Read one RC_CHANNELS message and update debounce state."""
        msg = self._px.recv("RC_CHANNELS", timeout=0.08)
        if msg is None:
            return

        ch1 = msg.chan1_raw
        tx_on = Config.RC_PWM_MIN <= ch1 <= Config.RC_PWM_MAX

        manual = self._ds.manual_mode

        if tx_on:
            self._off_count = 0
            self._on_count += 1
            if not manual and self._on_count >= Config.TX_ON_DEBOUNCE:
                # TX just appeared — hand off to pilot
                state_name = self._ds.current_state.value
                Logger.warn(f"TX OVERRIDE: manual control active "
                            f"at state {state_name} (ch1={ch1})")
                self._ds.set_manual_mode(True)
                self._px.set_mode("STABILIZE")
                if self._on_tx_detected:
                    self._on_tx_detected()
        else:
            self._on_count = 0
            self._off_count += 1
            if manual and self._off_count >= Config.TX_OFF_DEBOUNCE:
                # TX disappeared — return to autonomous
                Logger.info("TX lost: returning to IDLE, "
                            "awaiting new start command")
                self._ds.set_manual_mode(False)
                if self._on_tx_lost:
                    self._on_tx_lost()
