"""
ASCEND Phase 2 — Scheduler (Entry Point)
Instantiates shared objects, starts daemon threads, runs StateMachine.
Usage: python3 -m ascend.scheduler --mode fly
"""

import argparse
import signal
import sys
import threading
import time
from typing import List

from .config import Config
from .logger import Logger
from .drone_state import DroneState
from .hardware.tf02 import TF02Reader
from .hardware.pixhawk import PixhawkLink
from .threads.bridge import RangefinderBridge
from .threads.heartbeat import HeartbeatSender
from .threads.tx_override import TXOverrideWatcher
from .threads.safety import SafetyMonitor
from .threads.telemetry import TelemetryStreamer
from .threads.commands import CommandReceiver
from .state_machine import StateMachine


class Scheduler:
    """CLI entry point — wires up all components and runs the flight mission.

    Object creation order:
        1. DroneState (shared state)
        2. PixhawkLink (connect to FC)
        3. TF02Reader (start UART reading)
        4. Emergency flag (threading.Event)
        5. All daemon threads
        6. StateMachine (runs on main thread)

    Supports a single mode: ``--mode fly``
    """

    def __init__(self) -> None:
        self._threads: List[threading.Thread] = []
        self._ds = DroneState()
        self._px = PixhawkLink()
        self._tf02 = TF02Reader()
        self._eflag = threading.Event()
        self._sm: StateMachine = None  # type: ignore

    def run(self) -> int:
        """Start all components and run the state machine.

        Returns:
            Exit code (0 = success, 1 = failure).
        """
        Logger.banner()
        Logger.info("Mode: fly")
        Logger.info(f"Pixhawk port : {Config.PIXHAWK_PORT}")
        Logger.info(f"TF-02 port   : {Config.TF02_PORT}")
        Logger.info(f"Laptop IP    : {Config.LAPTOP_IP}")
        Logger.info(f"Telemetry    : UDP :{Config.TELEMETRY_PORT}")
        Logger.info(f"Commands     : UDP :{Config.CMD_PORT}")
        Logger.info("")

        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

        try:
            # 1. Connect to Pixhawk
            if not self._px.connect():
                return 1

            # 2. Start TF-02 reader
            self._tf02.start()
            self._threads.append(self._tf02)
            time.sleep(0.5)  # let TF-02 get first reading

            # 3. Command receiver (must start before state machine
            #    so start_flag is available for IDLE state)
            cmd_rx = CommandReceiver(self._ds)
            cmd_rx.start()
            self._threads.append(cmd_rx)

            # 4. Create state machine (needs cmd_rx reference)
            self._sm = StateMachine(
                pixhawk=self._px,
                tf02=self._tf02,
                drone_state=self._ds,
                emergency_flag=self._eflag,
                cmd_rx=cmd_rx,
            )

            # 5. TX Override Watcher — uses SM callback for on_tx_lost
            tx_watcher = TXOverrideWatcher(
                pixhawk=self._px,
                drone_state=self._ds,
                on_tx_detected=None,  # handled internally by TXOverrideWatcher
                on_tx_lost=self._sm.on_tx_lost,
            )
            tx_watcher.start()
            self._threads.append(tx_watcher)

            # 6. Safety Monitor
            safety = SafetyMonitor(
                tf02=self._tf02,
                pixhawk=self._px,
                drone_state=self._ds,
                emergency_flag=self._eflag,
            )
            safety.start()
            self._threads.append(safety)

            # 7. Rangefinder Bridge
            bridge = RangefinderBridge(self._tf02, self._px)
            bridge.start()
            self._threads.append(bridge)

            # 8. Heartbeat Sender
            hb = HeartbeatSender(self._px)
            hb.start()
            self._threads.append(hb)

            # 9. Telemetry Streamer
            telem = TelemetryStreamer(self._ds)
            telem.start()
            self._threads.append(telem)

            Logger.ok("All threads started — entering state machine\n")

            # 10. Run state machine on main thread (blocks)
            self._sm.run()
            return 0

        except KeyboardInterrupt:
            Logger.warn("Interrupted — shutting down")
            return 0
        finally:
            self._cleanup()

    # ── Internal ───────────────────────────────────────────────────────

    def _cleanup(self) -> None:
        """Stop all threads and close connections."""
        Logger.info("Cleaning up …")
        for t in self._threads:
            if hasattr(t, "stop"):
                t.stop()
        for t in self._threads:
            t.join(timeout=2.0)
        self._px.close()
        Logger.ok("Shutdown complete")

    def _sig_handler(self, signum: int, frame: object) -> None:
        """Translate OS signals into KeyboardInterrupt."""
        raise KeyboardInterrupt


# ═══════════════════════════════════════════════════════════════════════════
# Entry Point
# ═══════════════════════════════════════════════════════════════════════════

def main() -> None:
    """Parse CLI arguments and run the scheduler."""
    parser = argparse.ArgumentParser(
        prog="ascend",
        description="ASCEND Phase 2 — Autonomous drone flight system",
    )
    parser.add_argument(
        "--mode",
        required=True,
        choices=["fly"],
        help="Operating mode (Phase 2 supports: fly)",
    )
    parser.parse_args()

    scheduler = Scheduler()
    code = scheduler.run()
    sys.exit(code)


if __name__ == "__main__":
    main()
