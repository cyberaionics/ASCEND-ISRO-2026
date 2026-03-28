"""
ASCEND Phase 2 — Command Receiver
Daemon thread: UDP server listening for laptop commands on CMD_PORT.
"""

import json
import socket
import threading
import time

from ..config import Config
from ..logger import Logger
from ..drone_state import DroneState


class CommandReceiver(threading.Thread):
    """Listens for JSON commands from the laptop over UDP.

    Accepted commands:
        ``{"cmd": "start_mission"}``  → sets :attr:`start_flag`
        ``{"cmd": "abort_mission"}``  → sets :attr:`abort_flag`
        ``{"cmd": "heartbeat"}``      → updates ``last_heartbeat_rx`` only

    Every received packet (regardless of command) updates
    ``DroneState.last_heartbeat_rx`` for the SafetyMonitor Wi-Fi check.

    Malformed JSON and unknown ``cmd`` values are logged and ignored.
    ``start_mission`` is rejected if ``manual_mode`` is ``True``.

    Args:
        drone_state: Shared DroneState instance.
    """

    def __init__(self, drone_state: DroneState) -> None:
        super().__init__(daemon=True, name="CommandReceiver")
        self._ds = drone_state
        self._running = threading.Event()
        self._running.set()

        self.start_flag = threading.Event()
        self.abort_flag = threading.Event()

    def stop(self) -> None:
        self._running.clear()

    def run(self) -> None:
        Logger.info(f"CommandReceiver listening on UDP :{Config.CMD_PORT}")
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", Config.CMD_PORT))
            sock.settimeout(0.1)  # 100 ms blocking timeout for clean stop
        except OSError as exc:
            Logger.error(f"CommandReceiver cannot bind: {exc}")
            return

        try:
            while self._running.is_set():
                try:
                    data, addr = sock.recvfrom(1024)
                except socket.timeout:
                    continue
                except OSError:
                    continue

                # Every packet counts as a Wi-Fi heartbeat
                self._ds.set_last_heartbeat_rx(time.time())

                try:
                    payload = json.loads(data.decode("utf-8"))
                except (json.JSONDecodeError, UnicodeDecodeError):
                    Logger.warn(f"CommandReceiver: malformed JSON from {addr}")
                    continue

                cmd = payload.get("cmd")
                if cmd == "start_mission":
                    if self._ds.manual_mode:
                        Logger.warn("start_mission ignored — TX override active")
                    else:
                        Logger.ok(f"start_mission received from {addr}")
                        self.start_flag.set()
                elif cmd == "abort_mission":
                    Logger.warn(f"abort_mission received from {addr}")
                    self.abort_flag.set()
                elif cmd == "heartbeat":
                    pass  # timestamp already updated above
                else:
                    Logger.warn(f"Unknown command '{cmd}' from {addr}")
        finally:
            sock.close()
            Logger.info("CommandReceiver stopped")
