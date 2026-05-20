"""UDP broadcast for LAN auto-discovery of the rover server."""
from __future__ import annotations

import json
import socket
import threading

from logging_setup import get_logger

log = get_logger("server.beacon")


class RoverBeacon:
    def __init__(
        self,
        port: int        = 5002,
        interval: float  = 2.0,
        rover_id: str    = "drawing_rover_1",
        server_port: int = 5001,
    ) -> None:
        self._port     = port
        self._interval = interval
        self._stop     = threading.Event()
        self._thread: threading.Thread | None = None

        ip = self._get_local_ip()
        self._payload = json.dumps({
            "rover_id": rover_id,
            "ip":       ip,
            "port":     server_port,
            "type":     "drawing",
            "version":  "1.0.0",
        }).encode()
        log.info("beacon configured: %s:%d → broadcast :%d every %.1fs",
                 ip, server_port, port, interval)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="rover-beacon"
        )
        self._thread.start()

    def stop(self, join_timeout: float = 1.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=join_timeout)

    # ── Internal ──────────────────────────────────────────────────────────────

    @staticmethod
    def _get_local_ip() -> str:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "0.0.0.0"

    def _loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            while not self._stop.is_set():
                try:
                    sock.sendto(self._payload, ("<broadcast>", self._port))
                except Exception as exc:
                    log.warning("beacon send failed: %s", exc)
                # Event.wait returns True when set, allowing prompt shutdown
                if self._stop.wait(self._interval):
                    break
        finally:
            sock.close()
