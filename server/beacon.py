"""UDP broadcast for LAN auto-discovery of the rover server."""
from __future__ import annotations

import json
import socket
import threading
import time

from logging_setup import get_logger

log = get_logger("server.beacon")


# -- Beacon cache (shared between listener thread and API endpoint) --
_beacon_cache: dict[str, dict] = {}
_cache_lock = threading.Lock()
BEACON_PORT = 5002
BEACON_TTL = 12.0

class BeaconListener:
    def __init__(self, port: int = 5002) -> None:
        self._port = port
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="beacon-listener")
        self._thread.start()
        log.info("beacon listener started on :%d", self._port)

    def stop(self, join_timeout: float = 1.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=join_timeout)

    def _loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.5)
        try:
            sock.bind(("", self._port))
        except OSError:
            log.warning("beacon listener could not bind to :%d", self._port)
            sock.close()
            return
        try:
            while not self._stop.is_set():
                try:
                    data, addr = sock.recvfrom(4096)
                    self._handle(data, addr[0])
                except socket.timeout:
                    continue
                except Exception as exc:
                    if not self._stop.is_set():
                        log.debug("beacon recv error: %s", exc)
        finally:
            sock.close()

    @staticmethod
    def _handle(data: bytes, src_ip: str) -> None:
        try:
            msg = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return
        rid = msg.get("rover_id") or msg.get("id", "")
        if not rid:
            return
        entry = {
            "id": rid, "name": msg.get("name", rid),
            "host": msg.get("ip", src_ip), "port": msg.get("port", 5001),
            "type": msg.get("type", "drawing"), "version": msg.get("version", ""),
            "auth_required": msg.get("auth_required", True), "last_seen": time.time(),
        }
        with _cache_lock:
            _beacon_cache[rid] = entry


def get_beacons() -> list[dict]:
    now = time.time()
    with _cache_lock:
        active = [b for b in _beacon_cache.values() if now - b["last_seen"] < BEACON_TTL]
        active.sort(key=lambda b: b["last_seen"], reverse=True)
        return active


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
