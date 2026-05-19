#!/usr/bin/env python3
"""NTRIP client -> MAVROS RTK injection for PX4.

Connects to an NTRIP caster, parses RTCM3 frames from the stream,
and publishes them to /mavros/gps_rtk/send_rtcm for PX4 RTK injection.
Sends GGA back-feed every 10 seconds as required by NTRIP v1 casters.
"""

import os
import socket
import base64
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from mavros_msgs.msg import RTCM
from sensor_msgs.msg import NavSatFix, NavSatStatus

# ---------------------------------------------------------------------------
# Configuration via environment variables
# ---------------------------------------------------------------------------
NTRIP_HOST = os.environ.get("NTRIP_HOST", "caster.emlid.com")
NTRIP_PORT = int(os.environ.get("NTRIP_PORT", "2101"))
NTRIP_MOUNTPT = os.environ.get("NTRIP_MOUNTPT", "MP23960a")

_NTRIP_USER = os.environ.get("NTRIP_USER")
_NTRIP_PASS = os.environ.get("NTRIP_PASS")

if not _NTRIP_USER:
    raise RuntimeError("NTRIP_USER environment variable is required (no default)")
if not _NTRIP_PASS:
    raise RuntimeError("NTRIP_PASS environment variable is required (no default)")

# RTCM3 frame constants
_RTCM3_PREAMBLE = 0xD3
_RTCM3_HEADER_LEN = 3       # preamble(1) + length(2)
_RTCM3_CRC_LEN = 3

# CRC-24Q lookup table (RTCM3 standard polynomial 0x1864CFB)
_CRC24Q_TABLE = [0] * 256
for _i in range(256):
    _crc = _i << 16
    for _ in range(8):
        _crc = ((_crc << 1) ^ 0x1864CFB) if (_crc & 0x800000) else (_crc << 1)
        _crc &= 0xFFFFFF
    _CRC24Q_TABLE[_i] = _crc


def _rtcm3_crc(data: bytes, length: int) -> int:
    """Compute RTCM3 CRC-24Q over data[0:length]."""
    crc = 0
    for i in range(length):
        crc = ((crc << 8) ^ _CRC24Q_TABLE[((crc >> 16) & 0xFF) ^ data[i]]) & 0xFFFFFF
    return crc


def _nmea_checksum(sentence: str) -> str:
    """Compute NMEA checksum for a sentence (between $ and *)."""
    chk = 0
    for c in sentence:
        chk ^= ord(c)
    return f"{chk:02X}"


class NtripNode(Node):
    """ROS2 node that streams RTCM3 corrections from an NTRIP caster to MAVROS."""

    def __init__(self):
        super().__init__("ntrip_rtcm_node")

        # -- Publisher (BEST_EFFORT, depth=10) --
        rtcm_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(RTCM, "/mavros/gps_rtk/send_rtcm", rtcm_qos)

        # -- GGA back-feed subscriber --
        self._gps_fix = None
        self._gps_lock = threading.Lock()
        self.create_subscription(
            NavSatFix,
            "/mavros/global_position/raw/fix",
            self._gps_callback,
            10,
        )

        # -- Thread control --
        self._stop_event = threading.Event()
        self._gga_lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        # -- Health monitoring --
        self._last_data_time = self.get_clock().now()
        self._reconnect_count = 0
        self.create_timer(30.0, self._check_health)

        # -- GGA back-feed timer (every 10 s) --
        self._gga_sock = None  # set after connect
        self._gga_fail_count = 0
        self.create_timer(10.0, self._send_gga)

        self.get_logger().info(
            f"Starting NTRIP client: {NTRIP_HOST}:{NTRIP_PORT}/{NTRIP_MOUNTPT}"
        )

    # ------------------------------------------------------------------
    # GPS callback — store latest fix for GGA sentence
    # ------------------------------------------------------------------
    def _gps_callback(self, msg: NavSatFix):
        with self._gps_lock:
            self._gps_fix = msg

    # ------------------------------------------------------------------
    # GGA formatting
    # ------------------------------------------------------------------
    def _format_gga(self) -> str:
        """Build a NMEA GGA sentence from the latest GPS fix."""
        with self._gps_lock:
            fix = self._gps_fix

        if fix is None:
            return None

        lat = fix.latitude
        lon = fix.longitude
        alt = fix.altitude

        # Latitude: ddmm.mmmm
        lat_abs = abs(lat)
        lat_deg = int(lat_abs)
        lat_min = (lat_abs - lat_deg) * 60.0
        lat_dir = "N" if lat >= 0 else "S"
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"

        # Longitude: dddmm.mmmm
        lon_abs = abs(lon)
        lon_deg = int(lon_abs)
        lon_min = (lon_abs - lon_deg) * 60.0
        lon_dir = "E" if lon >= 0 else "W"
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"

        # Time from the stamp
        stamp_sec = fix.header.stamp.sec  # POSIX epoch seconds
        if stamp_sec > 0:
            t = time.gmtime(stamp_sec)
            time_str = f"{t.tm_hour:02d}{t.tm_min:02d}{t.tm_sec:02d}.00"
        else:
            time_str = "000000.00"

        # Quality: 1=GPS, 2=DGPS, 4=RTK Fixed, 5=RTK Float
        quality = 1
        if fix.status.status == NavSatStatus.STATUS_GBAS_FIX:
            quality = 4 if fix.position_covariance[0] < 0.01 else 5
        elif fix.status.status == NavSatStatus.STATUS_SBAS_FIX:
            quality = 2

        num_sats = 8  # placeholder; NavSatFix doesn't carry this
        hdop = 1.0    # placeholder

        body = (
            f"GPGGA,{time_str},{lat_str},{lat_dir},{lon_str},{lon_dir},"
            f"{quality},{num_sats},{hdop:.1f},{alt:.1f},M,0.0,M,,"
        )
        ck = _nmea_checksum(body)
        return f"${body}*{ck}\r\n"

    # ------------------------------------------------------------------
    # GGA send timer callback
    # ------------------------------------------------------------------
    def _send_gga(self):
        gga = self._format_gga()
        if gga is None:
            return
        with self._gga_lock:
            sock = self._gga_sock
        if sock is None:
            return
        try:
            sock.sendall(gga.encode())
            self._gga_fail_count = 0  # reset on success
            self.get_logger().debug("Sent GGA back-feed to caster")
        except Exception as e:
            self._gga_fail_count += 1
            if self._gga_fail_count <= 3:
                self.get_logger().warn(f"Failed to send GGA: {e}")
            elif self._gga_fail_count == 4:
                self.get_logger().warn(
                    "GGA send failures suppressed (socket may be reconnecting)"
                )

    # ------------------------------------------------------------------
    # Health check timer callback
    # ------------------------------------------------------------------
    def _check_health(self):
        now = self.get_clock().now()
        elapsed = (now - self._last_data_time).nanoseconds / 1e9
        if elapsed > 30.0:
            self.get_logger().warn(
                f"No RTCM data for {elapsed:.0f}s "
                f"(reconnects: {self._reconnect_count})"
            )

    # ------------------------------------------------------------------
    # NTRIP connection
    # ------------------------------------------------------------------
    def _connect(self):
        """Open NTRIP connection; returns (socket, leftover_bytes)."""
        creds = base64.b64encode(
            f"{_NTRIP_USER}:{_NTRIP_PASS}".encode()
        ).decode()

        req = (
            f"GET /{NTRIP_MOUNTPT} HTTP/1.0\r\n"
            f"Host: {NTRIP_HOST}\r\n"
            f"Ntrip-Version: Ntrip/2.0\r\n"
            f"User-Agent: NTRIP ROS2/1.0\r\n"
            f"Authorization: Basic {creds}\r\n\r\n"
        )

        s = socket.socket()
        s.settimeout(10)
        s.connect((NTRIP_HOST, NTRIP_PORT))
        s.sendall(req.encode())

        # Read response — handle both HTTP and ICY headers
        resp = b""
        while True:
            chunk = s.recv(256)
            if not chunk:
                raise ConnectionError("Caster closed connection during handshake")
            resp += chunk
            # Standard HTTP-style header end
            if b"\r\n\r\n" in resp:
                header, leftover = resp.split(b"\r\n\r\n", 1)
                header = header.decode(errors="ignore")
                break
            # ICY casters: "ICY 200 OK\r\n" then binary immediately
            if b"ICY 200 OK" in resp and len(resp) > 20:
                lines = resp.split(b"\r\n", 1)
                header = lines[0].decode(errors="ignore")
                leftover = lines[1] if len(lines) > 1 else b""
                break
            if len(resp) > 2048:
                raise ConnectionError(f"Bad response: {resp[:80]}")

        if "200" not in header:
            raise ConnectionError(f"Caster rejected: {header}")

        self.get_logger().info("Connected — streaming RTCM3")
        s.settimeout(30)
        with self._gga_lock:
            self._gga_sock = s
        return s, leftover

    # ------------------------------------------------------------------
    # RTCM3 frame parser
    # ------------------------------------------------------------------
    @staticmethod
    def _parse_rtcm_frames(buf: bytes, logger=None):
        """Extract complete RTCM3 frames from a byte buffer.

        Returns (frames: list[bytes], remaining: bytes).
        Incomplete frames are held in `remaining` for the next recv.
        Frames with invalid CRC-24Q are discarded.
        """
        frames = []
        i = 0
        while i < len(buf):
            # Scan for preamble
            if buf[i] != _RTCM3_PREAMBLE:
                i += 1
                continue

            # Need at least 3 header bytes to read length
            if i + _RTCM3_HEADER_LEN > len(buf):
                break  # partial header, wait for more data

            # Length is in bits 6-15 of the 2-byte field after preamble
            length_field = (buf[i + 1] << 8) | buf[i + 2]
            # Reserved bits 0-5 should be zero per spec, but some casters set them
            if length_field & 0xFC00 != 0:
                if logger:
                    logger.debug(
                        f"RTCM3 reserved bits set (0x{length_field:04X}) — "
                        f"proceeding anyway"
                    )
            msg_len = length_field & 0x03FF
            total_frame = _RTCM3_HEADER_LEN + msg_len + _RTCM3_CRC_LEN

            if i + total_frame > len(buf):
                break  # incomplete frame, wait for more data

            frame = buf[i : i + total_frame]

            # Validate CRC-24Q
            payload_len = _RTCM3_HEADER_LEN + msg_len
            expected_crc = (frame[payload_len] << 16) | (frame[payload_len + 1] << 8) | frame[payload_len + 2]
            computed_crc = _rtcm3_crc(frame, payload_len)
            if computed_crc != expected_crc:
                if logger:
                    logger.warn(
                        f"RTCM3 CRC mismatch at offset {i}: "
                        f"expected 0x{expected_crc:06X}, got 0x{computed_crc:06X} — "
                        f"discarding frame"
                    )
                i += 1  # skip this preamble, scan for next
                continue

            frames.append(frame)
            i += total_frame

        remaining = buf[i:]
        return frames, remaining

    # ------------------------------------------------------------------
    # Main NTRIP loop (runs in background thread)
    # ------------------------------------------------------------------
    def _run(self):
        attempt = 0

        while not self._stop_event.is_set():
            sock = None
            try:
                sock, buf = self._connect()
                attempt = 0  # reset backoff on successful connect

                while not self._stop_event.is_set():
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        self.get_logger().warn("Socket timeout, reconnecting...")
                        break

                    if not chunk:
                        self.get_logger().warn("Stream ended, reconnecting...")
                        break

                    buf += chunk

                    # Extract and publish complete RTCM3 frames
                    frames, buf = self._parse_rtcm_frames(buf, self.get_logger())
                    for frame in frames:
                        msg = RTCM()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.data = list(frame)
                        self.pub.publish(msg)
                        self._last_data_time = self.get_clock().now()

            except Exception as e:
                self._reconnect_count += 1
                self.get_logger().error(
                    f"NTRIP error: {e} — reconnect #{self._reconnect_count}"
                )
            finally:
                with self._gga_lock:
                    self._gga_sock = None
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass

            # Exponential backoff: min(5 * 2^attempt, 60) seconds
            backoff = min(5 * (2 ** attempt), 60)
            attempt += 1
            self.get_logger().info(f"Reconnecting in {backoff}s...")
            self._stop_event.wait(backoff)  # interruptible sleep

    # ------------------------------------------------------------------
    # Graceful shutdown
    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("Shutting down NTRIP node...")
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=5)
        super().destroy_node()


def main():
    rclpy.init()
    node = NtripNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()