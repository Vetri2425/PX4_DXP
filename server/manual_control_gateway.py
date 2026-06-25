"""Virtual joystick MANUAL_CONTROL transport and freshness gateway."""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Protocol

from config import (
    JOYSTICK_GATEWAY_RATE_HZ,
    JOYSTICK_GATEWAY_STALE_TIMEOUT_S,
    JOYSTICK_MANUAL_TRANSPORT,
    JOYSTICK_MAVROS_PUBLISH_ERROR_LIMIT,
    JOYSTICK_MAVROS_REQUIRE_SUBSCRIBER,
    JOYSTICK_NEUTRAL_PRESTREAM_S,
    JOYSTICK_PYMAVLINK_ENDPOINT,
    TOPIC_MAVROS_MANUAL_CONTROL,
)
from logging_setup import get_logger

log = get_logger("server.joystick.gateway")


@dataclass(frozen=True)
class ManualControlFrame:
    x: int
    y: int
    z: int
    r: int
    buttons: int = 0


NEUTRAL_FRAME = ManualControlFrame(x=0, y=0, z=500, r=0, buttons=0)


class ManualControlTransport(Protocol):
    name: str

    def is_healthy(self) -> bool:
        ...

    def health_reason(self) -> str:
        ...

    def send_frame(self, frame: ManualControlFrame) -> None:
        ...

    def shutdown(self) -> None:
        ...


def encode_manual_control(throttle: float, steering: float) -> ManualControlFrame:
    """Encode normalized operator demand to MAVLink MANUAL_CONTROL fields."""
    if not math.isfinite(throttle) or not math.isfinite(steering):
        raise ValueError("manual control values must be finite")
    throttle = max(-1.0, min(1.0, float(throttle)))
    steering = max(-1.0, min(1.0, float(steering)))
    return ManualControlFrame(
        x=0,
        y=max(-1000, min(1000, round(steering * 1000.0))),
        z=max(0, min(1000, round((throttle + 1.0) * 500.0))),
        r=0,
        buttons=0,
    )


class MavrosManualControlTransport:
    name = "mavros"

    def __init__(
        self,
        ros_node: Any,
        *,
        topic: str = TOPIC_MAVROS_MANUAL_CONTROL,
        require_subscriber: bool = JOYSTICK_MAVROS_REQUIRE_SUBSCRIBER,
    ) -> None:
        self._ros_node = ros_node
        self._topic = topic
        self._require_subscriber = require_subscriber
        self._publisher = None
        self._msg_type = None
        self._reason = ""
        self._publish_errors = 0
        self._publish_error_limit = int(JOYSTICK_MAVROS_PUBLISH_ERROR_LIMIT)
        if ros_node is None:
            self._reason = "ROS node not available"
            return
        try:
            from mavros_msgs.msg import ManualControl  # type: ignore
        except Exception as exc:
            self._reason = f"mavros_msgs/msg/ManualControl unavailable: {exc}"
            return
        try:
            self._publisher = ros_node.create_publisher(ManualControl, topic, 10)
            self._msg_type = ManualControl
        except Exception as exc:
            self._publisher = None
            self._reason = f"could not create {topic} publisher: {exc}"

    def is_healthy(self) -> bool:
        if self._publisher is None or self._msg_type is None:
            return False
        if self._publish_errors >= self._publish_error_limit:
            self._reason = (
                f"manual control publish failed {self._publish_errors} consecutive times"
            )
            return False
        if self._require_subscriber:
            try:
                if self._ros_node.count_subscribers(self._topic) < 1:
                    self._reason = f"no subscriber on {self._topic}"
                    return False
            except Exception as exc:
                self._reason = f"subscriber check failed: {exc}"
                return False
        self._reason = ""
        return True

    def health_reason(self) -> str:
        if self.is_healthy():
            return ""
        return self._reason or "MAVROS manual-control transport unhealthy"

    def send_frame(self, frame: ManualControlFrame) -> None:
        if self._publisher is None or self._msg_type is None:
            raise RuntimeError(self.health_reason())
        try:
            msg = self._msg_type()
            # mavros_msgs/msg/ManualControl declares x/y/z/r as float32; rclpy
            # rejects an int assigned to a float field. Cast here. buttons is
            # uint16 so it stays an int. Assignment is inside the try so a
            # serialization failure also trips _publish_errors / is_healthy().
            msg.x = float(frame.x)
            msg.y = float(frame.y)
            msg.z = float(frame.z)
            msg.r = float(frame.r)
            msg.buttons = int(frame.buttons)
            self._publisher.publish(msg)
        except Exception as exc:
            self._publish_errors += 1
            self._reason = f"publish failed: {exc}"
            raise
        self._publish_errors = 0
        self._reason = ""

    def shutdown(self) -> None:
        return None


class PymavlinkManualControlTransport:
    name = "pymavlink"

    def __init__(
        self,
        *,
        endpoint: str = JOYSTICK_PYMAVLINK_ENDPOINT,
        target_system: int = 1,
    ) -> None:
        self._endpoint = endpoint
        self._target_system = int(target_system)
        self._conn = None
        self._reason = ""
        try:
            from pymavlink import mavutil  # type: ignore
        except Exception as exc:
            self._reason = f"pymavlink unavailable: {exc}"
            return
        try:
            self._conn = mavutil.mavlink_connection(endpoint)
        except Exception as exc:
            self._conn = None
            self._reason = f"could not open {endpoint}: {exc}"

    def is_healthy(self) -> bool:
        return self._conn is not None

    def health_reason(self) -> str:
        if self.is_healthy():
            return ""
        return self._reason or "pymavlink transport unavailable"

    def send_frame(self, frame: ManualControlFrame) -> None:
        if self._conn is None:
            raise RuntimeError(self.health_reason())
        self._conn.mav.manual_control_send(
            self._target_system,
            int(frame.x),
            int(frame.y),
            int(frame.z),
            int(frame.r),
            int(frame.buttons),
        )

    def shutdown(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass


def build_manual_transport(ros_node: Any) -> ManualControlTransport:
    if JOYSTICK_MANUAL_TRANSPORT == "mavros":
        return MavrosManualControlTransport(ros_node)
    if JOYSTICK_MANUAL_TRANSPORT == "pymavlink":
        return PymavlinkManualControlTransport()
    raise RuntimeError(f"unsupported joystick transport: {JOYSTICK_MANUAL_TRANSPORT}")


class ManualControlGateway:
    """Fixed-rate sender with an independent stale-command watchdog.

    The worker uses a thread instead of the Socket.IO asyncio loop so the last
    nonzero command cannot be repeated forever when command handling stalls.
    """

    def __init__(
        self,
        transport: ManualControlTransport,
        *,
        rate_hz: float = JOYSTICK_GATEWAY_RATE_HZ,
        stale_timeout_s: float = JOYSTICK_GATEWAY_STALE_TIMEOUT_S,
    ) -> None:
        self._transport = transport
        self._period_s = 1.0 / float(rate_hz)
        self._stale_timeout_s = float(stale_timeout_s)
        self._lock = threading.Lock()
        self._active = False
        self._last_frame = NEUTRAL_FRAME
        self._last_receive_mono: float | None = None
        self._last_sent_frame = NEUTRAL_FRAME
        self._last_send_mono: float | None = None
        self._last_error = ""
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def transport_name(self) -> str:
        return self._transport.name

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="manual-control-gateway",
        )
        self._thread.start()

    def activate_neutral(self) -> None:
        with self._lock:
            self._active = True
            self._last_frame = NEUTRAL_FRAME
            self._last_receive_mono = time.monotonic()
        self.send_neutral(refresh=True)

    def deactivate_neutral(self) -> None:
        with self._lock:
            self._active = False
            self._last_frame = NEUTRAL_FRAME
            self._last_receive_mono = time.monotonic()
        self.flush_neutral(frames=3)

    def wait_neutral_barrier(self, duration_s: float = JOYSTICK_NEUTRAL_PRESTREAM_S) -> None:
        deadline = time.monotonic() + max(0.0, duration_s)
        while time.monotonic() < deadline:
            self.send_neutral(refresh=True)
            time.sleep(min(self._period_s, max(0.0, deadline - time.monotonic())))

    def accept_command(self, throttle: float, steering: float) -> ManualControlFrame:
        frame = encode_manual_control(throttle, steering)
        with self._lock:
            self._active = True
            self._last_frame = frame
            self._last_receive_mono = time.monotonic()
        return frame

    def send_neutral(self, *, refresh: bool = False) -> None:
        with self._lock:
            self._last_frame = NEUTRAL_FRAME
            if refresh:
                self._last_receive_mono = time.monotonic()
        self._send(NEUTRAL_FRAME)

    def flush_neutral(self, frames: int = 3) -> None:
        count = max(1, int(frames))
        for _ in range(count):
            self.send_neutral(refresh=True)
            time.sleep(self._period_s)

    def is_healthy(self) -> bool:
        return self._transport.is_healthy()

    def health_reason(self) -> str:
        reason = self._transport.health_reason()
        if reason:
            return reason
        return self._last_error

    def command_age_ms(self) -> float | None:
        with self._lock:
            ts = self._last_receive_mono
        if ts is None:
            return None
        return (time.monotonic() - ts) * 1000.0

    @property
    def is_neutral(self) -> bool:
        with self._lock:
            return self._last_frame == NEUTRAL_FRAME and self._last_sent_frame == NEUTRAL_FRAME

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            active = self._active
            last = self._last_frame
            sent = self._last_sent_frame
            receive_ts = self._last_receive_mono
            send_ts = self._last_send_mono
        now = time.monotonic()
        return {
            "transport": self.transport_name,
            "transport_healthy": self.is_healthy(),
            "transport_error": self.health_reason(),
            "gateway_active": active,
            "gateway_command_age_ms": (
                (now - receive_ts) * 1000.0 if receive_ts is not None else None
            ),
            "gateway_last_send_age_ms": (
                (now - send_ts) * 1000.0 if send_ts is not None else None
            ),
            "gateway_last_frame": {
                "x": last.x,
                "y": last.y,
                "z": last.z,
                "r": last.r,
                "buttons": last.buttons,
            },
            "gateway_last_sent_neutral": sent == NEUTRAL_FRAME,
        }

    def shutdown(self) -> None:
        self.deactivate_neutral()
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._transport.shutdown()

    def _run(self) -> None:
        while not self._stop_event.wait(self._period_s):
            now = time.monotonic()
            with self._lock:
                active = self._active
                frame = self._last_frame
                receive_ts = self._last_receive_mono
            if active and receive_ts is not None and now - receive_ts <= self._stale_timeout_s:
                self._send(frame)
            else:
                self._send(NEUTRAL_FRAME)

    def _send(self, frame: ManualControlFrame) -> None:
        try:
            self._transport.send_frame(frame)
        except Exception as exc:
            self._last_error = str(exc)
            return
        with self._lock:
            self._last_sent_frame = frame
            self._last_send_mono = time.monotonic()
            self._last_error = ""
