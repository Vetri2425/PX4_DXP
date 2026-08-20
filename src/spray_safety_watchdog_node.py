#!/usr/bin/env python3
"""Independent, fail-closed watchdog for the physical spray actuator.

The main spray controller must continuously publish a fresh ON lease.  This
separate process has its own MAVROS command client and sends only OFF when the
lease is false, malformed, missing, or stale.  A controller crash/freeze thus
causes independent OFF attempts instead of relying on process restart cleanup.

This is a companion-computer safety layer, not proof of physical feedback.
The actuator driver/valve should still be normally closed and field-tested.
"""

from __future__ import annotations

import json
import math
import signal
import time
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from mavros_msgs.srv import CommandLong
from std_msgs.msg import String

from spray_safety_lease import (
    LEASE_TOPIC,
    LeaseValidationError,
    SprayLeaseMonitor,
    SpraySafetyLease,
    validate_lease,
)


MAV_CMD_DO_SET_ACTUATOR = 187
MAV_CMD_DO_SET_SERVO = 183


def _lease_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


class SpraySafetyWatchdogNode(Node):
    """Send OFF unless a separate controller continuously leases ON."""

    def __init__(self) -> None:
        super().__init__("spray_safety_watchdog")
        self.declare_parameter("lease_timeout_s", 0.35)
        self.declare_parameter("off_retry_hz", 2.0)
        # A safety edge gets a short high-rate OFF burst. This closes the old
        # async reassert race: an already-in-flight ON that lands after the
        # controller's OFF is followed by rapid OFF retries (bounded by MAVROS
        # service acknowledgement latency). Retries then settle to the low
        # background rate so idle operation does not flood MAVROS.
        self.declare_parameter("off_burst_hz", 20.0)
        self.declare_parameter("off_burst_duration_s", 1.5)
        self.declare_parameter("command_ack_timeout_s", 1.0)
        self.declare_parameter("command_service", "/mavros/cmd/command")

        # Startup fallback matches the production controller defaults.  Once a
        # valid lease arrives, its mapping becomes authoritative and remains
        # available for OFF even if a later lease is malformed or stops.
        self.declare_parameter("actuator_backend", "mavlink_actuator")
        self.declare_parameter("actuator_set_index", 1)
        self.declare_parameter("off_value", -1.0)
        self.declare_parameter("servo_instance", 1)
        self.declare_parameter("off_pwm_us", 0)

        timeout_s = float(self.get_parameter("lease_timeout_s").value)
        self._monitor = SprayLeaseMonitor(timeout_s=timeout_s)
        self._fallback = SpraySafetyLease(
            allow_on=False,
            command_seq=0,
            backend=str(self.get_parameter("actuator_backend").value),
            actuator_set_index=int(self.get_parameter("actuator_set_index").value),
            off_value=float(self.get_parameter("off_value").value),
            servo_instance=int(self.get_parameter("servo_instance").value),
            off_pwm_us=int(self.get_parameter("off_pwm_us").value),
        )
        # Validate the fallback through the monitor contract before this process
        # is allowed to supervise a physical output.
        validate_lease(self._fallback)

        self._group = ReentrantCallbackGroup()
        command_service = str(self.get_parameter("command_service").value)
        self._command_cli = self.create_client(
            CommandLong, command_service, callback_group=self._group
        )
        self.create_subscription(
            String,
            LEASE_TOPIC,
            self._lease_cb,
            _lease_qos(),
            callback_group=self._group,
        )
        self._status_pub = self.create_publisher(
            String, "/spray/safety_watchdog_status", _lease_qos()
        )

        self._inflight = None
        self._inflight_since_s: Optional[float] = None
        # The controller may lease ON only after this independent process has
        # received a successful MAVROS acknowledgement for a physical OFF.
        # Service discovery alone does not prove PX4 accepts the command.
        self._off_confirmed = False
        self._next_off_s = 0.0
        self._off_burst_until_s = 0.0
        self._last_reason: Optional[str] = None
        self._last_status_s = 0.0
        self._shutdown = False
        self._timer = self.create_timer(0.05, self._tick)

        if not self._command_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"{command_service} not ready; watchdog will keep retrying OFF"
            )
        self.get_logger().info(
            f"spray safety watchdog started: lease timeout={timeout_s:.3f}s"
        )
        self._tick()  # startup is fail-closed before any controller lease

    def _lease_cb(self, msg: String) -> None:
        now_s = time.monotonic()
        try:
            self._monitor.observe(msg.data, now_s)
        except (LeaseValidationError, ValueError) as exc:
            reason = f"invalid controller lease: {exc}"
            self._monitor.invalidate(reason)
            self._next_off_s = 0.0
            self.get_logger().error(reason, throttle_duration_sec=1.0)

    def _active_mapping(self) -> SpraySafetyLease:
        return self._monitor.last_lease or self._fallback

    def _build_off_request(self) -> CommandLong.Request:
        mapping = self._active_mapping()
        req = CommandLong.Request()
        req.broadcast = False
        req.confirmation = 0
        if mapping.backend == "mavlink_servo_pwm":
            req.command = MAV_CMD_DO_SET_SERVO
            req.param1 = float(mapping.servo_instance)
            req.param2 = float(mapping.off_pwm_us)
            req.param3 = req.param4 = req.param5 = req.param6 = req.param7 = 0.0
            return req

        req.command = MAV_CMD_DO_SET_ACTUATOR
        params = [math.nan] * 6
        params[mapping.actuator_set_index - 1] = mapping.off_value
        req.param1, req.param2, req.param3 = params[0], params[1], params[2]
        req.param4, req.param5, req.param6 = params[3], params[4], params[5]
        req.param7 = 0.0
        return req

    def _tick(self) -> None:
        now_s = time.monotonic()
        ack_timeout_s = max(
            0.1, float(self.get_parameter("command_ack_timeout_s").value)
        )
        if (
            self._inflight is not None
            and self._inflight_since_s is not None
            and now_s - self._inflight_since_s > ack_timeout_s
        ):
            self.get_logger().error(
                "spray watchdog OFF acknowledgement timed out; retrying",
                throttle_duration_sec=1.0,
            )
            try:
                self._inflight.cancel()
            except Exception:
                pass
            self._inflight = None
            self._inflight_since_s = None
            self._off_confirmed = False
            self._next_off_s = 0.0

        reason = "service shutdown" if self._shutdown else self._monitor.off_reason(now_s)
        if reason != self._last_reason:
            if reason is None:
                self.get_logger().info("fresh spray ON lease received; OFF guard armed")
            else:
                self.get_logger().warn(f"spray fail-closed OFF: {reason}")
                self._next_off_s = 0.0
                self._off_burst_until_s = now_s + max(
                    0.0, float(self.get_parameter("off_burst_duration_s").value)
                )
            self._last_reason = reason

        if reason is not None and self._inflight is None and now_s >= self._next_off_s:
            rate_param = "off_burst_hz" if now_s < self._off_burst_until_s else "off_retry_hz"
            retry_hz = max(0.2, float(self.get_parameter(rate_param).value))
            self._next_off_s = now_s + (1.0 / retry_hz)
            self._dispatch_off(reason)

        if now_s - self._last_status_s >= 0.5:
            self._publish_status(now_s, reason)
            self._last_status_s = now_s

    def _dispatch_off(self, reason: str) -> None:
        if not self._command_cli.service_is_ready():
            self._off_confirmed = False
            self.get_logger().error(
                f"cannot send watchdog OFF ({reason}): MAVROS command service unavailable",
                throttle_duration_sec=1.0,
            )
            return
        future = self._command_cli.call_async(self._build_off_request())
        self._inflight = future
        self._inflight_since_s = time.monotonic()
        future.add_done_callback(self._off_done)

    def _off_done(self, future) -> None:
        if future is not self._inflight:
            return
        self._inflight = None
        self._inflight_since_s = None
        try:
            response = future.result()
        except Exception as exc:
            self._off_confirmed = False
            self.get_logger().error(f"spray watchdog OFF call failed: {exc}")
            self._next_off_s = 0.0
            return
        if not bool(getattr(response, "success", False)):
            self._off_confirmed = False
            self.get_logger().error(
                f"spray watchdog OFF rejected: result={getattr(response, 'result', None)}"
            )
            self._next_off_s = 0.0
            return
        self._off_confirmed = True

    def _publish_status(self, now_s: float, reason: Optional[str]) -> None:
        lease_age_s = None
        if self._monitor.last_receive_s is not None:
            lease_age_s = max(0.0, now_s - self._monitor.last_receive_s)
        msg = String()
        command_service_ready = self._command_cli.service_is_ready()
        msg.data = json.dumps(
            {
                "allow_on": reason is None,
                "watchdog_alive": True,
                "command_service_ready": command_service_ready,
                "off_authority_ready": bool(
                    command_service_ready and self._off_confirmed
                ),
                "off_reason": reason or "",
                "lease_age_s": lease_age_s,
                "off_inflight": self._inflight is not None,
                "backend": self._active_mapping().backend,
            },
            separators=(",", ":"),
            allow_nan=False,
        )
        self._status_pub.publish(msg)

    def shutdown_off(self) -> None:
        self._shutdown = True
        self._monitor.invalidate("service shutdown")
        self._next_off_s = 0.0
        self._tick()
        spin_once = getattr(rclpy, "spin_once", None)
        if spin_once is None:
            return
        deadline = time.monotonic() + 0.75
        while self._inflight is not None and time.monotonic() < deadline:
            spin_once(self, timeout_sec=0.05)


def main() -> None:
    rclpy.init()
    node: Optional[SpraySafetyWatchdogNode] = None
    try:
        node = SpraySafetyWatchdogNode()

        def _signal_handler(signum, frame):
            raise KeyboardInterrupt

        signal.signal(signal.SIGINT, _signal_handler)
        signal.signal(signal.SIGTERM, _signal_handler)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.shutdown_off()
            except Exception as exc:
                node.get_logger().error(f"watchdog shutdown OFF failed: {exc}")
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
