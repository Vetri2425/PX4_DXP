#!/usr/bin/env python3
"""Spray actuator controller for PX4 AUX outputs via MAVROS CommandLong.

Subscribes to /spray/active (desired MARK state from RPP), applies debounce
and safety gates, then commands MAV_CMD_DO_SET_ACTUATOR. The controller only
drives an already-configured PX4 actuator set output; QGC remains the source
of truth for AUX pin/function/PWM limits.
"""

from __future__ import annotations

import math
import signal
from typing import Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Bool


MAV_CMD_DO_SET_ACTUATOR = 187


def _best_effort_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def _state_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class SprayControllerNode(Node):
    """Edge-triggered spray servo/solenoid controller."""

    def __init__(self) -> None:
        super().__init__("spray_controller")

        self.declare_parameter("actuator_set_index", 1)
        self.declare_parameter("on_value", 1.0)
        self.declare_parameter("off_value", -1.0)
        self.declare_parameter("debounce_samples", 3)
        self.declare_parameter("reassert_hz", 2.0)
        self.declare_parameter("require_offboard", True)
        self.declare_parameter("active_timeout_s", 0.5)
        self.declare_parameter("command_service", "/mavros/cmd/command")

        self._group = ReentrantCallbackGroup()
        self._desired_raw = False
        self._candidate: Optional[bool] = None
        self._candidate_count = 0
        self._desired_debounced = False
        self._commanded = False
        self._last_active_time = None
        self._armed = False
        self._mode = "UNKNOWN"
        self._service_ready = False

        command_service = str(self.get_parameter("command_service").value)
        self._command_cli = self.create_client(
            CommandLong,
            command_service,
            callback_group=self._group,
        )

        self._state_pub = self.create_publisher(Bool, "/spray/state", _best_effort_qos())
        self.create_subscription(
            Bool,
            "/spray/active",
            self._active_cb,
            _best_effort_qos(),
            callback_group=self._group,
        )
        self.create_subscription(
            State,
            "/mavros/state",
            self._state_cb,
            _state_qos(),
            callback_group=self._group,
        )

        self._watchdog_timer = self.create_timer(0.02, self._watchdog_tick)
        reassert_hz = max(0.0, float(self.get_parameter("reassert_hz").value))
        self._reassert_timer = None
        if reassert_hz > 0.0:
            self._reassert_timer = self.create_timer(1.0 / reassert_hz, self._reassert_tick)

        if self._command_cli.wait_for_service(timeout_sec=2.0):
            self._service_ready = True
        else:
            self.get_logger().warn(
                f"{command_service} not ready; spray commands idle until service appears"
            )
            self.create_timer(1.0, self._service_probe_tick)

        self._publish_state(False)
        self.get_logger().info("spray_controller started")

    def _service_probe_tick(self) -> None:
        if self._service_ready:
            return
        if self._command_cli.service_is_ready():
            self._service_ready = True
            self.get_logger().info("spray command service is ready")

    def _state_cb(self, msg: State) -> None:
        prev_safe = self._safety_allows_on()
        self._armed = bool(msg.armed)
        self._mode = str(msg.mode)
        now_safe = self._safety_allows_on()
        if prev_safe and not now_safe:
            self._force_off("FCU left armed/OFFBOARD safe state")
        elif not prev_safe and now_safe and self._desired_debounced:
            self._commit_desired_state()

    def _active_cb(self, msg: Bool) -> None:
        self._last_active_time = self.get_clock().now()
        self._desired_raw = bool(msg.data)
        self._apply_debounce()

    def _apply_debounce(self) -> None:
        if self._candidate is None or self._candidate != self._desired_raw:
            self._candidate = self._desired_raw
            self._candidate_count = 1
        else:
            self._candidate_count += 1

        debounce_samples = max(0, int(self.get_parameter("debounce_samples").value))
        if self._candidate_count < max(1, debounce_samples):
            return
        if self._desired_debounced == self._candidate:
            if self._desired_debounced != self._commanded:
                self._commit_desired_state()
            return

        self._desired_debounced = bool(self._candidate)
        self._commit_desired_state()

    def _watchdog_tick(self) -> None:
        timeout_s = max(0.0, float(self.get_parameter("active_timeout_s").value))
        if self._last_active_time is not None:
            age_s = (self.get_clock().now() - self._last_active_time).nanoseconds * 1e-9
            if age_s > timeout_s:
                self._desired_raw = False
                self._desired_debounced = False
                self._candidate = False
                self._candidate_count = 0
                self._force_off(f"/spray/active stale ({age_s:.2f}s)")
                return
        if not self._safety_allows_on():
            self._force_off("safety gate")

    def _reassert_tick(self) -> None:
        if self._commanded and self._safety_allows_on():
            self._send_command(True, reason="reassert")

    def _commit_desired_state(self) -> None:
        if self._desired_debounced and not self._safety_allows_on():
            self._force_off("desired ON blocked by safety gate")
            return
        if self._desired_debounced != self._commanded:
            self._send_command(self._desired_debounced, reason="edge")

    def _safety_allows_on(self) -> bool:
        if not self._armed:
            return False
        require_offboard = bool(self.get_parameter("require_offboard").value)
        if require_offboard and self._mode != "OFFBOARD":
            return False
        return True

    def _force_off(self, reason: str) -> None:
        if self._commanded:
            self.get_logger().warn(f"forcing spray OFF: {reason}", throttle_duration_sec=1.0)
            self._send_command(False, reason="failsafe")
        else:
            self._publish_state(False)

    def _send_command(self, on: bool, reason: str) -> None:
        if on and not self._safety_allows_on():
            on = False
        if not self._service_ready:
            self.get_logger().warn(
                "spray command service not ready; command suppressed",
                throttle_duration_sec=1.0,
            )
            if not on:
                self._commanded = False
                self._publish_state(False)
            return

        req = self._build_command_request(on)
        future = self._command_cli.call_async(req)
        future.add_done_callback(lambda fut, requested=on, why=reason: self._command_done(fut, requested, why))
        self._commanded = bool(on)
        self._publish_state(self._commanded)

    def _build_command_request(self, on: bool) -> CommandLong.Request:
        set_index = int(self.get_parameter("actuator_set_index").value)
        if set_index < 1 or set_index > 6:
            self.get_logger().warn(
                f"actuator_set_index={set_index} out of range 1..6; using 1",
                throttle_duration_sec=5.0,
            )
            set_index = 1
        value = (
            float(self.get_parameter("on_value").value)
            if on else
            float(self.get_parameter("off_value").value)
        )

        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_DO_SET_ACTUATOR
        req.confirmation = 0
        params = [math.nan] * 6
        params[set_index - 1] = value
        req.param1, req.param2, req.param3 = params[0], params[1], params[2]
        req.param4, req.param5, req.param6 = params[3], params[4], params[5]
        req.param7 = 0.0
        return req

    def _command_done(self, future, requested: bool, reason: str) -> None:
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(f"spray command {reason} failed: {exc}")
            return
        success = bool(getattr(resp, "success", False))
        result = getattr(resp, "result", None)
        if not success:
            self.get_logger().warn(
                f"spray command {reason} rejected: requested={requested} result={result}"
            )

    def _publish_state(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self._state_pub.publish(msg)

    def shutdown_off(self) -> None:
        self._desired_raw = False
        self._desired_debounced = False
        self._send_command(False, reason="shutdown")


def main() -> None:
    rclpy.init()
    node: SprayControllerNode | None = None
    try:
        node = SprayControllerNode()

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
            except Exception:
                pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
