#!/usr/bin/env python3
"""Segment final endpoint precise-stop tests.

Run on a ROS2-sourced host:
    python3 -m pytest -q src/test_segment_endpoint_precise_stop.py
"""

import rclpy
from rclpy.parameter import Parameter

from test_smoke_rpp_controller import _CapturePub, _make_path_pose


def _p(name, value):
    if isinstance(value, bool):
        return Parameter(name, Parameter.Type.BOOL, value)
    return Parameter(name, Parameter.Type.DOUBLE, float(value))


def _build_node(**params):
    from rpp_controller_node import RPPControllerNode

    node = RPPControllerNode()
    defaults = {
        "segment_precise_endpoint_stop_enabled": True,
        "segment_endpoint_arrival_tolerance_m": 0.02,
        "segment_endpoint_cross_tolerance_m": 0.02,
        "segment_endpoint_max_correction_m": 0.15,
        "segment_endpoint_precise_decel_m_s2": 0.35,
        "segment_endpoint_trigger_margin_m": 0.10,
        "segment_endpoint_creep_speed": 0.10,
        "segment_endpoint_precise_max_s": 8.0,
    }
    defaults.update(params)
    node.set_parameters([_p(k, v) for k, v in defaults.items()])
    for attr in ("_vel_pub", "_yaw_rate_pub", "_dbg_pub", "_segment_dbg_pub"):
        setattr(node, attr, _CapturePub())

    node._path = [_make_path_pose(0.0, 0.0), _make_path_pose(10.0, 0.0)]
    node._path_s = [0.0, 10.0]
    node._spray_flags = [True, True]
    node._runs = [{"poses": node._path, "flags": node._spray_flags, "length": 10.0}]
    node._run_idx = 0
    node._segment_idx = 0
    node._latest_yaw_rate_ned = 0.0
    node._latest_vel_time = node.get_clock().now()
    node._latest_vel_ned = (0.0, 0.0)
    return node


def test_default_off_is_inert():
    rclpy.init()
    try:
        node = _build_node(segment_precise_endpoint_stop_enabled=False)
        assert node._segment_endpoint_precise_stop_tick(9.95, 0.0, 0.0, 0.01, 0.05) is False
        assert node._vel_pub.last is None
    finally:
        rclpy.shutdown()


def test_far_before_endpoint_leaves_normal_tracking_untouched():
    rclpy.init()
    try:
        node = _build_node()
        node._latest_vel_ned = (1.0, 0.0)
        # v²/(2a)+margin ≈ 1.53 m, so 4 m remaining must not engage the overlay.
        assert node._segment_endpoint_precise_stop_tick(6.0, 0.0, 0.0, 0.01, 4.0) is False
        assert node._segment_endpoint_stop_active is False
        assert node._vel_pub.last is None
    finally:
        rclpy.shutdown()


def test_overshoot_commands_reverse_along_segment():
    rclpy.init()
    try:
        node = _build_node()
        node._latest_vel_ned = (0.3, 0.0)
        node._corner_stop_satisfied = lambda: False
        assert node._segment_endpoint_precise_stop_tick(10.50, 0.0, 0.0, 0.01, 0.50) is True
        vel = node._vel_pub.last
        assert vel is not None
        assert vel.vector.x < 0.0
        assert abs(vel.vector.y) < 1e-9
    finally:
        rclpy.shutdown()


def test_lateral_miss_commands_endpoint_vector_not_zero_along_crawl():
    rclpy.init()
    try:
        node = _build_node()
        node._latest_vel_ned = (0.0, 0.0)
        node._corner_stop_satisfied = lambda: False
        # Along residual is zero, but cross-track is 6 cm. The old along-only
        # sketch would command ~0. This must command east/west toward endpoint.
        assert node._segment_endpoint_precise_stop_tick(10.0, 0.06, 0.0, 0.01, 0.06) is True
        vel = node._vel_pub.last
        assert vel is not None
        assert abs(vel.vector.x) < 1e-9
        assert vel.vector.y < 0.0
    finally:
        rclpy.shutdown()


def test_within_along_cross_and_stopped_hands_to_completion():
    rclpy.init()
    try:
        node = _build_node()
        node._corner_stop_satisfied = lambda: True
        called = {}
        node._hold_at_completion = lambda *args: called.setdefault("done", True)
        assert node._segment_endpoint_precise_stop_tick(9.99, 0.01, 0.0, 0.01, 0.014) is True
        assert called.get("done") is True
        assert node._completion_stop_pending is True
        assert node._segment_endpoint_stop_active is False
    finally:
        rclpy.shutdown()


def test_lateral_miss_outside_envelope_brakes_without_diagonal_chase():
    rclpy.init()
    try:
        node = _build_node()
        node._latest_vel_ned = (0.30, 0.0)
        node._corner_stop_satisfied = lambda: False
        assert node._segment_endpoint_precise_stop_tick(10.0, 0.30, 0.0, 0.01, 0.30) is True
        vel = node._vel_pub.last
        assert vel is not None
        assert vel.vector.x < 0.0
        assert abs(vel.vector.y) < 1e-9
    finally:
        rclpy.shutdown()


def test_timeout_backstop_hands_to_completion_when_stopped():
    rclpy.init()
    try:
        node = _build_node(segment_endpoint_precise_max_s=0.0)
        node._corner_stop_satisfied = lambda: True
        called = {}
        node._hold_at_completion = lambda *args: called.setdefault("done", True)
        assert node._segment_endpoint_precise_stop_tick(9.90, 0.0, 0.0, 0.01, 0.10) is True
        assert called.get("done") is True
        assert node._completion_stop_pending is True
        assert node._segment_endpoint_stop_active is False
    finally:
        rclpy.shutdown()
