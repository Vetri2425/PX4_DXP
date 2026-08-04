#!/usr/bin/env python3
"""Endpoint approach floor must stay BELOW the stop-confirmation threshold.

THE DEADLOCK (2026-08-04, bag stg_1e79599d, 2x2 square, 7 internal runs):
`_run_alignment_hold` refuses to release until `_corner_stop_satisfied()` sees
ground speed under `segment_stop_speed_threshold` (0.02 m/s), and its timeout
fires only on STALE velocity — fresh-but-moving never advances. The goal
approach ramp meanwhile commanded a floor of `segment_endpoint_approach_speed`
(0.03 m/s). 0.03 > 0.02, so the controller commanded a creep it simultaneously
refused to accept as stopped.

Offline replay through the real node: 1465/1465 ticks returned at the
alignment-hold line, `_control_segment_profile` was never entered, the segment
index never advanced, and the mission died at ~36 % coverage.

A straight line has ONE run so it never showed this; the square has seven.

Run:  python3 -m pytest -q test_endpoint_approach_clamp.py   (Jetson only)
"""

import rclpy


def _node(approach_v, stop_thresh):
    from rpp_controller_node import RPPControllerNode

    node = RPPControllerNode()
    node.set_parameters([
        rclpy.parameter.Parameter(
            "segment_endpoint_approach_speed",
            rclpy.parameter.Parameter.Type.DOUBLE, float(approach_v)),
        rclpy.parameter.Parameter(
            "segment_stop_speed_threshold",
            rclpy.parameter.Parameter.Type.DOUBLE, float(stop_thresh)),
    ])
    return node


def test_shipped_defaults_are_satisfiable():
    """The live pairing that deadlocked: 0.03 floor vs 0.02 threshold."""
    rclpy.init()
    try:
        node = _node(0.03, 0.02)
        got = node._endpoint_approach_speed()
        assert got < 0.02, got
        assert abs(got - 0.01) < 1e-12, got
    finally:
        rclpy.shutdown()


def test_invariant_holds_for_any_configuration():
    """No parameter pairing may command a floor at/above the threshold."""
    rclpy.init()
    try:
        for approach_v in (0.0, 0.005, 0.01, 0.02, 0.03, 0.10, 0.5):
            for stop_thresh in (0.01, 0.02, 0.05, 0.10):
                node = _node(approach_v, stop_thresh)
                got = node._endpoint_approach_speed()
                assert got < stop_thresh, (approach_v, stop_thresh, got)
                assert got <= approach_v + 1e-12, (approach_v, got)
                node.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_safe_configuration_is_left_alone():
    """Below the ceiling the configured value passes through untouched."""
    rclpy.init()
    try:
        node = _node(0.004, 0.02)          # ceiling is 0.010
        assert node._endpoint_approach_speed() == 0.004
    finally:
        rclpy.shutdown()


def test_clamp_warns_once_not_at_50_hz():
    """The misconfiguration must be visible, but not spam the log."""
    rclpy.init()
    try:
        node = _node(0.03, 0.02)
        seen = []
        node.get_logger().warn = lambda msg, **kw: seen.append(msg)
        for _ in range(200):
            node._endpoint_approach_speed()
        assert len(seen) == 1, len(seen)
        assert "could never be confirmed" in seen[0]
    finally:
        rclpy.shutdown()
