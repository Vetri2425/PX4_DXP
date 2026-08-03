#!/usr/bin/env python3
"""D1 pivot-to-intercept + D7 debug cross-track (in-env, needs rclpy).

D1 (2026-08-03 estimation audit): the corner stop leaves the rover up to
segment_corner_acceptance_radius short of the vertex, and the old pivot target
was the next LEG DIRECTION, computed with no reference to the actual stop
position — so the release gate preserved the lateral offset (the measured
2.4 cm pivot walk). _pivot_intercept_heading aims at a point
pivot_intercept_dist_m ahead on the new leg FROM THE CURRENT POSITION instead.

Load-bearing invariant: with the rover exactly ON the leg the intercept
bearing equals the leg direction, so aligned pivots are unchanged — the fix
only acts when there is an offset to null.

D7: _debug_xtrack must report the real signed offset (the pivot walk was
invisible because hold/pivot debug published a literal 0.0).

Run:  python3 -m pytest -q test_pivot_intercept.py   (Jetson only — rclpy)
"""

import math
from types import SimpleNamespace

import rclpy

from test_smoke_rpp_controller import _make_path_pose, _CapturePub


def _pt(n, e):
    return SimpleNamespace(x=float(n), y=float(e))


def _build_node(enabled=True, d_int=0.35):
    from rpp_controller_node import RPPControllerNode

    node = RPPControllerNode()
    node.set_parameters([
        rclpy.parameter.Parameter(
            "pivot_to_intercept_enabled",
            rclpy.parameter.Parameter.Type.BOOL, bool(enabled)),
        rclpy.parameter.Parameter(
            "pivot_intercept_dist_m",
            rclpy.parameter.Parameter.Type.DOUBLE, float(d_int)),
    ])
    return node


def test_on_leg_equals_leg_heading_exactly():
    """Zero offset ⇒ byte-identical to the old behaviour."""
    rclpy.init()
    try:
        node = _build_node()
        a, b = _pt(0.0, 0.0), _pt(5.0, 0.0)          # due-north leg
        leg = math.atan2(b.y - a.y, b.x - a.x)
        got = node._pivot_intercept_heading(0.0, 0.0, a, b, leg)
        assert got == leg
        # Also anywhere along the leg, still exact.
        got = node._pivot_intercept_heading(2.0, 0.0, a, b, leg)
        assert abs(got - leg) < 1e-12
    finally:
        rclpy.shutdown()


def test_lateral_offset_aims_at_intercept():
    """5 cm right of the leg start ⇒ bearing rotated atan(0.05/0.35)≈8.1°
    back toward the line, correct sign."""
    rclpy.init()
    try:
        node = _build_node()
        a, b = _pt(0.0, 0.0), _pt(5.0, 0.0)
        leg = 0.0                                     # due north
        pos_n, pos_e = 0.0, 0.05                      # 5 cm east of the line
        got = node._pivot_intercept_heading(pos_n, pos_e, a, b, leg)
        expected = math.atan2(0.0 - pos_e, 0.35 - pos_n)   # aim at (0.35, 0)
        assert abs(got - expected) < 1e-12
        assert got < 0.0                              # turns back toward -e
        assert abs(math.degrees(got) + 8.13) < 0.05
    finally:
        rclpy.shutdown()


def test_stop_short_of_vertex_includes_along_track():
    """Stopped 4 cm short of the corner AND 3 cm off the new leg: intercept
    bearing must point from the actual stop position onto the leg."""
    rclpy.init()
    try:
        node = _build_node()
        # New leg heads due EAST from the vertex at (5, 0).
        b, c = _pt(5.0, 0.0), _pt(5.0, 4.0)
        leg = math.atan2(c.y - b.y, c.x - b.x)        # +90° (east)
        pos_n, pos_e = 5.0 - 0.03, 0.0 - 0.04         # short + left of vertex
        got = node._pivot_intercept_heading(pos_n, pos_e, b, c, leg)
        expected = math.atan2(0.35 - pos_e, 5.0 - pos_n)   # aim at (5, 0.35)
        assert abs(got - expected) < 1e-12
        assert got != leg
    finally:
        rclpy.shutdown()


def test_disabled_falls_back_to_leg_heading():
    rclpy.init()
    try:
        node = _build_node(enabled=False)
        a, b = _pt(0.0, 0.0), _pt(5.0, 0.0)
        got = node._pivot_intercept_heading(0.0, 0.05, a, b, 0.0)
        assert got == 0.0
    finally:
        rclpy.shutdown()


def test_degenerate_intercept_falls_back():
    """Rover essentially ON the clamped intercept (end of a short leg) ⇒
    bearing would be pose-noise; must fall back to the leg direction."""
    rclpy.init()
    try:
        node = _build_node()
        a, b = _pt(0.0, 0.0), _pt(0.30, 0.0)          # leg shorter than d_int
        leg = 0.0
        got = node._pivot_intercept_heading(0.29, 0.001, a, b, leg)
        assert got == leg
    finally:
        rclpy.shutdown()


def test_debug_xtrack_signed_value():
    """D7: real signed cross-track from the active segment, not 0.0."""
    from nav_msgs.msg import Path

    rclpy.init()
    try:
        node = _build_node()
        node._vel_pub = _CapturePub()
        node._yaw_rate_pub = _CapturePub()
        node._dbg_pub = _CapturePub()
        node._segment_dbg_pub = _CapturePub()
        node._conditioned_path_pub = _CapturePub()
        node._spray_active_pub = _CapturePub()
        path = Path()
        path.header.frame_id = "local_ned"
        path.header.stamp = node.get_clock().now().to_msg()
        path.poses = [_make_path_pose(0.0, 0.0), _make_path_pose(5.0, 0.0)]
        node._path_cb(path)
        node._segment_idx = 0
        got = node._debug_xtrack(1.0, 0.03)
        assert abs(got - 0.03) < 1e-9                 # +e side, signed
        got = node._debug_xtrack(1.0, -0.02)
        assert abs(got + 0.02) < 1e-9
    finally:
        rclpy.shutdown()
