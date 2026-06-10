#!/usr/bin/env python3
"""Unit tests for Phase 3 spray flag propagation in RPP path conditioning."""

from __future__ import annotations

import os
import sys
import types


def _install_ros_stubs() -> None:
    if "rclpy" in sys.modules:
        return

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.try_shutdown = lambda *a, **k: None
    sys.modules["rclpy"] = rclpy

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    sys.modules["rclpy.node"] = rclpy_node

    rclpy_time = types.ModuleType("rclpy.time")
    rclpy_time.Time = object
    sys.modules["rclpy.time"] = rclpy_time

    rclpy_qos = types.ModuleType("rclpy.qos")
    class _Enum:
        BEST_EFFORT = RELIABLE = VOLATILE = TRANSIENT_LOCAL = KEEP_LAST = 1
    rclpy_qos.QoSProfile = lambda *a, **k: None
    rclpy_qos.ReliabilityPolicy = _Enum
    rclpy_qos.DurabilityPolicy = _Enum
    rclpy_qos.HistoryPolicy = _Enum
    sys.modules["rclpy.qos"] = rclpy_qos

    class _Point:
        x = 0.0
        y = 0.0
        z = 0.0

    class _Pose:
        def __init__(self):
            self.position = _Point()
            self.orientation = types.SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)

    class _PoseStamped:
        def __init__(self):
            self.pose = _Pose()

    class _Vector3Stamped:
        pass

    geom = types.ModuleType("geometry_msgs.msg")
    geom.PoseStamped = _PoseStamped
    geom.Vector3Stamped = _Vector3Stamped
    geom.TwistStamped = object
    sys.modules["geometry_msgs"] = types.ModuleType("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = geom

    mavros = types.ModuleType("mavros_msgs.msg")
    mavros.GPSRAW = object
    sys.modules["mavros_msgs"] = types.ModuleType("mavros_msgs")
    sys.modules["mavros_msgs.msg"] = mavros

    nav = types.ModuleType("nav_msgs.msg")
    nav.Path = object
    sys.modules["nav_msgs"] = types.ModuleType("nav_msgs")
    sys.modules["nav_msgs.msg"] = nav

    std = types.ModuleType("std_msgs.msg")
    std.Bool = object
    std.Float32 = object
    std.Float32MultiArray = object
    std.MultiArrayDimension = object
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std


try:
    import rclpy  # noqa: F401
except ImportError:
    _install_ros_stubs()

sys.path.insert(0, os.path.dirname(__file__))
from rpp_controller_node import RPPControllerNode  # noqa: E402


def test_resample_flags_use_segment_and_rule():
    pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    flags = [True, True, False]

    out_pts, out_flags = RPPControllerNode._resample_path(pts, 0.5, flags)

    assert out_pts[0] == pts[0]
    assert out_pts[-1] == pts[-1]
    assert out_flags[0] is True
    assert out_flags[-1] is False
    assert any(out_flags[1:-1])
    assert any(flag is False for flag in out_flags[1:-1])


def test_smooth_corner_mixed_flags_force_arc_off():
    node = object.__new__(RPPControllerNode)
    pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    flags = [True, True, False]

    out_pts, out_flags = node._smooth_corners(pts, radius=0.2, arc_pts=4, flags=flags)

    assert len(out_pts) == len(out_flags)
    assert out_flags[0] is True
    assert out_flags[-1] is False
    assert all(flag is False for flag in out_flags[1:-1])


def test_segment_spray_active_uses_conditioned_endpoint_and_rule():
    node = object.__new__(RPPControllerNode)
    node._path_done = False
    node._path = [object(), object(), object()]
    node._spray_flags = [True, True, False]

    assert node._segment_spray_active(0) is True
    assert node._segment_spray_active(1) is False


if __name__ == "__main__":
    test_resample_flags_use_segment_and_rule()
    test_smooth_corner_mixed_flags_force_arc_off()
    test_segment_spray_active_uses_conditioned_endpoint_and_rule()
    print("PASS")
