#!/usr/bin/env python3
"""Unit tests for Phase 3 spray flag propagation in RPP path conditioning."""

from __future__ import annotations

import os
import sys
import types


def _install_ros_stubs() -> None:
    rclpy = sys.modules.get("rclpy", types.ModuleType("rclpy"))
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
            self.header = types.SimpleNamespace(stamp=None, frame_id="")
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

    class _Path:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id="")
            self.poses = []

    nav = types.ModuleType("nav_msgs.msg")
    nav.Path = _Path
    sys.modules["nav_msgs"] = types.ModuleType("nav_msgs")
    sys.modules["nav_msgs.msg"] = nav

    std = types.ModuleType("std_msgs.msg")
    std.Bool = object
    std.Float32 = object
    std.Float32MultiArray = object
    std.MultiArrayDimension = object

    class _String:
        def __init__(self):
            self.data = ""

    std.String = _String
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std


try:
    import rclpy  # noqa: F401
except ImportError:
    pass
_install_ros_stubs()

sys.path.insert(0, os.path.dirname(__file__))
from rpp_controller_node import RPPControllerNode  # noqa: E402
from path_identity import path_geometry_fingerprint  # noqa: E402


class _CapturePub:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


def _pose(n, e, flag):
    position = types.SimpleNamespace(x=n, y=e, z=1.0 if flag else 0.0)
    orientation = types.SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)
    pose = types.SimpleNamespace(position=position, orientation=orientation)
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=None, frame_id="local_ned"),
        pose=pose,
    )


def _conditioned_points_flags(node):
    msg = node._conditioned_path_pub.msgs[-1]
    return (
        [(p.pose.position.x, p.pose.position.y) for p in msg.poses],
        [p.pose.position.z > 0.5 for p in msg.poses],
    )


def _publish_node_for_runs(runs, raw_points=None, raw_flags=None):
    node = object.__new__(RPPControllerNode)
    node._runs = runs
    node._path = []
    node._active_tracking_profile = "segment"
    node._conditioned_path_pub = _CapturePub()
    node._conditioned_path_identity_pub = _CapturePub()
    raw_points = raw_points or [(0.0, 0.0), (1.0, 0.0)]
    raw_flags = raw_flags or [False, True]
    node._raw_path_identity = {
        "mission_id": "m1",
        "path_fingerprint": path_geometry_fingerprint(raw_points, raw_flags),
        "configuration_revision": 7,
        "source": "raw_path",
    }
    node._last_raw_path_fingerprint = node._raw_path_identity["path_fingerprint"]
    node._conditioned_path_fingerprint = ""
    return node


def _run(points, flags, profile="segment"):
    poses = [_pose(n, e, flag) for (n, e), flag in zip(points, flags)]
    return {
        "poses": poses,
        "flags": list(flags),
        "profile": profile,
        "length": 1.0,
        "cum_s": [0.0 for _ in poses],
        "closed": False,
    }


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
    assert out_pts[0] == pts[0]
    assert out_pts[-1] == pts[-1]
    assert [(pt, flag) for pt, flag in zip(out_pts, out_flags)].count(((1.0, 0.0), True)) == 1
    assert [(pt, flag) for pt, flag in zip(out_pts, out_flags)].count(((1.0, 0.0), False)) == 1


def test_segment_spray_active_uses_conditioned_endpoint_and_rule():
    node = object.__new__(RPPControllerNode)
    node._path_done = False
    node._path = [object(), object(), object()]
    node._spray_flags = [True, True, False]

    assert node._segment_spray_active(0) is True
    assert node._segment_spray_active(1) is False


def test_resample_preserves_exact_mark_boundaries_with_duplicate_vertices():
    pts = [(-0.5, 0.0), (0.0, 0.0), (0.0, 0.0), (2.0, 0.0), (2.0, 0.0), (2.5, 0.0)]
    flags = [False, False, True, True, False, False]

    out_pts, out_flags = RPPControllerNode._resample_path(pts, 0.25, flags)
    transitions = [
        (a, b)
        for a, fa, b, fb in zip(out_pts, out_flags, out_pts[1:], out_flags[1:])
        if fa != fb
    ]

    assert ((0.0, 0.0), (0.0, 0.0)) in transitions
    assert ((2.0, 0.0), (2.0, 0.0)) in transitions


def test_simplify_preserves_flag_boundaries():
    pts = [(-0.5, 0.0), (0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.5, 0.0)]
    flags = [False, True, True, False, False]

    out_pts, out_flags = RPPControllerNode._simplify_path_for_profile(pts, flags)

    assert (0.0, 0.0) in out_pts
    assert (2.0, 0.0) in out_pts
    assert out_flags[out_pts.index((0.0, 0.0))] is True
    assert out_flags[out_pts.index((2.0, 0.0))] is False


def test_corner_smoothing_preserves_mark_boundary_vertices():
    node = object.__new__(RPPControllerNode)
    node.get_logger = lambda: types.SimpleNamespace(warn=lambda *a, **k: None)
    pts = [(0.0, -0.5), (0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.0, 0.5)]
    flags = [False, True, True, True, False]

    out_pts, out_flags = node._smooth_corners(pts, radius=0.1, arc_pts=4, flags=flags)

    assert out_pts[0] == pts[0]
    assert (0.0, 0.0) in out_pts
    assert (1.0, 0.0) in out_pts
    pairs = [(pt, flag) for pt, flag in zip(out_pts, out_flags)]
    assert ((0.0, 0.0), False) in pairs
    assert ((0.0, 0.0), True) in pairs
    assert ((1.0, 0.0), True) in pairs
    assert ((1.0, 0.0), False) in pairs


def test_publish_conditioned_path_preserves_cross_run_off_to_on_vertex():
    node = _publish_node_for_runs([
        _run([(0.0, 0.0), (1.0, 0.0)], [False, False]),
        _run([(1.0, 0.0), (2.0, 0.0)], [True, True]),
    ])

    node._publish_conditioned_path(stamp=None, frame_id="local_ned")

    points, flags = _conditioned_points_flags(node)
    assert list(zip(points, flags)) == [
        ((0.0, 0.0), False),
        ((1.0, 0.0), False),
        ((1.0, 0.0), True),
        ((2.0, 0.0), True),
    ]


def test_publish_conditioned_path_preserves_cross_run_on_to_off_vertex():
    node = _publish_node_for_runs([
        _run([(0.0, 0.0), (1.0, 0.0)], [True, True]),
        _run([(1.0, 0.0), (2.0, 0.0)], [False, False]),
    ], raw_flags=[True, False])

    node._publish_conditioned_path(stamp=None, frame_id="local_ned")

    points, flags = _conditioned_points_flags(node)
    assert list(zip(points, flags)) == [
        ((0.0, 0.0), True),
        ((1.0, 0.0), True),
        ((1.0, 0.0), False),
        ((2.0, 0.0), False),
    ]


def test_publish_conditioned_path_deduplicates_same_flag_same_profile_join():
    for flag in (False, True):
        node = _publish_node_for_runs([
            _run([(0.0, 0.0), (1.0, 0.0)], [flag, flag]),
            _run([(1.0, 0.0), (2.0, 0.0)], [flag, flag]),
        ], raw_flags=[flag, flag])

        node._publish_conditioned_path(stamp=None, frame_id="local_ned")

        points, flags = _conditioned_points_flags(node)
        assert points == [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
        assert flags == [flag, flag, flag]


def test_publish_conditioned_path_keeps_same_flag_profile_boundary():
    node = _publish_node_for_runs([
        _run([(0.0, 0.0), (1.0, 0.0)], [True, True], profile="segment"),
        _run([(1.0, 0.0), (1.5, 0.5)], [True, True], profile="smooth"),
    ], raw_flags=[True, True])

    node._publish_conditioned_path(stamp=None, frame_id="local_ned")

    points, flags = _conditioned_points_flags(node)
    assert list(zip(points, flags)).count(((1.0, 0.0), True)) == 2


def test_publish_conditioned_path_runtime_entry_off_then_mark_on():
    node = _publish_node_for_runs([
        _run([(-1.0, 0.0), (0.0, 0.0)], [False, False], profile="smooth"),
        _run([(0.0, 0.0), (1.0, 0.0)], [True, True], profile="segment"),
    ])

    node._publish_conditioned_path(stamp=None, frame_id="local_ned")

    points, flags = _conditioned_points_flags(node)
    assert ((0.0, 0.0), False) in list(zip(points, flags))
    assert ((0.0, 0.0), True) in list(zip(points, flags))


def test_publish_conditioned_path_multiple_runs_short_and_repeat_deterministic():
    runs = [
        _run([(0.0, 0.0), (0.01, 0.0)], [False, False], profile="segment"),
        _run([(0.01, 0.0), (0.02, 0.0)], [False, False], profile="segment"),
        _run([(0.02, 0.0), (0.02, 0.0), (0.5, 0.0)], [True, True, True], profile="smooth"),
    ]
    node = _publish_node_for_runs(runs)
    node._publish_conditioned_path(stamp=None, frame_id="local_ned")
    first = _conditioned_points_flags(node)

    node._publish_conditioned_path(stamp=None, frame_id="local_ned")
    second = _conditioned_points_flags(node)

    assert first == second
    assert ((0.02, 0.0), False) in list(zip(first[0], first[1]))
    assert ((0.02, 0.0), True) in list(zip(first[0], first[1]))


if __name__ == "__main__":
    test_resample_flags_use_segment_and_rule()
    test_smooth_corner_mixed_flags_force_arc_off()
    test_segment_spray_active_uses_conditioned_endpoint_and_rule()
    test_resample_preserves_exact_mark_boundaries_with_duplicate_vertices()
    test_simplify_preserves_flag_boundaries()
    test_corner_smoothing_preserves_mark_boundary_vertices()
    test_publish_conditioned_path_preserves_cross_run_off_to_on_vertex()
    test_publish_conditioned_path_preserves_cross_run_on_to_off_vertex()
    test_publish_conditioned_path_deduplicates_same_flag_same_profile_join()
    test_publish_conditioned_path_keeps_same_flag_profile_boundary()
    test_publish_conditioned_path_runtime_entry_off_then_mark_on()
    test_publish_conditioned_path_multiple_runs_short_and_repeat_deterministic()
    print("PASS")
