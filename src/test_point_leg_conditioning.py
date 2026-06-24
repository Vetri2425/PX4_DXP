#!/usr/bin/env python3
"""RPP conditioning tests for point-mode navigation legs."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from point_leg_trajectory import build_point_leg_path, interior_spacing_stats


def _make_path_pose(n: float, e: float, *, runtime_entry: bool = False) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = "local_ned"
    ps.pose.position.x = float(n)
    ps.pose.position.y = float(e)
    ps.pose.position.z = 0.0
    if runtime_entry:
        ps.pose.orientation.x = 1.0
        ps.pose.orientation.w = 0.0
    else:
        ps.pose.orientation.w = 1.0
    return ps


def _inject_path(node, poses: list[PoseStamped]) -> None:
    msg = Path()
    msg.header.frame_id = "local_ned"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.poses = poses
    node._path_cb(msg)


def test_two_point_runtime_entry_stays_segment():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    from rpp_controller_node import RPPControllerNode

    node = RPPControllerNode()
    try:
        _inject_path(
            node,
            [
                _make_path_pose(0.0, 0.0, runtime_entry=True),
                _make_path_pose(5.0, 0.0),
            ],
        )
        assert node._active_tracking_profile == "segment"
        assert len(node._path) == 2
        first = node._path[0].pose.position
        last = node._path[-1].pose.position
        assert abs(first.x) < 1e-6 and abs(first.y) < 1e-6
        assert abs(last.x - 5.0) < 1e-6 and abs(last.y) < 1e-6
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_densified_runtime_entry_uses_smooth_resample():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    from rpp_controller_node import RPPControllerNode

    node = RPPControllerNode()
    try:
        published = build_point_leg_path(
            (0.0, 0.0), (5.0, 0.0), mode="densified", spacing_m=0.08
        )
        poses = [
            _make_path_pose(n, e, runtime_entry=(i == 0))
            for i, (n, e) in enumerate(published)
        ]
        _inject_path(node, poses)
        assert node._active_tracking_profile == "smooth"
        assert len(node._path) > 2
        first = node._path[0].pose.position
        last = node._path[-1].pose.position
        assert abs(first.x) < 1e-6 and abs(first.y) < 1e-6
        assert abs(last.x - 5.0) < 1e-6 and abs(last.y) < 1e-6
        lo, hi = interior_spacing_stats(
            [(p.pose.position.x, p.pose.position.y) for p in node._path]
        )
        assert lo >= 0.07
        assert hi <= 0.09
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_short_densified_leg_stays_segment_two_point():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    from rpp_controller_node import RPPControllerNode

    node = RPPControllerNode()
    try:
        published = build_point_leg_path(
            (0.0, 0.0), (0.04, 0.0), mode="densified", spacing_m=0.08
        )
        poses = [
            _make_path_pose(n, e, runtime_entry=(i == 0))
            for i, (n, e) in enumerate(published)
        ]
        _inject_path(node, poses)
        assert node._active_tracking_profile == "segment"
        assert len(node._path) == 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    test_two_point_runtime_entry_stays_segment()
    test_densified_runtime_entry_uses_smooth_resample()
    test_short_densified_leg_stays_segment_two_point()
    print("PASS")


if __name__ == "__main__":
    main()