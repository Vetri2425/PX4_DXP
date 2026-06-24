#!/usr/bin/env python3
"""Behavioral tests for production PathEngine construction in path_publisher."""

from __future__ import annotations

import os
import sys
import types


def _install_ros_stubs() -> None:
    rclpy = sys.modules.get("rclpy", types.ModuleType("rclpy"))
    sys.modules["rclpy"] = rclpy
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    sys.modules["rclpy.node"] = rclpy_node
    rclpy_qos = types.ModuleType("rclpy.qos")

    class _Enum:
        BEST_EFFORT = RELIABLE = VOLATILE = TRANSIENT_LOCAL = KEEP_LAST = 1

    rclpy_qos.QoSProfile = lambda *a, **k: None
    rclpy_qos.ReliabilityPolicy = _Enum
    rclpy_qos.DurabilityPolicy = _Enum
    rclpy_qos.HistoryPolicy = _Enum
    sys.modules["rclpy.qos"] = rclpy_qos

    geom = types.ModuleType("geometry_msgs.msg")
    geom.PoseStamped = object
    sys.modules["geometry_msgs"] = types.ModuleType("geometry_msgs")
    sys.modules["geometry_msgs.msg"] = geom

    nav = types.ModuleType("nav_msgs.msg")
    nav.Path = object
    sys.modules["nav_msgs"] = types.ModuleType("nav_msgs")
    sys.modules["nav_msgs.msg"] = nav

    std = sys.modules.get("std_msgs.msg", types.ModuleType("std_msgs.msg"))
    std.Float32 = object
    sys.modules["std_msgs"] = sys.modules.get("std_msgs", types.ModuleType("std_msgs"))
    sys.modules["std_msgs"] = types.ModuleType("std_msgs")
    sys.modules["std_msgs.msg"] = std


_install_ros_stubs()
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import path_publisher_node as pp  # noqa: E402


class _FakePlan:
    merged_waypoints = [(0.0, 0.0), (1.0, 0.0)]
    spray_flags = [False, True]


class _FakePathEngine:
    calls: list[dict] = []

    def __init__(self, **kwargs):
        self.calls.append(dict(kwargs))

    def plan_file(self, *_args, **_kwargs):
        return _FakePlan()


def test_non_dxf_path_engine_construction_disables_compensation(monkeypatch, tmp_path):
    _FakePathEngine.calls = []
    monkeypatch.setattr(pp, "_PathEngine", _FakePathEngine)
    path = tmp_path / "mission.csv"
    path.write_text("north_m,east_m\n0,0\n1,0\n")

    pp._plan_file_with_engine(str(path))

    assert _FakePathEngine.calls == [{"compensate_spray": False}]


def test_dxf_path_engine_construction_disables_compensation(monkeypatch, tmp_path):
    _FakePathEngine.calls = []
    monkeypatch.setattr(pp, "_PathEngine", _FakePathEngine)
    path = tmp_path / "mission.dxf"
    path.write_text("0\nEOF\n")

    pp._plan_file_with_engine(str(path))

    assert len(_FakePathEngine.calls) == 1
    assert _FakePathEngine.calls[0]["compensate_spray"] is False
    assert "enable_path_extensions" in _FakePathEngine.calls[0]
