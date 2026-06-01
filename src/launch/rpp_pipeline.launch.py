#!/usr/bin/env python3
"""ROS2 launch file for the full RPP controller pipeline.

Brings up:
  1. twist_to_setpoint_node   — streams /mavros/setpoint_raw/local at 50 Hz
  2. rpp_controller_node      — computes /rpp/velocity_ned at 50 Hz
  3. xtrack_logger_node       — captures CSV for offline tuning analysis
  4. path_publisher_node      — publishes the requested test path
  5. mission_runner_node      — drives OFFBOARD lifecycle (off by default)

This launch file uses ExecuteProcess directly because the repo is not yet
packaged as a colcon ament_python package. When you create a package later,
swap ExecuteProcess for launch_ros.actions.Node.

Order matters
-------------
  - twist_to_setpoint starts FIRST and unconditionally streams zero velocity.
    This satisfies PX4's pre-stream requirement.
  - rpp_controller starts second; takes over the velocity stream once /path
    arrives.
  - mission_runner starts LAST (and only if `auto_run:=true`) so the operator
    has a chance to abort if startup looked wrong.

Launch arguments
----------------
  path_name    Which test path to publish (default: straight_5m)
                 Options: straight_5m, arc_quarter_1m5, lshape_2x2,
                          square_2x2, rectangle_3x2, circle_1m5
  auto_run     If true, mission_runner auto-switches to OFFBOARD + arms
                 (default: false — operator runs mission_runner manually)
  auto_origin  If true, offset path to start at rover's current position
                 instead of EKF origin (default: false)
  dry_run      If true, mission_runner skips arm/mode commands
                 (default: false)
  log_level    ROS2 log level for all nodes (default: info)

Examples
--------
  # SITL straight-line test, manual mission start:
  ros2 launch src/launch/rpp_pipeline.launch.py path_name:=straight_5m

  # Hardware arc test, auto-run, debug logging:
  ros2 launch src/launch/rpp_pipeline.launch.py path_name:=arc_quarter_1m5 \\
      auto_run:=true log_level:=debug

  # Dry-run telemetry capture, no arming:
  ros2 launch src/launch/rpp_pipeline.launch.py path_name:=lshape_2x2 \\
      auto_run:=true dry_run:=true
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration


def _node_cmd(script: str, log_level: str, params: dict | None = None) -> list[str]:
    """Build a python3 command line to run a node script with ROS args."""
    cmd = [sys.executable or "python3", script, "--ros-args", "--log-level", log_level]
    if params:
        for k, v in params.items():
            cmd.extend(["-p", f"{k}:={v}"])
    return cmd


def _build(context, *args, **kwargs):
    path_name = LaunchConfiguration("path_name").perform(context)
    auto_run = LaunchConfiguration("auto_run").perform(context).lower() == "true"
    auto_origin = LaunchConfiguration("auto_origin").perform(context).lower() == "true"
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() == "true"
    log_level = LaunchConfiguration("log_level").perform(context)

    # RPP tuning overrides (forwarded to rpp_controller_node)
    rpp_params = {}
    for name in (
        "min_lookahead_dist", "max_lookahead_dist", "lookahead_time",
        "regulated_linear_scaling_min_radius", "corner_smooth_radius_m",
        "use_feedforward_yaw_rate", "max_yaw_rate_body",
        "yaw_rate_feedback_gain", "max_linear_vel", "min_linear_vel",
    ):
        val = LaunchConfiguration(name).perform(context)
        if val != "__unset__":
            rpp_params[name] = val

    # Resolve script directory: this launch file lives at src/launch/, scripts at src/
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.dirname(launch_dir)

    twist_proc = ExecuteProcess(
        cmd=_node_cmd(
            os.path.join(src_dir, "twist_to_setpoint_node.py"),
            log_level,
        ),
        name="twist_to_setpoint",
        output="screen",
        emulate_tty=True,
    )

    rpp_proc = ExecuteProcess(
        cmd=_node_cmd(
            os.path.join(src_dir, "rpp_controller_node.py"),
            log_level,
            rpp_params if rpp_params else None,
        ),
        name="rpp_controller",
        output="screen",
        emulate_tty=True,
    )

    xtrack_proc = ExecuteProcess(
        cmd=_node_cmd(
            os.path.join(src_dir, "xtrack_logger_node.py"),
            log_level,
            {"path_name_hint": path_name},
        ),
        name="xtrack_logger",
        output="screen",
        emulate_tty=True,
    )

    path_proc = ExecuteProcess(
        cmd=_node_cmd(
            os.path.join(src_dir, "path_publisher_node.py"),
            log_level,
            {"path_name": path_name, "auto_origin": str(auto_origin).lower()},
        ),
        name="path_publisher",
        output="screen",
        emulate_tty=True,
    )

    mission_proc = ExecuteProcess(
        cmd=_node_cmd(
            os.path.join(src_dir, "mission_runner_node.py"),
            log_level,
            {"dry_run": "true" if dry_run else "false"},
        ),
        name="mission_runner",
        output="screen",
        emulate_tty=True,
    )

    actions = [
        # Phase 1: streamer + controller + logger come up together
        twist_proc,
        rpp_proc,
        xtrack_proc,
        # Phase 2: path publisher waits 2s so subscribers are ready
        TimerAction(period=0.3, actions=[path_proc]),
    ]

    if auto_run:
        # Phase 3: mission_runner waits 4s — path is up, RPP has started outputting
        actions.append(TimerAction(period=0.3, actions=[mission_proc]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("path_name", default_value="straight_5m",
            description="Test path name"),
        DeclareLaunchArgument("auto_run", default_value="false",
            description="If true, mission_runner auto-switches to OFFBOARD + arms"),
        DeclareLaunchArgument("dry_run", default_value="false",
            description="If true, mission_runner skips arm/mode commands"),
        DeclareLaunchArgument("auto_origin", default_value="false",
            description="If true, offset path to rover's current EKF position"),
        DeclareLaunchArgument("log_level", default_value="info",
            description="ROS2 log level (debug, info, warn, error)"),
        # RPP tuning overrides — pass any of these to override node defaults
        DeclareLaunchArgument("min_lookahead_dist",                  default_value="__unset__"),
        DeclareLaunchArgument("max_lookahead_dist",                  default_value="__unset__"),
        DeclareLaunchArgument("lookahead_time",                      default_value="__unset__"),
        DeclareLaunchArgument("regulated_linear_scaling_min_radius", default_value="1.0"),
        DeclareLaunchArgument("corner_smooth_radius_m",              default_value="__unset__"),
        DeclareLaunchArgument("use_feedforward_yaw_rate",            default_value="true"),
        DeclareLaunchArgument("max_yaw_rate_body",                   default_value="1.0"),
        DeclareLaunchArgument("yaw_rate_feedback_gain",              default_value="1.2"),
        DeclareLaunchArgument("max_linear_vel",                      default_value="__unset__"),
        DeclareLaunchArgument("min_linear_vel",                      default_value="__unset__"),
        OpaqueFunction(function=_build),
    ])
