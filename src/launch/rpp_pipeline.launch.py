#!/usr/bin/env python3
"""ROS2 launch file for the full RPP controller pipeline.

Brings up:
  1. twist_to_setpoint_node   — streams /mavros/setpoint_raw/local at 50 Hz
  2. rpp_controller_node      — computes /rpp/velocity_ned at 50 Hz
  3. spray_controller_node    — drives PX4 actuator-set AUX output from /spray/active
  4. xtrack_logger_node       — captures CSV for offline tuning analysis
  5. path_publisher_node      — optional requested test path publisher
  6. mission_runner_node      — drives OFFBOARD lifecycle (off by default)

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
  path_name    Which test path to publish when publish_test_path=true
                 Options: straight_5m, arc_quarter_1m5, lshape_2x2,
                          square_2x2, rectangle_3x2, circle_1m5
  auto_run     If true, mission_runner auto-switches to OFFBOARD + arms
                 (default: false — operator runs mission_runner manually)
  allow_legacy_mission_runner
               If true, permit mission_runner_node to own OFFBOARD lifecycle
               (default: false — server owns lifecycle in normal operation)
  publish_test_path
               If true, start path_publisher_node and publish path_name
               (default: false — server owns /path in normal operation)
  auto_origin  If true, offset path to start at rover's current position
                 instead of EKF origin (default: false)
  dry_run      If true, mission_runner skips arm/mode commands
                 (default: false)
  log_level    ROS2 log level for all nodes (default: info)

Examples
--------
  # SITL straight-line test, manual mission start:
  ros2 launch src/launch/rpp_pipeline.launch.py publish_test_path:=true path_name:=straight_5m

  # Hardware arc test, auto-run, debug logging:
  ros2 launch src/launch/rpp_pipeline.launch.py publish_test_path:=true \\
      path_name:=arc_quarter_1m5 auto_run:=true \\
      allow_legacy_mission_runner:=true log_level:=debug

  # Dry-run telemetry capture, no arming:
  ros2 launch src/launch/rpp_pipeline.launch.py publish_test_path:=true \\
      path_name:=lshape_2x2 auto_run:=true \\
      allow_legacy_mission_runner:=true dry_run:=true
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
    allow_legacy_mission_runner = (
        LaunchConfiguration("allow_legacy_mission_runner").perform(context).lower() == "true"
    )
    publish_test_path = LaunchConfiguration("publish_test_path").perform(context).lower() == "true"
    auto_origin = LaunchConfiguration("auto_origin").perform(context).lower() == "true"
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() == "true"
    log_level = LaunchConfiguration("log_level").perform(context)

    # RPP tuning overrides (forwarded to rpp_controller_node)
    rpp_params = {}
    for name in (
        "min_lookahead_dist", "max_lookahead_dist", "lookahead_time",
        "corner_smooth_radius_m", "use_feedforward_yaw_rate",
        "max_yaw_rate_body", "yaw_rate_feedback_gain",
        "max_linear_vel", "min_linear_vel", "mission_speed",
        "xtrack_lookahead_gain", "max_linear_accel",
        "tracking_profile", "segment_corner_threshold_deg",
        "segment_slowdown_dist", "segment_min_corner_speed",
        "segment_corner_acceptance_radius", "segment_heading_tolerance_deg",
        "segment_yaw_rate_gain", "segment_timeout_heading_tolerance_deg",
        "segment_pivot_release_max_deg", "segment_stop_speed_threshold",
        "segment_stop_yaw_rate_threshold", "segment_stop_dwell_s",
        "segment_brake_velocity_cap_m_s", "segment_align_settle_s",
        "segment_align_speed_threshold",
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

    spray_proc = ExecuteProcess(
        cmd=_node_cmd(
            os.path.join(src_dir, "spray_controller_node.py"),
            log_level,
        ),
        name="spray_controller",
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
            {
                "dry_run": "true" if dry_run else "false",
                "allow_legacy_lifecycle": "true" if allow_legacy_mission_runner else "false",
            },
        ),
        name="mission_runner",
        output="screen",
        emulate_tty=True,
    )

    actions = [
        # Phase 1: streamer + controller + logger come up together
        twist_proc,
        rpp_proc,
        spray_proc,
        xtrack_proc,
    ]

    if publish_test_path:
        # Optional test path injection. In normal server-driven operation, the
        # FastAPI bridge publishes /path directly and this process stays absent.
        actions.append(TimerAction(period=0.3, actions=[path_proc]))

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
        DeclareLaunchArgument("allow_legacy_mission_runner", default_value="false",
            description="If true, mission_runner_node may own OFFBOARD lifecycle"),
        DeclareLaunchArgument("publish_test_path", default_value="false",
            description="If true, start path_publisher_node and publish path_name"),
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
        DeclareLaunchArgument("corner_smooth_radius_m",              default_value="__unset__"),
        DeclareLaunchArgument("use_feedforward_yaw_rate",            default_value="__unset__"),
        DeclareLaunchArgument("max_yaw_rate_body",                   default_value="__unset__"),
        DeclareLaunchArgument("yaw_rate_feedback_gain",              default_value="__unset__"),
        DeclareLaunchArgument("max_linear_vel",                      default_value="__unset__"),
        DeclareLaunchArgument("min_linear_vel",                      default_value="__unset__"),
        DeclareLaunchArgument("mission_speed",                       default_value="__unset__"),
        DeclareLaunchArgument("xtrack_lookahead_gain",               default_value="__unset__"),
        DeclareLaunchArgument("max_linear_accel",                    default_value="__unset__"),
        DeclareLaunchArgument("tracking_profile",                    default_value="__unset__"),
        DeclareLaunchArgument("segment_corner_threshold_deg",         default_value="__unset__"),
        DeclareLaunchArgument("segment_slowdown_dist",               default_value="__unset__"),
        DeclareLaunchArgument("segment_min_corner_speed",             default_value="__unset__"),
        DeclareLaunchArgument("segment_corner_acceptance_radius",     default_value="__unset__"),
        DeclareLaunchArgument("segment_heading_tolerance_deg",        default_value="__unset__"),
        DeclareLaunchArgument("segment_yaw_rate_gain",                default_value="__unset__"),
        DeclareLaunchArgument("segment_timeout_heading_tolerance_deg", default_value="__unset__"),
        DeclareLaunchArgument("segment_pivot_release_max_deg",         default_value="__unset__"),
        DeclareLaunchArgument("segment_stop_speed_threshold",          default_value="__unset__"),
        DeclareLaunchArgument("segment_stop_yaw_rate_threshold",       default_value="__unset__"),
        DeclareLaunchArgument("segment_stop_dwell_s",                  default_value="__unset__"),
        DeclareLaunchArgument("segment_brake_velocity_cap_m_s",        default_value="__unset__"),
        DeclareLaunchArgument("segment_align_settle_s",                default_value="__unset__"),
        DeclareLaunchArgument("segment_align_speed_threshold",         default_value="__unset__"),
        OpaqueFunction(function=_build),
    ])
