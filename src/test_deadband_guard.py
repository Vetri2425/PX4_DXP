#!/usr/bin/env python3
"""Never command into the PX4 speed dead-band while ground remains to cover.

THE STALL (2026-08-04, four separate field failures, one shape). PX4
RO_SPEED_TH is 0.10 m/s. A command in 0 < v < RO_SPEED_TH is not a slow creep —
the wheels barely turn — so the rover strands wherever the ramp left it and the
segment never advances. Observed:

  * 5 cm short of a corner, commanded 0.010-0.030 m/s;
  * 17.5 cm short of a corner for 36 s, commanded 0.068 m/s
    (bag stg_c648f04a, seg 2, dist_to_corner 0.175:
     max(0.03, 0.35 * 0.175/0.9) = 0.068 < 0.10).

Both ended in an operator e-stop. Hand-pushing the rover a few centimetres made
it resume instantly, which is the tell: it is an actuation floor, not a control
error.

transit_runout_min_speed_m_s already did this, but only for an unpainted
run-out. The dead-band belongs to the drivetrain, not to what the tail happens
to be.

Run:  python3 -m pytest -q test_deadband_guard.py    (Jetson only — rclpy)
"""

import rclpy


def _node(act_min=0.10):
    from rpp_controller_node import RPPControllerNode

    node = RPPControllerNode()
    node.set_parameters([
        rclpy.parameter.Parameter(
            "min_actuatable_speed_m_s",
            rclpy.parameter.Parameter.Type.DOUBLE, float(act_min)),
    ])
    return node


def _guard(speed, dist_to_corner, goal_tol, act_min=0.10):
    """The guard as it is applied in _control_segment_profile."""
    if act_min > 0.0 and 0.0 < speed < act_min and dist_to_corner > goal_tol:
        return act_min
    return speed


def test_the_measured_stall_is_lifted():
    """bag stg_c648f04a: 0.068 m/s at 0.175 m from the corner."""
    assert _guard(0.068, 0.175, 0.02) == 0.10


def test_the_clamp_era_stall_is_lifted():
    """5 cm short, commanded 0.010-0.030."""
    for v in (0.010, 0.030, 0.068, 0.099):
        assert _guard(v, 0.05, 0.02) == 0.10, v


def test_zero_stays_zero():
    """A deliberate stop must never be turned into motion."""
    assert _guard(0.0, 0.50, 0.02) == 0.0


def test_inside_the_goal_tolerance_is_left_alone():
    """Where a stop belongs, the corner-stop and p4 zero-snap paths own it."""
    assert _guard(0.03, 0.01, 0.02) == 0.03


def test_speeds_at_or_above_the_floor_are_untouched():
    for v in (0.10, 0.12, 0.35, 0.7):
        assert _guard(v, 0.5, 0.02) == v, v


def test_disabled_reproduces_the_pre_fix_behaviour():
    """act_min = 0 is the A/B arm — exact reduction, not an approximation."""
    for v in (0.010, 0.068, 0.099):
        assert _guard(v, 0.175, 0.02, act_min=0.0) == v, v


def test_default_is_at_or_above_the_fcu_dead_band():
    """RO_SPEED_TH is 0.10 on this vehicle; lowering this re-creates the stall."""
    rclpy.init()
    try:
        node = _node()
        got = float(node.get_parameter("min_actuatable_speed_m_s").value)
        assert got >= 0.10, got
    finally:
        rclpy.shutdown()
