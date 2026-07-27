#!/usr/bin/env python3
"""G3.5 — precise-stop primitive tests (design §7). Pure, no rclpy.

Covers the feed-forward trigger distance + decel profile, the servo creep law,
the along-track residual sign convention, and the arrival check. The node glue
(`_precise_stop_ready`) is exercised in-env on the Jetson (G3.6).

Run:  python3 -m pytest -q test_precise_stop.py
"""

from __future__ import annotations

import math

import precise_stop as ps


# ---------------------------------------------------------------------------
# Feed-forward trigger distance:  d = v²/2a, floored at the coarse radius
# ---------------------------------------------------------------------------
def test_trigger_distance_basic():
    # v=0.35, a=0.30 → 0.1225/0.60 = 0.2042 m
    assert abs(ps.feedforward_trigger_distance(0.35, 0.30, 0.10) - 0.20417) < 1e-4


def test_trigger_distance_floored_at_min_radius():
    # Slow approach → v²/2a below the coarse radius → return the radius.
    assert ps.feedforward_trigger_distance(0.05, 0.30, 0.10) == 0.10


def test_trigger_distance_zero_decel_is_min_radius():
    assert ps.feedforward_trigger_distance(1.0, 0.0, 0.10) == 0.10


def test_trigger_distance_scales_with_v_squared():
    d1 = ps.feedforward_trigger_distance(0.4, 0.3, 0.0)
    d2 = ps.feedforward_trigger_distance(0.8, 0.3, 0.0)
    assert abs(d2 - 4.0 * d1) < 1e-9   # doubling v quadruples the distance


# ---------------------------------------------------------------------------
# Feed-forward decel profile:  v = √(2ad), capped, → 0 at the point
# ---------------------------------------------------------------------------
def test_profile_zero_at_point():
    assert ps.feedforward_brake_speed(0.0, 0.30, 1.0) == 0.0


def test_profile_reaches_cap_at_trigger_distance():
    # At exactly the trigger distance for the cap speed, the profile equals it.
    a, vcap = 0.30, 0.35
    d_trig = ps.feedforward_trigger_distance(vcap, a, 0.0)
    assert abs(ps.feedforward_brake_speed(d_trig, a, vcap) - vcap) < 1e-6


def test_profile_capped():
    # Far away the raw √(2ad) exceeds the cap → clamp to the cap.
    assert ps.feedforward_brake_speed(100.0, 0.30, 0.35) == 0.35


def test_profile_monotonic_decreasing_toward_point():
    a, cap = 0.30, 0.35
    prev = None
    for d in (0.20, 0.15, 0.10, 0.05, 0.02, 0.0):
        v = ps.feedforward_brake_speed(d, a, cap)
        if prev is not None:
            assert v <= prev + 1e-12
        prev = v


def test_profile_negative_remaining_is_zero():
    assert ps.feedforward_brake_speed(-0.05, 0.30, 1.0) == 0.0


# ---------------------------------------------------------------------------
# Along-track residual sign convention (NED, body-forward)
# ---------------------------------------------------------------------------
def test_residual_ahead_positive_facing_north():
    # Facing north (yaw=0). Target 0.5 m north → +0.5 ahead.
    assert abs(ps.along_track_residual(0.0, 0.0, 0.5, 0.0, 0.0) - 0.5) < 1e-9


def test_residual_behind_negative():
    # Target 0.3 m south of the nose while facing north → overshot, -0.3.
    assert abs(ps.along_track_residual(0.0, 0.0, -0.3, 0.0, 0.0) + 0.3) < 1e-9


def test_residual_ignores_cross_track():
    # Facing north; target dead abeam (0.4 m east) → 0 along-track.
    assert abs(ps.along_track_residual(0.0, 0.0, 0.0, 0.4, 0.0)) < 1e-9


def test_residual_facing_east():
    # yaw = +pi/2 is east in NED. Target 0.6 m east → +0.6 ahead.
    assert abs(ps.along_track_residual(0.0, 0.0, 0.0, 0.6, math.pi / 2.0) - 0.6) < 1e-9


# ---------------------------------------------------------------------------
# Servo creep law + arrival check
# ---------------------------------------------------------------------------
def test_servo_creeps_forward_when_ahead():
    assert ps.servo_speed(0.08, 0.05, 0.02) == 0.05


def test_servo_reverses_when_overshot():
    assert ps.servo_speed(-0.08, 0.05, 0.02) == -0.05


def test_servo_zero_within_tolerance():
    assert ps.servo_speed(0.01, 0.05, 0.02) == 0.0
    assert ps.servo_speed(-0.02, 0.05, 0.02) == 0.0   # exactly at tolerance


def test_reached_boundary():
    assert ps.reached(0.02, 0.02) is True
    assert ps.reached(0.021, 0.02) is False
    assert ps.reached(-0.019, 0.02) is True


# ---------------------------------------------------------------------------
# Integration sanity: profile brings a rover to rest at the coordinate
# ---------------------------------------------------------------------------
def test_profile_converges_to_point_in_sim():
    """Discrete-time integrate the profile as a velocity command; it should
    stop within tolerance of the point without overshooting past it."""
    a, cap, tol = 0.30, 0.35, 0.02
    n, e, yaw = 0.0, 0.0, 0.0
    tgt_n = 0.30  # 0.30 m ahead
    dt = 0.02     # 50 Hz
    for _ in range(2000):
        residual = ps.along_track_residual(n, e, tgt_n, 0.0, yaw)
        if ps.reached(residual, tol):
            break
        v = math.copysign(ps.feedforward_brake_speed(abs(residual), a, cap), residual)
        n += v * math.cos(yaw) * dt
        e += v * math.sin(yaw) * dt
    residual = ps.along_track_residual(n, e, tgt_n, 0.0, yaw)
    assert ps.reached(residual, tol), f"stopped at residual {residual:.4f} m"
    assert n <= tgt_n + tol, f"overshot: n={n:.4f} > target+tol"


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
