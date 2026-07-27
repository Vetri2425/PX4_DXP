#!/usr/bin/env python3
"""Precise 2 cm along-track stop primitives (design §7, phase G3). Pure, no
rclpy — runs on the Mac, unit-tested in `test_precise_stop.py`.

Two open-loop/closed-loop options, both used ONLY for must-hit points under
point mode and ONLY behind `point_precise_stop_enabled`. The frozen corner-stop
path is never touched. The node (`rpp_controller_node._point_hold_tick`) is thin
glue: it reads its own pose/velocity, calls these functions for the numbers, and
publishes the resulting velocity setpoint.

  * **Option A — feed-forward decel (default).** Begin braking `d = v²/2a`
    before the point and command the kinematic decel profile `v = √(2·a·d)`
    (capped) so speed reaches zero *at* the coordinate. Open-loop, no new
    closed loop — the trigger distance replaces the fixed acceptance radius.
  * **Option B — low-speed servo (fallback).** After a coarse stop, creep at
    `creep_speed` toward the point with along-track sign until the residual is
    within `tolerance`, bounded by a wall-clock timeout so it can never wedge.

All distances in metres, speeds in m/s, decel in m/s². Every function is a pure
number-in/number-out mapping with no state.
"""

from __future__ import annotations

import math


def feedforward_trigger_distance(
    speed_mps: float, decel_m_s2: float, min_radius_m: float
) -> float:
    """Distance before the point at which to start braking so `v→0` at it.

    `d = v²/2a`, but never less than `min_radius_m` (the coarse acceptance
    radius) so a rover already crawling still engages the hold near the point.
    A non-positive decel disables the feed-forward extension → `min_radius_m`.
    """
    speed = max(0.0, float(speed_mps))
    decel = float(decel_m_s2)
    floor = max(0.0, float(min_radius_m))
    if decel <= 0.0:
        return floor
    return max(floor, (speed * speed) / (2.0 * decel))


def feedforward_brake_speed(
    remaining_m: float, decel_m_s2: float, speed_cap_mps: float
) -> float:
    """Kinematic decel-profile speed at `remaining_m` from the point.

    `v = √(2·a·d)`, capped at `speed_cap_mps`. Monotonically → 0 as `d → 0`, so
    commanding this forward speed brings the rover to rest exactly at the point.
    Returns 0 for a non-positive remaining distance or decel.
    """
    remaining = float(remaining_m)
    decel = float(decel_m_s2)
    cap = max(0.0, float(speed_cap_mps))
    if remaining <= 0.0 or decel <= 0.0:
        return 0.0
    v = math.sqrt(2.0 * decel * remaining)
    return min(v, cap)


def along_track_residual(
    pos_n: float, pos_e: float, tgt_n: float, tgt_e: float, yaw_ned: float
) -> float:
    """Signed distance from pose to target along body-forward (NED).

    Positive = the point is still ahead of the nose (drive forward to reach it);
    negative = overshot (reverse to reach it). Cross-track is ignored — the
    tracking controller already holds it to ~2 cm; this axis is the one the
    coarse brake leaves loose.
    """
    fwd_n, fwd_e = math.cos(yaw_ned), math.sin(yaw_ned)
    return (float(tgt_n) - float(pos_n)) * fwd_n + (float(tgt_e) - float(pos_e)) * fwd_e


def servo_speed(residual_m: float, creep_speed_mps: float, tolerance_m: float) -> float:
    """Servo creep command toward the point; 0 once within tolerance.

    Signed along body-forward (same convention as `along_track_residual`), so a
    positive residual creeps forward and a negative one reverses. Magnitude is
    the fixed `creep_speed_mps` — a bang-bang creep, deliberately simple; the
    tolerance band stops the chatter.
    """
    residual = float(residual_m)
    tol = max(0.0, float(tolerance_m))
    if abs(residual) <= tol:
        return 0.0
    return math.copysign(max(0.0, float(creep_speed_mps)), residual)


def reached(residual_m: float, tolerance_m: float) -> bool:
    """True once the along-track residual is within the arrival tolerance."""
    return abs(float(residual_m)) <= max(0.0, float(tolerance_m))
