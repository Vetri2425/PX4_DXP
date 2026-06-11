#!/usr/bin/env python3
"""Final-segment goal-approach deceleration (no ROS required).

Regression guard for the "rover doesn't stop at point B" bug: a straight line
(segment profile) used to drive at full mission speed into its endpoint and
only zero velocity within xy_goal_tolerance, coasting past B. The fix mirrors
the smooth/arc profile's approach scaling. These tests pin the speed profile.
"""

import math
import unittest


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def approach_speed(dist_to_corner, max_v, max_decel, approach_v,
                   approach_scaling_dist):
    """Mirror of rpp_controller_node._control_segment_profile final-segment
    approach deceleration. Returns the commanded speed at dist_to_corner."""
    approach_d = max(approach_scaling_dist, (max_v * max_v) / (2.0 * max_decel) + 0.10)
    speed = max_v
    if dist_to_corner < approach_d:
        scale = clamp(dist_to_corner / approach_d, 0.0, 1.0)
        speed = min(speed, max(approach_v, max_v * scale))
    return speed, approach_d


class TestSegmentStop(unittest.TestCase):
    MAXV = 0.35
    DECEL = 0.5
    APPROACH_V = 0.1   # min_approach_linear_velocity default
    SCALE_D = 0.6

    def _spd(self, d):
        return approach_speed(d, self.MAXV, self.DECEL, self.APPROACH_V, self.SCALE_D)[0]

    def test_full_speed_far_from_goal(self):
        """Far from B the rover still cruises at max_v (no premature braking)."""
        self.assertAlmostEqual(self._spd(5.0), self.MAXV, places=6)

    def test_monotonic_deceleration(self):
        """Speed decreases monotonically as the rover nears B."""
        prev = None
        for d in [k * 0.02 for k in range(40, -1, -1)]:  # 0.80 → 0.0
            s = self._spd(d)
            if prev is not None:
                self.assertLessEqual(s, prev + 1e-9, msg=f"speed rose at d={d}")
            prev = s

    def test_arrives_slow_at_goal(self):
        """Near B the commanded speed is the low approach floor, not max_v."""
        self.assertAlmostEqual(self._spd(0.0), self.APPROACH_V, places=6)
        self.assertLessEqual(self._spd(0.02), self.APPROACH_V * 1.5)

    def test_braking_distance_covers_cruise(self):
        """approach_d must be long enough to brake from max_v (physics)."""
        for mv in [0.35, 0.8, 1.0]:
            _, approach_d = approach_speed(mv, mv, self.DECEL, self.APPROACH_V, self.SCALE_D)
            needed = (mv * mv) / (2.0 * self.DECEL)
            self.assertGreaterEqual(approach_d, needed,
                                    msg=f"max_v={mv}: runway {approach_d:.2f} < brake {needed:.2f}")

    def test_floor_above_p4_freeze(self):
        """Approach floor must stay above the firmware 0.01 m/s freeze so the
        rover keeps creeping to the goal instead of stalling short."""
        self.assertGreater(self.APPROACH_V, 0.01)


class CornerStopDwell:
    """Mirror of rpp_controller_node._corner_stop_satisfied (time injected)."""

    THRESH = 0.02   # segment_stop_speed_threshold
    DWELL = 0.30    # segment_stop_dwell_s
    MAX_HOLD = 2.0  # _CORNER_STOP_MAX_HOLD_S

    def __init__(self):
        self.entered = None
        self.low_since = None

    def step(self, now, speed):
        """speed=None models missing/stale velocity data."""
        if self.entered is None:
            self.entered = now
        if speed is not None and speed > self.THRESH:
            self.low_since = None
        else:
            if self.low_since is None:
                self.low_since = now
            elif now - self.low_since >= self.DWELL:
                return True
        if now - self.entered >= self.MAX_HOLD:
            return True
        return False


class TestCornerStopDwell(unittest.TestCase):
    """CORNER_STOP must hold until the rover is physically stopped — and must
    never deadlock if velocity data is missing or noisy."""

    def test_holds_while_moving(self):
        d = CornerStopDwell()
        t = 0.0
        for _ in range(20):  # 1 s of decaying-but-moving speed
            self.assertFalse(d.step(t, 0.09))
            t += 0.05

    def test_releases_after_dwell_at_rest(self):
        d = CornerStopDwell()
        t = 0.0
        released = None
        for _ in range(40):
            if d.step(t, 0.005):
                released = t
                break
            t += 0.05
        self.assertIsNotNone(released)
        self.assertGreaterEqual(released, d.DWELL - 0.06)  # not before dwell
        self.assertLess(released, 0.6)                     # promptly after

    def test_motion_blip_restarts_dwell(self):
        d = CornerStopDwell()
        t = 0.0
        # settle 0.2s, blip of motion, then settle again
        for _ in range(4):
            self.assertFalse(d.step(t, 0.005)); t += 0.05
        self.assertFalse(d.step(t, 0.05)); t += 0.05      # blip > threshold
        released = None
        for _ in range(20):
            if d.step(t, 0.005):
                released = t
                break
            t += 0.05
        # full dwell must elapse again after the blip (blip at t=0.20)
        self.assertGreaterEqual(released, 0.20 + d.DWELL - 0.06)

    def test_no_velocity_data_falls_back_to_dwell(self):
        d = CornerStopDwell()
        t = 0.0
        released = None
        for _ in range(40):
            if d.step(t, None):
                released = t
                break
            t += 0.05
        self.assertIsNotNone(released)
        self.assertLess(released, d.MAX_HOLD)  # dwell path, not the hard cap

    def test_hard_cap_prevents_deadlock(self):
        d = CornerStopDwell()
        t = 0.0
        released = None
        for _ in range(100):  # speed hovers just above threshold forever
            if d.step(t, 0.03):
                released = t
                break
            t += 0.05
        self.assertIsNotNone(released)
        self.assertAlmostEqual(released, d.MAX_HOLD, delta=0.06)


if __name__ == "__main__":
    unittest.main()
