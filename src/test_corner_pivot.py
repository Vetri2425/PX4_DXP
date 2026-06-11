#!/usr/bin/env python3
"""Corner-pivot actuation invariants (no ROS required).

Regression guard for the square-corner deadlock fix (bag
square_20260611_170539): PX4 rover_differential derives heading from the
velocity-vector bearing and FREEZES heading when |v| < 0.01 m/s. So a corner
pivot must be commanded as a small velocity VECTOR pointing at the exit
heading — never as zero velocity + yaw_rate (the firmware discards yaw_rate).

These tests pin the pure-math contract of that command so a future edit can't
silently reintroduce the zero-velocity pivot.
"""

import math
import unittest


# Firmware constant: DifferentialVelControl ZERO_VEL_THRESHOLD (m/s). Below
# this the rover holds heading instead of turning.
FW_ZERO_VEL_THRESHOLD = 0.01


# Forward-cone clamp (rpp_controller_node._CORNER_MAX_BEARING_OFFSET_RAD).
MAX_BEARING_OFFSET = math.radians(75.0)


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def corner_pivot_velocity(yaw_ned: float, target_heading_ned: float,
                          min_corner_speed: float):
    """Mirror of rpp_controller_node._corner_pivot_velocity.

    Aims at the exit heading but clamps the commanded bearing to ±75° of the
    current nose so PX4's reverse-detection (fwd_component<0) cannot flip the
    turn. Returns (v_n, v_e) in NED.
    """
    corner_speed = max(0.05, min_corner_speed)
    heading_err = wrap_pi(target_heading_ned - yaw_ned)
    step = max(-MAX_BEARING_OFFSET, min(MAX_BEARING_OFFSET, heading_err))
    cmd_bearing = yaw_ned + step
    return corner_speed * math.cos(cmd_bearing), corner_speed * math.sin(cmd_bearing)


def fwd_component(v_n, v_e, yaw_ned):
    """PX4 DifferentialVelControl forward projection onto the nose."""
    return v_n * math.cos(yaw_ned) + v_e * math.sin(yaw_ned)


class TestCornerPivot(unittest.TestCase):
    def test_points_at_exit_when_within_cone(self):
        """If the exit is within ±75° of the nose, command it exactly."""
        yaw = 0.3
        for target in [yaw + 0.2, yaw - 0.5, yaw + 1.0]:  # all <75°
            v_n, v_e = corner_pivot_velocity(yaw, target, 0.08)
            bearing = math.atan2(v_e, v_n)
            self.assertAlmostEqual(wrap_pi(bearing - target), 0.0, places=6)

    def test_forward_component_never_negative(self):
        """The reverse-flip trigger: a 90°+ corner must NOT make fwd<0.

        This is the exact failure in bag square_cornerfix_20260611_174508 —
        a +90° corner drove fwd_component negative, the firmware reversed and
        spot-turned to -90°, deadlocking at the 180° singularity.
        """
        for yaw in [-0.1, 0.0, 0.5, 1.5, -2.0, 3.0]:
            for target in [yaw + math.pi / 2, yaw - math.pi / 2,
                           yaw + 2.5, yaw - 2.9, yaw + math.pi]:
                v_n, v_e = corner_pivot_velocity(yaw, target, 0.08)
                fc = fwd_component(v_n, v_e, yaw)
                self.assertGreater(
                    fc, 0.0,
                    msg=f"yaw={yaw} target={target} → fwd={fc:.4f} would reverse-flip")

    def test_turns_the_short_way(self):
        """Commanded bearing must be on the same side as the shortest turn."""
        for yaw, target in [(0.0, math.pi / 2), (0.0, -math.pi / 2),
                            (1.0, 1.0 + 2.0), (1.0, 1.0 - 2.0)]:
            v_n, v_e = corner_pivot_velocity(yaw, target, 0.08)
            step = wrap_pi(math.atan2(v_e, v_n) - yaw)
            short = wrap_pi(target - yaw)
            self.assertGreater(step * short, 0.0,
                               msg=f"yaw={yaw} target={target}: turned wrong way")

    def test_magnitude_clears_firmware_freeze(self):
        """Even with a tiny/zero param, magnitude must exceed the freeze floor."""
        for param in [0.0, 0.01, 0.05, 0.08, 0.2]:
            v_n, v_e = corner_pivot_velocity(0.0, 1.0, param)
            self.assertGreater(math.hypot(v_n, v_e), FW_ZERO_VEL_THRESHOLD,
                               msg=f"min_corner_speed={param} froze the pivot")

    def test_never_emits_zero_velocity(self):
        """A 90° corner must produce a non-zero vector (the original bug)."""
        v_n, v_e = corner_pivot_velocity(0.0, math.pi / 2, 0.08)
        self.assertGreater(math.hypot(v_n, v_e), FW_ZERO_VEL_THRESHOLD)


if __name__ == "__main__":
    unittest.main()
