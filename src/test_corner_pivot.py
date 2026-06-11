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


def corner_pivot_velocity(target_heading_ned: float, min_corner_speed: float):
    """Mirror of rpp_controller_node CORNER_ALIGN / _run_alignment_hold.

    target_heading_ned: NED bearing (0=North, CW+) of the next segment.
    Returns (v_n, v_e) in NED.
    """
    corner_speed = max(0.05, min_corner_speed)
    v_n = corner_speed * math.cos(target_heading_ned)
    v_e = corner_speed * math.sin(target_heading_ned)
    return v_n, v_e


class TestCornerPivot(unittest.TestCase):
    def test_vector_points_along_exit_heading(self):
        """Published bearing atan2(vE, vN) must equal the target heading."""
        for heading in [0.0, math.pi / 2, math.pi, -math.pi / 2, 0.7, -2.3]:
            v_n, v_e = corner_pivot_velocity(heading, 0.08)
            bearing = math.atan2(v_e, v_n)  # firmware: atan2(velocity_E, velocity_N)
            d = (bearing - heading + math.pi) % (2 * math.pi) - math.pi
            self.assertAlmostEqual(d, 0.0, places=6, msg=f"heading={heading}")

    def test_magnitude_clears_firmware_freeze(self):
        """Even with a tiny/zero param, magnitude must exceed the freeze floor."""
        for param in [0.0, 0.01, 0.05, 0.08, 0.2]:
            v_n, v_e = corner_pivot_velocity(1.0, param)
            mag = math.hypot(v_n, v_e)
            self.assertGreater(mag, FW_ZERO_VEL_THRESHOLD,
                               msg=f"min_corner_speed={param} froze the pivot")

    def test_never_emits_zero_velocity(self):
        """A 90° corner must produce a non-zero vector (the original bug)."""
        v_n, v_e = corner_pivot_velocity(math.pi / 2, 0.08)
        self.assertGreater(math.hypot(v_n, v_e), FW_ZERO_VEL_THRESHOLD)


if __name__ == "__main__":
    unittest.main()
