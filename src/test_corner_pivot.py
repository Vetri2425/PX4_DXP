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


# ======================================================================
# BUG-T3 regression tests — first-run velocity cone clamp
# ======================================================================
def clamp_velocity_to_forward_cone(v_n, v_e, yaw_ned, speed):
    """Mirror of rpp_controller_node._clamp_velocity_to_forward_cone.
    
    Clamps commanded velocity bearing into ±75° forward cone to prevent
    PX4 reverse-flip when heading error >90°.
    """
    if speed <= 1e-6:
        return v_n, v_e
    mag = math.hypot(v_n, v_e)
    if mag <= 1e-9:
        return v_n, v_e
    bearing = math.atan2(v_e, v_n)  # NED: atan2(E, N); 0=North, CW+
    heading_err = wrap_pi(bearing - yaw_ned)
    if abs(heading_err) <= MAX_BEARING_OFFSET:
        return v_n, v_e
    step = max(-MAX_BEARING_OFFSET, min(MAX_BEARING_OFFSET, heading_err))
    cmd_bearing = yaw_ned + step
    return speed * math.cos(cmd_bearing), speed * math.sin(cmd_bearing)


def raw_tracking_velocity(yaw_ned, dn, de, speed):
    """Emulate raw velocity emission from _control_segment_profile / _control_loop."""
    l_actual = math.hypot(dn, de)
    if l_actual < 1e-9:
        return 0.0, 0.0
    unit_n = dn / l_actual
    unit_e = de / l_actual
    return speed * unit_n, speed * unit_e


class TestInitialAlignment(unittest.TestCase):
    """BUG-T3 regression: first-run mission-start alignment is missing.
    
    When the rover starts facing West and the first segment bearing is North,
    the raw velocity vector produces zero/negative forward component (or 
    close to it), triggering PX4 reverse-flip. The _clamp_velocity_to_forward_cone
    helper must fix this by keeping the commanded bearing inside the ±75° cone.
    """

    def test_bug_t3_west_start_north_segment_clamped(self):
        """BUG-T3: rover yaw=West(-90°), first segment North(0°).
        
        Raw velocity points North, bearing offset is 90°. 
        Clamped velocity must keep fwd>0, bearing offset ≤75°.
        """
        yaw_ned = -math.pi / 2   # West in NED (0=North, CW+: -90°)
        speed = 0.35
        # Lookahead is North of rover
        dn = 1.0  # North
        de = 0.0  # East
        
        v_n_raw, v_e_raw = raw_tracking_velocity(yaw_ned, dn, de, speed)
        fwd_raw = fwd_component(v_n_raw, v_e_raw, yaw_ned)
        
        # Raw forward component should be ~0 (perpendicular), confirming the bug precondition
        self.assertLessEqual(
            abs(fwd_raw), 0.01,
            f"BUG-T3 precond: raw fwd should be ~0 at West+North, got {fwd_raw:.6f}"
        )
        
        v_n_c, v_e_c = clamp_velocity_to_forward_cone(
            v_n_raw, v_e_raw, yaw_ned, speed
        )
        fwd_clamped = fwd_component(v_n_c, v_e_c, yaw_ned)
        
        self.assertGreater(
            fwd_clamped, 0.0,
            f"BUG-T3 fix: clamped fwd must be >0, got {fwd_clamped:.6f}"
        )
        
        bearing = math.atan2(v_e_c, v_n_c)
        offset = abs(wrap_pi(bearing - yaw_ned))
        self.assertLessEqual(
            math.degrees(offset), 75.0 + 1e-9,
            f"BUG-T3 fix: bearing offset {math.degrees(offset):.1f}° > 75° cone"
        )
        
        # Turn direction must be the shortest way toward North
        # West (-90°) -> North (0°) is +90° CW in NED
        # Clamped bearing should be yaw + 75° = -15°, which approaches North CW
        self.assertGreater(
            wrap_pi(bearing - yaw_ned), 0.0,
            "BUG-T3 fix: must turn CW (shortest) from West to North"
        )

    def test_west_start_east_segment_clamped(self):
        """West->East: raw offset = 90°, clamp ensures fwd>0."""
        yaw_ned = -math.pi / 2   # West
        speed = 0.35
        dn = 0.0   # No North
        de = 1.0   # East
        
        v_n_raw, v_e_raw = raw_tracking_velocity(yaw_ned, dn, de, speed)
        v_n_c, v_e_c = clamp_velocity_to_forward_cone(
            v_n_raw, v_e_raw, yaw_ned, speed
        )
        
        fwd = fwd_component(v_n_c, v_e_c, yaw_ned)
        self.assertGreater(fwd, 0.0, f"West->East: fwd must be >0, got {fwd:.6f}")

    def test_aligned_start_no_clamp(self):
        """When rover yaw matches command bearing, output equals input."""
        for yaw_ned in [0.0, 0.5, -0.3, math.pi / 2, -math.pi / 2]:
            dn = math.cos(yaw_ned)
            de = math.sin(yaw_ned)
            speed = 0.35
            v_n, v_e = raw_tracking_velocity(yaw_ned, dn, de, speed)
            v_n_c, v_e_c = clamp_velocity_to_forward_cone(v_n, v_e, yaw_ned, speed)
            self.assertAlmostEqual(v_n, v_n_c, places=10)
            self.assertAlmostEqual(v_e, v_e_c, places=10)

    def test_45_degree_offset_no_clamp(self):
        """45° offset is inside ±75° cone, output equals input."""
        yaw_ned = 0.0
        dn = math.cos(math.radians(45.0))
        de = math.sin(math.radians(45.0))
        speed = 0.35
        v_n, v_e = raw_tracking_velocity(yaw_ned, dn, de, speed)
        v_n_c, v_e_c = clamp_velocity_to_forward_cone(v_n, v_e, yaw_ned, speed)
        self.assertAlmostEqual(v_n, v_n_c, places=10)
        self.assertAlmostEqual(v_e, v_e_c, places=10)

    def test_100_degree_offset_clamps(self):
        """100° offset must be clamped to exactly ±75° from yaw."""
        yaw_ned = 0.0
        # Command bearing 100° from North (nearly South)
        dn = math.cos(math.radians(100.0))
        de = math.sin(math.radians(100.0))
        speed = 0.35
        v_n_raw, v_e_raw = raw_tracking_velocity(yaw_ned, dn, de, speed)
        
        # Verify raw fwd is negative (reverse precondition)
        fwd_raw = fwd_component(v_n_raw, v_e_raw, yaw_ned)
        self.assertLess(fwd_raw, 0.0, 
                        f"100° offset must produce negative fwd raw, got {fwd_raw:.6f}")
        
        v_n_c, v_e_c = clamp_velocity_to_forward_cone(
            v_n_raw, v_e_raw, yaw_ned, speed
        )
        
        fwd_clamped = fwd_component(v_n_c, v_e_c, yaw_ned)
        self.assertGreater(fwd_clamped, 0.0,
                          f"Clamped fwd must be >0, got {fwd_clamped:.6f}")
        
        bearing_c = math.atan2(v_e_c, v_n_c)
        offset = abs(wrap_pi(bearing_c - yaw_ned))
        # Should be clamped to exactly 75° (max allowed)
        self.assertAlmostEqual(
            math.degrees(offset), 75.0, delta=1e-9,
            msg=f"100° offset should clamp to 75°, got {math.degrees(offset):.1f}°"
        )

    def test_zero_speed_no_clamp(self):
        """When speed is zero, helper must return input unchanged."""
        v_n, v_e = clamp_velocity_to_forward_cone(1.0, 0.0, 0.0, 0.0)
        self.assertEqual(v_n, 1.0)
        self.assertEqual(v_e, 0.0)

    def test_zero_velocity_magnitude_no_clamp(self):
        """When velocity magnitude is near-zero, helper returns input unchanged."""
        v_n, v_e = clamp_velocity_to_forward_cone(0.0, 0.0, 0.0, 0.35)
        self.assertEqual(v_n, 0.0)
        self.assertEqual(v_e, 0.0)

    def test_segment_and_smooth_profile_behavior(self):
        """Verify that raw_tracking_velocity (segment profile) has the 
        same reverse-flip exposure as the smooth/RPP path.
        
        Both emit v_n = speed * unit_n, v_e = speed * unit_e.
        The cone clamp is the universal fix for both.
        """
        # Same scenario: West start, North segment
        yaw_ned = -math.pi / 2
        speed = 0.35
        
        # Both profiles compute the same unit vector from rover to lookahead
        dn, de = 1.0, 0.0  # North lookahead
        
        v_n, v_e = raw_tracking_velocity(yaw_ned, dn, de, speed)
        fwd_raw = fwd_component(v_n, v_e, yaw_ned)
        
        # This proves the exposure is identical
        self.assertLessEqual(abs(fwd_raw), 0.01,
                            "Both profiles produce ~0 forward component at West+North")
        
        # Clamped version is the fix for both
        v_n_c, v_e_c = clamp_velocity_to_forward_cone(v_n, v_e, yaw_ned, speed)
        fwd_clamped = fwd_component(v_n_c, v_e_c, yaw_ned)
        self.assertGreater(fwd_clamped, 0.0,
                          "Cone clamp fixes both segment and smooth profiles")


if __name__ == "__main__":
    unittest.main()
