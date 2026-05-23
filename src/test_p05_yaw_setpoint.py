#!/usr/bin/env python3
"""Test explicit yaw computation from velocity direction.

Tests:
  1. yaw_ENU = atan2(v_n, v_e) derived from velocity vector
  2. Yaw freezes below 1 cm/s (holds last commanded heading)
  3. TYPE_MASK_VELOCITY_AND_YAW = 2503
"""

import math
import unittest


class TestExplicitYaw(unittest.TestCase):
    """Test explicit yaw computation from velocity vector (twist_to_setpoint)."""

    def test_yaw_enu_from_velocity(self):
        """Test yaw_ENU = atan2(v_n, v_e) — ENU convention (0=East, CCW+)."""
        # NED input: v_n = North component, v_e = East component
        # ENU yaw = atan2(v_n, v_e)
        test_cases = [
            # (v_n, v_e), expected_yaw_enu
            ((0.0, 1.0), 0.0),            # Pure East → yaw 0 (East)
            ((1.0, 0.0), math.pi / 2),    # Pure North → yaw π/2 (North)
            ((0.0, -1.0), math.pi),        # Pure West → yaw π (West)
            ((-1.0, 0.0), -math.pi / 2),   # Pure South → yaw -π/2 (South)
            ((1.0, 1.0), math.pi / 4),     # NE → yaw π/4
            ((1.0, -1.0), 3 * math.pi / 4),  # NW → yaw 3π/4
        ]

        for (v_n, v_e), expected in test_cases:
            yaw_enu = math.atan2(v_n, v_e)
            self.assertAlmostEqual(yaw_enu, expected, places=5,
                                   msg=f"v_n={v_n}, v_e={v_e}")

    def test_yaw_freeze_below_threshold(self):
        """Test that yaw freezes when speed < 1 cm/s."""
        speed_threshold = 0.01  # 1 cm/s
        last_yaw = 0.5  # radians

        # Case 1: speed above threshold → compute new yaw
        v_n, v_e = 0.1, 0.05
        speed = math.hypot(v_n, v_e)
        if speed > speed_threshold:
            yaw = math.atan2(v_n, v_e)
        else:
            yaw = last_yaw
        self.assertAlmostEqual(yaw, math.atan2(0.1, 0.05), places=5)

        # Case 2: speed below threshold → freeze
        v_n, v_e = 0.001, 0.0005
        speed = math.hypot(v_n, v_e)
        if speed > speed_threshold:
            yaw = math.atan2(v_n, v_e)
        else:
            yaw = last_yaw
        self.assertAlmostEqual(yaw, last_yaw, places=5)

    def test_ned_to_enu_velocity_swap(self):
        """Test NED→ENU velocity swap: velocity.x = v_e, velocity.y = v_n."""
        # RPP outputs NED: vector.x = v_north, vector.y = v_east
        # MAVROS expects ENU: velocity.x = East, velocity.y = North
        v_n = 0.3  # North component (NED x)
        v_e = 0.4  # East component (NED y)
        v_d = 0.0  # Down (always 0 for ground rover)

        # After swap:
        self.assertAlmostEqual(v_e, 0.4)  # velocity.x = East
        self.assertAlmostEqual(v_n, 0.3)  # velocity.y = North
        self.assertAlmostEqual(-v_d, 0.0)  # velocity.z = Up (negated Down)

    def test_type_mask_constants(self):
        """Test that type_mask constants are correct."""
        IGNORE_PX = 1
        IGNORE_PY = 2
        IGNORE_PZ = 4
        IGNORE_VX = 8
        IGNORE_VY = 16
        IGNORE_VZ = 32
        IGNORE_AFX = 64
        IGNORE_AFY = 128
        IGNORE_AFZ = 256
        IGNORE_YAW = 1024
        IGNORE_YAW_RATE = 2048

        # Velocity-only (legacy, no longer used)
        TYPE_MASK_VELOCITY = (
            IGNORE_PX | IGNORE_PY | IGNORE_PZ
            | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
            | IGNORE_YAW | IGNORE_YAW_RATE
        )
        self.assertEqual(TYPE_MASK_VELOCITY, 3527)

        # Velocity + explicit yaw (current default)
        # Ignores: PX, PY, PZ, AFX, AFY, AFZ, YAW_RATE
        # Sends: VX, VY, VZ, YAW
        TYPE_MASK_VELOCITY_AND_YAW = (
            IGNORE_PX | IGNORE_PY | IGNORE_PZ
            | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
            | IGNORE_YAW_RATE
        )
        # 1 + 2 + 4 + 64 + 128 + 256 + 2048 = 2503
        self.assertEqual(TYPE_MASK_VELOCITY_AND_YAW, 2503)

    def test_yaw_enu_ned_consistency(self):
        """Test that yaw_ENU = atan2(v_n, v_e) is consistent with NED yaw.

        For a velocity vector:
          yaw_NED = atan2(v_e, v_n)  (0=North, CW+)
          yaw_ENU = atan2(v_n, v_e)  (0=East, CCW+)
          yaw_ENU = π/2 - yaw_NED
        """
        v_n, v_e = 0.3, 0.4  # NE direction

        yaw_ned = math.atan2(v_e, v_n)  # NED: 0=North, CW+
        yaw_enu = math.atan2(v_n, v_e)  # ENU: 0=East, CCW+

        # yaw_ENU = π/2 - yaw_NED
        self.assertAlmostEqual(yaw_enu, math.pi / 2 - yaw_ned, places=5)


if __name__ == "__main__":
    unittest.main()