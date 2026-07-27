#!/usr/bin/env python3
"""Phase E — FlowModulator unit tests (plan §8). Pure, no rclpy — runs on Mac.

Covers: linear scaling between floor and on_value, saturation at/above rated
speed, the slew-rate cap on a step change, floor at zero speed, and the
OFF→ON reset seeding to the floor.

Run:  python3 -m pytest -q test_spray_flow_model.py
"""

from spray_flow_model import FlowModulator


def _settle(m, speed, dt=1.0, n=200):
    """Run enough ticks that the slew-limited output reaches its target."""
    v = None
    for _ in range(n):
        v = m.update(speed, dt)
    return v


def test_scales_linearly_between_floor_and_on():
    m = FlowModulator(min_flow_value=0.2, on_value=1.0,
                      rated_marking_speed_mps=0.4, max_slew_per_s=1000.0)
    # Half rated speed → halfway between 0.2 and 1.0 = 0.6.
    assert abs(_settle(m, 0.2) - 0.6) < 1e-6
    # Full rated speed → on_value.
    assert abs(_settle(m, 0.4) - 1.0) < 1e-6


def test_saturates_above_rated():
    m = FlowModulator(0.2, 1.0, rated_marking_speed_mps=0.4, max_slew_per_s=1000.0)
    assert abs(_settle(m, 0.8) - 1.0) < 1e-6   # 2× rated still caps at on_value


def test_floor_at_zero_speed():
    m = FlowModulator(0.2, 1.0, 0.4, max_slew_per_s=1000.0)
    assert abs(_settle(m, 0.0) - 0.2) < 1e-6   # never below the usable floor


def test_slew_limits_a_step():
    # max_slew 0.5/s, dt 0.1 → at most 0.05 per tick.
    m = FlowModulator(0.2, 1.0, 0.4, max_slew_per_s=0.5)
    m.reset()                                   # seeded at 0.2
    v1 = m.update(0.4, 0.1)                      # target 1.0, but capped
    assert abs(v1 - 0.25) < 1e-6                 # 0.2 + 0.05
    v2 = m.update(0.4, 0.1)
    assert abs(v2 - 0.30) < 1e-6                 # ramps, does not jump


def test_reset_seeds_to_floor():
    m = FlowModulator(0.2, 1.0, 0.4, max_slew_per_s=1000.0)
    _settle(m, 0.4)                              # push to 1.0
    assert m.value > 0.9
    m.reset()
    assert abs(m.value - 0.2) < 1e-6             # OFF→ON edge: back to floor


def test_zero_rated_speed_is_full_flow():
    m = FlowModulator(0.2, 1.0, rated_marking_speed_mps=0.0, max_slew_per_s=1000.0)
    assert abs(_settle(m, 0.01) - 1.0) < 1e-6    # degenerate rated → always full


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("ALL FLOW MODEL TESTS PASSED")
