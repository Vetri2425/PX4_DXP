#!/usr/bin/env python3
"""Phase D — PointMeter unit tests (plan §8). Pure, no rclpy — runs on the Mac.

Covers §8 point requirements: arrival settle (+ reset-on-blip, no partial
credit), dwell then OFF-confirm gating before advance, position-only vs
heading-gated arrival, the unreachable-target watchdog, the pivot-gate
exemption scope, and empty / single-point lists.

Run:  python3 -m pytest -q test_spray_point_v2.py
"""

from spray_modes import PointMeter


def _at(m, n, e, t, *, yaw=0.0, speed=0.0, off_confirmed=True):
    return m.update(n, e, yaw, speed, t, off_confirmed)


def test_full_happy_path_two_points():
    m = PointMeter([(0.0, 0.0), (5.0, 0.0)], 0.10, arrival_settle_s=0.2, dwell_s=0.5)
    assert _at(m, 2.0, 0.0, 0.0).phase == "transit"        # far, driving
    assert _at(m, 0.0, 0.0, 1.0).phase == "arriving"        # on the dot, settling
    assert _at(m, 0.0, 0.0, 1.1).geometry_desired is False  # settle not done
    u = _at(m, 0.0, 0.0, 1.3)                               # settle done → dwell ON
    assert u.phase == "dwelling" and u.geometry_desired and u.exempt_pivot
    assert _at(m, 0.0, 0.0, 1.5).geometry_desired is True    # mid-dwell
    u = _at(m, 0.0, 0.0, 1.9)                               # dwell done, off ok → advance
    assert u.target_index == 1 and u.phase == "transit" and not u.geometry_desired
    # Second dot.
    _at(m, 5.0, 0.0, 3.0)
    assert _at(m, 5.0, 0.0, 3.3).geometry_desired is True    # dwelling on point 1
    u = _at(m, 5.0, 0.0, 4.0)                               # dwell done → done
    assert u.done and u.phase == "done" and not u.geometry_desired


def test_settle_resets_on_blip_no_partial_credit():
    m = PointMeter([(0.0, 0.0)], 0.10, arrival_settle_s=0.3, dwell_s=0.2)
    _at(m, 0.0, 0.0, 1.0)                                    # arriving, settle_start 1.0
    _at(m, 0.0, 0.0, 1.2)                                    # 0.2 < 0.3
    blip = _at(m, 0.5, 0.0, 1.25)                           # jumps off the dot
    assert blip.phase in ("transit", "arriving") and not blip.geometry_desired
    _at(m, 0.0, 0.0, 1.3)                                    # back on — settle RESTARTS at 1.3
    assert _at(m, 0.0, 0.0, 1.45).geometry_desired is False  # 0.15 < 0.3, not credited
    assert _at(m, 0.0, 0.0, 1.65).geometry_desired is True   # 0.35 ≥ 0.3 → spray


def test_advance_waits_for_off_confirmed():
    m = PointMeter([(0.0, 0.0), (5.0, 0.0)], 0.10, arrival_settle_s=0.0, dwell_s=0.3)
    _at(m, 0.0, 0.0, 1.0)                                    # arrive+settle(0) → dwelling
    assert _at(m, 0.0, 0.0, 1.1).geometry_desired is True
    # Dwell done but OFF not yet confirmed: spray desired-off, but DO NOT advance.
    u = _at(m, 0.0, 0.0, 1.4, off_confirmed=False)
    assert u.geometry_desired is False and u.target_index == 0 and u.phase == "off_wait"
    u = _at(m, 0.0, 0.0, 1.5, off_confirmed=False)          # still waiting
    assert u.target_index == 0
    u = _at(m, 0.0, 0.0, 1.6, off_confirmed=True)           # OFF confirmed → advance
    assert u.target_index == 1


def test_watchdog_skips_unreachable_point():
    m = PointMeter([(0.0, 0.0), (5.0, 0.0)], 0.10, arrival_settle_s=0.2, dwell_s=0.3,
                   point_arrival_timeout_s=10.0)
    # Never reach point 0: stay 1 m away past the timeout.
    _at(m, 1.0, 0.0, 0.0)                                    # transit_start = 0
    u = _at(m, 1.0, 0.0, 11.0)                              # > timeout → skip point 0
    assert u.skipped_index == 0
    assert u.target_index == 1                              # advanced, never sprayed
    assert m.skipped_indices == [0]
    # A skipped point is never counted as sprayed.
    assert u.geometry_desired is False


def test_heading_gate_blocks_until_aligned():
    import math
    # Point 1 approached along +N (bearing 0). Heading tol 10°.
    m = PointMeter([(0.0, 0.0), (5.0, 0.0)], 0.10, arrival_settle_s=0.0, dwell_s=0.3,
                   heading_tolerance_deg=10.0)
    # Skip point 0 quickly (position-only for i==0).
    _at(m, 0.0, 0.0, 0.0)
    _at(m, 0.0, 0.0, 0.1)                                    # dwell point 0
    _at(m, 0.0, 0.0, 0.5)                                    # advance to 1
    # At point 1 but yaw 90° off the approach bearing → not arrived.
    u = _at(m, 5.0, 0.0, 1.0, yaw=math.radians(90))
    assert u.phase == "transit" and not u.geometry_desired
    # Aligned within tol → arrives and sprays.
    u = _at(m, 5.0, 0.0, 1.1, yaw=math.radians(3))
    assert u.geometry_desired is True


def test_speed_gate_blocks_arrival():
    m = PointMeter([(0.0, 0.0)], 0.10, arrival_settle_s=0.0, dwell_s=0.3,
                   point_arrival_max_speed_mps=0.05)
    moving = _at(m, 0.0, 0.0, 1.0, speed=0.3)               # on the dot but fast
    assert moving.phase == "transit" and not moving.geometry_desired
    stopped = _at(m, 0.0, 0.0, 1.1, speed=0.0)
    assert stopped.geometry_desired is True


def test_exempt_pivot_only_while_dwelling_on_dot():
    m = PointMeter([(0.0, 0.0)], 0.10, arrival_settle_s=0.0, dwell_s=0.5)
    assert _at(m, 3.0, 0.0, 0.0).exempt_pivot is False       # transiting
    assert _at(m, 0.0, 0.0, 1.0).exempt_pivot is True        # dwelling on the dot
    # 0.5 m away while (hypothetically) dwelling would NOT exempt — but by then
    # we are not dwelling anyway; exemption requires both.


def test_empty_and_single_point():
    empty = PointMeter([], 0.10, 0.0, 0.3)
    u = empty.update(0, 0, 0, 0, 0.0, True)
    assert u.done and u.phase == "done"
    one = PointMeter([(1.0, 1.0)], 0.10, 0.0, 0.2)
    one.update(1.0, 1.0, 0, 0, 1.0, True)                    # dwell
    u = one.update(1.0, 1.0, 0, 0, 1.3, True)                # done after dwell
    assert u.done


def test_bad_tolerance_rejected():
    for tol in (0.0, -1.0):
        try:
            PointMeter([(0.0, 0.0)], tol, 0.0, 0.3)
            assert False, f"expected ValueError for tol={tol}"
        except ValueError:
            pass


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("ALL POINT TESTS PASSED")
