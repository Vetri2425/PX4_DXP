#!/usr/bin/env python3
"""Phase B — RTK/GPS fix-quality gate tests (plan §7.6). In-env (needs rclpy).

Covers: gate disabled (SITL/bench), no-data → stale, staleness distinct from
bad fix, below-threshold fail, asymmetric hysteresis (instant drop / delayed
re-enable), and the gate's position in _auto_safety_status.

Time is controlled via the mock clock's mutable `.ns` (see _Clock in
test_spray_manual_override): _gps_cb captures recv_time at the current ns, and
advancing ns ages it / advances the recover hold.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Import the harness FIRST — it installs the ROS stubs (incl. the GPSRAW stub),
# so this test runs on the Mac too. GPSRAW must be imported AFTER that.
from test_spray_manual_override import _Param, make_node  # noqa: E402
from spray_controller_node import _build_path_model  # noqa: E402
from mavros_msgs.msg import GPSRAW  # noqa: E402  (stub after harness import)


def _enable_gate(node):
    node._params["spray_require_rtk_fix"] = _Param(True)


def _set_fix(node, fix_type):
    msg = GPSRAW()
    msg.fix_type = fix_type
    node._gps_cb(msg)


def _advance(node, seconds):
    node._clock.ns += int(seconds * 1e9)


def test_gate_disabled_always_passes():
    """Fixture default (require_rtk_fix False) — SITL/bench: never blocks."""
    node = make_node()
    assert node._gps_gate() == (True, "")
    # ... even with no GPS data at all.
    assert node._gps_recv_time is None


def test_no_data_is_stale():
    node = make_node()
    _enable_gate(node)
    ok, reason = node._gps_gate()
    assert ok is False and reason == "gps stale"


def test_stale_after_timeout():
    node = make_node()
    _enable_gate(node)
    _set_fix(node, 6)          # good, fresh
    _advance(node, 2.5)        # > gps_fix_timeout_s (2.0)
    fresh, fix_ok, _ = node._gps_health()
    assert fresh is False
    ok, reason = node._gps_gate()
    assert ok is False and reason == "gps stale"


def test_below_threshold_fails_distinctly():
    node = make_node()
    _enable_gate(node)
    _set_fix(node, 5)          # RTK_FLOAT < required 6, fresh
    ok, reason = node._gps_gate()
    assert ok is False
    assert reason == "gps fix 5 < required 6"   # distinct from "gps stale"


def test_recover_hold_delays_reenable():
    """Good fix does not immediately re-open: must hold gps_recover_hold_s."""
    node = make_node()
    _enable_gate(node)
    _set_fix(node, 6)
    ok, reason = node._gps_gate()               # first good sample
    assert ok is False and "recovering" in reason
    _advance(node, 0.5)                         # still inside 1.0 s hold
    ok, _ = node._gps_gate()
    assert ok is False
    _advance(node, 0.6)                         # total 1.1 s ≥ hold
    ok, reason = node._gps_gate()
    assert ok is True and reason == ""


def test_drop_is_instant_and_resets_recovery():
    """A single bad sample fails immediately and re-arms the full hold."""
    node = make_node()
    _enable_gate(node)
    _set_fix(node, 6)
    node._gps_gate()
    _advance(node, 1.2)
    assert node._gps_gate()[0] is True          # recovered
    _set_fix(node, 5)                           # instant drop to FLOAT
    ok, reason = node._gps_gate()
    assert ok is False and reason == "gps fix 5 < required 6"
    assert node._gps_recover_since is None      # recovery re-armed
    # Back to good: must re-hold, not instantly re-open.
    _set_fix(node, 6)
    ok, reason = node._gps_gate()
    assert ok is False and "recovering" in reason


def test_health_reports_fix_name():
    node = make_node()
    _set_fix(node, 6)
    _, fix_ok, name = node._gps_health()
    assert name == "RTK_FIXED" and fix_ok is True
    _set_fix(node, 5)
    _, fix_ok, name = node._gps_health()
    assert name == "RTK_FLOAT" and fix_ok is False   # 5 < min 6


def test_gate_ordering_in_auto_safety():
    """With everything else healthy but no RTK, the blocking reason is GPS."""
    node = make_node(armed=True, mode="OFFBOARD", require_offboard=True)
    _enable_gate(node)
    node._path_model = _build_path_model([(0.0, 0.0), (5.0, 0.0)], [True, True])
    ok, reason = node._auto_safety_status(pose_fresh=True, speed=0.3, velocity_fresh=True)
    assert ok is False and reason == "gps stale"
    # Give a good fix; the recover hold starts on this first good tick, so it
    # takes another tick past gps_recover_hold_s before the gate opens.
    _set_fix(node, 6)
    ok, _ = node._auto_safety_status(pose_fresh=True, speed=0.3, velocity_fresh=True)
    assert ok is False  # recovering, hold not yet elapsed
    _advance(node, 1.2)
    ok, reason = node._auto_safety_status(pose_fresh=True, speed=0.3, velocity_fresh=True)
    assert ok is True and reason == ""  # no pivot active in the fixture


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("ALL RTK GATE TESTS PASSED")
