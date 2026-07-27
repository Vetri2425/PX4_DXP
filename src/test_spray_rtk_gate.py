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


# ── A14: the accuracy half of the gate (2026-07-27) ──────────────────────────
#
# Until A14, `_gps_health` compared fix_type and nothing else — GPSRAW.h_acc
# arrived on the same message and was discarded. An RTK_FIXED claim opened the
# valve at any reported error. These tests pin the FALLBACK contract:
#   accuracy reported  -> enforce it
#   accuracy missing   -> behave exactly as before (h_acc == 0 is the driver's
#                         "unknown" sentinel, latched per boot on ~half of
#                         boots on this hardware; refusing on it would ground
#                         the rover for a reason unrelated to safety)

def _set_fix_acc(node, fix_type, h_acc_mm):
    msg = GPSRAW()
    msg.fix_type = fix_type
    msg.h_acc = h_acc_mm
    node._gps_cb(msg)


def test_good_fix_with_bad_accuracy_is_refused():
    """The bug A14 names: fix_type 6 while the receiver reports 0.85 m of
    horizontal error. Pre-A14 this returned fix_ok True and the valve opened.

    Fails if the fix is wrong: with the accuracy half removed, `ok` is True
    because fix_type 6 >= 6, so the assertion breaks. It cannot pass by
    accident — 850 mm is 8.5x the 0.10 m default.
    """
    node = make_node()
    _enable_gate(node)
    _set_fix_acc(node, 6, 850)
    fresh, ok, name = node._gps_health()
    assert fresh is True
    assert ok is False, "RTK_FIXED with 0.85 m of reported error must not spray"
    assert "hacc" in name, "the operator must be told WHY, not just that it failed"


def test_good_fix_with_good_accuracy_sprays():
    """Scope guard: a genuinely good fix must still pass, or the gate is just
    an outage. 15 mm is what this rig measures at RTK_FIXED."""
    node = make_node()
    _enable_gate(node)
    _set_fix_acc(node, 6, 15)
    assert node._gps_health()[1] is True


def test_unreported_accuracy_falls_back_to_pre_a14_behaviour():
    """h_acc == 0 is 'unknown', NOT 'perfect' — but it must not block either.

    This is the whole reason the gate ships enabled: on boots where the driver
    supplies no accuracy (about half of them, latched per boot) the rover keeps
    working exactly as it did before A14. If this test fails, the fix has
    grounded the rover on the sentinel and must not be deployed.
    """
    node = make_node()
    _enable_gate(node)
    _set_fix_acc(node, 6, 0)
    assert node._gps_h_acc_m is None, "0 mm must be read as unknown, never 0.0 m"
    assert node._gps_health()[1] is True


def test_a_bad_fix_still_fails_regardless_of_accuracy():
    """Ordering guard: the fix-type test must run FIRST. A receiver reporting
    a tight h_acc on an RTK_FLOAT solution must not be promoted by the new
    accuracy branch."""
    node = make_node()
    _enable_gate(node)
    _set_fix_acc(node, 5, 12)
    assert node._gps_health()[1] is False


def test_accuracy_half_can_be_disabled_with_zero():
    """Escape hatch: spray_max_hrms_m = 0 restores pre-A14 behaviour exactly,
    for a site where the receiver's accuracy is known to be unreliable."""
    node = make_node()
    _enable_gate(node)
    node._params["spray_max_hrms_m"] = _Param(0.0)
    _set_fix_acc(node, 6, 850)
    assert node._gps_health()[1] is True
