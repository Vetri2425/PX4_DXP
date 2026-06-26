#!/usr/bin/env python3
"""F-01 (hybrid): node-side independent GPS_SURVEYED runtime gate.

Covers mandatory regression cases at the spray-node level:
  - #4 LOCAL_NED missions are never RTK-gated (gate inactive)
  - #1/#2 surveyed fault (server reports not-ok) blocks spray ON + forces OFF
  - #7 server watchdog death (gate feed stale/absent) independently blocks ON
"""

from __future__ import annotations

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Importing this first installs the ROS2 stubs (rclpy/std_msgs/…) so the node
# module can be imported without a real ROS2 environment.
from test_spray_manual_override import make_node  # noqa: E402
import spray_controller_node as scn  # noqa: E402


def _gate_msg(active, ok, reason="", seq=1):
    import json
    from std_msgs.msg import String

    msg = String()
    msg.data = json.dumps({"active": active, "ok": ok, "reason": reason, "seq": seq})
    return msg


def test_local_ned_not_gated():
    """#4: with no active surveyed gate, the node never blocks for GPS."""
    node = make_node()
    # default: _gps_gate_active False
    ok, reason = node._evaluate_gps_surveyed_runtime_safety()
    assert ok is True
    assert reason == ""


def test_surveyed_fault_blocks_spray_on():
    """#1/#2: server reports surveyed safety not-ok → node blocks spray ON."""
    node = make_node()
    node._gps_gate_active = True
    node._gps_gate_ok = False
    node._gps_gate_reason = "GPS fix_type=5 is below required (6)"
    node._gps_gate_recv_time = time.monotonic()
    ok, reason = node._evaluate_gps_surveyed_runtime_safety()
    assert ok is False
    assert "below required" in reason


def test_surveyed_ok_allows():
    node = make_node()
    node._gps_gate_active = True
    node._gps_gate_ok = True
    node._gps_gate_recv_time = time.monotonic()
    ok, reason = node._evaluate_gps_surveyed_runtime_safety()
    assert ok is True


def test_stale_feed_blocks_independently():
    """#7: server feed went stale (watchdog died) → node fail-closes on its own."""
    node = make_node()
    node._gps_gate_active = True
    node._gps_gate_ok = True  # last-known good — must NOT be trusted once stale
    node._gps_gate_recv_time = time.monotonic() - 10.0  # > max_age (3.0s)
    ok, reason = node._evaluate_gps_surveyed_runtime_safety()
    assert ok is False
    assert "stale" in reason or "absent" in reason


def test_absent_feed_blocks_when_active():
    node = make_node()
    node._gps_gate_active = True
    node._gps_gate_ok = True
    node._gps_gate_recv_time = None
    ok, _ = node._evaluate_gps_surveyed_runtime_safety()
    assert ok is False


def test_auto_safety_status_ands_in_gate(monkeypatch):
    """_auto_safety_status must AND-in the gate, even when base safety is OK."""
    node = make_node()
    monkeypatch.setattr(scn, "auto_safety_status", lambda **kw: (True, ""))
    # Gate inactive → base result passes through
    ok, _ = node._auto_safety_status(pose_fresh=True, speed=0.2)
    assert ok is True
    # Gate active + fault → blocked despite base OK
    node._gps_gate_active = True
    node._gps_gate_ok = False
    node._gps_gate_reason = "rtk lost"
    node._gps_gate_recv_time = time.monotonic()
    ok, reason = node._auto_safety_status(pose_fresh=True, speed=0.2)
    assert ok is False
    assert reason == "rtk lost"


def test_gate_cb_forces_off_on_fault_edge():
    """Fault message while spray is ON drops the actuator immediately."""
    node = make_node()
    node._commanded = True
    node._off_confirmed = False
    forced = {}
    node._force_off = lambda reason, force=False: forced.update(reason=reason, force=force)
    node._gps_gate_cb(_gate_msg(active=True, ok=False, reason="rtk lost", seq=2))
    assert forced.get("reason", "").startswith("GPS_SURVEYED safety")
    assert node._gps_gate_active is True
    assert node._gps_gate_ok is False


def test_gate_cb_no_force_when_already_off():
    """No redundant OFF spam when spray is already confirmed off."""
    node = make_node()
    node._commanded = False
    node._off_confirmed = True
    called = {"n": 0}
    node._force_off = lambda *a, **k: called.__setitem__("n", called["n"] + 1)
    node._gps_gate_cb(_gate_msg(active=True, ok=False, reason="rtk lost"))
    assert called["n"] == 0


def test_gate_cb_ignores_invalid_payload():
    node = make_node()
    from std_msgs.msg import String

    bad = String()
    bad.data = "{not json"
    node._gps_gate_cb(bad)  # must not raise
    assert node._gps_gate_active is False
