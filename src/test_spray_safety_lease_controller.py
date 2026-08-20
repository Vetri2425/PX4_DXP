#!/usr/bin/env python3
"""Controller-side proof that ON authority is continuously leased and revoked."""

import json

from test_spray_manual_override import _Param, make_node
from std_msgs.msg import String


def _last_lease(node):
    return json.loads(node._safety_lease_pub.msgs[-1])


def test_idle_controller_publishes_denied_lease():
    node = make_node()
    node._drive_fsm_tick("idle")
    lease = _last_lease(node)
    assert lease["allow_on"] is False
    assert lease["backend"] == "mavlink_actuator"
    assert lease["off_value"] == -1.0


def test_confirmed_on_controller_publishes_fresh_on_lease():
    node = make_node()
    node._manual_active = True
    node._desired_raw = True
    node._drive_fsm_tick("manual ON")
    lease = _last_lease(node)
    assert lease["allow_on"] is True
    assert lease["command_seq"] == node._fsm.cmd_seq


def test_safety_loss_revokes_lease_in_same_fsm_drive():
    node = make_node()
    node._manual_active = True
    node._desired_raw = True
    node._drive_fsm_tick("manual ON")
    assert _last_lease(node)["allow_on"] is True

    node._armed = False
    node._drive_fsm_tick("disarmed")
    assert _last_lease(node)["allow_on"] is False


def test_shutdown_revokes_lease_before_flush():
    node = make_node()
    node._manual_active = True
    node._desired_raw = True
    node._drive_fsm_tick("manual ON")
    assert _last_lease(node)["allow_on"] is True

    node.shutdown_off()
    assert any(json.loads(raw)["allow_on"] is False for raw in node._safety_lease_pub.msgs)


def test_controller_refuses_on_without_watchdog_heartbeat():
    node = make_node()
    node._params["spray_watchdog_required"] = _Param(True)
    node._manual_active = True
    node._desired_raw = True
    node._drive_fsm_tick("manual ON without watchdog")
    assert _last_lease(node)["allow_on"] is False
    assert node._fsm.commanded is False


def test_fresh_watchdog_heartbeat_allows_on_and_staleness_revokes_it():
    node = make_node()
    node._params["spray_watchdog_required"] = _Param(True)
    msg = String()
    msg.data = json.dumps(
        {
            "watchdog_alive": True,
            "command_service_ready": True,
            "off_authority_ready": True,
        }
    )
    node._spray_watchdog_status_cb(msg)
    node._manual_active = True
    node._desired_raw = True
    node._drive_fsm_tick("manual ON with watchdog")
    assert _last_lease(node)["allow_on"] is True

    node._clock.ns += 1_100_000_000
    node._drive_fsm_tick("watchdog stale")
    assert _last_lease(node)["allow_on"] is False
    assert node._fsm.commanded is False


def test_malformed_watchdog_heartbeat_revokes_on_immediately():
    node = make_node()
    node._params["spray_watchdog_required"] = _Param(True)
    healthy = String()
    healthy.data = json.dumps(
        {
            "watchdog_alive": True,
            "command_service_ready": True,
            "off_authority_ready": True,
        }
    )
    node._spray_watchdog_status_cb(healthy)
    node._manual_active = True
    node._desired_raw = True
    node._drive_fsm_tick("manual ON with watchdog")
    assert _last_lease(node)["allow_on"] is True

    malformed = String()
    malformed.data = "[]"
    node._spray_watchdog_status_cb(malformed)
    assert _last_lease(node)["allow_on"] is False
    assert node._fsm.commanded is False


def test_service_presence_without_confirmed_off_authority_refuses_on():
    node = make_node()
    node._params["spray_watchdog_required"] = _Param(True)
    heartbeat = String()
    heartbeat.data = json.dumps(
        {
            "watchdog_alive": True,
            "command_service_ready": True,
            "off_authority_ready": False,
        }
    )
    node._spray_watchdog_status_cb(heartbeat)
    node._manual_active = True
    node._desired_raw = True
    node._drive_fsm_tick("watchdog has no confirmed OFF authority")
    assert _last_lease(node)["allow_on"] is False
    assert node._fsm.commanded is False
