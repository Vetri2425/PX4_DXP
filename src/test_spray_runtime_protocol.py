#!/usr/bin/env python3
"""Runtime-status wire protocol and atomic dwell regression tests."""

from __future__ import annotations

import json
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from spray_runtime_protocol import (
    deserialize_runtime_status,
    parse_dwell_response,
    serialize_dwell_command,
    serialize_runtime_status,
)
from test_spray_manual_override import _Param, make_node
from spray_config import SprayConfiguration, SprayMode


def _status():
    return {
        "timestamp_monotonic_s": 12.5,
        "spray_mode": "point",
        "configuration_revision": 7,
        "model_revision": 4,
        "ready": True,
        "commanded_on": False,
        "confirmed_off": True,
        "active_dwell": False,
        "dwell_command_id": 9,
        "dwell_mission_id": "m1",
        "dwell_point_index": 2,
        "dwell_remaining_s": 0.0,
        "last_error": "",
    }


def test_runtime_status_round_trip():
    assert deserialize_runtime_status(serialize_runtime_status(_status())) == _status()


def test_runtime_status_rejects_missing_identity():
    broken = _status()
    del broken["dwell_command_id"]
    try:
        deserialize_runtime_status(json.dumps(broken))
        assert False, "missing field accepted"
    except ValueError:
        pass


def test_atomic_dwell_envelope_round_trip():
    payload = serialize_dwell_command(
        revision=10,
        mission_id="m1",
        point_index=2,
        command_id=3,
        duration_s=0.2,
        configuration_revision=7,
    )
    raw = json.loads(payload)
    assert raw["revision"] == 10
    assert raw["mission_id"] == "m1"
    assert raw["duration_s"] == 0.2


def test_dwell_acceptance_requires_identity():
    try:
        parse_dwell_response('{"accepted":false}')
        assert False, "rejected response accepted"
    except ValueError:
        pass


def _point_node():
    node = make_node()
    node._active_config = SprayConfiguration(
        mode=SprayMode.POINT, revision=7, mission_id="m1"
    )
    node._runtime_status_pub = node._state_pub
    return node


def _response():
    return type("Response", (), {"success": False, "message": ""})()


def _set_command(node, *, revision=10, mission_id="m1", point_index=2, command_id=3, duration_s=0.2, config_revision=7):
    node._params["pending_dwell_command_json"] = _Param(
        json.dumps({
            "revision": revision,
            "mission_id": mission_id,
            "point_index": point_index,
            "command_id": command_id,
            "duration_s": duration_s,
            "configuration_revision": config_revision,
        })
    )


def test_node_rejects_duplicate_and_active_dwell():
    node = _point_node()
    _set_command(node)
    first = node._start_dwell_srv(None, _response())
    assert first.success
    assert parse_dwell_response(first.message)["command_id"] == 3
    duplicate = node._start_dwell_srv(None, _response())
    assert not duplicate.success
    assert "duplicate" in duplicate.message or "stale" in duplicate.message


def test_node_rejects_cancelled_prepare_and_wrong_identity():
    node = _point_node()
    _set_command(node, revision=11)
    node._params["dwell_cancel_revision"] = _Param(12)
    node._cancel_dwell_srv(None, _response())
    cancelled = node._start_dwell_srv(None, _response())
    assert not cancelled.success
    assert "cancelled" in cancelled.message

    node = _point_node()
    _set_command(node, mission_id="wrong")
    wrong = node._start_dwell_srv(None, _response())
    assert not wrong.success and "mission_id" in wrong.message


def test_node_rejects_invalid_duration_point_and_config_revision():
    for changes, expected in [
        ({"duration_s": 0.0}, "duration"),
        ({"point_index": -1}, "point_index"),
        ({"config_revision": 8}, "configuration revision"),
    ]:
        node = _point_node()
        _set_command(node, **changes)
        result = node._start_dwell_srv(None, _response())
        assert not result.success and expected in result.message


def main():
    for name, test in sorted(globals().items()):
        if name.startswith("test_"):
            test()
            print(f"ok {name}")
    print("PASS")


if __name__ == "__main__":
    main()
