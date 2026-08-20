#!/usr/bin/env python3
"""Pure regression tests for the independent spray fail-closed lease."""

import json

import pytest

from spray_safety_lease import (
    LeaseValidationError,
    SprayLeaseMonitor,
    SpraySafetyLease,
    lease_from_json,
    lease_to_json,
)


def _lease(*, allow_on: bool = True, backend: str = "mavlink_actuator"):
    return SpraySafetyLease(
        allow_on=allow_on,
        command_seq=7,
        backend=backend,
        actuator_set_index=1,
        off_value=-1.0,
        servo_instance=1,
        off_pwm_us=0,
    )


def test_fresh_explicit_on_lease_is_the_only_state_that_suppresses_off():
    monitor = SprayLeaseMonitor(timeout_s=0.35)
    assert monitor.off_reason(1.0) == "no controller lease"
    monitor.observe(lease_to_json(_lease()), 1.0)
    assert monitor.off_reason(1.34) is None


def test_stale_on_lease_fails_closed():
    monitor = SprayLeaseMonitor(timeout_s=0.35)
    monitor.observe(lease_to_json(_lease()), 10.0)
    assert "stale" in monitor.off_reason(10.351)


def test_fresh_false_lease_fails_closed_immediately():
    monitor = SprayLeaseMonitor()
    monitor.observe(lease_to_json(_lease(allow_on=False)), 2.0)
    assert monitor.off_reason(2.0) == "controller lease denies ON"


def test_invalid_message_revokes_a_previous_fresh_on_lease():
    monitor = SprayLeaseMonitor()
    monitor.observe(lease_to_json(_lease()), 2.0)
    assert monitor.off_reason(2.1) is None
    monitor.invalidate("invalid controller lease: bad JSON")
    assert monitor.off_reason(2.1) == "invalid controller lease: bad JSON"


@pytest.mark.parametrize(
    "patch",
    [
        {"allow_on": 1},
        {"backend": "unknown"},
        {"actuator_set_index": 0},
        {"off_value": float("nan")},
        {"off_value": "-1.0"},
        {"servo_instance": 0},
        {"off_pwm_us": 9999},
    ],
)
def test_malformed_or_unsafe_lease_is_rejected(patch):
    raw = json.loads(lease_to_json(_lease()))
    raw.update(patch)
    with pytest.raises(LeaseValidationError):
        lease_from_json(json.dumps(raw))


def test_backend_configuration_round_trips_for_independent_off():
    lease = _lease(backend="mavlink_servo_pwm")
    assert lease_from_json(lease_to_json(lease)) == lease
