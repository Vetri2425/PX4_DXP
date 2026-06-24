"""Unit tests for GPS_SURVEYED placement safety evaluation."""

from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from gps_safety import GpsSurveyedSafetyParams, evaluate_gps_surveyed_safety
from mission_placement import PlacementError
from test_mission_placement import ANCHOR, SOURCE, healthy_state


def _surveyed_cfg(**overrides) -> GpsSurveyedSafetyParams:
    return GpsSurveyedSafetyParams(**overrides)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda s: s.update(connected=False), "FCU disconnected"),
        (lambda s: s.update(gps_fix=5), "below required"),
        (lambda s: s.update(global_position_age_ms=501.0), "fused global position is stale"),
        (lambda s: s.update(local_pose_age_ms=501.0), "local pose is stale"),
        (lambda s: s.update(gps_fix_age_ms=501.0), "GPS fix information is stale"),
        (lambda s: s.update(pose_global_skew_ms=101.0), "not sufficiently aligned"),
        (lambda s: s.update(pose_received=False), "local pose has not been received"),
    ],
)
def test_start_gate_rejects_unsafe_telemetry(mutation, message):
    state = healthy_state()
    mutation(state)
    verdict = evaluate_gps_surveyed_safety(state, ANCHOR, SOURCE, _surveyed_cfg())
    assert not verdict.ok
    assert message in verdict.reason


def test_missing_anchor_rejected():
    verdict = evaluate_gps_surveyed_safety(healthy_state(), None, SOURCE, _surveyed_cfg())
    assert not verdict.ok
    assert "anchor" in verdict.reason.lower()


def test_healthy_surveyed_passes():
    verdict = evaluate_gps_surveyed_safety(healthy_state(), ANCHOR, SOURCE, _surveyed_cfg())
    assert verdict.ok
    assert verdict.gps_safety_ok
    assert verdict.recovery_ready


def test_recovery_debounce_requires_stable_period():
    import time

    params = _surveyed_cfg(recovery_stable_s=2.0)
    state = healthy_state()
    verdict = evaluate_gps_surveyed_safety(
        state, ANCHOR, SOURCE, params, recovery_since=time.monotonic(), paused=True
    )
    assert verdict.ok
    assert not verdict.recovery_ready
    assert verdict.gps_safety_state == "recovering"