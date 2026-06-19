import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from mission_placement import PlacementError, resolve_surveyed_points


ANCHOR = (13.072066, 80.261956)
SOURCE = [(0.0, -0.035), (1.0, -0.035), (1.0, 0.965)]


def healthy_state():
    return {
        "pose_received": True,
        "global_position_received": True,
        "gps_fix_received": True,
        "local_pose_age_ms": 20.0,
        "global_position_age_ms": 15.0,
        "gps_fix_age_ms": 100.0,
        "pose_global_skew_ms": 5.0,
        "gps_fix": 6,
        "pos_n": 7.4629,
        "pos_e": -0.9070,
        "lat": 13.0720864,
        "lon": 80.2619557,
    }


def test_field_survey_translation_and_compensated_first_point():
    resolved, translation = resolve_surveyed_points(SOURCE, ANCHOR, healthy_state())

    assert translation == pytest.approx((5.192, -0.875), abs=0.02)
    assert resolved[0] == pytest.approx((5.192, -0.910), abs=0.02)


def test_uniform_translation_preserves_all_waypoint_deltas():
    resolved, _ = resolve_surveyed_points(SOURCE, ANCHOR, healthy_state())

    source_deltas = [
        (b[0] - a[0], b[1] - a[1]) for a, b in zip(SOURCE, SOURCE[1:])
    ]
    resolved_deltas = [
        (b[0] - a[0], b[1] - a[1]) for a, b in zip(resolved, resolved[1:])
    ]
    assert resolved_deltas == pytest.approx(source_deltas)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda s: s.update(pose_received=False), "local pose has not been received"),
        (
            lambda s: s.update(global_position_received=False),
            "fused global position has not been received",
        ),
        (
            lambda s: s.update(gps_fix_received=False),
            "GPS fix information has not been received",
        ),
        (lambda s: s.update(local_pose_age_ms=501.0), "local pose is stale"),
        (
            lambda s: s.update(global_position_age_ms=501.0),
            "fused global position is stale",
        ),
        (
            lambda s: s.update(gps_fix_age_ms=501.0),
            "GPS fix information is stale",
        ),
        (lambda s: s.update(pose_global_skew_ms=101.0), "not sufficiently aligned"),
        (lambda s: s.update(gps_fix=5), "below RTK_FIXED"),
        (lambda s: s.update(pos_n=math.nan), "non-finite"),
        (lambda s: s.update(lat=math.inf), "non-finite"),
    ],
)
def test_surveyed_placement_fails_closed_for_bad_telemetry(mutation, message):
    state = healthy_state()
    mutation(state)

    with pytest.raises(PlacementError, match=message):
        resolve_surveyed_points(SOURCE, ANCHOR, state)


@pytest.mark.parametrize("anchor", [None, (math.nan, 80.0), (91.0, 80.0)])
def test_surveyed_placement_rejects_invalid_anchor(anchor):
    with pytest.raises(PlacementError):
        resolve_surveyed_points(SOURCE, anchor, healthy_state())
