"""Unit tests for GPS_SURVEYED live EKF placement (Epic 2)."""

from __future__ import annotations

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
        "connected": True,
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


# ── EKF-declared origin: deterministic placement ─────────────────────────────
# The bug this fixes: the same staged mission published a path 0.39-1.53 cm
# displaced on every load, in scattered directions, because the translation was
# derived from ONE live pose/global pair. GLOBAL_POSITION_INT quantises lat/lon
# to 1e-7 deg (~1.1 cm here) and the two topics are never sampled together.

_ORIGIN_STATE = {
    "connected": True,
    "pose_received": True,
    "global_position_received": True,
    "gps_fix_received": True,
    "local_pose_age_ms": 10.0,
    "global_position_age_ms": 10.0,
    "gps_fix_age_ms": 10.0,
    "pose_global_skew_ms": 5.0,
    "gps_fix": 6,
    "ekf_origin_received": True,
    "ekf_origin_lat": 13.0720399,
    "ekf_origin_lon": 80.2619564,
}


def _origin_state(**over):
    s = dict(_ORIGIN_STATE)
    s.update(over)
    return s


def test_ekf_origin_placement_is_independent_of_rover_position():
    """THE regression test: same anchor + same origin => identical translation,
    no matter where the rover happens to be sitting."""
    anchor = (13.072071715, 80.261952775)
    pts = [(0.0, 0.0), (1.0, 2.0), (-3.0, 0.5)]

    a, ta = resolve_surveyed_points(
        pts, anchor, _origin_state(pos_n=0.0, pos_e=0.0, lat=13.0720, lon=80.2619))
    b, tb = resolve_surveyed_points(
        pts, anchor, _origin_state(pos_n=57.3, pos_e=-19.8, lat=13.0999, lon=80.2999))

    assert ta == tb, "translation must not depend on the rover sample"
    assert a == b


def test_ekf_origin_placement_is_bit_identical_across_repeated_loads():
    anchor = (13.072071715, 80.261952775)
    pts = [(0.0, 0.0), (0.963, 0.0), (1.922, 0.5)]
    runs = [resolve_surveyed_points(pts, anchor, _origin_state(
        pos_n=i * 0.37, pos_e=-i * 0.11,
        lat=13.0720 + i * 1e-7, lon=80.2619 - i * 1e-7))[0] for i in range(8)]
    for r in runs[1:]:
        assert r == runs[0], "repeated loads must publish the identical path"


def test_live_pair_fallback_DOES_vary_which_is_the_bug():
    """Contrast: without an origin, a 1e-7 deg change in the sample moves the path."""
    anchor = (13.072071715, 80.261952775)
    pts = [(0.0, 0.0)]
    base = _origin_state(ekf_origin_received=False, pos_n=1.0, pos_e=2.0,
                         lat=13.0720399, lon=80.2619564)
    _, t1 = resolve_surveyed_points(pts, anchor, base)
    _, t2 = resolve_surveyed_points(
        pts, anchor, {**base, "lat": 13.0720400})   # one degE7 tick
    assert t1 != t2
    moved = math.hypot(t1[0] - t2[0], t1[1] - t2[1])
    assert 0.005 < moved < 0.02, f"one degE7 tick should move ~1.1 cm, got {moved*100:.2f} cm"


def test_falls_back_to_live_pair_when_origin_absent():
    anchor = (13.072071715, 80.261952775)
    pts = [(0.0, 0.0)]
    out, t = resolve_surveyed_points(
        pts, anchor,
        _origin_state(ekf_origin_received=False, pos_n=1.0, pos_e=2.0,
                      lat=13.0720399, lon=80.2619564))
    assert len(out) == 1 and all(math.isfinite(v) for v in t)


def test_invalid_origin_falls_back_rather_than_raising():
    anchor = (13.072071715, 80.261952775)
    out, _ = resolve_surveyed_points(
        [(0.0, 0.0)], anchor,
        _origin_state(ekf_origin_lat=999.0, pos_n=1.0, pos_e=2.0,
                      lat=13.0720399, lon=80.2619564))
    assert len(out) == 1


def test_origin_path_still_enforces_the_safety_gates():
    """Using the fixed origin must not bypass RTK/connection checks."""
    anchor = (13.072071715, 80.261952775)
    for bad in ({"connected": False}, {"gps_fix": 3}, {"pose_received": False}):
        with pytest.raises(PlacementError):
            resolve_surveyed_points([(0.0, 0.0)], anchor, _origin_state(**bad))
