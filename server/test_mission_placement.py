"""Unit tests for GPS_SURVEYED live EKF placement (Epic 2).

2026-07-27: every fixture here is now PHYSICALLY CONSISTENT — the rover's
lat/lon, its local pos_n/pos_e and the declared EKF origin describe one frame.
They did not used to be (a rover at local (0,0) was given a lat/lon 4.4 m from
its own declared origin), which is exactly the state the new consistency gate
exists to catch. A fixture that cannot happen on the rig cannot defend the code
that runs on it.

The lat/lon literals below are built by ``_offset`` — a deliberately DIFFERENT
implementation from the ``latlon_to_ned`` under test (small-angle
equirectangular vs PX4's azimuthal-equidistant, same sphere). Verified to agree
to <= 0.02 mm over the distances used here, so it is an independent ruler, not
a mirror of the code being graded.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

import mission_placement
from mission_placement import PlacementError, resolve_surveyed_points

# PX4 geo.h CONSTANTS_RADIUS_OF_EARTH — the sphere PX4's local frame lives on.
_R = 6371000.0


def _offset(lat0: float, lon0: float, north_m: float, east_m: float):
    """Independent ruler: (lat, lon) `north_m`/`east_m` from (lat0, lon0).

    Small-angle equirectangular on PX4's sphere. NOT path_engine.ned.
    """
    lat = lat0 + math.degrees(north_m / _R)
    mid = math.radians((lat0 + lat) / 2.0)
    lon = lon0 + math.degrees(east_m / (_R * math.cos(mid)))
    return lat, lon


ANCHOR = (13.072066, 80.261956)
SOURCE = [(0.0, -0.035), (1.0, -0.035), (1.0, 0.965)]

# The 2026-06 field sample, plus the EKF origin it implies. Placement through
# this origin must reproduce the translation the live pair used to produce.
_FIELD_ORIGIN = (13.072019284527872, 80.26196407384334)


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
        "ekf_origin_received": True,
        "ekf_origin_lat": _FIELD_ORIGIN[0],
        "ekf_origin_lon": _FIELD_ORIGIN[1],
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

_ORIGIN = (13.0720399, 80.2619564)

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
    "ekf_origin_lat": _ORIGIN[0],
    "ekf_origin_lon": _ORIGIN[1],
    # Rover parked exactly on the origin — the trivially consistent sample.
    "pos_n": 0.0,
    "pos_e": 0.0,
    "lat": _ORIGIN[0],
    "lon": _ORIGIN[1],
}


def _origin_state(**over):
    s = dict(_ORIGIN_STATE)
    s.update(over)
    return s


def _consistent_state(north: float, east: float, **over):
    """State for a rover genuinely sitting at local (north, east) of _ORIGIN."""
    lat, lon = _offset(_ORIGIN[0], _ORIGIN[1], north, east)
    return _origin_state(pos_n=north, pos_e=east, lat=lat, lon=lon, **over)


def test_ekf_origin_placement_is_independent_of_rover_position():
    """THE regression test: same anchor + same origin => identical translation,
    no matter where the rover happens to be sitting.

    Both samples are physically real positions in the SAME frame — 60 m apart —
    so this can only pass if the translation ignores the rover sample entirely.
    """
    anchor = (13.072071715, 80.261952775)
    pts = [(0.0, 0.0), (1.0, 2.0), (-3.0, 0.5)]

    a, ta = resolve_surveyed_points(pts, anchor, _consistent_state(0.0, 0.0))
    b, tb = resolve_surveyed_points(pts, anchor, _consistent_state(57.3, -19.8))

    assert ta == tb, "translation must not depend on the rover sample"
    assert a == b


def test_ekf_origin_placement_is_bit_identical_across_repeated_loads():
    """Determinism is preserved by the new gate: the healthy path is unchanged."""
    anchor = (13.072071715, 80.261952775)
    pts = [(0.0, 0.0), (0.963, 0.0), (1.922, 0.5)]
    runs = [
        resolve_surveyed_points(
            pts, anchor, _consistent_state(i * 0.37, -i * 0.11)
        )[0]
        for i in range(8)
    ]
    for r in runs[1:]:
        assert r == runs[0], "repeated loads must publish the identical path"


def test_live_pair_fallback_DOES_vary_which_is_the_bug(monkeypatch):
    """Contrast: without an origin, a 1e-7 deg change in the sample moves the path.

    Only reachable with the fail-closed requirement explicitly disabled.
    """
    monkeypatch.setattr(mission_placement, "ORIGIN_REQUIRE_DECLARED", False)
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


def test_live_pair_fallback_is_opt_in_only(monkeypatch):
    """Same state, both settings: default refuses, opt-out places."""
    anchor = (13.072071715, 80.261952775)
    state = _origin_state(ekf_origin_received=False, pos_n=1.0, pos_e=2.0,
                          lat=13.0720399, lon=80.2619564)

    with pytest.raises(PlacementError, match="not trustworthy"):
        resolve_surveyed_points([(0.0, 0.0)], anchor, state)

    monkeypatch.setattr(mission_placement, "ORIGIN_REQUIRE_DECLARED", False)
    out, t = resolve_surveyed_points([(0.0, 0.0)], anchor, state)
    assert len(out) == 1 and all(math.isfinite(v) for v in t)


def test_origin_path_still_enforces_the_safety_gates():
    """Using the fixed origin must not bypass RTK/connection checks."""
    anchor = (13.072071715, 80.261952775)
    for bad in ({"connected": False}, {"gps_fix": 3}, {"pose_received": False}):
        with pytest.raises(PlacementError):
            resolve_surveyed_points([(0.0, 0.0)], anchor, _consistent_state(1.0, 2.0, **bad))


# ── Fail closed on an untrustworthy origin (field bug, 2026-07-27) ───────────
# Two runs at 14:47 were placed 2.15 m and 2.25 m off the surveyed line because
# the server served an EKF origin cached from a dead EKF session. There is no
# operator-visible symptom until the rover moves, so placement must refuse.

def test_stale_origin_from_a_dead_ekf_session_refuses():
    """The 14:47 failure, reconstructed: the rover is really at local (0,0) of
    an origin 2.15 m away from the one the server still has cached."""
    anchor = (13.072071715, 80.261952775)
    true_origin = _offset(_ORIGIN[0], _ORIGIN[1], -1.90, 1.00)   # |d| = 2.15 m
    lat, lon = true_origin                                        # rover at its (0,0)

    state = _origin_state(pos_n=0.0, pos_e=0.0, lat=lat, lon=lon)
    with pytest.raises(PlacementError) as exc:
        resolve_surveyed_points([(0.0, 0.0)], anchor, state)

    msg = str(exc.value)
    assert "not trustworthy" in msg
    # The refusal must be actionable: it names the measured displacement, the
    # limit, and the implied origin — not just "computer says no".
    assert "2.147 m" in msg, f"refusal must name the measured error: {msg}"
    assert "dN +1.900" in msg and "dE -1.000" in msg, msg
    assert "limit 0.30 m" in msg, msg


def test_the_measured_1510_field_fault_is_rejected_with_its_real_numbers():
    """Ground truth = the operator's own field measurement, computed off-rig:

        declared gp_origin : 13.07205210, 80.26197050
        implied live frame : 13.07204472, 80.26197415
        delta              : dN +0.816 m, dE -0.396 m = 0.91 m

    The operator used the WGS84 meridional radius for that north scale; this
    code deliberately uses PX4's sphere (see origin_health's module docstring),
    which is 0.51 % larger — 4 mm at this separation. Asserting to 1 cm both
    proves the implementation and documents that the models are NOT the same.
    """
    from origin_health import INCONSISTENT, evaluate_origin_health

    declared = (13.07205210, 80.26197050)
    implied = (13.07204472, 80.26197415)
    state = _origin_state(
        ekf_origin_lat=declared[0], ekf_origin_lon=declared[1],
        pos_n=0.0, pos_e=0.0, lat=implied[0], lon=implied[1],
    )

    health = evaluate_origin_health(state)
    assert health.status == INCONSISTENT
    assert health.trusted is False
    assert health.delta_n_m == pytest.approx(0.816, abs=0.01)
    assert health.delta_e_m == pytest.approx(-0.396, abs=0.01)
    assert health.delta_m == pytest.approx(0.91, abs=0.01)
    assert health.implied[0] == pytest.approx(implied[0], abs=1e-7)
    assert health.implied[1] == pytest.approx(implied[1], abs=1e-7)

    with pytest.raises(PlacementError, match="not trustworthy"):
        resolve_surveyed_points([(0.0, 0.0)], (13.072071715, 80.261952775), state)


def test_a_healthy_rig_reading_is_nowhere_near_the_threshold():
    """The measured healthy value on this rig is 1.9-2.0 cm. The gate is 0.30 m.

    This is the false-positive guard: if the projection model or the sign
    convention were wrong, a healthy sample 57 m from the origin would show a
    distance-proportional error (0.51 % north = 29 cm at 57 m for the WGS84
    model) and this test would fail.
    """
    from origin_health import OK, evaluate_origin_health

    health = evaluate_origin_health(_consistent_state(57.3, -19.8))
    assert health.status == OK and health.trusted
    assert health.delta_m < 0.01, f"healthy far-from-origin sample read {health.delta_m:.3f} m"


def test_origin_absent_refusal_names_the_cause_recorded_by_ros_node():
    """The invalidation reason travels from ros_node into the operator message."""
    anchor = (13.072071715, 80.261952775)
    state = _origin_state(
        ekf_origin_received=False,
        ekf_origin_invalid_reason="FCU link re-established (connected False->True)",
        pos_n=1.0, pos_e=2.0, lat=13.0720399, lon=80.2619564,
    )
    with pytest.raises(PlacementError, match="FCU link re-established"):
        resolve_surveyed_points([(0.0, 0.0)], anchor, state)


def test_invalid_declared_origin_refuses_instead_of_falling_back():
    """Previously this quietly used the non-deterministic live pair."""
    anchor = (13.072071715, 80.261952775)
    with pytest.raises(PlacementError, match="not trustworthy"):
        resolve_surveyed_points(
            [(0.0, 0.0)], anchor,
            _origin_state(ekf_origin_lat=999.0, pos_n=1.0, pos_e=2.0,
                          lat=13.0720399, lon=80.2619564))
