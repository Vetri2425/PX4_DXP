"""The EKF-origin consistency gate itself, plus its two observability surfaces.

Ground truth here is deliberately NOT `path_engine.ned` (the module under
test): positions are built with a small-angle equirectangular formula on PX4's
sphere, which is a different implementation of the same frame and agrees with
the azimuthal-equidistant projection to <= 0.02 mm at these distances.

Each test below is written so that a WRONG implementation fails it:
  * wrong projection model (WGS84 instead of PX4's sphere) -> the far-from-
    origin healthy case picks up a distance-proportional 0.51 % north error
    (29 cm at 57 m) and trips the 0.30 m gate.
  * sign flip in the delta -> the implied-origin assertions in
    test_mission_placement (real field numbers) land on the mirrored point.
  * gate compares the wrong pair -> the boundary tests below straddle it.
  * fail-open on missing data -> the UNVERIFIABLE tests would return OK.
"""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from origin_health import (
    INCONSISTENT,
    NO_ORIGIN,
    OK,
    ORIGIN_INVALID,
    UNVERIFIABLE,
    evaluate_origin_health,
)

_R = 6371000.0
_ORIGIN = (13.0720399, 80.2619564)


def _offset(lat0, lon0, north_m, east_m):
    lat = lat0 + math.degrees(north_m / _R)
    mid = math.radians((lat0 + lat) / 2.0)
    return lat, lon0 + math.degrees(east_m / (_R * math.cos(mid)))


def _state(north=0.0, east=0.0, delta_n=0.0, delta_e=0.0, **over):
    """Rover truly at local (north, east) of _ORIGIN, with the DECLARED origin
    displaced (delta_n, delta_e) metres from the true one.

    Sign convention, matching the field report: the declared origin sitting
    NORTH of the true one makes PX4's reported local position read further
    south than the declared origin predicts, i.e. delta_n positive. The
    2026-07-27 15:10 fault had declared 0.82 m north of implied and reported
    dN +0.816.
    """
    lat, lon = _offset(_ORIGIN[0], _ORIGIN[1], north, east)
    declared = _offset(_ORIGIN[0], _ORIGIN[1], delta_n, delta_e)
    s = {
        "connected": True,
        "pose_received": True,
        "global_position_received": True,
        "local_pose_age_ms": 10.0,
        "global_position_age_ms": 10.0,
        "pose_global_skew_ms": 5.0,
        "lat": lat, "lon": lon, "pos_n": north, "pos_e": east,
        "ekf_origin_received": True,
        "ekf_origin_lat": declared[0], "ekf_origin_lon": declared[1],
    }
    s.update(over)
    return s


# ── The measurement ──────────────────────────────────────────────────────────

def test_a_perfect_origin_reads_zero():
    h = evaluate_origin_health(_state())
    assert h.status == OK and h.trusted
    assert h.delta_m == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("north,east", [(0.0, 0.0), (57.3, -19.8), (-12.75, 4.02)])
def test_a_healthy_origin_stays_healthy_at_any_distance_from_it(north, east):
    """Distance-proportional model error is the failure mode that would make
    this gate misfire on a healthy rig. 57 m out, a WGS84 north scale would
    read ~0.29 m and trip the 0.30 m limit."""
    h = evaluate_origin_health(_state(north, east))
    assert h.status == OK, h.detail
    assert h.delta_m < 0.001, f"{north},{east} read {h.delta_m*1000:.3f} mm"


def test_the_gate_straddles_its_threshold():
    below = evaluate_origin_health(_state(delta_n=0.25))
    above = evaluate_origin_health(_state(delta_n=0.35))
    assert below.status == OK and below.delta_m == pytest.approx(0.25, abs=0.002)
    assert above.status == INCONSISTENT and above.delta_m == pytest.approx(0.35, abs=0.002)
    assert above.trusted is False


def test_the_error_is_reported_as_a_signed_local_frame_vector():
    h = evaluate_origin_health(_state(delta_n=1.50, delta_e=-0.80))
    assert h.delta_n_m == pytest.approx(1.50, abs=0.005)
    assert h.delta_e_m == pytest.approx(-0.80, abs=0.005)
    assert h.delta_m == pytest.approx(math.hypot(1.5, 0.8), abs=0.005)


def test_the_implied_origin_is_the_true_one():
    """The operator's field procedure reports an implied origin; it must land on
    the frame's real origin, not on the declared one."""
    h = evaluate_origin_health(_state(north=8.0, east=-3.0, delta_n=2.0, delta_e=1.0))
    assert h.implied[0] == pytest.approx(_ORIGIN[0], abs=1e-7)
    assert h.implied[1] == pytest.approx(_ORIGIN[1], abs=1e-7)
    assert h.declared != h.implied


def test_the_verdict_does_not_depend_on_where_the_rover_stands():
    """A real origin error is the same error whether the rover is on the origin
    or 60 m away. If it scaled with position, the model would be wrong."""
    near = evaluate_origin_health(_state(0.0, 0.0, delta_n=1.20))
    far = evaluate_origin_health(_state(57.3, -19.8, delta_n=1.20))
    assert far.delta_m == pytest.approx(near.delta_m, abs=0.002)


# ── Fail closed on anything that is not a measured agreement ─────────────────

@pytest.mark.parametrize(
    ("over", "expected"),
    [
        ({"ekf_origin_received": False}, NO_ORIGIN),
        ({"ekf_origin_lat": math.nan}, ORIGIN_INVALID),
        ({"ekf_origin_lat": 999.0}, ORIGIN_INVALID),
        ({"pose_received": False}, UNVERIFIABLE),
        ({"global_position_received": False}, UNVERIFIABLE),
        ({"local_pose_age_ms": 5000.0}, UNVERIFIABLE),
        ({"local_pose_age_ms": None}, UNVERIFIABLE),
        ({"global_position_age_ms": 5000.0}, UNVERIFIABLE),
        ({"pose_global_skew_ms": 900.0}, UNVERIFIABLE),
        ({"pose_global_skew_ms": None}, UNVERIFIABLE),
        ({"lat": math.nan}, UNVERIFIABLE),
        ({"pos_n": math.inf}, UNVERIFIABLE),
        ({"lat": 0.0, "lon": 0.0}, UNVERIFIABLE),
        ({"lat": 200.0}, UNVERIFIABLE),
    ],
)
def test_nothing_but_a_measured_agreement_is_trusted(over, expected):
    h = evaluate_origin_health(_state(**over))
    assert h.status == expected, h.detail
    assert h.trusted is False


def test_an_empty_state_is_not_trusted():
    h = evaluate_origin_health({})
    assert h.status == NO_ORIGIN and h.trusted is False


def test_the_verdict_serialises_with_its_numbers():
    d = evaluate_origin_health(_state(delta_n=2.0)).as_dict()
    assert d["status"] == INCONSISTENT and d["trusted"] is False
    for k in ("declared_lat", "declared_lon", "implied_lat", "implied_lon",
              "delta_n_m", "delta_e_m", "delta_m", "threshold_m", "detail"):
        assert d[k] is not None, k
    assert d["delta_m"] == pytest.approx(2.0, abs=0.005)


# ── Observability surfaces ───────────────────────────────────────────────────

@pytest.mark.anyio
async def test_health_origin_endpoint_fails_closed_without_a_ros_node(monkeypatch):
    import main
    from routes.system import health_origin

    monkeypatch.setattr(main, "ros_node", None, raising=False)
    out = await health_origin()
    assert out["trusted"] is False and out["status"] == UNVERIFIABLE


@pytest.mark.anyio
async def test_health_origin_endpoint_returns_the_nodes_verdict(monkeypatch):
    import main
    from routes.system import health_origin

    class _Node:
        def get_origin_health(self):
            return {"status": INCONSISTENT, "trusted": False, "delta_m": 0.91}

    monkeypatch.setattr(main, "ros_node", _Node(), raising=False)
    out = await health_origin()
    assert out["status"] == INCONSISTENT and out["delta_m"] == 0.91


@pytest.mark.anyio
async def test_telemetry_latest_carries_the_same_verdict(monkeypatch):
    """The operator's dashboard number and the placement decision must come from
    one evaluation, or they can disagree and the operator trusts the wrong one."""
    import main
    from routes.telemetry import telemetry_latest

    bad = _state(delta_n=2.0)

    class _Node:
        def get_state(self):
            return dict(bad)

    monkeypatch.setattr(main, "ros_node", _Node(), raising=False)
    monkeypatch.setattr(main, "offboard_ctrl", None, raising=False)
    t = await telemetry_latest()
    assert t.origin_status == INCONSISTENT
    assert t.origin_trusted is False
    assert t.origin_delta_m == pytest.approx(2.0, abs=0.005)


@pytest.fixture
def anyio_backend():
    return "asyncio"
