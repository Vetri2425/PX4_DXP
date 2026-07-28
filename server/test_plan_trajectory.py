"""POST /api/path/plan-trajectory — the app-planned trajectory endpoint.

The app fits the geometry and decides the order; this endpoint may ONLY insert
points along what it is given. Every test here is either that contract or one of
the silent wrong-paint failures the endpoint exists to make impossible.

Convention follows test_staged_endpoints.py: call the async route coroutines
directly with the route module's STAGING_DIR pointed at a tmp dir.
"""
import json
import math
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(__file__))

import pytest
from fastapi import HTTPException
from pydantic import ValidationError

import main
import routes.path as path_route
from models import PlanTrajectoryRequest
from offboard_controller import OffboardController

pytestmark = pytest.mark.anyio


@pytest.fixture
def anyio_backend():
    return "asyncio"


ORIGIN = [13.0721, 80.2620]


class _OriginNode:
    """Stand-in for RosBridgeNode — /load-to-controller fails closed without it."""

    def get_origin_health(self):
        return {"status": "OK", "trusted": True, "detail": "origin agrees",
                "delta_m": 0.019, "threshold_m": 0.30}


@pytest.fixture
def staging(tmp_path, monkeypatch):
    d = tmp_path / "staging"
    d.mkdir()
    monkeypatch.setattr(path_route, "STAGING_DIR", str(d))
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "ros_node", _OriginNode(), raising=False)
    return str(d)


def _run(kind, points, speed=None, label=None):
    return {
        "kind": kind,
        "points": points,
        "speed_m_s": speed if speed is not None else (0.35 if kind == "mark" else 0.50),
        "label": label,
    }


def _req(runs, **kw):
    body = {"mission_name": "haddows_road", "origin_gps": ORIGIN, "runs": runs}
    body.update(kw)
    return PlanTrajectoryRequest(**body)


def _mark_travel_mark():
    """The request's own §3.1 example: two roads joined by one travel leg."""
    return [
        _run("mark", [[0.0, 0.0], [10.0, 0.0]], label="Haddows Road"),
        _run("travel", [[10.0, 0.0], [10.0, 20.0]]),
        _run("mark", [[10.0, 20.0], [20.0, 20.0]], label="Kilpauk Garden Road"),
    ]


def _staged(staging_dir, plan):
    with open(os.path.join(staging_dir, f"{plan.mission_summary.mission_id}.json")) as f:
        return json.load(f)


# ── 1. run structure survives staging ─────────────────────────────────────────

async def test_mark_travel_mark_stages_four_runs_in_order(staging):
    plan = await path_route.plan_trajectory(_req(_mark_travel_mark()))

    staged = _staged(staging, plan)
    kinds = [r["type"] for r in path_route._spray_runs(
        staged["waypoints"], staged["spray_flags"])]
    assert kinds == ["MARK", "TRANSIT", "MARK", "TRANSIT"]   # incl. the run-out

    assert [e.kind for e in plan.run_echo] == ["mark", "travel", "mark", "travel"]
    assert [e.label for e in plan.run_echo] == [
        "Haddows Road", None, "Kilpauk Garden Road", "run-out"]
    assert [e.generated for e in plan.run_echo] == [False, False, False, True]
    # The client reconciles num_waypoints against the echo, so the run-out has
    # to be in both or its arithmetic is off by one.
    assert sum(e.num_points for e in plan.run_echo) == plan.num_waypoints == 538
    assert plan.num_segments == 4


async def test_response_matches_the_engine_the_app_measured(staging):
    """The figures in the change request came from a real run of this engine.

    If any of them move, either the densifier changed or a geometry pass that
    is supposed to be off came back on — both are reasons to stop and look.
    """
    plan = await path_route.plan_trajectory(_req(_mark_travel_mark()))

    assert plan.num_waypoints == 538
    assert plan.mark_length_m == 20.0
    assert plan.transit_length_m == 20.1          # 20.0 leg + 0.1 run-out
    assert plan.total_length_m == 40.1
    assert [e.num_points for e in plan.run_echo] == [201, 135, 201, 1]
    assert plan.alignment_metadata["method"] == "gps_origin"
    assert plan.alignment_metadata["rmse"] == 0.0
    assert plan.alignment_metadata["scale"] == 1.0


# ── 2. densification is the only transformation ───────────────────────────────

async def test_densifies_mark_at_5cm_and_travel_at_15cm(staging):
    plan = await path_route.plan_trajectory(_req(_mark_travel_mark()))

    steps = {True: [], False: []}
    wp, flags = plan.merged_waypoints, plan.spray_flags
    for i in range(len(wp) - 1):
        if flags[i] == flags[i + 1]:            # skip the boundary junctions
            steps[flags[i]].append(math.dist(wp[i], wp[i + 1]))

    assert max(steps[True]) <= 0.05 + 1e-9
    assert min(steps[True]) > 0.04
    assert max(steps[False]) <= 0.15 + 1e-9
    assert min(steps[False]) > 0.14


async def test_spacing_is_taken_from_the_request(staging):
    plan = await path_route.plan_trajectory(
        _req(_mark_travel_mark(), line_spacing=0.10, transit_spacing=0.50))
    wp, flags = plan.merged_waypoints, plan.spray_flags
    mark_steps = [math.dist(wp[i], wp[i + 1]) for i in range(len(wp) - 1)
                  if flags[i] and flags[i + 1]]
    assert max(mark_steps) <= 0.10 + 1e-9
    assert max(mark_steps) > 0.05


# ── 3-6. validation: reject what the file flow accepts ────────────────────────

async def test_two_adjacent_mark_runs_are_refused(staging):
    """The rule that matters. Contiguous spray_flags read as ONE region
    downstream, so without this the rover paints straight across the gap."""
    runs = [
        _run("mark", [[0.0, 0.0], [10.0, 0.0]]),
        _run("mark", [[50.0, 0.0], [60.0, 0.0]]),
    ]
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(_req(runs))
    assert exc.value.status_code == 422
    assert "runs[0] and runs[1]" in exc.value.detail
    assert "paint across the gap" in exc.value.detail


async def test_travel_leg_not_touching_its_neighbour_is_refused(staging):
    runs = [
        _run("mark", [[0.0, 0.0], [10.0, 0.0]]),
        _run("travel", [[22.4, 0.0], [30.0, 0.0]]),   # starts 12.4 m adrift
        _run("mark", [[30.0, 0.0], [40.0, 0.0]]),
    ]
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(_req(runs))
    assert exc.value.status_code == 422
    assert "runs[1] travel leg does not start where runs[0] ends" in exc.value.detail
    assert "12.4 m" in exc.value.detail


async def test_travel_leg_not_reaching_the_next_mark_is_refused(staging):
    runs = [
        _run("mark", [[0.0, 0.0], [10.0, 0.0]]),
        _run("travel", [[10.0, 0.0], [17.6, 0.0]]),
        _run("mark", [[30.0, 0.0], [40.0, 0.0]]),    # 12.4 m past the leg's end
    ]
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(_req(runs))
    assert exc.value.status_code == 422
    assert "runs[2]" in exc.value.detail and "12.4 m" in exc.value.detail


async def test_run_with_one_point_is_refused_at_both_layers(staging):
    # The HTTP layer: the model itself will not build.
    with pytest.raises(ValidationError):
        _req([_run("mark", [[0.0, 0.0]])])

    # And the route's own check, for a model built programmatically.
    req = _req(_mark_travel_mark())
    req.runs[2].points = [(10.0, 20.0)]
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(req)
    assert exc.value.status_code == 422
    assert "runs[2] has 1 point; a run needs at least 2" in exc.value.detail


async def test_non_finite_coordinate_is_refused(staging):
    runs = _mark_travel_mark()
    runs[0]["points"] = [[0.0, 0.0], [5.0, 0.0], [float("nan"), 0.0], [10.0, 0.0]]
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(_req(runs))
    assert exc.value.status_code == 422
    assert "runs[0].points[2] is not finite" in exc.value.detail


async def test_infinite_coordinate_is_refused(staging):
    """inf, not just nan: hypot(inf) is inf, which compares False against the
    gap threshold, so an unchecked inf sails past the continuity rules."""
    runs = _mark_travel_mark()
    runs[0]["points"] = [[0.0, 0.0], [float("inf"), 0.0]]
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(_req(runs))
    assert exc.value.status_code == 422
    assert "not finite" in exc.value.detail


async def test_trajectory_with_no_mark_runs_is_refused(staging):
    runs = [_run("travel", [[0.0, 0.0], [10.0, 0.0]])]
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(_req(runs))
    assert exc.value.status_code == 422
    assert "no marking runs" in exc.value.detail


async def test_placeholder_origin_is_refused(staging):
    """Rule 7 reuses _assert_origin_gps_usable, so the fixture anchors the file
    flow already refuses are refused here too."""
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(
            _req(_mark_travel_mark(), origin_gps=[13.0, 80.0]))
    assert exc.value.status_code == 422
    assert "placeholder" in exc.value.detail


async def test_out_of_range_origin_is_refused(staging):
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(
            _req(_mark_travel_mark(), origin_gps=[913.0, 80.0]))
    assert exc.value.status_code == 422
    assert "latitude/longitude bounds" in exc.value.detail


async def test_over_limit_waypoint_estimate_is_a_422_not_a_timeout(staging):
    """Mirrors path_manager's DXF guard, which only runs `if is_dxf` — a CSV
    mission spends the whole budget and surfaces an opaque 504 instead."""
    runs = [_run("mark", [[0.0, 0.0], [10000.0, 0.0]])]
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(_req(runs, max_waypoints=1000))
    assert exc.value.status_code == 422
    assert "Too many waypoints" in exc.value.detail
    assert "1000" in exc.value.detail


# ── 7-8. the two fusions/drops the file flow performs silently ────────────────

async def test_two_mark_runs_sharing_an_endpoint_are_not_fused(staging):
    """group_shapes (hardcoded True in plan_path) would chain these into one
    run. run_echo has to show two, or the client cannot tell."""
    runs = [
        _run("mark", [[0.0, 0.0], [10.0, 0.0]], label="A"),
        _run("travel", [[10.0, 0.0], [10.0, 5.0]]),
        _run("mark", [[10.0, 5.0], [20.0, 5.0]], label="B"),
        _run("travel", [[20.0, 5.0], [20.0, 0.0]]),
        _run("mark", [[20.0, 0.0], [30.0, 0.0]], label="C"),
    ]
    plan = await path_route.plan_trajectory(_req(runs))
    marks = [e for e in plan.run_echo if e.kind == "mark"]
    assert [e.label for e in marks] == ["A", "B", "C"]


def test_run_echo_sees_runs_that_spray_flags_fuse():
    """run_echo must come from the segment list, never from spray_flags.

    Two adjacent MARK segments produce one contiguous block of True flags, so a
    flag-derived view (_spray_runs, and the staged segment_runs it builds)
    reports ONE run where there are two. That view is correct for its own
    purpose and cannot verify structure — a fusion is precisely what it is blind
    to.

    Exercised as a unit because rule 1 makes this shape unreachable over HTTP
    today: two adjacent mark runs are a 422. The helper is the guard for the day
    that changes, or group_shapes comes back on.
    """
    from types import SimpleNamespace
    from path_engine.core import PathSegment, PlannedPath, SegmentType

    a = PathSegment(segment_type=SegmentType.MARK,
                    points=[(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)],
                    segment_id=0, source_entity="A")
    b = PathSegment(segment_type=SegmentType.MARK,
                    points=[(10.0, 0.0), (15.0, 0.0), (20.0, 0.0)],
                    segment_id=1, source_entity="B")
    plan = PlannedPath(
        segments=[a, b],
        merged_waypoints=a.points + b.points,
        spray_flags=[True] * 6,
        must_hit=[True] * 6,
    )
    runs = [SimpleNamespace(label="first pass"), SimpleNamespace(label="second pass")]

    echo = path_route._trajectory_run_echo(runs, plan)
    assert [e.kind for e in echo] == ["mark", "mark"]
    assert [e.label for e in echo] == ["first pass", "second pass"]
    assert [e.num_points for e in echo] == [3, 3]

    fused = [r for r in path_route._spray_runs(plan.merged_waypoints, plan.spray_flags)
             if r["spray_on"]]
    assert len(fused) == 1, (
        "premise of this test: the flag view fuses them into one run")


async def test_coincident_mark_runs_are_never_dropped_silently(staging):
    """Engine step 1c drops a coincident MARK and has no flag to disable it.
    It must at least be VISIBLE — a deliberate double pass vanishing without a
    word is the failure mode."""
    runs = [
        _run("mark", [[0.0, 0.0], [10.0, 0.0]], label="pass 1"),
        _run("travel", [[10.0, 0.0], [10.0, 1.0]]),
        _run("travel", [[10.0, 1.0], [0.0, 0.0]]),
        _run("mark", [[0.0, 0.0], [10.0, 0.0]], label="pass 2"),
    ]
    plan = await path_route.plan_trajectory(_req(runs))

    warnings = plan.warnings or []
    assert any("coincident mark runs" in w for w in warnings), warnings
    if len([e for e in plan.run_echo if e.kind == "mark"]) < 2:
        assert any("duplicate geometry" in w for w in warnings), (
            "a run was dropped and nothing said so")


async def test_warnings_are_capped_so_they_stay_readable(staging):
    """A systematic problem in a 4000-run import must not return 4000 warnings —
    that is the same as returning none."""
    runs = []
    n = 0.0
    for k in range(40):
        runs.append(_run("mark", [[n, 0.0], [n + 1.0, 0.0]], label=f"m{k}"))
        runs.append(_run("travel", [[n + 1.0, 0.0], [n, 0.0]]))
        runs.append(_run("mark", [[n, 0.0], [n + 1.0, 0.0]], label=f"dup{k}"))
        runs.append(_run("travel", [[n + 1.0, 0.0], [n + 2.0, 0.0]]))
        n += 2.0
    plan = await path_route.plan_trajectory(_req(runs))

    coincident = [w for w in (plan.warnings or []) if "coincident mark runs" in w]
    assert len(coincident) <= 10
    assert any("more coincident mark run pair(s)" in w for w in (plan.warnings or []))


async def test_empty_mission_name_is_refused(staging):
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(_req(_mark_travel_mark(), mission_name="/"))
    assert exc.value.status_code == 422
    assert "mission_name" in exc.value.detail


async def test_a_realistically_large_mission_plans_promptly(staging):
    """1.1 km of road ≈ the app's stated worst case. The validation pass must
    stay linear in the run count — an all-pairs coincidence scan would make this
    quadratic long before the densifier is the bottleneck."""
    import time

    runs = []
    n = 0.0
    for k in range(300):
        runs.append(_run("mark", [[n, 0.0], [n + 3.0, 0.0]], label=f"seg{k}"))
        runs.append(_run("travel", [[n + 3.0, 0.0], [n + 4.0, 0.0]]))
        n += 4.0

    t0 = time.perf_counter()
    plan = await path_route.plan_trajectory(_req(runs))
    elapsed = time.perf_counter() - t0

    assert len([e for e in plan.run_echo if e.kind == "mark"]) == 300
    assert plan.mark_length_m == pytest.approx(900.0, abs=0.01)
    assert elapsed < 5.0, f"took {elapsed:.1f}s — well inside the 15 s budget"


# ── 9. the contract: only insert points, never move them ──────────────────────

async def test_fitted_arc_is_not_re_fitted_or_re_smoothed(staging):
    """A surveyed/fitted arc arrives as a dense polyline. corner_smooth_radius_m
    and fit_arcs are not gated for this geometry in the file flow, so both are
    pinned off here — every output point must lie ON the polyline we sent."""
    r = 5.0
    arc = [[r * math.cos(t * math.pi / 40), r * math.sin(t * math.pi / 40)]
           for t in range(21)]                      # 21 points over a quarter arc
    runs = [_run("mark", arc, label="roundabout")]

    plan = await path_route.plan_trajectory(_req(runs))

    def dist_to_polyline(p, poly):
        best = float("inf")
        for a, b in zip(poly[:-1], poly[1:]):
            abn, abe = b[0] - a[0], b[1] - a[1]
            L2 = abn * abn + abe * abe
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, (
                (p[0] - a[0]) * abn + (p[1] - a[1]) * abe) / L2))
            best = min(best, math.dist(p, (a[0] + t * abn, a[1] + t * abe)))
        return best

    # Excluding the terminal run-out, which is a deliberate spur past the end.
    body = plan.merged_waypoints[:-1] if not plan.spray_flags[-1] else plan.merged_waypoints
    assert max(dist_to_polyline(p, arc) for p in body) < 1e-9

    # And every surveyed vertex survives as a must-hit point, in order.
    kept = [wp for wp, hit in zip(plan.merged_waypoints, plan.must_hit) if hit]
    for vertex in arc:
        assert any(math.dist(vertex, k) < 1e-9 for k in kept), (
            f"surveyed vertex {vertex} was dropped or demoted")


async def test_every_sent_vertex_comes_back_must_hit(staging):
    """The client blocks the load on a must_hit that is False at a vertex it
    sent, so this is a hard requirement, not a nicety."""
    plan = await path_route.plan_trajectory(_req(_mark_travel_mark()))
    wp, hit = plan.merged_waypoints, plan.must_hit
    for vertex in [(0.0, 0.0), (10.0, 0.0), (10.0, 20.0), (20.0, 20.0)]:
        idxs = [i for i, p in enumerate(wp) if math.dist(p, vertex) < 1e-9]
        assert idxs, f"{vertex} missing from the plan"
        assert any(hit[i] for i in idxs), f"{vertex} came back not must-hit"


async def test_run_order_is_never_reordered(staging):
    """optimize_order=False. The app decided the order; a nearest-neighbour
    re-sort would silently discard it (and it WOULD re-sort: the second mark is
    closer to the start than the first)."""
    runs = [
        _run("mark", [[20.0, 0.0], [30.0, 0.0]], label="far"),
        _run("travel", [[30.0, 0.0], [0.0, 0.0]]),
        _run("mark", [[0.0, 0.0], [10.0, 0.0]], label="near"),
    ]
    plan = await path_route.plan_trajectory(_req(runs))
    assert [e.label for e in plan.run_echo if e.kind == "mark"] == ["far", "near"]
    assert plan.merged_waypoints[0] == [20.0, 0.0]


# ── 10. ground truth rides in the artifact ────────────────────────────────────

async def test_ground_truth_is_staged_against_the_right_vertices(staging):
    plan = await path_route.plan_trajectory(_req(
        _mark_travel_mark(),
        ground_truth=[
            {"run_index": 0, "point_index": 0, "lat": 13.0721, "lon": 80.2620},
            {"run_index": 2, "point_index": 1, "lat": 13.07219, "lon": 80.26218},
        ],
    ))
    staged = _staged(staging, plan)
    assert staged["survey_ground_truth"] == [
        {"north_m": 0.0, "east_m": 0.0, "lat": 13.0721, "lon": 80.2620},
        {"north_m": 20.0, "east_m": 20.0, "lat": 13.07219, "lon": 80.26218},
    ]


async def test_no_ground_truth_leaves_the_key_absent(staging):
    """Absent, not null — a file-based artifact stays byte-identical to what it
    was before this key existed."""
    plan = await path_route.plan_trajectory(_req(_mark_travel_mark()))
    assert "survey_ground_truth" not in _staged(staging, plan)


async def test_ground_truth_index_out_of_range_is_refused(staging):
    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(_req(
            _mark_travel_mark(),
            ground_truth=[{"run_index": 9, "point_index": 0,
                           "lat": 13.0721, "lon": 80.2620}]))
    assert exc.value.status_code == 422
    assert "run_index 9 is out of range" in exc.value.detail

    with pytest.raises(HTTPException) as exc:
        await path_route.plan_trajectory(_req(
            _mark_travel_mark(),
            ground_truth=[{"run_index": 0, "point_index": 7,
                           "lat": 13.0721, "lon": 80.2620}]))
    assert exc.value.status_code == 422
    assert "point_index 7 is out of range" in exc.value.detail


def test_analyze_mission_prefers_staged_ground_truth():
    """§8 absolute accuracy must not go silent on a mission with no source file."""
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
    from analyze_mission import _surveyed_latlon_from_source

    manifest = {"plan_provenance": {
        "source_file": None,
        "survey_ground_truth": [
            {"north_m": 0.0, "east_m": 0.0, "lat": 13.0721, "lon": 80.2620},
            {"north_m": 10.0, "east_m": 0.0, "lat": 13.07219, "lon": 80.2620},
        ],
    }}
    pts, provenance = _surveyed_latlon_from_source(manifest)
    assert pts == [(13.0721, 80.2620), (13.07219, 80.2620)]
    assert "staged survey_ground_truth" in provenance


def test_analyze_mission_still_reads_the_source_file_when_there_is_one(tmp_path):
    """The fallback is untouched — existing DXF and survey-CSV missions are
    unaffected by the new preference."""
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
    from analyze_mission import _surveyed_latlon_from_source

    csv = tmp_path / "survey.csv"
    csv.write_text("Name,Latitude,Longitude\nP1,13.0721,80.2620\nP2,13.0722,80.2621\n")
    pts, provenance = _surveyed_latlon_from_source(
        {"plan_provenance": {"source_file": str(csv)}})
    assert pts == [(13.0721, 80.2620), (13.0722, 80.2621)]
    assert "survey CSV" in provenance


# ── 11. the rest of the pipeline is reused verbatim ───────────────────────────

async def test_staged_mission_loads_and_starts(staging, monkeypatch):
    from models import LoadMissionRequest, MissionStartRequest
    from routes.mission import start_mission

    ctrl = OffboardController(None, deque())
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)

    plan = await path_route.plan_trajectory(_req(_mark_travel_mark()))
    mid = plan.mission_summary.mission_id

    resp = await path_route.load_mission_to_controller(LoadMissionRequest(mission_id=mid))
    assert resp["status"] == "success"
    assert resp["placement_mode"] == "GPS_SURVEYED"
    assert resp["num_waypoints"] == 538
    assert ctrl.loaded_path_summary()["mission_id"] == mid

    started = {}

    async def _fake_start(auto_origin=False):
        started["called"] = True
        return True, "running"

    # start_mission's identity gate compares the caller's mission_id against
    # what the controller holds — that is the half this test is proving, so the
    # actual drive is stubbed out.
    monkeypatch.setattr(ctrl, "start_async", _fake_start)
    res = await start_mission(MissionStartRequest(mission_id=mid))
    assert started.get("called") is True, "the identity gate refused the mission"
    assert res["message"] == "running"


async def test_staged_artifact_is_readable_through_the_staged_endpoint(staging):
    plan = await path_route.plan_trajectory(_req(_mark_travel_mark()))
    got = await path_route.get_staged_mission(plan.mission_summary.mission_id)
    assert got.num_waypoints == 538
    assert got.metadata["source"] == "haddows_road"
    assert got.alignment_metadata["method"] == "gps_origin"


async def test_spray_session_defaults_match_the_file_flow(staging):
    plan = await path_route.plan_trajectory(_req(_mark_travel_mark()))
    assert _staged(staging, plan)["spray_session"] == {
        "mode": "continuous",
        "dash_on_distance_m": None,
        "dash_off_distance_m": None,
        "dash_start_state": "on",
        "point_dwell_s": 1.0,
        "point_arrival_tolerance_m": 0.10,
    }


async def test_dash_mode_rides_to_the_staged_artifact(staging):
    plan = await path_route.plan_trajectory(_req(
        _mark_travel_mark(), spray_mode="dash",
        dash_on_distance_m=2.0, dash_off_distance_m=3.0, dash_start_state="off"))
    session = _staged(staging, plan)["spray_session"]
    assert session["mode"] == "dash"
    assert session["dash_on_distance_m"] == 2.0
    assert session["dash_off_distance_m"] == 3.0
    assert session["dash_start_state"] == "off"


# ── 12. the DXF flow must not be able to reach any of this ────────────────────

def test_path_engine_group_shapes_default_is_still_true():
    """group_shapes=False must be reachable ONLY from this endpoint. The DXF
    flow depends on the default; flipping it would fuse nothing and interleave
    multi-shape drawings."""
    import inspect
    from path_engine.engine import PathEngine
    assert inspect.signature(PathEngine.__init__).parameters[
        "group_shapes"].default is True


def test_plan_and_stage_response_has_no_run_echo():
    """run_echo lives on a SUBCLASS. Adding it to PathPlanResponse would put a
    `"run_echo": null` into every DXF-flow response body."""
    from models import PathPlanResponse, PlanTrajectoryResponse
    assert "run_echo" not in PathPlanResponse.model_fields
    assert "run_echo" in PlanTrajectoryResponse.model_fields


def test_plan_trajectory_request_cannot_express_the_file_flow_traps():
    """The five silent failures are unreachable because the model has no field
    for them — not because the caller is expected to avoid them."""
    fields = set(PlanTrajectoryRequest.model_fields)
    for trap in ("optimize", "corner_smooth_radius_m", "fit_arcs", "close_shape",
                 "close_loop", "layer_mapping", "enable_path_extensions",
                 "compensate_spray", "ref_points"):
        assert trap not in fields, f"{trap} must not be settable on this endpoint"
