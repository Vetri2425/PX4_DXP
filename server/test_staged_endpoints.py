"""Tests for the staged path-planning endpoints (align / segments /
plan-and-stage / staged / loaded-path).

Follows the existing server-test convention: call the async route coroutines
directly with a real PathManager pointed at a tmp missions dir, monkeypatching
main.path_mgr and the route-module MISSION_DIR / STAGING_DIR globals.
"""
import os
import asyncio
import json
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(__file__))

import pytest

import main
import routes.path as path_route
import routes.mission as mission_route
from path_manager import PathManager
from offboard_controller import OffboardController
from models import AlignRequest, PathPlanRequest, RefPoint

ezdxf = pytest.importorskip("ezdxf")

# Run the async tests under the anyio plugin (same convention as test_path_api),
# pinned to the asyncio backend.
pytestmark = pytest.mark.anyio


@pytest.fixture
def anyio_backend():
    return "asyncio"


def _write_square_dxf(path, side=2.0):
    """Four connected LINE entities forming a closed square (metres)."""
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 6  # metres
    msp = doc.modelspace()
    pts = [(0, 0), (side, 0), (side, side), (0, side), (0, 0)]
    for a, b in zip(pts[:-1], pts[1:]):
        msp.add_line((a[0], a[1], 0), (b[0], b[1], 0), dxfattribs={"layer": "0"})
    doc.saveas(str(path))


class _OriginNode:
    """Stand-in for RosBridgeNode, exposing only get_origin_health().

    /load-to-controller fails CLOSED on a surveyed mission when the EKF
    local-frame origin cannot be verified, so a surveyed load test must say
    which of the two worlds it is in. Default = healthy.
    """

    def __init__(self, trusted=True, status="OK", detail="origin agrees", delta=0.019):
        self._h = {"status": status, "trusted": trusted, "detail": detail,
                   "delta_m": delta, "threshold_m": 0.30}

    def get_origin_health(self):
        return dict(self._h)


def _trusted_origin(monkeypatch, **kw):
    monkeypatch.setattr(main, "ros_node", _OriginNode(**kw), raising=False)


def _setup(tmp_path, monkeypatch, name="square.dxf"):
    """Real PathManager + tmp MISSION_DIR/STAGING_DIR. Returns (mgr, staging)."""
    _write_square_dxf(tmp_path / name)
    staging = tmp_path / "staging"
    staging.mkdir()
    mgr = PathManager(str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", mgr)
    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(path_route, "STAGING_DIR", str(staging))
    return mgr, str(staging)


# ── /segments ──────────────────────────────────────────────────────────────────

async def test_segments_returns_mark_with_points_and_spray(tmp_path, monkeypatch):
    mgr, _ = _setup(tmp_path, monkeypatch)

    resp = await path_route.path_segments("square.dxf")

    assert resp.name == "square.dxf"
    assert resp.num_segments >= 1
    mark = [s for s in resp.segments if s.type == "MARK"]
    assert mark, "expected at least one MARK segment"
    s = mark[0]
    assert s.spray_on is True
    assert len(s.points) >= 2          # per-segment geometry present
    assert s.source_entity
    assert resp.total_length_m > 0


async def test_segments_with_extensions_shows_transit_roles(tmp_path, monkeypatch):
    mgr, _ = _setup(tmp_path, monkeypatch, name="line.dxf")
    # Replace square with two collinear (open) lines so extensions apply.
    doc = ezdxf.new("R2010"); doc.header["$INSUNITS"] = 6
    m = doc.modelspace()
    m.add_line((0, 0, 0), (2, 0, 0), dxfattribs={"layer": "0"})
    m.add_line((2, 0, 0), (4, 0, 0), dxfattribs={"layer": "0"})
    doc.saveas(str(tmp_path / "line.dxf"))
    mgr.save_extension_config("line.dxf", True, 0.5, 0.5)

    resp = await path_route.path_segments("line.dxf")

    types = [s.type for s in resp.segments]
    roles = [s.segment_role for s in resp.segments]
    assert types == ["TRANSIT", "MARK", "TRANSIT"]
    assert roles[0] == "pre_transit" and roles[-1] == "aft_transit"
    assert resp.segments[0].is_extension is True
    assert resp.segments[0].spray_on is False
    assert resp.segments[1].spray_on is True


async def test_segments_non_dxf_415(tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch)
    (tmp_path / "p.csv").write_text("0,0\n1,0\n", encoding="utf-8")
    with pytest.raises(Exception) as ei:
        await path_route.path_segments("p.csv")
    assert getattr(ei.value, "status_code", None) == 415


# ── /align ─────────────────────────────────────────────────────────────────────

async def test_align_gps_origin_no_staging(tmp_path, monkeypatch):
    _, staging = _setup(tmp_path, monkeypatch)

    resp = await path_route.align_path(
        "square.dxf",
        AlignRequest(origin_gps=[37.7749, -122.4194], sample_points=5),
    )

    assert resp.method == "gps_origin"
    assert resp.origin_gps == [37.7749, -122.4194]
    assert resp.num_waypoints > 0
    assert 0 < len(resp.sample_coords) <= 5
    assert resp.residuals == []
    # Alignment must NOT stage anything.
    assert os.listdir(staging) == []


async def test_align_least_squares_residuals(tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch)
    refs = [
        RefPoint(dxf_x=0.0, dxf_y=0.0, lat=37.000000, lon=-122.000000),
        RefPoint(dxf_x=2.0, dxf_y=0.0, lat=37.000000, lon=-121.999977),
        RefPoint(dxf_x=0.0, dxf_y=2.0, lat=37.000018, lon=-122.000000),
    ]
    resp = await path_route.align_path(
        "square.dxf", AlignRequest(ref_points=refs, sample_points=3),
    )

    assert resp.method == "least_squares"
    assert len(resp.residuals) == 3
    assert all(isinstance(r.residual_m, float) for r in resp.residuals)
    assert resp.rmse_m >= 0.0


async def test_align_requires_inputs_422(tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch)
    with pytest.raises(Exception) as ei:
        await path_route.align_path("square.dxf", AlignRequest())
    assert getattr(ei.value, "status_code", None) == 422


# ── /plan-and-stage  +  /staged/{id} ───────────────────────────────────────────

async def test_plan_and_stage_then_get_staged(tmp_path, monkeypatch):
    _, staging = _setup(tmp_path, monkeypatch)

    plan = await path_route.plan_and_stage(
        "square.dxf",
        PathPlanRequest(source="square.dxf", origin_gps=[37.7749, -122.4194]),
    )
    assert plan.mission_summary is not None
    mid = plan.mission_summary.mission_id
    assert os.path.isfile(os.path.join(staging, f"{mid}.json"))

    staged = await path_route.get_staged_mission(mid)
    assert staged.mission_id == mid
    assert staged.num_waypoints == len(staged.waypoints) > 0
    assert len(staged.spray_flags) == staged.num_waypoints
    assert staged.anchor and staged.anchor["lat"] == 37.7749
    assert staged.segment_runs  # derived MARK/TRANSIT runs
    assert all(set(r) >= {"type", "spray_on", "start_index", "num_points"}
               for r in staged.segment_runs)


async def test_get_staged_reports_placement_mode(tmp_path, monkeypatch):
    """The inspection step must be able to see WHERE the mission will drive.

    The staged artifact records placement_mode, but the GET response dropped it
    while documenting itself as "the exact staged mission artifact". A mission
    that quietly staged LOCAL_NED drives against whatever the EKF origin happens
    to be — a whole-mission misplacement with no symptom in the app — and the
    verify-before-load step had no field to catch it on.

    anchor-is-not-None currently implies GPS_SURVEYED because both derive from
    origin_gps, but that is an implementation coincidence inside _stage_mission,
    not a contract a client should be asked to infer.
    """
    _setup(tmp_path, monkeypatch)

    surveyed = await path_route.plan_and_stage(
        "square.dxf",
        PathPlanRequest(source="square.dxf", origin_gps=[37.7749, -122.4194]),
    )
    staged = await path_route.get_staged_mission(surveyed.mission_summary.mission_id)
    assert staged.placement_mode == "GPS_SURVEYED"

    # No anchor supplied → the mission is local, and says so rather than
    # leaving the client to infer it from a null anchor.
    local = await path_route.plan_and_stage(
        "square.dxf", PathPlanRequest(source="square.dxf")
    )
    staged_local = await path_route.get_staged_mission(local.mission_summary.mission_id)
    assert staged_local.placement_mode == "LOCAL_NED"
    assert staged_local.anchor is None


async def test_staged_metadata_carries_source_detail_for_the_recorder(
    tmp_path, monkeypatch
):
    """The staged artifact must give the bag recorder a real file path.

    Regression: metadata.source was written as a bare filename string while
    tools/bag_autorecord.py read source.get("filepath") from it, so
    manifest.staged_mission.source_file was empty in every bundle and §8
    absolute accuracy could never run. metadata.source stays a string for the
    frontend; the provenance dict now rides alongside it.
    """
    _, staging = _setup(tmp_path, monkeypatch)

    plan = await path_route.plan_and_stage(
        "square.dxf",
        PathPlanRequest(source="square.dxf", origin_gps=[37.7749, -122.4194]),
    )
    mid = plan.mission_summary.mission_id
    with open(os.path.join(staging, f"{mid}.json")) as f:
        meta = json.load(f)["metadata"]

    assert meta["source"] == "square.dxf"  # unchanged shape for the UI

    detail = meta["source_detail"]
    assert os.path.isfile(detail["filepath"]), detail
    assert os.path.basename(detail["filepath"]) == "square.dxf"
    assert detail["extension"] == ".dxf"
    # $INSUNITS = 6 (metres) in the fixture, so 1 unit == 1 m.
    assert detail["unit_scale_m_per_unit"] == pytest.approx(1.0)


async def test_survey_tolerance_is_staged_when_the_operator_sets_it(
    tmp_path, monkeypatch
):
    """The tolerance that judges a run must be the one set when planning it.

    analyze_mission §7 decides FAIL on this number, so it belongs to the
    survey. Staged here, read from the manifest there.
    """
    _, staging = _setup(tmp_path, monkeypatch)

    plan = await path_route.plan_and_stage(
        "square.dxf",
        PathPlanRequest(source="square.dxf", origin_gps=[37.7749, -122.4194],
                        survey_tolerance_m=0.008),
    )
    with open(os.path.join(staging, f"{plan.mission_summary.mission_id}.json")) as f:
        assert json.load(f)["metadata"]["survey_tolerance_m"] == 0.008


async def test_survey_tolerance_absent_means_absent_not_defaulted(
    tmp_path, monkeypatch
):
    """None must NOT become a number here, or every mission would claim an
    explicit tolerance it never chose and the analyser could not tell the
    operator's 2.5 cm from its own fallback."""
    _, staging = _setup(tmp_path, monkeypatch)

    plan = await path_route.plan_and_stage(
        "square.dxf",
        PathPlanRequest(source="square.dxf", origin_gps=[37.7749, -122.4194]),
    )
    with open(os.path.join(staging, f"{plan.mission_summary.mission_id}.json")) as f:
        assert json.load(f)["metadata"]["survey_tolerance_m"] is None


async def test_survey_tolerance_rejects_nonsense(tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch)
    for bad in (0.0, -0.01, 2.0):   # zero, negative, 2 metres
        with pytest.raises(Exception):
            PathPlanRequest(source="square.dxf", survey_tolerance_m=bad)


async def test_get_staged_missing_404(tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch)
    with pytest.raises(Exception) as ei:
        await path_route.get_staged_mission("stg_does_not_exist")
    assert getattr(ei.value, "status_code", None) == 404


# ── /mission/loaded-path ───────────────────────────────────────────────────────

async def test_loaded_path_reports_controller_state(monkeypatch):
    ctrl = OffboardController(None, deque())
    ctrl.load_path(
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
        name="m1",
        spray_flags=[False, True, True],
    )
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)

    resp = await mission_route.loaded_path()

    assert resp.loaded is True
    assert resp.name == "m1"
    assert resp.num_waypoints == 3
    assert resp.num_mark == 2
    assert resp.num_transit == 1
    assert resp.has_spray_flags is True
    assert len(resp.sample_coords) == 3


async def test_loaded_path_empty_controller(monkeypatch):
    monkeypatch.setattr(main, "offboard_ctrl", None)
    resp = await mission_route.loaded_path()
    assert resp.loaded is False
    assert resp.num_waypoints == 0


# ── /segments reuses sidecars (order + overrides) ──────────────────────────────

def _two_separated_lines_dxf(path):
    """Two non-connected LINE entities (won't be shape-grouped)."""
    doc = ezdxf.new("R2010"); doc.header["$INSUNITS"] = 6
    m = doc.modelspace()
    m.add_line((0, 0, 0), (1, 0, 0), dxfattribs={"layer": "0"})
    m.add_line((10, 0, 0), (11, 0, 0), dxfattribs={"layer": "0"})
    doc.saveas(str(path))


async def test_segments_respects_spray_override(tmp_path, monkeypatch):
    mgr, _ = _setup(tmp_path, monkeypatch, name="two.dxf")
    _two_separated_lines_dxf(tmp_path / "two.dxf")
    ids = [e.entity_id for e in mgr.parse_dxf(str(tmp_path / "two.dxf"))]
    mgr.save_entity_overrides("two.dxf", {ids[0]: False})  # entity 0 → TRANSIT

    resp = await path_route.path_segments("two.dxf")

    # Exactly one MARK survives (entity 1); the overridden entity is not MARK.
    marks = [s for s in resp.segments if s.type == "MARK"]
    assert len(marks) == 1


async def test_segments_respects_saved_order(tmp_path, monkeypatch):
    mgr, _ = _setup(tmp_path, monkeypatch, name="two.dxf")
    _two_separated_lines_dxf(tmp_path / "two.dxf")
    ids = [e.entity_id for e in mgr.parse_dxf(str(tmp_path / "two.dxf"))]
    mgr.save_entity_order("two.dxf", [ids[1], ids[0]])  # reversed

    resp = await path_route.path_segments("two.dxf")

    first_mark = next(s for s in resp.segments if s.type == "MARK")
    assert ids[1] in first_mark.source_entity


# ── plan-and-stage guards ──────────────────────────────────────────────────────

async def test_plan_and_stage_rejects_unsupported_fields(tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch)
    with pytest.raises(Exception) as ei:
        await path_route.plan_and_stage(
            "square.dxf", PathPlanRequest(source="square.dxf", order=["A1"]),
        )
    assert getattr(ei.value, "status_code", None) == 422


async def test_plan_and_stage_source_mismatch_422(tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch)
    with pytest.raises(Exception) as ei:
        await path_route.plan_and_stage(
            "square.dxf", PathPlanRequest(source="other.dxf"),
        )
    assert getattr(ei.value, "status_code", None) == 422


async def test_plan_and_stage_missing_file_404(tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch)
    with pytest.raises(Exception) as ei:
        await path_route.plan_and_stage(
            "nope.dxf", PathPlanRequest(source="nope.dxf"),
        )
    assert getattr(ei.value, "status_code", None) == 404


# ── staged: TTL + malformed ────────────────────────────────────────────────────

async def test_get_staged_expired_404(tmp_path, monkeypatch):
    _, staging = _setup(tmp_path, monkeypatch)
    plan = await path_route.plan_and_stage(
        "square.dxf", PathPlanRequest(source="square.dxf", origin_gps=[37.0, -122.0]),
    )
    mid = plan.mission_summary.mission_id
    f = os.path.join(staging, f"{mid}.json")
    old = time.time() - (path_route.STAGING_TTL_S + 100)
    os.utime(f, (old, old))
    with pytest.raises(Exception) as ei:
        await path_route.get_staged_mission(mid)
    assert getattr(ei.value, "status_code", None) == 404


async def test_get_staged_malformed_waypoints_422(tmp_path, monkeypatch):
    _, staging = _setup(tmp_path, monkeypatch)
    bad = os.path.join(staging, "stg_bad.json")
    with open(bad, "w") as fh:
        json.dump({"mission_id": "stg_bad", "waypoints": [[1.0]], "spray_flags": []}, fh)
    with pytest.raises(Exception) as ei:
        await path_route.get_staged_mission("stg_bad")
    assert getattr(ei.value, "status_code", None) == 422


# ── loaded-path edge cases ─────────────────────────────────────────────────────

async def test_loaded_path_no_spray_flags(monkeypatch):
    ctrl = OffboardController(None, deque())
    ctrl.load_path([(0.0, 0.0), (1.0, 0.0)], name="noflags")
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)

    resp = await mission_route.loaded_path()
    assert resp.loaded is True
    assert resp.has_spray_flags is False
    assert resp.num_mark == 0 and resp.num_transit == 0
    assert resp.num_waypoints == 2


async def test_loaded_path_sample_truncation(monkeypatch):
    ctrl = OffboardController(None, deque())
    pts = [(float(i), 0.0) for i in range(100)]
    ctrl.load_path(pts, name="big", spray_flags=[True] * 100)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)

    resp = await mission_route.loaded_path()
    assert resp.num_waypoints == 100
    assert resp.sample_truncated is True
    assert len(resp.sample_coords) == 40   # head 20 + tail 20


# ── must_hit reaches EVERY load route, not just the staged one ─────────────────
#
# Vertex provenance was plumbed through the staged route only. The other three
# entry points dropped it, so a georeferenced survey loaded any other way had
# its near-collinear vertices simplified away again by RPP — the exact bug
# 64c12ff closed, reopened by omission at the call site.


async def test_mission_load_route_passes_must_hit(tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch)
    monkeypatch.setattr(mission_route, "MISSION_DIR", str(tmp_path), raising=False)
    ctrl = OffboardController(None, deque())
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)

    from models import MissionLoadRequest
    await mission_route.load_mission(MissionLoadRequest(path_name="square.dxf"))

    assert ctrl._loaded_must_hit is not None, "provenance dropped by /mission/load"
    assert len(ctrl._loaded_must_hit) == len(ctrl._loaded_pts)
    # Mixed, not uniform: densified points must be False or these are not
    # provenance flags at all. NOTE: this square yields fewer must-hit points
    # than it has corners — but the staged route yields exactly the same ones,
    # which is what this test pins. The under-marking is a separate engine
    # defect, tracked apart from A2; do not "fix" it by relaxing this test.
    assert any(ctrl._loaded_must_hit)
    assert not all(ctrl._loaded_must_hit)


async def test_non_staged_load_matches_staged_provenance(tmp_path, monkeypatch):
    """The non-staged route must agree with the staged one point for point.

    A weaker test ("some flag is set") would pass on provenance that is merely
    plausible. The staged route is the reference implementation, so compare.
    """
    mgr, _ = _setup(tmp_path, monkeypatch)
    ctrl = OffboardController(None, deque())
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)

    staged = mgr.plan_path("square.dxf", summary_only=False)

    from models import MissionLoadRequest
    await mission_route.load_mission(MissionLoadRequest(path_name="square.dxf"))

    assert ctrl._loaded_must_hit == [bool(f) for f in staged["must_hit"]]


async def test_load_path_for_controller_passes_must_hit(tmp_path, monkeypatch):
    mgr, _ = _setup(tmp_path, monkeypatch)
    ctrl = OffboardController(None, deque())

    from mission_loading import load_path_for_controller
    await load_path_for_controller(ctrl, mgr, "square.dxf")

    assert ctrl._loaded_must_hit is not None, "provenance dropped by the async loader"
    assert any(ctrl._loaded_must_hit)


def test_every_controller_load_path_call_site_passes_must_hit():
    """Structural guard: the socket handler cannot be called directly here, and
    a future load route would silently drop provenance again. Assert at the
    source level instead — this is the test the closure checklist asks for."""
    import re
    server_dir = os.path.dirname(os.path.abspath(__file__))
    call_sites = []
    for root, _dirs, files in os.walk(server_dir):
        if "test" in os.path.basename(root):
            continue
        for fname in files:
            if not fname.endswith(".py") or fname.startswith("test_"):
                continue
            fpath = os.path.join(root, fname)
            with open(fpath) as fh:
                src = fh.read()
            for m in re.finditer(r"offboard_ctrl\.load_path\(", src):
                # Slice to the matching close paren (no nested parens in these calls
                # beyond len(...), which carries no commas that confuse the check).
                depth, i = 0, m.end() - 1
                while i < len(src):
                    if src[i] == "(":
                        depth += 1
                    elif src[i] == ")":
                        depth -= 1
                        if depth == 0:
                            break
                    i += 1
                line = src[: m.start()].count("\n") + 1
                call_sites.append((os.path.relpath(fpath, server_dir), line,
                                   src[m.end():i]))

    assert len(call_sites) >= 4, f"expected the known load routes, found {call_sites}"
    missing = [(f, ln) for f, ln, args in call_sites if "must_hit" not in args]
    assert not missing, f"load_path call sites without must_hit: {missing}"


# ── regression: /plan stays light (no per-segment points by default) ────────────

def test_plan_path_default_has_no_segment_points(tmp_path):
    _write_square_dxf(tmp_path / "square.dxf")
    mgr = PathManager(str(tmp_path))
    result = mgr.plan_path("square.dxf")
    assert result["segments"]
    assert all("points" not in s for s in result["segments"])
    # opt-in flag adds them
    result2 = mgr.plan_path("square.dxf", include_segment_points=True)
    assert all("points" in s for s in result2["segments"])


# ── point-mission CSV bridge (mobile flow) ─────────────────────────────────────
#
# The app parses a lat,lon CSV (parse-point-gps-csv) then POSTs the resulting
# point_mission_points to /plan-and-stage with point_source_frame=GPS_SURVEYED.
# The bridge must stage those points as must-hit /path vertices (spray_mode
# "point") so the EXISTING load-to-controller + RPP point-hold model drives them —
# no PointMissionOrchestrator. These pin that bridge and its guard scope.

def _point_req(marks):
    """A GPS point-mission PathPlanRequest with one point per entry in `marks`."""
    from models import PointMissionPoint
    pts = [
        PointMissionPoint(north_m=float(i), east_m=0.0, dwell_s=2.0,
                          source_index=i + 1, mark=m)
        for i, m in enumerate(marks)
    ]
    from models import PathPlanRequest
    return PathPlanRequest(
        source="pts.csv",
        point_source_frame="GPS_SURVEYED",
        origin_gps=[13.0721, 80.2620],
        point_mission_points=pts,
        rotation_deg=0.0,
    )


async def test_point_mission_stages_marks_as_must_hit_dots(tmp_path, monkeypatch):
    _, staging = _setup(tmp_path, monkeypatch)

    plan = await path_route.plan_and_stage("pts.csv", _point_req([True, True, True]))

    assert plan.num_waypoints == 3
    assert plan.must_hit == [True, True, True]     # every mark is a dwell target
    assert plan.spray_flags == [True, True, True]
    mid = plan.mission_summary.mission_id

    with open(os.path.join(staging, f"{mid}.json")) as f:
        staged = json.load(f)
    assert staged["waypoints"] == [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]]
    assert staged["must_hit"] == [True, True, True]
    assert staged["spray_flags"] == [True, True, True]
    # Forced to point so the spray node runs its dwell FSM, not continuous.
    assert staged["spray_session"]["mode"] == "point"
    # Surveyed → must re-bind into the live EKF at start.
    assert staged["placement_mode"] == "GPS_SURVEYED"
    assert staged["anchor"]["lat"] == 13.0721 and staged["anchor"]["lon"] == 80.2620
    assert staged["origin_gps"] == [13.0721, 80.2620]


async def test_point_mission_unmarked_point_is_transit_never_sprayed(
    tmp_path, monkeypatch
):
    """mark=False ⇒ must_hit AND spray both False.

    The spray node dwells on EVERY must-hit vertex regardless of the spray bit
    (spray_controller_node.py: coords = must-hit vertices), so an unmarked point
    must NOT be must-hit or it would be sprayed. It stays a plain transit vertex.
    """
    _, staging = _setup(tmp_path, monkeypatch)

    plan = await path_route.plan_and_stage("pts.csv", _point_req([True, False, True]))

    assert plan.must_hit == [True, False, True]
    assert plan.spray_flags == [True, False, True]
    with open(os.path.join(staging, f"{plan.mission_summary.mission_id}.json")) as f:
        staged = json.load(f)
    assert staged["must_hit"] == [True, False, True]
    assert staged["spray_flags"] == [True, False, True]


async def test_point_mission_load_to_controller_gets_must_hit_points(
    tmp_path, monkeypatch
):
    """End-to-end: stage → load-to-controller hands the points + must-hit to the
    controller (the existing must-hit /path load path, unchanged)."""
    _, staging = _setup(tmp_path, monkeypatch)
    ctrl = OffboardController(None, deque())
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    _trusted_origin(monkeypatch)

    plan = await path_route.plan_and_stage("pts.csv", _point_req([True, True, True]))
    mid = plan.mission_summary.mission_id

    from models import LoadMissionRequest
    resp = await path_route.load_mission_to_controller(LoadMissionRequest(mission_id=mid))

    assert resp["status"] == "success"
    assert resp["num_waypoints"] == 3
    assert resp["placement_mode"] == "GPS_SURVEYED"
    assert ctrl._loaded_pts == [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert ctrl._loaded_must_hit == [True, True, True]


async def test_surveyed_load_refuses_when_the_ekf_origin_is_untrustworthy(
    tmp_path, monkeypatch
):
    """Fail closed at the COMMITMENT point, not just at start.

    Same staged mission as the test above; the only difference is the origin
    verdict. If the gate were missing this would return 200 and the operator
    would carry a mission that is silently displaced by the origin delta.
    """
    from fastapi import HTTPException
    from models import LoadMissionRequest

    _setup(tmp_path, monkeypatch)
    ctrl = OffboardController(None, deque())
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    _trusted_origin(monkeypatch)     # healthy while staging

    plan = await path_route.plan_and_stage("pts.csv", _point_req([True, True, True]))
    mid = plan.mission_summary.mission_id

    # ...then the FCU reboots between staging and loading.
    _trusted_origin(monkeypatch, trusted=False, status="INCONSISTENT",
                    detail="off by 2.147 m (limit 0.30 m)", delta=2.147)
    with pytest.raises(HTTPException) as exc:
        await path_route.load_mission_to_controller(LoadMissionRequest(mission_id=mid))
    assert exc.value.status_code == 409
    assert "INCONSISTENT" in exc.value.detail
    assert "2.147" in exc.value.detail
    assert ctrl._loaded_pts is None, "nothing may reach the controller"


async def test_surveyed_load_refuses_when_the_origin_cannot_be_probed(
    tmp_path, monkeypatch
):
    """No ROS bridge => no way to verify => refuse. An unavailable probe must
    never read as a pass; that is the fail-open hole this shape of bug lives in."""
    from fastapi import HTTPException
    from models import LoadMissionRequest

    _setup(tmp_path, monkeypatch)
    ctrl = OffboardController(None, deque())
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    _trusted_origin(monkeypatch)

    plan = await path_route.plan_and_stage("pts.csv", _point_req([True, True, True]))
    mid = plan.mission_summary.mission_id

    monkeypatch.setattr(main, "ros_node", None, raising=False)
    with pytest.raises(HTTPException) as exc:
        await path_route.load_mission_to_controller(LoadMissionRequest(mission_id=mid))
    assert exc.value.status_code == 503
    assert ctrl._loaded_pts is None


async def test_local_ned_load_is_not_gated_on_the_ekf_origin(tmp_path, monkeypatch):
    """Scope guard: a LOCAL_NED mission never touches the EKF origin, so a bad
    origin must not block it. Without this, the gate would be over-broad."""
    from models import LoadMissionRequest

    _setup(tmp_path, monkeypatch)
    ctrl = OffboardController(None, deque())
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "ros_node", None, raising=False)

    plan = await path_route.plan_and_stage(
        "square.dxf", PathPlanRequest(source="square.dxf")
    )
    mid = plan.mission_summary.mission_id
    resp = await path_route.load_mission_to_controller(LoadMissionRequest(mission_id=mid))

    assert resp["status"] == "success"
    assert resp["placement_mode"] == "LOCAL_NED"


async def test_plan_and_stage_ignores_points_without_gps_frame(tmp_path, monkeypatch):
    """Guard scope: point_mission_points WITHOUT the GPS_SURVEYED frame must NOT
    hijack the planner — it falls through to the ordinary DXF/line engine.

    Proven by pointing `name` at the real square.dxf: if the bridge wrongly fired
    it would stage the two bogus points; instead the engine plans the square.
    """
    _, staging = _setup(tmp_path, monkeypatch)  # writes square.dxf
    from models import PathPlanRequest, PointMissionPoint
    req = PathPlanRequest(
        source="square.dxf",
        point_mission_points=[
            PointMissionPoint(north_m=0.0, east_m=0.0, dwell_s=2.0, source_index=1),
            PointMissionPoint(north_m=9.0, east_m=9.0, dwell_s=2.0, source_index=2),
        ],
        # no point_source_frame, no origin_gps → not a point mission
    )
    plan = await path_route.plan_and_stage("square.dxf", req)
    # The square planned by the engine has many densified waypoints, never the 2
    # raw points, and its mode stays continuous (default).
    assert plan.num_waypoints > 2
    with open(os.path.join(staging, f"{plan.mission_summary.mission_id}.json")) as f:
        staged = json.load(f)
    assert staged["spray_session"]["mode"] == "continuous"


# ── A15: plan-and-stage must forward the shape controls it accepts ─────────────

async def test_plan_and_stage_forwards_the_arc_fit_controls(tmp_path, monkeypatch):
    """A15 regression. PathPlanRequest accepts fit_arcs + 3 companions +
    close_shape, /api/path/plan forwarded them, and plan-and-stage silently
    DROPPED all five — so the preview could honour a setting the driven mission
    ignored, with HTTP 200 and no warning.

    Measured on curve_6_points-1.csv before the fix:
      /api/path/plan  fit_arcs=false -> 98 wp, 8/8 must-hit, 0.00 cm from stations
      plan-and-stage  fit_arcs=false -> 97 wp, 2/8 must-hit, 1.95 cm mean

    Fails if the fix is wrong: on pre-fix source none of the five keys reach
    plan_path, so every assertion below raises KeyError. It cannot pass by
    accident — the values asserted are non-default and distinct from each other.
    """
    mgr, _ = _setup(tmp_path, monkeypatch)
    seen = {}
    real = mgr.plan_path

    def spy(name, **kwargs):
        seen.update(kwargs)
        return real(name, **kwargs)

    monkeypatch.setattr(mgr, "plan_path", spy)

    await path_route.plan_and_stage(
        "square.dxf",
        PathPlanRequest(
            source="square.dxf",
            origin_gps=[37.7749, -122.4194],
            fit_arcs=False,
            fit_arcs_rms_m=0.031,
            fit_arcs_corner_deg=41.0,
            fit_arcs_max_dev_m=0.017,
            close_shape=True,
        ),
    )

    assert seen["fit_arcs"] is False
    assert seen["fit_arcs_rms_m"] == 0.031
    assert seen["fit_arcs_corner_deg"] == 41.0
    assert seen["fit_arcs_max_dev_m"] == 0.017
    assert seen["close_shape"] is True


async def test_omitting_the_arc_fit_controls_still_means_auto(tmp_path, monkeypatch):
    """Scope guard for the fix above — the dangerous half.

    path_manager reads fit_arcs=None as AUTO ("on for a survey CSV"). If the
    forwarding had been written to send a concrete default instead of None, every
    call would carry an EXPLICIT value and silently override that auto rule for
    every mission — the exact trap models.py already warns about. So an omitted
    field must arrive as None, not False.
    """
    mgr, _ = _setup(tmp_path, monkeypatch)
    seen = {}
    real = mgr.plan_path

    def spy(name, **kwargs):
        seen.update(kwargs)
        return real(name, **kwargs)

    monkeypatch.setattr(mgr, "plan_path", spy)

    await path_route.plan_and_stage(
        "square.dxf",
        PathPlanRequest(source="square.dxf", origin_gps=[37.7749, -122.4194]),
    )

    assert seen["fit_arcs"] is None, "omitted must stay None so AUTO survives"
    assert seen["fit_arcs_rms_m"] is None
    assert seen["fit_arcs_corner_deg"] is None
    assert seen["fit_arcs_max_dev_m"] is None
    assert seen["close_shape"] is False
