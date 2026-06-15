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
