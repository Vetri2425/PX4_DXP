import os
import sys

# Ensure server directory is in python path
sys.path.insert(0, os.path.dirname(__file__))

import pytest
from fastapi import HTTPException
from types import SimpleNamespace
from models import MissionState, PathPlanRequest, PathPreviewResponse, RefPoint
from routes.path import path_entities, plan_path, preview_path
import main
from path_manager import PathManager


def test_path_manager_preview_returns_bounds_and_local_ned_points(tmp_path):
    mission_file = tmp_path / "line.csv"
    mission_file.write_text("0,0\n1.5,-0.25\n2.0,0.75\n", encoding="utf-8")

    mgr = PathManager(str(tmp_path))
    preview = mgr.preview_path("line.csv")

    assert preview.name == "line.csv"
    assert preview.frame == "local_ned"
    assert preview.num_points == 3
    assert preview.bounds is not None
    assert preview.bounds.north_min == 0.0
    assert preview.bounds.north_max == 2.0
    assert preview.bounds.east_min == -0.25
    assert preview.bounds.east_max == 0.75
    assert preview.waypoints[1].north == 1.5
    assert preview.waypoints[1].east == -0.25
    assert all(pt.spray is True for pt in preview.waypoints)


def test_path_manager_preview_preserves_dxf_spray_flags(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakeEngine:
        def plan_file(self, filepath):
            assert filepath == str(mission_file)
            return SimpleNamespace(
                merged_waypoints=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
                spray_flags=[True, False, True],
            )

    import path_engine

    monkeypatch.setattr(path_engine, "PathEngine", FakeEngine)

    mgr = PathManager(str(tmp_path))
    preview = mgr.preview_path("field.dxf")

    assert [pt.spray for pt in preview.waypoints] == [True, False, True]


def test_path_manager_preview_caches_uploaded_file_result(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")
    calls = {"count": 0}

    class FakeEngine:
        def plan_file(self, filepath):
            calls["count"] += 1
            return SimpleNamespace(
                merged_waypoints=[(0.0, 0.0), (1.0, 0.0)],
                spray_flags=[True, True],
            )

    import path_engine

    monkeypatch.setattr(path_engine, "PathEngine", FakeEngine)

    mgr = PathManager(str(tmp_path))
    first = mgr.preview_path("field.dxf")
    second = mgr.preview_path("field.dxf")

    assert calls["count"] == 1
    assert first is second


@pytest.mark.anyio
async def test_preview_api_returns_path_preview(monkeypatch):
    class FakePathManager:
        def preview_path(self, name):
            return PathPreviewResponse(
                name=name,
                num_points=2,
                bounds={
                    "north_min": 0.0,
                    "north_max": 1.0,
                    "east_min": 0.0,
                    "east_max": 0.5,
                },
                waypoints=[
                    {"north": 0.0, "east": 0.0, "spray": True},
                    {"north": 1.0, "east": 0.5, "spray": True},
                ],
            )

    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await preview_path("square_2x2")

    assert data.name == "square_2x2"
    assert data.frame == "local_ned"
    assert data.num_points == 2
    assert data.bounds.north_max == 1.0
    assert data.waypoints[1].east == 0.5


@pytest.mark.anyio
async def test_preview_api_missing_path_is_404(monkeypatch):
    class FakePathManager:
        def preview_path(self, name):
            raise FileNotFoundError(f"Path not found: {name!r}")

    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    with pytest.raises(HTTPException) as exc:
        await preview_path("missing.csv")

    assert exc.value.status_code == 404
    assert "missing.csv" in exc.value.detail


@pytest.mark.anyio
async def test_entities_api_returns_line_preview_points(tmp_path, monkeypatch):
    mission_file = tmp_path / "field.dxf"
    mission_file.write_text("0\nEOF\n", encoding="utf-8")

    class FakePathManager:
        def parse_dxf(self, filepath):
            assert filepath == str(mission_file)
            return [
                SimpleNamespace(
                    entity_id="A1",
                    entity_type="LINE",
                    layer="MARKING",
                    color=7,
                    geometry={
                        "start": (0.0, 0.0),
                        "end": (1.0, 2.0),
                    },
                    is_mark=lambda: True,
                )
            ]

    import routes.path as path_route

    monkeypatch.setattr(path_route, "MISSION_DIR", str(tmp_path))
    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    data = await path_entities("field.dxf")

    assert data.name == "field.dxf"
    assert data.frame == "local_ned"
    assert data.num_entities == 1
    assert data.bounds.north_max == 1.0
    assert data.bounds.east_max == 2.0
    ent = data.entities[0]
    assert ent.entity_id == "A1"
    assert ent.entity_type == "LINE"
    assert ent.length_m == pytest.approx(2.236, abs=0.001)
    assert [pt.model_dump() for pt in ent.preview_points] == [
        {"north": 0.0, "east": 0.0},
        {"north": 1.0, "east": 2.0},
    ]


def test_path_plan_request_extension_defaults_are_safe():
    req = PathPlanRequest(source="soccer_field_penalty_area.dxf")

    assert req.enable_path_extensions is False
    assert req.pre_extension_m == 0.5
    assert req.aft_extension_m == 0.5


def test_path_plan_request_rejects_negative_extensions():
    from pydantic import ValidationError

    for field in ("pre_extension_m", "aft_extension_m"):
        with pytest.raises(ValidationError):
            PathPlanRequest(source="soccer_field_penalty_area.dxf", **{field: -0.1})


@pytest.mark.anyio
async def test_plan_api_passes_extension_flags(monkeypatch):
    captured = {}

    class FakePathManager:
        def plan_path(self, source, summary_only=False, **kwargs):
            captured["source"] = source
            captured["summary_only"] = summary_only
            captured["kwargs"] = kwargs
            return {
                "source": source,
                "num_waypoints": 2,
                "num_segments": 1,
                "mark_length_m": 1.0,
                "transit_length_m": 0.0,
                "total_length_m": 1.0,
                "segments": [],
                "merged_waypoints": [(0.0, 0.0), (1.0, 0.0)],
                "spray_flags": [True, True],
                "alignment_metadata": {},
                "warnings": [],
            }

    monkeypatch.setattr(main, "path_mgr", FakePathManager())

    req = PathPlanRequest(
        source="soccer_field_penalty_area.dxf",
        enable_path_extensions=True,
        pre_extension_m=0.25,
        aft_extension_m=0.75,
    )

    data = await plan_path(req)

    assert data.source == "soccer_field_penalty_area.dxf"
    assert captured["kwargs"]["enable_path_extensions"] is True
    assert captured["kwargs"]["pre_extension_m"] == 0.25
    assert captured["kwargs"]["aft_extension_m"] == 0.75


def test_path_manager_passes_extension_flags_to_engine(tmp_path, monkeypatch):
    mission_file = tmp_path / "line.csv"
    mission_file.write_text("north,east\n0,0\n1,0\n", encoding="utf-8")
    captured = {}

    class FakeEngine:
        def __init__(self, **kwargs):
            captured["engine_kwargs"] = kwargs

        def plan_file(self, filepath, **kwargs):
            captured["plan_filepath"] = filepath
            captured["plan_kwargs"] = kwargs
            return SimpleNamespace(
                num_waypoints=2,
                segments=[],
                total_mark_length=1.0,
                total_transit_length=0.0,
                total_length=1.0,
                alignment_metadata={},
                merged_waypoints=[(0.0, 0.0), (1.0, 0.0)],
                spray_flags=[True, True],
            )

    class FakeValidator:
        def __init__(self, *args, **kwargs):
            pass

        def validate(self, plan):
            return []

        def validate_or_raise(self, plan):
            return []

    import path_engine.engine as engine_module
    import path_engine.validator as validator_module

    monkeypatch.setattr(engine_module, "PathEngine", FakeEngine)
    monkeypatch.setattr(validator_module, "PathValidator", FakeValidator)

    mgr = PathManager(str(tmp_path))
    result = mgr.plan_path(
        "line.csv",
        enable_path_extensions=True,
        pre_extension_m=0.25,
        aft_extension_m=0.75,
    )

    assert result["source"] == "line.csv"
    assert captured["engine_kwargs"]["enable_path_extensions"] is True
    assert captured["engine_kwargs"]["pre_extension_m"] == 0.25
    assert captured["engine_kwargs"]["aft_extension_m"] == 0.75

@pytest.mark.anyio
async def test_plan_api_dxf_ref_points():
    if main.path_mgr is None:
        main.path_mgr = PathManager(main.MISSION_DIR)

    req = PathPlanRequest(
        source="soccer_field_penalty_area.dxf",
        include_waypoints=True,
        line_spacing=0.1,
        transit_spacing=0.3,
        marking_speed=0.4,
        transit_speed=0.6,
        close_loop=True,
        ref_points=[
            RefPoint(dxf_x=0.0, dxf_y=0.0, lat=13.0, lon=80.0),
            RefPoint(dxf_x=10.0, dxf_y=0.0, lat=13.0001, lon=80.0),
        ],
        origin_gps=[13.0, 80.0]
    )
    
    data = await plan_path(req)
    
    assert data.source == "soccer_field_penalty_area.dxf"
    assert data.num_waypoints > 0
    assert data.num_segments > 0
    assert len(data.merged_waypoints) > 0
    assert len(data.spray_flags) > 0
    assert data.alignment_metadata is not None
    assert data.warnings is not None
    
    meta = data.alignment_metadata
    assert meta["method"] == "least_squares"
    assert "scale" in meta
    assert "rmse" in meta
    assert "residuals" in meta
    assert len(meta["residuals"]) == 2

@pytest.mark.anyio
async def test_plan_api_dxf_simple_rotation():
    if main.path_mgr is None:
        main.path_mgr = PathManager(main.MISSION_DIR)

    req = PathPlanRequest(
        source="soccer_field_penalty_area.dxf",
        include_waypoints=True,
        line_spacing=0.1,
        transit_spacing=0.3,
        marking_speed=0.4,
        transit_speed=0.6,
        close_loop=False,
        rotation_deg=45.0,
        origin_gps=[13.0, 80.0]
    )
    
    data = await plan_path(req)
    
    assert data.source == "soccer_field_penalty_area.dxf"
    assert data.num_waypoints > 0
    assert data.num_segments > 0
    assert len(data.merged_waypoints) > 0
    assert len(data.spray_flags) > 0
    assert data.alignment_metadata is not None
    assert data.alignment_metadata["method"] == "gps_origin"
    assert data.alignment_metadata["rotation_deg"] == 45.0

@pytest.mark.anyio
async def test_plan_api_single_point_heading():
    if main.path_mgr is None:
        main.path_mgr = PathManager(main.MISSION_DIR)

    # Gap B: one ref point + heading is now a valid alignment mode (was a
    # silent fall-back to gps_origin about (0,0)).
    req = PathPlanRequest(
        source="soccer_field_penalty_area.dxf",
        include_waypoints=True,
        line_spacing=0.1,
        transit_spacing=0.3,
        marking_speed=0.4,
        transit_speed=0.6,
        rotation_deg=30.0,
        ref_points=[
            RefPoint(dxf_x=5.0, dxf_y=5.0, lat=13.0001, lon=80.0001),
        ],
        origin_gps=[13.0, 80.0]
    )

    data = await plan_path(req)
    meta = data.alignment_metadata
    assert meta["method"] == "single_point_heading"
    assert meta["rotation_deg"] == 30.0
    assert meta["scale"] == 1.0
    # Clicked point is offset from origin_gps, so translation must be non-zero.
    assert meta["offset_n"] != 0.0 or meta["offset_e"] != 0.0
    assert meta["rmse"] == 0.0

@pytest.mark.anyio
async def test_plan_api_coincident_ref_points():
    if main.path_mgr is None:
        main.path_mgr = PathManager(main.MISSION_DIR)

    # Two coincident ref points should fail inside dxf_to_ned_affine and raise 422 HTTP exception
    req = PathPlanRequest(
        source="soccer_field_penalty_area.dxf",
        include_waypoints=True,
        line_spacing=0.1,
        transit_spacing=0.3,
        marking_speed=0.4,
        transit_speed=0.6,
        ref_points=[
            RefPoint(dxf_x=0.0, dxf_y=0.0, lat=13.0, lon=80.0),
            RefPoint(dxf_x=0.0, dxf_y=0.0, lat=13.0, lon=80.0),
        ],
        origin_gps=[13.0, 80.0]
    )
    
    with pytest.raises(HTTPException) as exc:
        await plan_path(req)
    assert exc.value.status_code == 422
    assert "coincident" in exc.value.detail


# ── Gap A: unit-scale frame consistency ───────────────────────────────────────

def test_affine_scale_is_unity_when_ref_points_share_metric_frame():
    """Gap A regression.

    A cm-unit DXF square whose ref points are 10 m apart in GPS must yield an
    affine scale ≈ 1.0 once the ref points are scaled into the metric frame —
    not ≈100 (raw cm fed against metric NED) or ≈0.01.
    """
    from path_engine.ned import dxf_to_ned_affine

    unit_scale = 0.01  # cm → m
    # Two ref points 1000 DXF units (= 10 m) apart along DXF-x.
    raw_dxf = [(0.0, 0.0), (0.0, 1000.0)]  # stored as (dxf_y, dxf_x)
    ned = [(0.0, 0.0), (0.0, 10.0)]        # 10 m east

    # Wrong (pre-fix): raw cm points vs metric NED → scale ~0.01.
    raw_scale = dxf_to_ned_affine(raw_dxf, ned)[0]
    assert abs(raw_scale - 1.0) > 0.5  # demonstrably off

    # Correct (post-fix): scale ref points into metres first.
    metric_dxf = [(p[0] * unit_scale, p[1] * unit_scale) for p in raw_dxf]
    fixed_scale = dxf_to_ned_affine(metric_dxf, ned)[0]
    assert abs(fixed_scale - 1.0) < 1e-6


# ── Gap D: RMSE quality gate ───────────────────────────────────────────────────

@pytest.mark.anyio
async def test_plan_api_rmse_gate_rejects_high_residual(monkeypatch):
    """Gap D: alignment RMSE above RMSE_MAX returns 422 and stages nothing."""
    from config import RMSE_MAX

    class FakePathManager:
        def plan_path(self, source, summary_only=False, **kwargs):
            return {
                "source": source,
                "num_waypoints": 2,
                "num_segments": 1,
                "mark_length_m": 1.0,
                "transit_length_m": 0.0,
                "total_length_m": 1.0,
                "segments": [],
                "merged_waypoints": [(0.0, 0.0), (1.0, 0.0)],
                "spray_flags": [True, True],
                "alignment_metadata": {
                    "method": "least_squares",
                    "rmse": RMSE_MAX + 0.10,
                    "origin_gps": (13.0, 80.0),
                },
                "warnings": [],
            }

    monkeypatch.setattr(main, "path_mgr", FakePathManager())
    req = PathPlanRequest(source="soccer_field_penalty_area.dxf")

    with pytest.raises(HTTPException) as exc:
        await plan_path(req)
    assert exc.value.status_code == 422
    assert "rmse" in exc.value.detail.lower()


# ── Gaps C & E: staging + load-to-controller round-trip ────────────────────────

@pytest.mark.anyio
async def test_plan_then_load_to_controller_round_trip(monkeypatch, tmp_path):
    """Gaps C/E: plan stages the aligned mission; load-to-controller pushes the
    identical waypoints to the controller and forwards the GPS anchor."""
    import routes.path as path_routes
    from models import LoadMissionRequest

    staging = tmp_path / "staging"
    monkeypatch.setattr(path_routes, "STAGING_DIR", str(staging))

    waypoints = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]

    class FakePathManager:
        def plan_path(self, source, summary_only=False, **kwargs):
            return {
                "source": source,
                "num_waypoints": len(waypoints),
                "num_segments": 1,
                "mark_length_m": 2.0,
                "transit_length_m": 0.0,
                "total_length_m": 2.0,
                "segments": [],
                "merged_waypoints": list(waypoints),
                "spray_flags": [True, True, True],
                "alignment_metadata": {
                    "method": "least_squares",
                    "rmse": 0.004,
                    "rotation_deg": 12.0,
                    "scale": 1.0,
                    "origin_gps": (13.0, 80.0),
                },
                "warnings": [],
            }

    class FakeController:
        def __init__(self):
            self.loaded = None
            self.state = MissionState.IDLE

        def load_path(self, points, name=None, spray_flags=None):
            self.loaded = (list(points), name, spray_flags)

    fake_ctrl = FakeController()
    monkeypatch.setattr(main, "path_mgr", FakePathManager())
    monkeypatch.setattr(main, "offboard_ctrl", fake_ctrl)

    req = PathPlanRequest(source="soccer_field_penalty_area.dxf")
    data = await plan_path(req)

    assert data.mission_summary is not None
    mid = data.mission_summary.mission_id
    assert data.mission_summary.estimated_paint_l > 0
    assert data.mission_summary.estimated_runtime_s > 0
    assert (staging / f"{mid}.json").is_file()

    resp = await path_routes.load_mission_to_controller(LoadMissionRequest(mission_id=mid))
    assert resp["status"] == "success"
    assert resp["num_waypoints"] == len(waypoints)
    assert resp["anchor_loaded"] is True
    # Controller received the exact aligned waypoints.
    assert fake_ctrl.loaded[0] == waypoints


@pytest.mark.anyio
async def test_load_to_controller_missing_mission_is_404(monkeypatch, tmp_path):
    import routes.path as path_routes
    from models import LoadMissionRequest

    monkeypatch.setattr(path_routes, "STAGING_DIR", str(tmp_path / "staging"))

    class FakeController:
        state = MissionState.IDLE

        def load_path(self, points, name=None, spray_flags=None):
            pass

    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    with pytest.raises(HTTPException) as exc:
        await path_routes.load_mission_to_controller(
            LoadMissionRequest(mission_id="stg_does_not_exist")
        )
    assert exc.value.status_code == 404


@pytest.mark.anyio
async def test_load_to_controller_rejects_while_running(monkeypatch, tmp_path):
    """Field-safety: loading a new mission while one is RUNNING returns 409
    and never reads the staged artifact."""
    import routes.path as path_routes
    from models import LoadMissionRequest

    monkeypatch.setattr(path_routes, "STAGING_DIR", str(tmp_path / "staging"))

    class FakeController:
        state = MissionState.RUNNING

        def load_path(self, points, name=None, spray_flags=None):
            raise AssertionError("load_path must not be called while RUNNING")

    monkeypatch.setattr(main, "offboard_ctrl", FakeController())

    with pytest.raises(HTTPException) as exc:
        await path_routes.load_mission_to_controller(
            LoadMissionRequest(mission_id="stg_anything")
        )
    assert exc.value.status_code == 409
