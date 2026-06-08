import os
import sys

# Ensure server directory is in python path
sys.path.insert(0, os.path.dirname(__file__))

import pytest
from fastapi import HTTPException
from types import SimpleNamespace
from models import PathPlanRequest, RefPoint
from routes.path import plan_path
import main
from path_manager import PathManager


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
        def validate(self, plan):
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
async def test_plan_api_insufficient_ref_points():
    if main.path_mgr is None:
        main.path_mgr = PathManager(main.MISSION_DIR)

    # Only 1 ref point - should fail validation or planning with a 422 error
    req = PathPlanRequest(
        source="soccer_field_penalty_area.dxf",
        include_waypoints=True,
        line_spacing=0.1,
        transit_spacing=0.3,
        marking_speed=0.4,
        transit_speed=0.6,
        ref_points=[
            RefPoint(dxf_x=0.0, dxf_y=0.0, lat=13.0, lon=80.0),
        ],
        origin_gps=[13.0, 80.0]
    )
    
    # Having only 1 ref_point should fall back or raise ValueError in dxf_to_ned_affine.
    # In engine.py: if ref_points_dxf and ref_points_gps and len(...) >= 2:
    # So with 1 ref point it won't trigger least_squares, it will check origin_gps.
    # Let's verify it falls back to gps_origin method.
    data = await plan_path(req)
    assert data.alignment_metadata["method"] == "gps_origin"

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
