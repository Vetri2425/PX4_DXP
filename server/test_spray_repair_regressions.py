"""Integration regressions for spray-mode load policy and operator enable ownership."""

from __future__ import annotations

import json
import os
import sys

import pytest
from fastapi import HTTPException

sys.path.insert(0, os.path.dirname(__file__))

import main
import routes.path as path_routes
from models import LoadMissionRequest, MissionState
from spray_config import SprayConfiguration, SprayMode
from spray_mission_config import apply_spray_mission_config


class _Controller:
    state = MissionState.IDLE

    def __init__(self):
        self.loaded = None

    def load_path(self, points, **kwargs):
        self.loaded = (list(points), kwargs)


def _write_staged(tmp_path, mode_marker, *, mode="continuous"):
    staged = {
        "mission_id": mode_marker,
        "waypoints": [[0.0, 0.0], [1.0, 0.0]],
        "spray_flags": [True, True],
        "configuration_revision": 1,
        "path_fingerprint": "fp-" + mode_marker,
    }
    if mode is not None:
        staged["spray_mode"] = mode
    path = tmp_path / f"{mode_marker}.json"
    path.write_text(json.dumps(staged), encoding="utf-8")


@pytest.mark.anyio
@pytest.mark.parametrize("mode", [None, "continuous"])
async def test_legacy_and_continuous_load_degraded_without_spray_node(monkeypatch, tmp_path, mode):
    mission_id = "legacy" if mode is None else "continuous"
    _write_staged(tmp_path, mission_id, mode=mode)
    ctrl = _Controller()
    monkeypatch.setattr(path_routes, "STAGING_DIR", str(tmp_path))
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "ros_node", None)
    monkeypatch.setattr(main, "point_mission", None)
    response = await path_routes.load_mission_to_controller(LoadMissionRequest(mission_id=mission_id))
    assert response["spray_config_applied"] is False
    assert ctrl.loaded is not None
    assert ctrl.loaded[1]["spray_flags"] == [False, False]


@pytest.mark.anyio
@pytest.mark.parametrize("mode", ["dash", "point"])
async def test_dependent_modes_fail_503_before_controller_load(monkeypatch, tmp_path, mode):
    mission_id = mode
    _write_staged(tmp_path, mission_id, mode=mode)
    if mode == "point":
        staged_path = tmp_path / f"{mission_id}.json"
        staged = json.loads(staged_path.read_text())
        staged["point_mission_points"] = [{"north_m": 0, "east_m": 0, "dwell_s": 1, "source_index": 0}]
        staged["point_source_frame"] = "LOCAL_NED"
        staged_path.write_text(json.dumps(staged))
    ctrl = _Controller()
    monkeypatch.setattr(path_routes, "STAGING_DIR", str(tmp_path))
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "ros_node", None)
    monkeypatch.setattr(main, "point_mission", None)
    with pytest.raises(HTTPException) as exc:
        await path_routes.load_mission_to_controller(LoadMissionRequest(mission_id=mission_id))
    assert exc.value.status_code == 503
    assert ctrl.loaded is None


@pytest.mark.anyio
async def test_point_mode_load_succeeds_when_dependencies_ready(monkeypatch, tmp_path):
    mission_id = "point_ok"
    staged = {
        "mission_id": mission_id,
        "waypoints": [],
        "spray_mode": "point",
        "point_mission_points": [
            {"north_m": 0.0, "east_m": 0.0, "dwell_s": 1.0, "source_index": 0, "mark": True}
        ],
        "point_source_frame": "LOCAL_NED",
        "configuration_revision": 1,
        "path_fingerprint": "fp-" + mission_id,
    }
    path = tmp_path / f"{mission_id}.json"
    path.write_text(json.dumps(staged), encoding="utf-8")

    class Ros:
        async def cancel_spray_dwell_async(self):
            return True, "ok"

        def publish_spray_manual(self, on: bool):
            self.manual = on

        async def set_spray_params_bulk_async(self, params):
            return True, [True] * len(params), ""

        async def trigger_spray_apply_mission_config_async(self):
            return True, "ok"

    class PointOrch:
        loaded = None

        async def replace_from_staged(self, staged_payload, spray_config, ros_node):
            self.loaded = staged_payload["mission_id"]

        async def cancel_and_drain(self, ros_node, reason="cancelled"):
            return None

    ctrl = _Controller()
    ros = Ros()
    orch = PointOrch()
    monkeypatch.setattr(path_routes, "STAGING_DIR", str(tmp_path))
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "ros_node", ros)
    monkeypatch.setattr(main, "point_mission", orch)

    response = await path_routes.load_mission_to_controller(LoadMissionRequest(mission_id=mission_id))
    assert response["status"] == "success"
    assert orch.loaded == mission_id
    assert ctrl.loaded is not None
    assert ctrl.loaded[0] == [(0.0, 0.0), (0.0, 0.0)]


@pytest.mark.anyio
async def test_mission_config_apply_never_sets_operator_enable():
    class Ros:
        def __init__(self):
            self.params = None

        async def set_spray_params_bulk_async(self, params):
            self.params = dict(params)
            return True, [True] * len(params), ""

        async def trigger_spray_apply_mission_config_async(self):
            return True, "ok"

    ros = Ros()
    ok, _, _ = await apply_spray_mission_config(
        ros,
        {"mission_id": "m", "spray_mode": "continuous", "configuration_revision": 4, "path_fingerprint": "fp-m"},
    )
    assert ok
    assert "spray_enabled" not in ros.params
