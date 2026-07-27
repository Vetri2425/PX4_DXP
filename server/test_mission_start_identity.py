"""`/api/mission/start` must verify the mission the caller asked for.

The mobile app has always sent `{"mission_id": ..., "auto_origin": false}` after
committing a staged mission. MissionStartRequest had no such field, so Pydantic
dropped it (extras are ignored by default) and nothing checked that the mission
the client verified was the one the controller held. It happened to be safe only
because dropping it left `path_name` None, so start drove whatever was loaded.

That is now an explicit cross-check instead of a lucky no-op.
"""
import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest
from fastapi import HTTPException

import main
from models import MissionState, MissionStartRequest
from routes.mission import start_mission

STAGED = "stg_1d5be788_1784974430"


class FakeController:
    """Only the surface start_mission touches before it would really start."""

    def __init__(self, mission_id=None):
        self._mission_id = mission_id
        self.started = False
        self.state = MissionState.RUNNING

    def loaded_path_summary(self, sample=20):
        return {"mission_id": self._mission_id, "loaded": bool(self._mission_id)}

    async def start_async(self, auto_origin=False):
        self.started = True
        return True, "running"


def _start(req):
    return asyncio.run(start_mission(req))


def test_matching_mission_id_is_accepted(monkeypatch):
    ctrl = FakeController(STAGED)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    res = _start(MissionStartRequest(mission_id=STAGED))
    assert ctrl.started is True
    assert res["state"] == "running"


def test_mismatched_mission_id_is_refused(monkeypatch):
    """The real hazard: a client verified mission A, something re-staged as B."""
    ctrl = FakeController("stg_other_1784900000")
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    with pytest.raises(HTTPException) as exc:
        _start(MissionStartRequest(mission_id=STAGED))
    assert exc.value.status_code == 409
    assert STAGED in exc.value.detail and "stg_other_1784900000" in exc.value.detail
    assert ctrl.started is False, "must not start a mission the caller did not verify"


def test_mission_id_with_nothing_loaded_is_refused(monkeypatch):
    ctrl = FakeController(None)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    with pytest.raises(HTTPException) as exc:
        _start(MissionStartRequest(mission_id=STAGED))
    assert exc.value.status_code == 409
    assert "no staged mission" in exc.value.detail
    assert ctrl.started is False


def test_mission_id_and_path_name_together_are_refused(monkeypatch):
    """Contradictory: mission_id starts what is loaded, path_name re-loads from
    disk at LOCAL_NED and would discard the surveyed placement. Refuse rather
    than silently pick one."""
    ctrl = FakeController(STAGED)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    with pytest.raises(HTTPException) as exc:
        _start(MissionStartRequest(mission_id=STAGED, path_name="curve_6_points.csv"))
    assert exc.value.status_code == 422
    assert "not both" in exc.value.detail
    assert ctrl.started is False


def test_omitting_mission_id_keeps_the_old_behaviour(monkeypatch):
    """An empty body still starts whatever is loaded — no new precondition."""
    ctrl = FakeController(STAGED)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    _start(None)
    assert ctrl.started is True


def test_blank_mission_id_is_treated_as_absent(monkeypatch):
    ctrl = FakeController(STAGED)
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    _start(MissionStartRequest(mission_id="   "))
    assert ctrl.started is True


def test_mission_id_is_no_longer_silently_dropped():
    """Regression on the model itself: the field must exist, or the check above
    can never fire no matter what the route does."""
    req = MissionStartRequest(**{"mission_id": STAGED, "auto_origin": False})
    assert req.mission_id == STAGED
    assert req.path_name is None
