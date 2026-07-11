import asyncio
import os
import sys
from collections import deque

import pytest

sys.path.insert(0, os.path.dirname(__file__))

import offboard_controller as offboard_module
from config import RPP_IDLE, RPP_TRACKING
from mission_placement import PlacementError
from models import MissionState
from offboard_controller import OffboardController


class FakeRppMonitor:
    def reset(self):
        pass


class FakeNode:
    def __init__(self, states):
        self._states = list(states)
        self.calls = []

    def get_state(self):
        if len(self._states) > 1:
            return self._states.pop(0)
        return self._states[0]

    def get_rpp_monitor(self):
        return FakeRppMonitor()

    def publish_path(self, points, frame_id="local_ned", spray_flags=None):
        self.calls.append(("publish_path", list(points), spray_flags))

    async def arm_async(self, arm):
        self.calls.append(("arm", arm))
        return True, ""

    async def set_mode_async(self, mode):
        self.calls.append(("set_mode", mode))
        return True, ""


def run(coro):
    return asyncio.run(coro)


def test_start_publishes_path_before_arm_and_offboard():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        node = FakeNode([
            {"connected": True, "rpp_state": RPP_TRACKING},
            {"connected": True, "rpp_state": RPP_TRACKING},
        ])
        ctrl = OffboardController(node, deque())
        ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="test")

        ok, msg = run(ctrl.start_async())

        assert ok is True
        assert msg == "running"
        assert ctrl.state == MissionState.RUNNING
        assert node.calls == [
            ("publish_path", [(1.0, 2.0), (3.0, 4.0)], None),
            ("arm", True),
            ("set_mode", "OFFBOARD"),
        ]
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_start_disarms_if_rpp_stays_idle_after_path_publish():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        node = FakeNode([
            {"connected": True, "rpp_state": RPP_TRACKING},
            {"connected": True, "rpp_state": RPP_IDLE},
        ])
        ctrl = OffboardController(node, deque())
        ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="test")

        ok, msg = run(ctrl.start_async())

        assert ok is False
        assert "RPP IDLE after path publish" in msg
        assert ctrl.state == MissionState.ERROR
        assert node.calls == [
            ("publish_path", [(1.0, 2.0), (3.0, 4.0)], None),
            ("arm", True),
            ("arm", False),
        ]
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def _healthy_survey_state(rpp_state=RPP_TRACKING):
    return {
        "connected": True,
        "rpp_state": rpp_state,
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


def test_surveyed_start_rejects_auto_origin():
    ctrl = OffboardController(FakeNode([_healthy_survey_state()]), deque())
    ctrl.load_path(
        [(0.0, -0.035), (1.0, -0.035)],
        name="surveyed",
        placement_mode="GPS_SURVEYED",
        origin_gps=(13.072066, 80.261956),
    )
    with pytest.raises(PlacementError, match="incompatible with auto_origin"):
        run(ctrl.start_async(auto_origin=True))


def test_surveyed_start_publishes_live_ekf_points():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        state = _healthy_survey_state()
        node = FakeNode([state, dict(state)])
        ctrl = OffboardController(node, deque())
        source = [(0.0, -0.035), (1.0, -0.035)]
        ctrl.load_path(
            source,
            name="surveyed",
            placement_mode="GPS_SURVEYED",
            origin_gps=(13.072066, 80.261956),
        )

        ok, msg = run(ctrl.start_async())

        assert ok is True
        assert msg == "running"
        published = node.calls[0]
        assert published[0] == "publish_path"
        # Live placement shifts source uniformly; first point near field golden.
        assert published[1][0] == pytest.approx((5.192, -0.910), abs=0.02)
        # Source resident path remains anchor-relative (not mutated).
        assert ctrl._loaded_pts == source
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_clear_mission_resets_resident_state():
    ctrl = OffboardController(FakeNode([{"connected": True, "rpp_state": RPP_IDLE}]), deque())
    ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="sq")
    assert ctrl._loaded_pts == [(1.0, 2.0), (3.0, 4.0)]

    status = run(ctrl.clear_mission_async())

    assert ctrl.state == MissionState.IDLE
    assert ctrl._loaded_pts is None
    assert ctrl.loaded_path_name is None
    assert status["loaded"] is False
    assert status["num_waypoints"] == 0


def test_clear_mission_rejected_while_running():
    ctrl = OffboardController(FakeNode([{"connected": True, "rpp_state": RPP_IDLE}]), deque())
    ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="sq")
    ctrl.state = MissionState.RUNNING

    with pytest.raises(offboard_module.MissionClearConflict):
        run(ctrl.clear_mission_async())

    # A rejected clear leaves state and resident path untouched.
    assert ctrl.state == MissionState.RUNNING
    assert ctrl._loaded_pts == [(1.0, 2.0), (3.0, 4.0)]


def test_loaded_path_summary_exposes_staged_identity():
    # A staged surveyed load, exactly as POST /load-to-controller does
    # (name == the staged mission_id). The operator app's post-load
    # verifyStagedLoadedMission reads mission_id/is_staged/protected/placement_mode.
    ctrl = OffboardController(FakeNode([{"connected": True, "rpp_state": RPP_IDLE}]), deque())
    ctrl.load_path(
        [(1.0, 2.0), (3.0, 4.0)],
        name="stg_abc123_1700",
        placement_mode="GPS_SURVEYED",
        origin_gps=(12.9716, 80.1946),
        is_staged=True,
    )
    s = ctrl.loaded_path_summary()
    assert s["mission_id"] == "stg_abc123_1700"
    assert s["is_staged"] is True
    assert s["protected"] is True
    assert s["placement_mode"] == "GPS_SURVEYED"

    # A plain (non-staged) path must NOT masquerade as a staged mission.
    ctrl2 = OffboardController(FakeNode([{"connected": True, "rpp_state": RPP_IDLE}]), deque())
    ctrl2.load_path([(0.0, 0.0), (1.0, 1.0)], name="square_2x2")
    s2 = ctrl2.loaded_path_summary()
    assert s2["mission_id"] is None
    assert s2["protected"] is False
