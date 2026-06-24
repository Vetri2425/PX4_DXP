import asyncio
import os
import sys
from collections import deque

import pytest

sys.path.insert(0, os.path.dirname(__file__))

import offboard_controller as offboard_module
from config import RPP_IDLE, RPP_TRACKING
from models import MissionState
from mission_placement import GPS_SURVEYED, PlacementError
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

    def publish_path(
        self, points, frame_id="local_ned", spray_flags=None, runtime_entry=False
    ):
        self.calls.append(("publish_path", list(points), spray_flags, runtime_entry))

    def publish_path_clear(self):
        self.calls.append(("publish_path_clear",))

    async def arm_async(self, arm):
        self.calls.append(("arm", arm))
        return True, ""

    async def set_mode_async(self, mode):
        self.calls.append(("set_mode", mode))
        return True, ""


def surveyed_state(**overrides):
    state = {
        "connected": True,
        "rpp_state": RPP_TRACKING,
        "pose_received": True,
        "global_position_received": True,
        "gps_fix_received": True,
        "local_pose_age_ms": 10.0,
        "global_position_age_ms": 10.0,
        "gps_fix_age_ms": 10.0,
        "pose_global_skew_ms": 0.0,
        "gps_fix": 6,
        "pos_n": 7.4629,
        "pos_e": -0.9070,
        "lat": 13.0720864,
        "lon": 80.2619557,
    }
    state.update(overrides)
    return state


def load_surveyed(ctrl, spray_flags=None):
    ctrl.load_path(
        [(0.0, -0.035), (1.0, -0.035), (1.0, 0.965)],
        name="square_2x2.dxf",
        spray_flags=spray_flags,
        placement_mode=GPS_SURVEYED,
        origin_gps=(13.072066, 80.261956),
        mission_id="stg_field",
        source_name="square_2x2.dxf",
    )


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
            ("publish_path", [(1.0, 2.0), (3.0, 4.0)], None, False),
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
            ("publish_path", [(1.0, 2.0), (3.0, 4.0)], None, False),
            ("arm", True),
            ("arm", False),
        ]
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_surveyed_start_preserves_spray_flags_and_does_not_accumulate_translation():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        state = surveyed_state()
        node = FakeNode([state])
        ctrl = OffboardController(node, deque())
        flags = [False, True, True]
        load_surveyed(ctrl, spray_flags=flags)

        ok, _ = run(ctrl.start_async(expected_mission_id="stg_field"))
        assert ok is True
        first_publish = node.calls[0]
        assert first_publish[0] == "publish_path"
        assert first_publish[1][0] == pytest.approx((state["pos_n"], state["pos_e"]))
        assert first_publish[1][1] == pytest.approx((5.192, -0.910), abs=0.02)
        assert first_publish[1][2] == pytest.approx(first_publish[1][1])
        assert first_publish[2] == [False, False, *flags]
        assert first_publish[3] is True

        ctrl.state = MissionState.IDLE
        node.calls.clear()
        ok, _ = run(ctrl.start_async(expected_mission_id="stg_field"))
        assert ok is True
        assert node.calls[0] == first_publish
        assert ctrl.loaded_path_summary()["sample_coords"][0] == [0.0, -0.035]
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_surveyed_mark_first_adds_off_duplicate_boundary_and_preserves_source():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        state = surveyed_state()
        node = FakeNode([state])
        ctrl = OffboardController(node, deque())
        flags = [True, True, False]
        load_surveyed(ctrl, spray_flags=flags)
        source_before = ctrl.loaded_path_summary()["sample_coords"]
        evidence = []

        assert run(ctrl.start_async(pre_publish_hook=evidence.append))[0] is True

        published = node.calls[0]
        resolved_first = published[1][1]
        offset_n = resolved_first[0] - source_before[0][0]
        offset_e = resolved_first[1] - source_before[0][1]
        expected_mission = [
            (point[0] + offset_n, point[1] + offset_e) for point in source_before
        ]
        assert published[1][0] == pytest.approx((state["pos_n"], state["pos_e"]))
        assert published[1][2] == pytest.approx(resolved_first)
        assert published[1][2:] == pytest.approx(expected_mission)
        assert published[2] == [False, False, *flags]
        assert published[3] is True
        assert ctrl.loaded_path_summary()["sample_coords"] == source_before
        assert evidence[0]["resolved_first_waypoint_ned"] == pytest.approx(resolved_first)
        assert evidence[0]["published_first_waypoint_ned"] == pytest.approx(
            (state["pos_n"], state["pos_e"])
        )
        assert evidence[0]["entry_transit_added"] is True
        assert evidence[0]["source_point_count"] == len(source_before)
        assert evidence[0]["published_point_count"] == len(source_before) + 2
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_surveyed_coincident_start_does_not_inject_duplicate():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        state = surveyed_state(lat=13.0, lon=80.0, pos_n=4.0, pos_e=5.0)
        node = FakeNode([state])
        ctrl = OffboardController(node, deque())
        ctrl.load_path(
            [(0.0, 0.0), (1.0, 0.0)],
            spray_flags=[True, True],
            placement_mode=GPS_SURVEYED,
            origin_gps=(13.0, 80.0),
        )

        assert run(ctrl.start_async())[0] is True
        assert node.calls[0][1] == pytest.approx([(4.0, 5.0), (5.0, 5.0)])
        assert node.calls[0][2] == [True, True]
        assert node.calls[0][3] is False
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_surveyed_missing_flags_keeps_runtime_path_fail_closed():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        state = surveyed_state()
        node = FakeNode([state])
        ctrl = OffboardController(node, deque())
        load_surveyed(ctrl)

        assert run(ctrl.start_async())[0] is True
        assert node.calls[0][1][0] == pytest.approx((state["pos_n"], state["pos_e"]))
        assert node.calls[0][2] is None
        assert node.calls[0][3] is True
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


@pytest.mark.parametrize(
    "override",
    [
        {"local_pose_age_ms": 501.0},
        {"pos_n": float("nan")},
    ],
)
def test_bad_final_survey_pose_fails_before_publish_or_arm(override):
    node = FakeNode([surveyed_state(), surveyed_state(**override)])
    ctrl = OffboardController(node, deque())
    load_surveyed(ctrl, spray_flags=[True, True, True])

    with pytest.raises(PlacementError, match="surveyed placement failed"):
        run(ctrl.start_async())
    assert node.calls == []


def test_local_mission_never_gets_runtime_entry_prefix():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        node = FakeNode([
            {"connected": True, "rpp_state": RPP_TRACKING,
             "pose_received": True, "pos_n": 50.0, "pos_e": 60.0},
        ])
        ctrl = OffboardController(node, deque())
        ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], spray_flags=[True, True])

        assert run(ctrl.start_async())[0] is True
        assert node.calls[0] == (
            "publish_path", [(1.0, 2.0), (3.0, 4.0)], [True, True], False
        )
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_surveyed_restart_recomputes_from_changed_local_frame():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        state = surveyed_state()
        node = FakeNode([state])
        ctrl = OffboardController(node, deque())
        load_surveyed(ctrl)

        assert run(ctrl.start_async())[0] is True
        first = node.calls[0][1]

        ctrl.state = MissionState.IDLE
        state["pos_n"] += 10.0
        state["pos_e"] -= 4.0
        node.calls.clear()
        assert run(ctrl.start_async())[0] is True
        second = node.calls[0][1]

        for before, after in zip(first, second):
            assert after == pytest.approx((before[0] + 10.0, before[1] - 4.0))
        assert ctrl.loaded_path_summary()["sample_coords"][0] == [0.0, -0.035]
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_surveyed_auto_origin_fails_before_publish_or_arm():
    node = FakeNode([surveyed_state()])
    ctrl = OffboardController(node, deque())
    load_surveyed(ctrl)

    with pytest.raises(PlacementError, match="incompatible with auto_origin"):
        run(ctrl.start_async(auto_origin=True))
    assert node.calls == []
    assert ctrl.running_mission_id is None


def test_mission_id_mismatch_fails_before_publish_or_arm():
    node = FakeNode([surveyed_state()])
    ctrl = OffboardController(node, deque())
    load_surveyed(ctrl)

    ok, msg = run(ctrl.start_async(expected_mission_id="stg_other"))

    assert ok is False
    assert "identity mismatch" in msg
    assert node.calls == []


@pytest.mark.parametrize(
    "override",
    [
        {"pose_received": False},
        {"global_position_received": False},
        {"gps_fix_received": False},
        {"local_pose_age_ms": 501.0},
        {"global_position_age_ms": 501.0},
        {"gps_fix_age_ms": 501.0},
        {"gps_fix": 5},
        {"lat": float("nan")},
    ],
)
def test_bad_survey_telemetry_fails_before_publish_or_arm(override):
    node = FakeNode([surveyed_state(**override)])
    ctrl = OffboardController(node, deque())
    load_surveyed(ctrl)

    with pytest.raises(PlacementError, match="surveyed placement failed"):
        run(ctrl.start_async())
    assert node.calls == []
    assert ctrl.running_mission_id is None


def test_missing_survey_anchor_fails_before_publish_or_arm():
    node = FakeNode([surveyed_state()])
    ctrl = OffboardController(node, deque())
    ctrl.load_path(
        [(0.0, -0.035), (1.0, -0.035)],
        name="square_2x2.dxf",
        placement_mode=GPS_SURVEYED,
        origin_gps=None,
        mission_id="stg_field",
    )

    with pytest.raises(PlacementError, match="survey GPS anchor"):
        run(ctrl.start_async())
    assert node.calls == []
    assert ctrl.running_mission_id is None


def test_clear_mission_publishes_latched_path_reset():
    node = FakeNode([{"connected": True, "rpp_state": RPP_TRACKING}])
    ctrl = OffboardController(node, deque())
    ctrl.load_path(
        [(0.0, 0.0), (1.0, 0.0)],
        name="staged",
        spray_flags=[False, True],
        mission_id="stg_clear",
        is_staged=True,
        path_fingerprint="fp",
        configuration_revision=9,
    )

    summary = run(ctrl.clear_mission_async())

    assert summary["loaded"] is False
    assert ctrl.loaded_mission_id is None
    assert ("publish_path_clear",) in node.calls
