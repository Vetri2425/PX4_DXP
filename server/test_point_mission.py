#!/usr/bin/env python3
"""Unit tests for point mission orchestrator (mocked ROS)."""

from __future__ import annotations

import asyncio
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from point_ingest import SprayPoint
from mission_placement import GPS_SURVEYED, LOCAL_NED, PlacementError
from point_mission import PointExecutionMode, PointMissionOrchestrator, PointMissionRun, PointMissionState
from spray_config import PointSprayParams, SprayConfiguration, SprayMode


class FakeRos:
    def __init__(self):
        self.state = {
            "pose_received": True,
            "pos_n": 0.0,
            "pos_e": 0.0,
            "speed_m_s": 0.0,
            "yaw_rate_rad_s": 0.0,
            "velocity_age_ms": 10.0,
            "pose_age_ms": 10.0,
            "rpp_state": 3,
            "spraying": False,
            "connected": True,
            "heading_ned_rad": 0.0,
        }
        self.paths = []
        self.dwells = []
        self.runtime_statuses = []
        self.last_runtime_status = None
        self.auto_arrive = False
        self.live_dwell = None

    def get_state(self):
        return dict(self.state)

    def publish_path(self, points, spray_flags=None, runtime_entry=False):
        self.paths.append((list(points), spray_flags, runtime_entry))
        if self.auto_arrive:
            self.state["pos_n"], self.state["pos_e"] = points[-1]

    async def cancel_spray_dwell_async(self):
        self.live_dwell = None
        return True, "ok"

    async def start_spray_dwell_async(self, **kwargs):
        self.dwells.append(kwargs)
        if self.auto_arrive:
            self.live_dwell = {
                **kwargs,
                "deadline": time.monotonic() + kwargs["duration_s"],
            }
        return True, "ok"

    def get_spray_runtime_status(self):
        if self.live_dwell is not None:
            remaining = self.live_dwell["deadline"] - time.monotonic()
            active = remaining > 0.0
            return {
                "status_stale": False,
                "ready": True,
                "active_dwell": active,
                "dwell_remaining_s": max(0.0, remaining),
                "commanded_on": active,
                "confirmed_off": not active,
                "dwell_command_id": self.live_dwell["command_id"],
                "last_error": "",
            }
        if self.runtime_statuses:
            self.last_runtime_status = self.runtime_statuses.pop(0)
            return self.last_runtime_status
        if self.last_runtime_status is not None:
            return self.last_runtime_status
        return {
            "status_stale": False,
            "ready": True,
            "active_dwell": False,
            "dwell_remaining_s": 0.0,
            "commanded_on": False,
            "confirmed_off": True,
            "dwell_command_id": self.dwells[-1]["command_id"] if self.dwells else None,
            "last_error": "",
        }

    def publish_spray_manual(self, on: bool):
        self.manual = on


class FakeOffboard:
    state = "running"
    _running_mission_id = "m1"


async def _run_cancel_during_nav():
    ros = FakeRos()
    offboard = FakeOffboard()
    orch = PointMissionOrchestrator()
    cfg = SprayConfiguration(mode=SprayMode.POINT)
    orch.load(
        mission_id="m1",
        points=[
            SprayPoint(5.0, 0.0, 1.0, 0),
            SprayPoint(10.0, 0.0, 1.0, 1),
        ],
        config=cfg,
    )
    started, _ = await orch.start(ros, offboard)
    assert started
    await asyncio.sleep(0.05)
    await orch.abort(ros)
    assert orch.status.state in {PointMissionState.FAILED, PointMissionState.ABORTING}


def test_arrival_requires_settle_conditions():
    ros = FakeRos()
    ros.state.update({"pos_n": 1.0, "pos_e": 0.0, "speed_m_s": 0.5})
    orch = PointMissionOrchestrator()
    cfg = SprayConfiguration(mode=SprayMode.POINT)
    point = SprayPoint(1.0, 0.0, 1.0, 0)
    assert orch._arrival_conditions_met(ros.get_state(), point, cfg.point) is False


def test_cancel_during_navigation():
    asyncio.run(_run_cancel_during_nav())


def test_dwell_must_become_active_before_completion():
    async def run():
        ros = FakeRos()
        orch = PointMissionOrchestrator()
        orch.load(mission_id="m1", points=[SprayPoint(0, 0, 0.01, 0)], config=SprayConfiguration(mode=SprayMode.POINT))
        token = PointMissionRun(orch.status.generation, "m1", asyncio.Event())
        orch._run_token = token
        ros.dwells.append({"command_id": 5})
        try:
            point = SprayPoint(0, 0, 0.01, 0)
            await orch._wait_dwell_complete(
                token, ros, None, point, 0.01, 5, SprayConfiguration(mode=SprayMode.POINT).point
            )
            assert False, "inactive default status passed as completion"
        except TimeoutError as exc:
            assert "never became active" in str(exc)
    asyncio.run(run())


def test_dwell_identity_and_final_off_required():
    async def run():
        ros = FakeRos()
        orch = PointMissionOrchestrator()
        orch.load(mission_id="m1", points=[SprayPoint(0, 0, 0.1, 0)], config=SprayConfiguration(mode=SprayMode.POINT))
        token = PointMissionRun(orch.status.generation, "m1", asyncio.Event())
        orch._run_token = token
        base = {"status_stale": False, "ready": True, "last_error": "", "dwell_remaining_s": 0.05}
        ros.runtime_statuses = [
            {**base, "dwell_command_id": 8, "active_dwell": True, "commanded_on": True, "confirmed_off": False},
        ]
        try:
            point = SprayPoint(0, 0, 0.1, 0)
            await orch._wait_dwell_complete(
                token, ros, None, point, 0.1, 7, SprayConfiguration(mode=SprayMode.POINT).point
            )
            assert False, "wrong command ID accepted"
        except RuntimeError as exc:
            assert "mismatch" in str(exc)

        ros.runtime_statuses = [
            {**base, "dwell_command_id": 7, "active_dwell": True, "commanded_on": True, "confirmed_off": False},
            {**base, "dwell_command_id": 7, "active_dwell": False, "commanded_on": True, "confirmed_off": False},
        ]
        try:
            await orch._wait_dwell_complete(
                token, ros, None, point, 0.1, 7, SprayConfiguration(mode=SprayMode.POINT).point
            )
            assert False, "unconfirmed OFF accepted"
        except TimeoutError as exc:
            assert "completion" in str(exc)
    asyncio.run(run())


def test_generation_guard_blocks_old_status_write():
    orch = PointMissionOrchestrator()
    cfg = SprayConfiguration(mode=SprayMode.POINT)
    orch.load(mission_id="old", points=[SprayPoint(0, 0, 1, 0)], config=cfg)
    old = PointMissionRun(orch.status.generation, "old", asyncio.Event())
    orch._run_token = old
    orch._task = None
    orch._install("new", [SprayPoint(1, 2, 1, 0)], cfg, LOCAL_NED, None, PointExecutionMode.AUTO)
    orch._write(old, state=PointMissionState.FAILED, last_error="late old failure")
    assert orch.status.mission_id == "new"
    assert orch.status.last_error == ""


def test_coordinate_resolution_local_and_surveyed_translation():
    orch = PointMissionOrchestrator()
    cfg = SprayConfiguration(mode=SprayMode.POINT)
    orch._install("local", [SprayPoint(3, 4, 1, 0)], cfg, LOCAL_NED, None, PointExecutionMode.AUTO)
    assert orch._resolve_points({})[0].north_m == 3
    assert orch._resolve_points({})[0].east_m == 4

    orch._install("gps", [SprayPoint(3, 4, 1, 0)], cfg, GPS_SURVEYED, (13.0, 80.0), PointExecutionMode.AUTO)
    state = {
        "connected": True,
        "pose_received": True, "global_position_received": True, "gps_fix_received": True,
        "local_pose_age_ms": 10, "global_position_age_ms": 10, "gps_fix_age_ms": 10,
        "pose_global_skew_ms": 0, "gps_fix": 6, "pos_n": 100, "pos_e": 200,
        "lat": 13.0, "lon": 80.0,
    }
    resolved = orch._resolve_points(state)[0]
    assert abs(resolved.north_m - 103.0) < 1e-6
    assert abs(resolved.east_m - 204.0) < 1e-6


def test_missing_point_frame_metadata_rejected():
    async def run():
        orch = PointMissionOrchestrator()
        ros = FakeRos()
        staged = {"mission_id": "m", "point_mission_points": [{"north_m": 1, "east_m": 2, "dwell_s": 1, "source_index": 0}]}
        try:
            await orch.replace_from_staged(staged, SprayConfiguration(mode=SprayMode.POINT), ros)
            assert False, "missing frame accepted"
        except PlacementError:
            pass
    asyncio.run(run())


async def _run_clear_mission_resets_unloaded_state():
    ros = FakeRos()
    orch = PointMissionOrchestrator()
    cfg = SprayConfiguration(mode=SprayMode.POINT)
    orch.load(mission_id="m1", points=[SprayPoint(1.0, 2.0, 1.0, 0)], config=cfg)
    orch._resolved_points = [SprayPoint(1.0, 2.0, 1.0, 0)]
    orch._run_token = PointMissionRun(orch.status.generation, "m1", asyncio.Event())
    await orch.clear_mission(ros, reason="cleared")
    assert orch.status.state == PointMissionState.IDLE
    assert orch.status.ready is False
    assert orch._points == []
    assert orch._resolved_points == []
    assert orch._config is None
    assert orch._run_token is None
    assert orch._task is None


def test_clear_mission_resets_unloaded_state():
    asyncio.run(_run_clear_mission_resets_unloaded_state())


def test_full_three_point_progression_observes_nonzero_dwells():
    async def run():
        ros = FakeRos()
        ros.auto_arrive = True
        orch = PointMissionOrchestrator()
        cfg = SprayConfiguration(
            mode=SprayMode.POINT,
            point=PointSprayParams(
                default_dwell_s=0.06,
                arrival_tolerance_m=0.05,
                settle_time_s=0.0,
                leg_timeout_s=2.0,
                settle_speed_mps=0.05,
                settle_yaw_rate_rad_s=0.05,
            ),
            revision=4,
            mission_id="m3",
        )
        orch.load(
            mission_id="m3",
            points=[SprayPoint(i, i * 2, 0.06, i) for i in range(3)],
            config=cfg,
        )
        started_at = time.monotonic()
        started, _ = await orch.start(ros, FakeOffboard())
        assert started
        await asyncio.wait_for(orch._task, timeout=3.0)
        assert orch.status.state == PointMissionState.COMPLETED
        assert len(ros.paths) == 3
        assert len(ros.dwells) == 3
        assert time.monotonic() - started_at >= 0.18
    asyncio.run(run())


def main():
    test_arrival_requires_settle_conditions()
    test_cancel_during_navigation()
    test_dwell_must_become_active_before_completion()
    test_dwell_identity_and_final_off_required()
    test_generation_guard_blocks_old_status_write()
    test_coordinate_resolution_local_and_surveyed_translation()
    test_missing_point_frame_metadata_rejected()
    test_clear_mission_resets_unloaded_state()
    test_full_three_point_progression_observes_nonzero_dwells()
    print("PASS")


if __name__ == "__main__":
    main()
