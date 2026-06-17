#!/usr/bin/env python3
"""Unit tests for Spray Controller V2 distance-aware decisions."""

from __future__ import annotations

import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_spray_manual_override import _Cli, _Param, _bool_msg, make_node  # noqa: E402
from spray_controller_node import (  # noqa: E402
    MARK_TO_TRANSIT,
    TRANSIT_TO_MARK,
    _build_path_model,
    _make_spray_decision,
    _nozzle_position_ned,
    _project_onto_path,
)


def _straight_mark_path():
    return _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        flags=[False, True, True, False],
    )


def _mark_only_path():
    return _build_path_model(
        points=[(0.0, 0.0), (2.0, 0.0)],
        flags=[True, True],
    )


def _decision(**kwargs):
    defaults = {
        "model": _straight_mark_path(),
        "nozzle_n": 1.5,
        "nozzle_e": 0.0,
        "speed_mps": 1.0,
        "safety_ok": True,
        "safety_reason": "",
        "solenoid_open_delay_s": 0.10,
        "solenoid_close_delay_s": 0.05,
        "on_overspray_margin_m": 0.02,
        "off_overspray_margin_m": 0.0,
        "max_xtrack_error_m": 0.10,
    }
    defaults.update(kwargs)
    return _make_spray_decision(**defaults)


def _make_distance_node(path_model=None, pose_n=1.0, pose_e=0.0, speed=1.0):
    node = make_node()
    node._params["use_distance_aware_spray"] = _Param(True)
    node._path_model = path_model if path_model is not None else _straight_mark_path()
    node._pose_ned = (pose_n, pose_e, 0.0)
    node._pose_recv_time = node.get_clock().now()
    node._vel_ned = (speed, 0.0)
    node._vel_recv_time = node.get_clock().now()
    return node


def test_path_boundary_extraction():
    model = _straight_mark_path()
    assert model.cumulative_s == [0.0, 1.0, 2.0, 3.0]
    assert [(b.s, b.kind) for b in model.boundaries] == [
        (1.0, TRANSIT_TO_MARK),
        (3.0, MARK_TO_TRANSIT),
    ]


def test_projection_onto_straight_path():
    model = _build_path_model(
        points=[(0.0, 0.0), (2.0, 0.0)],
        flags=[True, True],
    )
    proj = _project_onto_path(model, 0.75, 0.20)
    assert proj is not None
    assert abs(proj.s - 0.75) < 1e-9
    assert abs(proj.proj_n - 0.75) < 1e-9
    assert abs(proj.proj_e) < 1e-9
    assert abs(proj.xtrack_error_m - 0.20) < 1e-9
    assert proj.current_flag is True


def test_transit_to_mark_anticipatory_on():
    decision = _decision(
        nozzle_n=0.91,
        nozzle_e=0.0,
    )
    assert decision.event == "on_early"
    assert decision.desired is True
    assert decision.next_boundary is not None
    assert decision.next_boundary.kind == TRANSIT_TO_MARK


def test_mark_to_transit_anticipatory_off():
    decision = _decision(
        nozzle_n=2.96,
        nozzle_e=0.0,
    )
    assert decision.event == "off_early"
    assert decision.desired is False
    assert decision.next_boundary is not None
    assert decision.next_boundary.kind == MARK_TO_TRANSIT


def test_off_margin_does_not_cut_mark_tail_short():
    decision = _decision(nozzle_n=2.94, nozzle_e=0.0)
    assert decision.event == ""
    assert decision.desired is True


def test_on_overspray_margin_extends_mark_start():
    with_margin = _decision(nozzle_n=0.89, nozzle_e=0.0)
    without_margin = _decision(
        nozzle_n=0.89,
        nozzle_e=0.0,
        on_overspray_margin_m=0.0,
    )
    assert with_margin.event == "on_early"
    assert with_margin.desired is True
    assert without_margin.desired is False


def test_safety_off_when_disarmed():
    node = make_node(armed=False)
    node._path_model = _straight_mark_path()
    ok, reason = node._auto_safety_status(pose_fresh=True, speed=1.0)
    assert ok is False
    assert reason == "disarmed"


def test_safety_off_when_not_offboard():
    node = make_node(mode="MANUAL", require_offboard=True)
    node._path_model = _straight_mark_path()
    ok, reason = node._auto_safety_status(pose_fresh=True, speed=1.0)
    assert ok is False
    assert reason == "not OFFBOARD"


def test_nozzle_offset_changes_projection():
    model = _build_path_model(
        points=[(0.0, 0.0), (2.0, 0.0)],
        flags=[True, True],
    )
    base_n, base_e = _nozzle_position_ned(
        pose_n=0.0,
        pose_e=0.0,
        yaw_ned=0.0,
        forward_offset_m=0.0,
        lateral_offset_m=0.0,
    )
    offset_n, offset_e = _nozzle_position_ned(
        pose_n=0.0,
        pose_e=0.0,
        yaw_ned=0.0,
        forward_offset_m=1.0,
        lateral_offset_m=0.0,
    )
    base_proj = _project_onto_path(model, base_n, base_e)
    offset_proj = _project_onto_path(model, offset_n, offset_e)
    assert base_proj is not None and offset_proj is not None
    assert abs(base_proj.s - 0.0) < 1e-9
    assert abs(offset_proj.s - 1.0) < 1e-9
    assert offset_proj.s > base_proj.s


def test_lateral_nozzle_offset_changes_xtrack():
    model = _build_path_model(
        points=[(0.0, 0.0), (2.0, 0.0)],
        flags=[True, True],
    )
    nozzle_n, nozzle_e = _nozzle_position_ned(
        pose_n=1.0,
        pose_e=0.0,
        yaw_ned=0.0,
        forward_offset_m=0.0,
        lateral_offset_m=0.25,
    )
    proj = _project_onto_path(model, nozzle_n, nozzle_e)
    assert proj is not None
    assert math.isclose(proj.s, 1.0)
    assert math.isclose(proj.xtrack_error_m, 0.25)


def test_fallback_to_spray_active_when_distance_aware_disabled():
    node = make_node()
    node._params["use_distance_aware_spray"] = _Param(False)
    node._params["allow_legacy_spray_active_fallback"] = _Param(True)

    node._active_cb(_bool_msg(True))

    assert node._desired_debounced is True
    assert node._commanded is True
    assert node._command_cli.requests[-1].param1 == 1.0


def test_off_retry_does_not_mark_off_until_confirmed():
    node = make_node()
    node._commanded = True
    node._off_confirmed = True
    node._publish_state(True)
    node._command_cli = _Cli(responses=[(False, 99), True])

    node._force_off("test failure")

    assert len(node._command_cli.requests) == 1
    assert node._commanded is True
    assert node._off_confirmed is False
    assert node._state_pub.msgs[-1] is True

    node._clock.ns += 600_000_000
    node._reassert_tick()

    assert len(node._command_cli.requests) == 2
    assert node._commanded is False
    assert node._off_confirmed is True
    assert node._state_pub.msgs[-1] is False


def test_cross_track_gate_forces_off_on_mark_geometry():
    decision = _decision(
        model=_mark_only_path(),
        nozzle_n=1.0,
        nozzle_e=0.25,
        max_xtrack_error_m=0.10,
    )
    assert decision.geometry_desired is True
    assert decision.desired is False
    assert decision.safety_ok is False
    assert "xtrack error" in decision.safety_reason


def test_velocity_stale_forces_off():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)
    node._clock.ns += 600_000_000
    node._pose_recv_time = node.get_clock().now()

    node._distance_aware_tick()

    assert node._desired_debounced is False
    assert node._commanded is False
    assert "velocity stale" in node._last_safety_block_reason


def test_min_speed_end_to_end_forces_off():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=0.01)

    node._distance_aware_tick()

    assert node._desired_debounced is False
    assert node._commanded is False
    assert "below min spray speed" in node._last_safety_block_reason


def test_anticipation_lead_scales_with_fresh_speed():
    slow = _decision(nozzle_n=0.93, speed_mps=0.2)
    fast = _decision(nozzle_n=0.93, speed_mps=1.0)
    assert slow.desired is False
    assert fast.event == "on_early"
    assert fast.desired is True


def test_disarm_retains_path_briefly_then_clears_after_dwell():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)
    node._distance_aware_tick()
    assert node._commanded is True

    # Disarm edge: spray OFF immediately, but path retained (flap guard).
    node._state_cb(types.SimpleNamespace(armed=False, mode="OFFBOARD"))
    assert node._commanded is False
    assert node._path_model is not None

    # Brief disarm (< dwell): a watchdog tick must NOT clear the path.
    node._clock.ns += 1_000_000_000  # 1.0s < 2.0s dwell
    node._watchdog_tick()
    assert node._path_model is not None

    # Sustained disarm (>= dwell): path is discarded.
    node._clock.ns += 2_000_000_000  # 3.0s total
    node._watchdog_tick()
    assert node._path_model is None


def test_brief_disarm_flap_keeps_path_and_resumes_spray():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)
    node._distance_aware_tick()
    assert node._commanded is True

    # Transient single-message disarm then immediate re-arm (State flap).
    node._state_cb(types.SimpleNamespace(armed=False, mode="OFFBOARD"))
    node._state_cb(types.SimpleNamespace(armed=True, mode="OFFBOARD"))
    assert node._path_model is not None
    assert node._disarm_time_ns is None

    # Spray resumes on the next tick without any path republish.
    node._pose_recv_time = node.get_clock().now()
    node._vel_recv_time = node.get_clock().now()
    node._distance_aware_tick()
    assert node._commanded is True


def test_sustained_disarm_clears_path_and_prevents_stale_refire():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)
    node._distance_aware_tick()
    assert node._commanded is True

    node._state_cb(types.SimpleNamespace(armed=False, mode="OFFBOARD"))
    node._clock.ns += 3_000_000_000  # past the dwell
    node._watchdog_tick()
    assert node._path_model is None

    # Re-arm with no fresh path: spray must stay OFF (fail-closed).
    node._state_cb(types.SimpleNamespace(armed=True, mode="OFFBOARD"))
    node._pose_recv_time = node.get_clock().now()
    node._vel_recv_time = node.get_clock().now()
    node._distance_aware_tick()
    assert node._desired_debounced is False
    assert node._commanded is False


def test_xtrack_gate_forces_off_through_tick():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)
    node._pose_ned = (1.0, 0.25, 0.0)  # 0.25m off the e=0 line > 0.10 gate
    node._pose_recv_time = node.get_clock().now()
    node._distance_aware_tick()
    assert node._desired_debounced is False
    assert node._commanded is False
    assert "xtrack error" in node._last_safety_block_reason


def test_startup_unconfirmed_off_is_commanded_while_disarmed():
    # Mirrors the __init__ startup state: actuator OFF not yet confirmed.
    node = make_node(armed=False)
    node._params["use_distance_aware_spray"] = _Param(True)
    node._off_confirmed = False
    node._commanded = False

    node._watchdog_tick()

    assert node._command_cli.requests
    assert node._command_cli.requests[-1].param1 == -1.0
    assert node._off_confirmed is True


def test_boundary_projection_at_transit_mark_vertex_uses_later_segment():
    model = _straight_mark_path()
    projection = _project_onto_path(model, 1.0, 0.0)
    assert projection is not None
    assert projection.current_flag is True
    assert math.isclose(projection.s, 1.0)


def test_boundary_projection_at_mark_transit_vertex_uses_later_segment():
    model = _straight_mark_path()
    projection = _project_onto_path(model, 3.0, 0.0)
    assert projection is not None
    assert projection.current_flag is False
    assert math.isclose(projection.s, 3.0)


def test_duplicate_zero_length_segment_no_crash_and_fail_closed_when_unsafe():
    model = _build_path_model(
        points=[(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)],
        flags=[False, True, True],
    )
    decision = _decision(
        model=model,
        nozzle_n=0.0,
        nozzle_e=0.0,
        safety_ok=False,
        safety_reason="test unsafe",
    )
    assert decision.desired is False
    assert decision.safety_ok is False


def test_full_distance_aware_tick_anticipatory_on_and_off():
    node = _make_distance_node(path_model=_straight_mark_path(), pose_n=0.91, speed=1.0)
    node._distance_aware_tick()

    assert node._desired_pub.msgs[-1] is True
    assert node._commanded_pub.msgs[-1] is True
    assert node._commanded is True

    node._pose_ned = (2.96, 0.0, 0.0)
    node._pose_recv_time = node.get_clock().now()
    node._distance_aware_tick()

    assert node._desired_pub.msgs[-1] is False
    assert node._commanded_pub.msgs[-1] is False
    assert node._commanded is False


def test_late_on_success_does_not_resurrect_commanded_on():
    # ON dispatched, then OFF supersedes and confirms; the late ON reply must
    # not flip state back to ON.
    node = make_node()
    node._command_cli = _Cli(deferred=True)

    node._send_command(True, reason="edge")    # seq=1, commanded optimistic ON
    on_future = node._command_cli.futures[-1]
    assert node._commanded is True

    node._send_command(False, reason="edge")   # seq=2 supersedes
    off_future = node._command_cli.futures[-1]
    off_future.fire(success=True)              # latest OFF confirms
    assert node._commanded is False
    assert node._off_confirmed is True

    on_future.fire(success=True)               # stale ON reply — ignored
    assert node._commanded is False
    assert node._off_confirmed is True


def test_late_off_success_does_not_clear_newer_on():
    # OFF dispatched, then ON supersedes; the late OFF success must not clear
    # the newer ON state. This is the dangerous direction the guard fixes.
    node = make_node()
    node._command_cli = _Cli(deferred=True)

    node._send_command(False, reason="edge")   # seq=1
    off_future = node._command_cli.futures[-1]

    node._send_command(True, reason="edge")    # seq=2, commanded ON
    assert node._commanded is True

    off_future.fire(success=True)              # stale OFF success — ignored
    assert node._commanded is True
    assert node._off_confirmed is False


def test_failed_off_for_latest_command_triggers_retry():
    node = make_node()
    node._commanded = True
    node._off_confirmed = True
    node._command_cli = _Cli(deferred=True)

    node._send_command(False, reason="edge")   # latest OFF
    off_future = node._command_cli.futures[-1]
    off_future.fire(success=False, result=4)   # rejected
    assert node._off_confirmed is False         # stays unconfirmed (fail-closed)
    assert node._commanded is True              # not cleared until confirmed

    node._clock.ns += 600_000_000               # past 0.5s retry throttle
    node._reassert_tick()
    assert len(node._command_cli.requests) == 2  # OFF retried
    node._command_cli.futures[-1].fire(success=True)
    assert node._off_confirmed is True
    assert node._commanded is False


def test_stale_failed_result_ignored_does_not_corrupt_state():
    node = make_node()
    node._command_cli = _Cli(deferred=True)

    node._send_command(False, reason="edge")   # seq=1
    off_future = node._command_cli.futures[-1]
    node._send_command(True, reason="edge")    # seq=2, commanded ON
    assert node._commanded is True

    off_future.fire(exc=RuntimeError("late failure"))  # stale failed OFF — ignored
    assert node._commanded is True
    assert node._off_confirmed is False


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print(f"ok {test.__name__}")
    print("PASS")


if __name__ == "__main__":
    main()
