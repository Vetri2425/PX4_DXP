#!/usr/bin/env python3
"""Unit tests for Spray Controller V2 distance-aware decisions."""

from __future__ import annotations

import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_spray_manual_override import _Cli, _Param, _bool_msg, make_node  # noqa: E402
from spray_config import SprayConfiguration, UnsafeSpeedBehavior  # noqa: E402
from spray_controller_node import (  # noqa: E402
    MARK_TO_TRANSIT,
    TRANSIT_TO_MARK,
    _build_path_model,
    _make_spray_decision,
    _nozzle_position_ned,
    _project_onto_path,
)
from path_identity import path_geometry_fingerprint  # noqa: E402


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
    speed = kwargs.pop("speed_mps", 1.0)
    defaults = {
        "model": _straight_mark_path(),
        "nozzle_n": 1.5,
        "nozzle_e": 0.0,
        "vel_n": speed,
        "vel_e": 0.0,
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


def _make_distance_node(path_model=None, pose_n=1.0, pose_e=0.0, speed=0.35):
    node = make_node()
    node._params["use_distance_aware_spray"] = _Param(True)
    node._path_model = path_model if path_model is not None else _straight_mark_path()
    node._pose_ned = (pose_n, pose_e, 0.0)
    node._pose_recv_time = node.get_clock().now()
    node._vel_ned = (speed, 0.0)
    node._vel_recv_time = node.get_clock().now()
    return node


def _path_msg(points, flags):
    poses = []
    for (n, e), flag in zip(points, flags):
        pos = types.SimpleNamespace(x=n, y=e, z=1.0 if flag else 0.0)
        poses.append(types.SimpleNamespace(pose=types.SimpleNamespace(position=pos)))
    return types.SimpleNamespace(poses=poses)


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
        nozzle_n=0.88,
        nozzle_e=0.0,
    )
    assert decision.event == "ON_EARLY"
    assert decision.desired is True
    assert decision.next_boundary is not None
    assert decision.next_boundary.kind == TRANSIT_TO_MARK


def test_mark_to_transit_anticipatory_off():
    decision = _decision(
        nozzle_n=2.96,
        nozzle_e=0.0,
    )
    assert decision.event == "OFF_EARLY"
    assert decision.desired is False
    assert decision.next_boundary is not None
    assert decision.next_boundary.kind == MARK_TO_TRANSIT


def test_off_margin_does_not_cut_mark_tail_short():
    decision = _decision(nozzle_n=2.94, nozzle_e=0.0)
    assert decision.event == "FOLLOW_FLAG"
    assert decision.desired is True


def test_on_overspray_margin_extends_mark_start():
    with_margin = _decision(nozzle_n=0.88, nozzle_e=0.0)
    without_margin = _decision(
        nozzle_n=0.88,
        nozzle_e=0.0,
        on_overspray_margin_m=0.0,
    )
    assert with_margin.event == "ON_EARLY"
    assert with_margin.desired is True
    assert without_margin.desired is False


def test_safety_off_when_disarmed():
    node = make_node(armed=False)
    node._path_model = _straight_mark_path()
    ok, reason = node._auto_safety_status(pose_fresh=True, along_track_speed=1.0)
    assert ok is False
    assert reason == "disarmed"


def test_safety_off_when_not_offboard():
    node = make_node(mode="MANUAL", require_offboard=True)
    node._path_model = _straight_mark_path()
    ok, reason = node._auto_safety_status(pose_fresh=True, along_track_speed=1.0)
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
    node._params["diagnostic_profile"] = _Param(True)
    node._params["diagnostic_lease_active"] = _Param(True)

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
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=0.35)
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


def test_above_max_speed_blocks_spray():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.2)
    node._params["max_spray_speed_mps"] = _Param(0.5)
    node._active_config = node._configuration_from_node_parameters()

    node._distance_aware_tick()

    assert node._desired_debounced is False
    assert "above max spray speed" in node._last_safety_block_reason


def test_clamp_pwm_policy_allows_speed_outside_window():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.2)
    node._params["max_spray_speed_mps"] = _Param(0.5)
    node._params["unsafe_speed_behavior"] = _Param(UnsafeSpeedBehavior.CLAMP_PWM.value)
    node._params["high_speed_underflow_behavior"] = _Param("clamp")
    node._params["max_pwm_change_per_s"] = _Param(5000.0)
    node._active_config = node._configuration_from_node_parameters()
    node._continuous_runtime.previous_pwm = 1800.0

    node._distance_aware_tick()

    assert node._desired_debounced is True
    assert node._current_pwm == 1800.0


def test_dynamic_pwm_interpolation_reaches_actuator_request():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=0.2)
    node._params["speed_pwm_table"] = _Param(
        '[{"speed_mps":0.1,"pwm":1200.0},{"speed_mps":0.3,"pwm":2000.0}]'
    )
    node._params["max_pwm_change_per_s"] = _Param(5000.0)
    node._active_config = node._configuration_from_node_parameters()
    node._continuous_runtime.previous_pwm = 1600.0

    node._distance_aware_tick()

    assert node._current_pwm == 1600.0
    assert node._command_cli.requests[-1].param1 != 1.0


def test_conditioned_path_identity_mismatch_fails_closed():
    node = make_node()
    node._active_config = SprayConfiguration(
        mission_id="stg_a",
        path_fingerprint="expected",
        revision=1,
    )
    node._conditioned_path_identity = {
        "mission_id": "stg_a",
        "path_fingerprint": "wrong",
        "configuration_revision": 1,
        "source": "rpp_conditioned_path",
    }

    node._path_cb(_path_msg([(0.0, 0.0), (1.0, 0.0)], [True, True]))

    assert node._path_model is None
    assert node._desired_debounced is False


def test_conditioned_path_identity_match_loads_model():
    points = [(0.0, 0.0), (1.0, 0.0)]
    flags = [True, True]
    fp = path_geometry_fingerprint(points, flags)
    node = make_node()
    node._active_config = SprayConfiguration(
        mission_id="stg_a",
        path_fingerprint=fp,
        revision=1,
    )
    node._conditioned_path_identity = {
        "mission_id": "stg_a",
        "path_fingerprint": fp,
        "configuration_revision": 1,
        "source": "rpp_conditioned_path",
    }

    node._path_cb(_path_msg(points, flags))

    assert node._path_model is not None


def test_conditioned_path_configuration_revision_missing_fails_closed():
    points = [(0.0, 0.0), (1.0, 0.0)]
    flags = [True, True]
    fp = path_geometry_fingerprint(points, flags)
    node = make_node()
    node._active_config = SprayConfiguration(
        mission_id="stg_a",
        path_fingerprint=fp,
        revision=3,
    )
    node._conditioned_path_identity = {
        "mission_id": "stg_a",
        "path_fingerprint": fp,
        "configuration_revision": 0,
        "source": "rpp_conditioned_path",
    }

    node._path_cb(_path_msg(points, flags))

    assert node._path_model is None
    assert "configuration revision" in node._last_safety_block_reason


def test_conditioned_path_configuration_revision_mismatch_fails_closed():
    points = [(0.0, 0.0), (1.0, 0.0)]
    flags = [True, True]
    fp = path_geometry_fingerprint(points, flags)
    node = make_node()
    node._active_config = SprayConfiguration(
        mission_id="stg_a",
        path_fingerprint=fp,
        revision=3,
    )
    node._conditioned_path_identity = {
        "mission_id": "stg_a",
        "path_fingerprint": fp,
        "configuration_revision": 2,
        "source": "rpp_conditioned_path",
    }

    node._path_cb(_path_msg(points, flags))

    assert node._path_model is None
    assert "configuration revision mismatch" in node._last_safety_block_reason


def test_conditioned_path_clear_identity_clears_model_and_forces_off():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=0.35)
    msg = types.SimpleNamespace(
        data='{"configuration_revision":0,"mission_id":"","path_fingerprint":"","source":"clear"}'
    )

    node._conditioned_path_identity_cb(msg)

    assert node._path_model is None
    assert node._desired_debounced is False
    assert "cleared" in node._last_safety_block_reason


def test_runtime_status_includes_task02_diagnostics():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=0.2)
    node._distance_aware_tick()

    status = node.get_runtime_status()

    assert "actual_speed_mps" in status
    assert "target_flow" in status
    assert "current_pwm" in status
    assert "current_decision_event" in status
    assert "conditioned_path_source" in status
    assert "path_identity" in status
    assert "actuator_failure_state" in status
    assert status["target_speed_mps"] is None
    assert status["target_speed_source"] == "unavailable"
    assert status["physical_actuator_state"] == "UNAVAILABLE"
    assert status["physical_confirmation_available"] is False


def test_on_command_failure_fails_closed_and_retries_off():
    node = make_node()
    node._command_cli = _Cli(responses=[(False, 99), True])

    node._send_command(True, reason="edge")

    assert node._commanded is False
    assert node._actuator_state.current_on is False
    assert len(node._command_cli.requests) == 2


def test_anticipation_lead_scales_with_fresh_speed():
    slow = _decision(nozzle_n=0.93, speed_mps=0.2)
    fast = _decision(nozzle_n=0.88, speed_mps=1.0)
    assert slow.desired is False
    assert fast.event == "ON_EARLY"
    assert fast.desired is True


def test_disarm_retains_path_across_sustained_disarm():
    """The spray model is NEVER discarded on disarm: spray is already gated off
    by the armed/OFFBOARD safety gate, so retaining the path is harmless, and
    discarding it broke the normal mission flow (path is published once,
    sometimes before arm — clearing it left no model for the armed drive)."""
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=0.35)
    node._distance_aware_tick()
    assert node._commanded is True

    # Disarm edge: spray OFF immediately, but path retained.
    node._state_cb(types.SimpleNamespace(armed=False, mode="OFFBOARD"))
    assert node._commanded is False
    assert node._path_model is not None

    # Even a long sustained disarm must NOT clear the path now.
    node._clock.ns += 10_000_000_000  # 10s disarmed
    node._watchdog_tick()
    assert node._path_model is not None


def test_brief_disarm_flap_keeps_path_and_resumes_spray():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=0.35)
    node._distance_aware_tick()
    assert node._commanded is True

    # Transient single-message disarm then immediate re-arm (State flap).
    node._state_cb(types.SimpleNamespace(armed=False, mode="OFFBOARD"))
    node._state_cb(types.SimpleNamespace(armed=True, mode="OFFBOARD"))
    assert node._path_model is not None

    # Spray resumes on the next tick without any path republish.
    node._pose_recv_time = node.get_clock().now()
    node._vel_recv_time = node.get_clock().now()
    node._distance_aware_tick()
    assert node._commanded is True


def test_path_published_before_arm_survives_to_drive():
    """Regression for auto-spray never firing: the mission /path is published
    once (sometimes while still disarmed), then the rover arms and drives.
    The model must persist so spray engages on the armed drive — no republish."""
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=0.35)

    # Path already loaded while disarmed; a long pre-arm dwell elapses.
    node._state_cb(types.SimpleNamespace(armed=False, mode="OFFBOARD"))
    node._clock.ns += 5_000_000_000  # 5s disarmed before arming
    node._watchdog_tick()
    assert node._path_model is not None  # not cleared

    # Now arm + OFFBOARD and drive — spray must engage off the retained model.
    node._state_cb(types.SimpleNamespace(armed=True, mode="OFFBOARD"))
    node._pose_recv_time = node.get_clock().now()
    node._vel_recv_time = node.get_clock().now()
    node._distance_aware_tick()
    assert node._commanded is True


def test_xtrack_gate_forces_off_through_tick():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=0.35)
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


def test_runtime_entry_stays_off_until_original_mark_start():
    model = _build_path_model(
        points=[(-2.0, 0.0), (0.0, 0.0), (0.0, 0.0), (1.0, 0.0)],
        flags=[False, False, True, True],
    )

    entry = _decision(model=model, nozzle_n=-1.0, nozzle_e=0.0, speed_mps=0.0)
    boundary = _decision(model=model, nozzle_n=0.0, nozzle_e=0.0, speed_mps=0.0)

    assert entry.geometry_desired is False
    assert entry.desired is False
    # Progress-aware projection keeps TRANSIT at the shared vertex until
    # forward progress enters the MARK segment.
    assert boundary.geometry_desired is False


def test_full_distance_aware_tick_anticipatory_on_and_off():
    node = _make_distance_node(path_model=_straight_mark_path(), pose_n=0.945, speed=0.35)
    node._distance_aware_tick()

    assert node._desired_pub.msgs[-1] is True
    assert node._commanded_pub.msgs[-1] is True
    assert node._commanded is True

    node._pose_ned = (2.985, 0.0, 0.0)
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

    node._send_command(True, reason="edge")    # seq=1, pending ON
    on_future = node._command_cli.futures[-1]
    assert node._commanded is False
    assert node._actuator_state.pending is True
    assert node._actuator_state.pending_on is True

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

    node._send_command(True, reason="edge")    # seq=2, pending ON
    assert node._commanded is False
    assert node._actuator_state.pending_on is True

    off_future.fire(success=True)              # stale OFF success — ignored
    assert node._commanded is False
    assert node._off_confirmed is False

    node._command_cli.futures[-1].fire(success=True)
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
    node._send_command(True, reason="edge")    # seq=2, pending ON
    assert node._commanded is False
    assert node._actuator_state.pending_on is True

    off_future.fire(exc=RuntimeError("late failure"))  # stale failed OFF — ignored
    assert node._commanded is False
    assert node._off_confirmed is False


def test_on_command_does_not_publish_state_until_accepted():
    node = make_node()
    node._command_cli = _Cli(deferred=True)

    node._send_command(True, reason="edge")

    assert node._commanded is False
    assert node._actuator_state.pending is True
    assert node._actuator_state.pending_on is True
    assert node._state_pub.msgs == []
    assert node.get_runtime_status()["spray_state"] == "PENDING_ON"

    node._command_cli.futures[-1].fire(success=True)

    assert node._commanded is True
    assert node._state_pub.msgs[-1] is True
    assert node.get_runtime_status()["spray_state"] == "ACCEPTED_ON"


def test_on_rejection_never_reports_spray_on():
    node = make_node()
    node._command_cli = _Cli(responses=[(False, 99), True])

    node._send_command(True, reason="edge")

    assert node._commanded is False
    assert node._actuator_state.current_on is False
    assert node._actuator_state.pending is False
    assert node._state_pub.msgs[-1] is False
    assert len(node._command_cli.requests) == 2


def _runtime_ack(node):
    return node.get_runtime_status()


def test_successful_off_sets_off_acknowledged_and_confirmed_off():
    node = make_node()
    node._command_cli = _Cli(deferred=True)
    node._send_command(False, reason="edge")
    node._command_cli.futures[-1].fire(success=True)
    status = _runtime_ack(node)
    assert status["off_acknowledged"] is True
    assert status["confirmed_off"] is True


def test_on_after_off_clears_acknowledgement_flags():
    node = make_node()
    node._command_cli = _Cli(deferred=True)
    node._send_command(False, reason="edge")
    node._command_cli.futures[-1].fire(success=True)
    node._send_command(True, reason="edge")
    node._command_cli.futures[-1].fire(success=True)
    status = _runtime_ack(node)
    assert status["off_acknowledged"] is False
    assert status["confirmed_off"] is False


def test_off_pending_leaves_acknowledgement_flags_false():
    node = make_node()
    node._command_cli = _Cli(deferred=True)
    node._send_command(False, reason="edge")
    status = _runtime_ack(node)
    assert status["off_acknowledged"] is False
    assert status["confirmed_off"] is False


def test_rejected_off_leaves_acknowledgement_flags_false():
    node = make_node()
    node._command_cli = _Cli(deferred=True)
    node._send_command(False, reason="edge")
    node._command_cli.futures[-1].fire(success=False, result=4)
    status = _runtime_ack(node)
    assert status["off_acknowledged"] is False
    assert status["confirmed_off"] is False


def test_confirmed_off_false_when_commanded_on():
    node = make_node()
    node._actuator_state.off_confirmed = True
    node._commanded = True
    status = _runtime_ack(node)
    assert status["off_acknowledged"] is True
    assert status["confirmed_off"] is False


def test_runtime_status_physical_feedback_unavailable_contract():
    node = make_node()
    status = _runtime_ack(node)
    assert status["physical_confirmation_available"] is False
    assert status["physical_feedback_timestamp_monotonic_s"] is None
    assert status["physical_feedback_age_s"] is None
    assert status["physical_feedback_stale"] is True


def test_safety_loss_edge_forces_off_once():
    node = make_node(armed=True, mode="OFFBOARD")
    node._commanded = True
    node._off_confirmed = False
    node._last_safety_allows_on = True
    node._armed = False
    node._watchdog_tick()
    assert node._desired_pub.msgs[-1] is False
    assert node._last_safety_allows_on is False


def test_sustained_safety_loss_does_not_spam_desired_off():
    node = make_node(armed=False)
    node._last_safety_allows_on = False
    node._commanded = False
    node._off_confirmed = True
    for _ in range(20):
        node._watchdog_tick()
    assert len(node._desired_pub.msgs) == 0


def test_safety_loss_retries_failed_off_without_spamming_desired():
    node = make_node(armed=True, mode="OFFBOARD")
    node._commanded = True
    node._off_confirmed = False
    node._last_safety_allows_on = True
    node._command_cli = _Cli(deferred=True)
    node._armed = False
    node._watchdog_tick()
    assert node._desired_pub.msgs[-1] is False
    node._command_cli.futures[-1].fire(success=False, result=4)
    desired_count = len(node._desired_pub.msgs)
    request_count = len(node._command_cli.requests)
    node._clock.ns += 600_000_000
    for _ in range(5):
        node._watchdog_tick()
    assert len(node._desired_pub.msgs) == desired_count
    assert len(node._command_cli.requests) > request_count


def test_safety_recovery_does_not_auto_command_on():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=0.35)
    node._distance_aware_tick()
    assert node._commanded is True
    node._armed = False
    node._last_safety_allows_on = True
    node._watchdog_tick()
    assert node._commanded is False
    node._armed = True
    node._mode = "OFFBOARD"
    node._last_safety_allows_on = False
    node._watchdog_tick()
    assert node._commanded is False


def test_reassert_on_rejection_keeps_prior_accepted_on_until_off_ack():
    node = make_node()
    node._send_command(True, reason="edge")
    assert node._commanded is True
    node._command_cli = _Cli(deferred=True)

    node._send_command(True, reason="reassert")
    reassert_future = node._command_cli.futures[-1]
    reassert_future.fire(success=False, result=99)

    assert node._commanded is True
    assert node._actuator_state.current_on is True
    assert node._actuator_state.pending_on is False

    node._command_cli.futures[-1].fire(success=True)
    assert node._commanded is False
    assert node._off_confirmed is True


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print(f"ok {test.__name__}")
    print("PASS")


if __name__ == "__main__":
    main()
