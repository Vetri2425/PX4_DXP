#!/usr/bin/env python3
"""Unit tests for Spray Controller V2 distance-aware decisions."""

from __future__ import annotations

import math
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_spray_manual_override import _Cli, _Param, _bool_msg, make_node  # noqa: E402
import spray_controller_node as scn  # noqa: E402  (for monkeypatching time.monotonic)
from spray_controller_node import (  # noqa: E402
    MARK_TO_TRANSIT,
    TRANSIT_TO_MARK,
    _build_path_model,
    _make_spray_decision,
    _nozzle_position_ned,
    _project_onto_path,
)
from spray_fsm import SpraySafetyStateMachine, SprayState  # noqa: E402  (pure, no ROS)


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
    assert node._fsm.commanded is True
    assert node._command_cli.requests[-1].param1 == 1.0


def test_off_retry_after_failure_reaches_off_confirmed_via_recovery():
    """Supersedes the old _commanded/_off_confirmed boolean version of this
    test (which poked _force_off/_send_command directly — both deleted).
    A failed OFF ack must not be silently treated as confirmed-off: the FSM
    enters RECOVERY and retries once its backoff deadline elapses (plan
    §4). Intent preserved: /spray/state must never report True during the
    retry window, and the actuator is only reported off-confirmed once a
    retry actually succeeds. Drives the OFF via a normal desired-off
    transition (manual cancel) so safety_ok stays True throughout and the
    RECOVERY backoff path (not the safety-loss bypass — see
    test_disarm_safety_loss_forces_off_every_tick_bypassing_backoff below)
    is what's actually exercised."""
    node = make_node()
    node._manual_cb(_bool_msg(True))  # reach ON_CONFIRMED
    assert node._fsm.state == SprayState.ON_CONFIRMED

    fake_time = [1000.0]
    original_monotonic = scn.time.monotonic
    scn.time.monotonic = lambda: fake_time[0]
    try:
        node._command_cli = _Cli(responses=[(False, 99), True])
        node._manual_cb(_bool_msg(False))  # OFF dispatch; first ack fails

        assert len(node._command_cli.requests) == 1
        assert node._fsm.state == SprayState.RECOVERY
        assert node._fsm.spraying is False
        assert node._state_pub.msgs[-1] is False

        fake_time[0] += 0.6  # past the 0.5s first-attempt RECOVERY backoff
        node._reassert_tick()

        assert len(node._command_cli.requests) == 2
        assert node._fsm.state == SprayState.OFF_CONFIRMED
        assert node._state_pub.msgs[-1] is False
    finally:
        scn.time.monotonic = original_monotonic


def test_disarm_safety_loss_forces_off_then_stays_quiet_and_honors_backoff():
    """Safety loss (disarm) forces OFF immediately on the edge, but a
    SUSTAINED unsafe condition must NOT re-dispatch a forced OFF every tick.
    Regression guard for the ~50 Hz /mavros/cmd/command flood a disarmed
    rover would otherwise generate: once OFF is confirmed the FSM is quiet,
    and a failed OFF honors the RECOVERY backoff instead of hammering."""
    node = make_node()
    node._manual_cb(_bool_msg(True))
    assert node._fsm.state == SprayState.ON_CONFIRMED

    fake_time = [1000.0]
    original_monotonic = scn.time.monotonic
    scn.time.monotonic = lambda: fake_time[0]
    try:
        node._command_cli = _Cli(deferred=True)
        # Disarm: safety-loss edge forces OFF once (dispatch #1).
        node._state_cb(types.SimpleNamespace(armed=False, mode="OFFBOARD"))
        assert len(node._command_cli.requests) == 1
        off_future_1 = node._command_cli.futures[-1]
        off_future_1.fire(success=False, result=4)  # ack fails -> RECOVERY
        assert node._fsm.state == SprayState.RECOVERY

        # Still disarmed, NO time advance: backoff not elapsed -> no retry.
        for _ in range(10):
            node._reassert_tick()
        assert len(node._command_cli.requests) == 1, "must not hammer OFF within backoff"

        # Past the 0.5 s first-attempt backoff -> exactly one retry.
        fake_time[0] += 0.6
        node._reassert_tick()
        assert len(node._command_cli.requests) == 2
        node._command_cli.futures[-1].fire(success=True)
        assert node._fsm.state == SprayState.OFF_CONFIRMED

        # Confirmed OFF while still disarmed: sustained-unsafe ticks stay
        # silent (the anti-flood guarantee) even as time keeps advancing.
        for _ in range(20):
            fake_time[0] += 0.1
            node._reassert_tick()
        assert len(node._command_cli.requests) == 2, "must stay quiet once OFF confirmed"
        assert node._fsm.spraying is False
    finally:
        scn.time.monotonic = original_monotonic


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
    assert node._fsm.commanded is False
    assert "velocity stale" in node._last_safety_block_reason


def test_crawl_speed_end_to_end_keeps_spraying():
    """INVERTED 2026-07-17. This test used to assert that speed<min forced spray
    OFF ("below min spray speed"). That gate is gone: it collided with the frozen
    RPP corner speeds and dithered (157 valve fires for 25 real boundaries), and
    it made endpoint approach (0.03 m/s) structurally unsprayable.

    Speed now governs HOW MUCH (flow), never WHETHER. A rover crawling mid-MARK
    must keep painting -- only an explicit RPP pivot suppresses it.
    """
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=0.01)

    node._distance_aware_tick()

    assert node._desired_debounced is True
    assert node._fsm.commanded is True
    assert node._last_safety_block_reason == ""


def test_anticipation_lead_scales_with_fresh_speed():
    slow = _decision(nozzle_n=0.93, speed_mps=0.2)
    fast = _decision(nozzle_n=0.93, speed_mps=1.0)
    assert slow.desired is False
    assert fast.event == "on_early"
    assert fast.desired is True


def test_disarm_retains_path_across_sustained_disarm():
    """The spray model is NEVER discarded on disarm: spray is already gated off
    by the armed/OFFBOARD safety gate, so retaining the path is harmless, and
    discarding it broke the normal mission flow (path is published once,
    sometimes before arm — clearing it left no model for the armed drive)."""
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)
    node._distance_aware_tick()
    assert node._fsm.commanded is True

    # Disarm edge: spray OFF immediately, but path retained.
    node._state_cb(types.SimpleNamespace(armed=False, mode="OFFBOARD"))
    assert node._fsm.commanded is False
    assert node._path_model is not None

    # Even a long sustained disarm must NOT clear the path now.
    node._clock.ns += 10_000_000_000  # 10s disarmed
    node._watchdog_tick()
    assert node._path_model is not None


def test_brief_disarm_flap_keeps_path_and_resumes_spray():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)
    node._distance_aware_tick()
    assert node._fsm.commanded is True

    # Transient single-message disarm then immediate re-arm (State flap).
    node._state_cb(types.SimpleNamespace(armed=False, mode="OFFBOARD"))
    node._state_cb(types.SimpleNamespace(armed=True, mode="OFFBOARD"))
    assert node._path_model is not None

    # Spray resumes on the next tick without any path republish.
    node._pose_recv_time = node.get_clock().now()
    node._vel_recv_time = node.get_clock().now()
    node._distance_aware_tick()
    assert node._fsm.commanded is True


def test_path_published_before_arm_survives_to_drive():
    """Regression for auto-spray never firing: the mission /path is published
    once (sometimes while still disarmed), then the rover arms and drives.
    The model must persist so spray engages on the armed drive — no republish."""
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)

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
    assert node._fsm.commanded is True


def test_xtrack_gate_forces_off_through_tick():
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)
    node._pose_ned = (1.0, 0.25, 0.0)  # 0.25m off the e=0 line > 0.10 gate
    node._pose_recv_time = node.get_clock().now()
    node._distance_aware_tick()
    assert node._desired_debounced is False
    assert node._fsm.commanded is False
    assert "xtrack error" in node._last_safety_block_reason


def test_startup_unconfirmed_off_is_commanded_while_disarmed():
    # Mirrors the __init__ startup state: actuator OFF not yet confirmed.
    node = make_node(armed=False)
    node._params["use_distance_aware_spray"] = _Param(True)
    node._fsm = SpraySafetyStateMachine()  # fresh: OFF_UNCONFIRMED

    node._watchdog_tick()

    assert node._command_cli.requests
    assert node._command_cli.requests[-1].param1 == -1.0
    assert node._fsm.state == SprayState.OFF_CONFIRMED


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
    assert node._fsm.commanded is True

    node._pose_ned = (2.96, 0.0, 0.0)
    node._pose_recv_time = node.get_clock().now()
    node._distance_aware_tick()

    assert node._desired_pub.msgs[-1] is False
    assert node._commanded_pub.msgs[-1] is False
    assert node._fsm.commanded is False


def test_late_on_success_does_not_resurrect_commanded_on():
    # ON dispatched, then OFF supersedes and confirms; the late ON reply must
    # not flip state back to ON. Rewritten against the FSM-backed node API
    # (_send_command no longer exists — the FSM owns dispatch); driven via
    # manual_cb so an ON_PENDING -> desired(OFF) supersede is reachable
    # (per the FSM's own transition table, this direction IS supersedable,
    # unlike OFF_PENDING — see the tests below).
    node = make_node()
    node._command_cli = _Cli(deferred=True)

    node._manual_cb(_bool_msg(True))             # seq=1, ON_PENDING (deferred)
    on_future = node._command_cli.futures[-1]
    assert node._fsm.state == SprayState.ON_PENDING

    node._manual_cb(_bool_msg(False))             # seq=2 supersedes: ON_PENDING + desired(OFF) -> OFF_PENDING
    off_future = node._command_cli.futures[-1]
    off_future.fire(success=True)                 # latest OFF confirms
    assert node._fsm.state == SprayState.OFF_CONFIRMED

    on_future.fire(success=True)                  # stale ON reply — ignored
    assert node._fsm.state == SprayState.OFF_CONFIRMED


def test_stale_off_ack_after_newer_on_does_not_clobber_state():
    # Supersedes test_late_off_success_does_not_clear_newer_on. Note: unlike
    # ON_PENDING (which the FSM allows a desired-flip to supersede with a
    # fresh OFF before its ack lands), OFF_PENDING is NOT supersedable — the
    # FSM's transition table has no "OFF_PENDING + desired(ON)" row, so an
    # ON is queued behind an in-flight OFF's ack rather than racing ahead of
    # it (spray_fsm.py). This test therefore drives the OFF to completion
    # first, then manufactures a stale/duplicate reply for that same
    # (now-superseded) seq after a newer ON has since been dispatched — the
    # original hazard this test protects against: a late/duplicate reply
    # carrying a superseded cmd_seq must never override newer state.
    node = make_node()
    node._command_cli = _Cli(deferred=True)

    node._manual_cb(_bool_msg(True))    # seq=1, ON_PENDING (deferred)
    node._manual_cb(_bool_msg(False))   # seq=2 supersedes -> OFF_PENDING (deferred)
    off_future = node._command_cli.futures[-1]
    off_future.fire(success=True)
    assert node._fsm.state == SprayState.OFF_CONFIRMED

    node._manual_cb(_bool_msg(True))    # seq=3, newer ON dispatch (deferred)
    assert node._fsm.state == SprayState.ON_PENDING

    off_future.fire(success=True)       # duplicate/late reply for the stale seq=2 — must be ignored
    assert node._fsm.state == SprayState.ON_PENDING
    assert node._fsm.commanded is True

    on_future = node._command_cli.futures[-1]
    on_future.fire(success=True)
    assert node._fsm.state == SprayState.ON_CONFIRMED


def test_stale_exception_reply_ignored_does_not_corrupt_newer_state():
    # Supersedes test_stale_failed_result_ignored_does_not_corrupt_state. A
    # stale reply that raises (rather than returning success=False) must
    # also be ignored via the seq guard — exercises the same INVARIANT 2
    # guard as the test above, but through the future.result() exception
    # path in _command_done.
    node = make_node()
    node._command_cli = _Cli(deferred=True)

    node._manual_cb(_bool_msg(True))
    node._manual_cb(_bool_msg(False))   # OFF dispatch (seq=2), deferred
    off_future = node._command_cli.futures[-1]
    off_future.fire(success=True)
    assert node._fsm.state == SprayState.OFF_CONFIRMED

    node._manual_cb(_bool_msg(True))    # newer ON dispatch (seq=3), deferred
    assert node._fsm.state == SprayState.ON_PENDING

    off_future.fire(exc=RuntimeError("late failure"))  # stale exception reply — ignored
    assert node._fsm.state == SprayState.ON_PENDING
    assert node._fsm.commanded is True

    node._command_cli.futures[-1].fire(success=True)
    assert node._fsm.state == SprayState.ON_CONFIRMED


# --------------------------------------------------------------------------
# B4 — terminal shutoff + mission-level fail-closed watchdog. Field bug
# 2026-07-25: the valve never closed at mission end. The rover finishes at a
# creep speed (0.005-0.03 m/s) where the speed-scaled off_lead is ~1 mm, so it
# stops ~14 mm short of the final station and the geometric MARK->TRANSIT
# boundary is never crossed. Every pre-existing decision test used speed 1.0,
# where off_lead = 0.05 m swallows the gap — which is exactly why this was
# missed. These use the real terminal speed regime.
# --------------------------------------------------------------------------

def _terminal_mark_path():
    """A path that ends ON a MARK — the final station carries the terminal
    MARK->TRANSIT boundary (real mission geometry, 4.804 m long)."""
    return _build_path_model([(0.0, 0.0), (4.804, 0.0)], [True, True])


def test_terminal_shutoff_when_stopped_short_at_endpoint():
    """Reproduces B4: stop 13.7 mm short at 0.008 m/s -> valve must go OFF.

    Before the fix this returned geometry_desired=True with event='' — the
    valve latched ON and only a disarm/e-stop could close it."""
    d = _make_spray_decision(
        model=_terminal_mark_path(), nozzle_n=4.790, nozzle_e=0.0,
        speed_mps=0.008, safety_ok=True, safety_reason="",
        solenoid_open_delay_s=0.10, solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02, off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
    )
    assert d.geometry_desired is False
    assert d.event == "terminal_off"
    assert d.desired is False


def test_terminal_shutoff_at_exact_endpoint():
    d = _make_spray_decision(
        model=_terminal_mark_path(), nozzle_n=4.804, nozzle_e=0.0,
        speed_mps=0.0, safety_ok=True, safety_reason="",
        solenoid_open_delay_s=0.10, solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02, off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
    )
    assert d.geometry_desired is False


def test_terminal_shutoff_is_end_specific_not_a_speed_gate():
    """Stopped mid-MARK (far from the final station) must KEEP painting — the
    terminal shutoff keys on proximity to the last station, not on speed."""
    d = _make_spray_decision(
        model=_terminal_mark_path(), nozzle_n=2.0, nozzle_e=0.0,
        speed_mps=0.008, safety_ok=True, safety_reason="",
        solenoid_open_delay_s=0.10, solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02, off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
    )
    assert d.geometry_desired is True
    assert d.event == ""


def test_terminal_shutoff_does_not_fire_while_moving_near_end():
    """Near the end but moving at cruise: the terminal path stays inactive
    (off_early owns that regime); terminal shutoff must not pre-empt the tail
    at speed. off_lead = 1.0*0.05 = 0.05 >= 0.014 gap -> off_early here."""
    d = _make_spray_decision(
        model=_terminal_mark_path(), nozzle_n=4.790, nozzle_e=0.0,
        speed_mps=1.0, safety_ok=True, safety_reason="",
        solenoid_open_delay_s=0.10, solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02, off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
    )
    assert d.event == "off_early"          # NOT terminal_off
    assert d.geometry_desired is False


def test_active_stale_forces_off_in_distance_aware_mode():
    """RPP stops publishing /spray/active -> fail-closed OFF after the timeout."""
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)
    node._active_cb(_bool_msg(True))       # RPP asserting a MARK
    node._distance_aware_tick()
    assert node._fsm.commanded is True     # geometry sprays

    node._clock.ns += 600_000_000          # 0.6 s since last /spray/active > 0.5
    node._pose_recv_time = node.get_clock().now()
    node._vel_recv_time = node.get_clock().now()
    node._distance_aware_tick()
    assert node._fsm.commanded is False
    # ...and the fail-closed watchdog is what named the cause.
    assert any(
        rec[0] == "warn" and "watchdog" in rec[1][0]
        for rec in node._logger.records
    )


def test_active_false_sustained_forces_off_in_distance_aware_mode():
    """/spray/active held False (mission ended) forces OFF after active_timeout_s
    even while the topic is still fresh."""
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)
    node._active_cb(_bool_msg(False))      # RPP: not in a MARK
    node._distance_aware_tick()            # within timeout: geometry still sprays
    assert node._fsm.commanded is True

    node._clock.ns += 600_000_000
    node._active_cb(_bool_msg(False))      # fresh False, but false_since is old
    node._pose_recv_time = node.get_clock().now()
    node._vel_recv_time = node.get_clock().now()
    node._distance_aware_tick()
    assert node._fsm.commanded is False


def test_active_true_does_not_force_off():
    """A fresh, True /spray/active must never trip the watchdog."""
    node = _make_distance_node(path_model=_mark_only_path(), pose_n=1.0, speed=1.0)
    for _ in range(5):
        node._active_cb(_bool_msg(True))
        node._clock.ns += 100_000_000
        node._pose_recv_time = node.get_clock().now()
        node._vel_recv_time = node.get_clock().now()
        node._distance_aware_tick()
    assert node._fsm.commanded is True


# --------------------------------------------------------------------------
# Phase C — dash mode integration through _make_spray_decision + the node's
# /spray/session_config callback (B0). The dash arc-length math itself is
# covered purely in test_spray_dash_v2.py; these tests prove the WIRING:
# the decision routes to the meter, and the callback selects the mode.
# --------------------------------------------------------------------------

import json as _json  # noqa: E402

from spray_modes import DashMeter, PointMeter, PointUpdate  # noqa: E402
from spray_session_config import (  # noqa: E402
    SCHEMA_VERSION,
    DashConfig,
    PointsModeConfig,
    SpraySessionConfig,
    to_dict,
)


class _Msg:
    """Minimal std_msgs/String stand-in — the callback only reads .data."""

    def __init__(self, data):
        self.data = data


def _dash_drive(model, meter, n_start, n_end, *, step=0.02, speed=1.0, dt=1.0):
    """Walk the nozzle n_start→n_end along a straight N-axis MARK path.

    On this path projection.s == nozzle_n, so driving the nozzle drives the
    meter's arc-length. Fine steps stay inside the jump-tolerance window.
    """
    d = None
    k = max(1, int(round((n_end - n_start) / step)))
    for i in range(k + 1):
        n = n_start + i * step
        d = _make_spray_decision(
            model=model, nozzle_n=n, nozzle_e=0.0, speed_mps=speed,
            safety_ok=True, safety_reason="",
            solenoid_open_delay_s=0.0, solenoid_close_delay_s=0.0,
            on_overspray_margin_m=0.0, off_overspray_margin_m=0.0,
            max_xtrack_error_m=0.10, mode="dash", dash_meter=meter, dt_s=dt,
        )
    return d


def test_dash_decision_routes_to_meter():
    """6-on/3-off dash: ON in [0,6), OFF in [6,9); no static next_boundary."""
    model = _build_path_model([(0.0, 0.0), (10.0, 0.0)], [True, True])
    meter = DashMeter(6.0, 3.0, "on")
    on_region = _dash_drive(model, meter, 0.0, 5.9)
    assert on_region.desired is True
    assert on_region.geometry_desired is True
    assert on_region.next_boundary is None  # dash boundaries are dynamic (§6)
    off_region = _dash_drive(model, meter, 5.9, 6.1)
    assert off_region.desired is False


def test_dash_respects_safety_gate():
    """Even mid-ON dash, a safety_ok=False input forces desired False."""
    model = _build_path_model([(0.0, 0.0), (10.0, 0.0)], [True, True])
    meter = DashMeter(6.0, 3.0, "on")
    _dash_drive(model, meter, 0.0, 3.0)  # arm + into ON region
    blocked = _make_spray_decision(
        model=model, nozzle_n=3.0, nozzle_e=0.0, speed_mps=1.0,
        safety_ok=False, safety_reason="pivoting in place",
        solenoid_open_delay_s=0.0, solenoid_close_delay_s=0.0,
        on_overspray_margin_m=0.0, off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10, mode="dash", dash_meter=meter, dt_s=1.0,
    )
    assert blocked.geometry_desired is True   # pattern still wants ON
    assert blocked.desired is False           # but safety gate wins


def test_dash_never_paints_transit_connector():
    """R5: meter may be ON through a TRANSIT run; valve must stay shut there.

    Path: TRANSIT [0,2) then MARK [2,12]. 6-on pattern arms on the transit and
    would paint it without the current_flag AND.
    """
    model = _build_path_model(
        [(0.0, 0.0), (2.0, 0.0), (12.0, 0.0)],
        [False, True, True],
    )
    meter = DashMeter(6.0, 3.0, "on", anchor_s=2.0)
    # Fine-step the transit; every tick must refuse to spray.
    k = max(1, int(round(1.9 / 0.02)))
    for i in range(k + 1):
        n = 0.0 + i * 0.02
        d = _make_spray_decision(
            model=model, nozzle_n=n, nozzle_e=0.0, speed_mps=1.0,
            safety_ok=True, safety_reason="",
            solenoid_open_delay_s=0.0, solenoid_close_delay_s=0.0,
            on_overspray_margin_m=0.0, off_overspray_margin_m=0.0,
            max_xtrack_error_m=0.10, mode="dash", dash_meter=meter, dt_s=1.0,
        )
        assert d.desired is False, f"painted transit at n={n:.3f}"
        assert d.geometry_desired is False
    # Enter the MARK: meter still ON (anchor at 2, 6 m on) → may spray.
    on_mark = _make_spray_decision(
        model=model, nozzle_n=2.5, nozzle_e=0.0, speed_mps=1.0,
        safety_ok=True, safety_reason="",
        solenoid_open_delay_s=0.0, solenoid_close_delay_s=0.0,
        on_overspray_margin_m=0.0, off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10, mode="dash", dash_meter=meter, dt_s=1.0,
    )
    assert on_mark.desired is True
    assert on_mark.geometry_desired is True


def test_dash_terminal_shutoff_when_stopped_short():
    """R6: B4 terminal shutoff must fire in dash mode, not only continuous."""
    model = _terminal_mark_path()  # 4.804 m all-MARK, ends ON
    meter = DashMeter(6.0, 3.0, "on", anchor_s=0.0)
    # Arm and drive into the ON phase near the end (still within first 6 m ON).
    _dash_drive(model, meter, 0.0, 4.79, step=0.02, speed=1.0, dt=1.0)
    d = _make_spray_decision(
        model=model, nozzle_n=4.790, nozzle_e=0.0, speed_mps=0.008,
        safety_ok=True, safety_reason="",
        solenoid_open_delay_s=0.10, solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02, off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10, mode="dash", dash_meter=meter, dt_s=1.0,
    )
    assert d.geometry_desired is False
    assert d.event == "terminal_off"
    assert d.desired is False


def test_session_config_cb_selects_dash():
    node = make_node()
    cfg = SpraySessionConfig(
        schema_version=SCHEMA_VERSION, mode="dash", points=(), flags=(),
        dash=DashConfig(6.0, 3.0, "on"), points_mode=None,
    )
    node._session_config_cb(_Msg(_json.dumps(to_dict(cfg))))
    assert node._session_mode == "dash"
    assert node._dash_meter is not None
    assert node._dash_meter.on_distance_m == 6.0
    assert node._dash_meter.off_distance_m == 3.0


def test_session_config_cb_back_to_continuous_clears_meter():
    node = make_node()
    node._session_config_cb(_Msg(_json.dumps(to_dict(SpraySessionConfig(
        SCHEMA_VERSION, "dash", (), (), DashConfig(6.0, 3.0, "on"), None)))))
    assert node._dash_meter is not None
    node._session_config_cb(_Msg(_json.dumps(to_dict(SpraySessionConfig(
        SCHEMA_VERSION, "continuous", (), (), None, None)))))
    assert node._session_mode == "continuous"
    assert node._dash_meter is None


def test_bad_session_config_keeps_last_mode():
    node = make_node()
    node._session_config_cb(_Msg(_json.dumps(to_dict(SpraySessionConfig(
        SCHEMA_VERSION, "dash", (), (), DashConfig(6.0, 3.0, "on"), None)))))
    assert node._session_mode == "dash"
    node._session_config_cb(_Msg("{ not valid json"))          # malformed
    assert node._session_mode == "dash"                         # unchanged
    node._session_config_cb(_Msg(_json.dumps({"schema_version": 999, "mode": "continuous"})))
    assert node._session_mode == "dash"                         # schema mismatch ignored


# --------------------------------------------------------------------------
# Phase D — point mode wiring through _make_spray_decision + the callback.
# The point FSM itself is covered in test_spray_point_v2.py; these prove the
# node routing, config selection, and the pivot-gate exemption.
# --------------------------------------------------------------------------

def test_session_config_cb_selects_point():
    node = make_node()
    cfg = SpraySessionConfig(
        SCHEMA_VERSION, "point", (), (), None,
        PointsModeConfig(((0.0, 0.0), (1.0, 0.0)), 0.05, None, 0.2, 1.0),
    )
    node._session_config_cb(_Msg(_json.dumps(to_dict(cfg))))
    assert node._session_mode == "point"
    assert node._point_meter is not None
    assert len(node._point_meter.coordinates) == 2
    assert node._dash_meter is None


def _path_msg(points_xyz):
    """Build a nav_msgs/Path stub: each tuple is (x=north, y=east, z=bitfield)."""
    poses = [
        types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=x, y=y, z=z)
            )
        )
        for (x, y, z) in points_xyz
    ]
    return types.SimpleNamespace(poses=poses)


def test_point_meter_uses_path_must_hit_over_config_coords():
    """The fix: point dwell targets are the /path must-hit vertices (placed,
    frame-correct), not the server-staged coords. Config coords are a fallback
    used only until a must-hit /path arrives."""
    node = make_node()
    # Point mode selected via config carrying only a bench FALLBACK coord.
    cfg = SpraySessionConfig(
        SCHEMA_VERSION, "point", (), (), None,
        PointsModeConfig(((99.0, 99.0),), 0.12, None, 0.2, 2.0),
    )
    node._session_config_cb(_Msg(_json.dumps(to_dict(cfg))))
    # No /path yet → the config fallback coordinate is used.
    assert node._point_meter is not None
    assert node._point_meter.coordinates == ((99.0, 99.0),)

    # A placed /path arrives: 4 pts, 2 flagged must-hit (z bit1=2; z=3 = on+must).
    node._path_cb(_path_msg([
        (0.0, 0.0, 1.0),   # spray on, not must-hit
        (1.0, 0.0, 3.0),   # spray on + must-hit  → dwell target
        (2.0, 0.0, 1.0),
        (3.0, 0.0, 3.0),   # spray on + must-hit  → dwell target
    ]))
    # Meter rebuilt from the /path must-hit vertices, NOT the config fallback.
    assert node._path_must_hit_points == [(1.0, 0.0), (3.0, 0.0)]
    assert node._point_meter.coordinates == ((1.0, 0.0), (3.0, 0.0))
    # Params from the session_config are preserved across the rebuild.
    assert node._point_meter.dwell_s == 2.0
    assert node._point_meter.arrival_tolerance_m == 0.12
    # Still point mode (did NOT fall back to continuous).
    assert node._session_mode == "point"


def test_point_mode_no_meter_commands_off_not_continuous():
    """Point mode with no resolved coordinates must spray OFF, never fall
    through to path-projection (continuous) spraying."""
    d = _make_spray_decision(
        model=_straight_mark_path(), nozzle_n=1.5, nozzle_e=0.0, speed_mps=1.0,
        safety_ok=True, safety_reason="",
        solenoid_open_delay_s=0.0, solenoid_close_delay_s=0.0,
        on_overspray_margin_m=0.0, off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10, mode="point", point_meter=None,
        yaw=0.0, now_s=1.0, off_confirmed=True,
    )
    assert d.geometry_desired is False and d.desired is False
    assert d.point_update is None


def test_point_decision_routes_to_meter():
    """mode=point: geometry comes from the PointMeter, no path projection."""
    meter = PointMeter([(0.0, 0.0)], 0.10, arrival_settle_s=0.0, dwell_s=0.5)
    d = _make_spray_decision(
        model=None, nozzle_n=0.0, nozzle_e=0.0, speed_mps=0.0,
        safety_ok=True, safety_reason="",
        solenoid_open_delay_s=0.0, solenoid_close_delay_s=0.0,
        on_overspray_margin_m=0.0, off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10, mode="point", point_meter=meter,
        yaw=0.0, now_s=1.0, off_confirmed=True,
    )
    assert d.geometry_desired is True and d.desired is True   # dwelling on the dot
    assert d.next_boundary is None and d.point_update is not None
    assert d.point_update.phase == "dwelling"


def test_point_config_gate_and_pivot_exemption():
    node = make_node(armed=True, mode="OFFBOARD", require_offboard=True)
    node._session_mode = "point"
    # No point config yet → its own gate reason (not "path not loaded").
    ok, reason = node._auto_safety_status(pose_fresh=True, speed=0.0, velocity_fresh=True)
    assert ok is False and reason == "point config not loaded"
    # Load a point meter and simulate the RPP pivoting in place.
    node._point_meter = PointMeter([(0.0, 0.0)], 0.10, 0.0, 0.5)
    node._segment_state = 3  # CORNER_ALIGN
    node._segment_state_recv_time = node.get_clock().now()
    # Not dwelling on a dot → pivot gate blocks.
    ok, reason = node._auto_safety_status(pose_fresh=True, speed=0.0, velocity_fresh=True)
    assert ok is False and reason == "pivoting in place"
    # Dwelling on the active dot → exemption lets spray through the pivot.
    node._last_point_update = PointUpdate(True, "dwelling", 0, True, False, -1)
    ok, reason = node._auto_safety_status(pose_fresh=True, speed=0.0, velocity_fresh=True)
    assert ok is True and reason == ""


# --------------------------------------------------------------------------
# Phase E — speed-proportional flow wiring (the FlowModulator math itself is
# covered in test_spray_flow_model.py). Drives _fsm._state directly to ON to
# exercise the commanded path without the full ACK handshake.
# --------------------------------------------------------------------------

def test_flow_current_on_value_defaults_to_full():
    node = make_node()
    node._commanded_flow_value = None
    assert node._current_on_value() == 1.0          # on_value
    node._commanded_flow_value = 0.55
    assert node._current_on_value() == 0.55


def test_flow_disabled_is_full_flow():
    node = make_node()                               # flow_modulation_enabled False
    node._fsm._state = SprayState.ON_CONFIRMED
    node._update_flow(0.2, 0.02)
    assert node._commanded_flow_value is None
    assert node._flow_source == "n/a"
    assert node._current_on_value() == 1.0


def test_flow_speed_scaled_when_enabled_and_moving():
    node = make_node()
    node._params["flow_modulation_enabled"] = _Param(True)
    node._fsm._state = SprayState.ON_CONFIRMED
    for _ in range(400):                             # ramp up at rated speed
        node._update_flow(0.35, 0.02)                # rated = 0.35 → full flow
    assert node._flow_source == "speed_scaled"
    assert abs(node._commanded_flow_value - 1.0) < 1e-3
    for _ in range(400):                             # slow to a crawl
        node._update_flow(0.0, 0.02)
    assert abs(node._commanded_flow_value - 0.2) < 1e-3   # floors at min_flow_value


def test_flow_point_mode_uses_fixed_dwell_value():
    node = make_node()
    node._params["flow_modulation_enabled"] = _Param(True)
    node._params["point_dwell_flow_value"] = _Param(0.8)
    node._session_mode = "point"
    node._fsm._state = SprayState.ON_CONFIRMED
    node._update_flow(0.0, 0.02)                     # a dot sprays at standstill
    assert node._flow_source == "point_fixed"
    assert node._commanded_flow_value == 0.8


def test_flow_not_modulated_during_manual():
    node = make_node()
    node._params["flow_modulation_enabled"] = _Param(True)
    node._manual_active = True
    node._fsm._state = SprayState.ON_CONFIRMED
    node._update_flow(0.35, 0.02)
    assert node._commanded_flow_value is None and node._flow_source == "n/a"


# --------------------------------------------------------------------------
# B5 — spurious spray pulse on path load. Field bug 2026-07-25: the mission
# /path lands with the rover parked ON vertex 0 (spray bit set), already armed
# + OFFBOARD, so geometry alone opened the valve (~271 ms actuator ON — a paint
# blob at the start vertex) until the RPP's first CORNER_ALIGN suppressed it.
# The fix: auto-spray waits for positive evidence the run has started — an
# actively-tracking state on /rpp/segment_debug SINCE the /path load. These
# tests drive the real _path_cb (which arms the gate); the many other distance-
# aware tests inject _path_model directly, so they stay permissive as before.
# --------------------------------------------------------------------------


def _b5_node_on_vertex0():
    node = make_node()  # armed + OFFBOARD
    node._params["use_distance_aware_spray"] = _Param(True)
    node._pose_ned = (0.0, 0.0, 0.0)  # parked ON vertex 0
    node._pose_recv_time = node.get_clock().now()
    node._vel_ned = (0.0, 0.0)  # stationary
    node._vel_recv_time = node.get_clock().now()
    # Mission /path arrives via the real callback (resets the tracking gate).
    # z=3 => spray bit + must-hit bit set on both vertices (a MARK line).
    node._path_cb(_path_msg([(0.0, 0.0, 3.0), (2.0, 0.0, 3.0)]))
    return node


def test_b5_path_load_on_vertex0_does_not_spray_before_tracking():
    node = _b5_node_on_vertex0()
    assert node._tracking_seen_since_path_load is False
    # Geometry wants ON (nozzle over MARK at s=0), but the run has not started.
    ok, reason = node._auto_safety_status(
        pose_fresh=True, speed=0.0, velocity_fresh=True)
    assert ok is False and reason == "awaiting tracking"
    node._distance_aware_tick()
    assert node._fsm.commanded is False  # no blob at vertex 0
    assert node._last_safety_block_reason == "awaiting tracking"


def test_b5_geometry_rules_once_tracking_seen():
    node = _b5_node_on_vertex0()
    node._segment_debug_cb(_Msg([1.0, 1.0]))  # RPP starts tracking: TRACK_SEGMENT
    assert node._tracking_seen_since_path_load is True
    node._pose_recv_time = node.get_clock().now()
    node._vel_recv_time = node.get_clock().now()
    node._distance_aware_tick()
    assert node._fsm.commanded is True  # now geometry rules -> ON


def test_b5_pretracking_states_do_not_release_gate():
    """The pre-tracking states the RPP actually emits at a run boundary
    (CORNER_STOP / DONE) are NOT tracking and must NOT release the B5 gate —
    only a genuinely-tracking state does. Guards against B3's fix (the extra
    segment_debug edge) accidentally re-opening B5."""
    node = _b5_node_on_vertex0()
    node._segment_debug_cb(_Msg([1.0, 5.0]))  # CORNER_STOP
    node._segment_debug_cb(_Msg([1.0, 4.0]))  # DONE
    node._segment_debug_cb(_Msg([1.0, 3.0]))  # CORNER_ALIGN
    assert node._tracking_seen_since_path_load is False
    ok, reason = node._auto_safety_status(
        pose_fresh=True, speed=0.0, velocity_fresh=True)
    assert ok is False and reason == "awaiting tracking"


def test_b5_new_path_re_arms_gate():
    """Two-stage entry: after the entry path is tracked, the marking /path
    lands and must re-arm the gate — no spray until it, too, is tracked."""
    node = _b5_node_on_vertex0()
    node._segment_debug_cb(_Msg([1.0, 1.0]))  # tracked the (entry) path
    assert node._tracking_seen_since_path_load is True
    node._path_cb(_path_msg([(0.0, 0.0, 3.0), (2.0, 0.0, 3.0)]))  # marking path
    assert node._tracking_seen_since_path_load is False
    ok, reason = node._auto_safety_status(
        pose_fresh=True, speed=0.0, velocity_fresh=True)
    assert ok is False and reason == "awaiting tracking"


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for test in tests:
        test()
        print(f"ok {test.__name__}")
    print("PASS")


if __name__ == "__main__":
    main()
