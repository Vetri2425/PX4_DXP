#!/usr/bin/env python3
"""Task_18 spray runtime distance and flow control hardening tests."""

from __future__ import annotations

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from path_identity import path_geometry_fingerprint  # noqa: E402
from spray_config import SprayConfiguration, validate_spray_configuration  # noqa: E402
from spray_controller_modes import (  # noqa: E402
    ContinuousSprayRuntimeState,
    continuous_distance_decision,
    point_mode_decision,
)
from spray_path_model import (  # noqa: E402
    SprayProjectionState,
    apply_distance_hysteresis,
    build_path_model,
    make_spray_decision,
    project_onto_path,
    ramp_pwm,
)
from spray_runtime_protocol import serialize_runtime_status  # noqa: E402
from test_spray_manual_override import _Param, make_node  # noqa: E402


def _config(**overrides) -> SprayConfiguration:
    raw = {
        "mission_id": "task18",
        "path_fingerprint": "abc",
        "configuration_revision": 1,
        **overrides,
    }
    return validate_spray_configuration(raw)


def _runtime_state() -> ContinuousSprayRuntimeState:
    return ContinuousSprayRuntimeState(projection=SprayProjectionState())


# ── Projection ───────────────────────────────────────────────────────────────


def test_tie_does_not_choose_later_segment():
    model = build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
        flags=[True, True, True],
    )
    state = SprayProjectionState(valid=True, segment_index=0, s=0.5)
    proj = project_onto_path(
        model,
        1.0,
        0.05,
        previous_segment_index=state.segment_index,
        previous_s=state.s,
    )
    assert proj is not None
    assert proj.segment_index == 0


def test_parallel_mark_line_cannot_steal_projection():
    model = build_path_model(
        points=[(0.0, 0.0), (2.0, 0.0), (0.0, 0.05), (2.0, 0.05)],
        flags=[False, False, True, True],
    )
    state = SprayProjectionState(valid=True, segment_index=0, s=1.0)
    proj = project_onto_path(
        model,
        1.0,
        0.01,
        previous_segment_index=state.segment_index,
        previous_s=state.s,
        max_projection_jump_m=0.50,
        ambiguity_distance_m=0.03,
    )
    assert proj is not None
    assert not proj.ambiguous
    assert proj.segment_index == 0


def test_ambiguity_clearance_confidence_on_segment_is_one():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    proj = project_onto_path(model, 1.5, 0.0, ambiguity_distance_m=0.10)
    assert proj is not None
    assert proj.ambiguity_clearance_confidence == 1.0
    assert proj.confidence == 1.0


def test_ambiguity_clearance_confidence_at_threshold_is_zero():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    threshold = 0.10
    proj = project_onto_path(model, 1.5, threshold, ambiguity_distance_m=threshold)
    assert proj is not None
    assert proj.ambiguity_clearance_confidence == 0.0
    assert proj.confidence == 0.0


def test_ambiguity_clearance_confidence_scales_with_xtrack_not_max_xtrack_gate():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    threshold = 0.10
    proj = project_onto_path(model, 1.5, 0.05, ambiguity_distance_m=threshold)
    assert proj is not None
    expected = 1.0 - 0.05 / threshold
    assert math.isclose(proj.ambiguity_clearance_confidence, expected, rel_tol=1e-9)


def test_self_crossing_geometry_ambiguous():
    model = build_path_model(
        points=[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)],
        flags=[True, True, True, True, True],
    )
    proj = project_onto_path(model, 1.0, 1.0, ambiguity_distance_m=0.50)
    assert proj is not None
    assert proj.ambiguous


def test_large_forward_projection_jump_rejected():
    model = build_path_model(
        points=[(0.0, 0.0), (5.0, 0.0)],
        flags=[True, True],
    )
    state = SprayProjectionState(valid=True, segment_index=0, s=0.5)
    proj = project_onto_path(
        model,
        4.5,
        0.0,
        previous_segment_index=state.segment_index,
        previous_s=state.s,
        max_projection_jump_m=0.50,
    )
    assert proj is not None
    assert proj.ambiguous


def test_backward_arc_length_jump_rejected():
    model = build_path_model(
        points=[(0.0, 0.0), (5.0, 0.0)],
        flags=[True, True],
    )
    state = SprayProjectionState(valid=True, segment_index=0, s=3.0)
    proj = project_onto_path(
        model,
        1.0,
        0.0,
        previous_segment_index=state.segment_index,
        previous_s=state.s,
        max_backward_jump_m=0.10,
    )
    assert proj is not None
    assert proj.ambiguous


def test_normal_forward_progress_advances():
    model = build_path_model(
        points=[(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)],
        flags=[True, True, True],
    )
    state = SprayProjectionState(valid=True, segment_index=0, s=0.5)
    proj = project_onto_path(
        model,
        2.5,
        0.0,
        previous_segment_index=state.segment_index,
        previous_s=state.s,
        max_projection_jump_m=2.5,
    )
    assert proj is not None
    assert not proj.ambiguous
    assert proj.s > state.s


def test_duplicate_points_deterministic():
    model = build_path_model(
        points=[(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)],
        flags=[False, True, True],
    )
    a = project_onto_path(model, 0.0, 0.1)
    b = project_onto_path(model, 0.0, 0.1)
    assert a is not None and b is not None
    assert a.segment_index == b.segment_index
    assert abs(a.s - b.s) < 1e-9


# ── Velocity ─────────────────────────────────────────────────────────────────


def test_reverse_motion_blocks_spray():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    decision = make_spray_decision(
        model,
        1.5,
        0.0,
        vel_n=-0.2,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.1,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
    )
    assert not decision.desired
    assert decision.event == "REVERSE_MOTION"


def test_sideways_motion_blocks_spray():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    decision = make_spray_decision(
        model,
        1.5,
        0.0,
        vel_n=0.0,
        vel_e=0.2,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.1,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        max_cross_track_speed_mps=0.10,
    )
    assert not decision.desired
    assert decision.event == "SIDEWAYS_MOTION"


def test_heading_disagreement_blocks_spray():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    decision = make_spray_decision(
        model,
        1.5,
        0.0,
        vel_n=0.12,
        vel_e=0.08,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.1,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        max_cross_track_speed_mps=0.10,
        max_along_track_heading_error_deg=20.0,
    )
    assert not decision.desired
    assert decision.event == "HEADING_DISAGREEMENT"


def test_high_total_speed_low_along_track_blocks():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    decision = make_spray_decision(
        model,
        1.5,
        0.0,
        vel_n=0.01,
        vel_e=0.5,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.1,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        min_spray_speed_mps=0.05,
        low_speed_anti_puddle_behavior="block",
    )
    assert not decision.desired


def test_point_dwell_stationary_exception():
    decision = point_mode_decision(
        dwell=type(
            "D",
            (),
            {
                "active": True,
                "expiry_mono_ns": 10_000_000_000,
                "command_id": 1,
                "mission_id": "m",
                "point_index": 0,
                "start_mono_ns": 0,
                "cancelled": False,
            },
        )(),
        now_mono_ns=1_000_000_000,
        safety_ok=True,
        safety_reason="",
    )
    assert decision.desired is True


# ── Lead safety ──────────────────────────────────────────────────────────────


def test_valid_anticipatory_on_is_not_blocked():
    model = build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (4.0, 0.0)],
        flags=[False, True, False],
    )
    decision = make_spray_decision(
        model,
        0.88,
        0.0,
        vel_n=1.0,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.10,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        min_on_distance_m=0.05,
        min_off_distance_m=0.05,
        max_lead_distance_m=0.50,
    )
    assert decision.desired is True
    assert decision.event == "ON_EARLY"
    assert decision.safety_ok is True
    assert decision.lead_block_reason == ""


def test_on_lead_longer_than_full_pre_run_is_blocked():
    model = build_path_model(
        points=[(0.0, 0.0), (0.08, 0.0), (2.0, 0.0)],
        flags=[False, True, False],
    )
    decision = make_spray_decision(
        model,
        0.04,
        0.0,
        vel_n=1.0,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.10,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        min_on_distance_m=0.05,
        max_lead_distance_m=0.50,
    )
    assert not decision.desired
    assert decision.event == "LEAD_GEOMETRY_UNSAFE"


def test_short_off_gap_is_rejected():
    model = build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (1.02, 0.0), (1.04, 0.0), (3.0, 0.0)],
        flags=[True, True, False, True, True],
    )
    decision = make_spray_decision(
        model,
        0.98,
        0.0,
        vel_n=0.35,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.10,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        min_off_distance_m=0.05,
        max_lead_distance_m=0.50,
    )
    assert not decision.desired
    assert decision.event == "LEAD_GEOMETRY_UNSAFE"
    assert "OFF gap" in decision.safety_reason


def test_valid_off_early_with_sufficient_gap():
    model = build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (1.02, 0.0), (1.20, 0.0), (3.0, 0.0)],
        flags=[True, True, False, True, True],
    )
    decision = make_spray_decision(
        model,
        0.98,
        0.0,
        vel_n=1.0,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.10,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        min_off_distance_m=0.05,
        max_lead_distance_m=0.50,
    )
    assert decision.event == "OFF_EARLY"
    assert decision.desired is False
    assert decision.safety_ok is True
    assert decision.lead_block_reason == ""


def test_short_mark_blocks_early_on():
    model = build_path_model(
        points=[(0.0, 0.0), (0.5, 0.0), (0.52, 0.0), (0.54, 0.0), (2.0, 0.0)],
        flags=[False, True, False, True, False],
    )
    decision = make_spray_decision(
        model,
        0.45,
        0.0,
        vel_n=1.0,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.10,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        min_on_distance_m=0.05,
        max_lead_distance_m=0.50,
    )
    assert not decision.desired
    assert decision.event == "LEAD_GEOMETRY_UNSAFE"


def test_max_lead_clamps_and_reports():
    model = build_path_model([(0.0, 0.0), (5.0, 0.0)], [True, True])
    decision = make_spray_decision(
        model,
        1.0,
        0.0,
        vel_n=2.0,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.50,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        max_lead_distance_m=0.20,
    )
    assert decision.lead_clamped is True
    assert decision.bounded_on_lead_m <= 0.20 + 1e-9


# ── Hysteresis ───────────────────────────────────────────────────────────────


def test_on_transition_held_for_minimum_distance():
    held = apply_distance_hysteresis(
        False,
        1.02,
        last_transition_s=1.0,
        last_geometry_state=True,
        min_on_distance_m=0.05,
        min_off_distance_m=0.05,
        safety_ok=True,
        progress_forward=True,
    )
    assert held is True


def test_off_gap_held_for_minimum_distance():
    held = apply_distance_hysteresis(
        True,
        1.02,
        last_transition_s=1.0,
        last_geometry_state=False,
        min_on_distance_m=0.05,
        min_off_distance_m=0.05,
        safety_ok=True,
        progress_forward=True,
    )
    assert held is False


def test_safety_failure_overrides_hysteresis():
    held = apply_distance_hysteresis(
        True,
        1.02,
        last_transition_s=1.0,
        last_geometry_state=True,
        min_on_distance_m=0.05,
        min_off_distance_m=0.05,
        safety_ok=False,
        progress_forward=True,
    )
    assert held is False


def test_backward_motion_does_not_satisfy_latch():
    held = apply_distance_hysteresis(
        False,
        0.9,
        last_transition_s=1.0,
        last_geometry_state=True,
        min_on_distance_m=0.05,
        min_off_distance_m=0.05,
        safety_ok=True,
        progress_forward=False,
    )
    assert held is True


# ── Flow ─────────────────────────────────────────────────────────────────────


def test_target_flow_uses_along_track_speed():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    decision = make_spray_decision(
        model,
        1.5,
        0.0,
        vel_n=0.35,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.1,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        target_paint_density=2.0,
        dwell_active=False,
    )
    assert abs(decision.raw_target_flow - 0.7) < 1e-9


def test_mapped_pwm_changes_with_along_track_speed():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    cfg = _config(min_spray_speed_mps=0.05)
    slow = continuous_distance_decision(
        model=model,
        pose_ned=(1.5, 0.0, 0.0),
        vel_ned=(0.10, 0.0),
        safety_ok=True,
        safety_reason="",
        config=cfg,
        runtime_state=_runtime_state(),
    )
    fast = continuous_distance_decision(
        model=model,
        pose_ned=(1.5, 0.0, 0.0),
        vel_ned=(0.35, 0.0),
        safety_ok=True,
        safety_reason="",
        config=cfg,
        runtime_state=_runtime_state(),
    )
    assert slow.desired and fast.desired
    assert fast.target_pwm > slow.target_pwm


def test_sideways_speed_does_not_increase_flow():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    along = make_spray_decision(
        model,
        1.5,
        0.0,
        vel_n=0.2,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.1,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        max_cross_track_speed_mps=1.0,
        target_paint_density=1.0,
    )
    sideways = make_spray_decision(
        model,
        1.5,
        0.0,
        vel_n=0.0,
        vel_e=0.2,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.1,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        max_cross_track_speed_mps=1.0,
        target_paint_density=1.0,
    )
    assert along.raw_target_flow > sideways.raw_target_flow


def test_ramp_rate_limits_pwm_change():
    command, limited = ramp_pwm(
        1800.0,
        1200.0,
        max_pwm_change_per_s=100.0,
        dt_s=1.0,
    )
    assert limited is True
    assert abs(command - 1300.0) < 1e-9


def test_low_speed_anti_puddle_blocks():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    decision = make_spray_decision(
        model,
        1.5,
        0.0,
        vel_n=0.01,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.1,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        min_spray_speed_mps=0.05,
        low_speed_anti_puddle_behavior="block",
    )
    assert not decision.desired
    assert decision.event == "LOW_SPEED_ANTI_PUDDLE"


def test_flow_status_reports_clamp_reason():
    model = build_path_model([(0.0, 0.0), (3.0, 0.0)], [True, True])
    decision = make_spray_decision(
        model,
        1.5,
        0.0,
        vel_n=2.0,
        vel_e=0.0,
        safety_ok=True,
        safety_reason="",
        solenoid_open_delay_s=0.1,
        solenoid_close_delay_s=0.05,
        on_overspray_margin_m=0.02,
        off_overspray_margin_m=0.0,
        max_xtrack_error_m=0.10,
        target_paint_density=1.0,
        max_target_flow=0.5,
        high_speed_underflow_behavior="clamp",
        speed_pwm_table_max_speed=0.35,
    )
    assert decision.flow_clamp_reason == "above_max"


# ── Legacy ───────────────────────────────────────────────────────────────────


def test_production_default_rejects_legacy_fallback():
    node = make_node()
    node._params["allow_legacy_spray_active_fallback"] = _Param(False)
    node._params["use_distance_aware_spray"] = _Param(False)
    node._params["diagnostic_profile"] = _Param(False)
    node._params["diagnostic_lease_active"] = _Param(False)
    assert not node._legacy_fallback_allowed()


def test_diagnostic_profile_may_enable_legacy():
    node = make_node()
    node._params["use_distance_aware_spray"] = _Param(False)
    node._params["allow_legacy_spray_active_fallback"] = _Param(True)
    node._params["diagnostic_profile"] = _Param(True)
    node._params["diagnostic_lease_active"] = _Param(True)
    assert node._legacy_fallback_allowed()


def test_runtime_status_exposes_legacy_mode():
    node = make_node()
    node._params["use_distance_aware_spray"] = _Param(True)
    status = node.get_runtime_status()
    assert "legacy_fallback_active" in status
    assert status["distance_aware_spray_enabled"] is True


# ── Identity and telemetry ───────────────────────────────────────────────────


def test_dash_runtime_transform_does_not_break_base_identity():
    from test_spray_controller_v2 import _path_msg

    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    flags = [False, True, True, False]
    expected_base_fingerprint = path_geometry_fingerprint(points, flags)
    node = make_node()
    node._params["spray_mode"] = _Param("dash")
    node._active_config = validate_spray_configuration(
        {
            "spray_mode": "dash",
            "mission_id": "dash_mission",
            "path_fingerprint": expected_base_fingerprint,
            "configuration_revision": 1,
            "dash_on_distance_m": 0.30,
            "dash_off_distance_m": 0.30,
        }
    )
    node._conditioned_path_identity = {
        "mission_id": "dash_mission",
        "path_fingerprint": expected_base_fingerprint,
        "configuration_revision": 1,
        "source": "rpp_conditioned_path",
    }
    node._path_cb(_path_msg(points, flags))
    assert node._path_model is not None
    assert node._geometry_hash == expected_base_fingerprint
    assert node._runtime_spray_geometry_hash != node._geometry_hash
    status = node.get_runtime_status()
    assert status["geometry_hash"] == expected_base_fingerprint
    assert status["runtime_spray_geometry_hash"] == node._runtime_spray_geometry_hash


def test_base_geometry_mismatch_still_fails_closed():
    from test_spray_controller_v2 import _path_msg

    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    flags = [False, True, True, False]
    node = make_node()
    node._active_config = validate_spray_configuration(
        {
            "mission_id": "dash_mission",
            "path_fingerprint": "wrong_fingerprint",
            "configuration_revision": 1,
        }
    )
    node._conditioned_path_identity = {
        "mission_id": "dash_mission",
        "path_fingerprint": "wrong_fingerprint",
        "configuration_revision": 1,
        "source": "rpp_conditioned_path",
    }
    node._path_cb(_path_msg(points, flags))
    assert node._path_model is None
    assert "geometry hash mismatch" in node._last_safety_block_reason


def test_geometry_counts_and_hash_exposed():
    node = make_node()
    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    flags = [False, True, False]
    model = build_path_model(points, flags)
    node._path_model = model
    node._geometry_hash = path_geometry_fingerprint(points, flags)
    node._waypoint_count = len(points)
    node._spray_flag_count = len(flags)
    node._mark_waypoint_count = sum(1 for f in flags if f)
    node._boundary_count = len(model.boundaries)
    status = node.get_runtime_status()
    assert status["geometry_hash"] == node._geometry_hash
    assert status["waypoint_count"] == 3
    assert status["mark_waypoint_count"] == 1


def test_production_rejects_legacy_in_staged_config():
    try:
        validate_spray_configuration(
            {
                "mission_id": "m1",
                "path_fingerprint": "abc",
                "configuration_revision": 1,
                "allow_legacy_spray_active_fallback": True,
            }
        )
        assert False, "expected ValueError"
    except ValueError as exc:
        assert "legacy" in str(exc).lower()


def test_dash_1m_feasible_not_falsely_rejected():
    from spray_dash import extract_mark_region_runs, validate_dash_feasibility

    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    flags = [False, True, True, False]
    base = build_path_model(points, flags)
    config = _config(
        spray_mode="dash",
        dash_on_distance_m=0.3,
        dash_off_distance_m=0.3,
    )
    from spray_dash import apply_dash_pattern
    from spray_config import DashPhaseReset

    dashed = apply_dash_pattern(base, 0.3, 0.3, DashPhaseReset.PER_MARK_REGION)
    runs = extract_mark_region_runs(dashed, base)
    assert len(runs) == 4
    for (length, is_on), exp_len, exp_on in zip(
        runs,
        [0.3, 0.3, 0.3, 0.1],
        [True, False, True, False],
    ):
        assert abs(length - exp_len) < 0.02
        assert is_on is exp_on
    result = validate_dash_feasibility(base, config)
    assert result.dash_feasible is True
    assert "erase dash run 0.100" not in result.dash_feasibility_reason


def test_dash_terminal_off_gap_seen_by_hysteresis():
    from spray_config import DashPhaseReset
    from spray_dash import apply_dash_pattern

    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    flags = [False, True, True, False]
    dashed = apply_dash_pattern(
        build_path_model(points, flags),
        0.3,
        0.3,
        DashPhaseReset.PER_MARK_REGION,
    )
    proj = project_onto_path(dashed, 1.95, 0.0)
    assert proj is not None
    assert proj.current_flag is False
    hysteresis = apply_distance_hysteresis(
        False,
        proj.s,
        last_transition_s=1.6,
        last_geometry_state=True,
        min_on_distance_m=0.05,
        min_off_distance_m=0.05,
        safety_ok=True,
        progress_forward=True,
    )
    assert hysteresis is False


def test_dash_lead_uses_corrected_terminal_boundary():
    from spray_config import DashPhaseReset
    from spray_dash import apply_dash_pattern

    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    flags = [False, True, True, False]
    dashed = apply_dash_pattern(
        build_path_model(points, flags),
        0.3,
        0.3,
        DashPhaseReset.PER_MARK_REGION,
    )
    config = _config(
        spray_mode="dash",
        dash_on_distance_m=0.3,
        dash_off_distance_m=0.3,
    )
    runtime = _runtime_state()
    runtime.projection = SprayProjectionState(valid=True, segment_index=2, s=1.85)
    decision = continuous_distance_decision(
        model=dashed,
        pose_ned=(1.85, 0.0, 0.0),
        vel_ned=(0.35, 0.0),
        safety_ok=True,
        safety_reason="",
        config=config,
        runtime_state=runtime,
    )
    assert decision.safety_ok is True
    assert decision.lead_block_reason == ""


def test_runtime_fields_serialize_safely():
    status = {
        "timestamp_monotonic_s": 1.0,
        "spray_mode": "continuous",
        "configuration_revision": 1,
        "model_revision": 1,
        "ready": True,
        "commanded_on": False,
        "confirmed_off": True,
        "active_dwell": False,
        "dwell_command_id": None,
        "dwell_mission_id": None,
        "dwell_point_index": None,
        "dwell_remaining_s": 0.0,
        "last_error": "",
        "projected_arc_length_m": float("nan"),
        "along_track_speed_mps": float("inf"),
        "distance_to_next_boundary_m": float("inf"),
    }
    payload = serialize_runtime_status(status)
    parsed = json.loads(payload)
    assert parsed["projected_arc_length_m"] is None
    assert parsed["along_track_speed_mps"] is None