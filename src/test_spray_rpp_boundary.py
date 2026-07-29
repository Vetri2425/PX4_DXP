#!/usr/bin/env python3
"""G2 — spray boundary sourcing from /rpp/progress (design §5, tasks G2).

Two layers, both pure (the rclpy import is stubbed by test_spray_manual_override):

  * `_rpp_kind_for` + `_make_spray_decision` — the boundary-source swap itself.
    With RPP inputs supplied the lead math anticipates RPP's authoritative
    boundary; with them absent it is byte-for-byte the /path projection.
  * `_rpp_boundary_inputs` on the node — the gate + staleness fallback: RPP only
    when consume_rpp_progress is set, mode is continuous, and progress is fresh.

Run:  python3 -m pytest -q test_spray_rpp_boundary.py
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from test_spray_manual_override import _Param, make_node  # noqa: E402
from spray_controller_node import (  # noqa: E402
    MARK_TO_TRANSIT,
    TRANSIT_TO_MARK,
    _build_path_model,
    _make_spray_decision,
    _rpp_kind_for,
)
from mission_progress import MissionPhase, ProgressMsg  # noqa: E402


def _straight_mark_path():
    # points 0..3 at n=0,1,2,3 ; flags off,on,on,off → MARK region s∈[1,2]
    return _build_path_model(
        points=[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        flags=[False, True, True, False],
    )


def _decision(**kwargs):
    defaults = {
        "model": _straight_mark_path(),
        "nozzle_n": 0.5,
        "nozzle_e": 0.0,
        "speed_mps": 1.0,
        "safety_ok": True,
        "safety_reason": "",
        "solenoid_open_delay_s": 0.10,
        "solenoid_close_delay_s": 0.05,
        "on_overspray_margin_m": 0.02,
        "off_overspray_margin_m": 0.0,
        "max_xtrack_error_m": 0.03,
    }
    defaults.update(kwargs)
    return _make_spray_decision(**defaults)


# ---------------------------------------------------------------------------
# _rpp_kind_for — progress boundary → spray kind
# ---------------------------------------------------------------------------
def test_kind_mapping():
    assert _rpp_kind_for(False, "MARK_START") == TRANSIT_TO_MARK
    assert _rpp_kind_for(True, "MARK_END") == MARK_TO_TRANSIT
    # Path ends inside a mark → terminal shutoff even without an explicit end.
    assert _rpp_kind_for(True, "REACHED_END") == MARK_TO_TRANSIT
    # Reaching the end while NOT in a mark: nothing to lead onto.
    assert _rpp_kind_for(False, "REACHED_END") == ""
    assert _rpp_kind_for(False, "") == ""


# ---------------------------------------------------------------------------
# RPP-sourced lead equals local-projection lead on a clean stream (G2.4)
# ---------------------------------------------------------------------------
def test_rpp_and_local_agree_on_on_early():
    # Nozzle at n=0.85, 0.15 m before the MARK start at s=1.0. on_lead =
    # 1.0*0.10 + 0.02 = 0.12 → 0.15 > 0.12, so NOT yet on for either source.
    local = _decision(nozzle_n=0.85)
    rpp = _decision(nozzle_n=0.85, rpp_in_mark=False,
                    rpp_boundary_kind=TRANSIT_TO_MARK, rpp_dist_to_boundary_m=0.15)
    assert local.geometry_desired is False
    assert rpp.geometry_desired is False
    assert abs(rpp.distance_to_boundary_m - 0.15) < 1e-9

    # Nozzle at n=0.90 → 0.10 m out ≤ 0.12 lead → on_early for BOTH sources.
    local2 = _decision(nozzle_n=0.90)
    rpp2 = _decision(nozzle_n=0.90, rpp_in_mark=False,
                     rpp_boundary_kind=TRANSIT_TO_MARK, rpp_dist_to_boundary_m=0.10)
    assert local2.geometry_desired is True and local2.event == "on_early"
    assert rpp2.geometry_desired is True and rpp2.event == "on_early"


def test_rpp_off_early_at_mark_end():
    # Inside the mark, 0.04 m before the end. off_lead = max(0, 1.0*0.05 - 0) =
    # 0.05 → 0.04 ≤ 0.05 → off_early.
    rpp = _decision(nozzle_n=1.96, rpp_in_mark=True,
                    rpp_boundary_kind=MARK_TO_TRANSIT, rpp_dist_to_boundary_m=0.04)
    assert rpp.geometry_desired is False and rpp.event == "off_early"


def test_rpp_in_mark_sprays_when_far_from_boundary():
    # Deep inside the mark, boundary 0.5 m away (> off_lead) → stays ON.
    rpp = _decision(nozzle_n=1.5, rpp_in_mark=True,
                    rpp_boundary_kind=MARK_TO_TRANSIT, rpp_dist_to_boundary_m=0.5)
    assert rpp.geometry_desired is True and rpp.event == ""


def test_rpp_overrides_disagreeing_local_projection():
    # The nozzle is physically at n=0.5 (local projection says TRANSIT, far from
    # the mark), but RPP asserts we are already inside the mark. RPP wins — this
    # is the dual-projection drift the sourcing exists to kill.
    d = _decision(nozzle_n=0.5, rpp_in_mark=True,
                  rpp_boundary_kind=MARK_TO_TRANSIT, rpp_dist_to_boundary_m=0.9)
    assert d.geometry_desired is True


def test_rpp_xtrack_gate_still_local_and_independent():
    # RPP says spray, but the nozzle is 0.25 m off the line (> 0.03 gate). The
    # xtrack safety is computed from the LOCAL projection and must still block.
    d = _decision(nozzle_n=1.5, nozzle_e=0.25, rpp_in_mark=True,
                  rpp_boundary_kind=MARK_TO_TRANSIT, rpp_dist_to_boundary_m=0.5)
    assert d.geometry_desired is True   # geometry still wants it
    assert d.safety_ok is False         # ...but the independent gate refuses
    assert d.desired is False


# ---------------------------------------------------------------------------
# Node-level source selection + staleness fallback (G2.1/G2.2)
# ---------------------------------------------------------------------------
def _progress(phase, next_boundary, dist):
    msg = ProgressMsg(phase=phase, next_boundary=next_boundary,
                      dist_to_next_boundary_m=dist)
    return msg


def test_inputs_off_when_flag_disabled():
    node = make_node()
    node._params["consume_rpp_progress"] = _Param(False)
    node._rpp_progress = _progress(MissionPhase.MARK_TRACKING, "MARK_END", 0.4)
    node._rpp_progress_recv_time = node.get_clock().now()
    assert node._rpp_boundary_inputs("continuous") == (None, "", float("inf"))


def test_inputs_off_for_non_continuous_mode():
    node = make_node()
    node._params["consume_rpp_progress"] = _Param(True)
    node._rpp_progress = _progress(MissionPhase.MARK_TRACKING, "MARK_END", 0.4)
    node._rpp_progress_recv_time = node.get_clock().now()
    # Dash meters arc-length locally; point uses the handshake — neither is
    # boundary-sourced in G2.
    assert node._rpp_boundary_inputs("dash") == (None, "", float("inf"))
    assert node._rpp_boundary_inputs("point") == (None, "", float("inf"))


def test_inputs_used_when_fresh_and_enabled():
    node = make_node()
    node._params["consume_rpp_progress"] = _Param(True)
    node._rpp_progress = _progress(MissionPhase.MARK_TRACKING, "MARK_END", 0.42)
    node._rpp_progress_recv_time = node.get_clock().now()
    in_mark, kind, dist = node._rpp_boundary_inputs("continuous")
    assert in_mark is True
    assert kind == MARK_TO_TRANSIT
    assert abs(dist - 0.42) < 1e-9


def test_inputs_fall_back_to_path_when_stale():
    node = make_node()
    node._params["consume_rpp_progress"] = _Param(True)
    node._params["progress_timeout_s"] = _Param(0.3)
    node._rpp_progress = _progress(MissionPhase.MARK_TRACKING, "MARK_END", 0.42)
    node._rpp_progress_recv_time = node.get_clock().now()
    node._clock.ns += 500_000_000  # +0.5 s → older than progress_timeout_s
    assert node._rpp_boundary_inputs("continuous") == (None, "", float("inf"))


def test_inputs_fall_back_before_first_message():
    node = make_node()
    node._params["consume_rpp_progress"] = _Param(True)
    # No progress ever received → path fallback, never a crash.
    assert node._rpp_boundary_inputs("continuous") == (None, "", float("inf"))


def test_approach_mark_phase_leads_on_but_reports_not_in_mark():
    node = make_node()
    node._params["consume_rpp_progress"] = _Param(True)
    node._rpp_progress = _progress(MissionPhase.APPROACH_MARK, "MARK_START", 0.08)
    node._rpp_progress_recv_time = node.get_clock().now()
    in_mark, kind, dist = node._rpp_boundary_inputs("continuous")
    assert in_mark is False               # APPROACH_MARK is outside the mark
    assert kind == TRANSIT_TO_MARK        # ...but leading onto its start


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
