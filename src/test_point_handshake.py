#!/usr/bin/env python3
"""G4/G5 — point handshake, pure parts (design §5/§6). No rclpy — runs on Mac.

Two pure surfaces of the handshake:
  * **Spray side (PointMeter gate):** `require_arrival_gate` makes RPP the
    arrival authority — the dwell fires on the AT_POINT proof, not the meter's
    own pose guess. `completed_index` reports the dot whose dwell just finished
    (OFF-confirmed) so the node can publish `/spray/point_done` for it.
  * **RPP side (classifier):** point mode reports WAIT_OPERATOR when the RPP is
    holding after dwell for the operator's advance (manual gate), and the
    DWELL_HOLD→WAIT_OPERATOR edge still emits DWELL_DONE_RPP.

The node glue (`rpp_controller_node._point_handshake_ready`,
`spray_controller_node._point_handshake_gate` / `_publish_point_done`) is
exercised in-env on the Jetson (`test_point_handshake_rpp.py`).

Run:  python3 -m pytest -q test_point_handshake.py
"""

import progress_classifier as pc
from mission_progress import MilestoneEvent, MissionPhase
from spray_modes import PointMeter


# ---------------------------------------------------------------------------
# Spray side — PointMeter arrival gate (design §5: proof gates, not self-guess)
# ---------------------------------------------------------------------------
def _tick(m, n, e, t, *, gate=None, off=True, speed=0.0):
    return m.update(n, e, 0.0, speed, t, off, require_arrival_gate=gate)


def test_gate_none_is_frozen_self_arrival():
    """gate=None → identical to the pre-G4 pose-based arrival (byte-for-byte)."""
    m = PointMeter([(0.0, 0.0)], 0.10, arrival_settle_s=0.0, dwell_s=0.2)
    # Sitting on the dot, slow → arrives from pose alone, no gate needed.
    u = _tick(m, 0.0, 0.0, 1.0)
    assert u.phase in ("holding", "dwelling")
    assert u.geometry_desired is True


def test_gate_false_holds_off_even_when_pose_on_point():
    """RPP has NOT confirmed AT_POINT → no spray, even parked on the dot."""
    m = PointMeter([(0.0, 0.0)], 0.10, arrival_settle_s=0.0, dwell_s=0.2)
    u = _tick(m, 0.0, 0.0, 1.0, gate=False)
    assert u.phase == "transit"
    assert u.geometry_desired is False


def test_gate_true_fires_even_when_pose_off_point():
    """RPP proof overrides the pose guess (nozzle-offset tolerant, design §5)."""
    m = PointMeter([(0.0, 0.0)], 0.10, arrival_settle_s=0.0, dwell_s=0.2)
    # 0.5 m from the dot — the pose check would FAIL — but AT_POINT gates True.
    u = _tick(m, 0.5, 0.0, 1.0, gate=True)
    assert u.phase == "dwelling"
    assert u.geometry_desired is True


def test_completed_index_fires_once_on_off_confirmed_advance():
    m = PointMeter([(0.0, 0.0), (5.0, 0.0)], 0.10, arrival_settle_s=0.0, dwell_s=0.2)
    assert _tick(m, 0.0, 0.0, 1.0, gate=True).phase == "dwelling"
    # Mid-dwell: not done yet, nothing completed.
    assert _tick(m, 0.0, 0.0, 1.1, gate=True).completed_index == -1
    # Dwell over + OFF-confirmed → advance, completed_index reports the dot.
    u = _tick(m, 0.0, 0.0, 1.4, gate=True, off=True)
    assert u.completed_index == 0
    assert u.target_index == 1
    # The very next tick does not re-report it.
    assert _tick(m, 0.0, 0.0, 1.5, gate=False).completed_index == -1


def test_completed_index_waits_for_off_confirm():
    """Dwell time elapsed but actuator not OFF yet → no completion (safety)."""
    m = PointMeter([(0.0, 0.0)], 0.10, arrival_settle_s=0.0, dwell_s=0.2)
    _tick(m, 0.0, 0.0, 1.0, gate=True)
    u = _tick(m, 0.0, 0.0, 1.4, gate=True, off=False)   # dwell done, OFF not acked
    assert u.completed_index == -1
    assert u.phase == "off_wait"
    assert u.geometry_desired is False                   # desired-off while waiting
    # Now OFF is confirmed → completes.
    assert _tick(m, 0.0, 0.0, 1.5, gate=True, off=True).completed_index == 0


def test_skip_is_not_a_completion():
    """A watchdog skip advances but must never count as a sprayed completion."""
    m = PointMeter([(0.0, 0.0)], 0.10, arrival_settle_s=0.0, dwell_s=0.2,
                   point_arrival_timeout_s=1.0)
    _tick(m, 9.0, 0.0, 0.0, gate=False)          # far, transit_start=0
    u = _tick(m, 9.0, 0.0, 2.0, gate=False)      # 2 s > 1 s timeout → skip
    assert u.skipped_index == 0
    assert u.completed_index == -1


# ---------------------------------------------------------------------------
# RPP side — classifier WAIT_OPERATOR phase + milestone edge
# ---------------------------------------------------------------------------
def _point_classify(**kw):
    base = dict(
        has_path=True, path_done=False, spray_flags=[False, False],
        cum_s=[0.0, 5.0], seg_idx=0, proj_t=0.0, along_s=0.0,
        signed_xtrack=0.0, speed=0.0, stopped=True, approach_dist_m=0.30,
        point_mode=True, point_active=True, point_target_rank=0,
        point_target_vertex=1,
    )
    base.update(kw)
    return pc.classify(**base)


def test_classifier_wait_operator_phase():
    msg = _point_classify(point_dwelling=True, point_wait_operator=True)
    assert msg.phase == MissionPhase.WAIT_OPERATOR
    assert msg.point_index == 0


def test_classifier_dwell_without_wait_is_dwell_hold():
    msg = _point_classify(point_dwelling=True, point_wait_operator=False)
    assert msg.phase == MissionPhase.DWELL_HOLD


def test_dwell_to_wait_operator_edge_emits_dwell_done():
    ev = pc.milestones_for(MissionPhase.DWELL_HOLD, MissionPhase.WAIT_OPERATOR, 0, 0)
    assert MilestoneEvent.DWELL_DONE_RPP in ev


def test_wait_operator_to_transit_edge_is_quiet():
    ev = pc.milestones_for(MissionPhase.WAIT_OPERATOR, MissionPhase.TRANSIT, 0, 1)
    assert ev == []


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-q"]))
