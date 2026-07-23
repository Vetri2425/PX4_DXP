#!/usr/bin/env python3
"""G0.5 — mission-progress contract tests (design §3, §4). Pure, no rclpy —
runs on the Mac.

Pins the wire contract so the RPP node (producer) and the spray node + server
(consumers) can never silently drift: enum numeric values, milestone names,
QoS specs, JSON round-trips, and the lenient-parse / NaN→null robustness that
lets a consumer degrade instead of crash on a malformed tick.

Run:  python3 -m pytest -q test_mission_progress.py
"""

import json

import mission_progress as mp
from mission_progress import (
    AdvanceMsg,
    MilestoneEvent,
    MilestoneMsg,
    MissionPhase,
    PointDoneMsg,
    ProgressMsg,
)


# ---------------------------------------------------------------------------
# Enum + event-name contract (frozen numeric values — consumers pin to these)
# ---------------------------------------------------------------------------
def test_mission_phase_values_frozen():
    # Exact numeric contract from design §3. A renumber here is a breaking
    # change and must be a deliberate, reviewed edit — this test is the gate.
    assert [(p.name, int(p)) for p in MissionPhase] == [
        ("IDLE", 0), ("TRANSIT", 1), ("PRE_EXT", 2), ("APPROACH_MARK", 3),
        ("MARK_TRACKING", 4), ("MARK_END", 5), ("AFT_EXT", 6),
        ("APPROACH_POINT", 7), ("AT_POINT", 8), ("DWELL_HOLD", 9),
        ("WAIT_OPERATOR", 10), ("REACHED_END", 11),
    ]


def test_mission_phase_disjoint_from_segment_state():
    # Sanity: MissionPhase is its own enum, not SegmentStateCode. It must never
    # be imported from the controller module — this module is the single source.
    assert MissionPhase.__module__ == "mission_progress"


def test_milestone_event_names():
    assert MilestoneEvent.ALL == {
        "MARK_START", "MARK_STOP", "PRE_START", "AFT_STOP",
        "AT_POINT", "DWELL_DONE_RPP", "REACHED_END",
    }


def test_topic_names():
    assert mp.TOPIC_PROGRESS == "/rpp/progress"
    assert mp.TOPIC_MILESTONE == "/rpp/milestone"
    assert mp.TOPIC_POINT_DONE == "/spray/point_done"
    assert mp.TOPIC_ADVANCE == "/point/advance"


def test_qos_specs():
    # §4 QoS rationale, pinned. Command/event channels must NOT be
    # TRANSIENT_LOCAL (no stale replay on restart).
    assert mp.PROGRESS_QOS == mp.QoSSpec("BEST_EFFORT", "VOLATILE", 1)
    assert mp.MILESTONE_QOS == mp.QoSSpec("RELIABLE", "VOLATILE", 10)
    assert mp.POINT_DONE_QOS == mp.QoSSpec("RELIABLE", "VOLATILE", 10)
    assert mp.ADVANCE_QOS == mp.QoSSpec("RELIABLE", "VOLATILE", 1)
    for spec in (mp.MILESTONE_QOS, mp.POINT_DONE_QOS, mp.ADVANCE_QOS):
        assert spec.durability != "TRANSIENT_LOCAL"


# ---------------------------------------------------------------------------
# ProgressMsg round-trip + wire shape
# ---------------------------------------------------------------------------
def test_progress_roundtrip():
    m = ProgressMsg(
        phase=MissionPhase.MARK_TRACKING, segment_index=12, point_index=-1,
        dist_to_next_boundary_m=0.34, next_boundary="MARK_END",
        speed_mps=0.35, stopped=False, xtrack_m=0.011,
    )
    back = ProgressMsg.from_json(m.to_json())
    assert back == m


def test_progress_wire_includes_phase_name():
    m = ProgressMsg(phase=MissionPhase.AT_POINT)
    d = json.loads(m.to_json())
    assert d["phase"] == 8 and d["phase_name"] == "AT_POINT"


def test_progress_nan_becomes_null_on_wire():
    # JSON has no NaN — the wire must stay strictly valid JSON. dist/xtrack
    # default to NaN; they must serialize to null and parse back to NaN.
    m = ProgressMsg(phase=MissionPhase.TRANSIT)  # dist + xtrack default NaN
    raw = m.to_json()
    assert "NaN" not in raw and "Infinity" not in raw
    d = json.loads(raw)
    assert d["dist_to_next_boundary_m"] is None
    back = ProgressMsg.from_json(raw)
    assert back.dist_to_next_boundary_m != back.dist_to_next_boundary_m  # NaN


def test_progress_inf_becomes_null():
    m = ProgressMsg(phase=MissionPhase.TRANSIT, dist_to_next_boundary_m=float("inf"))
    assert json.loads(m.to_json())["dist_to_next_boundary_m"] is None


# ---------------------------------------------------------------------------
# Milestone / point_done / advance round-trips
# ---------------------------------------------------------------------------
def test_milestone_roundtrip():
    m = MilestoneMsg(event=MilestoneEvent.AT_POINT, index=2, seq=41, stamp_ns=173)
    assert MilestoneMsg.from_json(m.to_json()) == m


def test_point_done_roundtrip():
    m = PointDoneMsg(point_index=2, seq=17, done=True, reason="dwell_complete")
    assert PointDoneMsg.from_json(m.to_json()) == m


def test_advance_roundtrip():
    m = AdvanceMsg(advance=True, expect_index=2)
    assert AdvanceMsg.from_json(m.to_json()) == m


# ---------------------------------------------------------------------------
# Lenient parse — a consumer must degrade to defaults, never raise
# ---------------------------------------------------------------------------
def test_from_json_tolerates_garbage():
    for bad in ("", "not json", "[]", "null", "{", '{"phase": "banana"}'):
        p = ProgressMsg.from_json(bad)          # must not raise
        assert p.phase == MissionPhase.IDLE
        assert MilestoneMsg.from_json(bad).seq == -1
        assert PointDoneMsg.from_json(bad).point_index == -1
        assert AdvanceMsg.from_json(bad).advance is False


def test_from_json_partial_fields_fill_defaults():
    p = ProgressMsg.from_json('{"phase": 4}')
    assert p.phase == MissionPhase.MARK_TRACKING
    assert p.segment_index == -1 and p.point_index == -1
    assert p.next_boundary == "" and p.stopped is False


def test_out_of_range_phase_degrades_to_idle():
    assert ProgressMsg.from_json('{"phase": 99}').phase == MissionPhase.IDLE
    assert ProgressMsg.from_json('{"phase": -1}').phase == MissionPhase.IDLE
