#!/usr/bin/env python3
"""G1.5 — mission-phase classifier tests (design §3). Pure, no rclpy.

Covers the phase-transition table (continuous, extensions, point), the
distance-to-next-boundary math, and the milestone-edge derivation.

Run:  python3 -m pytest -q test_progress_classifier.py
"""

import progress_classifier as pc
from mission_progress import MilestoneEvent, MissionPhase


# ---------------------------------------------------------------------------
# Boundary geometry
# ---------------------------------------------------------------------------
def test_segment_mark_flags():
    # points:  0    1    2    3
    # spray : off  on   on  off  → seg MARK = [off&on, on&on, on&off] = [F,T,F]
    assert pc.segment_mark_flags([False, True, True, False]) == [False, True, False]


def test_next_boundary_mark_start_then_end():
    # 5 points, cum_s = 0,1,2,3,4 ; spray off,off,on,on,off
    # seg_mark = [F, F, T, F]
    spray = [False, False, True, True, False]
    cum_s = [0.0, 1.0, 2.0, 3.0, 4.0]
    seg_mark = pc.segment_mark_flags(spray)
    assert seg_mark == [False, False, True, False]
    # from seg 0 at along_s 0.5 → next transition is seg 2 (MARK) at cum_s 2.0
    dist, name = pc.next_mark_boundary(seg_mark, cum_s, 0, 0.5)
    assert name == "MARK_START" and abs(dist - 1.5) < 1e-9
    # from inside the mark (seg 2) at along_s 2.4 → next is seg 3 (leaving) at 3.0
    dist, name = pc.next_mark_boundary(seg_mark, cum_s, 2, 2.4)
    assert name == "MARK_END" and abs(dist - 0.6) < 1e-9


def test_next_boundary_none_reaches_end():
    seg_mark = [True, True, True]
    cum_s = [0.0, 1.0, 2.0, 3.0]
    dist, name = pc.next_mark_boundary(seg_mark, cum_s, 0, 0.0)
    assert name == MilestoneEvent.REACHED_END and abs(dist - 3.0) < 1e-9


# ---------------------------------------------------------------------------
# Continuous / extension phase classification
# ---------------------------------------------------------------------------
def _cont(seg_idx, along_s, spray, cum_s, approach=0.30, **kw):
    return pc.classify(
        has_path=True, path_done=False, spray_flags=spray, cum_s=cum_s,
        seg_idx=seg_idx, proj_t=0.0, along_s=along_s, signed_xtrack=0.0,
        speed=0.35, stopped=False, approach_dist_m=approach, **kw)


def test_phase_mark_tracking_and_end():
    spray = [False, False, True, True, False]     # seg_mark [F,F,T,F]
    cum_s = [0.0, 1.0, 2.0, 3.0, 4.0]
    # deep inside the mark → MARK_TRACKING
    assert _cont(2, 2.1, spray, cum_s).phase == MissionPhase.MARK_TRACKING
    # within approach of the mark end → MARK_END
    assert _cont(2, 2.85, spray, cum_s).phase == MissionPhase.MARK_END


def test_phase_approach_mark_vs_pre_ext():
    spray = [False, False, True, True, False]
    cum_s = [0.0, 1.0, 2.0, 3.0, 4.0]
    # seg 1, close to mark start (dist 0.2 <= 0.30) → APPROACH_MARK
    assert _cont(1, 1.8, spray, cum_s).phase == MissionPhase.APPROACH_MARK
    # seg 0, far from mark start (dist 1.5) but a mark lies ahead → PRE_EXT
    assert _cont(0, 0.5, spray, cum_s).phase == MissionPhase.PRE_EXT


def test_phase_aft_ext_after_mark():
    spray = [False, True, True, False, False]     # seg_mark [F,T,F,F]
    cum_s = [0.0, 1.0, 2.0, 3.0, 4.0]
    # seg 2 is non-mark and the previous seg (1) was MARK → AFT_EXT
    assert _cont(2, 2.5, spray, cum_s).phase == MissionPhase.AFT_EXT


def test_phase_transit_no_mark_anywhere():
    spray = [False, False, False]
    cum_s = [0.0, 1.0, 2.0]
    assert _cont(0, 0.5, spray, cum_s).phase == MissionPhase.TRANSIT


def test_boundary_distance_surfaced():
    spray = [False, False, True, True, False]
    cum_s = [0.0, 1.0, 2.0, 3.0, 4.0]
    m = _cont(0, 0.5, spray, cum_s)
    assert m.next_boundary == "MARK_START"
    assert abs(m.dist_to_next_boundary_m - 1.5) < 1e-9
    assert m.xtrack_m == 0.0 and m.speed_mps == 0.35


# ---------------------------------------------------------------------------
# Idle / done
# ---------------------------------------------------------------------------
def test_no_path_is_idle():
    m = pc.classify(has_path=False, path_done=False, spray_flags=[], cum_s=[],
                    seg_idx=0, proj_t=0.0, along_s=0.0, signed_xtrack=0.0,
                    speed=0.0, stopped=True, approach_dist_m=0.3)
    assert m.phase == MissionPhase.IDLE


def test_done_is_reached_end():
    m = pc.classify(has_path=True, path_done=True, spray_flags=[True, True],
                    cum_s=[0.0, 1.0], seg_idx=0, proj_t=1.0, along_s=1.0,
                    signed_xtrack=0.0, speed=0.0, stopped=True, approach_dist_m=0.3)
    assert m.phase == MissionPhase.REACHED_END


# ---------------------------------------------------------------------------
# Point mode
# ---------------------------------------------------------------------------
def _point(**kw):
    base = dict(
        has_path=True, path_done=False, spray_flags=[True, True, True],
        cum_s=[0.0, 1.0, 2.0], seg_idx=0, proj_t=0.0, along_s=0.0,
        signed_xtrack=0.0, speed=0.1, stopped=False, approach_dist_m=0.3,
        point_mode=True,
    )
    base.update(kw)
    return pc.classify(**base)


def test_point_approach():
    m = _point(point_active=False, point_target_rank=1, point_target_vertex=2,
               along_s=0.4)
    assert m.phase == MissionPhase.APPROACH_POINT
    assert m.point_index == 1
    assert abs(m.dist_to_next_boundary_m - 1.6) < 1e-9   # cum_s[2]=2.0 - 0.4


def test_point_braking_then_dwell():
    braking = _point(point_active=True, point_dwelling=False, point_target_rank=0)
    assert braking.phase == MissionPhase.APPROACH_POINT
    dwell = _point(point_active=True, point_dwelling=True, point_target_rank=0,
                   stopped=True, speed=0.0)
    assert dwell.phase == MissionPhase.DWELL_HOLD


def test_point_all_done_transit():
    m = _point(all_points_done=True)
    assert m.phase == MissionPhase.TRANSIT and m.point_index == -1


# ---------------------------------------------------------------------------
# Milestone edges
# ---------------------------------------------------------------------------
def test_milestone_mark_start_stop():
    assert pc.milestones_for(MissionPhase.APPROACH_MARK, MissionPhase.MARK_TRACKING,
                             -1, -1) == [MilestoneEvent.MARK_START]
    assert pc.milestones_for(MissionPhase.MARK_END, MissionPhase.AFT_EXT,
                             -1, -1) == [MilestoneEvent.MARK_STOP]


def test_milestone_pre_start_and_aft_stop():
    assert MilestoneEvent.PRE_START in pc.milestones_for(
        MissionPhase.TRANSIT, MissionPhase.PRE_EXT, -1, -1)
    assert MilestoneEvent.AFT_STOP in pc.milestones_for(
        MissionPhase.AFT_EXT, MissionPhase.TRANSIT, -1, -1)


def test_milestone_at_point_and_dwell_done():
    assert pc.milestones_for(MissionPhase.APPROACH_POINT, MissionPhase.DWELL_HOLD,
                             0, 0) == [MilestoneEvent.AT_POINT]
    assert pc.milestones_for(MissionPhase.DWELL_HOLD, MissionPhase.APPROACH_POINT,
                             0, 1) == [MilestoneEvent.DWELL_DONE_RPP]


def test_milestone_reached_end():
    # From TRANSIT → end: just REACHED_END.
    assert pc.milestones_for(MissionPhase.TRANSIT, MissionPhase.REACHED_END,
                             -1, -1) == [MilestoneEvent.REACHED_END]
    # Ending a mission from *inside* a mark correctly emits BOTH: the mark
    # stopped and the mission reached its end.
    assert pc.milestones_for(MissionPhase.MARK_TRACKING, MissionPhase.REACHED_END,
                             -1, -1) == [MilestoneEvent.MARK_STOP,
                                         MilestoneEvent.REACHED_END]


def test_milestone_no_change_no_events():
    assert pc.milestones_for(MissionPhase.MARK_TRACKING, MissionPhase.MARK_TRACKING,
                             -1, -1) == []
