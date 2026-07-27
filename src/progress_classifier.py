#!/usr/bin/env python3
"""Mission-phase classifier (design §3, phase G1). Pure, no rclpy — runs on
the Mac.

Given the geometry the RPP node already has (path vertices' spray flags,
must-hit indices, cumulative arc length) plus where the rover is on the path
this tick (segment index, projection fraction, along-path arc length, speed,
stopped), decide the `MissionPhase` and the along-path distance to the next
phase boundary. The node (`rpp_controller_node.py`) is thin glue: it gathers
these primitives from its own state and calls `classify()`; all the phase
logic lives here so it is unit-testable off-robot (`test_progress_classifier`).

Two boundary regimes (design §3 per-mode phase paths):
  * **Continuous / dash** — the boundary is the next MARK↔non-MARK spray-flag
    transition (`MARK_START` / `MARK_END`), i.e. the edge of a painted region.
  * **Point** — the boundary is the next must-hit point; phases are the
    APPROACH_POINT → AT_POINT/DWELL_HOLD stops.

Nothing here mutates state or emits milestones; `milestones_for()` derives the
discrete events from a (prev_phase → new_phase) edge, and the node owns the
monotonic `seq`.
"""

from __future__ import annotations

from mission_progress import MilestoneEvent, MissionPhase, ProgressMsg


def segment_mark_flags(spray_flags: list) -> list:
    """Per-segment MARK status: segment i is MARK iff both its endpoints spray.

    Length n_pts-1. A segment paints only when both endpoints are spray-ON,
    matching `_segment_spray_active` in the controller.
    """
    return [bool(spray_flags[i] and spray_flags[i + 1])
            for i in range(len(spray_flags) - 1)]


def next_mark_boundary(seg_mark: list, cum_s: list, seg_idx: int,
                       along_s: float) -> tuple:
    """Along-path distance + name of the next MARK↔non-MARK transition.

    Scans forward from the current segment for the first segment whose MARK
    status differs. Returns (dist_m, name) where name is "MARK_START" (entering
    a painted region), "MARK_END" (leaving one), or "REACHED_END" if no further
    transition exists before the path ends.
    """
    n_seg = len(seg_mark)
    if n_seg == 0:
        return 0.0, MilestoneEvent.REACHED_END
    seg_idx = max(0, min(seg_idx, n_seg - 1))
    cur = seg_mark[seg_idx]
    for j in range(seg_idx + 1, n_seg):
        if seg_mark[j] != cur:
            dist = max(0.0, cum_s[j] - along_s)
            return dist, ("MARK_START" if seg_mark[j] else "MARK_END")
    return max(0.0, cum_s[-1] - along_s), MilestoneEvent.REACHED_END


def _mark_ahead(seg_mark: list, seg_idx: int) -> bool:
    return any(seg_mark[j] for j in range(seg_idx + 1, len(seg_mark)))


def classify(
    *,
    has_path: bool,
    path_done: bool,
    spray_flags: list,
    cum_s: list,
    seg_idx: int,
    proj_t: float,
    along_s: float,
    signed_xtrack: float,
    speed: float,
    stopped: bool,
    approach_dist_m: float,
    # point-mode inputs (all ignored unless point_mode is True)
    point_mode: bool = False,
    point_active: bool = False,
    point_dwelling: bool = False,
    point_wait_operator: bool = False,
    point_target_rank: int = -1,
    point_target_vertex: int = -1,
    all_points_done: bool = False,
) -> ProgressMsg:
    """Classify this tick into a ProgressMsg (design §3, §4.1)."""
    if not has_path:
        return ProgressMsg(phase=MissionPhase.IDLE)
    if path_done:
        return ProgressMsg(phase=MissionPhase.REACHED_END, segment_index=seg_idx,
                           speed_mps=speed, stopped=stopped, xtrack_m=signed_xtrack,
                           dist_to_next_boundary_m=0.0,
                           next_boundary=MilestoneEvent.REACHED_END)

    if point_mode:
        return _classify_point(
            seg_idx=seg_idx, along_s=along_s, cum_s=cum_s, speed=speed,
            stopped=stopped, signed_xtrack=signed_xtrack,
            point_active=point_active, point_dwelling=point_dwelling,
            point_wait_operator=point_wait_operator,
            point_target_rank=point_target_rank,
            point_target_vertex=point_target_vertex,
            all_points_done=all_points_done,
        )

    seg_mark = segment_mark_flags(spray_flags)
    if not seg_mark:
        return ProgressMsg(phase=MissionPhase.TRANSIT, segment_index=seg_idx,
                           speed_mps=speed, stopped=stopped, xtrack_m=signed_xtrack)
    seg_idx = max(0, min(seg_idx, len(seg_mark) - 1))
    dist, boundary = next_mark_boundary(seg_mark, cum_s, seg_idx, along_s)
    cur_mark = seg_mark[seg_idx]

    if cur_mark:
        phase = (MissionPhase.MARK_END
                 if boundary == "MARK_END" and dist <= approach_dist_m
                 else MissionPhase.MARK_TRACKING)
    else:
        prev_mark = seg_idx > 0 and seg_mark[seg_idx - 1]
        if boundary == "MARK_START" and dist <= approach_dist_m:
            phase = MissionPhase.APPROACH_MARK
        elif prev_mark:
            phase = MissionPhase.AFT_EXT
        elif _mark_ahead(seg_mark, seg_idx):
            phase = MissionPhase.PRE_EXT
        else:
            phase = MissionPhase.TRANSIT

    return ProgressMsg(
        phase=phase, segment_index=seg_idx, point_index=-1,
        dist_to_next_boundary_m=dist, next_boundary=boundary,
        speed_mps=speed, stopped=stopped, xtrack_m=signed_xtrack,
    )


def _classify_point(
    *, seg_idx: int, along_s: float, cum_s: list, speed: float, stopped: bool,
    signed_xtrack: float, point_active: bool, point_dwelling: bool,
    point_wait_operator: bool, point_target_rank: int, point_target_vertex: int,
    all_points_done: bool,
) -> ProgressMsg:
    if all_points_done:
        return ProgressMsg(phase=MissionPhase.TRANSIT, segment_index=seg_idx,
                           speed_mps=speed, stopped=stopped, xtrack_m=signed_xtrack,
                           point_index=-1)
    if point_active and point_wait_operator:
        phase = MissionPhase.WAIT_OPERATOR    # G5 manual: dwell done, awaiting advance
    elif point_active and point_dwelling:
        phase = MissionPhase.DWELL_HOLD
    elif point_active:
        phase = MissionPhase.APPROACH_POINT   # braking toward the confirmed stop
    else:
        phase = MissionPhase.APPROACH_POINT   # closing on the next undone point

    dist = 0.0
    if point_target_vertex >= 0 and point_target_vertex < len(cum_s):
        dist = max(0.0, cum_s[point_target_vertex] - along_s)
    return ProgressMsg(
        phase=phase, segment_index=seg_idx, point_index=point_target_rank,
        dist_to_next_boundary_m=dist, next_boundary=MilestoneEvent.AT_POINT,
        speed_mps=speed, stopped=stopped, xtrack_m=signed_xtrack,
    )


# Phases that mean "inside a painted region" for MARK_START/STOP edge detection.
_MARK_PHASES = frozenset({MissionPhase.MARK_TRACKING, MissionPhase.MARK_END})


def milestones_for(prev: MissionPhase, new: MissionPhase,
                   prev_point: int, new_point: int) -> list:
    """Discrete milestone events for a phase edge (design §4.2).

    Pure: same inputs → same list. The node owns the monotonic `seq` and the
    once-per-transition guard (it only calls this when phase or point changed).
    """
    events: list = []
    entered_mark = new in _MARK_PHASES and prev not in _MARK_PHASES
    left_mark = prev in _MARK_PHASES and new not in _MARK_PHASES
    if entered_mark:
        events.append(MilestoneEvent.MARK_START)
    if left_mark:
        events.append(MilestoneEvent.MARK_STOP)
    if new == MissionPhase.PRE_EXT and prev != MissionPhase.PRE_EXT:
        events.append(MilestoneEvent.PRE_START)
    if prev == MissionPhase.AFT_EXT and new != MissionPhase.AFT_EXT:
        events.append(MilestoneEvent.AFT_STOP)
    # Point handshake: AT_POINT at the moment dwell begins; DWELL_DONE when the
    # hold releases (dwell → non-dwell for the same or advancing point).
    if new == MissionPhase.DWELL_HOLD and prev != MissionPhase.DWELL_HOLD:
        events.append(MilestoneEvent.AT_POINT)
    if prev == MissionPhase.DWELL_HOLD and new != MissionPhase.DWELL_HOLD:
        events.append(MilestoneEvent.DWELL_DONE_RPP)
    if new == MissionPhase.REACHED_END and prev != MissionPhase.REACHED_END:
        events.append(MilestoneEvent.REACHED_END)
    return events
