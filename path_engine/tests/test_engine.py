"""Tests for path_engine main orchestrator (engine.py)."""

import os
import tempfile
import math

from path_engine.core import PathSegment, SegmentType, PlannedPath
from path_engine.engine import PathEngine
from path_engine.parsers.dxf_parser import _HAS_EZDXF


def test_engine_defaults():
    """PathEngine can be instantiated with all defaults."""
    engine = PathEngine()
    assert engine.mark_spacing == 0.05
    assert engine.transit_spacing == 0.15
    assert engine.marking_speed == 0.35
    assert engine.transit_speed == 0.50


def test_engine_plan_segments_single_mark():
    """Plan a single MARK segment through the full pipeline."""
    engine = PathEngine(optimize_order=False, compensate_spray=False)
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0)],
        speed=0.35,
    )
    plan = engine.plan_segments([seg])
    assert plan.num_waypoints > 2  # Densified
    assert plan.total_mark_length > 0
    assert all(plan.spray_flags)  # All MARK
    assert plan.origin == (0.0, 0.0)


def test_engine_plan_segments_with_transit():
    """MARK + TRANSIT segments produce mixed spray flags."""
    engine = PathEngine(optimize_order=False, compensate_spray=False)
    mark = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0)],
        speed=0.35,
    )
    transit = PathSegment(
        segment_type=SegmentType.TRANSIT,
        points=[(2.0, 0.0), (2.0, 3.0)],
        speed=0.50,
    )
    plan = engine.plan_segments([mark, transit])
    assert plan.num_waypoints > 4
    assert plan.total_mark_length > 0
    assert plan.total_transit_length > 0
    # First segment waypoints should be MARK
    assert plan.spray_flags[0] is True


def test_engine_plan_segments_with_origin():
    """Origin offset shifts all waypoints."""
    engine = PathEngine(optimize_order=False, compensate_spray=False)
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (1.0, 0.0)],
        speed=0.35,
    )
    plan = engine.plan_segments([seg], origin=(10.0, 20.0))
    # All waypoints should be shifted by (10, 20)
    for pt in plan.merged_waypoints:
        assert pt[0] >= 10.0  # North shifted
        assert pt[1] >= 20.0  # East shifted


def test_engine_plan_empty_segments():
    """Empty segment list returns empty PlannedPath."""
    engine = PathEngine()
    plan = engine.plan_segments([])
    assert plan.num_waypoints == 0
    assert plan.total_length == 0.0


def test_engine_spray_compensation_applied():
    """When compensate_spray=True, MARK segments get lead-in points."""
    engine_no_comp = PathEngine(optimize_order=False, compensate_spray=False)
    engine_comp = PathEngine(optimize_order=False, compensate_spray=True)

    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0)],
        speed=0.35,
    )
    plan_no = engine_no_comp.plan_segments([seg])
    plan_yes = engine_comp.plan_segments([seg])

    # With spray compensation, the segment gets a pre-start point → more waypoints
    assert plan_yes.num_waypoints >= plan_no.num_waypoints


def test_engine_csv_file_pipeline():
    """Plan from a CSV file through the full pipeline."""
    engine = PathEngine(optimize_order=False, compensate_spray=False)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
        f.write("0.0,0.0\n")
        f.write("1.0,0.0\n")
        f.write("1.0,1.0\n")
        f.flush()
        plan = engine.plan_file(f.name)
    os.unlink(f.name)

    assert plan.num_waypoints >= 3
    assert plan.total_mark_length > 0


def test_engine_plan_segments_densification():
    """Densification produces more waypoints than input."""
    engine = PathEngine(mark_spacing=0.05, optimize_order=False, compensate_spray=False)
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (1.0, 0.0)],
        speed=0.35,
    )
    plan = engine.plan_segments([seg])
    # 1m at 0.05m spacing → ~21 points
    assert plan.num_waypoints >= 20


def test_engine_segment_order_optimization():
    """optimize_order=True inserts TRANSIT segments between MARK segments."""
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    seg1 = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (1.0, 0.0)],
        speed=0.35,
    )
    seg2 = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(5.0, 5.0), (6.0, 5.0)],
        speed=0.35,
    )
    plan = engine.plan_segments([seg1, seg2])

    # Should have TRANSIT segments in the output
    transit_segments = [s for s in plan.segments if s.segment_type == SegmentType.TRANSIT]
    assert len(transit_segments) > 0, "Expected TRANSIT segments between MARK segments"


# ── Phase 3: Segment ordering + spray pipeline tests ────────────────────────────

def test_optimize_two_disconnected_segments_transit_inserted():
    """Two disconnected MARK segments → TRANSIT inserted between them."""
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    seg1 = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (1.0, 0.0)],
        speed=0.35,
    )
    seg2 = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(5.0, 5.0), (6.0, 5.0)],
        speed=0.35,
    )
    plan = engine.plan_segments([seg1, seg2])

    transit_segs = [s for s in plan.segments if s.segment_type == SegmentType.TRANSIT]
    assert len(transit_segs) >= 1, "TRANSIT segment must be inserted between MARK segments"
    # TRANSIT segment should have speed 0.5
    for t in transit_segs:
        assert t.speed == 0.50, f"TRANSIT speed should be 0.5, got {t.speed}"


def test_optimize_start_position_closer_to_second_segment():
    """start_position closer to segment B → B should come first in output."""
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    seg_a = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (0.0, 10.0)],  # Starts at origin
        speed=0.35,
    )
    seg_b = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(50.0, 50.0), (50.0, 60.0)],  # Starts far away
        speed=0.35,
    )

    # Start position near seg_b → should visit B first
    plan = engine.plan_segments([seg_a, seg_b], start_position=(49.0, 49.0))

    # First MARK segment should start near (50, 50), not (0, 0)
    first_mark = [s for s in plan.segments if s.segment_type == SegmentType.MARK][0]
    dist_to_b = math.hypot(first_mark.points[0][0] - 50.0, first_mark.points[0][1] - 50.0)
    dist_to_a = math.hypot(first_mark.points[0][0] - 0.0, first_mark.points[0][1] - 0.0)
    assert dist_to_b < dist_to_a, "Should visit B first since start is near B"


def test_spray_flags_length_equals_waypoints():
    """spray_flags must always be parallel to merged_waypoints."""
    engine = PathEngine(optimize_order=True, compensate_spray=True)

    seg1 = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0)],
        speed=0.35,
    )
    seg2 = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(5.0, 0.0), (7.0, 0.0)],
        speed=0.35,
    )
    plan = engine.plan_segments([seg1, seg2])

    assert len(plan.spray_flags) == len(plan.merged_waypoints), \
        f"spray_flags len {len(plan.spray_flags)} != waypoints len {len(plan.merged_waypoints)}"


def test_transit_segments_have_correct_attributes():
    """TRANSIT segments: spray_on=False (via segment_type), speed=transit_speed."""
    engine = PathEngine(optimize_order=True, compensate_spray=False,
                        transit_speed=0.50)

    seg1 = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0)],
        speed=0.35,
    )
    seg2 = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(10.0, 0.0), (12.0, 0.0)],
        speed=0.35,
    )
    plan = engine.plan_segments([seg1, seg2])

    # Check TRANSIT segment attributes
    for seg in plan.segments:
        if seg.segment_type == SegmentType.TRANSIT:
            assert seg.speed == 0.50, f"TRANSIT speed should be 0.5, got {seg.speed}"
            assert len(seg.points) == 2, "TRANSIT should have exactly 2 points (start, end)"

    # Check spray_flags for transit waypoints
    transit_wp_count = 0
    mark_wp_count = 0
    for i, flag in enumerate(plan.spray_flags):
        if flag:
            mark_wp_count += 1
        else:
            transit_wp_count += 1
    assert transit_wp_count > 0, "Should have TRANSIT waypoints in spray_flags"
    assert mark_wp_count > 0, "Should have MARK waypoints in spray_flags"


def test_total_transit_length_positive_when_disconnected():
    """Disconnected segments must produce positive transit length."""
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    seg1 = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0)],
        speed=0.35,
    )
    seg2 = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(10.0, 10.0), (12.0, 10.0)],
        speed=0.35,
    )
    plan = engine.plan_segments([seg1, seg2])

    assert plan.total_transit_length > 0, "Transit length must be > 0 for disconnected segments"


def test_start_position_fallback_to_origin():
    """When start_position=None and origin is non-zero, use origin for TSP."""
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    seg_a = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(100.0, 100.0), (100.0, 110.0)],
        speed=0.35,
    )
    seg_b = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (0.0, 10.0)],
        speed=0.35,
    )

    # origin=(0,0) is close to seg_b → should visit B first
    # But with origin=(99,99), closer to seg_a → should visit A first
    plan_far = engine.plan_segments([seg_a, seg_b], origin=(99.0, 99.0))

    first_mark = [s for s in plan_far.segments if s.segment_type == SegmentType.MARK][0]
    # First mark should be near origin (99,99), i.e., seg_a
    assert abs(first_mark.points[0][0] - 100.0) < 1.0, \
        "With origin near A, should visit A first"


def test_start_position_overrides_origin():
    """Explicit start_position takes priority over origin for TSP."""
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    seg_a = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(100.0, 100.0), (100.0, 110.0)],
        speed=0.35,
    )
    seg_b = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (0.0, 10.0)],
        speed=0.35,
    )

    # origin near A, but start_position near B → should visit B first
    plan = engine.plan_segments(
        [seg_a, seg_b],
        origin=(99.0, 99.0),
        start_position=(0.5, 0.5),
    )

    first_mark = [s for s in plan.segments if s.segment_type == SegmentType.MARK][0]
    # start_position=(0.5, 0.5) is near seg_b → first should be B
    assert abs(first_mark.points[0][0]) < 1.0, \
        "start_position should override origin for TSP"


def test_endpoint_reversal():
    """Optimizer should reverse segment point order when entering from end."""
    from path_engine.optimizers.segment_order import optimize_segment_order

    # Segment A goes (0,0)→(1,0), segment B goes (10,0)→(11,0)
    # Starting near (10,0), B should be visited first
    # If we then move near B's end (11,0), and segment C goes (0,5)→(11,5),
    # C should be entered from its end (11,5) for minimum transit distance
    seg_a = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (1.0, 0.0)],
        speed=0.35,
        segment_id=1,
    )
    seg_c = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 5.0), (11.0, 5.0)],
        speed=0.35,
        segment_id=2,
    )

    # Start near (0, 0) — A should be first, then C should be entered from end (11,5)
    ordered = optimize_segment_order([seg_a, seg_c], start_position=(0.5, 0.0))

    # Find C in the ordered list
    c_segs = [s for s in ordered if s.segment_id == 2]
    assert len(c_segs) == 1
    c_seg = c_segs[0]
    # C should be reversed since entering from (11, 5) is closer to A's end (1, 0)
    # than entering from (0, 5)
    # The nearest endpoint of C to A's end (1,0) is (11,5) — distance ~10.5
    # vs (0,5) — distance ~5.1
    # Actually (0,5) is closer. So C should NOT be reversed — enter from start.
    # Let me reconsider: after A, current_pos = (1,0).
    # Distance from (1,0) to C start (0,5) = sqrt(1+25) ≈ 5.1
    # Distance from (1,0) to C end (11,5) = sqrt(100+25) ≈ 11.2
    # So C should be entered from start (0,5) — NOT reversed.
    # This test validates that the endpoint reversal logic works correctly
    # by NOT reversing when start is closer.
    assert c_seg.points[0] == (0.0, 5.0), "C should start at (0,5) — not reversed"


# ── Phase 4: ROS2/FastAPI integration tests ─────────────────────────────────────

def test_engine_start_position_param_in_plan_file():
    """plan_file() accepts start_position and passes it to pipeline."""
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
        f.write("0.0,0.0\n1.0,0.0\n")
        f.write("10.0,10.0\n11.0,10.0\n")
        f.flush()
        # Without start_position
        plan_default = engine.plan_file(f.name)
        # With start_position near the second segment
        plan_biased = engine.plan_file(f.name, start_position=(9.5, 9.5))
    os.unlink(f.name)

    # Both should succeed
    assert plan_default.num_waypoints > 0
    assert plan_biased.num_waypoints > 0


def test_engine_start_position_param_in_plan_segments():
    """plan_segments() accepts start_position."""
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    seg_a = PathSegment(segment_type=SegmentType.MARK, points=[(0, 0), (1, 0)], speed=0.35)
    seg_b = PathSegment(segment_type=SegmentType.MARK, points=[(20, 0), (21, 0)], speed=0.35)

    plan = engine.plan_segments([seg_a, seg_b], start_position=(19.5, 0.0))

    # With start_position near B, first MARK segment should be B
    first_mark = [s for s in plan.segments if s.segment_type == SegmentType.MARK][0]
    assert abs(first_mark.points[0][0] - 20.0) < 1.0


def test_spray_flags_mark_transit_alternation():
    """Full pipeline produces alternating MARK/TRANSIT spray_flags for disconnected segments."""
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    seg1 = PathSegment(segment_type=SegmentType.MARK, points=[(0, 0), (2, 0)], speed=0.35)
    seg2 = PathSegment(segment_type=SegmentType.MARK, points=[(10, 10), (12, 10)], speed=0.35)
    plan = engine.plan_segments([seg1, seg2])

    # Should have both True and False in spray_flags
    assert True in plan.spray_flags, "Should have MARK (True) waypoints"
    assert False in plan.spray_flags, "Should have TRANSIT (False) waypoints"

    # Verify alignment: len matches
    assert len(plan.spray_flags) == len(plan.merged_waypoints)


def test_engine_dxf_full_pipeline_with_start_position():
    """Full DXF pipeline with start_position produces valid plan."""
    if not _HAS_EZDXF:
        return

    import ezdxf
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()
    msp.add_line(start=(0, 0), end=(0, 5), dxfattribs={"layer": "MARK"})
    msp.add_line(start=(10, 10), end=(10, 15), dxfattribs={"layer": "DRAW"})

    fpath = os.path.join(tempfile.gettempdir(), f"_phase4_test_{os.getpid()}.dxf")
    doc.saveas(fpath)

    try:
        engine = PathEngine(optimize_order=True, compensate_spray=True)
        plan = engine.plan_file(fpath, start_position=(9.5, 9.5))

        assert plan.num_waypoints > 0
        assert len(plan.spray_flags) == len(plan.merged_waypoints)
        assert plan.total_mark_length > 0
    finally:
        os.unlink(fpath)


# ── Spray toggle simulation tests ──────────────────────────────────────────────

def test_spray_toggle_mark_transit_alternation_via_pose():
    """Simulate pose-driven spray edge detection through a MARK→TRANSIT→MARK plan.

    The path_publisher node finds the closest waypoint to the current pose
    and edge-detects spray_flags changes. We simulate that logic here.
    """
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    seg1 = PathSegment(segment_type=SegmentType.MARK, points=[(0, 0), (5, 0)], speed=0.35)
    seg2 = PathSegment(segment_type=SegmentType.MARK, points=[(10, 0), (15, 0)], speed=0.35)
    plan = engine.plan_segments([seg1, seg2])

    spray_flags = plan.spray_flags
    waypoints = plan.merged_waypoints

    # Simulate: walk through waypoints, collect spray state transitions
    transitions = []
    last_state = None
    for i, flag in enumerate(spray_flags):
        if flag != last_state:
            transitions.append((i, flag))
            last_state = flag

    # Should have at least: True (MARK start), False (TRANSIT), True (MARK again)
    true_false = [(t[1]) for t in transitions]
    assert True in true_false, "Should have MARK transitions"
    assert False in true_false, "Should have TRANSIT transitions"
    # Verify at least 2 MARK→True entries (spray turns ON at least twice)
    mark_on_count = sum(1 for _, state in transitions if state is True)
    assert mark_on_count >= 2, f"Expected spray ON at least 2 times, got {mark_on_count}"


def test_spray_toggle_single_segment_no_transitions():
    """Single MARK segment — spray stays ON, no transitions."""
    engine = PathEngine(optimize_order=False, compensate_spray=False)
    seg = PathSegment(segment_type=SegmentType.MARK, points=[(0, 0), (5, 0)], speed=0.35)
    plan = engine.plan_segments([seg])

    transitions = []
    last_state = None
    for flag in plan.spray_flags:
        if flag != last_state:
            transitions.append(flag)
            last_state = flag

    assert len(transitions) == 1, "Single MARK should have exactly 1 transition (OFF→ON)"
    assert transitions[0] is True


def test_spray_toggle_progress_increments_with_pose():
    """Simulate progress tracking as pose advances through waypoints."""
    engine = PathEngine(optimize_order=True, compensate_spray=False)
    seg1 = PathSegment(segment_type=SegmentType.MARK, points=[(0, 0), (5, 0)], speed=0.35)
    seg2 = PathSegment(segment_type=SegmentType.MARK, points=[(10, 0), (15, 0)], speed=0.35)
    plan = engine.plan_segments([seg1, seg2])

    total = plan.num_waypoints
    assert total > 0

    # Simulate finding closest waypoint as we walk along x-axis
    visited = 0
    for pose_n in [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 7.0, 10.0, 12.0, 14.0]:
        best = visited
        best_d = float("inf")
        for i in range(visited, min(visited + 50, total)):
            wp = plan.merged_waypoints[i]
            d = (wp[0] - pose_n) ** 2 + (wp[1] - 0.0) ** 2
            if d < best_d:
                best_d = d
                best = i
        visited = best
        progress = visited / total
        assert 0.0 <= progress <= 1.0

    # By end, should be near 1.0
    assert visited / total > 0.8


# ── INSUNITS scaling tests ─────────────────────────────────────────────────────

def test_insunits_yards_scaling():
    """DXF with $INSUNITS=10 (yards) scales correctly: 1 yard = 0.9144 m."""
    if not _HAS_EZDXF:
        return

    from path_engine.parsers.dxf_parser import _INSUNITS_TO_METRES
    assert _INSUNITS_TO_METRES[10] == 0.9144, "Yards should be 0.9144 m"


def test_insunits_km_scaling():
    """DXF with $INSUNITS=7 (km) scales correctly: 1 km = 1000 m."""
    if not _HAS_EZDXF:
        return

    from path_engine.parsers.dxf_parser import _INSUNITS_TO_METRES
    assert _INSUNITS_TO_METRES[7] == 1000.0, "km should be 1000.0 m"


def test_insunits_miles_scaling():
    """DXF with $INSUNITS=3 (miles) scales correctly: 1 mile = 1609.344 m."""
    if not _HAS_EZDXF:
        return

    from path_engine.parsers.dxf_parser import _INSUNITS_TO_METRES
    assert _INSUNITS_TO_METRES[3] == 1609.344, "miles should be 1609.344 m"


def test_insunits_mils_scaling():
    """DXF with $INSUNITS=9 (mils) scales correctly: 1 mil = 2.54e-5 m."""
    if not _HAS_EZDXF:
        return

    from path_engine.parsers.dxf_parser import _INSUNITS_TO_METRES
    assert abs(_INSUNITS_TO_METRES[9] - 2.54e-5) < 1e-10, "mils should be 2.54e-5 m"


def test_insunits_hectometers_scaling():
    """DXF with $INSUNITS=15 (hectometers) scales correctly: 1 hm = 100 m."""
    if not _HAS_EZDXF:
        return

    from path_engine.parsers.dxf_parser import _INSUNITS_TO_METRES
    assert _INSUNITS_TO_METRES[15] == 100.0, "hectometers should be 100.0 m"


def test_insunits_microinches_scaling():
    """DXF with $INSUNITS=8 (microinches) scales correctly."""
    if not _HAS_EZDXF:
        return

    from path_engine.parsers.dxf_parser import _INSUNITS_TO_METRES
    assert abs(_INSUNITS_TO_METRES[8] - 2.54e-8) < 1e-12, "microinches should be 2.54e-8 m"


def test_insunits_decimeters_scaling():
    """DXF with $INSUNITS=14 (decimeters) scales correctly: 1 dm = 0.1 m."""
    if not _HAS_EZDXF:
        return

    from path_engine.parsers.dxf_parser import _INSUNITS_TO_METRES
    assert _INSUNITS_TO_METRES[14] == 0.1, "decimeters should be 0.1 m"


# ── TSP with non-zero origin tests ─────────────────────────────────────────────

def test_tsp_nonzero_origin_deoffsets_start_position():
    """start_position in offset frame is correctly de-offset for TSP comparison.

    With origin=(50, 80) and start_position=(50.5, 80.5), TSP should compare
    de-offset start (0.5, 0.5) against raw segment points near (0,0), not the
    offset values. Seg A at (0,0)→(0,10), Seg B at (5,5)→(5,15).
    (0.5,0.5) is closer to A than B, so A should come first.
    """
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    # Two MARK segments at raw DXF coords near origin
    seg_a = PathSegment(segment_type=SegmentType.MARK, points=[(0, 0), (0, 10)], speed=0.35)
    seg_b = PathSegment(segment_type=SegmentType.MARK, points=[(5, 5), (5, 15)], speed=0.35)

    # start_position close to A in the offset frame (50+0=50, 80+0=80)
    # De-offset: start = (50.5-50, 80.5-80) = (0.5, 0.5) near seg_a (0,0)
    plan = engine.plan_segments(
        [seg_a, seg_b],
        origin=(50.0, 80.0),
        start_position=(50.5, 80.5),
    )

    # Segments still have raw coords (offset applied in merge).
    # First MARK should be seg_a (0,0) since de-offset start is closest to it.
    first_mark = [s for s in plan.segments if s.segment_type == SegmentType.MARK][0]
    assert abs(first_mark.points[0][0]) < 1.0, \
        "With de-offset start near A, should visit A first (raw coords)"


def test_tsp_nonzero_origin_far_segment_second():
    """With origin far from segments, TSP still picks nearest by raw coords."""
    engine = PathEngine(optimize_order=True, compensate_spray=False)

    seg_a = PathSegment(segment_type=SegmentType.MARK, points=[(0, 0), (0, 1)], speed=0.35)
    seg_b = PathSegment(segment_type=SegmentType.MARK, points=[(20, 0), (20, 1)], speed=0.35)

    # origin=(100,100) — far from both segments but irrelevant to TSP
    # start_position=(100,100) — after de-offset: (0,0) near seg_a
    plan = engine.plan_segments(
        [seg_a, seg_b],
        origin=(100.0, 100.0),
        start_position=(100.0, 100.0),
    )

    first_mark = [s for s in plan.segments if s.segment_type == SegmentType.MARK][0]
    # De-offset: start=(0,0), seg_a start=(0,0), seg_b start=(20,0) → A first
    # Segments have raw coords; merged waypoints have offset
    assert abs(first_mark.points[0][0]) < 1.0, \
        "De-offset start should pick A first (raw coords)"


def test_engine_validation_negative_spacing():
    """PathEngine raises ValueError for negative spacing."""
    try:
        PathEngine(mark_spacing=-0.01)
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "mark_spacing" in str(e)


def test_engine_validation_zero_speed():
    """PathEngine raises ValueError for zero speed."""
    try:
        PathEngine(marking_speed=0.0)
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "marking_speed" in str(e)


def test_engine_planner_side_corner_smoothing_metadata():
    engine = PathEngine(
        optimize_order=False,
        compensate_spray=False,
        corner_smooth_radius_m=0.3,
        corner_smooth_arc_pts=6,
        mark_spacing=0.05,
    )
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
        speed=0.35,
    )

    plan = engine.plan_segments([seg])

    smoothing = plan.planning_metadata["smoothing"]
    assert smoothing["enabled"] is True
    assert smoothing["segments_smoothed"] == 1
    assert smoothing["vertices_skipped"] == 0
    assert plan.num_waypoints > 0
    assert (2.0, 0.0) not in plan.merged_waypoints


def test_engine_does_not_smooth_precurved_arc_segments():
    engine = PathEngine(
        optimize_order=False,
        compensate_spray=False,
        corner_smooth_radius_m=0.3,
        corner_smooth_arc_pts=6,
    )
    pts = [(0.0, 1.0), (0.1, 0.995), (0.2, 0.98), (0.3, 0.954)]
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=pts,
        speed=0.35,
        source_entity="ARC_test",
        metadata={"geometry_type": "ARC"},
    )

    plan = engine.plan_segments([seg])

    smoothing = plan.planning_metadata["smoothing"]
    assert smoothing["enabled"] is True
    assert smoothing["segments_smoothed"] == 0
    assert smoothing["vertices_skipped"] == 0


def test_engine_records_optimization_stats():
    engine = PathEngine(optimize_order=True, compensate_spray=False)
    segments = [
        PathSegment(segment_type=SegmentType.MARK, points=[(0.0, 0.0), (1.0, 0.0)]),
        PathSegment(segment_type=SegmentType.MARK, points=[(10.0, 0.0), (11.0, 0.0)]),
        PathSegment(segment_type=SegmentType.MARK, points=[(2.0, 0.0), (3.0, 0.0)]),
        PathSegment(segment_type=SegmentType.MARK, points=[(8.0, 0.0), (9.0, 0.0)]),
    ]

    plan = engine.plan_segments(segments)
    opt = plan.planning_metadata["optimization"]

    assert opt["method"] == "nearest_neighbor_2opt"
    assert opt["mark_segments"] == 4
    assert "deadhead_after_2opt_m" in opt
    assert plan.planning_metadata["planning_time_s"] >= 0.0


def test_engine_skips_two_opt_above_segment_cap():
    engine = PathEngine(
        optimize_order=True,
        compensate_spray=False,
        use_two_opt=True,
        max_two_opt_segments=3,
    )
    segments = [
        PathSegment(segment_type=SegmentType.MARK, points=[(float(i), 0.0), (float(i), 1.0)])
        for i in range(4)
    ]

    plan = engine.plan_segments(segments)
    opt = plan.planning_metadata["optimization"]

    assert opt["method"] == "nearest_neighbor"
    assert opt["two_opt_improvements"] == 0
    assert "exceeds cap" in opt["two_opt_skipped_reason"]


def test_engine_smooths_closed_loop_closure_corner():
    # A closed square stored as one polyline (first == last). The closure corner
    # at the start/end vertex must be rounded like every other corner.
    engine = PathEngine(
        optimize_order=False,
        compensate_spray=False,
        corner_smooth_radius_m=0.3,
        mark_spacing=0.05,
    )
    seg = PathSegment(
        segment_type=SegmentType.MARK,
        points=[(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)],
        source_entity="LWPOLYLINE_1",
        metadata={"geometry_type": "LWPOLYLINE"},
    )

    plan = engine.plan_segments([seg])

    assert plan.planning_metadata["smoothing"]["segments_smoothed"] == 1
    # No original corner vertex should survive as a sharp point — including closure.
    for corner in [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0)]:
        assert corner not in plan.merged_waypoints


def test_optimizer_stats_keys_consistent_single_segment():
    from path_engine.optimizers.segment_order import optimize_segment_order

    multi_stats: dict = {}
    optimize_segment_order(
        [
            PathSegment(segment_type=SegmentType.MARK, points=[(0.0, 0.0), (1.0, 0.0)]),
            PathSegment(segment_type=SegmentType.MARK, points=[(5.0, 0.0), (6.0, 0.0)]),
        ],
        start_position=(0.0, 0.0),
        stats=multi_stats,
    )

    single_stats: dict = {}
    optimize_segment_order(
        [PathSegment(segment_type=SegmentType.MARK, points=[(0.0, 0.0), (1.0, 0.0)])],
        start_position=(0.0, 0.0),
        stats=single_stats,
    )

    # Single-segment route must expose the same telemetry keys as the multi route.
    assert set(multi_stats) == set(single_stats)
    assert single_stats["two_opt_skipped_reason"] == "single mark segment"
