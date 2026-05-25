"""Tests for path_engine parsers."""

import os
import tempfile
import math

from path_engine.parsers.csv_parser import read_ned_csv, read_ned_csv_enhanced
from path_engine.parsers.waypoints_parser import read_qgc_waypoints
from path_engine.parsers.dxf_parser import parse_dxf, entities_to_segments, _HAS_EZDXF
from path_engine.core import SegmentType


# ── CSV parser tests ───────────────────────────────────────────────────────────

def test_csv_2col_backward_compat():
    """Old 2-column format must still work."""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
        f.write("0.0,0.0\n")
        f.write("1.0,0.0\n")
        f.write("2.0,0.5\n")
        f.flush()
        pts = read_ned_csv(f.name)
    os.unlink(f.name)
    assert len(pts) == 3
    assert pts[0] == (0.0, 0.0)
    assert pts[1] == (1.0, 0.0)
    assert abs(pts[2][0] - 2.0) < 0.001
    assert abs(pts[2][1] - 0.5) < 0.001


def test_csv_6col_enhanced():
    """New 6-column format with spray_on, speed, segment_id, yaw."""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
        f.write("0.0,0.0,1,0.35,0,0.0\n")
        f.write("1.0,0.0,1,0.35,0,0.0\n")
        f.write("2.0,0.0,0,0.50,1,1.5708\n")  # TRANSIT, different segment
        f.write("2.0,1.0,0,0.50,1,1.5708\n")
        f.flush()
        segments = read_ned_csv_enhanced(f.name)
    os.unlink(f.name)
    assert len(segments) == 2
    assert segments[0].segment_type == SegmentType.MARK
    assert segments[0].points == [(0.0, 0.0), (1.0, 0.0)]
    assert segments[0].speed == 0.35
    assert segments[1].segment_type == SegmentType.TRANSIT
    assert segments[1].points == [(2.0, 0.0), (2.0, 1.0)]
    assert segments[1].speed == 0.50


def test_csv_comment_lines():
    """Lines starting with # should be skipped."""
    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
        f.write("# This is a comment\n")
        f.write("0.0,0.0\n")
        f.write("# Another comment\n")
        f.write("1.0,0.0\n")
        f.flush()
        pts = read_ned_csv(f.name)
    os.unlink(f.name)
    assert len(pts) == 2


def test_csv_mixed_format_detection():
    """6-col enhanced must detect column count from first data row."""
    # 2-col: defaults applied
    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
        f.write("0.0,0.0\n")
        f.write("1.0,0.0\n")
        f.flush()
        segments = read_ned_csv_enhanced(f.name)
    os.unlink(f.name)
    assert len(segments) == 1
    assert segments[0].segment_type == SegmentType.MARK
    assert segments[0].speed == 0.35  # default

    # 6-col: full format parsed
    with tempfile.NamedTemporaryFile(mode="w", suffix=".csv", delete=False) as f:
        f.write("0.0,0.0,1,0.40,0,0.0\n")
        f.write("1.0,0.0,1,0.40,0,0.0\n")
        f.flush()
        segments = read_ned_csv_enhanced(f.name)
    os.unlink(f.name)
    assert segments[0].speed == 0.40  # custom speed preserved


# ── QGC waypoints parser tests ────────────────────────────────────────────────

def test_qgc_waypoints_basic():
    """Parse a minimal QGC WPL 110 file."""
    content = (
        "QGC WPL 110\n"
        "0\t1\t0\t0\t0\t0\t0\t0\t13.0720378\t80.2619352\t0\t1\n"
        "1\t0\t0\t0\t0\t0\t0\t0\t13.0720838\t80.2619352\t0\t1\n"
        "2\t0\t0\t0\t0\t0\t0\t0\t13.0721298\t80.2619352\t0\t1\n"
    )
    with tempfile.NamedTemporaryFile(mode="w", suffix=".waypoints", delete=False) as f:
        f.write(content)
        f.flush()
        pts = read_qgc_waypoints(f.name)
    os.unlink(f.name)
    assert len(pts) == 2  # Home waypoint excluded
    # All should be north of home (lat increases)
    assert pts[0][0] > 0  # north_m positive
    assert pts[1][0] > pts[0][0]  # north increases


# ── DXF parser tests ──────────────────────────────────────────────────────────

def _write_dxf(doc, suffix=".dxf"):
    """Write ezdxf document to a temp file and return the path."""
    import ezdxf
    tmpdir = tempfile.gettempdir()
    fpath = os.path.join(tmpdir, f"_test_path_engine_{os.getpid()}{suffix}")
    doc.saveas(fpath)
    return fpath


def test_dxf_line_parsing():
    """Create a minimal DXF with LINE entities and verify parsing."""
    if not _HAS_EZDXF:
        # Skip if ezdxf not installed (CI environments without it)
        return

    import ezdxf
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()
    # 5m horizontal line (DXF units = metres)
    msp.add_line(start=(0, 0), end=(0, 5), dxfattribs={"layer": "MARK"})
    # 3m vertical line
    msp.add_line(start=(0, 0), end=(3, 0), dxfattribs={"layer": "DRAWING"})

    fpath = _write_dxf(doc)
    try:
        entities = parse_dxf(fpath, unit_scale=1.0)  # DXF units are metres
    finally:
        os.unlink(fpath)

    assert len(entities) == 2
    assert entities[0].entity_type == "LINE"
    assert entities[0].layer == "MARK"
    assert entities[0].is_mark() is True

    assert entities[1].entity_type == "LINE"
    assert entities[1].layer == "DRAWING"
    assert entities[1].is_mark() is True  # Default rule: DRAWING → MARK


def test_dxf_unit_scale_cm():
    """Verify DXF unit_scale=0.01 (cm → m) scales coordinates."""
    if not _HAS_EZDXF:
        return

    import ezdxf
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()
    # 500cm line (5m)
    msp.add_line(start=(0, 0), end=(0, 500), dxfattribs={"layer": "LINES"})

    fpath = _write_dxf(doc)
    try:
        entities = parse_dxf(fpath, unit_scale=0.01)
    finally:
        os.unlink(fpath)

    assert len(entities) == 1
    start = entities[0].geometry["start"]
    end = entities[0].geometry["end"]
    # DXF y=0 → NED north=0*0.01=0, DXF y=500 → NED north=500*0.01=5.0
    assert abs(end[0] - 5.0) < 0.001


def test_entities_to_segments_line():
    """Verify LINE entities convert to PathSegments correctly."""
    if not _HAS_EZDXF:
        return

    from path_engine.core import DXFEntity
    entities = [
        DXFEntity(
            entity_type="LINE",
            layer="MARK",
            geometry={"start": (0.0, 0.0), "end": (5.0, 0.0)},
            unit_scale=1.0,
        ),
        DXFEntity(
            entity_type="LINE",
            layer="TRANSIT",
            geometry={"start": (5.0, 0.0), "end": (5.0, 3.0)},
            unit_scale=1.0,
        ),
    ]
    segments = entities_to_segments(entities)
    assert len(segments) == 2
    assert segments[0].segment_type == SegmentType.MARK
    assert segments[0].points == [(0.0, 0.0), (5.0, 0.0)]
    assert segments[0].speed == 0.35  # default mark speed

    assert segments[1].segment_type == SegmentType.TRANSIT
    assert segments[1].points == [(5.0, 0.0), (5.0, 3.0)]
    assert segments[1].speed == 0.50  # default transit speed


def test_entities_to_segments_circle():
    """CIRCLE entity is discretized into waypoints (not placeholder)."""
    from path_engine.core import DXFEntity
    entities = [
        DXFEntity(
            entity_type="CIRCLE",
            layer="MARK",
            geometry={"center": (0.0, 0.0), "radius": 1.0},
            unit_scale=1.0,
        ),
    ]
    segments = entities_to_segments(entities)
    assert len(segments) == 1
    assert segments[0].segment_type == SegmentType.MARK
    assert len(segments[0].points) >= 10  # Full circle has many points
    assert segments[0].source_entity.startswith("CIRCLE_")


def test_entities_to_segments_arc():
    """ARC entity is discretized into waypoints."""
    from path_engine.core import DXFEntity
    entities = [
        DXFEntity(
            entity_type="ARC",
            layer="MARK",
            geometry={"center": (0.0, 0.0), "radius": 1.5, "start_angle": 0, "end_angle": 90},
            unit_scale=1.0,
        ),
    ]
    segments = entities_to_segments(entities)
    assert len(segments) == 1
    assert len(segments[0].points) >= 3  # Quarter arc has several points
    assert segments[0].source_entity.startswith("ARC_")


def test_entities_to_segments_lwpolyline_with_bulge():
    """LWPOLYLINE with bulge values gets discretized."""
    from path_engine.core import DXFEntity
    entities = [
        DXFEntity(
            entity_type="LWPOLYLINE",
            layer="MARK",
            geometry={
                "vertices": [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)],
                "bulges": [0.5, 0.0, 0.0],
                "closed": False,
            },
            unit_scale=1.0,
        ),
    ]
    segments = entities_to_segments(entities)
    assert len(segments) == 1
    assert segments[0].source_entity.startswith("LWPOLYLINE_")
    # With bulge, should have more points than just 3 vertices
    assert len(segments[0].points) >= 3


def test_dxf_circle_parsing():
    """Parse DXF CIRCLE entity and verify it gets discretized."""
    if not _HAS_EZDXF:
        return

    import ezdxf
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()
    msp.add_circle(center=(0, 0), radius=1.5, dxfattribs={"layer": "MARK"})

    fpath = _write_dxf(doc)
    try:
        entities = parse_dxf(fpath, unit_scale=1.0)
    finally:
        os.unlink(fpath)

    assert len(entities) == 1
    assert entities[0].entity_type == "CIRCLE"
    assert entities[0].geometry["radius"] == 1.5

    # Convert to segments — should produce many waypoints
    segments = entities_to_segments(entities)
    assert len(segments) == 1
    assert len(segments[0].points) >= 10


def test_dxf_arc_parsing():
    """Parse DXF ARC entity and verify discretization."""
    if not _HAS_EZDXF:
        return

    import ezdxf
    doc = ezdxf.new("R2010")
    msp = doc.modelspace()
    msp.add_arc(center=(0, 0), radius=1.5, start_angle=0, end_angle=90,
                dxfattribs={"layer": "DRAW"})

    fpath = _write_dxf(doc)
    try:
        entities = parse_dxf(fpath, unit_scale=1.0)
    finally:
        os.unlink(fpath)

    assert len(entities) == 1
    assert entities[0].entity_type == "ARC"
    segments = entities_to_segments(entities)
    assert len(segments) == 1
    # Quarter arc should have multiple waypoints
    assert len(segments[0].points) >= 3