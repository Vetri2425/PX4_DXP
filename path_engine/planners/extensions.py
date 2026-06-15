"""Drive extension logic for marking paths.

For every extendable MARK geometry, the rover drives extra distance before and
after the real CAD marking geometry.  Spray is ON only on the original CAD
geometry; the PRE/AFT extensions are TRANSIT (spray OFF).

Extension direction priority (evaluated in order):
  1. ``metadata["start_tangent"]`` / ``metadata["end_tangent"]`` — exact analytic
     tangent vectors injected by ``dxf_parser`` for ARC and CIRCLE entities.
  2. ``_is_line_like_segment()`` — line-like metadata or ``LINE_``,
     ``LWPOLYLINE_``, ``POLYLINE_`` source prefixes: infer direction from
     first/last adjacent densified points.
  3. No match → return unchanged copy (unknown geometry, no metadata).

Tangent formula (Stage 7A audit, verified against ``arc_waypoints()``):
  DXF angle θ (0°=East, CCW positive), in (north, east) tuple convention:
    CCW tangent at θ = (cos θ, -sin θ)
    CW  tangent at θ = (-cos θ,  sin θ)

ARC/CIRCLE extension is only active when ``metadata["start_tangent"]`` and
``metadata["end_tangent"]`` are present.  A raw ``ARC_...`` or ``CIRCLE_...``
PathSegment created without going through ``dxf_parser`` will have no metadata
and will be returned unchanged (safe fallback).

Metadata survival chain:
  dxf_parser.entities_to_segments()
    → densify_segment()           [straight_line.py — copies metadata]
    → optimize_segment_order()    [segment_order.py — swaps/negates on reversal]
    → split_mark_segment_with_extensions()   [this module — copies metadata]
    → apply_spray_latency_compensation()     [spray.py — copies metadata]
"""

from __future__ import annotations

import logging
import math

from ..core import DXFEntity, PathSegment, SegmentType, dxf_arc_tangent

log = logging.getLogger("path_engine.extensions")


# ---------------------------------------------------------------------------
# Low-level geometry helpers
# ---------------------------------------------------------------------------

def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Euclidean distance between two (north, east) points."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _unit_vector(
    a: tuple[float, float],
    b: tuple[float, float],
) -> tuple[float, float] | None:
    """Unit vector from *a* towards *b*.

    Returns None when the two points are coincident (distance < 1e-9 m).
    """
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return None
    return (dx / length, dy / length)


def _offset_point(
    p: tuple[float, float],
    direction: tuple[float, float],
    distance: float,
) -> tuple[float, float]:
    """Offset point *p* by *distance* metres along *direction* (unit vector).

    Use a negative *distance* to step backwards (opposite to *direction*).
    """
    return (p[0] + direction[0] * distance, p[1] + direction[1] * distance)


def _copy_segment(segment: PathSegment) -> PathSegment:
    """Return a shallow-copy of *segment* with all six fields preserved.

    Using a helper keeps every guard return site consistent so that adding
    a new field to PathSegment only requires updating this one function.
    """
    return PathSegment(
        segment_type=segment.segment_type,
        points=list(segment.points),
        speed=segment.speed,
        segment_id=segment.segment_id,
        source_entity=segment.source_entity,
        metadata=dict(segment.metadata),
    )


# ---------------------------------------------------------------------------
# Geometry classification
# ---------------------------------------------------------------------------

_LINE_LIKE_GEOMETRY_TYPES = {
    "LINE",
    "LWPOLYLINE",
    "POLYLINE",
    "LINE_CHAIN",
}

_LINE_LIKE_PREFIXES = ("LINE_", "LWPOLYLINE_", "POLYLINE_")


def _is_line_like_source(source_entity: str) -> bool:
    """Return True when a source_entity label names line-like geometry."""
    src = (source_entity or "").upper()
    if src.startswith(_LINE_LIKE_PREFIXES):
        return True

    if not src.startswith("GROUP:"):
        return False

    # Grouped labels are synthetic. Current grouping writes
    # ``group:<first_source>+<extra_count>``; older/audit examples may carry
    # ``group:LINE_A+LINE_B``. Accept only when every explicit geometry token is
    # line-like. Numeric count suffixes are bookkeeping, not geometry tokens.
    body = src.split(":", 1)[1]
    geometry_tokens = [
        token for token in body.split("+")
        if token and not token.isdigit()
    ]
    return bool(geometry_tokens) and all(
        token.startswith(_LINE_LIKE_PREFIXES) for token in geometry_tokens
    )


def _is_line_like_segment(segment: PathSegment) -> bool:
    """Return True for geometry profiles whose direction can be inferred from
    adjacent densified points (LINE / LWPOLYLINE / POLYLINE).

    ARC / CIRCLE / SPLINE / ELLIPSE / POINT are excluded unless they carry
    explicit tangent metadata, which is handled before this classifier.  An
    empty or unrecognised source_entity also returns False.
    """
    meta = segment.metadata or {}
    if meta.get("line_like") is True:
        return True

    geom = str(meta.get("geometry_type", "")).upper()
    if geom in _LINE_LIKE_GEOMETRY_TYPES:
        return True

    grouped_from = meta.get("grouped_from")
    if isinstance(grouped_from, (list, tuple)):
        sources = [str(src) for src in grouped_from if src]
        if sources and all(_is_line_like_source(src) for src in sources):
            return True

    return _is_line_like_source(segment.source_entity)


def _normalise(v: tuple[float, float]) -> tuple[float, float] | None:
    """Normalise vector *v* to a unit vector.  Returns None if near-zero."""
    length = math.hypot(v[0], v[1])
    if length < 1e-9:
        return None
    return (v[0] / length, v[1] / length)


def entity_extension_directions(
    entity: DXFEntity,
    points: list[tuple[float, float]],
) -> tuple[tuple[float, float], tuple[float, float]] | None:
    """Start/end unit tangents for PRE/AFT extensions of one DXF entity.

    Entity-level mirror of split_mark_segment_with_extensions() direction
    priority, used by the server's extension *preview* so that preview and
    planned geometry come from the same formulas:

      - ARC / CIRCLE: analytic tangents (dxf_arc_tangent — same source as
        the dxf_parser segment metadata).
      - LINE / LWPOLYLINE / SPLINE / ELLIPSE: finite differences from the
        densified *points*, matching the planner's line-like inference and
        the spline/ellipse finite-difference metadata.
      - POINT / unknown / degenerate geometry: None (the planner adds no
        extension for these either).

    Returns (start_dir, end_dir) unit vectors in (north, east), or None when
    no extension direction can be derived.
    """
    etype = entity.entity_type
    if etype == "ARC":
        geom = entity.geometry
        return (
            dxf_arc_tangent(geom.get("start_angle", 0.0)),
            dxf_arc_tangent(geom.get("end_angle", 360.0)),
        )
    if etype == "CIRCLE":
        # densify_circle starts at 0° (East point) and travels CCW; a full
        # circle ends where it started.
        return (dxf_arc_tangent(0.0), dxf_arc_tangent(0.0))
    if etype in ("LINE", "LWPOLYLINE", "POLYLINE", "SPLINE", "ELLIPSE"):
        if len(points) < 2:
            return None
        start_dir = _unit_vector(points[0], points[1])
        end_dir = _unit_vector(points[-2], points[-1])
        if start_dir is None or end_dir is None:
            return None
        return (start_dir, end_dir)
    return None


def offset_point(
    p: tuple[float, float],
    direction: tuple[float, float],
    distance: float,
) -> tuple[float, float]:
    """Public alias of _offset_point for preview/extension consumers."""
    return _offset_point(p, direction, distance)


# ---------------------------------------------------------------------------
# Main public function
# ---------------------------------------------------------------------------

def split_mark_segment_with_extensions(
    segment: PathSegment,
    pre_extension_m: float,
    aft_extension_m: float,
    transit_speed: float,
) -> list[PathSegment]:
    """Expand one MARK segment into [PRE-TRANSIT, MARK, AFT-TRANSIT].

    Extension direction is determined by the following priority:

    1. **Metadata tangents** (ARC / CIRCLE from dxf_parser):
       ``metadata["start_tangent"]`` and ``metadata["end_tangent"]`` are used
       as-is (normalised defensively).  These survive densification, optimizer
       reversal (with negation/swap), and spray compensation.

    2. **Line-like source_entity** (LINE / LWPOLYLINE / POLYLINE):
       Direction inferred from ``points[0]→points[1]`` (start) and
       ``points[-2]→points[-1]`` (end) of the densified point array.

    3. **No match** → return a copy of the original segment unchanged.

    Guards (returns single-element copy list):
      - segment is not MARK
      - segment has fewer than 2 points
      - no metadata tangents AND not line-like
      - direction vectors are degenerate (coincident points or zero-length)

    Original ``segment.points`` is **never mutated**.
    The returned MARK element contains ``dict(segment.metadata)`` — a copy.

    PRE/AFT TRANSIT segments carry:
      ``metadata["extension_role"]``        : "pre" | "aft"
      ``metadata["parent_source_entity"]``  : source_entity of original MARK

    Args:
        segment:          Source PathSegment (expected MARK).
        pre_extension_m:  Metres before start (0.0 → no PRE segment added).
        aft_extension_m:  Metres after end    (0.0 → no AFT segment added).
        transit_speed:    Speed (m/s) for PRE and AFT TRANSIT segments.

    Returns:
        List of 1, 2, or 3 PathSegments:
          [PRE-TRANSIT?] + [MARK] + [AFT-TRANSIT?]
    """
    # Guard 1: only MARK segments are extended
    if segment.segment_type != SegmentType.MARK:
        return [_copy_segment(segment)]

    # Guard 2: need at least 2 points to define a direction
    if len(segment.points) < 2:
        return [_copy_segment(segment)]

    start = segment.points[0]
    end   = segment.points[-1]
    meta  = segment.metadata

    # ── Direction priority ──────────────────────────────────────────────────

    has_tangent_meta = (
        "start_tangent" in meta and "end_tangent" in meta
    )

    if has_tangent_meta:
        # Priority 1: analytic tangent from metadata (ARC / CIRCLE)
        start_dir = _normalise(meta["start_tangent"])
        end_dir   = _normalise(meta["end_tangent"])
        if start_dir is None or end_dir is None:
            return [_copy_segment(segment)]

    elif _is_line_like_segment(segment):
        # Priority 2: infer from adjacent densified points (LINE / LWPOLYLINE)
        start_dir = _unit_vector(segment.points[0], segment.points[1])
        end_dir   = _unit_vector(segment.points[-2], segment.points[-1])
        if start_dir is None or end_dir is None:
            return [_copy_segment(segment)]

    else:
        # Priority 3: unknown geometry, no metadata → skip, return copy.
        # EX1 fix: warn instead of silently skipping, so the operator knows
        # this MARK segment will get no PRE/AFT run-up even though
        # enable_path_extensions is on.
        log.warning(
            "Path extension skipped for MARK segment %s (id=%s): no tangent "
            "metadata and geometry is not line-like — no PRE/AFT run-up "
            "will be added.",
            segment.source_entity, segment.segment_id,
        )
        return [_copy_segment(segment)]

    # ── Build result list ───────────────────────────────────────────────────

    result: list[PathSegment] = []

    # PRE extension: step backwards from start along start_dir
    if pre_extension_m > 0:
        pre_start = _offset_point(start, start_dir, -pre_extension_m)
        result.append(PathSegment(
            segment_type=SegmentType.TRANSIT,
            points=[pre_start, start],
            speed=transit_speed,
            segment_id=segment.segment_id,
            source_entity=f"{segment.source_entity}:pre",
            metadata={
                "extension_role": "pre",
                "parent_source_entity": segment.source_entity,
            },
        ))

    # Original MARK segment — copy, never mutate; preserve metadata
    result.append(PathSegment(
        segment_type=SegmentType.MARK,
        points=list(segment.points),
        speed=segment.speed,
        segment_id=segment.segment_id,
        source_entity=segment.source_entity,
        metadata=dict(meta),
    ))

    # AFT extension: step forward from end along end_dir
    if aft_extension_m > 0:
        aft_end = _offset_point(end, end_dir, aft_extension_m)
        result.append(PathSegment(
            segment_type=SegmentType.TRANSIT,
            points=[end, aft_end],
            speed=transit_speed,
            segment_id=segment.segment_id,
            source_entity=f"{segment.source_entity}:aft",
            metadata={
                "extension_role": "aft",
                "parent_source_entity": segment.source_entity,
            },
        ))

    return result
