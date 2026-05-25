"""DXF file parser using ezdxf.

Extracts geometric entities from DXF files and converts them to
DXFEntity objects suitable for path planning.

Supported entities:
  - LINE: straight line segments
  - POINT: single points (become zero-length MARK segments)
  - CIRCLE: full circle (discretized via chord-error method)
  - ARC: partial arc (discretized via chord-error method)
  - LWPOLYLINE: with bulge values (mixed line+arc segments)
  - SPLINE/HELIX: flattened via ezdxf make_path + flattening
  - ELLIPSE: elliptical arcs (flattened via ezdxf make_path)
  - INSERT: block references (decomposed recursively)

Layer-to-SegmentType mapping:
  Default rules (applied when layer_mapping is None or no match):
    - Layer name contains TRANSIT/TRAVEL/MOVE/RAPID → TRANSIT
    - Everything else → MARK
  Custom mapping via layer_mapping dict: {pattern: "mark" | "transit" | "ignore"}

Unit handling:
  ezdxf provides $INSUNITS header variable. Common values:
    1 = inches, 2 = feet, 4 = mm, 5 = cm, 6 = m
  If $INSUNITS is missing or 0, the unit_scale parameter is used (default 0.01 = cm).
"""

from __future__ import annotations

import logging
import math
import os

try:
    import ezdxf
    from ezdxf.path import make_path
    _HAS_EZDXF = True
except ImportError:
    _HAS_EZDXF = False

from ..core import DXFEntity, PathSegment, SegmentType
from ..planners.arc_curve import (
    densify_circle,
    densify_arc_from_dxf,
    densify_lwpolyline_bulge,
)

log = logging.getLogger("path_engine.dxf_parser")

# DXF $INSUNITS values to metres
_INSUNITS_TO_METRES = {
    0: None,     # unspecified — use unit_scale param
    1: 0.0254,   # inches
    2: 0.3048,   # feet
    3: 0.001,    # miles (unlikely but defined)
    4: 0.001,    # mm
    5: 0.01,     # cm
    6: 1.0,      # m
    7: 0.0,      # km (unlikely)
    8: 0.0000001, # microinches
    9: 0.000001,  # mils
    10: 0.0000254, # yards
    11: 0.00000001, # angstroms (unlikely)
    12: 0.000000001, # nanometers (unlikely)
    13: 0.000001,   # microns
    14: 0.001,      # decimeters (not standard, but some DXF use this)
    15: 0.0001,     # centimeters (some DXF use 15 for cm)
}


def _get_unit_scale(filepath: str, fallback: float = 0.01) -> float:
    """Read $INSUNITS from DXF file and return metres-per-unit scale.

    If $INSUNITS is missing or 0, returns the fallback value.
    """
    if not _HAS_EZDXF:
        raise ImportError("ezdxf is required for DXF files. Install: pip install ezdxf")

    try:
        doc = ezdxf.readfile(filepath)
        insunits = doc.header.get("$INSUNITS", 0)
        scale = _INSUNITS_TO_METRES.get(insunits)
        if scale is not None and scale > 0:
            return scale
    except Exception:
        pass
    return fallback


def parse_dxf(
    filepath: str,
    unit_scale: float | None = None,
    layer_mapping: dict[str, str] | None = None,
) -> list[DXFEntity]:
    """Parse a DXF file and extract geometric entities.

    Args:
        filepath: Path to the .dxf file.
        unit_scale: Metres per DXF unit. If None, auto-detected from $INSUNITS.
                    Common values: 0.01 (cm), 0.001 (mm), 1.0 (m).
        layer_mapping: Dict mapping layer name patterns to segment types.
                       Values: "mark", "transit", "ignore".
                       Example: {"TRANSIT": "transit", "DRAW": "mark"}

    Returns:
        List of DXFEntity objects with geometry dicts populated.
    """
    if not _HAS_EZDXF:
        raise ImportError("ezdxf is required for DXF files. Install: pip install ezdxf")

    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"DXF file not found: {filepath}")

    doc = ezdxf.readfile(filepath)
    msp = doc.modelspace()

    # Auto-detect unit scale
    if unit_scale is None:
        unit_scale = _get_unit_scale(filepath)

    entities: list[DXFEntity] = []

    for entity in msp:
        etype = entity.dxftype()
        layer = entity.dxf.layer
        color = entity.dxf.get("color", 7)
        handle = entity.dxf.get("handle", "")
        s = unit_scale  # shorthand

        if etype == "LINE":
            start = entity.dxf.start
            end = entity.dxf.end
            entities.append(DXFEntity(
                entity_type="LINE",
                layer=layer,
                color=color,
                entity_id=handle,
                geometry={
                    "start": (start.y * s, start.x * s),  # DXF y→NED north, x→east
                    "end": (end.y * s, end.x * s),
                },
                unit_scale=unit_scale,
            ))

        elif etype == "POINT":
            pos = entity.dxf.location
            entities.append(DXFEntity(
                entity_type="POINT",
                layer=layer,
                color=color,
                entity_id=handle,
                geometry={
                    "position": (pos.y * s, pos.x * s),
                },
                unit_scale=unit_scale,
            ))

        elif etype == "CIRCLE":
            center = entity.dxf.center
            radius = entity.dxf.radius * s
            entities.append(DXFEntity(
                entity_type="CIRCLE",
                layer=layer,
                color=color,
                entity_id=handle,
                geometry={
                    "center": (center.y * s, center.x * s),
                    "radius": radius,
                },
                unit_scale=unit_scale,
            ))

        elif etype == "ARC":
            center = entity.dxf.center
            radius = entity.dxf.radius * s
            start_angle = entity.dxf.start_angle
            end_angle = entity.dxf.end_angle
            entities.append(DXFEntity(
                entity_type="ARC",
                layer=layer,
                color=color,
                entity_id=handle,
                geometry={
                    "center": (center.y * s, center.x * s),
                    "radius": radius,
                    "start_angle": start_angle,
                    "end_angle": end_angle,
                },
                unit_scale=unit_scale,
            ))

        elif etype == "LWPOLYLINE":
            # LWPOLYLINE with vertices and optional bulge values
            vertices = list(entity.get_points(format="xyb"))
            # vertices: list of (x, y, bulge)
            pts = [(v[1] * s, v[0] * s) for v in vertices]  # (north, east)
            bulges = [v[2] if len(v) > 2 else 0.0 for v in vertices]
            closed = entity.closed
            entities.append(DXFEntity(
                entity_type="LWPOLYLINE",
                layer=layer,
                color=color,
                entity_id=handle,
                geometry={
                    "vertices": pts,
                    "bulges": bulges,
                    "closed": closed,
                },
                unit_scale=unit_scale,
            ))

        elif etype in ("SPLINE", "HELIX"):
            # SPLINE — use ezdxf's make_path + flattening for accurate discretization
            try:
                path = make_path(entity)
                flat_pts = list(path.flattening(distance=0.005 * s / unit_scale if unit_scale > 0 else 0.005))
                pts = [(p.y * s, p.x * s) for p in flat_pts]
                entities.append(DXFEntity(
                    entity_type="SPLINE",
                    layer=layer,
                    color=color,
                    entity_id=handle,
                    geometry={
                        "vertices": pts,
                        "closed": False,
                    },
                    unit_scale=unit_scale,
                ))
            except Exception:
                # Fallback: store control points for manual flattening
                control_points = list(entity.control_points) if hasattr(entity, "control_points") else []
                pts = [(cp.y * s, cp.x * s) for cp in control_points] if control_points else []
                entities.append(DXFEntity(
                    entity_type="SPLINE",
                    layer=layer,
                    color=color,
                    entity_id=handle,
                    geometry={
                        "vertices": pts,
                        "closed": False,
                    },
                    unit_scale=unit_scale,
                ))

        elif etype == "ELLIPSE":
            # ELLIPSE — use ezdxf's make_path + flattening (more accurate than manual parametric)
            try:
                path = make_path(entity)
                flat_pts = list(path.flattening(distance=0.005 * s / unit_scale if unit_scale > 0 else 0.005))
                pts = [(p.y * s, p.x * s) for p in flat_pts]
                entities.append(DXFEntity(
                    entity_type="ELLIPSE",
                    layer=layer,
                    color=color,
                    entity_id=handle,
                    geometry={
                        "vertices": pts,
                        "closed": False,
                    },
                    unit_scale=unit_scale,
                ))
            except Exception:
                # Fallback: store raw ellipse params
                center = entity.dxf.center
                major_axis = entity.dxf.major_axis
                ratio = entity.dxf.ratio
                start_param = entity.dxf.start_param
                end_param = entity.dxf.end_param
                entities.append(DXFEntity(
                    entity_type="ELLIPSE",
                    layer=layer,
                    color=color,
                    entity_id=handle,
                    geometry={
                        "center": (center.y * s, center.x * s),
                        "major_axis": (major_axis.y * s, major_axis.x * s),
                        "ratio": ratio,
                        "start_param": start_param,
                        "end_param": end_param,
                    },
                    unit_scale=unit_scale,
                ))

        # INSERT (block references) — decompose recursively
        elif etype == "INSERT":
            try:
                from ezdxf.disassemble import recursive_decompose
                for sub_entity in recursive_decompose(entity):
                    sub_etype = sub_entity.dxftype()
                    if sub_etype == "LINE":
                        start = sub_entity.dxf.start
                        end = sub_entity.dxf.end
                        entities.append(DXFEntity(
                            entity_type="LINE",
                            layer=layer,
                            color=color,
                            entity_id=handle + "_sub",
                            geometry={
                                "start": (start.y * s, start.x * s),
                                "end": (end.y * s, end.x * s),
                            },
                            unit_scale=unit_scale,
                        ))
            except Exception:
                pass  # Skip block references that can't be decomposed

        else:
            log.debug("Skipping unsupported DXF entity type: %s (layer=%s, handle=%s)",
                       etype, layer, handle)

    return entities


def entities_to_segments(
    entities: list[DXFEntity],
    layer_mapping: dict[str, str] | None = None,
    mark_speed: float = 0.35,
    transit_speed: float = 0.50,
    chord_error: float = 0.005,
    min_spacing: float = 0.02,
    max_spacing: float = 0.10,
) -> list[PathSegment]:
    """Convert DXFEntity list to PathSegment list.

    LINE entities become 2-point segments.
    POINT entities become single-point MARK segments (zero-length).
    CIRCLE entities are discretized into full circles.
    ARC entities are discretized into partial arcs.
    LWPOLYLINE entities are discretized with bulge-to-arc conversion.
    SPLINE/ELLIPSE entities (already flattened) become polyline segments.

    Args:
        entities: List of parsed DXFEntity objects.
        layer_mapping: Dict mapping layer name patterns to "mark"/"transit"/"ignore".
        mark_speed: Speed for MARK segments (m/s).
        transit_speed: Speed for TRANSIT segments (m/s).
        chord_error: Max deviation from true arc for curve discretization (m).
        min_spacing: Min waypoint spacing on curves (m).
        max_spacing: Max waypoint spacing on curves (m).

    Returns:
        List of PathSegment with discretized waypoints.
    """
    segments: list[PathSegment] = []
    seg_id = 0

    # Filter out ignored entities
    filtered: list[DXFEntity] = []
    for ent in entities:
        if layer_mapping:
            upper = ent.layer.upper()
            ignored = False
            for pattern, seg_type in layer_mapping.items():
                if pattern.upper() in upper and seg_type.upper() == "IGNORE":
                    ignored = True
                    break
            if ignored:
                continue
        filtered.append(ent)

    for ent in filtered:
        is_mark = ent.is_mark(layer_mapping)
        seg_type = SegmentType.MARK if is_mark else SegmentType.TRANSIT
        speed = mark_speed if is_mark else transit_speed

        if ent.entity_type == "LINE":
            start = ent.geometry["start"]
            end = ent.geometry["end"]
            segments.append(PathSegment(
                segment_type=seg_type,
                points=[start, end],
                speed=speed,
                segment_id=seg_id,
                source_entity=f"LINE_{ent.entity_id}",
            ))
            seg_id += 1

        elif ent.entity_type == "POINT":
            pos = ent.geometry["position"]
            segments.append(PathSegment(
                segment_type=seg_type,
                points=[pos],
                speed=speed,
                segment_id=seg_id,
                source_entity=f"POINT_{ent.entity_id}",
            ))
            seg_id += 1

        elif ent.entity_type == "CIRCLE":
            center = ent.geometry["center"]
            radius = ent.geometry["radius"]
            pts = densify_circle(
                center, radius,
                chord_error=chord_error,
                min_spacing=min_spacing,
                max_spacing=max_spacing,
            )
            segments.append(PathSegment(
                segment_type=seg_type,
                points=pts,
                speed=speed,
                segment_id=seg_id,
                source_entity=f"CIRCLE_{ent.entity_id}",
            ))
            seg_id += 1

        elif ent.entity_type == "ARC":
            center = ent.geometry["center"]
            radius = ent.geometry["radius"]
            start_angle = ent.geometry["start_angle"]
            end_angle = ent.geometry["end_angle"]
            pts = densify_arc_from_dxf(
                center, radius, start_angle, end_angle,
                unit_scale=ent.unit_scale,
                chord_error=chord_error,
                min_spacing=min_spacing,
                max_spacing=max_spacing,
            )
            segments.append(PathSegment(
                segment_type=seg_type,
                points=pts,
                speed=speed,
                segment_id=seg_id,
                source_entity=f"ARC_{ent.entity_id}",
            ))
            seg_id += 1

        elif ent.entity_type == "LWPOLYLINE":
            vertices = ent.geometry.get("vertices", [])
            bulges = ent.geometry.get("bulges", [0.0] * len(vertices))
            closed = ent.geometry.get("closed", False)

            has_bulge = any(abs(b) > 1e-9 for b in bulges)

            if has_bulge:
                # Mixed line+arc segments — use bulge-to-arc conversion
                pts = densify_lwpolyline_bulge(
                    vertices, bulges, closed,
                    chord_error=chord_error,
                    min_spacing=min_spacing,
                    max_spacing=max_spacing,
                )
            else:
                # Pure polyline (no arcs) — just use vertices
                pts = list(vertices)
                if closed and pts and pts[0] != pts[-1]:
                    pts.append(pts[0])

            if len(pts) >= 2:
                segments.append(PathSegment(
                    segment_type=seg_type,
                    points=pts,
                    speed=speed,
                    segment_id=seg_id,
                    source_entity=f"LWPOLYLINE_{ent.entity_id}",
                ))
                seg_id += 1

        elif ent.entity_type in ("SPLINE", "ELLIPSE"):
            # Already flattened by make_path + flattening in parse_dxf
            pts = ent.geometry.get("vertices", [])
            if len(pts) >= 2:
                segments.append(PathSegment(
                    segment_type=seg_type,
                    points=list(pts),
                    speed=speed,
                    segment_id=seg_id,
                    source_entity=f"{ent.entity_type}_{ent.entity_id}",
                ))
                seg_id += 1

    return segments