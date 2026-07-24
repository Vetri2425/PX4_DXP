"""Survey-export CSV parser (named-header point files).

This is the format a GNSS rover actually produces in the field — an Emlid Reach
/ Trimble / Leica / Topcon point export — as opposed to the headerless NED
metres file `csv_parser.py` handles. A real example (Emlid Reach RS3, 42
columns, one row per surveyed point):

    Name,Code,Code description,Easting,Northing,Elevation,Description,
    Longitude,Latitude,Ellipsoidal height,...,Lateral RMS,...,Solution status,
    Correction type,...,Samples,PDOP,...,CS name,...
    3,L_2,Line,1204637.765,1243758.291,9.890,,80.26195346,13.07208106,-82.342,
    ...,0.017,...,FIX,RTK,...,1,1.3,...,WGS 84 / Tamil Nadu + EGM96 height,...

Three columns carry the structure, and they are the survey industry's own
convention rather than anything invented here:

    Name  -> point number: ORDER within a line
    Code  -> feature code: WHICH LINE this point belongs to (L_1, L_2, ...)
    Lat/Lon or Northing/Easting -> geometry

`Code` is the "field-to-finish" base-code + instance grammar (Carlson `EP BEG`
/ `EP END`, Civil 3D `EP1 B`, Trimble .fxl start/end join sequence codes). Points
sharing a code, in Name order, form one line — which is exactly how the survey
software generated the companion DXF's linework in the first place.

Coordinate source, in preference order:

1. **Latitude/Longitude** -> projected to local ENU via `georef.metres_per_degree`.
   PREFERRED. This yields true ground distances, which is what the rover's EKF
   local frame is: a tangent plane. It also gives a `geo_origin` for
   GPS_SURVEYED placement.
2. **Northing/Easting** -> shifted to the local centroid, used as-is.
   FALLBACK ONLY. A projected grid carries a scale factor (the Tamil Nadu grid
   reads +0.042 % vs ground on our test line — 4 cm per 100 m). Ground truth is
   the geodesic, so grid coordinates are very slightly long. Use them only when
   the export carries no lat/lon, and note there is no geo_origin in that case.

Quality columns (`Solution status`, `Lateral RMS`, `PDOP`, `Samples`) are read
and surfaced as warnings, never silently dropped — a non-FIX or unaveraged point
is still imported, but the operator is told.
"""

from __future__ import annotations

import csv
import logging
import math
from typing import Optional

from ..core import PathSegment, SegmentType
from .georef import metres_per_degree

log = logging.getLogger(__name__)

# Column aliases, lower-cased and stripped. Order within each tuple is
# preference order. Kept generous because every vendor spells these slightly
# differently, and a rejected import is worse than a tolerant one.
# "seq"/"sequence" order the points WITHIN a line (planning-export style:
# feature,seq,lat,lon,chainage_m). "id" stays last so a per-point unique id
# never beats a real sequence column.
_COL_NAME = ("name", "point", "point name", "pointname", "point number", "seq",
             "sequence", "pt", "id")
# "feature"/"road"/"line" name WHICH line a point belongs to in a planning
# export (one segment per distinct value), the same role Code plays for a field
# survey. "description"/"desc" stay ahead of them for real survey exports.
_COL_CODE = ("code", "feature code", "featurecode", "description", "desc",
             "feature", "road", "line")
_COL_LAT = ("latitude", "lat")
_COL_LON = ("longitude", "lon", "long", "lng")
_COL_NORTH = ("northing", "north", "n", "y")
_COL_EAST = ("easting", "east", "e", "x")
_COL_STATUS = ("solution status", "solution", "fix", "fix type", "quality")
_COL_RMS = ("lateral rms", "horizontal rms", "hrms")
_COL_SAMPLES = ("samples", "epochs")
_COL_PDOP = ("pdop",)
_COL_CS = ("cs name", "coordinate system", "crs")

# A survey CSV is identified by a header row that names a coordinate pair.
# Without this the file is the legacy headerless NED format.
_REQUIRED_ANY = (_COL_LAT, _COL_LON, _COL_NORTH, _COL_EAST)

# Coincidence tolerance when a caller asks which points are control points.
CONTROL_SNAP_M = 0.01


def _pick(header_map: dict, aliases: tuple) -> Optional[str]:
    for alias in aliases:
        if alias in header_map:
            return header_map[alias]
    return None


def _header_map_from_line(first_line: str) -> Optional[dict]:
    """{lowercased column: original} from a header LINE, or None if not a survey header."""
    if not first_line or not first_line.strip():
        return None
    cols = [c.strip() for c in next(csv.reader([first_line]), [])]
    if len(cols) < 3:
        return None
    header_map = {c.strip().lower(): c for c in cols if c.strip()}
    hits = sum(1 for group in _REQUIRED_ANY if _pick(header_map, group))
    # Need a full coordinate pair, not just one axis.
    has_geo = bool(_pick(header_map, _COL_LAT) and _pick(header_map, _COL_LON))
    has_grid = bool(_pick(header_map, _COL_NORTH) and _pick(header_map, _COL_EAST))
    if not (has_geo or has_grid) or hits < 2:
        return None
    return header_map


def _read_header(filepath: str) -> Optional[dict]:
    """Return {lowercased column name: original name}, or None if not a survey CSV."""
    try:
        with open(filepath, "r", encoding="utf-8-sig", errors="replace") as f:
            first = f.readline()
    except OSError:
        return None
    return _header_map_from_line(first)


def read_survey_latlon_points(text: str) -> Optional[list[dict]]:
    """Ordered surveyed lat/lon points from a named-header survey CSV *text*.

    For POINT missions (Emlid/Trimble/Leica field exports, e.g. the 42-column
    Reach RS3 file): returns ``[{'lat','lon','name','code'}, ...]`` ordered by
    Name (numeric when possible, else file order). Lat/Lon ONLY — a point
    mission needs a geographic anchor, so a grid-only (Northing/Easting) export
    returns None. Returns None when the text is not a survey CSV at all (no
    named coordinate-pair header), so the caller can fall back to the bare
    ``lat,lon`` format. Shares the vendor column aliases with read_survey_csv.
    """
    if text.startswith("﻿"):    # strip a BOM the endpoint didn't
        text = text[1:]
    lines = text.splitlines()
    header_map = _header_map_from_line(lines[0]) if lines else None
    if header_map is None:
        return None
    c_lat = _pick(header_map, _COL_LAT)
    c_lon = _pick(header_map, _COL_LON)
    if not (c_lat and c_lon):
        return None                      # grid-only: no geo anchor for a point mission
    c_name = _pick(header_map, _COL_NAME)
    c_code = _pick(header_map, _COL_CODE)
    out: list[dict] = []
    reader = csv.DictReader(lines)
    # Vendor headers carry trailing spaces (e.g. "Latitude "). Detection strips
    # them, but DictReader keys rows on the raw header — so strip its fieldnames
    # to match the stripped column names picked above, or every row drops.
    reader.fieldnames = [(h or "").strip() for h in reader.fieldnames or []]
    for row in reader:
        lat, lon = _float(row.get(c_lat)), _float(row.get(c_lon))
        if lat is None or lon is None:
            continue
        out.append({
            "lat": lat,
            "lon": lon,
            "name": (row.get(c_name) or "").strip() if c_name else "",
            "code": (row.get(c_code) or "").strip() if c_code else "",
        })
    if not out:
        return None
    out.sort(key=lambda r: _order_key(r["name"]))
    return out


def looks_like_survey_csv(filepath: str) -> bool:
    """True when the file has a named header carrying a coordinate pair."""
    return _read_header(filepath) is not None


def _float(value) -> Optional[float]:
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return None


def _order_key(name: str):
    """Sort by point number when it is numeric, else lexically. Stable either way."""
    s = (name or "").strip()
    try:
        return (0, float(s), "")
    except ValueError:
        return (1, 0.0, s)


class SurveyCsvResult:
    """Segments plus everything the caller needs to explain the import."""

    def __init__(self, segments, geo_origin, warnings, cs_name, point_count, source):
        self.segments = segments
        self.geo_origin = geo_origin       # (lat, lon) or None
        self.warnings = warnings           # list[str], operator-facing
        self.cs_name = cs_name             # declared CRS string, or None
        self.point_count = point_count     # points IMPORTED (after any filtering)
        self.coordinate_source = source    # "latlon" | "grid"


def read_survey_csv(
    filepath: str,
    *,
    mark_speed: float = 0.35,
    require_fix: bool = False,
    max_lateral_rms_m: Optional[float] = None,
) -> SurveyCsvResult:
    """Parse a named-header survey point export into MARK segments.

    One segment per distinct `Code`, points ordered by `Name`. A file with no
    usable `Code` column becomes a single segment in file order.

    Args:
        require_fix: drop rows whose Solution status is not FIX (default: keep
            and warn — an operator who surveyed in FLOAT usually knows).
        max_lateral_rms_m: warn above this; None disables the check.
    """
    header_map = _read_header(filepath)
    if header_map is None:
        raise ValueError(
            f"{filepath} has no survey CSV header (expected named Latitude/Longitude "
            f"or Northing/Easting columns)"
        )

    c_name = _pick(header_map, _COL_NAME)
    c_code = _pick(header_map, _COL_CODE)
    c_lat = _pick(header_map, _COL_LAT)
    c_lon = _pick(header_map, _COL_LON)
    c_north = _pick(header_map, _COL_NORTH)
    c_east = _pick(header_map, _COL_EAST)
    c_status = _pick(header_map, _COL_STATUS)
    c_rms = _pick(header_map, _COL_RMS)
    c_samples = _pick(header_map, _COL_SAMPLES)
    c_pdop = _pick(header_map, _COL_PDOP)
    c_cs = _pick(header_map, _COL_CS)

    use_geo = bool(c_lat and c_lon)

    rows: list[dict] = []
    with open(filepath, "r", encoding="utf-8-sig", errors="replace") as f:
        reader = csv.DictReader(f)
        # Strip trailing-space vendor headers so row keys match the stripped
        # column names picked from the header (see read_survey_latlon_points).
        reader.fieldnames = [(h or "").strip() for h in reader.fieldnames or []]
        for row in reader:
            if not row:
                continue
            if use_geo:
                a, b = _float(row.get(c_lat)), _float(row.get(c_lon))
            else:
                a, b = _float(row.get(c_north)), _float(row.get(c_east))
            if a is None or b is None:
                continue
            rows.append({
                "a": a, "b": b,
                "name": (row.get(c_name) or "").strip() if c_name else "",
                "code": (row.get(c_code) or "").strip() if c_code else "",
                "status": (row.get(c_status) or "").strip() if c_status else "",
                "rms": _float(row.get(c_rms)) if c_rms else None,
                "samples": _float(row.get(c_samples)) if c_samples else None,
                "pdop": _float(row.get(c_pdop)) if c_pdop else None,
            })

    if not rows:
        raise ValueError(f"{filepath} contains no rows with usable coordinates")

    warnings: list[str] = []
    cs_name = None
    if c_cs:
        with open(filepath, "r", encoding="utf-8-sig", errors="replace") as f:
            reader = csv.DictReader(f)
            reader.fieldnames = [(h or "").strip() for h in reader.fieldnames or []]
            for row in reader:
                cs_name = (row.get(c_cs) or "").strip() or None
                break

    # --- quality reporting (never silent) ---------------------------------
    if c_status:
        bad = [r for r in rows if r["status"] and r["status"].upper() != "FIX"]
        if bad and require_fix:
            names = ", ".join(r["name"] or "?" for r in bad[:5])
            rows = [r for r in rows if not (r["status"] and r["status"].upper() != "FIX")]
            warnings.append(
                f"dropped {len(bad)} point(s) without an RTK FIX ({names}"
                f"{'…' if len(bad) > 5 else ''})")
            if not rows:
                raise ValueError(f"{filepath}: no points left after the FIX filter")
        elif bad:
            warnings.append(
                f"{len(bad)} of {len(rows)} point(s) are not RTK FIX — imported anyway")
    if c_samples:
        single = [r for r in rows if r["samples"] is not None and r["samples"] <= 1]
        if single:
            warnings.append(
                f"{len(single)} of {len(rows)} point(s) are single-epoch (Samples=1, no "
                f"averaging) — positional noise is at its full per-sample value")
    if c_rms and max_lateral_rms_m is not None:
        noisy = [r for r in rows if r["rms"] is not None and r["rms"] > max_lateral_rms_m]
        if noisy:
            worst = max(r["rms"] for r in noisy)
            warnings.append(
                f"{len(noisy)} point(s) exceed lateral RMS {max_lateral_rms_m:.3f} m "
                f"(worst {worst:.3f} m)")

    # --- coordinates -> local metres --------------------------------------
    geo_origin = None
    if use_geo:
        lat0 = sum(r["a"] for r in rows) / len(rows)
        lon0 = sum(r["b"] for r in rows) / len(rows)
        mdeg_n, mdeg_e = metres_per_degree(lat0)
        for r in rows:
            r["n"] = (r["a"] - lat0) * mdeg_n
            r["e"] = (r["b"] - lon0) * mdeg_e
        geo_origin = (lat0, lon0)
        source = "latlon"
        log.info("Survey CSV: projecting lat/lon about origin (%.8f, %.8f)", lat0, lon0)
    else:
        n0 = sum(r["a"] for r in rows) / len(rows)
        e0 = sum(r["b"] for r in rows) / len(rows)
        for r in rows:
            r["n"] = r["a"] - n0
            r["e"] = r["b"] - e0
        source = "grid"
        warnings.append(
            "no Latitude/Longitude columns — using projected grid coordinates, which "
            "carry the projection's scale factor (typically a few cm per 100 m) and "
            "give no GPS origin for surveyed placement")
        log.info("Survey CSV: grid coordinates, local origin (%.3f, %.3f)", n0, e0)

    # --- group by code, order by name -------------------------------------
    groups: dict[str, list[dict]] = {}
    order: list[str] = []
    for r in rows:
        key = r["code"] or "_"
        if key not in groups:
            groups[key] = []
            order.append(key)
        groups[key].append(r)

    segments: list[PathSegment] = []
    for idx, key in enumerate(order):
        pts_rows = sorted(groups[key], key=lambda r: _order_key(r["name"]))
        pts = [(r["n"], r["e"]) for r in pts_rows]
        if len(pts) < 2:
            warnings.append(
                f"code {key!r} has only {len(pts)} point(s) — kept as a marker, not a line")
        seg = PathSegment(
            segment_type=SegmentType.MARK,
            points=pts,
            speed=mark_speed,
            segment_id=idx,
            source_entity=f"csv:{key}" if key != "_" else f"csv:segment_{idx}",
            metadata={
                "geometry_type": "LINE_CHAIN",
                "line_like": True,
                "survey_code": key if key != "_" else None,
                "survey_names": [r["name"] for r in pts_rows],
                # Every point here came from a surveyed measurement, so all of
                # them are declared control points, not densification fill.
                "control_indices": list(range(len(pts))),
            },
        )
        if geo_origin is not None:
            seg.metadata["geo_origin"] = geo_origin
        segments.append(seg)

    for w in warnings:
        log.warning("Survey CSV %s: %s", filepath, w)

    return SurveyCsvResult(
        segments=segments,
        geo_origin=geo_origin,
        warnings=warnings,
        cs_name=cs_name,
        point_count=len(rows),
        source=source,
    )
