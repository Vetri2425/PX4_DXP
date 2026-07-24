"""Point-mission coordinate ingest (CSV rows and DXF POINT entities)."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from typing import Any, Iterable

_CSV_HEADER_SCHEMAS: dict[tuple[str, ...], tuple[str, ...]] = {
    ("north", "east"): ("north", "east"),
    ("north", "east", "dwell_s"): ("north", "east", "dwell_s"),
    ("north", "east", "dwell_s", "mark"): ("north", "east", "dwell_s", "mark"),
}

_GPS_CSV_HEADER_SCHEMAS: dict[tuple[str, ...], tuple[str, ...]] = {
    ("lat", "lon"): ("lat", "lon"),
    ("lat", "lon", "dwell_s"): ("lat", "lon", "dwell_s"),
    ("lat", "lon", "mark"): ("lat", "lon", "mark"),
    ("lat", "lon", "dwell_s", "mark"): ("lat", "lon", "dwell_s", "mark"),
}

GPS_SURVEYED_FRAME = "GPS_SURVEYED"


@dataclass(frozen=True)
class SprayPoint:
    north_m: float
    east_m: float
    dwell_s: float | None
    source_index: int
    mark: bool = True


@dataclass(frozen=True)
class GpsPointMissionParseResult:
    anchor_lat: float
    anchor_lon: float
    points: list[SprayPoint]


def _finite_coord(name: str, value: Any) -> float:
    try:
        num = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(num):
        raise ValueError(f"{name} must be finite")
    return num


def _finite_lat(name: str, value: Any) -> float:
    lat = _finite_coord(name, value)
    if not -90.0 <= lat <= 90.0:
        raise ValueError(f"{name} must be within [-90, 90]")
    return lat


def _finite_lon(name: str, value: Any) -> float:
    lon = _finite_coord(name, value)
    if not -180.0 <= lon <= 180.0:
        raise ValueError(f"{name} must be within [-180, 180]")
    return lon


def _latlon_to_ned(
    lat: float,
    lon: float,
    anchor_lat: float,
    anchor_lon: float,
) -> tuple[float, float]:
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from path_engine.ned import latlon_to_ned

    return latlon_to_ned(lat, lon, anchor_lat, anchor_lon)


def _normalize_header_cell(cell: str) -> str:
    return cell.strip().lower()


def _looks_numeric(value: str) -> bool:
    text = value.strip()
    if not text:
        return False
    try:
        float(text)
    except ValueError:
        return False
    return True


def _parse_mark(value: str, line_no: int) -> bool:
    text = value.strip().lower()
    if text in {"true", "1", "yes", "y"}:
        return True
    if text in {"false", "0", "no", "n"}:
        return False
    raise ValueError(
        f"line {line_no}: mark must be a boolean (true/false, 1/0, yes/no); got {value!r}"
    )


def _resolve_header(row: list[str], line_no: int) -> tuple[str, ...] | None:
    cells = tuple(_normalize_header_cell(cell) for cell in row)
    if cells[0] != "north" or cells[1] != "east":
        return None
    if cells not in _CSV_HEADER_SCHEMAS:
        raise ValueError(
            f"line {line_no}: unknown CSV header columns {', '.join(cells)}; "
            "expected north,east[,dwell_s[,mark]]"
        )
    return cells


def _resolve_gps_header(row: list[str], line_no: int) -> tuple[str, ...] | None:
    cells = tuple(_normalize_header_cell(cell) for cell in row)
    if cells[0] != "lat" or cells[1] != "lon":
        return None
    if cells not in _GPS_CSV_HEADER_SCHEMAS:
        raise ValueError(
            f"line {line_no}: unknown CSV header columns {', '.join(cells)}; "
            "expected lat,lon[,dwell_s][,mark]"
        )
    return cells


def _validate_effective_dwell(
    dwell_s: float,
    *,
    mark: bool,
    line_no: int,
    max_dwell_s: float,
) -> None:
    if not math.isfinite(dwell_s):
        raise ValueError(f"line {line_no}: dwell_s must be finite")
    if mark and dwell_s <= 0.0:
        raise ValueError(f"line {line_no}: dwell_s must be > 0 when mark=true")
    if not mark and dwell_s < 0.0:
        raise ValueError(f"line {line_no}: dwell_s must be >= 0 when mark=false")
    if dwell_s > max_dwell_s:
        raise ValueError(
            f"line {line_no}: dwell_s {dwell_s} exceeds maximum {max_dwell_s}"
        )


def validate_point_dwells(
    points: list[SprayPoint],
    *,
    default_dwell_s: float,
    max_dwell_s: float,
) -> None:
    """Validate resolved dwell durations against mark-aware policy."""
    if default_dwell_s <= 0.0 or not math.isfinite(default_dwell_s):
        raise ValueError("default_dwell_s must be finite and > 0")
    if max_dwell_s <= 0.0 or not math.isfinite(max_dwell_s):
        raise ValueError("max_dwell_s must be finite and > 0")
    if default_dwell_s > max_dwell_s:
        raise ValueError("default_dwell_s exceeds max_dwell_s")

    for pt in points:
        effective = pt.dwell_s if pt.dwell_s is not None else default_dwell_s
        _validate_effective_dwell(
            effective,
            mark=pt.mark,
            line_no=pt.source_index,
            max_dwell_s=max_dwell_s,
        )


def _parse_csv_row(
    row: list[str],
    line_no: int,
    *,
    columns: tuple[str, ...],
) -> SprayPoint:
    if not row or row[0].strip().startswith("#"):
        raise ValueError("empty row")
    if len(row) < len(columns):
        raise ValueError(
            f"line {line_no}: expected {len(columns)} column(s) "
            f"({', '.join(columns)}), got {len(row)}"
        )
    if len(row) > len(columns):
        raise ValueError(
            f"line {line_no}: unknown extra column(s); expected {', '.join(columns)}"
        )

    north = _finite_coord("north", row[0].strip())
    east = _finite_coord("east", row[1].strip())
    dwell: float | None = None
    mark = True

    if "dwell_s" in columns:
        dwell_text = row[columns.index("dwell_s")].strip()
        if dwell_text:
            dwell = _finite_coord("dwell_s", dwell_text)
    if "mark" in columns:
        mark = _parse_mark(row[columns.index("mark")], line_no)

    return SprayPoint(
        north_m=north,
        east_m=east,
        dwell_s=dwell,
        source_index=line_no,
        mark=mark,
    )


@dataclass(frozen=True)
class _GpsCsvRow:
    lat: float
    lon: float
    dwell_s: float | None
    mark: bool
    source_index: int


def _parse_gps_csv_row(
    row: list[str],
    line_no: int,
    *,
    columns: tuple[str, ...],
) -> _GpsCsvRow:
    if not row or row[0].strip().startswith("#"):
        raise ValueError("empty row")
    if len(row) < len(columns):
        raise ValueError(
            f"line {line_no}: expected {len(columns)} column(s) "
            f"({', '.join(columns)}), got {len(row)}"
        )
    if len(row) > len(columns):
        raise ValueError(
            f"line {line_no}: unknown extra column(s); expected {', '.join(columns)}"
        )

    lat = _finite_lat("lat", row[0].strip())
    lon = _finite_lon("lon", row[1].strip())
    dwell: float | None = None
    mark = True

    if "dwell_s" in columns:
        dwell_text = row[columns.index("dwell_s")].strip()
        if dwell_text:
            dwell = _finite_coord("dwell_s", dwell_text)
            if dwell < 0.0:
                raise ValueError(f"line {line_no}: dwell_s must be >= 0")
    if "mark" in columns:
        mark = _parse_mark(row[columns.index("mark")], line_no)

    return _GpsCsvRow(
        lat=lat,
        lon=lon,
        dwell_s=dwell,
        mark=mark,
        source_index=line_no,
    )


def _gps_rows_to_spray_points(
    rows: list[_GpsCsvRow],
    *,
    default_dwell_s: float,
    max_dwell_s: float,
    duplicate_tolerance_m: float,
) -> GpsPointMissionParseResult:
    if not rows:
        raise ValueError("point mission must contain at least one point")

    anchor_lat = rows[0].lat
    anchor_lon = rows[0].lon
    points: list[SprayPoint] = []
    for row in rows:
        north_m, east_m = _latlon_to_ned(row.lat, row.lon, anchor_lat, anchor_lon)
        points.append(
            SprayPoint(
                north_m=north_m,
                east_m=east_m,
                dwell_s=row.dwell_s,
                source_index=row.source_index,
                mark=row.mark,
            )
        )

    return GpsPointMissionParseResult(
        anchor_lat=anchor_lat,
        anchor_lon=anchor_lon,
        points=_finalize_points(
            points,
            default_dwell_s=default_dwell_s,
            max_dwell_s=max_dwell_s,
            duplicate_tolerance_m=duplicate_tolerance_m,
        ),
    )


def _survey_latlon_rows(text: str):
    """Named-header survey export (Emlid/Trimble/…) → list[_GpsCsvRow], or None.

    Reuses path_engine's survey-CSV column detection (Name/Code/Latitude/
    Longitude + vendor aliases) so a real 42-column field export becomes a point
    mission — not only the bare `lat,lon` file. Every surveyed point is a mark;
    order is the survey Name order; dwell defaults are applied downstream.
    """
    import sys
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from path_engine.parsers.survey_csv import read_survey_latlon_points

    pts = read_survey_latlon_points(text)
    if not pts:
        return None
    return [
        _GpsCsvRow(lat=p["lat"], lon=p["lon"], dwell_s=None, mark=True, source_index=i + 1)
        for i, p in enumerate(pts)
    ]


def parse_point_gps_csv_text(
    text: str,
    *,
    default_dwell_s: float = 2.0,
    max_dwell_s: float = 60.0,
    duplicate_tolerance_m: float = 1e-3,
) -> GpsPointMissionParseResult:
    """Parse a GPS point-mission CSV into anchor-relative NED spray points.

    Two shapes are accepted, tried in order:
      1. Bare header ``lat,lon[,dwell_s][,mark]`` (the app's simple export).
      2. A named-header **survey export** (Emlid/Trimble/Leica —
         ``Name,Code,…,Longitude,Latitude,…``), via path_engine's survey-CSV
         column detection. Every surveyed point becomes a mark, in Name order.

    The first data row (1) or the first Name-ordered point (2) is the survey
    anchor; every point is projected to anchor-relative NED (Karney geodesic).
    """
    if default_dwell_s <= 0.0 or not math.isfinite(default_dwell_s):
        raise ValueError("default_dwell_s must be finite and > 0")
    if max_dwell_s <= 0.0 or not math.isfinite(max_dwell_s):
        raise ValueError("max_dwell_s must be finite and > 0")
    try:
        return _parse_bare_gps_csv_text(
            text,
            default_dwell_s=default_dwell_s,
            max_dwell_s=max_dwell_s,
            duplicate_tolerance_m=duplicate_tolerance_m,
        )
    except ValueError as bare_err:
        rows = _survey_latlon_rows(text)
        if rows:
            return _gps_rows_to_spray_points(
                rows,
                default_dwell_s=default_dwell_s,
                max_dwell_s=max_dwell_s,
                duplicate_tolerance_m=duplicate_tolerance_m,
            )
        raise bare_err


def _parse_bare_gps_csv_text(
    text: str,
    *,
    default_dwell_s: float = 2.0,
    max_dwell_s: float = 60.0,
    duplicate_tolerance_m: float = 1e-3,
) -> GpsPointMissionParseResult:
    """Parse the bare header ``lat,lon[,dwell_s][,mark]`` (first row = anchor)."""
    if default_dwell_s <= 0.0 or not math.isfinite(default_dwell_s):
        raise ValueError("default_dwell_s must be finite and > 0")
    if max_dwell_s <= 0.0 or not math.isfinite(max_dwell_s):
        raise ValueError("max_dwell_s must be finite and > 0")

    rows = list(csv.reader(text.splitlines()))
    data_rows: list[tuple[int, list[str]]] = []
    columns: tuple[str, ...] | None = None

    for line_no, row in enumerate(rows, 1):
        if not row or row[0].strip().startswith("#"):
            continue
        if columns is None:
            header = _resolve_gps_header(row, line_no)
            if header is None:
                raise ValueError(
                    f"line {line_no}: expected lat,lon CSV header "
                    "(lat,lon[,dwell_s][,mark])"
                )
            columns = header
            continue
        data_rows.append((line_no, row))

    if columns is None:
        raise ValueError("point mission must contain at least one point")

    gps_rows: list[_GpsCsvRow] = []
    for line_no, row in data_rows:
        try:
            gps_rows.append(_parse_gps_csv_row(row, line_no, columns=columns))
        except ValueError as exc:
            raise ValueError(str(exc)) from exc

    return _gps_rows_to_spray_points(
        gps_rows,
        default_dwell_s=default_dwell_s,
        max_dwell_s=max_dwell_s,
        duplicate_tolerance_m=duplicate_tolerance_m,
    )


def parse_point_gps_csv_file(
    filepath: str,
    *,
    default_dwell_s: float = 2.0,
    max_dwell_s: float = 60.0,
    duplicate_tolerance_m: float = 1e-3,
) -> GpsPointMissionParseResult:
    with open(filepath, "r", encoding="utf-8", errors="replace") as handle:
        return parse_point_gps_csv_text(
            handle.read(),
            default_dwell_s=default_dwell_s,
            max_dwell_s=max_dwell_s,
            duplicate_tolerance_m=duplicate_tolerance_m,
        )


def gps_point_mission_parse_payload(result: GpsPointMissionParseResult) -> dict[str, Any]:
    return {
        "num_points": len(result.points),
        "anchor": {"lat": result.anchor_lat, "lon": result.anchor_lon},
        "point_source_frame": GPS_SURVEYED_FRAME,
        "point_mission_points": points_to_staged_dict(result.points),
    }


def parse_point_csv_text(
    text: str,
    *,
    default_dwell_s: float = 2.0,
    max_dwell_s: float = 60.0,
    duplicate_tolerance_m: float = 1e-3,
) -> list[SprayPoint]:
    """Parse CSV rows with optional header: north,east[,dwell_s[,mark]]."""
    if default_dwell_s <= 0.0 or not math.isfinite(default_dwell_s):
        raise ValueError("default_dwell_s must be finite and > 0")
    if max_dwell_s <= 0.0 or not math.isfinite(max_dwell_s):
        raise ValueError("max_dwell_s must be finite and > 0")

    rows = list(csv.reader(text.splitlines()))
    data_rows: list[tuple[int, list[str]]] = []
    columns: tuple[str, ...] = ("north", "east")
    schema_locked = False

    for line_no, row in enumerate(rows, 1):
        if not row or row[0].strip().startswith("#"):
            continue
        if not schema_locked:
            header = _resolve_header(row, line_no)
            if header is not None:
                columns = header
                schema_locked = True
                continue
            if not _looks_numeric(row[0]):
                raise ValueError(
                    f"line {line_no}: expected numeric north coordinate or CSV header"
                )
            if len(row) > 3:
                raise ValueError(
                    f"line {line_no}: too many columns for headerless CSV "
                    "(expected north,east[,dwell_s]; use a header row for mark)"
                )
            if len(row) == 3:
                columns = ("north", "east", "dwell_s")
            else:
                columns = ("north", "east")
            schema_locked = True
        data_rows.append((line_no, row))

    points: list[SprayPoint] = []
    for line_no, row in data_rows:
        try:
            points.append(_parse_csv_row(row, line_no, columns=columns))
        except ValueError as exc:
            raise ValueError(str(exc)) from exc

    return _finalize_points(
        points,
        default_dwell_s=default_dwell_s,
        max_dwell_s=max_dwell_s,
        duplicate_tolerance_m=duplicate_tolerance_m,
    )


def parse_point_csv_file(
    filepath: str,
    *,
    default_dwell_s: float = 2.0,
    max_dwell_s: float = 60.0,
    duplicate_tolerance_m: float = 1e-3,
) -> list[SprayPoint]:
    with open(filepath, "r", encoding="utf-8", errors="replace") as handle:
        return parse_point_csv_text(
            handle.read(),
            default_dwell_s=default_dwell_s,
            max_dwell_s=max_dwell_s,
            duplicate_tolerance_m=duplicate_tolerance_m,
        )


def parse_dxf_point_entities(
    entities: Iterable[Any],
    *,
    default_dwell_s: float = 2.0,
    max_dwell_s: float = 60.0,
    duplicate_tolerance_m: float = 1e-3,
) -> list[SprayPoint]:
    """Extract POINT entities from path_engine DXFEntity objects."""
    points: list[SprayPoint] = []
    for idx, ent in enumerate(entities):
        if getattr(ent, "entity_type", "") != "POINT":
            continue
        geom = getattr(ent, "geometry", {}) or {}
        pos = geom.get("position")
        if pos is None:
            raise ValueError(f"POINT entity {getattr(ent, 'entity_id', idx)} missing position")
        north = _finite_coord("north", pos[0])
        east = _finite_coord("east", pos[1] if len(pos) > 1 else 0.0)
        points.append(
            SprayPoint(
                north_m=north,
                east_m=east,
                dwell_s=None,
                source_index=idx,
                mark=True,
            )
        )
    return _finalize_points(
        points,
        default_dwell_s=default_dwell_s,
        max_dwell_s=max_dwell_s,
        duplicate_tolerance_m=duplicate_tolerance_m,
    )


def _finalize_points(
    points: list[SprayPoint],
    default_dwell_s: float,
    max_dwell_s: float,
    duplicate_tolerance_m: float,
) -> list[SprayPoint]:
    if not points:
        raise ValueError("point mission must contain at least one point")

    seen: list[tuple[float, float]] = []
    finalized: list[SprayPoint] = []
    for pt in points:
        for prev_n, prev_e in seen:
            if (
                math.hypot(pt.north_m - prev_n, pt.east_m - prev_e)
                <= duplicate_tolerance_m
            ):
                raise ValueError(
                    f"duplicate point at ({pt.north_m:.4f}, {pt.east_m:.4f})"
                )
        seen.append((pt.north_m, pt.east_m))

        effective_dwell = pt.dwell_s if pt.dwell_s is not None else default_dwell_s
        _validate_effective_dwell(
            effective_dwell,
            mark=pt.mark,
            line_no=pt.source_index,
            max_dwell_s=max_dwell_s,
        )
        finalized.append(
            SprayPoint(
                north_m=pt.north_m,
                east_m=pt.east_m,
                dwell_s=effective_dwell,
                source_index=pt.source_index,
                mark=pt.mark,
            )
        )

    return finalized


def points_to_staged_dict(points: list[SprayPoint]) -> list[dict[str, Any]]:
    return [
        {
            "north_m": pt.north_m,
            "east_m": pt.east_m,
            "dwell_s": pt.dwell_s,
            "source_index": pt.source_index,
            "mark": pt.mark,
        }
        for pt in points
    ]


def points_from_staged_dict(
    rows: list[dict[str, Any]],
    *,
    default_dwell_s: float = 2.0,
    max_dwell_s: float = 60.0,
) -> list[SprayPoint]:
    points: list[SprayPoint] = []
    for idx, row in enumerate(rows):
        # Preserve original CSV/DXF provenance so downstream validation errors
        # reference the true source line, not the staged-list ordinal. Fall back
        # to a 1-based ordinal (matching CSV line numbering) when absent.
        if row.get("source_index") is not None:
            source_index = int(row["source_index"])
        else:
            source_index = idx + 1
        mark_raw = row.get("mark", True)
        if isinstance(mark_raw, bool):
            mark = mark_raw
        elif isinstance(mark_raw, str):
            mark = _parse_mark(mark_raw, source_index)
        else:
            raise ValueError(f"row {source_index}: mark must be a boolean")

        points.append(
            SprayPoint(
                north_m=_finite_coord("north_m", row.get("north_m")),
                east_m=_finite_coord("east_m", row.get("east_m")),
                dwell_s=(
                    _finite_coord("dwell_s", row["dwell_s"])
                    if row.get("dwell_s") is not None
                    else None
                ),
                source_index=source_index,
                mark=mark,
            )
        )
    return _finalize_points(
        points,
        default_dwell_s=default_dwell_s,
        max_dwell_s=max_dwell_s,
        duplicate_tolerance_m=1e-3,
    )