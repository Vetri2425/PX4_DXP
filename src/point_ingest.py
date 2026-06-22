"""Point-mission coordinate ingest (CSV rows and DXF POINT entities)."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from typing import Any, Iterable


@dataclass(frozen=True)
class SprayPoint:
    north_m: float
    east_m: float
    dwell_s: float | None
    source_index: int


def _finite_coord(name: str, value: Any) -> float:
    try:
        num = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(num):
        raise ValueError(f"{name} must be finite")
    return num


def _parse_csv_row(
    row: list[str],
    line_no: int,
    default_dwell_s: float,
) -> SprayPoint:
    if not row or row[0].strip().startswith("#"):
        raise ValueError("empty row")
    if len(row) < 2:
        raise ValueError(f"line {line_no}: expected north,east[,dwell_s]")
    north = _finite_coord("north", row[0].strip())
    east = _finite_coord("east", row[1].strip())
    dwell: float | None = None
    if len(row) >= 3 and row[2].strip():
        dwell = _finite_coord("dwell_s", row[2].strip())
        if dwell <= 0.0:
            raise ValueError(f"line {line_no}: dwell_s must be > 0")
    return SprayPoint(
        north_m=north,
        east_m=east,
        dwell_s=dwell,
        source_index=line_no,
    )


def parse_point_csv_text(
    text: str,
    *,
    default_dwell_s: float = 2.0,
    duplicate_tolerance_m: float = 1e-3,
) -> list[SprayPoint]:
    """Parse CSV rows: north,east or north,east,dwell_s."""
    if default_dwell_s <= 0.0 or not math.isfinite(default_dwell_s):
        raise ValueError("default_dwell_s must be finite and > 0")

    points: list[SprayPoint] = []
    reader = csv.reader(text.splitlines())
    for line_no, row in enumerate(reader, 1):
        if not row or row[0].strip().startswith("#"):
            continue
        try:
            points.append(_parse_csv_row(row, line_no, default_dwell_s))
        except ValueError as exc:
            raise ValueError(str(exc)) from exc

    return _finalize_points(points, default_dwell_s, duplicate_tolerance_m)


def parse_point_csv_file(
    filepath: str,
    *,
    default_dwell_s: float = 2.0,
    duplicate_tolerance_m: float = 1e-3,
) -> list[SprayPoint]:
    with open(filepath, "r", encoding="utf-8", errors="replace") as handle:
        return parse_point_csv_text(
            handle.read(),
            default_dwell_s=default_dwell_s,
            duplicate_tolerance_m=duplicate_tolerance_m,
        )


def parse_dxf_point_entities(
    entities: Iterable[Any],
    *,
    default_dwell_s: float = 2.0,
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
            )
        )
    return _finalize_points(points, default_dwell_s, duplicate_tolerance_m)


def _finalize_points(
    points: list[SprayPoint],
    default_dwell_s: float,
    duplicate_tolerance_m: float,
) -> list[SprayPoint]:
    if not points:
        raise ValueError("point mission must contain at least one point")

    seen: list[tuple[float, float]] = []
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

    return [
        SprayPoint(
            north_m=pt.north_m,
            east_m=pt.east_m,
            dwell_s=pt.dwell_s if pt.dwell_s is not None else default_dwell_s,
            source_index=pt.source_index,
        )
        for pt in points
    ]


def points_to_staged_dict(points: list[SprayPoint]) -> list[dict[str, Any]]:
    return [
        {
            "north_m": pt.north_m,
            "east_m": pt.east_m,
            "dwell_s": pt.dwell_s,
            "source_index": pt.source_index,
        }
        for pt in points
    ]


def points_from_staged_dict(rows: list[dict[str, Any]]) -> list[SprayPoint]:
    points: list[SprayPoint] = []
    for idx, row in enumerate(rows):
        points.append(
            SprayPoint(
                north_m=_finite_coord("north_m", row.get("north_m")),
                east_m=_finite_coord("east_m", row.get("east_m")),
                dwell_s=(
                    _finite_coord("dwell_s", row["dwell_s"])
                    if row.get("dwell_s") is not None
                    else None
                ),
                source_index=int(row.get("source_index", idx)),
            )
        )
    return _finalize_points(points, default_dwell_s=2.0, duplicate_tolerance_m=1e-3)