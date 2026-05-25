"""NED CSV parser with backward compatibility.

Supports two formats:

Old format (2 columns):
    north_m,east_m
    0.0,0.0
    1.0,0.0

New format (6 columns):
    north_m,east_m,spray_on,speed_m_s,segment_id,yaw_rad
    0.0,0.0,1,0.35,0,0.0
    1.0,0.0,1,0.35,0,0.0
    1.0,1.0,0,0.50,1,1.5708

Lines starting with # are ignored.
"""

from __future__ import annotations

import csv

from ..core import PathSegment, SegmentType


def read_ned_csv(filepath: str) -> list[tuple[float, float]]:
    """Read simple 2-column CSV (north_m, east_m) with no header.

    Backward compatible: reads only the first two columns.
    Lines starting with # are ignored.

    Returns:
        List of (north_m, east_m) tuples.
    """
    pts: list[tuple[float, float]] = []
    with open(filepath, "r", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            try:
                n = float(row[0].strip())
                e = float(row[1].strip()) if len(row) > 1 else 0.0
                pts.append((n, e))
            except (ValueError, IndexError):
                continue
    return pts


def read_ned_csv_enhanced(filepath: str) -> list[PathSegment]:
    """Read CSV with optional spray_on, speed, segment_id, yaw columns.

    Detects column count from the first data row:
      - 2 columns: old format. Defaults: spray_on=1, speed=0.35, segment_id=0, yaw=0.0
      - 6 columns: new format. All columns read.

    Format is decided from the first non-comment row and enforced for
    all subsequent rows. Column-count mismatch raises ValueError.

    Lines starting with # are ignored.

    Returns:
        List of PathSegment, grouped by segment_id and spray_on.
        Consecutive rows with the same (segment_id, spray_on) pair are
        merged into a single segment.
    """
    rows: list[dict] = []
    expected_cols: int | None = None
    with open(filepath, "r", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)
        for line_no, row in enumerate(reader, 1):
            if not row or row[0].strip().startswith("#"):
                continue
            n_cols = len(row)
            if expected_cols is None:
                expected_cols = n_cols
            elif n_cols != expected_cols:
                raise ValueError(
                    f"CSV column count mismatch at line {line_no}: "
                    f"expected {expected_cols}, got {n_cols}"
                )
            try:
                n = float(row[0].strip())
                e = float(row[1].strip()) if len(row) > 1 else 0.0
                if expected_cols >= 6:
                    spray_on = int(float(row[2].strip()))
                    speed = float(row[3].strip())
                    seg_id = int(float(row[4].strip()))
                    yaw = float(row[5].strip())
                else:
                    # Old format: 2 or 3 columns
                    spray_on = 1
                    speed = 0.35
                    seg_id = 0
                    yaw = 0.0
                rows.append({
                    "n": n, "e": e,
                    "spray_on": spray_on,
                    "speed": speed,
                    "segment_id": seg_id,
                    "yaw": yaw,
                })
            except (ValueError, IndexError):
                continue

    if not rows:
        return []

    # Group consecutive rows with same (segment_id, spray_on) into segments
    segments: list[PathSegment] = []
    current_key = (rows[0]["segment_id"], rows[0]["spray_on"])
    current_points: list[tuple[float, float]] = [(rows[0]["n"], rows[0]["e"])]
    current_speed = rows[0]["speed"] if current_key[1] == 1 else 0.50

    for row in rows[1:]:
        key = (row["segment_id"], row["spray_on"])
        point = (row["n"], row["e"])
        if key == current_key:
            current_points.append(point)
        else:
            seg_type = SegmentType.MARK if current_key[1] == 1 else SegmentType.TRANSIT
            segments.append(PathSegment(
                segment_type=seg_type,
                points=list(current_points),
                speed=current_speed,
                segment_id=current_key[0],
                source_entity=f"csv:segment_{current_key[0]}",
            ))
            current_key = key
            current_points = [point]
            current_speed = row["speed"] if key[1] == 1 else 0.50

    # Flush last segment
    seg_type = SegmentType.MARK if current_key[1] == 1 else SegmentType.TRANSIT
    segments.append(PathSegment(
        segment_type=seg_type,
        points=list(current_points),
        speed=current_speed,
        segment_id=current_key[0],
        source_entity=f"csv:segment_{current_key[0]}",
    ))

    return segments