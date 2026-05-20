"""Path loading, generation, and uploaded-file management.

Hardcoded path generators mirror `path_publisher_node.py` so the server
produces identical waypoint sets without importing from the src/ ROS
package.
"""
from __future__ import annotations

import csv
import math
import os
from functools import lru_cache
from typing import Optional

from config import ALLOWED_UPLOAD_EXTENSIONS, MAX_UPLOAD_BYTES
from logging_setup import get_logger
from models import PathInfo

log = get_logger("server.path")


# ── Hardcoded path generators (mirror path_publisher_node.py) ─────────────────

def gen_straight_5m(spacing: float = 0.5) -> list[tuple[float, float]]:
    return [(i * spacing, 0.0) for i in range(int(5.0 / spacing) + 1)]


def gen_arc_quarter_1m5(
    radius: float = 1.5, arc_spacing: float = 0.1
) -> list[tuple[float, float]]:
    arc_len = radius * (math.pi / 2.0)
    n = max(2, int(arc_len / arc_spacing) + 1)
    return [
        (radius * math.sin((math.pi / 2.0) * i / (n - 1)),
         radius * (1.0 - math.cos((math.pi / 2.0) * i / (n - 1))))
        for i in range(n)
    ]


def gen_lshape_2x2(spacing: float = 0.25) -> list[tuple[float, float]]:
    pts = [(i * spacing, 0.0) for i in range(int(2.0 / spacing) + 1)]
    pts += [(2.0, i * spacing) for i in range(1, int(2.0 / spacing) + 1)]
    return pts


def gen_square_2x2(spacing: float = 0.25) -> list[tuple[float, float]]:
    side = 2.0
    steps = int(side / spacing)
    pts = [(i * spacing, 0.0) for i in range(steps + 1)]
    pts += [(side, i * spacing)        for i in range(1, steps + 1)]
    pts += [(side - i * spacing, side) for i in range(1, steps + 1)]
    pts += [(0.0, side - i * spacing)  for i in range(1, steps + 1)]
    return pts


def gen_rectangle_3x2(spacing: float = 0.25) -> list[tuple[float, float]]:
    ln, le = 3.0, 2.0
    pts  = [(i * spacing, 0.0) for i in range(int(ln / spacing) + 1)]
    pts += [(ln, i * spacing)       for i in range(1, int(le / spacing) + 1)]
    pts += [(ln - i * spacing, le)  for i in range(1, int(ln / spacing) + 1)]
    pts += [(0.0, le - i * spacing) for i in range(1, int(le / spacing) + 1)]
    return pts


def gen_circle_1m5(
    radius: float = 1.5, arc_spacing: float = 0.1
) -> list[tuple[float, float]]:
    n = max(4, int(radius * 2 * math.pi / arc_spacing) + 1)
    pts = [
        (radius * math.sin(2 * math.pi * i / n),
         radius * (1.0 - math.cos(2 * math.pi * i / n)))
        for i in range(n)
    ]
    pts.append((0.0, 0.0))
    return pts


BUILTIN_PATHS: dict[str, dict] = {
    "straight_5m":     {"gen": gen_straight_5m,      "desc": "5 m straight north, 50 cm spacing"},
    "arc_quarter_1m5": {"gen": gen_arc_quarter_1m5,  "desc": "Quarter circle, R=1.5 m, north then east"},
    "lshape_2x2":      {"gen": gen_lshape_2x2,       "desc": "2 m north then 2 m east, 25 cm spacing"},
    "square_2x2":      {"gen": gen_square_2x2,       "desc": "2 m × 2 m closed square, 25 cm spacing"},
    "rectangle_3x2":   {"gen": gen_rectangle_3x2,    "desc": "3 m north × 2 m east rectangle"},
    "circle_1m5":      {"gen": gen_circle_1m5,       "desc": "Full circle, R=1.5 m, closed loop"},
}


@lru_cache(maxsize=None)
def _cached_builtin(name: str) -> tuple[tuple[float, float], ...]:
    """Cache builtin generation across calls to list_paths()."""
    return tuple(BUILTIN_PATHS[name]["gen"]())


# ── File readers ──────────────────────────────────────────────────────────────

def read_qgc_waypoints(filepath: str) -> list[tuple[float, float]]:
    """QGC WPL 110 → NED metres. Home waypoint (current=1) is the origin."""
    try:
        from geographiclib.geodesic import Geodesic
    except ImportError:
        raise ImportError(
            "geographiclib required for .waypoints files. "
            "Install: pip install geographiclib"
        )
    geod = Geodesic.WGS84
    wps: list[tuple[float, float]] = []
    home_lat = home_lon = None

    with open(filepath, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("QGC"):
                continue
            fields = line.split("\t")
            if len(fields) < 11:
                continue
            try:
                current = int(fields[1])
                lat     = float(fields[8])
                lon     = float(fields[9])
            except (ValueError, IndexError):
                continue
            if current == 1:
                home_lat, home_lon = lat, lon
            else:
                wps.append((lat, lon))

    if home_lat is None:
        if wps:
            home_lat, home_lon = wps[0]
            wps = wps[1:]
        else:
            raise ValueError(f"No waypoints in {filepath}")

    pts: list[tuple[float, float]] = []
    for lat, lon in wps:
        r = geod.Inverse(home_lat, home_lon, lat, lon)
        bearing = math.radians(r["azi1"])
        pts.append((r["s12"] * math.cos(bearing), r["s12"] * math.sin(bearing)))
    return pts


def read_ned_csv(filepath: str) -> list[tuple[float, float]]:
    """Simple CSV: north_m,east_m  (no header; '#' = comment)."""
    pts: list[tuple[float, float]] = []
    with open(filepath, encoding="utf-8", errors="replace") as f:
        for row in csv.reader(f):
            if not row or row[0].strip().startswith("#"):
                continue
            try:
                n = float(row[0].strip())
                e = float(row[1].strip()) if len(row) > 1 else 0.0
                pts.append((n, e))
            except ValueError:
                continue
    return pts


# ── Validation helpers (for routes/path.py) ──────────────────────────────────

class UploadValidationError(Exception):
    """Raised when an upload violates size or type constraints."""


def validate_upload(filename: str, content: bytes) -> str:
    """Validates extension and size. Returns sanitised filename."""
    safe = os.path.basename(filename or "")
    if not safe:
        raise UploadValidationError("empty filename")
    ext = os.path.splitext(safe)[1].lower()
    if ext not in ALLOWED_UPLOAD_EXTENSIONS:
        allowed = ", ".join(sorted(ALLOWED_UPLOAD_EXTENSIONS))
        raise UploadValidationError(f"extension {ext!r} not allowed (need {allowed})")
    if len(content) > MAX_UPLOAD_BYTES:
        raise UploadValidationError(
            f"file too large: {len(content)} > {MAX_UPLOAD_BYTES} bytes"
        )
    return safe


# ── PathManager ───────────────────────────────────────────────────────────────

class PathManager:
    def __init__(self, missions_dir: str) -> None:
        self._dir = missions_dir
        os.makedirs(missions_dir, exist_ok=True)

    def list_paths(self) -> list[PathInfo]:
        result: list[PathInfo] = []
        for name, info in BUILTIN_PATHS.items():
            pts = _cached_builtin(name)
            result.append(PathInfo(
                name=name, description=info["desc"],
                num_points=len(pts), source="builtin",
            ))
        for fname in sorted(os.listdir(self._dir)):
            fpath = os.path.join(self._dir, fname)
            if not os.path.isfile(fpath) or fname.startswith("."):
                continue
            try:
                pts = self._load_file(fpath)
                result.append(PathInfo(
                    name=fname,
                    description=f"Uploaded: {fname}",
                    num_points=len(pts),
                    source="file",
                ))
            except Exception as exc:
                log.warning("skipping uploaded file %s: %s", fname, exc)
                continue
        return result

    def load_path(self, name: str) -> list[tuple[float, float]]:
        if name in BUILTIN_PATHS:
            return list(_cached_builtin(name))
        fpath = os.path.join(self._dir, os.path.basename(name))
        if os.path.isfile(fpath):
            return self._load_file(fpath)
        raise FileNotFoundError(f"Path not found: {name!r}")

    def save_uploaded(self, filename: str, content: bytes) -> str:
        """Save raw bytes to missions dir. Validates extension + size."""
        safe = validate_upload(filename, content)
        fpath = os.path.join(self._dir, safe)
        with open(fpath, "wb") as f:
            f.write(content)
        log.info("uploaded mission file: %s (%d bytes)", safe, len(content))
        return safe

    def delete_file(self, filename: str) -> bool:
        fpath = os.path.join(self._dir, os.path.basename(filename))
        if os.path.isfile(fpath):
            os.remove(fpath)
            log.info("deleted mission file: %s", filename)
            return True
        return False

    # ── Internal ──────────────────────────────────────────────────────────────

    def _load_file(self, fpath: str) -> list[tuple[float, float]]:
        ext = os.path.splitext(fpath)[1].lower()
        if ext == ".waypoints":
            return read_qgc_waypoints(fpath)
        if ext == ".csv":
            return read_ned_csv(fpath)
        try:
            return read_qgc_waypoints(fpath)
        except Exception:
            return read_ned_csv(fpath)
