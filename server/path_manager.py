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

def gen_straight_5m(spacing: float = 0.1) -> list[tuple[float, float]]:
    return [(i * spacing, 0.0) for i in range(int(5.0 / spacing) + 1)]


def gen_arc_quarter_1m5(
    radius: float = 1.5, arc_spacing: float = 0.05
) -> list[tuple[float, float]]:
    arc_len = radius * (math.pi / 2.0)
    n = max(2, int(arc_len / arc_spacing) + 1)
    return [
        (radius * math.sin((math.pi / 2.0) * i / (n - 1)),
         radius * (1.0 - math.cos((math.pi / 2.0) * i / (n - 1))))
        for i in range(n)
    ]


def gen_arc_half_1m5(
    radius: float = 1.5, arc_spacing: float = 0.05
) -> list[tuple[float, float]]:
    arc_len = radius * math.pi
    n = max(2, int(arc_len / arc_spacing) + 1)
    return [
        (radius * math.sin(math.pi * i / (n - 1)),
         radius * (1.0 - math.cos(math.pi * i / (n - 1))))
        for i in range(n)
    ]


def gen_lshape_2x2(spacing: float = 0.15) -> list[tuple[float, float]]:
    pts = [(i * spacing, 0.0) for i in range(int(2.0 / spacing) + 1)]
    pts += [(2.0, i * spacing) for i in range(1, int(2.0 / spacing) + 1)]
    return pts


def gen_square_2x2(spacing: float = 0.15) -> list[tuple[float, float]]:
    side = 2.0
    steps = int(side / spacing)
    pts = [(i * spacing, 0.0) for i in range(steps + 1)]
    pts += [(side, i * spacing)        for i in range(1, steps + 1)]
    pts += [(side - i * spacing, side) for i in range(1, steps + 1)]
    pts += [(0.0, side - i * spacing)  for i in range(1, steps + 1)]
    return pts


def gen_rectangle_3x2(spacing: float = 0.15) -> list[tuple[float, float]]:
    ln, le = 3.0, 2.0
    pts  = [(i * spacing, 0.0) for i in range(int(ln / spacing) + 1)]
    pts += [(ln, i * spacing)       for i in range(1, int(le / spacing) + 1)]
    pts += [(ln - i * spacing, le)  for i in range(1, int(ln / spacing) + 1)]
    pts += [(0.0, le - i * spacing) for i in range(1, int(le / spacing) + 1)]
    return pts


def gen_circle_1m5(
    radius: float = 1.5, arc_spacing: float = 0.05
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
    "straight_5m":     {"gen": gen_straight_5m,      "desc": "5 m straight north, 10 cm spacing"},
    "arc_quarter_1m5": {"gen": gen_arc_quarter_1m5,  "desc": "Quarter circle, R=1.5 m, 5 cm arc spacing, north then east"},
    "arc_half_1m5":    {"gen": gen_arc_half_1m5,     "desc": "Half circle, R=1.5 m, 5 cm arc spacing, north then east"},
    "lshape_2x2":      {"gen": gen_lshape_2x2,       "desc": "2 m north then 2 m east, 15 cm spacing"},
    "square_2x2":      {"gen": gen_square_2x2,       "desc": "2 m × 2 m closed square, 15 cm spacing"},
    "rectangle_3x2":   {"gen": gen_rectangle_3x2,    "desc": "3 m north × 2 m east rectangle, 15 cm spacing"},
    "circle_1m5":      {"gen": gen_circle_1m5,       "desc": "Full circle, R=1.5 m, 5 cm arc spacing, closed loop"},
}


@lru_cache(maxsize=None)
def _cached_builtin(name: str) -> tuple[tuple[float, float], ...]:
    """Cache builtin generation across calls to list_paths()."""
    return tuple(BUILTIN_PATHS[name]["gen"]())


def _path_length(points: list[tuple[float, float]]) -> float:
    return sum(
        math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
        for i in range(1, len(points))
    )


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

    def load_path(
        self,
        name: str,
        origin: tuple[float, float] = (0.0, 0.0),
        start_position: tuple[float, float] | None = None,
    ) -> list[tuple[float, float]]:
        if name.startswith("builtin:"):
            name = name.removeprefix("builtin:")
        if name in BUILTIN_PATHS:
            pts = list(_cached_builtin(name))
            if origin != (0.0, 0.0):
                return [(n + origin[0], e + origin[1]) for n, e in pts]
            return pts
        fpath = os.path.join(self._dir, os.path.basename(name))
        if os.path.isfile(fpath):
            ext = os.path.splitext(fpath)[1].lower()
            if ext in (".dxf", ".csv") and (
                origin != (0.0, 0.0) or start_position is not None
            ):
                from path_engine import PathEngine
                engine = PathEngine()
                plan = engine.plan_file(
                    fpath,
                    origin=origin,
                    start_position=start_position,
                )
                return plan.merged_waypoints
            if origin != (0.0, 0.0):
                pts = self._load_file(fpath)
                return [(n + origin[0], e + origin[1]) for n, e in pts]
            return self._load_file(fpath)
        raise FileNotFoundError(f"Path not found: {name!r}")

    def save_uploaded(self, filename: str, content: bytes) -> str:
        """Save raw bytes to missions dir. Validates extension + size + disk quota."""
        safe = validate_upload(filename, content)
        # Check aggregate disk quota (200 MB)
        total_bytes = sum(
            os.path.getsize(os.path.join(self._dir, f))
            for f in os.listdir(self._dir)
            if os.path.isfile(os.path.join(self._dir, f))
        ) if os.path.isdir(self._dir) else 0
        quota = 200 * 1024 * 1024
        if total_bytes + len(content) > quota:
            raise UploadValidationError(
                f"disk quota exceeded: {total_bytes + len(content)} > {quota} bytes"
            )
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

    def parse_dxf(self, filepath: str, unit_scale: float | None = None,
                   layer_mapping: dict[str, str] | None = None) -> list:
        """Parse a DXF file and return DXFEntity list via path_engine.

        Args:
            filepath: Path to .dxf file (may be in missions dir or absolute).
            unit_scale: Metres per DXF unit (None = auto-detect from $INSUNITS).
            layer_mapping: Dict mapping layer patterns to "mark"/"transit"/"ignore".

        Returns:
            List of DXFEntity objects.
        """
        from path_engine.parsers.dxf_parser import parse_dxf
        return parse_dxf(filepath, unit_scale=unit_scale, layer_mapping=layer_mapping)

    def plan_path(self, name: str, summary_only: bool = False, **kwargs) -> dict:
        """Run the full planning pipeline on a file and return PlannedPath info.

        Args:
            name: Filename in missions dir or builtin path name. Builtins may
                  optionally be prefixed with "builtin:".
            summary_only: If True, return only counts/lengths (no waypoints).
            **kwargs: Passed to PathEngine.plan_file().

        Returns:
            Dict with waypoints, segments, and metadata.
        """
        source_name = name
        if name.startswith("builtin:"):
            name = name.removeprefix("builtin:")

        origin = kwargs.pop("origin", (0.0, 0.0))
        start_position = kwargs.pop("start_position", None)
        layer_mapping = kwargs.pop("layer_mapping", None)
        optimize = kwargs.pop("optimize", True)
        compensate_spray = kwargs.pop("compensate_spray", True)
        line_spacing = kwargs.pop("line_spacing", 0.05)
        transit_spacing = kwargs.pop("transit_spacing", 0.15)
        marking_speed = kwargs.pop("marking_speed", 0.35)
        transit_speed = kwargs.pop("transit_speed", 0.50)
        origin_gps = kwargs.pop("origin_gps", None)
        rotation_deg = kwargs.pop("rotation_deg", 0.0)
        ref_points_dxf = kwargs.pop("ref_points_dxf", None)
        ref_points_gps = kwargs.pop("ref_points_gps", None)
        close_loop = kwargs.pop("close_loop", False)

        if name in BUILTIN_PATHS:
            # Builtin preview must match the path that mission/load publishes:
            # these generators are already densified to their tuned spacing.
            pts = list(_cached_builtin(name))
            shifted = [(n + origin[0], e + origin[1]) for n, e in pts]
            mark_length = _path_length(pts)
            result = {
                "source": source_name,
                "num_waypoints": len(shifted),
                "num_segments": 1 if shifted else 0,
                "mark_length_m": round(mark_length, 3),
                "transit_length_m": 0.0,
                "total_length_m": round(mark_length, 3),
                "segments": [{
                    "type": "MARK",
                    "speed": marking_speed,
                    "source": f"builtin:{name}",
                    "length_m": round(mark_length, 3),
                }] if shifted else [],
                "alignment_metadata": {},
                "warnings": [],
            }
            if not summary_only:
                result["merged_waypoints"] = shifted
                result["spray_flags"] = [True] * len(shifted)
            return result

        from path_engine.engine import PathEngine
        from path_engine.validator import PathValidator
        engine = PathEngine(
            mark_spacing=line_spacing,
            transit_spacing=transit_spacing,
            marking_speed=marking_speed,
            transit_speed=transit_speed,
            optimize_order=optimize,
            compensate_spray=compensate_spray,
        )

        # Resolve file path
        fpath = os.path.join(self._dir, os.path.basename(name))
        if os.path.isfile(fpath):
            plan = engine.plan_file(
                fpath,
                layer_mapping=layer_mapping,
                origin=origin,
                start_position=start_position,
                origin_gps=origin_gps,
                rotation_deg=rotation_deg,
                ref_points_dxf=ref_points_dxf,
                ref_points_gps=ref_points_gps,
                close_loop=close_loop,
            )
        else:
            raise FileNotFoundError(f"Path not found: {name!r}")

        # Run safety validation check
        validator = PathValidator()
        warnings = validator.validate(plan)

        result = {
            "source": source_name,
            "num_waypoints": plan.num_waypoints,
            "num_segments": len(plan.segments),
            "mark_length_m": round(plan.total_mark_length, 3),
            "transit_length_m": round(plan.total_transit_length, 3),
            "total_length_m": round(plan.total_length, 3),
            "segments": [
                {
                    "type": "MARK" if s.segment_type == 0 else "TRANSIT",
                    "speed": s.speed,
                    "source": s.source_entity,
                    "length_m": round(s.length, 3),
                }
                for s in plan.segments
            ],
            "alignment_metadata": plan.alignment_metadata,
            "warnings": warnings,
        }

        if not summary_only:
            result["merged_waypoints"] = plan.merged_waypoints
            result["spray_flags"] = plan.spray_flags

        return result

    # ── Internal ──────────────────────────────────────────────────────────────

    def _load_file(self, fpath: str) -> list[tuple[float, float]]:
        ext = os.path.splitext(fpath)[1].lower()
        if ext == ".waypoints":
            return read_qgc_waypoints(fpath)
        if ext == ".csv":
            return read_ned_csv(fpath)
        if ext == ".dxf":
            from path_engine import PathEngine
            engine = PathEngine()
            plan = engine.plan_file(fpath)
            return plan.merged_waypoints
        try:
            return read_qgc_waypoints(fpath)
        except Exception:
            return read_ned_csv(fpath)
