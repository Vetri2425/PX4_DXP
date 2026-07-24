"""Path loading, generation, and uploaded-file management.

Hardcoded path generators mirror `path_publisher_node.py` so the server
produces identical waypoint sets without importing from the src/ ROS
package.
"""
from __future__ import annotations

import copy
import csv
import json
import math
import os
from functools import lru_cache
from typing import Optional

from config import ALLOWED_UPLOAD_EXTENSIONS, MAX_UPLOAD_BYTES
from logging_setup import get_logger
from models import PathInfo, PathPreviewResponse

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


def _densify_edge(
    a: tuple[float, float], b: tuple[float, float], spacing: float
) -> list[tuple[float, float]]:
    """Points from a (exclusive) to b (INCLUSIVE), ~spacing apart.

    The endpoint b is always emitted exactly, so corners land on their true
    coordinate regardless of whether the edge length divides evenly by spacing.
    (The naive `i*spacing` stepping left a `len - floor(len/spacing)*spacing`
    gap before every corner — e.g. 2.0 m at 0.15 m stopped at 1.95 m.)
    """
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, round(length / spacing))
    return [
        (a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
        for i in range(1, n + 1)
    ]


def _polyline(
    corners: list[tuple[float, float]], spacing: float
) -> list[tuple[float, float]]:
    """Densify a corner list into a point path; shared corners appear once.

    Pass a closed corner list (last == first) for closed shapes so the path
    returns exactly to its origin.
    """
    pts = [corners[0]]
    for k in range(len(corners) - 1):
        pts += _densify_edge(corners[k], corners[k + 1], spacing)
    return pts


def gen_lshape_2x2(spacing: float = 0.15) -> list[tuple[float, float]]:
    # Open L: north 2 m then east 2 m. Corners land exactly at (2,0) and (2,2).
    return _polyline([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], spacing)


def gen_square_2x2(spacing: float = 0.15) -> list[tuple[float, float]]:
    # Closed 2x2 square — exact corners, returns to (0,0).
    return _polyline(
        [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)], spacing
    )


def gen_rectangle_3x2(spacing: float = 0.15) -> list[tuple[float, float]]:
    # Closed 3 m (north) x 2 m (east) rectangle — exact corners, returns to (0,0).
    return _polyline(
        [(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (0.0, 2.0), (0.0, 0.0)], spacing
    )


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
        # Listing cache: fpath -> (mtime, size, PathInfo). Avoids re-parsing
        # unchanged files on every /api/paths call. Invalidated per-file on
        # mtime/size change; deleted files are evicted on the next listing.
        self._list_cache: dict[str, tuple[float, int, PathInfo]] = {}
        # Preview cache: fpath -> (mtime_ns, size, PathPreviewResponse). DXF
        # preview planning can be expensive; unchanged files can reuse the
        # already materialized response.
        self._preview_cache: dict[str, tuple[int, int, PathPreviewResponse]] = {}
        # Parsed-entity cache: fpath -> (mtime_ns, size, [DXFEntity]). A full
        # ezdxf parse can take seconds on large files and runs for the
        # entities endpoint, override save validation, preview, and planning.
        # Cached entries are pristine; callers get per-entity shallow copies
        # so apply_entity_overrides() never mutates the cached objects.
        self._entity_cache: dict[str, tuple[int, int, list]] = {}

    def _require_dxf(self, filename: str, what: str) -> tuple[str, str]:
        """Validate *filename* is an existing .dxf in the missions dir.

        Returns (safe_basename, fpath). Raises FileNotFoundError / ValueError.
        """
        safe = os.path.basename(filename)
        fpath = os.path.join(self._dir, safe)
        if not os.path.isfile(fpath):
            raise FileNotFoundError(f"Path not found: {filename!r}")
        if os.path.splitext(fpath)[1].lower() != ".dxf":
            raise ValueError(f"{what} only available for DXF files")
        return safe, fpath

    @staticmethod
    def _write_sidecar(sidecar: str, payload: dict) -> None:
        """Atomically write a JSON sidecar (tmp + os.replace)."""
        tmp = sidecar + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(payload, f, sort_keys=True)
        os.replace(tmp, sidecar)

    @staticmethod
    def _entity_overrides_path(fpath: str) -> str:
        """Hidden sidecar path for per-entity spray overrides."""
        dirname = os.path.dirname(fpath)
        basename = os.path.basename(fpath)
        return os.path.join(dirname, f".{basename}.entities.json")

    @staticmethod
    def _extension_config_path(fpath: str) -> str:
        """Hidden sidecar path for per-file extension settings."""
        dirname = os.path.dirname(fpath)
        basename = os.path.basename(fpath)
        return os.path.join(dirname, f".{basename}.extensions.json")

    @staticmethod
    def _entity_order_path(fpath: str) -> str:
        """Hidden sidecar path for per-file entity execution order."""
        dirname = os.path.dirname(fpath)
        basename = os.path.basename(fpath)
        return os.path.join(dirname, f".{basename}.entity_order.json")

    def load_entity_order(self, filename: str) -> list[str]:
        """Load saved entity execution order for a mission file.

        Returns the saved ID order, or [] if no sidecar exists.
        Malformed JSON is logged and treated as missing ([]).
        """
        fpath = os.path.join(self._dir, os.path.basename(filename))
        sidecar = self._entity_order_path(fpath)
        try:
            with open(sidecar, encoding="utf-8") as f:
                payload = json.load(f)
        except FileNotFoundError:
            return []
        except (OSError, ValueError) as exc:
            log.warning("ignoring invalid entity order sidecar %s: %s", sidecar, exc)
            return []

        raw = payload.get("entity_order", [])
        if not isinstance(raw, list) or not all(isinstance(v, str) for v in raw):
            log.warning("ignoring malformed entity_order in %s", sidecar)
            return []
        return raw

    def save_entity_order(self, filename: str, entity_order: list[str]) -> None:
        """Persist entity execution order for a DXF mission file.

        Args:
            filename: DXF filename in the missions directory.
            entity_order: Ordered list of entity IDs.

        Raises FileNotFoundError if the DXF does not exist.
        """
        safe, fpath = self._require_dxf(filename, "Entity ordering is")
        self._write_sidecar(
            self._entity_order_path(fpath),
            {"source": safe, "entity_order": list(entity_order)},
        )
        self._preview_cache.pop(fpath, None)

    def clear_entity_order(self, filename: str) -> None:
        """Remove saved entity execution order for a mission file, if present."""
        fpath = os.path.join(self._dir, os.path.basename(filename))
        sidecar = self._entity_order_path(fpath)
        try:
            os.remove(sidecar)
        except FileNotFoundError:
            pass
        self._preview_cache.pop(fpath, None)

    def _estimate_waypoint_count(
        self,
        fpath: str,
        layer_mapping: dict[str, str] | None,
        mark_spacing: float,
        transit_spacing: float,
    ) -> int:
        """Cheap pre-plan estimate of the densified waypoint count.

        Parses the DXF (fast) and sums sprayed/transit arc-length ÷ spacing.
        Ignored (annotation) entities are excluded, matching the planner. This
        is a lower bound (chord sums undercount curves, extensions add a little)
        so it never over-rejects a valid mission — the post-plan validator stays
        authoritative. Returns 0 on any parse issue so a real failure surfaces
        later with a precise message rather than being masked here.
        """
        try:
            entities = self.parse_dxf(fpath)
        except Exception:
            return 0
        mark_len = 0.0
        transit_len = 0.0
        for ent in entities:
            cls = ent.classify(layer_mapping)
            if cls == "ignore":
                continue
            length = ent.approx_length_m()
            if cls == "transit":
                transit_len += length
            else:
                mark_len += length
        ms = max(float(mark_spacing), 1e-3)
        ts = max(float(transit_spacing), 1e-3)
        return int(mark_len / ms + transit_len / ts)

    def load_extension_config(self, filename: str) -> dict[str, float | bool]:
        """Load saved path extension config for a mission file."""
        default = {
            "enabled": False,
            "pre_extension_m": 0.5,
            "aft_extension_m": 0.5,
            # Default per-entity: when extensions are enabled without an explicit
            # per_line choice, every CAD edge gets its own PRE→MARK→AFT run-up
            # (each side of a square/polygon and every corner), rather than only a
            # chain's open ends. Explicitly-saved per_line values still win.
            "per_line": True,
        }
        fpath = os.path.join(self._dir, os.path.basename(filename))
        sidecar = self._extension_config_path(fpath)
        try:
            with open(sidecar, encoding="utf-8") as f:
                payload = json.load(f)
        except FileNotFoundError:
            return default
        except (OSError, ValueError) as exc:
            log.warning("ignoring invalid extension config sidecar %s: %s", sidecar, exc)
            return default

        try:
            pre = float(payload.get("pre_extension_m", default["pre_extension_m"]))
            aft = float(payload.get("aft_extension_m", default["aft_extension_m"]))
        except (TypeError, ValueError):
            return default
        if pre < 0.0 or aft < 0.0:
            return default
        return {
            "enabled": bool(payload.get("enabled", default["enabled"])),
            "pre_extension_m": pre,
            "aft_extension_m": aft,
            "per_line": bool(payload.get("per_line", default["per_line"])),
        }

    def resolve_extension_settings(
        self, name: str
    ) -> tuple[bool, float, float, bool]:
        """Resolve (enabled, pre_extension_m, aft_extension_m, per_line) for a path.

        Builtin paths never run through the extension stage, so they always
        resolve to disabled. Other paths use the saved per-file sidecar config.
        This is the single source of truth shared by preview_path() and
        plan_path() so the displayed preview, the spray-flag schedule, and the
        executed mission stay in lock-step.
        """
        if name in BUILTIN_PATHS:
            return False, 0.5, 0.5, False
        cfg = self.load_extension_config(name)
        return (
            bool(cfg["enabled"]),
            float(cfg["pre_extension_m"]),
            float(cfg["aft_extension_m"]),
            bool(cfg["per_line"]),
        )

    def save_extension_config(
        self,
        filename: str,
        enabled: bool,
        pre_extension_m: float,
        aft_extension_m: float,
        per_line: bool | None = None,
    ) -> dict[str, float | bool]:
        """Persist path extension config for a DXF mission file.

        ``per_line=None`` means "leave unchanged": preserve whatever was last
        saved. This stops an older frontend that POSTs without the per_line field
        from silently resetting it to False. Pass an explicit True/False to set it.
        """
        safe, fpath = self._require_dxf(filename, "Path extensions are")
        if pre_extension_m < 0.0:
            raise ValueError("pre_extension_m must be >= 0.0")
        if aft_extension_m < 0.0:
            raise ValueError("aft_extension_m must be >= 0.0")

        if per_line is None:
            # Sticky: keep the previously saved per_line rather than defaulting.
            per_line = bool(self.load_extension_config(filename)["per_line"])

        config = {
            "enabled": bool(enabled),
            "pre_extension_m": float(pre_extension_m),
            "aft_extension_m": float(aft_extension_m),
            "per_line": bool(per_line),
        }
        self._write_sidecar(
            self._extension_config_path(fpath),
            {"source": safe, **config},
        )
        self._preview_cache.pop(fpath, None)
        return config

    def clear_extension_config(self, filename: str) -> None:
        """Remove saved extension config for a mission file, if present."""
        fpath = os.path.join(self._dir, os.path.basename(filename))
        sidecar = self._extension_config_path(fpath)
        try:
            os.remove(sidecar)
        except FileNotFoundError:
            pass

    def load_entity_overrides(self, filename: str) -> dict[str, bool]:
        """Load saved entity_id -> is_mark overrides for a mission file."""
        fpath = os.path.join(self._dir, os.path.basename(filename))
        sidecar = self._entity_overrides_path(fpath)
        try:
            with open(sidecar, encoding="utf-8") as f:
                payload = json.load(f)
        except FileNotFoundError:
            return {}
        except (OSError, ValueError) as exc:
            log.warning("ignoring invalid entity override sidecar %s: %s", sidecar, exc)
            return {}

        raw = payload.get("overrides", {})
        if not isinstance(raw, dict):
            return {}
        overrides: dict[str, bool] = {}
        for entity_id, value in raw.items():
            # Canonical shape (the only one save_entity_overrides writes):
            # {"<entity_id>": {"is_mark": bool}}. Anything else is corrupt —
            # skip it loudly rather than guessing a spray decision.
            if isinstance(value, dict) and isinstance(value.get("is_mark"), bool):
                overrides[str(entity_id)] = value["is_mark"]
            else:
                log.warning(
                    "ignoring malformed entity override %r=%r in %s",
                    entity_id, value, sidecar,
                )
        return overrides

    def save_entity_overrides(self, filename: str, overrides: dict[str, bool]) -> int:
        """Persist entity_id -> is_mark overrides for a DXF mission file."""
        safe, fpath = self._require_dxf(filename, "Entity overrides are")

        entities = self.parse_dxf(fpath)
        valid_ids = {ent.entity_id for ent in entities}
        unknown = sorted(set(overrides) - valid_ids)
        if unknown:
            raise ValueError(f"Unknown entity_id(s): {', '.join(unknown)}")

        cleaned = {str(entity_id): {"is_mark": bool(is_mark)} for entity_id, is_mark in overrides.items()}
        self._write_sidecar(
            self._entity_overrides_path(fpath),
            {"source": safe, "overrides": cleaned},
        )
        self._preview_cache.pop(fpath, None)
        return len(cleaned)

    def clear_entity_overrides(self, filename: str) -> None:
        """Remove saved per-entity overrides for a mission file, if present."""
        fpath = os.path.join(self._dir, os.path.basename(filename))
        sidecar = self._entity_overrides_path(fpath)
        try:
            os.remove(sidecar)
        except FileNotFoundError:
            pass
        self._preview_cache.pop(fpath, None)

    @staticmethod
    def apply_entity_overrides(entities: list, overrides: dict[str, bool]) -> None:
        """Stamp saved spray overrides onto parsed entities in-place.

        Sets DXFEntity.is_mark_override; classify() consults it, and an
        'ignore' layer_mapping classification still wins over the override.
        """
        for ent in entities:
            if ent.entity_id in overrides:
                ent.is_mark_override = bool(overrides[ent.entity_id])

    @staticmethod
    def _cheap_point_count(fpath: str) -> int:
        """Fast size metric for the path listing — never runs the planner.

        CSV/.waypoints are already cheap line reads (exact point count). DXF
        is counted by raw geometry vertices via a parse only (NO densify /
        2-opt / corner smoothing), so listing stays sub-second regardless of
        how many files accumulate. This is a coarse size indicator, not the
        densified planned-waypoint count (use /preview or /plan for that).
        """
        ext = os.path.splitext(fpath)[1].lower()
        if ext == ".waypoints":
            return len(read_qgc_waypoints(fpath))
        if ext == ".csv":
            return len(read_ned_csv(fpath))
        if ext == ".dxf":
            from path_engine.parsers.dxf_parser import parse_dxf
            total = 0
            for ent in parse_dxf(fpath):
                geom = ent.geometry
                verts = geom.get("vertices")
                if verts is not None:
                    total += len(verts)
                elif ent.entity_type == "LINE":
                    total += 2
                elif ent.entity_type == "POINT":
                    total += 1
                else:  # ARC / CIRCLE / ELLIPSE / SPLINE — nominal
                    total += 2
            return total
        # Unknown extension: fall back to the cheap readers.
        try:
            return len(read_qgc_waypoints(fpath))
        except Exception:
            return len(read_ned_csv(fpath))

    def list_paths(self) -> list[PathInfo]:
        result: list[PathInfo] = []
        for name, info in BUILTIN_PATHS.items():
            pts = _cached_builtin(name)
            result.append(PathInfo(
                name=name, description=info["desc"],
                num_points=len(pts), source="builtin",
            ))
        seen: set[str] = set()
        for fname in sorted(os.listdir(self._dir)):
            fpath = os.path.join(self._dir, fname)
            if not os.path.isfile(fpath) or fname.startswith("."):
                continue
            try:
                st = os.stat(fpath)
            except OSError:
                continue
            seen.add(fpath)
            cached = self._list_cache.get(fpath)
            if cached and cached[0] == st.st_mtime and cached[1] == st.st_size:
                result.append(cached[2])
                continue
            try:
                n_points = self._cheap_point_count(fpath)
            except Exception as exc:
                log.warning("skipping uploaded file %s: %s", fname, exc)
                continue
            info_obj = PathInfo(
                name=fname,
                description=f"Uploaded: {fname}",
                num_points=n_points,
                source="file",
            )
            self._list_cache[fpath] = (st.st_mtime, st.st_size, info_obj)
            result.append(info_obj)
        # Evict cache entries for files that no longer exist.
        for stale in set(self._list_cache) - seen:
            self._list_cache.pop(stale, None)
        return result

    def load_path(
        self,
        name: str,
        origin: tuple[float, float] = (0.0, 0.0),
        start_position: tuple[float, float] | None = None,
        auto_origin: bool = False,
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
            if ext == ".dxf":
                # Route DXF through the full planning pipeline so the executed
                # mission honors the saved per-file extension config + tuning
                # (and extension-aware auto-origin), matching the preview. A bare
                # PathEngine() here would silently drop PRE/AFT legs and tuning.
                result = self.plan_path(
                    name,
                    summary_only=False,
                    origin=origin,
                    start_position=start_position,
                    auto_origin=auto_origin,
                )
                return result["merged_waypoints"]
            if ext == ".csv" and (
                origin != (0.0, 0.0) or start_position is not None
            ):
                from path_engine import PathEngine
                # Arc-fit surveyed curves (no-op for a legacy NED CSV, which has
                # no LINE_CHAIN); matches preview_path + plan_path.
                engine = PathEngine(fit_arcs=self._is_survey_csv(name))
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

    def _is_survey_csv(self, name: str) -> bool:
        """True when *name* is a named-header survey CSV in the missions dir.

        Survey CSVs are the pre-line source: their curves must be arc-fitted, not
        densified into straight chords. DXF (whose grouped polylines are also
        LINE_CHAINs) and legacy headerless NED CSVs are excluded, so their frozen
        behaviour is untouched.
        """
        if name.startswith("builtin:"):
            return False
        fpath = os.path.join(self._dir, os.path.basename(name))
        if not fpath.lower().endswith(".csv") or not os.path.isfile(fpath):
            return False
        from path_engine.parsers.survey_csv import looks_like_survey_csv
        return looks_like_survey_csv(fpath)

    @staticmethod
    def _survey_geo_origin(fpath: str) -> list[float] | None:
        """WGS84 (lat, lon) a survey CSV's local frame is anchored at, or None."""
        try:
            from path_engine.parsers.survey_csv import read_survey_csv
            res = read_survey_csv(fpath)
        except Exception:
            return None
        if res.geo_origin is None:
            return None
        return [float(res.geo_origin[0]), float(res.geo_origin[1])]

    def preview_path(self, name: str) -> PathPreviewResponse:
        """Return local-NED points for display without touching mission state."""
        geo_origin: list[float] | None = None
        lookup_name = (
            name.removeprefix("builtin:")
            if name.startswith("builtin:")
            else name
        )
        if lookup_name in BUILTIN_PATHS:
            pts = list(_cached_builtin(lookup_name))
            spray_flags = [True] * len(pts)
            # Builtins are pre-densified, so which points are "vertices" is not
            # recoverable. Claim none rather than claim all.
            must_hit = [False] * len(pts)
        else:
            fpath = os.path.join(self._dir, os.path.basename(name))
            if not os.path.isfile(fpath):
                raise FileNotFoundError(f"Path not found: {name!r}")
            st = os.stat(fpath)
            cached = self._preview_cache.get(fpath)
            if cached and cached[0] == st.st_mtime_ns and cached[1] == st.st_size:
                return cached[2]
            if os.path.splitext(fpath)[1].lower() == ".dxf":
                from path_engine import PathEngine
                from path_engine.entity_order import apply_entity_order
                # Honor the saved extension config so the preview, its spray
                # schedule, and the executed mission (load_path → plan_path)
                # all share the same waypoint count and spray flags. A bare
                # PathEngine() here would drop PRE/AFT legs and desync the
                # spray-flag length from the executed path.
                enabled, pre_m, aft_m, per_line = self.resolve_extension_settings(name)
                overrides = self.load_entity_overrides(name)
                saved_order = self.load_entity_order(name)
                # Suppress optimizer when saved entity order is present so
                # preview matches planned execution.
                effective_optimize = not bool(saved_order)
                engine = PathEngine(
                    enable_path_extensions=enabled,
                    pre_extension_m=pre_m,
                    aft_extension_m=aft_m,
                    per_line_extensions=per_line,
                    optimize_order=effective_optimize,
                )
                if overrides or saved_order:
                    entities = self.parse_dxf(fpath)
                    if overrides:
                        self.apply_entity_overrides(entities, overrides)
                    if saved_order:
                        entities = apply_entity_order(entities, saved_order)
                    plan = engine.plan_dxf_entities(entities)
                else:
                    plan = engine.plan_file(fpath)
                pts = list(plan.merged_waypoints)
                spray_flags = list(plan.spray_flags)
                must_hit = list(getattr(plan, "must_hit", []) or [])
            else:
                # A survey CSV (named lat/lon or grid header) runs through the
                # full planner so the preview MATCHES the executed mission:
                # densified, grouped by feature code, real spray flags and
                # must-hit provenance (only surveyed vertices, not fill), plus the
                # WGS84 origin its frame is anchored at — the WYSIWYG contract the
                # map needs. A legacy headerless NED CSV / .waypoints file is an
                # explicit point list with no provenance, so every point is a
                # vertex.
                from path_engine.parsers.survey_csv import looks_like_survey_csv
                if fpath.lower().endswith(".csv") and looks_like_survey_csv(fpath):
                    from path_engine import PathEngine
                    # Arc-fit a surveyed curve so the preview shows a true arc,
                    # not straight chords between vertices. Matches execution
                    # (plan_path auto-enables fit_arcs for survey CSVs too).
                    plan = PathEngine(fit_arcs=True).plan_file(fpath)
                    pts = list(plan.merged_waypoints)
                    spray_flags = list(plan.spray_flags)
                    must_hit = list(getattr(plan, "must_hit", []) or [])
                    geo_origin = self._survey_geo_origin(fpath)
                else:
                    pts = self._load_file(fpath)
                    spray_flags = [True] * len(pts)
                    must_hit = [True] * len(pts)

        if len(spray_flags) != len(pts):
            spray_flags = [True] * len(pts)
        if len(must_hit) != len(pts):
            must_hit = [False] * len(pts)

        waypoints = [
            {"north": n, "east": e, "spray": spray, "must_hit": vertex}
            for (n, e), spray, vertex in zip(pts, spray_flags, must_hit)
        ]
        if pts:
            norths = [n for n, _ in pts]
            easts = [e for _, e in pts]
            bounds = {
                "north_min": min(norths),
                "north_max": max(norths),
                "east_min": min(easts),
                "east_max": max(easts),
            }
        else:
            bounds = None

        response = PathPreviewResponse(
            name=name,
            num_points=len(pts),
            bounds=bounds,
            waypoints=waypoints,
            geo_origin=geo_origin,
        )
        if lookup_name not in BUILTIN_PATHS:
            self._preview_cache[fpath] = (st.st_mtime_ns, st.st_size, response)
        return response

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
        self._list_cache.pop(fpath, None)
        self._preview_cache.pop(fpath, None)
        self._entity_cache.pop(fpath, None)
        self.clear_entity_overrides(safe)
        self.clear_extension_config(safe)
        self.clear_entity_order(safe)
        log.info("uploaded mission file: %s (%d bytes)", safe, len(content))
        return safe

    def delete_file(self, filename: str) -> bool:
        fpath = os.path.join(self._dir, os.path.basename(filename))
        if os.path.isfile(fpath):
            os.remove(fpath)
            self._list_cache.pop(fpath, None)
            self._preview_cache.pop(fpath, None)
            self._entity_cache.pop(fpath, None)
            self.clear_entity_overrides(filename)
            self.clear_extension_config(filename)
            self.clear_entity_order(filename)
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
        if unit_scale is not None or layer_mapping is not None:
            # Non-default parse parameters: bypass the cache rather than key on
            # them — every current caller uses the defaults.
            return parse_dxf(filepath, unit_scale=unit_scale, layer_mapping=layer_mapping)

        st = os.stat(filepath)
        cached = self._entity_cache.get(filepath)
        if cached and cached[0] == st.st_mtime_ns and cached[1] == st.st_size:
            entities = cached[2]
        else:
            entities = parse_dxf(filepath)
            self._entity_cache[filepath] = (st.st_mtime_ns, st.st_size, entities)
        # Shallow copies: apply_entity_overrides() mutates is_mark_override,
        # which must never leak into the pristine cached entities.
        return [copy.copy(ent) for ent in entities]

    # ── Segment serialization helpers ────────────────────────────────────────

    @staticmethod
    def _segment_role(segment) -> str:
        """Human-readable role for a planned segment.

        Returns one of: "pre_transit" | "aft_transit" | "mark" | "transit".
        """
        role = segment.metadata.get("extension_role")
        if role == "pre":
            return "pre_transit"
        if role == "aft":
            return "aft_transit"
        if segment.segment_type == 0:  # SegmentType.MARK == 0
            return "mark"
        return "transit"

    @staticmethod
    def _parent_source_entity(segment) -> str | None:
        """Clean parent source_entity name (strips :pre/:aft suffix).

        Returns the ``parent_source_entity`` from metadata when present
        (injected by extensions.py), otherwise strips any trailing colon-suffix
        from source_entity itself.
        """
        parent = segment.metadata.get("parent_source_entity")
        if parent:
            return parent
        source = segment.source_entity or ""
        return source.split(":")[0] if source else None

    @staticmethod
    def _extract_entity_id(parent_source: str | None) -> str | None:
        """Extract the raw DXF ezdxf handle from a source_entity name.

        Format: ``{TYPE}_{handle}``  e.g. ``LINE_1A3`` → ``"1A3"``.
        Synthetic sources (``transit:N``, ``group:...``, ``builtin:...``)
        return ``None``.
        """
        if not parent_source:
            return None
        if ":" in parent_source:
            # Any colon-prefixed synthetic source: transit:N, group:..., builtin:...
            return None
        if "_" not in parent_source:
            return None
        return parent_source.split("_", 1)[1]

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
        auto_origin = bool(kwargs.pop("auto_origin", False))
        layer_mapping = kwargs.pop("layer_mapping", None)
        optimize = kwargs.pop("optimize", True)
        # OFF by default: the spray controller node already compensates for solenoid
        # latency at runtime, from actual speed. See PathPlanRequest.compensate_spray.
        compensate_spray = kwargs.pop("compensate_spray", False)
        extension_kwargs_provided = any(
            key in kwargs
            for key in ("enable_path_extensions", "pre_extension_m", "aft_extension_m")
        )
        enable_path_extensions = kwargs.pop("enable_path_extensions", None)
        pre_extension_m = kwargs.pop("pre_extension_m", None)
        aft_extension_m = kwargs.pop("aft_extension_m", None)
        per_line_extensions = kwargs.pop("per_line_extensions", None)
        corner_smooth_radius_m = kwargs.pop("corner_smooth_radius_m", 0.0)
        corner_smooth_arc_pts = kwargs.pop("corner_smooth_arc_pts", 6)
        # Arc fit for surveyed LINE_CHAINs. Auto-ON for a survey CSV (its curves
        # must come out as arcs, not chords; corner-split keeps squares/lines
        # unchanged), OFF otherwise so DXF/builtin/legacy stay byte-for-byte
        # frozen. An explicit fit_arcs kwarg always wins.
        fit_arcs_kw = kwargs.pop("fit_arcs", None)
        fit_arcs = self._is_survey_csv(source_name) if fit_arcs_kw is None else bool(fit_arcs_kw)
        fit_arcs_rms_m = kwargs.pop("fit_arcs_rms_m", 0.025)
        fit_arcs_corner_deg = kwargs.pop("fit_arcs_corner_deg", 35.0)
        # Paint the closing side of an open MARK shape (distinct from close_loop,
        # which deadheads). Default OFF → every existing plan is unchanged.
        close_shape = bool(kwargs.pop("close_shape", False))
        use_two_opt = kwargs.pop("use_two_opt", True)
        max_two_opt_segments = kwargs.pop("max_two_opt_segments", 80)
        max_waypoints = kwargs.pop("max_waypoints", 10000)
        max_segments = kwargs.pop("max_segments", 2000)
        line_spacing = kwargs.pop("line_spacing", 0.05)
        transit_spacing = kwargs.pop("transit_spacing", 0.15)
        marking_speed = kwargs.pop("marking_speed", 0.35)
        transit_speed = kwargs.pop("transit_speed", 0.50)
        origin_gps = kwargs.pop("origin_gps", None)
        rotation_deg = kwargs.pop("rotation_deg", 0.0)
        ref_points_dxf = kwargs.pop("ref_points_dxf", None)
        ref_points_gps = kwargs.pop("ref_points_gps", None)
        close_loop = kwargs.pop("close_loop", False)
        # Additive: when True, each segment dict carries its full point list and
        # an explicit spray_on flag (used by GET /api/path/{name}/segments for
        # spray verification). Default False keeps /api/path/plan byte-identical.
        include_segment_points = bool(kwargs.pop("include_segment_points", False))

        # Resolve extension settings once, for every branch below. Explicit
        # kwargs (legacy callers/tests) win; otherwise the per-DXF sidecar
        # config saved via /api/path/{name}/extensions applies.
        if extension_kwargs_provided:
            enable_path_extensions = bool(enable_path_extensions)
            pre_extension_m = float(pre_extension_m) if pre_extension_m is not None else 0.5
            aft_extension_m = float(aft_extension_m) if aft_extension_m is not None else 0.5
            per_line_extensions = bool(per_line_extensions)
        elif name in BUILTIN_PATHS:
            enable_path_extensions = False
            pre_extension_m = 0.5
            aft_extension_m = 0.5
            per_line_extensions = False
        else:
            enable_path_extensions, pre_extension_m, aft_extension_m, per_line_extensions = (
                self.resolve_extension_settings(name)
            )

        # Auto-origin anchoring: when the rover is anchoring the mission to its
        # current pose, place the first driven waypoint at the rover position.
        # With extensions active this is the PRE run-up point; without extensions
        # it is the first marking waypoint of the first entity. Either way the
        # rover starts driving forward from where it stands.
        # drawing_origin (DXF 0,0 → rover) was the prior default but caused RPP
        # to skip the first segment when the first entity's endpoint happened to
        # coincide with the DXF origin. Builtins are pre-shifted and exempt.
        anchor = (
            "first_waypoint"
            if (auto_origin and name not in BUILTIN_PATHS)
            else "drawing_origin"
        )

        if name in BUILTIN_PATHS:
            # Builtin preview must match the path that mission/load publishes:
            # these generators are already densified to their tuned spacing.
            if enable_path_extensions:
                log.warning(
                    "enable_path_extensions ignored for builtin path %r: "
                    "builtins are pre-densified and not run through PathEngine.",
                    name,
                )
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
                    "runtime_segment_index": 0,
                    "runtime_sequence": 1,
                    "type": "MARK",
                    "segment_role": "mark",
                    "source": f"builtin:{name}",
                    "parent_source_entity": f"builtin:{name}",
                    "parent_entity_id": None,
                    "order_source": "parser_order",
                    "is_extension": False,
                    "speed": marking_speed,
                    "length_m": round(mark_length, 3),
                    **({"points": [list(p) for p in shifted], "spray_on": True}
                       if include_segment_points else {}),
                }] if shifted else [],
                "alignment_metadata": {},
                "planning_metadata": {
                    "source": {"kind": "builtin", "name": name},
                    "final_waypoints": len(shifted),
                    "final_segments": 1 if shifted else 0,
                },
                "warnings": [],
            }
            from path_engine.core import PlannedPath, PathSegment, SegmentType
            from path_engine.validator import PathValidator
            plan = PlannedPath(
                segments=[PathSegment(segment_type=SegmentType.MARK, points=shifted)] if shifted else [],
                merged_waypoints=shifted,
                spray_flags=[True] * len(shifted),
            )
            validator = PathValidator(max_waypoints=max_waypoints, max_segments=max_segments)
            result["warnings"] = validator.validate_or_raise(plan)
            if not summary_only:
                result["merged_waypoints"] = shifted
                result["spray_flags"] = [True] * len(shifted)
            return result

        # Resolve file path
        fpath = os.path.join(self._dir, os.path.basename(name))
        if not os.path.isfile(fpath):
            raise FileNotFoundError(f"Path not found: {name!r}")

        from path_engine.engine import PathEngine
        from path_engine.validator import PathValidator

        # Load all sidecar state before constructing the engine so the
        # optimizer flag, entity overrides, and saved order all come from a
        # single consistent read of the filesystem.
        is_dxf = os.path.splitext(fpath)[1].lower() == ".dxf"

        # Pre-flight guard: reject an obviously-oversized mission from a cheap
        # length estimate BEFORE the full plan. The planner itself is fast, but
        # on a CPU-contended companion it runs behind the telemetry/MAVROS event
        # loop, so planning a 17k-waypoint pitch can exceed the request budget
        # and surface as an opaque 504 timeout instead of this actionable error.
        # The estimate uses the caller's spacing, so coarser spacing (the real
        # fix for a too-dense drawing) is honoured. Authoritative validation
        # still runs post-plan; this only short-circuits the clear over-limit case.
        if is_dxf:
            est_waypoints = self._estimate_waypoint_count(
                fpath, layer_mapping, line_spacing, transit_spacing
            )
            if est_waypoints > int(max_waypoints * 1.1):
                raise ValueError(
                    f"Too many waypoints: ~{est_waypoints} exceeds limit {max_waypoints}. "
                    "Increase spacing, fix units, or simplify the drawing before publishing."
                )

        overrides = self.load_entity_overrides(name) if is_dxf else {}
        saved_order: list[str] = self.load_entity_order(name) if is_dxf else []
        # When the user has committed a manual entity order, the optimizer would
        # silently re-arrange that order — disable it.
        effective_optimize = optimize and not bool(saved_order)

        engine = PathEngine(
            mark_spacing=line_spacing,
            transit_spacing=transit_spacing,
            marking_speed=marking_speed,
            transit_speed=transit_speed,
            optimize_order=effective_optimize,
            compensate_spray=compensate_spray,
            enable_path_extensions=enable_path_extensions,
            pre_extension_m=pre_extension_m,
            aft_extension_m=aft_extension_m,
            per_line_extensions=per_line_extensions,
            corner_smooth_radius_m=corner_smooth_radius_m,
            corner_smooth_arc_pts=corner_smooth_arc_pts,
            fit_arcs=fit_arcs,
            fit_arcs_rms_m=fit_arcs_rms_m,
            fit_arcs_corner_deg=fit_arcs_corner_deg,
            close_shape=close_shape,
            use_two_opt=use_two_opt,
            max_two_opt_segments=max_two_opt_segments,
        )

        plan_metadata: dict = {}

        # When either overrides or a saved order exist we must pass entities
        # explicitly so we can mutate / reorder them before planning.
        if overrides or saved_order:
            entities = self.parse_dxf(fpath)
            if overrides:
                self.apply_entity_overrides(entities, overrides)
            if saved_order:
                from path_engine.entity_order import apply_entity_order
                entities = apply_entity_order(entities, saved_order)
                log.debug(
                    "plan_path(%s): applying saved entity order (%d ids)",
                    name, len(saved_order),
                )
            plan = engine.plan_dxf_entities(
                entities,
                layer_mapping=layer_mapping,
                origin=origin,
                start_position=start_position,
                origin_gps=origin_gps,
                rotation_deg=rotation_deg,
                ref_points_dxf=ref_points_dxf,
                ref_points_gps=ref_points_gps,
                close_loop=close_loop,
                anchor=anchor,
            )
            if overrides:
                plan_metadata["entity_overrides"] = {
                    "num_overrides": len(overrides),
                    "entity_ids": sorted(overrides),
                }
            if saved_order:
                plan_metadata["entity_order"] = {
                    "num_ids": len(saved_order),
                    "optimizer_skipped": True,
                }
        else:
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
                anchor=anchor,
            )

        plan_metadata["extension_config"] = {
            "enabled": enable_path_extensions,
            "pre_extension_m": pre_extension_m,
            "aft_extension_m": aft_extension_m,
        }
        # PlannedPath always carries planning_metadata, but plan_path is also
        # exercised with lightweight plan fakes — don't assume the attribute.
        if getattr(plan, "planning_metadata", None) is None:
            plan.planning_metadata = {}
        plan.planning_metadata.update(plan_metadata)

        # Run safety validation check
        validator = PathValidator(max_waypoints=max_waypoints, max_segments=max_segments)
        warnings = validator.validate_or_raise(plan)

        # Determine order_source once — same value for every segment in this plan.
        if saved_order:
            _order_source = "saved_entity_order"
        elif effective_optimize:
            _order_source = "optimizer"
        else:
            _order_source = "parser_order"

        result = {
            "source": source_name,
            "num_waypoints": plan.num_waypoints,
            "num_segments": len(plan.segments),
            "mark_length_m": round(plan.total_mark_length, 3),
            "transit_length_m": round(plan.total_transit_length, 3),
            "total_length_m": round(plan.total_length, 3),
            "segments": [
                {
                    "runtime_segment_index": idx,
                    "runtime_sequence": idx + 1,
                    "type": "MARK" if s.segment_type == 0 else "TRANSIT",
                    "segment_role": self._segment_role(s),
                    "source": s.source_entity,
                    "parent_source_entity": self._parent_source_entity(s),
                    "parent_entity_id": self._extract_entity_id(
                        self._parent_source_entity(s)
                    ),
                    "order_source": _order_source,
                    "is_extension": s.metadata.get("extension_role") is not None,
                    "speed": s.speed,
                    "length_m": round(s.length, 3),
                    **({"points": [list(p) for p in s.points],
                        "spray_on": s.segment_type == 0}
                       if include_segment_points else {}),
                }
                for idx, s in enumerate(plan.segments)
            ],
            "alignment_metadata": getattr(plan, "alignment_metadata", {}),
            "planning_metadata": getattr(plan, "planning_metadata", {}),
            "warnings": warnings,
        }

        if not summary_only:
            result["merged_waypoints"] = plan.merged_waypoints
            result["spray_flags"] = plan.spray_flags
            # Provenance: True = source geometry vertex, never simplify away.
            result["must_hit"] = list(getattr(plan, "must_hit", []) or [])

        return result

    # ── Internal ──────────────────────────────────────────────────────────────

    def _load_file(self, fpath: str) -> list[tuple[float, float]]:
        ext = os.path.splitext(fpath)[1].lower()
        if ext == ".waypoints":
            return read_qgc_waypoints(fpath)
        if ext == ".csv":
            # A named-header survey export (Emlid/Trimble point file) is a
            # different format from the legacy headerless NED metres CSV, and
            # feeding one to read_ned_csv would read lat/lon or grid metres as
            # NED. Route it through the full planner so it is densified,
            # grouped by feature code and projected like any other mission.
            from path_engine.parsers.survey_csv import looks_like_survey_csv
            if looks_like_survey_csv(fpath):
                from path_engine import PathEngine
                # Arc-fit surveyed curves (matches preview_path + plan_path).
                return PathEngine(fit_arcs=True).plan_file(fpath).merged_waypoints
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
