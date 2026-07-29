"""Build /spray/session_config JSON payloads (B0, plan §3).

The spray node (`src/spray_session_config.py`) is the SINGLE parser and fails
static on anything it does not accept. This module only *constructs* the blob
the operator selected; it never validates. Geometry (points/flags) is NOT sent
here — it rides `/path` as it always has — so these payloads carry only the
mode and its mode-specific params, with empty points/flags.

⚠ Schema mirror: these dicts must match `src/spray_session_config.py`
SCHEMA_VERSION. If they drift, the node logs "session_config rejected" and
keeps its last-known-good mode (fail static) — a visible, safe failure, not a
silent one. Bump both together.
"""

from __future__ import annotations

import json
from typing import Optional

# Must equal src/spray_session_config.py SCHEMA_VERSION.
SPRAY_SCHEMA_VERSION = 1

VALID_MODES = ("continuous", "dash", "point")


def build_session_config(
    mode: str = "continuous",
    *,
    dash_on_distance_m: Optional[float] = None,
    dash_off_distance_m: Optional[float] = None,
    dash_start_state: str = "on",
    point_coordinates=None,
    point_arrival_tolerance_m: float = 0.05,
    point_arrival_settle_s: float = 0.2,
    point_dwell_s: float = 1.0,
    point_heading_tolerance_deg: Optional[float] = None,
    max_xtrack_error_m: Optional[float] = None,
) -> dict:
    """Return a schema-1 SpraySessionConfig dict for the given mode.

    Unknown/empty modes fall back to continuous (the safe default). Dash needs
    both distances; if incomplete it falls back to continuous so a
    half-configured dash can never silently ship a broken mode. Point does NOT
    require coordinates here — its dwell targets come from the /path must-hit
    vertices (see the point branch below), so point always emits point mode with
    its params and (usually empty) coordinates.

    max_xtrack_error_m is additive optional (R3): when None the key is omitted
    so the wire format stays identical for every existing caller; when set the
    node uses it instead of its ROS param.
    """
    mode = (mode or "continuous").lower()
    cfg = {
        "schema_version": SPRAY_SCHEMA_VERSION,
        "mode": "continuous",
        "points": [],
        "flags": [],
        "dash": None,
        "points_mode": None,
    }
    if mode == "dash" and dash_on_distance_m and dash_off_distance_m:
        cfg["mode"] = "dash"
        cfg["dash"] = {
            "on_distance_m": float(dash_on_distance_m),
            "off_distance_m": float(dash_off_distance_m),
            "start_state": "off" if dash_start_state == "off" else "on",
        }
    elif mode == "point":
        # Coordinates are OPTIONAL here: the frame-correct dwell targets are the
        # must-hit vertices on /path (placed into the live EKF frame at mission
        # start), which only the spray node sees. The server cannot know the
        # placement offset at plan/load time, so it emits point mode with the
        # dwell/tolerance PARAMS and (usually empty) coordinates; the node fills
        # the coordinates from /path. An explicit list, when given, is a bench
        # fallback the node uses only when /path carries no must-hit vertices.
        cfg["mode"] = "point"
        cfg["points_mode"] = {
            "coordinates": [[float(n), float(e)] for n, e in (point_coordinates or [])],
            "arrival_tolerance_m": float(point_arrival_tolerance_m),
            "heading_tolerance_deg": (
                None if point_heading_tolerance_deg is None
                else float(point_heading_tolerance_deg)
            ),
            "arrival_settle_s": float(point_arrival_settle_s),
            "dwell_s": float(point_dwell_s),
        }
    if max_xtrack_error_m is not None:
        cfg["max_xtrack_error_m"] = float(max_xtrack_error_m)
    return cfg


def build_session_config_json(mode: str = "continuous", **kwargs) -> str:
    """build_session_config(...) serialized with allow_nan=False (never inf/nan)."""
    return json.dumps(build_session_config(mode, **kwargs), allow_nan=False)


def cleared_config_json() -> str:
    """Explicit cleared config for mission teardown (continuous, empty)."""
    return build_session_config_json("continuous")
