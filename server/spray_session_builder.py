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
) -> dict:
    """Return a schema-1 SpraySessionConfig dict for the given mode.

    Unknown/empty modes fall back to continuous (the safe default). Dash needs
    both distances; if either is missing it also falls back to continuous so a
    half-configured request can never silently ship a broken dash.
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
    # NOTE: mode == "point" is intentionally NOT emitted yet — Phase D builds
    # the points_mode payload + the planner-side per-point hold together.
    return cfg


def build_session_config_json(mode: str = "continuous", **kwargs) -> str:
    """build_session_config(...) serialized with allow_nan=False (never inf/nan)."""
    return json.dumps(build_session_config(mode, **kwargs), allow_nan=False)


def cleared_config_json() -> str:
    """Explicit cleared config for mission teardown (continuous, empty)."""
    return build_session_config_json("continuous")
