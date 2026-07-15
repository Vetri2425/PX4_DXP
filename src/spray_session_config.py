#!/usr/bin/env python3
"""Spray session config schema — single source of truth (Spray Controller V2 §3).

Pure module: stdlib only, NO rclpy/ROS imports. This is the only parser for
the `SpraySessionConfig` schema described in
`docs/Architecture/SPRAY_CONTROLLER_V2_PLAN.md` §3 — there is deliberately no
second "declare these fields on the node" list to drift out of sync with this
one (that drift was defect #2 in `main`'s history; see plan §1).

`path_fingerprint()` is computed locally by whoever holds a
`SpraySessionConfig` (the node, at load time) purely for its own
change-detection. It is never supplied by a caller and never validated
against a caller-supplied value (that was defect #3) — `parse_session_config`
does not even look for a fingerprint field in the input.

Degraded loads (spray unavailable, flags zeroed) are not a separate code
path: they are just a `continuous_config_from_path(points, flags)` call
where every entry in `flags` happens to be `False`.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from typing import Optional, Sequence

SCHEMA_VERSION = 1

_VALID_MODES = ("continuous", "dash", "point")
_VALID_START_STATES = ("on", "off")


class ConfigSchemaError(ValueError):
    """Raised when a session-config dict fails schema validation.

    The caller (the node) is expected to catch this, log it, and keep its
    last-known-good config — fail static, not fail open (plan §3).
    """


@dataclass(frozen=True)
class DashConfig:
    on_distance_m: float
    off_distance_m: float
    start_state: str  # "on" | "off"


@dataclass(frozen=True)
class PointsModeConfig:
    coordinates: tuple  # tuple[tuple[float, float], ...]  (NED)
    arrival_tolerance_m: float
    heading_tolerance_deg: Optional[float]  # None = position-only arrival
    arrival_settle_s: float
    dwell_s: float


@dataclass(frozen=True)
class SpraySessionConfig:
    schema_version: int
    mode: str  # "continuous" | "dash" | "point"
    points: tuple  # tuple[tuple[float, float], ...]  (NED)
    flags: tuple  # tuple[bool, ...]  (per-point MARK/transit)
    dash: Optional[DashConfig]
    points_mode: Optional[PointsModeConfig]

    def path_fingerprint(self) -> str:
        """Stable hash over (points, flags) only — never over anything else.

        Used purely for node-local "has the mission geometry actually
        changed since last tick" change-detection (plan §3). Coordinates
        are rounded before hashing so float-formatting noise (e.g.
        1.0 vs 1.0000000001 from a round-trip) does not register as a
        change; the hash is otherwise a plain deterministic digest, not a
        security primitive.
        """
        # `+ 0.0` normalizes negative zero to positive zero: round(-1e-9, 6)
        # is -0.0, which formats as "-0.000000" and would hash differently
        # from an identical "0.000000" point — a spurious "geometry changed".
        parts = [
            f"{round(n, 6) + 0.0:.6f},{round(e, 6) + 0.0:.6f},{int(bool(f))}"
            for (n, e), f in zip(self.points, self.flags)
        ]
        canonical = "|".join(parts)
        return hashlib.sha1(canonical.encode("utf-8")).hexdigest()


def _finite_float(value, name: str) -> float:
    if isinstance(value, bool):
        # bool is a subclass of int; a stray True/False in a numeric field
        # is almost certainly a caller bug, not an intentional 0.0/1.0.
        raise ConfigSchemaError(f"{name} must be a number, got {value!r}")
    try:
        f = float(value)
    except (TypeError, ValueError):
        raise ConfigSchemaError(f"{name} must be a number, got {value!r}") from None
    if math.isnan(f) or math.isinf(f):
        raise ConfigSchemaError(f"{name} must be finite, got {value!r}")
    return f


def _coerce_point(raw, name: str) -> tuple:
    if not isinstance(raw, (list, tuple)) or len(raw) != 2:
        raise ConfigSchemaError(f"{name} must be a [n, e] pair, got {raw!r}")
    n = _finite_float(raw[0], f"{name}[0]")
    e = _finite_float(raw[1], f"{name}[1]")
    return (n, e)


def _coerce_points_flags(points_raw, flags_raw, *, label: str = "points/flags") -> tuple:
    if not isinstance(points_raw, (list, tuple)):
        raise ConfigSchemaError(f"'{label}' points must be a list")
    if not isinstance(flags_raw, (list, tuple)):
        raise ConfigSchemaError(f"'{label}' flags must be a list")
    if len(points_raw) != len(flags_raw):
        raise ConfigSchemaError(
            f"'{label}' points and flags must have equal length "
            f"(got {len(points_raw)} vs {len(flags_raw)})"
        )
    points = tuple(
        _coerce_point(p, f"points[{i}]") for i, p in enumerate(points_raw)
    )
    flags = tuple(bool(f) for f in flags_raw)
    return points, flags


def _parse_dash(raw) -> DashConfig:
    if not isinstance(raw, dict):
        raise ConfigSchemaError("'dash' sub-config must be a dict")
    missing = [k for k in ("on_distance_m", "off_distance_m", "start_state") if k not in raw]
    if missing:
        raise ConfigSchemaError(f"'dash' sub-config missing required field(s): {missing}")

    on_distance_m = _finite_float(raw["on_distance_m"], "dash.on_distance_m")
    off_distance_m = _finite_float(raw["off_distance_m"], "dash.off_distance_m")
    if on_distance_m <= 0:
        raise ConfigSchemaError(f"dash.on_distance_m must be > 0, got {on_distance_m}")
    if off_distance_m <= 0:
        raise ConfigSchemaError(f"dash.off_distance_m must be > 0, got {off_distance_m}")

    start_state = raw["start_state"]
    if start_state not in _VALID_START_STATES:
        raise ConfigSchemaError(
            f"dash.start_state must be one of {_VALID_START_STATES}, got {start_state!r}"
        )

    return DashConfig(
        on_distance_m=on_distance_m,
        off_distance_m=off_distance_m,
        start_state=start_state,
    )


def _parse_points_mode(raw) -> PointsModeConfig:
    if not isinstance(raw, dict):
        raise ConfigSchemaError("'points_mode' sub-config must be a dict")
    missing = [
        k
        for k in ("coordinates", "arrival_tolerance_m", "arrival_settle_s", "dwell_s")
        if k not in raw
    ]
    if missing:
        raise ConfigSchemaError(f"'points_mode' sub-config missing required field(s): {missing}")

    coordinates_raw = raw["coordinates"]
    if not isinstance(coordinates_raw, (list, tuple)):
        raise ConfigSchemaError("'points_mode.coordinates' must be a list")
    coordinates = tuple(
        _coerce_point(c, f"points_mode.coordinates[{i}]")
        for i, c in enumerate(coordinates_raw)
    )

    arrival_tolerance_m = _finite_float(raw["arrival_tolerance_m"], "points_mode.arrival_tolerance_m")
    if arrival_tolerance_m < 0:
        raise ConfigSchemaError(
            f"points_mode.arrival_tolerance_m must be >= 0, got {arrival_tolerance_m}"
        )

    arrival_settle_s = _finite_float(raw["arrival_settle_s"], "points_mode.arrival_settle_s")
    if arrival_settle_s < 0:
        raise ConfigSchemaError(
            f"points_mode.arrival_settle_s must be >= 0, got {arrival_settle_s}"
        )

    dwell_s = _finite_float(raw["dwell_s"], "points_mode.dwell_s")
    if dwell_s < 0:
        raise ConfigSchemaError(f"points_mode.dwell_s must be >= 0, got {dwell_s}")

    heading_tolerance_deg_raw = raw.get("heading_tolerance_deg")
    if heading_tolerance_deg_raw is None:
        heading_tolerance_deg = None
    else:
        heading_tolerance_deg = _finite_float(
            heading_tolerance_deg_raw, "points_mode.heading_tolerance_deg"
        )
        if heading_tolerance_deg < 0:
            raise ConfigSchemaError(
                f"points_mode.heading_tolerance_deg must be >= 0, got {heading_tolerance_deg}"
            )

    return PointsModeConfig(
        coordinates=coordinates,
        arrival_tolerance_m=arrival_tolerance_m,
        heading_tolerance_deg=heading_tolerance_deg,
        arrival_settle_s=arrival_settle_s,
        dwell_s=dwell_s,
    )


def parse_session_config(data: dict) -> SpraySessionConfig:
    """Parse+validate a raw session-config dict (e.g. from JSON).

    Raises ConfigSchemaError on any malformed input. Never raises anything
    else for bad input — callers can catch this one exception type.
    """
    if not isinstance(data, dict):
        raise ConfigSchemaError(f"session config must be a dict, got {type(data).__name__}")

    schema_version = data.get("schema_version")
    if schema_version != SCHEMA_VERSION:
        raise ConfigSchemaError(
            f"schema_version mismatch: expected {SCHEMA_VERSION}, got {schema_version!r}"
        )

    mode = data.get("mode")
    if mode not in _VALID_MODES:
        raise ConfigSchemaError(f"mode must be one of {_VALID_MODES}, got {mode!r}")

    points, flags = _coerce_points_flags(
        data.get("points", []), data.get("flags", []), label="session config"
    )

    dash_raw = data.get("dash")
    points_mode_raw = data.get("points_mode")
    dash_present = dash_raw is not None
    points_mode_present = points_mode_raw is not None

    dash: Optional[DashConfig] = None
    points_mode: Optional[PointsModeConfig] = None

    if mode == "dash":
        if not dash_present:
            raise ConfigSchemaError("mode='dash' requires a 'dash' sub-config")
        if points_mode_present:
            raise ConfigSchemaError("mode='dash' must not include a 'points_mode' sub-config")
        dash = _parse_dash(dash_raw)
    elif mode == "point":
        if not points_mode_present:
            raise ConfigSchemaError("mode='point' requires a 'points_mode' sub-config")
        if dash_present:
            raise ConfigSchemaError("mode='point' must not include a 'dash' sub-config")
        points_mode = _parse_points_mode(points_mode_raw)
    else:  # continuous
        if dash_present:
            raise ConfigSchemaError("mode='continuous' must not include a 'dash' sub-config")
        if points_mode_present:
            raise ConfigSchemaError("mode='continuous' must not include a 'points_mode' sub-config")

    return SpraySessionConfig(
        schema_version=SCHEMA_VERSION,
        mode=mode,
        points=points,
        flags=flags,
        dash=dash,
        points_mode=points_mode,
    )


def to_dict(cfg: SpraySessionConfig) -> dict:
    """Serialize a SpraySessionConfig back to a plain dict.

    Always JSON-serializable with `json.dumps(..., allow_nan=False)` since
    every float in a valid SpraySessionConfig was already checked finite at
    parse time. `dash`/`points_mode` keys are always present, `None` when
    unused, so `parse_session_config(to_dict(cfg))` round-trips structurally.
    """
    d = {
        "schema_version": cfg.schema_version,
        "mode": cfg.mode,
        "points": [[n, e] for n, e in cfg.points],
        "flags": list(cfg.flags),
        "dash": None,
        "points_mode": None,
    }
    if cfg.dash is not None:
        d["dash"] = {
            "on_distance_m": cfg.dash.on_distance_m,
            "off_distance_m": cfg.dash.off_distance_m,
            "start_state": cfg.dash.start_state,
        }
    if cfg.points_mode is not None:
        d["points_mode"] = {
            "coordinates": [[n, e] for n, e in cfg.points_mode.coordinates],
            "arrival_tolerance_m": cfg.points_mode.arrival_tolerance_m,
            "heading_tolerance_deg": cfg.points_mode.heading_tolerance_deg,
            "arrival_settle_s": cfg.points_mode.arrival_settle_s,
            "dwell_s": cfg.points_mode.dwell_s,
        }
    return d


def cleared_config() -> SpraySessionConfig:
    """Explicit cleared config: mode=continuous, empty points/flags.

    Used by mission-clear so a TRANSIENT_LOCAL-latched topic re-delivers an
    *explicit* clear to a restarted node instead of silently resurrecting a
    stale mission (plan §3).
    """
    return SpraySessionConfig(
        schema_version=SCHEMA_VERSION,
        mode="continuous",
        points=tuple(),
        flags=tuple(),
        dash=None,
        points_mode=None,
    )


def continuous_config_from_path(points: Sequence, flags: Sequence) -> SpraySessionConfig:
    """Build a continuous-mode SpraySessionConfig from `/path` geometry.

    Mirrors how `spray_controller_node.py` derives `points`/`flags` from the
    `/path` topic today (MARK flag = `position.z > 0.5`); this is how the
    node represents that geometry internally in Phase A. A degraded load
    (spray unavailable) is just this call with every flag False — no
    separate code path.
    """
    coerced_points, coerced_flags = _coerce_points_flags(
        list(points), list(flags), label="continuous_config_from_path"
    )
    return SpraySessionConfig(
        schema_version=SCHEMA_VERSION,
        mode="continuous",
        points=coerced_points,
        flags=coerced_flags,
        dash=None,
        points_mode=None,
    )
