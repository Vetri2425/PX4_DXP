"""Deterministic path identity helpers shared by server, RPP, and spray.

The identity intentionally covers only geometry and spray flags. Runtime
conditioning may resample or smooth between those flagged boundaries, so the
raw mission fingerprint is used to bind the mission configuration to the
conditioned geometry source, while the conditioned fingerprint is diagnostic.
"""

from __future__ import annotations

import hashlib
import json
import math
from typing import Iterable


PATH_IDENTITY_TOPIC = "/path/identity"
CONDITIONED_PATH_IDENTITY_TOPIC = "/rpp/conditioned_path_identity"


def path_geometry_fingerprint(
    points: Iterable[tuple[float, float]],
    flags: Iterable[bool],
    *,
    precision: int = 6,
) -> str:
    """Return a stable SHA-256 fingerprint for waypoint geometry + flags."""
    point_list = list(points)
    flag_list = list(flags)
    if len(point_list) != len(flag_list):
        raise ValueError("path points and flags must have equal length")
    normalized = []
    for index, (point, flag) in enumerate(zip(point_list, flag_list)):
        try:
            n_raw, e_raw = point
            n = float(n_raw)
            e = float(e_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"path point {index} must contain numeric north/east") from exc
        if not math.isfinite(n) or not math.isfinite(e):
            raise ValueError(f"path point {index} must contain finite coordinates")
        if not isinstance(flag, bool):
            raise ValueError(f"path flag {index} must be bool")
        n = round(n, precision)
        e = round(e, precision)
        if n == 0.0:
            n = 0.0
        if e == 0.0:
            e = 0.0
        normalized.append([n, e, 1 if flag else 0])
    if len(normalized) < 2:
        raise ValueError("path fingerprint requires at least two flagged points")
    payload = json.dumps(normalized, separators=(",", ":"), sort_keys=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def make_path_identity(
    *,
    mission_id: str,
    path_fingerprint: str,
    configuration_revision: int = 0,
    source: str = "mission",
) -> str:
    """Serialize a path identity envelope as compact JSON."""
    return json.dumps(
        {
            "mission_id": str(mission_id or ""),
            "path_fingerprint": str(path_fingerprint or ""),
            "configuration_revision": int(configuration_revision or 0),
            "source": str(source or ""),
        },
        separators=(",", ":"),
        sort_keys=True,
    )


def parse_path_identity(raw: str) -> dict[str, object]:
    """Parse a path identity envelope, returning empty fields on failure."""
    try:
        data = json.loads(raw)
    except (TypeError, ValueError):
        data = {}
    if not isinstance(data, dict):
        data = {}
    try:
        revision = int(data.get("configuration_revision", 0) or 0)
    except (TypeError, ValueError):
        revision = 0
    return {
        "mission_id": str(data.get("mission_id", "") or ""),
        "path_fingerprint": str(data.get("path_fingerprint", "") or ""),
        "configuration_revision": revision,
        "source": str(data.get("source", "") or ""),
    }
