"""Small, dependency-free NTRIP protocol validation helpers."""

from __future__ import annotations

import math
import re


_SUCCESS_LINE = re.compile(r"^(?:ICY|HTTP/\d+(?:\.\d+)?)\s+200(?:\s|$)", re.IGNORECASE)


def response_is_success(header: str) -> bool:
    """Accept only an ICY/HTTP status line whose exact status code is 200."""
    first_line = header.splitlines()[0].strip() if header.splitlines() else ""
    return bool(_SUCCESS_LINE.match(first_line))


def gga_position_is_usable(
    latitude: float,
    longitude: float,
    altitude: float,
    age_s: float,
    *,
    max_age_s: float = 5.0,
) -> bool:
    """Reject stale, non-finite, or out-of-range fixes before GGA back-feed."""
    values = (latitude, longitude, altitude, age_s)
    if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
        return False
    return (
        -90.0 <= latitude <= 90.0
        and -180.0 <= longitude <= 180.0
        and 0.0 <= age_s <= max(0.0, max_age_s)
    )
