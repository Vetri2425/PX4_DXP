"""Pure helpers for RPP path conditioning."""
from __future__ import annotations

import math


def split_leading_entry_transit(
    points: list[tuple[float, float]], flags: list[bool], *, marked: bool
) -> tuple[
    tuple[list[tuple[float, float]], list[bool]] | None,
    list[tuple[float, float]],
    list[bool],
]:
    """Extract rover->waypoint0 OFF before the duplicated mission entry."""
    if (
        marked
        and len(points) >= 4
        and not flags[0]
        and not flags[1]
        and math.hypot(
            points[1][0] - points[2][0], points[1][1] - points[2][1]
        ) < 1e-6
        and math.hypot(
            points[0][0] - points[1][0], points[0][1] - points[1][1]
        ) >= 1e-6
    ):
        entry = ([points[0], points[1]], [False, False])
        return entry, list(points[2:]), list(flags[2:])
    return None, list(points), list(flags)
