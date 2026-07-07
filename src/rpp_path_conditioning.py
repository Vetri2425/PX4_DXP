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
    """Extract rover->waypoint0 OFF before the duplicated mission entry.

    Runtime GPS_SURVEYED entry prepends an OFF acquisition leg and duplicates
    waypoint 0 as OFF then ON. The leg may be densified, so the duplicate is not
    necessarily at points[1]/points[2].
    """
    if not marked or len(points) < 4 or len(points) != len(flags) or flags[0]:
        return None, list(points), list(flags)

    for i in range(1, len(points)):
        if flags[i - 1]:
            break
        duplicate = (
            math.hypot(
                points[i - 1][0] - points[i][0],
                points[i - 1][1] - points[i][1],
            )
            < 1e-6
        )
        moved = (
            math.hypot(
                points[0][0] - points[i - 1][0],
                points[0][1] - points[i - 1][1],
            )
            >= 1e-6
        )
        leading_off = not any(flags[:i])
        mark_or_pre_boundary = bool(flags[i] or any(flags[i:]))
        if duplicate and moved and leading_off and mark_or_pre_boundary:
            entry_pts = list(points[:i])
            entry = (entry_pts, [False] * len(entry_pts))
            return entry, list(points[i:]), list(flags[i:])
        if flags[i]:
            break
    return None, list(points), list(flags)
