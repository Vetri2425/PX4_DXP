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

    The split is keyed on the DUPLICATED waypoint, not on the first spray-ON
    point. Waypoint 0 is only immediately ON when the mission starts marking
    there; with per-line PRE extensions the mission opens with an OFF run-up, so
    the first ON point sits several waypoints past the duplicate. Keying on the
    ON point missed that case entirely and returned no entry run, which left the
    acquisition leg fused into the first mission run.
    """
    if not marked or len(points) < 4 or len(points) != len(flags) or flags[0]:
        return None, list(points), list(flags)

    for i in range(1, len(points)):
        if flags[i - 1]:
            break  # marking began before any duplicate — not a runtime entry
        duplicate = (
            math.hypot(
                points[i - 1][0] - points[i][0],
                points[i - 1][1] - points[i][1],
            )
            < 1e-6
        )
        if not duplicate:
            continue
        moved = (
            math.hypot(
                points[0][0] - points[i - 1][0],
                points[0][1] - points[i - 1][1],
            )
            >= 1e-6
        )
        leading_off = not any(flags[:i])
        if moved and leading_off:
            entry_pts = list(points[:i])
            entry = (entry_pts, [False] * len(entry_pts))
            return entry, list(points[i:]), list(flags[i:])
        break
    return None, list(points), list(flags)
