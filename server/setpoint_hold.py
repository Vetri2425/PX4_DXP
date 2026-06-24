"""OFFBOARD hold owner — single-writer mux between RPP tracking and fixed hold.

Hold strategy (Batch 3): publish a one-point ``nav_msgs/Path`` at the latched
NED pose so RPP enters DONE and ``twist_to_setpoint`` streams zero velocity.
This is **not** absolute position-target OFFBOARD; it is the proven stop-path
semantic documented in ``RosBridgeNode.publish_stop_path()``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass
class HoldLatch:
    north_m: float
    east_m: float
    heading_ned_rad: float | None = None


class SetpointHoldOwner:
    SOURCE_RPP = "rpp"
    SOURCE_HOLD = "hold"

    def __init__(self) -> None:
        self._active = False
        self._latch: HoldLatch | None = None
        self._reason = ""

    @property
    def active(self) -> bool:
        return self._active

    @property
    def source(self) -> str:
        return self.SOURCE_HOLD if self._active else self.SOURCE_RPP

    def activate(
        self,
        ros_node,
        *,
        north_m: float,
        east_m: float,
        heading_ned_rad: float | None = None,
        reason: str = "",
    ) -> None:
        """Latch pose and publish single-point hold path (RPP → zero velocity)."""
        self._latch = HoldLatch(
            north_m=float(north_m),
            east_m=float(east_m),
            heading_ned_rad=heading_ned_rad,
        )
        self._reason = str(reason or "")
        self._active = True
        if ros_node is not None:
            ros_node.publish_path(
                [(self._latch.north_m, self._latch.east_m)],
                spray_flags=[False],
            )

    def refresh(self, ros_node) -> None:
        """Re-publish hold path to keep RPP at DONE (drift recovery)."""
        if not self._active or self._latch is None or ros_node is None:
            return
        ros_node.publish_path(
            [(self._latch.north_m, self._latch.east_m)],
            spray_flags=[False],
        )

    def deactivate(self, ros_node=None) -> None:
        self._active = False
        self._latch = None
        self._reason = ""

    def hold_error_m(self, ros_node) -> float | None:
        if not self._active or self._latch is None or ros_node is None:
            return None
        state = ros_node.get_state()
        if not state.get("pose_received", False):
            return None
        return math.hypot(
            float(state.get("pos_n", 0.0)) - self._latch.north_m,
            float(state.get("pos_e", 0.0)) - self._latch.east_m,
        )

    def as_dict(self, ros_node=None) -> dict[str, Any]:
        latch = self._latch
        return {
            "setpoint_source": self.source,
            "hold_active": self._active,
            "hold_north_m": latch.north_m if latch else None,
            "hold_east_m": latch.east_m if latch else None,
            "hold_heading_ned_rad": latch.heading_ned_rad if latch else None,
            "hold_error_m": self.hold_error_m(ros_node),
            "hold_reason": self._reason,
        }