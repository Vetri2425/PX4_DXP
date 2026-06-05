"""Emergency stop handler (async).

Flow:
  1. Publish single-point stop-path → RPP outputs zero velocity
     (empty Path is ignored by the upstream RPP node)
  2. Switch to MANUAL → PX4 exits OFFBOARD, stops motors
  3. Disarm
  4. Update offboard controller state → ABORTED
"""
from __future__ import annotations

import datetime
from collections import deque

from logging_setup import get_logger
from models import MissionState

log = get_logger("server.emergency")


class EmergencyHandler:
    def __init__(
        self,
        ros_node,
        offboard_controller,
        activity_log: deque,
    ) -> None:
        self._node       = ros_node
        self._controller = offboard_controller
        self._log        = activity_log

    async def estop_async(self) -> dict:
        """Execute emergency stop. Returns {success, message}."""
        # Guard: if ROS node is unavailable, short-circuit cleanly
        if self._node is None:
            msg = "ROS node not available — e-stop cannot reach FCU"
            ts = datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"
            self._log.append({"timestamp": ts, "level": "error", "message": msg})
            log.error(msg)
            return {"success": False, "message": msg}

        errors: list[str] = []

        # 1. Stop-path → RPP IDLE
        try:
            self._node.publish_stop_path()
        except Exception as exc:
            errors.append(f"publish_stop_path: {exc}")
            log.exception("publish_stop_path failed")

        # 2. Switch to MANUAL
        try:
            ok, why = await self._node.set_mode_async("MANUAL")
            if not ok:
                errors.append(f"set_mode(MANUAL): {why}")
        except Exception as exc:
            errors.append(f"set_mode raised: {exc}")
            log.exception("set_mode(MANUAL) raised")

        # 3. Disarm
        try:
            ok, why = await self._node.arm_async(False)
            if not ok:
                errors.append(f"disarm: {why}")
        except Exception as exc:
            errors.append(f"arm(False) raised: {exc}")
            log.exception("arm(False) raised")

        # 4. Update mission state (hold lock only for the write, not around awaits)
        if self._controller is not None:
            async with self._controller._lock:
                self._controller.state = MissionState.ABORTED

        msg = "EMERGENCY STOP executed"
        if errors:
            msg += " (with errors: " + "; ".join(errors) + ")"
            level = "warning"
        else:
            level = "error"

        ts = datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"
        self._log.append({"timestamp": ts, "level": "error", "message": msg})
        getattr(log, level)(msg)

        return {"success": not errors, "message": msg}
