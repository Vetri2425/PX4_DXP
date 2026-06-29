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
from spray_safety import force_spray_off_confirmed

log = get_logger("server.emergency")


class EmergencyHandler:
    def __init__(
        self,
        ros_node,
        offboard_controller,
        activity_log: deque,
        mission_capture=None,
    ) -> None:
        self._node       = ros_node
        self._controller = offboard_controller
        self._log        = activity_log
        self._mission_capture = mission_capture

    async def estop_async(self) -> dict:
        """Execute emergency stop. Returns {success, message}."""
        from mission_ops import MissionOperationCoordinator

        coordinator = MissionOperationCoordinator()
        try:
            from main import operation_coordinator

            if operation_coordinator is not None:
                coordinator = operation_coordinator
        except Exception:
            pass
        estop_token = coordinator.begin_estop_nowait()

        try:
            return await self._estop_async_with_token(estop_token)
        finally:
            await coordinator.finish(estop_token)

    async def _estop_async_with_token(self, estop_token) -> dict:
        # Guard: if ROS node is unavailable, short-circuit cleanly
        if self._node is None:
            msg = "ROS node not available — e-stop cannot reach FCU"
            ts = datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"
            self._log.append({"timestamp": ts, "level": "error", "message": msg})
            log.error(msg)
            return {"success": False, "message": msg}

        errors: list[str] = []

        try:
            from main import joystick_ctrl

            if joystick_ctrl is not None:
                joystick_ctrl.emergency_neutralize(reason="estop")
        except Exception as exc:
            errors.append(f"joystick neutralize: {exc}")
            log.exception("joystick neutralize during estop failed")

        # 1. Stop-path → RPP IDLE
        try:
            self._node.publish_stop_path()
        except Exception as exc:
            errors.append(f"publish_stop_path: {exc}")
            log.exception("publish_stop_path failed")

        # 2. Force spray OFF before changing mode/disarming.
        try:
            spray_off = await force_spray_off_confirmed(self._node, timeout_s=2.0)
            if not spray_off.success and spray_off.live:
                errors.append(f"spray OFF: {spray_off.message}")
        except Exception as exc:
            errors.append(f"spray OFF raised: {exc}")
            log.exception("spray OFF during estop raised")

        # 3. Switch to MANUAL
        try:
            ok, why = await self._node.set_mode_async("MANUAL")
            if not ok:
                errors.append(f"set_mode(MANUAL): {why}")
        except Exception as exc:
            errors.append(f"set_mode raised: {exc}")
            log.exception("set_mode(MANUAL) raised")

        # 4. Disarm
        try:
            ok, why = await self._node.arm_async(False)
            if not ok:
                errors.append(f"disarm: {why}")
        except Exception as exc:
            errors.append(f"arm(False) raised: {exc}")
            log.exception("arm(False) raised")

        try:
            from main import joystick_ctrl

            if joystick_ctrl is not None:
                await joystick_ctrl.force_release(reason="estop")
        except Exception as exc:
            errors.append(f"joystick release: {exc}")
            log.exception("joystick release during estop failed")

        # 5. Update mission state (hold lock only for the write, not around awaits)
        if self._controller is not None:
            async with self._controller._lifecycle_lock():
                self._controller.state = MissionState.ABORTED

        try:
            from control_arbiter import get_control_arbiter

            get_control_arbiter().mark_idle_if_not_joystick()
        except Exception as exc:
            errors.append(f"control_arbiter: {exc}")
            log.exception("control arbiter reset during estop failed")

        try:
            import asyncio

            from main import hold_owner, point_mission
            from point_mission import PointMissionState

            if point_mission is not None:
                try:
                    await asyncio.wait_for(
                        point_mission.terminal_cleanup(
                            self._node,
                            hold_owner,
                            reason="emergency_stop",
                            terminal_state=PointMissionState.ABORTING,
                            operation_token=estop_token,
                            require_spray_confirm=False,
                        ),
                        timeout=0.5,
                    )
                except asyncio.TimeoutError:
                    errors.append("point terminal cleanup bookkeeping timed out")
        except Exception as exc:
            errors.append(f"point terminal cleanup: {exc}")
            log.exception("point terminal cleanup during estop failed")

        msg = "EMERGENCY STOP executed"
        if errors:
            msg += " (with errors: " + "; ".join(errors) + ")"
            level = "warning"
        else:
            level = "error"

        ts = datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"
        self._log.append({"timestamp": ts, "level": "error", "message": msg})
        getattr(log, level)(msg)

        if self._mission_capture is not None:
            self._mission_capture.record_terminal(
                None,
                "emergency_stop",
                state=MissionState.ABORTED.value,
                details={"success": not errors, "errors": errors},
            )

        return {"success": not errors, "message": msg}
