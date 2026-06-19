"""OFFBOARD mission lifecycle state machine (async).

States:
  IDLE → ARMING → SWITCHING_OFFBOARD → RUNNING → STOPPING → IDLE
                                         ↓ (estop / abort / safety)
                                       ABORTED
                          RUNNING ─→ COMPLETED  (auto, when RPP DONE settled)

All public methods that touch ROS services are async — they delegate to
`RosBridgeNode.arm_async() / set_mode_async()` so the FastAPI event loop
is never blocked.
"""
from __future__ import annotations

import asyncio
import datetime
from collections import deque
from typing import Any, Optional

from config import (
    RPP_IDLE,
    RPP_STALE,
    RPP_UNHEALTHY_CODES,
    SETPOINT_STREAM_GRACE_S,
)
from logging_setup import get_logger
from mission_loading import MissionLoadConflict, load_block_reason, pose_origin_or_error
from mission_placement import (
    GPS_SURVEYED,
    LOCAL_NED,
    PlacementError,
    resolve_surveyed_points,
)
from models import MissionState

log = get_logger("server.offboard")

STOP_ALLOWED_STATES = {
    MissionState.RUNNING,
    MissionState.ARMING,
    MissionState.SWITCHING_OFFBOARD,
}
ABORT_NOOP_STATES = {
    MissionState.IDLE,
    MissionState.COMPLETED,
    MissionState.ABORTED,
}
STOP_SETTLE_S = 0.1


class OffboardController:
    def __init__(self, ros_node, activity_log: deque) -> None:
        self._node       = ros_node
        self._log        = activity_log
        self._state      = MissionState.IDLE
        self._loaded_source_pts: tuple[tuple[float, float], ...] = ()
        self._loaded_spray_flags: tuple[bool, ...] | None = None
        self._path_name: str | None = None
        self._loaded_mission_id: str | None = None
        self._running_mission_id: str | None = None
        self._source_name: str | None = None
        self._placement_mode = LOCAL_NED
        self._origin_gps: tuple[float, float] | None = None
        self._is_staged_mission = False
        # Serialises lifecycle calls. Created lazily on first use: on
        # Python 3.9 asyncio.Lock() binds an event loop at construction,
        # and the controller is built at server startup outside any loop.
        self._lock: asyncio.Lock | None = None

    def _lifecycle_lock(self) -> asyncio.Lock:
        # Only ever called from coroutines on the server's single event
        # loop, so the check-then-create is race-free.
        if self._lock is None:
            self._lock = asyncio.Lock()
        return self._lock

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def state(self) -> MissionState:
        return self._state

    @state.setter
    def state(self, value: MissionState) -> None:
        self._state = value

    @property
    def loaded_path_name(self) -> Optional[str]:
        return self._path_name

    @property
    def loaded_mission_id(self) -> Optional[str]:
        return self._loaded_mission_id

    @property
    def running_mission_id(self) -> Optional[str]:
        return self._running_mission_id

    @property
    def placement_mode(self) -> str:
        return self._placement_mode

    @property
    def has_protected_mission(self) -> bool:
        return bool(self._loaded_source_pts) and (
            self._placement_mode == GPS_SURVEYED
            or self._is_staged_mission
        )

    def loaded_path_summary(self, sample: int = 20) -> dict:
        """Read-only snapshot of the path currently resident in the controller.

        Used by GET /api/mission/loaded-path to confirm what coordinates were
        actually committed (stage 10). Returns counts + a head/tail coordinate
        sample so the operator can verify without shipping the full array.
        """
        pts = self._loaded_source_pts
        flags = self._loaded_spray_flags
        num_mark = sum(1 for f in flags) if flags else 0
        if flags is not None:
            num_mark = sum(1 for f in flags if f)
        sample = max(0, int(sample))
        if sample and len(pts) > 2 * sample:
            sample_coords = [list(p) for p in pts[:sample]] + [list(p) for p in pts[-sample:]]
            sample_truncated = True
        else:
            sample_coords = [list(p) for p in pts]
            sample_truncated = False
        return {
            "loaded": bool(pts),
            "name": self._path_name,
            "mission_id": self._loaded_mission_id,
            "running_mission_id": self._running_mission_id,
            "source_name": self._source_name,
            "placement_mode": self._placement_mode,
            "origin_gps": list(self._origin_gps) if self._origin_gps else None,
            "is_staged": self._is_staged_mission,
            "protected": self.has_protected_mission,
            "state": self._state.value,
            "num_waypoints": len(pts),
            "num_mark": num_mark,
            "num_transit": (len(flags) - num_mark) if flags else 0,
            "has_spray_flags": flags is not None,
            "sample_coords": sample_coords,
            "sample_truncated": sample_truncated,
        }

    # ── Path management ───────────────────────────────────────────────────────

    def load_path(
        self,
        points: list[tuple[float, float]],
        name: Optional[str] = None,
        spray_flags: Optional[list[bool]] = None,
        *,
        placement_mode: str = LOCAL_NED,
        origin_gps: tuple[float, float] | None = None,
        mission_id: str | None = None,
        source_name: str | None = None,
        is_staged: bool = False,
        allow_replace_protected: bool = False,
    ) -> None:
        reason = load_block_reason(self._state)
        if reason:
            raise MissionLoadConflict(reason)
        if placement_mode not in (LOCAL_NED, GPS_SURVEYED):
            raise ValueError(f"unsupported placement mode: {placement_mode!r}")

        new_mission_id = mission_id or name or "unknown"
        if (
            self.has_protected_mission
            and not allow_replace_protected
        ):
            raise MissionLoadConflict(
                f"Loaded mission {self._loaded_mission_id!r} is staged/surveyed; "
                "use the staged mission workflow to replace it"
            )

        self._loaded_source_pts = tuple(
            (float(n), float(e)) for n, e in points
        )
        if spray_flags is not None and len(spray_flags) == len(points):
            self._loaded_spray_flags = tuple(bool(f) for f in spray_flags)
        elif spray_flags is not None:
            self._loaded_spray_flags = None
            self._log_entry(
                "warning",
                f"spray_flags length mismatch for {name or 'unknown'} — loading path with spray OFF",
            )
        else:
            self._loaded_spray_flags = None
        self._path_name = name or source_name or new_mission_id
        self._loaded_mission_id = new_mission_id
        self._running_mission_id = None
        self._source_name = source_name or name or new_mission_id
        self._placement_mode = placement_mode
        self._is_staged_mission = bool(is_staged)
        self._origin_gps = (
            (float(origin_gps[0]), float(origin_gps[1]))
            if origin_gps is not None else None
        )
        if self._state in (MissionState.COMPLETED, MissionState.ABORTED, MissionState.ERROR):
            self._state = MissionState.IDLE
        # Reset RPP done-settle timer so a leftover DONE from the previous
        # mission does not trigger instant auto-completion of the new one.
        if self._node is not None:
            try:
                self._node.get_rpp_monitor().reset()
            except Exception:
                pass
        self._log_entry(
            "info",
            f"Path loaded: {self._path_name} ({len(points)} pts, "
            f"id={self._loaded_mission_id}, placement={self._placement_mode})",
        )

    # ── Lifecycle (async) ─────────────────────────────────────────────────────

    def _rpp_unhealthy_start_message(self, rpp_code: int) -> str:
        if rpp_code == RPP_STALE:
            return "start: RPP STALE — is twist_to_setpoint_node running?"
        if rpp_code == 4:  # RPP_RTK_WAIT
            return (
                "start: RPP RTK_WAIT — GPS fix < RTK_FIXED. "
                "Wait for fix or set require_rtk_fix:=false on the controller."
            )
        if rpp_code == 5:  # RPP_JUMP_SKIP
            return (
                "start: RPP JUMP_SKIP — EKF position jump in progress; "
                "retry in ~1 s once the estimator settles."
            )
        return f"start: RPP unhealthy (code={rpp_code})"

    async def start_async(
        self,
        auto_origin: bool = False,
        expected_mission_id: str | None = None,
    ) -> tuple[bool, str]:
        async with self._lifecycle_lock():
            if self._node is None:
                return False, "ROS node not available"

            # Guard: re-starting while already running re-arms and re-switches
            # OFFBOARD, which is wrong. Operator must stop first.
            if self._state == MissionState.RUNNING:
                msg = "start: mission already running — call stop first"
                self._log_entry("warning", msg)
                return False, msg

            if self._state in (
                MissionState.LOADING,
                MissionState.ARMING,
                MissionState.SWITCHING_OFFBOARD,
                MissionState.STOPPING,
                MissionState.DISARMING,
            ):
                msg = f"start: controller state is {self._state.value} — wait until idle"
                self._log_entry("warning", msg)
                return False, msg

            if not self._loaded_source_pts:
                self._state = MissionState.ERROR
                msg = "start: no path loaded"
                self._log_entry("error", msg)
                return False, msg

            if (
                expected_mission_id is not None
                and expected_mission_id != self._loaded_mission_id
            ):
                msg = (
                    f"start: mission identity mismatch: expected {expected_mission_id!r}, "
                    f"loaded {self._loaded_mission_id!r}"
                )
                self._log_entry("error", msg)
                return False, msg

            if self._placement_mode == GPS_SURVEYED and auto_origin:
                msg = "start: GPS_SURVEYED missions are incompatible with auto_origin"
                self._log_entry("error", msg)
                self._running_mission_id = None
                raise PlacementError(msg)

            fcu = self._node.get_state()
            if not fcu.get("connected", False):
                self._state = MissionState.ERROR
                msg = "start: FCU not connected"
                self._log_entry("error", msg)
                return False, msg

            pts_to_publish = list(self._loaded_source_pts)
            spray_flags_to_publish = (
                list(self._loaded_spray_flags)
                if self._loaded_spray_flags is not None else None
            )
            if self._placement_mode == GPS_SURVEYED:
                try:
                    pts_to_publish, translation = resolve_surveyed_points(
                        self._loaded_source_pts,
                        self._origin_gps,
                        fcu,
                    )
                except (PlacementError, ImportError) as exc:
                    self._state = MissionState.ERROR
                    msg = f"start: surveyed placement failed: {exc}"
                    self._log_entry("error", msg)
                    self._running_mission_id = None
                    raise PlacementError(msg) from exc
                self._log_entry(
                    "info",
                    "survey placement offset: "
                    f"{translation[0]:+.3f}N {translation[1]:+.3f}E",
                )

            # Surveyed placement is validated first so stale pose/GPS and RTK
            # quality failures retain their typed 422 contract instead of being
            # masked by the RPP STALE/RTK_WAIT state.
            rpp_code = fcu.get("rpp_state", RPP_STALE)
            if rpp_code in RPP_UNHEALTHY_CODES:
                self._state = MissionState.ERROR
                msg = self._rpp_unhealthy_start_message(rpp_code)
                self._log_entry("error", msg)
                return False, msg

            if self._placement_mode != GPS_SURVEYED and auto_origin:
                pose_origin = pose_origin_or_error(self._node.get_state())
                if isinstance(pose_origin, str):
                    self._state = MissionState.ERROR
                    msg = f"start: {pose_origin}"
                    self._log_entry("error", msg)
                    return False, msg
                off_n, off_e = pose_origin
                pts_to_publish = [
                    (n + off_n, e + off_e) for n, e in self._loaded_source_pts
                ]
                self._log_entry(
                    "info", f"auto_origin offset: +{off_n:.3f}N +{off_e:.3f}E"
                )

            armed_here = False
            try:
                # Publish the mission path before the OFFBOARD request so the
                # 50 Hz setpoint stream carries mission setpoints, not just the
                # streamer's zero-velocity bootstrap, when PX4 evaluates entry.
                self._node.publish_path(
                    pts_to_publish,
                    spray_flags=spray_flags_to_publish,
                )

                # ── Arm ───────────────────────────────────────────────────────
                self._state = MissionState.ARMING
                self._log_entry("info", "arming…")
                ok, why = await self._node.arm_async(True)
                if not ok:
                    self._state = MissionState.ERROR
                    self._log_entry("error", f"arming failed: {why}")
                    return False, f"arm failed: {why}"
                armed_here = True

                # ── Switch to OFFBOARD ────────────────────────────────────────
                self._state = MissionState.SWITCHING_OFFBOARD
                self._log_entry("info", "switching to OFFBOARD…")
                await asyncio.sleep(SETPOINT_STREAM_GRACE_S)
                fcu = self._node.get_state()
                rpp_code = fcu.get("rpp_state", RPP_STALE)
                if rpp_code in RPP_UNHEALTHY_CODES:
                    self._state = MissionState.ERROR
                    msg = self._rpp_unhealthy_start_message(rpp_code)
                    self._log_entry("error", msg)
                    await self._node.arm_async(False)
                    return False, msg
                if rpp_code == RPP_IDLE:
                    self._state = MissionState.ERROR
                    msg = (
                        "start: RPP IDLE after path publish — "
                        "setpoint chain not ready"
                    )
                    self._log_entry("error", msg)
                    await self._node.arm_async(False)
                    return False, msg
                ok, why = await self._node.set_mode_async("OFFBOARD")
                if not ok:
                    self._state = MissionState.ERROR
                    self._log_entry("error", f"OFFBOARD switch failed: {why}")
                    # Best-effort disarm; ignore result
                    await self._node.arm_async(False)
                    return False, f"OFFBOARD failed: {why}"

                self._state = MissionState.RUNNING
                self._running_mission_id = self._loaded_mission_id
                self._log_entry("info", f"mission running: {self._path_name}")
                return True, "running"
            except Exception as exc:
                self._state = MissionState.ERROR
                self._running_mission_id = None
                self._log_entry("error", f"unexpected start failure: {exc}")
                if armed_here:
                    try:
                        await self._node.arm_async(False)
                    except Exception:
                        pass
                return False, f"unexpected start failure: {exc}"

    async def stop_async(self) -> dict[str, Any]:
        """Soft stop: publish a single-point stop-path → RPP zeroes velocity.

        Empty Path is **ignored** by upstream RPP (early-return), so we
        publish a stop-path at the rover's current position. RPP treats it
        as DONE immediately and outputs zero velocity. Vehicle stays armed.
        """
        async with self._lifecycle_lock():
            if self._node is None:
                msg = "stop: ROS node not available"
                self._log_entry("warning", msg)
                return {
                    "success": False,
                    "state": self._state.value,
                    "action": "no_node",
                    "armed": None,
                    "message": msg,
                }

            if self._state not in STOP_ALLOWED_STATES:
                msg = f"stop called from {self._state.value} — no active mission to stop"
                self._log_entry("info", msg)
                s = self._node.get_state()
                return {
                    "success": False,
                    "state": self._state.value,
                    "action": "no_op",
                    "armed": s.get("armed"),
                    "message": msg,
                }

            try:
                self._state = MissionState.STOPPING
                stop_position = self._node.publish_stop_path()
                if stop_position is None:
                    self._state = MissionState.ERROR
                    s = self._node.get_state()
                    msg = "stop: no local pose available; stop-path not published"
                    self._log_entry("error", msg)
                    return {
                        "success": False,
                        "state": self._state.value,
                        "action": "no_pose",
                        "armed": s.get("armed"),
                        "message": msg,
                    }

                await asyncio.sleep(STOP_SETTLE_S)
                self._state = MissionState.IDLE
                self._running_mission_id = None
                s = self._node.get_state()
                n, e = stop_position
                msg = f"mission stopped at N={n:.3f}, E={e:.3f}"
                self._log_entry("info", msg)
                return {
                    "success": True,
                    "state": self._state.value,
                    "action": "hold_position",
                    "armed": s.get("armed"),
                    "message": msg,
                    "stop_position": {"n": n, "e": e},
                }
            except Exception as exc:
                self._state = MissionState.ERROR
                msg = f"stop failed: {exc}"
                self._log_entry("error", msg)
                try:
                    s = self._node.get_state()
                    armed = s.get("armed")
                except Exception:
                    armed = None
                return {
                    "success": False,
                    "state": self._state.value,
                    "action": "error",
                    "armed": armed,
                    "message": msg,
                }

    async def abort_async(self) -> dict[str, Any]:
        """Hard abort: stop-path + MANUAL + disarm."""
        async with self._lifecycle_lock():
            if self._node is None:
                msg = "abort: ROS node not available"
                self._log_entry("warning", msg)
                return {
                    "success": False,
                    "state": self._state.value,
                    "action": "no_node",
                    "message": msg,
                    "errors": [msg],
                    "stop_path_sent": False,
                    "manual_mode": False,
                    "disarmed": False,
                    "armed": None,
                }

            if self._state in ABORT_NOOP_STATES:
                msg = f"abort called from {self._state.value} — no active mission to abort"
                self._log_entry("info", msg)
                s = self._node.get_state()
                return {
                    "success": True,
                    "state": self._state.value,
                    "action": "no_op",
                    "message": msg,
                    "errors": [],
                    "stop_path_sent": False,
                    "manual_mode": s.get("mode") == "MANUAL",
                    "disarmed": not bool(s.get("armed")),
                    "armed": s.get("armed"),
                }

            errors: list[str] = []
            stop_position: tuple[float, float] | None = None
            manual_mode = False
            disarmed = False

            try:
                stop_position = self._node.publish_stop_path()
                if stop_position is None:
                    errors.append("publish_stop_path: no local pose available")
            except Exception as exc:
                errors.append(f"publish_stop_path raised: {exc}")
                log.exception("abort publish_stop_path raised")

            try:
                ok, why = await self._node.set_mode_async("MANUAL")
                manual_mode = bool(ok)
                if not ok:
                    errors.append(f"set_mode(MANUAL): {why}")
            except Exception as exc:
                errors.append(f"set_mode(MANUAL) raised: {exc}")
                log.exception("abort set_mode(MANUAL) raised")

            self._state = MissionState.DISARMING
            try:
                ok, why = await self._node.arm_async(False)
                disarmed = bool(ok)
                if not ok:
                    errors.append(f"disarm: {why}")
            except Exception as exc:
                errors.append(f"disarm raised: {exc}")
                log.exception("abort disarm raised")

            self._state = MissionState.ABORTED
            self._running_mission_id = None
            try:
                s = self._node.get_state()
                armed = s.get("armed")
                if s.get("mode") == "MANUAL":
                    manual_mode = True
                if armed is False:
                    disarmed = True
            except Exception:
                armed = None

            msg = "mission ABORTED — MANUAL + disarm"
            if errors:
                msg += " (with errors: " + "; ".join(errors) + ")"
            self._log_entry("warning" if errors else "error", msg)

            result: dict[str, Any] = {
                "success": not errors,
                "state": self._state.value,
                "action": "abort",
                "message": msg,
                "errors": errors,
                "stop_path_sent": stop_position is not None,
                "manual_mode": manual_mode,
                "disarmed": disarmed,
                "armed": armed,
            }
            if stop_position is not None:
                n, e = stop_position
                result["stop_position"] = {"n": n, "e": e}
            return result

    async def disarm_async(self) -> bool:
        async with self._lifecycle_lock():
            if self._node is None:
                self._log_entry("warning", "disarm: ROS node not available")
                return False

            ok, why = await self._node.arm_async(False)
            self._state = MissionState.IDLE
            self._running_mission_id = None
            self._log_entry(
                "info" if ok else "error",
                f"disarm {'ok' if ok else f'failed: {why}'}",
            )
            return ok

    # Called from telemetry loop — no async lock to avoid blocking the loop.
    def mark_completed(self) -> None:
        if self._state == MissionState.RUNNING:
            self._state = MissionState.COMPLETED
            self._running_mission_id = None
            self._log_entry("info", f"mission completed: {self._path_name}")

    # ── Internal ──────────────────────────────────────────────────────────────

    def _log_entry(self, level: str, message: str) -> None:
        ts = datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"
        self._log.append({"timestamp": ts, "level": level, "message": message})
        getattr(log, level if level in ("info", "warning", "error") else "info")(message)
