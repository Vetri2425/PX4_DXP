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
import math
from collections import deque
from typing import Any, Optional

import config
from config import (
    RPP_IDLE,
    RPP_STALE,
    RPP_UNHEALTHY_CODES,
    SETPOINT_STREAM_GRACE_S,
)
from control_arbiter import ControlArbiter, ControlArbiterError, get_control_arbiter
from logging_setup import get_logger
from mission_loading import pose_origin_or_error
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
    MissionState.ENTRY,          # D1: entry leg is stoppable like a running mission
    MissionState.ARMING,
    MissionState.SWITCHING_OFFBOARD,
}
# D1: if the live rover is already within this of the first mission point, skip
# the entry leg and publish the marking path directly (degenerate entry —
# e.g. LOCAL_NED auto-origin places wp0 at the rover). Metres.
ENTRY_SKIP_DIST_M = 0.20
ABORT_NOOP_STATES = {
    MissionState.IDLE,
    MissionState.COMPLETED,
    MissionState.ABORTED,
}
# Resident mission may be cleared only from idle/terminal states — never
# mid-flight. Matches CLEAR_ALLOWED_STATES on fix/runtime-entry-stop.
CLEAR_ALLOWED_STATES = {
    MissionState.IDLE,
    MissionState.COMPLETED,
    MissionState.ABORTED,
    MissionState.ERROR,
}
STOP_SETTLE_S = 0.1


class MissionClearConflict(Exception):
    """Raised when resident mission state cannot be cleared safely."""


class OffboardController:
    def __init__(
        self,
        ros_node,
        activity_log: deque,
        *,
        arbiter: ControlArbiter | None = None,
    ) -> None:
        self._node       = ros_node
        self._log        = activity_log
        # Mission↔joystick mutual-exclusion. Wired into start_async so a mission
        # cannot begin while the joystick owns manual control; the reverse guard
        # (joystick cannot acquire mid-mission) lives in the joystick controller
        # and reads self.state. See docs/Architecture/JOYSTICK_CONTROLLER_PLAN.md
        # §7.4 and control_arbiter.mission_start().
        self._arbiter    = arbiter or get_control_arbiter()
        self._state      = MissionState.IDLE
        self._loaded_pts: list[tuple[float, float]] | None = None
        self._loaded_spray_flags: list[bool] | None = None
        self._loaded_must_hit: list[bool] | None = None
        self._path_name: str | None = None
        self._placement_mode = LOCAL_NED
        self._origin_gps: tuple[float, float] | None = None
        self._is_staged_mission = False
        # D1: marking path + flags stashed while the ENTRY leg drives to the
        # first point; the telemetry loop publishes them via
        # advance_entry_to_marking() once the entry stop is confirmed (RPP DONE).
        self._entry_marking_pts: list[tuple[float, float]] | None = None
        self._entry_marking_flags: list[bool] | None = None
        self._entry_marking_must_hit: list[bool] | None = None
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
    def placement_mode(self) -> str:
        return self._placement_mode

    def loaded_path_summary(self, sample: int = 20) -> dict:
        """Read-only snapshot of the path currently resident in the controller.

        Used by GET /api/mission/loaded-path to confirm what coordinates were
        actually committed (stage 10). Returns counts + a head/tail coordinate
        sample so the operator can verify without shipping the full array.
        """
        pts = self._loaded_pts or []
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
            "state": self._state.value,
            "num_waypoints": len(pts),
            "num_mark": num_mark,
            "num_transit": (len(flags) - num_mark) if flags else 0,
            "has_spray_flags": flags is not None,
            "sample_coords": sample_coords,
            "sample_truncated": sample_truncated,
            "placement_mode": self._placement_mode,
            "origin_gps": list(self._origin_gps) if self._origin_gps else None,
            "is_staged": self._is_staged_mission,
            # Staged missions load with name=<stg mission_id> (see
            # /load-to-controller), so _path_name IS the staged id here. Expose it
            # as mission_id + mark protected so the operator app's post-load
            # verifyStagedLoadedMission can match staged↔loaded instead of failing
            # "does not match staged path".
            "mission_id": self._path_name if self._is_staged_mission else None,
            "protected": self._is_staged_mission
            or self._placement_mode == GPS_SURVEYED,
        }

    async def clear_mission_async(self) -> dict[str, Any]:
        """Clear the resident mission without deleting its source artifact.

        In-memory only: wipes the loaded path + placement so a fresh mission can
        be loaded. Guarded to idle/terminal states — a live mission must be
        stopped or aborted first. Ported from fix/runtime-entry-stop, reduced to
        this baseline's controller state (the reference also reset mission_id /
        fingerprint / dash / spray-config fields that do not exist here).
        """
        async with self._lifecycle_lock():
            if self._state not in CLEAR_ALLOWED_STATES:
                raise MissionClearConflict(
                    f"Cannot clear mission while controller state is "
                    f"{self._state.value}; stop or abort the mission first"
                )
            cleared_name = self._path_name
            self._loaded_pts = None
            self._loaded_spray_flags = None
            self._loaded_must_hit = None
            self._path_name = None
            self._placement_mode = LOCAL_NED
            self._origin_gps = None
            self._is_staged_mission = False
            self._entry_marking_pts = None
            self._entry_marking_flags = None
            self._entry_marking_must_hit = None
            self._state = MissionState.IDLE
            # Optional path-topic clear if this branch's node grows the hook;
            # baseline has publish_stop_path only, so this self-skips (clear is
            # in-memory only and runs from idle/terminal states anyway).
            if self._node is not None and hasattr(self._node, "publish_path_clear"):
                self._node.publish_path_clear()
            self._log_entry(
                "info", f"Resident mission cleared: {cleared_name or 'none'}"
            )
            return self.loaded_path_summary()

    # ── Path management ───────────────────────────────────────────────────────

    def load_path(
        self,
        points: list[tuple[float, float]],
        name: Optional[str] = None,
        spray_flags: Optional[list[bool]] = None,
        must_hit: Optional[list[bool]] = None,
        *,
        placement_mode: str = LOCAL_NED,
        origin_gps: tuple[float, float] | None = None,
        is_staged: bool = False,
    ) -> None:
        if placement_mode not in (LOCAL_NED, GPS_SURVEYED):
            raise ValueError(f"unsupported placement mode: {placement_mode!r}")
        if self._state == MissionState.RUNNING:
            self._log_entry(
                "warning",
                f"load_path called while RUNNING — overwriting loaded path. "
                f"Stop the mission first if this is unintentional.",
            )
        self._loaded_pts = points
        self._entry_marking_pts = None       # D1: wipe any stale entry stash
        self._entry_marking_flags = None
        self._entry_marking_must_hit = None
        # Provenance is advisory: a length mismatch degrades to "no provenance"
        # (geometry-only simplification), never to a wrong flag alignment.
        if must_hit is not None and len(must_hit) == len(points):
            self._loaded_must_hit = [bool(f) for f in must_hit]
        else:
            if must_hit:
                self._log_entry(
                    "warning",
                    f"must_hit length mismatch for {name or 'unknown'} — "
                    "publishing without vertex provenance",
                )
            self._loaded_must_hit = None
        if spray_flags is not None and len(spray_flags) == len(points):
            self._loaded_spray_flags = [bool(f) for f in spray_flags]
        elif spray_flags is not None:
            self._loaded_spray_flags = None
            self._log_entry(
                "warning",
                f"spray_flags length mismatch for {name or 'unknown'} — loading path with spray OFF",
            )
        else:
            self._loaded_spray_flags = None
        self._path_name = name or "unknown"
        self._placement_mode = placement_mode
        self._is_staged_mission = bool(is_staged)
        if origin_gps is not None:
            lat = float(origin_gps[0])
            lon = float(origin_gps[1])
            if not math.isfinite(lat) or not math.isfinite(lon):
                raise ValueError("origin_gps must contain finite latitude/longitude")
            self._origin_gps = (lat, lon)
        else:
            self._origin_gps = None
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
            f"placement={self._placement_mode})",
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

    async def start_async(self, auto_origin: bool = False) -> tuple[bool, str]:
        """Begin a mission, bracketed by the mission↔joystick arbiter.

        The arbiter claim happens before any FCU I/O: if the joystick owns
        manual control, the mission is refused here (typed 409-style reject)
        rather than racing the vehicle. On any exit — success, early guard
        return, PlacementError, or unexpected failure — the bracket relinquishes
        MISSION ownership (see control_arbiter.mission_start()); the running
        mission is thereafter guarded by self.state, not by a sticky owner.
        """
        try:
            async with self._arbiter.mission_start(self):
                return await self._start_locked(auto_origin)
        except ControlArbiterError as exc:
            self._log_entry("warning", f"start rejected: {exc.message}")
            return False, exc.message

    async def _start_locked(self, auto_origin: bool = False) -> tuple[bool, str]:
        async with self._lifecycle_lock():
            if self._node is None:
                return False, "ROS node not available"

            # Guard: re-starting while already running/entering re-arms and
            # re-switches OFFBOARD, which is wrong. Operator must stop first.
            if self._state in (MissionState.RUNNING, MissionState.ENTRY):
                msg = f"start: mission already {self._state.value} — call stop first"
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

            if not self._loaded_pts:
                self._state = MissionState.ERROR
                msg = "start: no path loaded"
                self._log_entry("error", msg)
                return False, msg

            if self._placement_mode == GPS_SURVEYED and auto_origin:
                msg = "start: GPS_SURVEYED missions are incompatible with auto_origin"
                self._log_entry("error", msg)
                raise PlacementError(msg)

            fcu = self._node.get_state()
            if not fcu.get("connected", False):
                self._state = MissionState.ERROR
                msg = "start: FCU not connected"
                self._log_entry("error", msg)
                return False, msg

            pts_to_publish = list(self._loaded_pts)
            spray_flags_to_publish = self._loaded_spray_flags
            must_hit_to_publish = self._loaded_must_hit

            # Surveyed placement first so RTK/pose/skew failures keep a typed
            # PlacementError (HTTP 422) instead of being masked by RPP STALE.
            if self._placement_mode == GPS_SURVEYED:
                try:
                    pts_to_publish, translation = resolve_surveyed_points(
                        self._loaded_pts,
                        self._origin_gps,
                        fcu,
                    )
                except (PlacementError, ImportError) as exc:
                    self._state = MissionState.ERROR
                    msg = f"start: surveyed placement failed: {exc}"
                    self._log_entry("error", msg)
                    raise PlacementError(msg) from exc
                self._log_entry(
                    "info",
                    "survey placement offset: "
                    f"{translation[0]:+.3f}N {translation[1]:+.3f}E",
                )

            # Pre-stream / pre-conditions check.
            # B2: any unhealthy code blocks OFFBOARD start.
            #   STALE     → no fresh pose → setpoint chain not ready
            #   RTK_WAIT  → GPS fix < RTK_FIXED → would refuse to drive anyway
            #   JUMP_SKIP → mid-EKF-reset → wait for it to settle
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
                    (n + off_n, e + off_e) for n, e in self._loaded_pts
                ]
                self._log_entry(
                    "info", f"auto_origin offset: +{off_n:.3f}N +{off_e:.3f}E"
                )

            # ── D1: runtime-entry two-phase decision ──────────────────────────
            # For a GPS-surveyed mission the placed path's first point is almost
            # never where the rover is sitting. Rather than let RPP acquire the
            # shape from an arbitrary offset (uncontrolled, not spray-safe — E2E
            # audit gap 9), drive a spray-OFF entry leg [live_pose → first point],
            # stop there (D3 completion latch), then publish the marking path
            # (advance_entry_to_marking, from the telemetry loop on RPP DONE).
            # Skipped when the rover is already on the first point (degenerate,
            # e.g. LOCAL_NED auto-origin) — then it's a normal single publish.
            entry_two_phase = False
            publish_pts = pts_to_publish
            publish_flags = spray_flags_to_publish
            publish_must_hit = must_hit_to_publish
            self._entry_marking_pts = None
            self._entry_marking_flags = None
            self._entry_marking_must_hit = None
            if self._placement_mode == GPS_SURVEYED and len(pts_to_publish) >= 2:
                live_n, live_e = fcu.get("pos_n"), fcu.get("pos_e")
                tgt_n, tgt_e = pts_to_publish[0]
                if (
                    live_n is not None and live_e is not None
                    and all(math.isfinite(v) for v in (live_n, live_e, tgt_n, tgt_e))
                    and math.hypot(tgt_n - live_n, tgt_e - live_e) > ENTRY_SKIP_DIST_M
                ):
                    entry_two_phase = True
                    self._entry_marking_pts = list(pts_to_publish)
                    self._entry_marking_flags = (
                        list(spray_flags_to_publish) if spray_flags_to_publish else None
                    )
                    self._entry_marking_must_hit = (
                        list(must_hit_to_publish) if must_hit_to_publish else None
                    )
                    publish_pts = [
                        (float(live_n), float(live_e)),
                        (float(tgt_n), float(tgt_e)),
                    ]
                    publish_flags = [False, False]   # entry leg is spray-OFF
                    # Both entry-leg points are live geometry, not survey intent.
                    publish_must_hit = [False, False]
                    self._log_entry(
                        "info",
                        f"entry leg: ({live_n:+.3f}N,{live_e:+.3f}E) → first point "
                        f"({tgt_n:+.3f}N,{tgt_e:+.3f}E), "
                        f"{math.hypot(tgt_n - live_n, tgt_e - live_e):.2f} m, spray OFF",
                    )

            armed_here = False
            try:
                # Publish the (entry or marking) path before the OFFBOARD request
                # so the 50 Hz setpoint stream carries setpoints when PX4
                # evaluates entry.
                self._node.publish_path(
                    publish_pts,
                    spray_flags=publish_flags,
                    must_hit_flags=publish_must_hit,
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

                if entry_two_phase:
                    self._state = MissionState.ENTRY
                    self._log_entry(
                        "info", f"entry: driving to first point ({self._path_name})"
                    )
                    return True, "entry"
                self._state = MissionState.RUNNING
                self._log_entry("info", f"mission running: {self._path_name}")
                return True, "running"
            except Exception as exc:
                self._state = MissionState.ERROR
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
            self._log_entry(
                "info" if ok else "error",
                f"disarm {'ok' if ok else f'failed: {why}'}",
            )
            return ok

    # Called from telemetry loop — no async lock to avoid blocking the loop.
    # Returns True iff it actually transitioned RUNNING→COMPLETED (so the caller
    # runs the completion sequence exactly once, on the edge).
    def mark_completed(self) -> bool:
        if self._state == MissionState.RUNNING:
            self._state = MissionState.COMPLETED
            self._log_entry("info", f"mission completed: {self._path_name}")
            return True
        return False

    # Called from the telemetry loop immediately after mark_completed() reports
    # the RUNNING→COMPLETED edge. Ends the mission spray-OFF and DISARMED without
    # an operator E-stop (field bug B4, 2026-07-25). The RPP has already settled
    # DONE (the mission-complete gate upstream), so the rover is stationary
    # before this runs. Self-gates on config.DISARM_ON_COMPLETE — when OFF the
    # rover is left armed (old behaviour). No lifecycle lock (mirrors
    # mark_completed): it only sends a spray-OFF command and a single disarm and
    # must not block the telemetry loop. Never raises.
    async def disarm_on_complete_async(self) -> dict[str, Any]:
        result = {"attempted": False, "spray_off_sent": False, "disarmed": False}
        if not config.DISARM_ON_COMPLETE:
            return result
        result["attempted"] = True
        if self._node is None:
            self._log_entry("warning", "complete: ROS node unavailable — cannot disarm")
            return result
        # 1. Command spray OFF — the same primitive /api/spray/off and the
        #    emergency disable path use. The disarm below also forces the spray
        #    node's actuator OFF via its disarm fail-safe, so this is belt-and-
        #    suspenders that closes any lingering manual hold immediately.
        try:
            self._node.publish_spray_manual(False)
            result["spray_off_sent"] = True
        except Exception as exc:
            self._log_entry("warning", f"complete spray-off raised: {exc}")
            log.exception("completion spray-off raised")
        # 2. Disarm. arm_async(False) (not disarm_async) so the mission stays in
        #    COMPLETED rather than being reset to IDLE. The spray node's disarm
        #    fail-safe drives the AUX output OFF as the hard guarantee.
        try:
            ok, why = await self._node.arm_async(False)
            result["disarmed"] = bool(ok)
            self._log_entry(
                "info" if ok else "error",
                f"completion disarm {'ok' if ok else f'failed: {why}'}",
            )
        except Exception as exc:
            self._log_entry("error", f"completion disarm raised: {exc}")
            log.exception("completion disarm raised")
        return result

    # Called from telemetry loop when state==ENTRY and RPP has settled DONE at
    # the entry point (D1 phase 2). Publishes the stashed marking path and
    # transitions ENTRY→RUNNING. No async lock (mirrors mark_completed) — it
    # only publishes a path + resets the monitor. Returns True if it advanced.
    def advance_entry_to_marking(self) -> bool:
        if self._state != MissionState.ENTRY:
            return False
        pts = self._entry_marking_pts
        if not pts:
            # No stash (should not happen) — fail safe to RUNNING so the
            # watchdog/auto-complete take over rather than sticking in ENTRY.
            self._state = MissionState.RUNNING
            self._log_entry("warning", "entry complete but no marking path stashed")
            return False
        flags = self._entry_marking_flags
        must = self._entry_marking_must_hit
        self._entry_marking_pts = None
        self._entry_marking_flags = None
        self._entry_marking_must_hit = None
        if self._node is not None:
            self._node.publish_path(pts, spray_flags=flags, must_hit_flags=must)
            # Clear the entry-leg DONE so RUNNING does not instantly auto-complete
            # on the stale settle before RPP re-latches on the marking path.
            try:
                self._node.get_rpp_monitor().reset()
            except Exception:
                pass
        self._state = MissionState.RUNNING
        self._log_entry(
            "info",
            f"entry complete — marking path published ({len(pts)} pts): {self._path_name}",
        )
        return True

    # ── Internal ──────────────────────────────────────────────────────────────

    def _log_entry(self, level: str, message: str) -> None:
        ts = datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"
        self._log.append({"timestamp": ts, "level": level, "message": message})
        getattr(log, level if level in ("info", "warning", "error") else "info")(message)
