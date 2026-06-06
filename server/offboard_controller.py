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

from config import RPP_STALE, RPP_UNHEALTHY_CODES, SETPOINT_STREAM_GRACE_S
from logging_setup import get_logger
from models import MissionState

log = get_logger("server.offboard")


class OffboardController:
    def __init__(self, ros_node, activity_log: deque) -> None:
        self._node       = ros_node
        self._log        = activity_log
        self._state      = MissionState.IDLE
        self._loaded_pts: list[tuple[float, float]] | None = None
        self._path_name: str | None = None
        self._lock       = asyncio.Lock()  # serialise lifecycle calls

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

    # ── Path management ───────────────────────────────────────────────────────

    def load_path(
        self, points: list[tuple[float, float]], name: Optional[str] = None
    ) -> None:
        if self._state == MissionState.RUNNING:
            self._log_entry(
                "warning",
                f"load_path called while RUNNING — overwriting loaded path. "
                f"Stop the mission first if this is unintentional.",
            )
        self._loaded_pts = points
        self._path_name  = name or "unknown"
        if self._state in (MissionState.COMPLETED, MissionState.ABORTED, MissionState.ERROR):
            self._state = MissionState.IDLE
        # Reset RPP done-settle timer so a leftover DONE from the previous
        # mission does not trigger instant auto-completion of the new one.
        if self._node is not None:
            try:
                self._node.get_rpp_monitor().reset()
            except Exception:
                pass
        self._log_entry("info", f"Path loaded: {self._path_name} ({len(points)} pts)")

    # ── Lifecycle (async) ─────────────────────────────────────────────────────

    async def start_async(self, auto_origin: bool = False) -> tuple[bool, str]:
        async with self._lock:
            if self._node is None:
                return False, "ROS node not available"

            # Guard: re-starting while already running re-arms and re-switches
            # OFFBOARD, which is wrong. Operator must stop first.
            if self._state == MissionState.RUNNING:
                msg = "start: mission already running — call stop first"
                self._log_entry("warning", msg)
                return False, msg

            if not self._loaded_pts:
                self._state = MissionState.ERROR
                msg = "start: no path loaded"
                self._log_entry("error", msg)
                return False, msg

            fcu = self._node.get_state()
            if not fcu.get("connected", False):
                self._state = MissionState.ERROR
                msg = "start: FCU not connected"
                self._log_entry("error", msg)
                return False, msg

            # Pre-stream / pre-conditions check.
            # B2: any unhealthy code blocks OFFBOARD start.
            #   STALE     → no fresh pose → setpoint chain not ready
            #   RTK_WAIT  → GPS fix < RTK_FIXED → would refuse to drive anyway
            #   JUMP_SKIP → mid-EKF-reset → wait for it to settle
            rpp_code = fcu.get("rpp_state", RPP_STALE)
            if rpp_code in RPP_UNHEALTHY_CODES:
                self._state = MissionState.ERROR
                if rpp_code == RPP_STALE:
                    msg = "start: RPP STALE — is twist_to_setpoint_node running?"
                elif rpp_code == 4:  # RPP_RTK_WAIT
                    msg = ("start: RPP RTK_WAIT — GPS fix < RTK_FIXED. "
                           "Wait for fix or set require_rtk_fix:=false on the controller.")
                elif rpp_code == 5:  # RPP_JUMP_SKIP
                    msg = ("start: RPP JUMP_SKIP — EKF position jump in progress; "
                           "retry in ~1 s once the estimator settles.")
                else:
                    msg = f"start: RPP unhealthy (code={rpp_code})"
                self._log_entry("error", msg)
                return False, msg

            # ── Arm ───────────────────────────────────────────────────────────
            self._state = MissionState.ARMING
            self._log_entry("info", "arming…")
            ok, why = await self._node.arm_async(True)
            if not ok:
                self._state = MissionState.ERROR
                self._log_entry("error", f"arming failed: {why}")
                return False, f"arm failed: {why}"

            # ── Switch to OFFBOARD ────────────────────────────────────────────
            self._state = MissionState.SWITCHING_OFFBOARD
            self._log_entry("info", "switching to OFFBOARD…")
            await asyncio.sleep(SETPOINT_STREAM_GRACE_S)
            ok, why = await self._node.set_mode_async("OFFBOARD")
            if not ok:
                self._state = MissionState.ERROR
                self._log_entry("error", f"OFFBOARD switch failed: {why}")
                # Best-effort disarm; ignore result
                await self._node.arm_async(False)
                return False, f"OFFBOARD failed: {why}"

            # ── Publish path ──────────────────────────────────────────────────
            pts_to_publish = self._loaded_pts
            if auto_origin:
                s = self._node.get_state()
                off_n = float(s.get("pos_n", 0.0))
                off_e = float(s.get("pos_e", 0.0))
                pts_to_publish = [(n + off_n, e + off_e) for n, e in self._loaded_pts]
                self._log_entry("info", f"auto_origin offset: +{off_n:.3f}N +{off_e:.3f}E")
            self._node.publish_path(pts_to_publish)
            self._state = MissionState.RUNNING
            self._log_entry("info", f"mission running: {self._path_name}")
            return True, "running"

    async def stop_async(self) -> None:
        """Soft stop: publish a single-point stop-path → RPP zeroes velocity.

        Empty Path is **ignored** by upstream RPP (early-return), so we
        publish a stop-path at the rover's current position. RPP treats it
        as DONE immediately and outputs zero velocity. Vehicle stays armed.
        """
        async with self._lock:
            if self._node is None:
                self._log_entry("warning", "stop: ROS node not available")
                return

            self._state = MissionState.STOPPING
            self._node.publish_stop_path()
            self._state = MissionState.IDLE
            self._log_entry("info", "mission stopped (stop-path published)")

    async def abort_async(self) -> None:
        """Hard abort: stop-path + MANUAL + disarm."""
        async with self._lock:
            if self._node is None:
                self._log_entry("warning", "abort: ROS node not available")
                return

            if self._state == MissionState.IDLE:
                self._log_entry("warning", "abort called from IDLE — no mission to abort")
                return
            self._node.publish_stop_path()
            await self._node.set_mode_async("MANUAL")
            await self._node.arm_async(False)
            self._state = MissionState.ABORTED
            self._log_entry("error", "mission ABORTED — MANUAL + disarm")

    async def disarm_async(self) -> bool:
        async with self._lock:
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
    def mark_completed(self) -> None:
        if self._state == MissionState.RUNNING:
            self._state = MissionState.COMPLETED
            self._log_entry("info", f"mission completed: {self._path_name}")

    # ── Internal ──────────────────────────────────────────────────────────────

    def _log_entry(self, level: str, message: str) -> None:
        ts = datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"
        self._log.append({"timestamp": ts, "level": level, "message": message})
        getattr(log, level if level in ("info", "warning", "error") else "info")(message)
