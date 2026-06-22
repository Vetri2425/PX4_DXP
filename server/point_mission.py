"""Point-mission orchestrator state machine (async, non-blocking)."""

from __future__ import annotations

import asyncio
import sys
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable

_SRC = Path(__file__).resolve().parents[1] / "src"
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from point_ingest import SprayPoint, points_from_staged_dict  # noqa: E402
from spray_config import PointSprayParams, SprayConfiguration  # noqa: E402

from config import RPP_STALE
from logging_setup import get_logger
from mission_placement import GPS_SURVEYED, LOCAL_NED, PlacementError, resolve_surveyed_points

log = get_logger("server.point_mission")


class PointMissionState(str, Enum):
    IDLE = "idle"
    PREPARING_LEG = "preparing_leg"
    NAVIGATING = "navigating"
    SETTLING = "settling"
    DWELLING = "dwelling"
    ADVANCING = "advancing"
    COMPLETED = "completed"
    ABORTING = "aborting"
    FAILED = "failed"


@dataclass
class PointMissionStatus:
    state: PointMissionState = PointMissionState.IDLE
    mission_id: str = ""
    generation: int = 0
    current_point_index: int = 0
    total_points: int = 0
    active_dwell: bool = False
    dwell_remaining_s: float = 0.0
    last_transition: str = ""
    last_error: str = ""
    ready: bool = False
    source_frame: str = ""
    resolved_runtime_frame: str = ""

    def as_dict(self) -> dict[str, Any]:
        return {
            "point_mission_state": self.state.value,
            "point_mission_id": self.mission_id,
            "point_mission_generation": self.generation,
            "current_point_index": self.current_point_index,
            "total_points": self.total_points,
            "active_dwell": self.active_dwell,
            "dwell_remaining_s": self.dwell_remaining_s,
            "last_transition": self.last_transition,
            "last_error": self.last_error,
            "ready": self.ready,
            "source_frame": self.source_frame,
            "resolved_runtime_frame": self.resolved_runtime_frame,
        }


@dataclass(frozen=True)
class PointMissionRun:
    generation: int
    mission_id: str
    cancel_event: asyncio.Event


class PointMissionOrchestrator:
    def __init__(self) -> None:
        self._status = PointMissionStatus()
        self._points: list[SprayPoint] = []
        self._resolved_points: list[SprayPoint] = []
        self._config: SprayConfiguration | None = None
        self._task: asyncio.Task | None = None
        self._run_token: PointMissionRun | None = None
        self._generation = 0
        self._command_seq = 0
        self._source_frame = ""
        self._origin_gps: tuple[float, float] | None = None
        self._log_cb: Callable[[str, str], None] | None = None

    def set_logger(self, cb: Callable[[str, str], None]) -> None:
        self._log_cb = cb

    def _record(self, level: str, message: str) -> None:
        if self._log_cb is not None:
            self._log_cb(level, message)
        getattr(log, level if level in ("info", "warning", "error", "debug") else "info")(message)

    @property
    def status(self) -> PointMissionStatus:
        return self._status

    def is_active(self) -> bool:
        return self._task is not None and not self._task.done()

    def _is_current(self, run: PointMissionRun) -> bool:
        return self._run_token is run and self._generation == run.generation

    def _write(self, run: PointMissionRun, **changes: Any) -> None:
        if not self._is_current(run):
            return
        for key, value in changes.items():
            setattr(self._status, key, value)

    def load(self, *, mission_id: str, points: list[SprayPoint], config: SprayConfiguration) -> None:
        """Synchronous load for an idle orchestrator (used by unit callers)."""
        if self.is_active():
            raise RuntimeError("active point mission must be replaced asynchronously")
        self._install(mission_id, points, config, LOCAL_NED, None)

    async def replace_from_staged(
        self, staged: dict[str, Any], config: SprayConfiguration, ros_node
    ) -> None:
        await self.cancel_and_drain(ros_node, reason="reload")
        rows = staged.get("point_mission_points") or []
        points = points_from_staged_dict(rows)
        frame = str(staged.get("point_source_frame") or "").upper()
        anchor = staged.get("anchor")
        if not frame:
            raise PlacementError("Point mission is missing explicit point_source_frame metadata")
        if frame not in {LOCAL_NED, GPS_SURVEYED}:
            raise PlacementError(f"unsupported Point source_frame {frame!r}")
        origin = None
        if frame == GPS_SURVEYED:
            if not anchor or anchor.get("lat") is None or anchor.get("lon") is None:
                raise PlacementError("GPS_SURVEYED Point mission is missing its survey anchor")
            origin = (float(anchor["lat"]), float(anchor["lon"]))
        self._install(str(staged.get("mission_id", "") or ""), points, config, frame, origin)

    def _install(self, mission_id, points, config, source_frame, origin_gps) -> None:
        self._generation += 1
        self._points = list(points)
        self._resolved_points = []
        self._config = config
        self._source_frame = source_frame
        self._origin_gps = origin_gps
        self._run_token = None
        self._status = PointMissionStatus(
            state=PointMissionState.IDLE,
            mission_id=mission_id,
            generation=self._generation,
            total_points=len(points),
            ready=True,
            last_transition="loaded",
            source_frame=source_frame,
        )

    async def cancel_and_drain(self, ros_node, *, reason: str = "cancelled") -> None:
        run, task = self._run_token, self._task
        if run is not None:
            run.cancel_event.set()
            self._write(run, state=PointMissionState.ABORTING, last_transition=reason, ready=False)
        if task is not None and not task.done():
            task.cancel()
            try:
                await asyncio.wait_for(asyncio.shield(task), timeout=1.0)
            except asyncio.CancelledError:
                pass
            except asyncio.TimeoutError:
                self._record("error", "point mission cancellation did not drain within 1s")
                raise RuntimeError("point mission cancellation timeout")
            except Exception:
                # The run owns failure reporting; replacement only guarantees drain.
                pass
        if ros_node is not None:
            await ros_node.cancel_spray_dwell_async()
            ros_node.publish_spray_manual(False)
        if self._run_token is run:
            self._task = None
            self._run_token = None

    async def abort(self, ros_node) -> None:
        await self.cancel_and_drain(ros_node, reason="abort")

    async def start(self, ros_node, offboard_ctrl) -> tuple[bool, str]:
        if self._config is None or not self._points:
            return False, "point mission not loaded"
        await self.cancel_and_drain(ros_node, reason="start_replace")
        if not self._resolved_points:
            try:
                self.prepare(ros_node.get_state())
            except PlacementError as exc:
                self._status.state = PointMissionState.FAILED
                self._status.last_error = str(exc)
                self._status.ready = False
                return False, str(exc)
        run = PointMissionRun(self._generation, self._status.mission_id, asyncio.Event())
        self._run_token = run
        self._write(
            run,
            state=PointMissionState.PREPARING_LEG,
            current_point_index=0,
            last_error="",
            ready=True,
            resolved_runtime_frame=LOCAL_NED,
        )
        self._task = asyncio.create_task(self._run(run, ros_node, offboard_ctrl), name=f"point-{run.mission_id}-{run.generation}")
        return True, "point mission started"

    def prepare(self, state: dict[str, Any]) -> None:
        """Resolve design coordinates before the controller arms or enters OFFBOARD."""
        self._resolved_points = self._resolve_points(state)
        self._status.resolved_runtime_frame = LOCAL_NED

    def _resolve_points(self, state: dict[str, Any]) -> list[SprayPoint]:
        coords = [(p.north_m, p.east_m) for p in self._points]
        if self._source_frame == LOCAL_NED:
            resolved = coords
        elif self._source_frame == GPS_SURVEYED:
            resolved, _ = resolve_surveyed_points(coords, self._origin_gps, state)
        else:
            raise PlacementError("Point mission frame is missing or ambiguous")
        return [SprayPoint(n, e, p.dwell_s, p.source_index) for p, (n, e) in zip(self._points, resolved)]

    async def _run(self, run: PointMissionRun, ros_node, offboard_ctrl) -> None:
        try:
            params = self._config.point if self._config else PointSprayParams()
            for index, point in enumerate(self._resolved_points):
                self._check_cancel(run)
                self._write(run, current_point_index=index)
                await self._execute_point(run, ros_node, point, params, index)
                self._write(run, state=PointMissionState.ADVANCING, last_transition=f"advanced:{index}")
            await self._confirm_spray_off(run, ros_node)
            self._write(run, state=PointMissionState.COMPLETED, last_transition="completed", ready=False)
            if self._is_current(run) and offboard_ctrl is not None:
                from models import MissionState
                offboard_ctrl.state = MissionState.COMPLETED
                offboard_ctrl._running_mission_id = None
        except asyncio.CancelledError:
            self._write(run, state=PointMissionState.FAILED, last_error="cancelled", last_transition="cancelled", ready=False)
            raise
        except Exception as exc:
            self._write(run, state=PointMissionState.FAILED, last_error=str(exc), last_transition="failed", ready=False)
            self._record("error", f"point mission failed: {exc}")
        finally:
            if ros_node is not None:
                await ros_node.cancel_spray_dwell_async()
                ros_node.publish_spray_manual(False)

    def _check_cancel(self, run: PointMissionRun) -> None:
        if run.cancel_event.is_set() or not self._is_current(run):
            raise asyncio.CancelledError()

    async def _execute_point(self, run, ros_node, point, params, index) -> None:
        self._write(run, state=PointMissionState.PREPARING_LEG, last_transition=f"preparing_leg:{index}")
        await ros_node.cancel_spray_dwell_async()
        state = ros_node.get_state()
        if not state.get("pose_received", False):
            raise RuntimeError("no rover pose for point leg")
        ros_node.publish_path(
            [(float(state["pos_n"]), float(state["pos_e"])), (point.north_m, point.east_m)],
            spray_flags=[False, False], runtime_entry=True,
        )
        started = time.monotonic()
        self._write(run, state=PointMissionState.NAVIGATING, last_transition=f"navigating:{index}")
        await self._wait_arrival(run, ros_node, point, params, started)
        self._write(run, state=PointMissionState.SETTLING, last_transition=f"settling:{index}")
        await self._wait_settled(run, ros_node, point, params, started)
        self._write(run, state=PointMissionState.DWELLING, last_transition=f"dwelling:{index}")
        self._command_seq += 1
        command_id = self._command_seq
        dwell_s = float(point.dwell_s or params.default_dwell_s)
        ok, why = await ros_node.start_spray_dwell_async(
            mission_id=run.mission_id, point_index=index, duration_s=dwell_s,
            command_id=command_id, configuration_revision=self._config.revision,
        )
        if not ok:
            raise RuntimeError(why or "dwell rejected")
        await self._wait_dwell_complete(run, ros_node, dwell_s, command_id)
        await self._confirm_spray_off(run, ros_node)

    def _telemetry_stale(self, state: dict[str, Any]) -> bool:
        pose_age = float(state.get("pose_age_ms", float("inf")))
        velocity_age = state.get("velocity_age_ms")
        return (
            pose_age > 500.0
            or velocity_age is None
            or float(velocity_age) > 500.0
            or int(state.get("rpp_state", RPP_STALE)) == RPP_STALE
        )

    def _arrival_conditions_met(self, state, point, params) -> bool:
        if self._telemetry_stale(state):
            return False
        dist = ((float(state.get("pos_n", 0.0)) - point.north_m) ** 2 + (float(state.get("pos_e", 0.0)) - point.east_m) ** 2) ** 0.5
        return dist <= params.arrival_tolerance_m and float(state.get("speed_m_s", 0.0)) <= params.settle_speed_mps and abs(float(state.get("yaw_rate_rad_s", 0.0))) <= params.settle_yaw_rate_rad_s

    async def _wait_arrival(self, run, ros_node, point, params, started) -> None:
        while True:
            self._check_cancel(run)
            if time.monotonic() - started > params.leg_timeout_s:
                raise TimeoutError(f"leg timeout at point {self._status.current_point_index}")
            state = ros_node.get_state()
            if self._telemetry_stale(state):
                raise RuntimeError("stale telemetry during navigation")
            if self._arrival_conditions_met(state, point, params):
                return
            await asyncio.sleep(0.05)

    async def _wait_settled(self, run, ros_node, point, params, started) -> None:
        settled_since = None
        while True:
            self._check_cancel(run)
            if time.monotonic() - started > params.leg_timeout_s:
                raise TimeoutError(f"settle timeout at point {self._status.current_point_index}")
            state = ros_node.get_state()
            if self._telemetry_stale(state):
                raise RuntimeError("stale telemetry during settle")
            if self._arrival_conditions_met(state, point, params):
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since >= params.settle_time_s:
                    return
            else:
                settled_since = None
            await asyncio.sleep(0.05)

    async def _wait_dwell_complete(self, run, ros_node, dwell_s, command_id) -> None:
        deadline = time.monotonic() + dwell_s + 1.0
        observed_active = False
        while time.monotonic() < deadline:
            self._check_cancel(run)
            status = ros_node.get_spray_runtime_status()
            if status.get("status_stale", True):
                raise RuntimeError("spray runtime status is stale")
            if status.get("last_error") or not status.get("ready", False):
                raise RuntimeError(status.get("last_error") or "spray node is not ready")
            seen_id = status.get("dwell_command_id")
            if seen_id is not None and int(seen_id) != command_id:
                raise RuntimeError("spray dwell command mismatch")
            active = bool(status.get("active_dwell", False))
            self._write(run, active_dwell=active, dwell_remaining_s=float(status.get("dwell_remaining_s", 0.0)))
            if active:
                observed_active = True
            elif observed_active:
                if not status.get("commanded_on", False) and status.get("confirmed_off", False):
                    self._write(run, active_dwell=False, dwell_remaining_s=0.0)
                    return
            await asyncio.sleep(0.05)
        raise TimeoutError("dwell never became active" if not observed_active else "dwell completion timeout")

    async def _confirm_spray_off(self, run, ros_node) -> None:
        await ros_node.cancel_spray_dwell_async()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self._check_cancel(run)
            status = ros_node.get_spray_runtime_status()
            if not status.get("status_stale", True) and status.get("confirmed_off", False) and not status.get("commanded_on", True):
                return
            await asyncio.sleep(0.05)
        raise TimeoutError("spray OFF was not confirmed")
