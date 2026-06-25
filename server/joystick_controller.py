"""Virtual joystick V2 lease, validation, dead-man and watchdog logic."""
from __future__ import annotations

import asyncio
import math
import time
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Any

from config import (
    JOYSTICK_COMMAND_RATE_HZ,
    JOYSTICK_GATEWAY_STALE_TIMEOUT_S,
    JOYSTICK_LEASE_EXPIRY_S,
    JOYSTICK_LEASE_REVOKE_TIMEOUT_S,
    JOYSTICK_MANUAL_ENABLED,
    JOYSTICK_MAX_ABS_STEERING,
    JOYSTICK_MAX_ABS_THROTTLE,
    JOYSTICK_MODE_CONFIRM_TIMEOUT_S,
    JOYSTICK_NEUTRAL_PRESTREAM_S,
    JOYSTICK_SERVER_STOP_TIMEOUT_S,
)
from control_arbiter import (
    ControlArbiter,
    clear_reentry_context,
    get_control_arbiter,
    reset_reentry_context,
)
from logging_setup import get_logger
from manual_control_gateway import ManualControlGateway

log = get_logger("server.joystick")


class JoystickState(str, Enum):
    INACTIVE = "inactive"
    ACQUIRING = "acquiring"
    ACTIVE = "active"
    HELD = "held"
    RELEASING = "releasing"


class JoystickError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class JoystickCommand:
    session_id: str
    lease_id: str
    sequence: int
    client_monotonic_ms: int
    deadman: bool
    throttle: float
    steering: float


class JoystickController:
    def __init__(
        self,
        ros_node: Any,
        offboard_ctrl: Any,
        gateway: ManualControlGateway,
        *,
        arbiter: ControlArbiter | None = None,
        manual_enabled: bool = JOYSTICK_MANUAL_ENABLED,
        server_stop_timeout_s: float = JOYSTICK_SERVER_STOP_TIMEOUT_S,
        lease_revoke_timeout_s: float = JOYSTICK_LEASE_REVOKE_TIMEOUT_S,
        lease_expiry_s: float = JOYSTICK_LEASE_EXPIRY_S,
        mode_confirm_timeout_s: float = JOYSTICK_MODE_CONFIRM_TIMEOUT_S,
        neutral_prestream_s: float = JOYSTICK_NEUTRAL_PRESTREAM_S,
        max_abs_throttle: float = JOYSTICK_MAX_ABS_THROTTLE,
        max_abs_steering: float = JOYSTICK_MAX_ABS_STEERING,
        command_rate_hz: float = JOYSTICK_COMMAND_RATE_HZ,
        gateway_stale_timeout_s: float = JOYSTICK_GATEWAY_STALE_TIMEOUT_S,
    ) -> None:
        self._ros_node = ros_node
        self._offboard_ctrl = offboard_ctrl
        self._gateway = gateway
        self._arbiter = arbiter or get_control_arbiter()
        self._manual_enabled = bool(manual_enabled)
        self._server_stop_timeout_s = float(server_stop_timeout_s)
        self._lease_revoke_timeout_s = float(lease_revoke_timeout_s)
        self._lease_expiry_s = float(lease_expiry_s)
        self._mode_confirm_timeout_s = float(mode_confirm_timeout_s)
        self._neutral_prestream_s = float(neutral_prestream_s)
        self._max_abs_throttle = float(max_abs_throttle)
        self._max_abs_steering = float(max_abs_steering)
        self._min_command_interval_s = 1.0 / float(command_rate_hz)
        self._gateway_stale_timeout_s = float(gateway_stale_timeout_s)

        self._state = JoystickState.INACTIVE
        self._owner_sid: str | None = None
        self._session_id: str | None = None
        self._lease_id: str | None = None
        self._last_seq: int | None = None
        self._last_client_mono_ms: int | None = None
        self._last_valid_cmd_mono: float | None = None
        self._last_rate_mono: float | None = None
        self._last_deadman = False
        self._last_throttle = 0.0
        self._last_steering = 0.0
        self._stop_reason: str | None = None
        self._watchdog_task: asyncio.Task | None = None

    @property
    def is_active(self) -> bool:
        return self._state in {JoystickState.ACTIVE, JoystickState.HELD}

    @property
    def owner_sid(self) -> str | None:
        return self._owner_sid

    @property
    def lease_id(self) -> str | None:
        return self._lease_id

    async def acquire(self, sid: str, data: dict[str, Any]) -> dict[str, Any]:
        if not self._manual_enabled:
            raise JoystickError(
                "manual_control_disabled",
                "manual joystick is disabled by deployment configuration",
            )
        session_id = _required_str(data, "session_id")
        async with self._arbiter.joystick_acquire(self._offboard_ctrl):
            self._state = JoystickState.ACQUIRING
            self._owner_sid = sid
            self._session_id = session_id
            self._lease_id = None
            self._last_seq = None
            self._last_client_mono_ms = None
            self._last_valid_cmd_mono = None
            self._last_rate_mono = None
            self._last_deadman = False
            self._last_throttle = 0.0
            self._last_steering = 0.0
            self._stop_reason = None

            try:
                self._check_fcu_ready_for_acquire()
                self._check_transport_healthy()
                self._gateway.activate_neutral()
                await asyncio.to_thread(
                    self._gateway.wait_neutral_barrier,
                    self._neutral_prestream_s,
                )
                ok, why = await self._ros_node.set_mode_async("MANUAL")
                if not ok:
                    raise JoystickError("mode_unavailable", f"MANUAL request failed: {why}")
                if not await self._wait_for_mode("MANUAL", self._mode_confirm_timeout_s):
                    raise JoystickError("mode_unavailable", "MANUAL mode was not confirmed")
                if (
                    self._owner_sid != sid
                    or self._session_id != session_id
                    or self._state != JoystickState.ACQUIRING
                ):
                    raise JoystickError(
                        "acquire_cancelled",
                        "joystick acquire was cancelled before MANUAL confirmation",
                    )

                lease_id = uuid.uuid4().hex
                self._lease_id = lease_id
                now = time.monotonic()
                self._last_valid_cmd_mono = now
                self._last_rate_mono = None
                self._state = JoystickState.ACTIVE
                self._arbiter.mark_joystick_active(session_id, lease_id)
                self._start_watchdog()
                return {
                    "type": "joystick_acquired",
                    "lease_id": lease_id,
                    "state": self._state.value,
                    "command_rate_hz": round(1.0 / self._min_command_interval_s, 3),
                    "server_stop_timeout_ms": round(self._server_stop_timeout_s * 1000.0),
                    "gateway_stop_timeout_ms": round(self._gateway_stale_timeout_s * 1000.0),
                    "max_throttle": self._max_abs_throttle,
                    "max_steering": self._max_abs_steering,
                }
            except Exception:
                await asyncio.to_thread(self._gateway.deactivate_neutral)
                self._clear_local(reason="acquire_failed")
                raise

    def handle_command(self, sid: str, data: dict[str, Any]) -> dict[str, Any]:
        cmd = _parse_command(data)
        now = time.monotonic()

        self._validate_owner(sid, cmd)
        if self._state not in {JoystickState.ACTIVE, JoystickState.HELD}:
            raise JoystickError("lease_inactive", "joystick lease is not active")
        self._check_transport_healthy()
        self._check_manual_mode()
        self._validate_sequence(cmd)
        self._validate_rate(now)
        self._validate_values(cmd)

        throttle = _clamp(cmd.throttle, -self._max_abs_throttle, self._max_abs_throttle)
        steering = _clamp(cmd.steering, -self._max_abs_steering, self._max_abs_steering)
        if not cmd.deadman:
            throttle = 0.0
            steering = 0.0
            self._state = JoystickState.HELD
            self._arbiter.mark_joystick_held()
        else:
            self._state = JoystickState.ACTIVE
            self._arbiter.mark_joystick_active(cmd.session_id, cmd.lease_id)

        self._gateway.accept_command(throttle, steering)
        self._last_seq = cmd.sequence
        self._last_client_mono_ms = cmd.client_monotonic_ms
        self._last_valid_cmd_mono = now
        self._last_rate_mono = now
        self._last_deadman = bool(cmd.deadman)
        self._last_throttle = throttle
        self._last_steering = steering
        self._stop_reason = "deadman_released" if not cmd.deadman else None
        return {
            "accepted": True,
            "state": self._state.value,
            "throttle": throttle,
            "steering": steering,
        }

    async def release(
        self,
        sid: str | None = None,
        *,
        session_id: str | None = None,
        lease_id: str | None = None,
        reason: str = "explicit",
        force: bool = False,
    ) -> dict[str, Any]:
        if not force and sid is not None and self._owner_sid is not None and sid != self._owner_sid:
            raise JoystickError("not_owner", "only the joystick owner can release")
        if not force and session_id is not None and self._session_id != session_id:
            raise JoystickError("not_owner", "session does not own joystick lease")
        if not force and lease_id is not None and self._lease_id != lease_id:
            raise JoystickError("not_owner", "lease does not own joystick control")

        self._stop_reason = reason
        async with self._arbiter.hold():
            self._state = JoystickState.RELEASING
            self._arbiter.mark_releasing()
        self._stop_watchdog()
        await asyncio.to_thread(self._gateway.deactivate_neutral)
        async with self._arbiter.hold():
            self._clear_local(reason=reason)
        return {"type": "joystick_released", "state": self._state.value, "reason": reason}

    async def force_release(self, *, reason: str = "forced") -> dict[str, Any]:
        return await self.release(reason=reason, force=True)

    def emergency_neutralize(self, *, reason: str = "estop") -> None:
        self._gateway.send_neutral(refresh=True)
        self._state = JoystickState.INACTIVE
        self._owner_sid = None
        self._session_id = None
        self._lease_id = None
        self._last_seq = None
        self._last_client_mono_ms = None
        self._last_valid_cmd_mono = None
        self._last_rate_mono = None
        self._last_deadman = False
        self._last_throttle = 0.0
        self._last_steering = 0.0
        self._stop_reason = reason

    async def shutdown(self) -> None:
        self._stop_watchdog()
        await self.force_release(reason="shutdown")
        await asyncio.to_thread(self._gateway.shutdown)

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        age_ms = (
            (now - self._last_valid_cmd_mono) * 1000.0
            if self._last_valid_cmd_mono is not None
            else None
        )
        return {
            **self._arbiter.snapshot(),
            **self._gateway.snapshot(),
            "joystick_state": self._state.value,
            "joystick_active": self.is_active,
            "joystick_owner_present": self._session_id is not None,
            "joystick_has_lease": self._lease_id is not None,
            "joystick_last_valid_cmd_age_ms": age_ms,
            "joystick_deadman": self._last_deadman,
            "joystick_commanded_throttle": self._last_throttle,
            "joystick_commanded_steering": self._last_steering,
            "joystick_stop_reason": self._stop_reason,
        }

    def _check_fcu_ready_for_acquire(self) -> None:
        if self._ros_node is None:
            raise JoystickError("mode_unavailable", "ROS node not ready")
        state = self._ros_node.get_state()
        if not state.get("connected", False):
            raise JoystickError("fcu_disconnected", "FCU is not connected")
        if not state.get("armed", False):
            raise JoystickError("not_armed", "vehicle must be armed before joystick acquire")

    def _check_transport_healthy(self) -> None:
        if not self._gateway.is_healthy():
            raise JoystickError("transport_unavailable", self._gateway.health_reason())

    def _check_manual_mode(self) -> None:
        if self._ros_node is None:
            raise JoystickError("mode_unavailable", "ROS node not ready")
        mode = self._ros_node.get_state().get("mode")
        if str(mode).upper() != "MANUAL":
            raise JoystickError("mode_unavailable", "PX4 is not in confirmed MANUAL mode")

    async def _wait_for_mode(self, mode: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        expected = mode.upper()
        while time.monotonic() < deadline:
            state = self._ros_node.get_state()
            if str(state.get("mode", "")).upper() == expected:
                return True
            await asyncio.sleep(0.05)
        return False

    def _validate_owner(self, sid: str, cmd: JoystickCommand) -> None:
        if sid != self._owner_sid:
            raise JoystickError("not_owner", "socket does not own joystick lease")
        if cmd.session_id != self._session_id:
            raise JoystickError("not_owner", "session does not own joystick lease")
        if cmd.lease_id != self._lease_id:
            raise JoystickError("not_owner", "lease does not own joystick control")

    def _validate_sequence(self, cmd: JoystickCommand) -> None:
        if self._last_seq is not None and cmd.sequence <= self._last_seq:
            raise JoystickError("out_of_order", "sequence must strictly increase")
        if (
            self._last_client_mono_ms is not None
            and cmd.client_monotonic_ms < self._last_client_mono_ms
        ):
            raise JoystickError("replay", "client monotonic timestamp moved backward")

    def _validate_rate(self, now: float) -> None:
        if (
            self._last_rate_mono is not None
            and now - self._last_rate_mono < self._min_command_interval_s
        ):
            raise JoystickError("rate_exceeded", "joystick command rate exceeded")

    def _validate_values(self, cmd: JoystickCommand) -> None:
        if not math.isfinite(cmd.throttle) or not math.isfinite(cmd.steering):
            raise JoystickError("nan_value", "throttle and steering must be finite")
        if not -1.0 <= cmd.throttle <= 1.0 or not -1.0 <= cmd.steering <= 1.0:
            raise JoystickError("out_of_range", "throttle and steering must be in [-1, 1]")

    def _start_watchdog(self) -> None:
        self._stop_watchdog()
        token = clear_reentry_context()
        try:
            self._watchdog_task = asyncio.create_task(
                self._watchdog_loop(),
                name="joystick-watchdog",
            )
        finally:
            reset_reentry_context(token)

    def _stop_watchdog(self) -> None:
        task = self._watchdog_task
        self._watchdog_task = None
        if task is not None and task is not asyncio.current_task() and not task.done():
            task.cancel()

    async def _watchdog_loop(self) -> None:
        try:
            while self.is_active:
                await asyncio.sleep(0.05)
                if self._last_valid_cmd_mono is None:
                    continue
                age = time.monotonic() - self._last_valid_cmd_mono
                if age > self._server_stop_timeout_s:
                    self._gateway.send_neutral(refresh=True)
                    self._last_throttle = 0.0
                    self._last_steering = 0.0
                    self._stop_reason = "server_timeout_neutral"
                if age > self._lease_revoke_timeout_s or age > self._lease_expiry_s:
                    await self.release(reason="lease_timeout", force=True)
                    return
        except asyncio.CancelledError:
            return
        except Exception:
            log.exception("joystick watchdog crashed; forcing neutral")
            self._gateway.send_neutral(refresh=True)

    def _clear_local(self, *, reason: str) -> None:
        self._state = JoystickState.INACTIVE
        self._owner_sid = None
        self._session_id = None
        self._lease_id = None
        self._last_seq = None
        self._last_client_mono_ms = None
        self._last_valid_cmd_mono = None
        self._last_rate_mono = None
        self._last_deadman = False
        self._last_throttle = 0.0
        self._last_steering = 0.0
        self._stop_reason = reason
        self._arbiter.clear_joystick(reason=reason)


def _required_str(data: dict[str, Any], field: str) -> str:
    value = data.get(field)
    if not isinstance(value, str) or not value:
        raise JoystickError("malformed", f"{field} is required")
    return value


def _parse_command(data: dict[str, Any]) -> JoystickCommand:
    try:
        sequence = int(data["sequence"])
        client_mono = int(data["client_monotonic_ms"])
        throttle = float(data["throttle"])
        steering = float(data["steering"])
    except KeyError as exc:
        raise JoystickError("malformed", f"missing field {exc.args[0]}") from exc
    except (TypeError, ValueError) as exc:
        raise JoystickError("malformed", f"invalid command field: {exc}") from exc
    return JoystickCommand(
        session_id=_required_str(data, "session_id"),
        lease_id=_required_str(data, "lease_id"),
        sequence=sequence,
        client_monotonic_ms=client_mono,
        deadman=bool(data.get("deadman", False)),
        throttle=throttle,
        steering=steering,
    )


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))
