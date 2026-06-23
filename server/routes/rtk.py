"""RTK correction stream control routes."""

from __future__ import annotations

import datetime

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field

from auth import require_token
from rtk_manager import RTKConflictError, RTKProcessError, RTKValidationError

router = APIRouter(prefix="/rtk", tags=["rtk"], dependencies=[Depends(require_token)])


class NtripStartRequest(BaseModel):
    host: str = Field(min_length=1)
    port: int = Field(default=2101, ge=1, le=65535)
    mountpoint: str = Field(min_length=1)
    user: str = Field(min_length=1)
    password: str = Field(alias="pass", min_length=1)


class LoraStartRequest(BaseModel):
    baudrate: int = Field(default=115200, ge=1)
    serial_port: str = Field(min_length=1)


class RTKStatusResponse(BaseModel):
    # Backward-compatible fields
    mode: str
    pid: int | None
    running: bool
    healthy: bool
    source_state: str
    frames: int
    bytes: int
    last_frame_age_s: float | None
    last_error: str | None

    # Task_03 fields
    active_source: str | None = None
    desired_source: str | None = None
    lifecycle_state: str = "IDLE"
    serial_port: str | None = None
    baudrate: int | None = None
    session_started_at: str | None = None
    process_alive: bool = False
    serial_open: bool = False
    reconnecting: bool = False
    restart_count: int = 0
    valid_frames: int = 0
    invalid_frames: int = 0
    crc_error_count: int = 0
    dropped_frames: int = 0
    bytes_received: int = 0
    bytes_injected: int = 0
    last_valid_rtcm_age_s: float | None = None
    valid_frame_rate_hz: float | None = None
    bytes_per_sec: float | None = None
    injection_topic_ready: bool | None = None
    stream_healthy: bool | None = None
    transport_reason: str | None = None
    stop_reason: str | None = None
    gps_fix_type: int | None = None
    rtk_fixed: bool | None = None
    rtk_float: bool | None = None
    pose_age_s: float | None = None
    rpp_rtk_wait: bool | None = None


def _now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def _record(level: str, message: str) -> None:
    from main import activity_log

    activity_log.append({"timestamp": _now(), "level": level, "message": message})


def _status_response(status) -> RTKStatusResponse:
    return RTKStatusResponse(
        mode=status.mode,
        pid=status.pid,
        running=status.running,
        healthy=status.healthy,
        source_state=status.source_state,
        frames=status.frames,
        bytes=status.bytes,
        last_frame_age_s=status.last_frame_age_s,
        last_error=status.last_error,
        active_source=status.active_source,
        desired_source=status.desired_source,
        lifecycle_state=status.lifecycle_state,
        serial_port=status.serial_port,
        baudrate=status.baudrate,
        session_started_at=status.session_started_at,
        process_alive=status.process_alive,
        serial_open=status.serial_open,
        reconnecting=status.reconnecting,
        restart_count=status.restart_count,
        valid_frames=status.valid_frames,
        invalid_frames=status.invalid_frames,
        crc_error_count=status.crc_error_count,
        dropped_frames=status.dropped_frames,
        bytes_received=status.bytes_received,
        bytes_injected=status.bytes_injected,
        last_valid_rtcm_age_s=status.last_valid_rtcm_age_s,
        valid_frame_rate_hz=status.valid_frame_rate_hz,
        bytes_per_sec=status.bytes_per_sec,
        injection_topic_ready=status.injection_topic_ready,
        stream_healthy=status.stream_healthy,
        transport_reason=status.transport_reason,
        stop_reason=status.stop_reason,
        gps_fix_type=status.gps_fix_type,
        rtk_fixed=status.rtk_fixed,
        rtk_float=status.rtk_float,
        pose_age_s=status.pose_age_s,
        rpp_rtk_wait=status.rpp_rtk_wait,
    )


def _manager():
    from main import rtk_manager

    if rtk_manager is None:
        raise HTTPException(503, "RTK manager not ready")
    return rtk_manager


@router.post("/ntrip/start", response_model=RTKStatusResponse)
async def start_ntrip(req: NtripStartRequest):
    try:
        status = await _manager().start_ntrip(
            host=req.host,
            port=req.port,
            mountpoint=req.mountpoint,
            user=req.user,
            password=req.password,
        )
    except RTKConflictError as exc:
        _record("error", f"NTRIP RTK start conflict: {exc}")
        raise HTTPException(409, str(exc)) from exc
    except RTKProcessError as exc:
        _record("error", f"NTRIP RTK start failed: {exc}")
        raise HTTPException(503, str(exc)) from exc

    _record("info", f"NTRIP RTK started pid={status.pid}")
    return _status_response(status)


@router.post("/lora/start", response_model=RTKStatusResponse)
async def start_lora(req: LoraStartRequest):
    try:
        status = await _manager().start_lora(
            baudrate=req.baudrate,
            serial_port=req.serial_port,
        )
    except RTKValidationError as exc:
        _record("error", f"LoRa RTK start invalid: {exc}")
        raise HTTPException(422, str(exc)) from exc
    except RTKConflictError as exc:
        _record("error", f"LoRa RTK start conflict: {exc}")
        raise HTTPException(409, str(exc)) from exc
    except RTKProcessError as exc:
        _record("error", f"LoRa RTK start failed: {exc}")
        raise HTTPException(503, str(exc)) from exc

    _record("info", f"LoRa RTK started desired_source=lora pid={status.pid}")
    return _status_response(status)


@router.post("/lora/stop", response_model=RTKStatusResponse)
async def stop_lora():
    status = await _manager().stop_lora(reason="user_stop")
    _record("info", "LoRa RTK stopped by user")
    return _status_response(status)


@router.post("/stop", response_model=RTKStatusResponse)
async def stop_rtk():
    status = await _manager().stop_all()
    _record("info", "RTK stream stopped")
    return _status_response(status)


@router.get("/status", response_model=RTKStatusResponse)
async def rtk_status():
    status = await _manager().status()
    return _status_response(status)