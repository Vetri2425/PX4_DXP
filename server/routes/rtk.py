"""RTK correction stream control routes."""

from __future__ import annotations

import datetime

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field

from auth import require_token
from rtk_manager import RTKProcessError

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
    mode: str
    pid: int | None
    running: bool
    healthy: bool
    source_state: str
    frames: int
    bytes: int
    last_frame_age_s: float | None
    last_error: str | None


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
    except RTKProcessError as exc:
        _record("error", f"NTRIP RTK start failed: {exc}")
        raise HTTPException(500, str(exc)) from exc

    _record("info", f"NTRIP RTK started pid={status.pid}")
    return _status_response(status)


@router.post("/lora/start", response_model=RTKStatusResponse)
async def start_lora(req: LoraStartRequest):
    try:
        status = await _manager().start_lora(
            baudrate=req.baudrate,
            serial_port=req.serial_port,
        )
    except RTKProcessError as exc:
        _record("error", f"LoRa RTK start failed: {exc}")
        raise HTTPException(500, str(exc)) from exc

    _record("info", f"LoRa RTK started pid={status.pid}")
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
