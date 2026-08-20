"""RTK correction stream control routes."""

from __future__ import annotations

import asyncio
import datetime
import re
from typing import Any

from fastapi import APIRouter, Depends, Header, HTTPException, Response, status as http_status
from pydantic import BaseModel, ConfigDict, Field

from auth import require_token
from ntrip_profile_store import (
    NtripProfileConflictError,
    NtripProfileNotFoundError,
    NtripProfileProtectedError,
    NtripProfileStoreError,
    NtripProfileValidationError,
)
from rtk_manager import RTKProcessError

router = APIRouter(prefix="/rtk", tags=["rtk"], dependencies=[Depends(require_token)])


class NtripStartRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", populate_by_name=True)

    host: str = Field(min_length=1)
    port: int = Field(default=2101, ge=1, le=65535)
    mountpoint: str = Field(min_length=1)
    user: str = Field(min_length=1)
    password: str = Field(alias="pass", min_length=1)


class LoraStartRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    baudrate: int = Field(default=115200, ge=1)
    serial_port: str = Field(min_length=1)


class RTKStatusResponse(BaseModel):
    mode: str
    desired_mode: str
    pid: int | None
    running: bool
    healthy: bool
    source_state: str
    frames: int
    bytes: int
    last_frame_age_s: float | None
    last_error: str | None
    supervisor_restarts: int
    active_profile_id: str | None = None
    active_profile_revision: int | None = None


class NtripProfileCreateRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    name: str
    host: str
    port: int = 2101
    mountpoint: str
    username: str
    password: str


class NtripProfileUpdateRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    name: str | None = None
    host: str | None = None
    port: int | None = None
    mountpoint: str | None = None
    username: str | None = None
    password: str | None = None


class NtripProfileResponse(BaseModel):
    id: str
    revision: int
    name: str
    host: str
    port: int
    mountpoint: str
    username: str
    password_configured: bool
    is_default: bool
    is_active: bool
    pending_apply: bool
    created_at: str
    updated_at: str


class NtripProfilesResponse(BaseModel):
    schema_version: int
    registry_revision: int
    default_profile_id: str | None
    active_profile_id: str | None
    migration_warning: str | None
    profiles: list[NtripProfileResponse]


class NtripDefaultResponse(BaseModel):
    registry_revision: int
    default_profile_id: str
    active_profile_id: str | None
    takes_effect: str = "next_server_start"


def _now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def _record(level: str, message: str) -> None:
    from main import activity_log

    activity_log.append({"timestamp": _now(), "level": level, "message": message})


def _status_response(status) -> RTKStatusResponse:
    return RTKStatusResponse(
        mode=status.mode,
        desired_mode=status.desired_mode,
        pid=status.pid,
        running=status.running,
        healthy=status.healthy,
        source_state=status.source_state,
        frames=status.frames,
        bytes=status.bytes,
        last_frame_age_s=status.last_frame_age_s,
        last_error=status.last_error,
        supervisor_restarts=status.supervisor_restarts,
        active_profile_id=status.active_profile_id,
        active_profile_revision=status.active_profile_revision,
    )


def _manager():
    from main import rtk_manager

    if rtk_manager is None:
        raise HTTPException(503, "RTK manager not ready")
    return rtk_manager


def _profile_store():
    from main import ntrip_profile_store

    if ntrip_profile_store is None:
        raise HTTPException(503, "NTRIP profile registry not ready")
    return ntrip_profile_store


_ETAG_REVISION = re.compile(r'^"([0-9]{1,19})"$')
_MAX_REGISTRY_REVISION = 9_223_372_036_854_775_807


def _expected_revision(if_match: str | None) -> int:
    if if_match is None:
        raise HTTPException(
            status_code=http_status.HTTP_428_PRECONDITION_REQUIRED,
            detail="If-Match profile registry revision is required",
        )
    # Accept only the exact strong ETag emitted by this API.  Bound the header
    # before parsing so hostile digit strings cannot reach int conversion.
    if not isinstance(if_match, str) or len(if_match) > 21:
        raise HTTPException(400, "If-Match must be a quoted registry revision")
    match = _ETAG_REVISION.fullmatch(if_match)
    if match is None:
        raise HTTPException(400, "If-Match must be a quoted registry revision")
    digits = match.group(1)
    try:
        revision = int(digits)
    except (TypeError, ValueError, OverflowError) as exc:
        raise HTTPException(400, "If-Match must be a quoted registry revision") from exc
    if revision > _MAX_REGISTRY_REVISION or str(revision) != digits:
        raise HTTPException(400, "If-Match must be a quoted registry revision")
    return revision


def _etag(response: Response, revision: int) -> None:
    response.headers["ETag"] = f'"{revision}"'


async def _active_selection() -> tuple[str | None, int | None]:
    from main import rtk_manager

    if rtk_manager is None:
        return None, None
    current = await rtk_manager.status()
    return current.active_profile_id, current.active_profile_revision


def _translate_store_error(exc: NtripProfileStoreError) -> HTTPException:
    if isinstance(exc, NtripProfileNotFoundError):
        return HTTPException(404, str(exc))
    if isinstance(exc, (NtripProfileConflictError, NtripProfileProtectedError)):
        return HTTPException(409, str(exc))
    if isinstance(exc, NtripProfileValidationError):
        return HTTPException(422, str(exc))
    return HTTPException(500, "NTRIP profile registry operation failed")


async def _snapshot() -> dict[str, Any]:
    active_id, active_revision = await _active_selection()
    try:
        return await asyncio.to_thread(
            _profile_store().snapshot,
            active_profile_id=active_id,
            active_profile_revision=active_revision,
        )
    except NtripProfileStoreError as exc:
        raise _translate_store_error(exc) from exc


def _profile_from_snapshot(snapshot: dict[str, Any], profile_id: str) -> dict[str, Any]:
    for profile in snapshot["profiles"]:
        if profile["id"] == profile_id:
            return profile
    raise HTTPException(500, "saved NTRIP profile missing from registry snapshot")


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


@router.get("/profiles", response_model=NtripProfilesResponse)
async def list_ntrip_profiles(response: Response):
    snapshot = await _snapshot()
    _etag(response, snapshot["registry_revision"])
    return snapshot


@router.post(
    "/profiles",
    response_model=NtripProfileResponse,
    status_code=http_status.HTTP_201_CREATED,
)
async def create_ntrip_profile(
    req: NtripProfileCreateRequest,
    response: Response,
    if_match: str | None = Header(default=None, alias="If-Match"),
):
    try:
        created = await asyncio.to_thread(
            _profile_store().create,
            req.model_dump(),
            expected_revision=_expected_revision(if_match),
        )
        snapshot = await _snapshot()
    except NtripProfileStoreError as exc:
        raise _translate_store_error(exc) from exc
    _etag(response, snapshot["registry_revision"])
    _record("info", f"NTRIP profile created id={created['id']}")
    return _profile_from_snapshot(snapshot, created["id"])


@router.patch("/profiles/{profile_id}", response_model=NtripProfileResponse)
async def update_ntrip_profile(
    profile_id: str,
    req: NtripProfileUpdateRequest,
    response: Response,
    if_match: str | None = Header(default=None, alias="If-Match"),
):
    changes = req.model_dump(exclude_unset=True)
    try:
        await asyncio.to_thread(
            _profile_store().update,
            profile_id,
            changes,
            expected_revision=_expected_revision(if_match),
        )
        snapshot = await _snapshot()
    except NtripProfileStoreError as exc:
        raise _translate_store_error(exc) from exc
    _etag(response, snapshot["registry_revision"])
    _record("info", f"NTRIP profile updated id={profile_id}")
    return _profile_from_snapshot(snapshot, profile_id)


@router.put("/profiles/{profile_id}/default", response_model=NtripDefaultResponse)
async def set_default_ntrip_profile(
    profile_id: str,
    response: Response,
    if_match: str | None = Header(default=None, alias="If-Match"),
):
    try:
        result = await asyncio.to_thread(
            _profile_store().set_default,
            profile_id,
            expected_revision=_expected_revision(if_match),
        )
        active_id, _ = await _active_selection()
    except NtripProfileStoreError as exc:
        raise _translate_store_error(exc) from exc
    _etag(response, result["registry_revision"])
    _record("info", f"NTRIP default profile changed id={profile_id}")
    return {
        **result,
        "active_profile_id": active_id,
        "takes_effect": "next_server_start",
    }


@router.delete(
    "/profiles/{profile_id}",
    status_code=http_status.HTTP_204_NO_CONTENT,
)
async def delete_ntrip_profile(
    profile_id: str,
    if_match: str | None = Header(default=None, alias="If-Match"),
):
    active_id, _ = await _active_selection()
    try:
        revision = await asyncio.to_thread(
            _profile_store().delete,
            profile_id,
            expected_revision=_expected_revision(if_match),
            active_profile_id=active_id,
        )
    except NtripProfileStoreError as exc:
        raise _translate_store_error(exc) from exc
    _record("info", f"NTRIP profile deleted id={profile_id}")
    return Response(
        status_code=http_status.HTTP_204_NO_CONTENT,
        headers={"ETag": f'"{revision}"'},
    )
