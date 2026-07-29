"""Operator authentication endpoints."""
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field

from auth import (
    AuthContext,
    login,
    operator_token_ids,
    require_operator_token_allow_default,
    revoke_other_sessions,
    revoke_token_id,
    rotate_session_token,
    set_password_after_verified,
    socket_sids_for_token,
)
from models import MissionState

router = APIRouter(prefix="/auth", tags=["auth"])


class LoginRequest(BaseModel):
    password: str = Field(min_length=1)


class LoginResponse(BaseModel):
    token: str
    session_id: str
    expires_at: str
    ttl_s: int
    must_change_password: bool


class ChangePasswordRequest(BaseModel):
    current_password: str = Field(min_length=1)
    new_password: str = Field(min_length=8)


class ChangePasswordResponse(BaseModel):
    token: str
    session_id: str
    expires_at: str
    ttl_s: int
    revoked_sessions: int


def _assert_password_change_safe() -> None:
    from main import offboard_ctrl

    if offboard_ctrl is not None and offboard_ctrl.state not in {
        MissionState.IDLE,
        MissionState.COMPLETED,
        MissionState.ABORTED,
        MissionState.ERROR,
    }:
        raise HTTPException(
            409,
            "Password change is blocked while a mission is active",
        )


@router.post("/login", response_model=LoginResponse)
async def auth_login(req: LoginRequest):
    return LoginResponse(**login(req.password))


@router.post("/logout")
async def auth_logout(
    context: AuthContext = Depends(require_operator_token_allow_default),
):
    # allow_default: logout must work while bootstrap password is in force.
    revoked = revoke_token_id(context.token_id)
    return {"logged_out": revoked}


@router.post("/change-password", response_model=ChangePasswordResponse)
async def change_password(
    req: ChangePasswordRequest,
    context: AuthContext = Depends(require_operator_token_allow_default),
):
    # allow_default: this is the only way off the bootstrap password.
    _assert_password_change_safe()
    set_password_after_verified(req.current_password, req.new_password)

    other_token_ids = [tid for tid in operator_token_ids() if tid != context.token_id]
    other_sids = []
    for tid in other_token_ids:
        other_sids.extend(socket_sids_for_token(tid))

    rotated = rotate_session_token(context.token_id)
    revoked = revoke_other_sessions(rotated["token_id"])

    if other_sids:
        from main import sio

        for sid in other_sids:
            await sio.emit(
                "auth_revoked",
                {"reason": "password_changed"},
                to=sid,
            )
            await sio.disconnect(sid)

    return ChangePasswordResponse(
        token=rotated["token"],
        session_id=rotated["session_id"],
        expires_at=rotated["expires_at"],
        ttl_s=rotated["ttl_s"],
        revoked_sessions=len(revoked),
    )
