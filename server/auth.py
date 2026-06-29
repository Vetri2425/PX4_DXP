"""Local rover authentication.

Operator login uses a human password stored as PBKDF2-HMAC-SHA256. Successful
login returns a random session token; the server stores only SHA-256 token
identifiers in memory. Bag auto-record gets a separate read-only machine token
with a tiny allowlist.
"""
from __future__ import annotations

import base64
import datetime as dt
import hashlib
import hmac
import json
import os
import secrets
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

from fastapi import Header, HTTPException, status

from config import (
    AUTH_MACHINE_TOKENS_FILE,
    AUTH_PASSWORD_FILE,
    AUTH_PBKDF2_ITERATIONS,
    AUTH_SESSION_TTL_S,
    TOKEN_HEADER_NAME,
)
from logging_setup import get_logger

log = get_logger("server.auth")

AuthKind = Literal["operator", "machine"]
MachineScope = Literal["mission:status", "mission:loaded-path", "activity:read"]
MACHINE_READ_SCOPES: set[MachineScope] = {
    "mission:status",
    "mission:loaded-path",
    "activity:read",
}


@dataclass(frozen=True)
class AuthContext:
    kind: AuthKind
    token_id: str
    session_id: str | None = None
    name: str | None = None


@dataclass
class OperatorSession:
    token_id: str
    session_id: str
    created_at: float
    expires_at: float
    last_seen: float
    revoked: bool = False
    socket_sids: set[str] = field(default_factory=set)


_password_file = Path(AUTH_PASSWORD_FILE)
_machine_tokens_file = Path(AUTH_MACHINE_TOKENS_FILE)
_password_hash: dict | None = None
_machine_tokens: dict[str, dict] = {}
_sessions: dict[str, OperatorSession] = {}
_sid_to_token_id: dict[str, str] = {}
_sid_disconnect_cb = None


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds").replace(
        "+00:00", "Z"
    )


def _b64(data: bytes) -> str:
    return base64.urlsafe_b64encode(data).decode("ascii")


def _unb64(data: str) -> bytes:
    return base64.urlsafe_b64decode(data.encode("ascii"))


def _token_id(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def _ensure_private(path: Path) -> None:
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass


def _atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}.{secrets.token_hex(4)}.tmp")
    with open(tmp, "w", encoding="utf-8") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    _ensure_private(tmp)
    os.replace(tmp, path)
    _ensure_private(path)
    try:
        dir_fd = os.open(path.parent, os.O_DIRECTORY)
    except OSError:
        return
    try:
        os.fsync(dir_fd)
    finally:
        os.close(dir_fd)


def _read_json(path: Path) -> dict | None:
    try:
        with open(path, encoding="utf-8") as stream:
            data = json.load(stream)
    except FileNotFoundError:
        return None
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"invalid auth file: {path}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"auth file must contain a JSON object: {path}")
    _ensure_private(path)
    return data


def hash_password(password: str, *, iterations: int = AUTH_PBKDF2_ITERATIONS) -> dict:
    if not password:
        raise ValueError("password must not be empty")
    salt = secrets.token_bytes(16)
    digest = hashlib.pbkdf2_hmac(
        "sha256", password.encode("utf-8"), salt, int(iterations)
    )
    return {
        "version": 1,
        "algorithm": "pbkdf2_hmac_sha256",
        "iterations": int(iterations),
        "salt": _b64(salt),
        "hash": _b64(digest),
        "updated_at_utc": _utc_now(),
    }


def verify_password(password: str) -> bool:
    record = _password_hash
    if not record:
        return False
    if record.get("algorithm") != "pbkdf2_hmac_sha256":
        return False
    try:
        expected = _unb64(str(record["hash"]))
        salt = _unb64(str(record["salt"]))
        iterations = int(record["iterations"])
    except (KeyError, TypeError, ValueError):
        return False
    actual = hashlib.pbkdf2_hmac(
        "sha256", password.encode("utf-8"), salt, iterations
    )
    return hmac.compare_digest(actual, expected)


def write_password_hash(password: str, path: Path | None = None) -> None:
    global _password_hash
    target = path or _password_file
    record = hash_password(password)
    _atomic_write_json(target, record)
    if target == _password_file:
        _password_hash = record


def init_auth(
    password_path: str = AUTH_PASSWORD_FILE,
    machine_tokens_path: str = AUTH_MACHINE_TOKENS_FILE,
) -> None:
    """Load persisted auth material.

    Missing password files are allowed so the local CLI can perform first setup
    without the API auto-generating a secret.
    """
    global _password_file, _machine_tokens_file, _password_hash, _machine_tokens
    _password_file = Path(password_path)
    _machine_tokens_file = Path(machine_tokens_path)
    _password_hash = _read_json(_password_file)
    raw_machine = _read_json(_machine_tokens_file) or {"version": 1, "tokens": []}
    tokens = raw_machine.get("tokens", [])
    if not isinstance(tokens, list):
        raise RuntimeError("machine token file tokens must be a list")
    _machine_tokens = {
        str(item.get("token_id")): item
        for item in tokens
        if isinstance(item, dict) and item.get("token_id")
    }
    if _password_hash is None:
        log.warning("auth: operator password not configured; run rover-auth setup")
    log.info(
        "auth: loaded operator_configured=%s machine_tokens=%d",
        _password_hash is not None,
        len(_machine_tokens),
    )


def is_configured() -> bool:
    return _password_hash is not None


def login(password: str) -> dict:
    if not is_configured():
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Operator password is not configured",
        )
    if not verify_password(password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid password",
        )
    token = secrets.token_urlsafe(32)
    tid = _token_id(token)
    now = time.time()
    session = OperatorSession(
        token_id=tid,
        session_id=secrets.token_hex(12),
        created_at=now,
        expires_at=now + AUTH_SESSION_TTL_S,
        last_seen=now,
    )
    _sessions[tid] = session
    return {
        "token": token,
        "session_id": session.session_id,
        "expires_at": dt.datetime.fromtimestamp(
            session.expires_at, dt.timezone.utc
        ).isoformat(timespec="seconds").replace("+00:00", "Z"),
        "ttl_s": int(AUTH_SESSION_TTL_S),
    }


def _purge_expired(now: float | None = None) -> None:
    current = time.time() if now is None else now
    expired = [
        tid for tid, session in _sessions.items()
        if session.revoked or session.expires_at <= current
    ]
    for tid in expired:
        session = _sessions.pop(tid, None)
        if session:
            for sid in list(session.socket_sids):
                _sid_to_token_id.pop(sid, None)


def validate_operator_token(token: str | None) -> AuthContext | None:
    if not token:
        return None
    _purge_expired()
    tid = _token_id(token)
    session = _sessions.get(tid)
    if not session or session.revoked or session.expires_at <= time.time():
        return None
    session.last_seen = time.time()
    return AuthContext(kind="operator", token_id=tid, session_id=session.session_id)


def _validate_machine_token(token: str | None, scope: MachineScope) -> AuthContext | None:
    if not token:
        return None
    tid = _token_id(token)
    record = _machine_tokens.get(tid)
    if not record or record.get("revoked"):
        return None
    scopes = set(record.get("scopes") or [])
    if scope not in scopes:
        return None
    return AuthContext(kind="machine", token_id=tid, name=record.get("name"))


def require_operator_token(
    x_rover_token: str | None = Header(default=None, alias=TOKEN_HEADER_NAME),
) -> AuthContext:
    context = validate_operator_token(x_rover_token)
    if context is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or missing rover session",
        )
    return context


def require_token(
    x_rover_token: str | None = Header(default=None, alias=TOKEN_HEADER_NAME),
) -> AuthContext:
    """Backward-compatible dependency name for operator-protected endpoints."""
    return require_operator_token(x_rover_token)


def require_operator_or_machine(scope: MachineScope):
    def dependency(
        x_rover_token: str | None = Header(default=None, alias=TOKEN_HEADER_NAME),
    ) -> AuthContext:
        operator = validate_operator_token(x_rover_token)
        if operator is not None:
            return operator
        machine = _validate_machine_token(x_rover_token, scope)
        if machine is not None:
            return machine
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or missing rover credential",
        )

    return dependency


def logout(token: str | None) -> bool:
    if not token:
        return False
    return revoke_token_id(_token_id(token))


def revoke_token_id(token_id: str) -> bool:
    session = _sessions.pop(token_id, None)
    if not session:
        return False
    for sid in list(session.socket_sids):
        _sid_to_token_id.pop(sid, None)
    return True


def revoke_other_sessions(keep_token_id: str) -> list[str]:
    revoked: list[str] = []
    for tid in list(_sessions):
        if tid == keep_token_id:
            continue
        revoke_token_id(tid)
        revoked.append(tid)
    return revoked


def rotate_session_token(old_token_id: str) -> dict:
    old = _sessions.pop(old_token_id, None)
    if old is None:
        raise HTTPException(status_code=401, detail="Session expired")
    token = secrets.token_urlsafe(32)
    tid = _token_id(token)
    now = time.time()
    session = OperatorSession(
        token_id=tid,
        session_id=old.session_id,
        created_at=now,
        expires_at=now + AUTH_SESSION_TTL_S,
        last_seen=now,
        socket_sids=set(old.socket_sids),
    )
    _sessions[tid] = session
    for sid in session.socket_sids:
        _sid_to_token_id[sid] = tid
    return {
        "token": token,
        "token_id": tid,
        "session_id": session.session_id,
        "expires_at": dt.datetime.fromtimestamp(
            session.expires_at, dt.timezone.utc
        ).isoformat(timespec="seconds").replace("+00:00", "Z"),
        "ttl_s": int(AUTH_SESSION_TTL_S),
    }


def bind_socket_sid(sid: str, token: str | None) -> AuthContext | None:
    context = validate_operator_token(token)
    if context is None:
        return None
    session = _sessions.get(context.token_id)
    if session is None:
        return None
    session.socket_sids.add(sid)
    _sid_to_token_id[sid] = context.token_id
    return context


def unbind_socket_sid(sid: str) -> None:
    tid = _sid_to_token_id.pop(sid, None)
    if tid:
        session = _sessions.get(tid)
        if session:
            session.socket_sids.discard(sid)


def socket_authenticated(sid: str) -> bool:
    tid = _sid_to_token_id.get(sid)
    if not tid:
        return False
    _purge_expired()
    session = _sessions.get(tid)
    return bool(session and not session.revoked and session.expires_at > time.time())


def authenticated_sids() -> list[str]:
    _purge_expired()
    return list(_sid_to_token_id)


def socket_sids_for_token(token_id: str) -> list[str]:
    session = _sessions.get(token_id)
    return list(session.socket_sids) if session else []


def operator_token_ids() -> list[str]:
    _purge_expired()
    return list(_sessions)


def set_socket_disconnect_callback(callback) -> None:
    global _sid_disconnect_cb
    _sid_disconnect_cb = callback


async def disconnect_sids_for_revoked(token_ids: list[str], preserve: str | None = None) -> None:
    if _sid_disconnect_cb is None:
        return
    preserve_sids = set(socket_sids_for_token(preserve)) if preserve else set()
    sids: list[str] = []
    for tid in token_ids:
        session = _sessions.get(tid)
        if session:
            sids.extend(sid for sid in session.socket_sids if sid not in preserve_sids)
    for sid in sids:
        await _sid_disconnect_cb(sid)


def set_password_after_verified(current_password: str, new_password: str) -> None:
    if not verify_password(current_password):
        raise HTTPException(status_code=401, detail="Invalid current password")
    write_password_hash(new_password)


def create_machine_token(name: str, *, path: Path | None = None) -> str:
    target = path or _machine_tokens_file
    token = "rm_" + secrets.token_urlsafe(32)
    tid = _token_id(token)
    data = _read_json(target) or {"version": 1, "tokens": []}
    tokens = data.setdefault("tokens", [])
    if not isinstance(tokens, list):
        raise RuntimeError("machine token file tokens must be a list")
    tokens.append(
        {
            "token_id": tid,
            "name": name,
            "created_at_utc": _utc_now(),
            "scopes": sorted(MACHINE_READ_SCOPES),
            "revoked": False,
        }
    )
    _atomic_write_json(target, data)
    if target == _machine_tokens_file:
        _machine_tokens[tid] = tokens[-1]
    return token


def reset_for_tests() -> None:
    global _password_hash, _machine_tokens
    _password_hash = None
    _machine_tokens = {}
    _sessions.clear()
    _sid_to_token_id.clear()
