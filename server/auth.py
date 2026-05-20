"""Shared-secret token authentication for control endpoints.

Loads a 16-byte token from a file (default `~/.rover_token`). Generates one if
the file is missing, with mode 0600. All `/api/arm`, `/api/set_mode`,
`/api/mission/*`, `/api/path/*`, `/api/params/*` routes (and the matching
Socket.IO control events) require the `X-Rover-Token` header.

In dev / LAN-only mode, set environment variable `ROVER_DISABLE_AUTH=1` to
bypass entirely (the dependency becomes a no-op).
"""
from __future__ import annotations

import os
import secrets
from pathlib import Path

from fastapi import Header, HTTPException, status

from config import TOKEN_FILE_DEFAULT, TOKEN_HEADER_NAME
from logging_setup import get_logger

log = get_logger("server.auth")

_TOKEN: str | None = None
_DISABLED: bool = os.environ.get("ROVER_DISABLE_AUTH", "0") == "1"


def _load_or_create_token(path: str) -> str:
    p = Path(path)
    if p.exists():
        token = p.read_text(encoding="utf-8").strip()
        if token:
            return token
    # Generate fresh
    token = secrets.token_urlsafe(16)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(token + "\n", encoding="utf-8")
    try:
        os.chmod(p, 0o600)
    except OSError:
        pass  # Windows / non-POSIX filesystem
    log.warning(
        "auth: generated new rover token at %s — copy to client config", path
    )
    return token


def init_auth(token_path: str = TOKEN_FILE_DEFAULT) -> None:
    """Load the auth token at server startup."""
    global _TOKEN
    if _DISABLED:
        log.warning("auth: DISABLED via ROVER_DISABLE_AUTH=1")
        return
    _TOKEN = _load_or_create_token(token_path)


def require_token(
    x_rover_token: str | None = Header(default=None, alias=TOKEN_HEADER_NAME),
) -> None:
    """FastAPI dependency: rejects with 401 unless the token matches."""
    if _DISABLED:
        return
    if _TOKEN is None:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Auth not initialised",
        )
    if x_rover_token is None or not secrets.compare_digest(x_rover_token, _TOKEN):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or missing rover token",
        )


def check_socket_token(token: str | None) -> bool:
    """Validate a token sent over Socket.IO. Returns True on success."""
    if _DISABLED:
        return True
    if _TOKEN is None or token is None:
        return False
    return secrets.compare_digest(token, _TOKEN)
