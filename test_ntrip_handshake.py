"""NTRIP handshake classification tests."""

from __future__ import annotations

import pytest

from rtk_transport import classify_ntrip_handshake


def test_auth_failed_only_401_403():
    outcome, _ = classify_ntrip_handshake("HTTP/1.0 401 Unauthorized")
    assert outcome == "auth_failed"
    outcome, _ = classify_ntrip_handshake("HTTP/1.1 403 Forbidden")
    assert outcome == "auth_failed"


def test_mount_not_found_404():
    outcome, _ = classify_ntrip_handshake("HTTP/1.0 404 Not Found")
    assert outcome == "mount_not_found"


def test_caster_unreachable_5xx():
    outcome, _ = classify_ntrip_handshake("HTTP/1.0 503 Service Unavailable")
    assert outcome == "caster_unreachable"


def test_ok_http_and_icy():
    assert classify_ntrip_handshake("HTTP/1.0 200 OK")[0] == "ok"
    assert classify_ntrip_handshake("ICY 200 OK")[0] == "ok"


def test_protocol_error_other_4xx():
    outcome, _ = classify_ntrip_handshake("HTTP/1.0 400 Bad Request")
    assert outcome == "protocol_error"