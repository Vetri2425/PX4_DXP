from __future__ import annotations

import os
import sys
import asyncio

sys.path.insert(0, os.path.dirname(__file__))

import pytest
from fastapi import HTTPException, Response

import main
from auth import require_token
from ntrip_profile_store import NtripProfileStore
from routes.rtk import (
    NtripProfileCreateRequest,
    NtripProfileResponse,
    NtripProfileUpdateRequest,
    NtripProfilesResponse,
    create_ntrip_profile,
    _expected_revision,
    list_ntrip_profiles,
    router,
    set_default_ntrip_profile,
    update_ntrip_profile,
)
from rtk_manager import AsyncRTKManager


def _payload(**overrides):
    payload = {
        "name": "Tablet Profile",
        "host": "caster.example.com",
        "port": 2101,
        "mountpoint": "ROVER",
        "username": "tablet-user",
        "password": "tablet-secret",
    }
    payload.update(overrides)
    return payload


def _configure(monkeypatch, tmp_path):
    store = NtripProfileStore(
        tmp_path / "ntrip_profiles.json", tmp_path / "missing.env"
    )
    store.initialize()
    manager = AsyncRTKManager()
    monkeypatch.setattr(main, "ntrip_profile_store", store)
    monkeypatch.setattr(main, "rtk_manager", manager)
    return store, manager


def _assert_no_password(value):
    serialized = str(value).lower()
    assert "tablet-secret" not in serialized
    assert "replacement-secret" not in serialized
    if isinstance(value, dict):
        assert "password" not in value
        for child in value.values():
            _assert_no_password(child)
    elif isinstance(value, list):
        for child in value:
            _assert_no_password(child)


def test_profile_router_requires_x_rover_token(monkeypatch, tmp_path):
    _configure(monkeypatch, tmp_path)
    profile_routes = [
        route
        for route in router.routes
        if getattr(route, "path", "").startswith("/rtk/profiles")
    ]
    assert profile_routes
    for route in profile_routes:
        assert any(dependency.call is require_token for dependency in route.dependant.dependencies)


def test_profile_response_schemas_cannot_serialize_passwords():
    assert "password" not in NtripProfileResponse.model_fields
    assert "password" not in NtripProfilesResponse.model_json_schema()["$defs"][
        "NtripProfileResponse"
    ]["properties"]


def test_profile_crud_responses_are_redacted_and_password_is_editable(
    monkeypatch, tmp_path
):
    store, _ = _configure(monkeypatch, tmp_path)

    async def scenario():
        create_response = Response()
        created = await create_ntrip_profile(
            NtripProfileCreateRequest(**_payload()), create_response, '"0"'
        )
        _assert_no_password(created)
        assert created["password_configured"] is True
        profile_id = created["id"]

        renamed = await update_ntrip_profile(
            profile_id,
            NtripProfileUpdateRequest(name="Renamed"),
            Response(),
            '"1"',
        )
        _assert_no_password(renamed)

        default_result = await set_default_ntrip_profile(
            profile_id, Response(), '"2"'
        )
        assert default_result["takes_effect"] == "next_server_start"
        assert store.default_config()[2].password == "tablet-secret"

        replaced = await update_ntrip_profile(
            profile_id,
            NtripProfileUpdateRequest(password="replacement-secret"),
            Response(),
            '"3"',
        )
        _assert_no_password(replaced)
        assert store.default_config()[2].password == "replacement-secret"

        response = Response()
        listed = await list_ntrip_profiles(response)
        assert response.headers["etag"] == '"4"'
        _assert_no_password(listed)

    asyncio.run(scenario())


def test_mutations_require_current_if_match(monkeypatch, tmp_path):
    _configure(monkeypatch, tmp_path)

    async def scenario():
        with pytest.raises(HTTPException) as missing:
            await create_ntrip_profile(
                NtripProfileCreateRequest(**_payload()), Response(), None
            )
        assert missing.value.status_code == 428
        with pytest.raises(HTTPException) as stale:
            await create_ntrip_profile(
                NtripProfileCreateRequest(**_payload()), Response(), '"9"'
            )
        assert stale.value.status_code == 409

    asyncio.run(scenario())


@pytest.mark.parametrize(
    "value",
    [
        "0",
        'W/"0"',
        ' "0"',
        '"0" ',
        '"00"',
        '"9223372036854775808"',
        '"' + ("9" * 10_000) + '"',
        '"1", "2"',
        "",
    ],
)
def test_if_match_rejects_every_noncanonical_or_unbounded_etag(value):
    with pytest.raises(HTTPException) as exc:
        _expected_revision(value)
    assert exc.value.status_code == 400


def test_if_match_accepts_exact_strong_quoted_integer_etag():
    assert _expected_revision('"0"') == 0
    assert _expected_revision('"9223372036854775807"') == 9_223_372_036_854_775_807


def test_set_default_does_not_start_or_restart_rtk(monkeypatch, tmp_path):
    _, manager = _configure(monkeypatch, tmp_path)
    calls = []

    async def forbidden_start(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("default selection must not start NTRIP")

    monkeypatch.setattr(manager, "start_ntrip_profile", forbidden_start)

    async def scenario():
        created = await create_ntrip_profile(
            NtripProfileCreateRequest(**_payload()), Response(), '"0"'
        )
        response = await set_default_ntrip_profile(
            created["id"], Response(), '"1"'
        )
        assert response["takes_effect"] == "next_server_start"

    asyncio.run(scenario())
    assert calls == []
