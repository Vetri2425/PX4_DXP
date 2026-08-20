from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from ntrip_profile_store import (
    NtripProfileConflictError,
    NtripProfileProtectedError,
    NtripProfileStore,
    NtripProfileStoreError,
    NtripProfileValidationError,
)


def _values(**overrides):
    values = {
        "name": "Primary Caster",
        "host": "caster.example.com",
        "port": 2101,
        "mountpoint": "/ROVER",
        "username": "field-user",
        "password": "first-secret",
    }
    values.update(overrides)
    return values


def _store(tmp_path: Path) -> NtripProfileStore:
    store = NtripProfileStore(
        tmp_path / "config" / "ntrip_profiles.json",
        tmp_path / "config" / "ntrip.env",
    )
    assert store.initialize() is False
    return store


def test_empty_registry_is_private_and_git_style_json(tmp_path):
    store = _store(tmp_path)
    path = store.registry_path
    assert path.exists()
    assert path.stat().st_mode & 0o777 == 0o600
    assert json.loads(path.read_text(encoding="utf-8")) == {
        "schema_version": 1,
        "registry_revision": 0,
        "default_profile_id": None,
        "migration_warning": None,
        "profiles": [],
    }


def test_secret_temp_file_is_private_from_first_open(monkeypatch, tmp_path):
    observed_modes = []
    real_fdopen = os.fdopen

    def checked_fdopen(fd, *args, **kwargs):
        observed_modes.append(os.fstat(fd).st_mode & 0o777)
        return real_fdopen(fd, *args, **kwargs)

    monkeypatch.setattr(os, "fdopen", checked_fdopen)
    _store(tmp_path)
    assert observed_modes == [0o600]


def test_registry_creation_fails_closed_if_private_mode_cannot_be_set(
    monkeypatch, tmp_path
):
    def denied_fchmod(fd, mode):
        raise PermissionError("denied")

    monkeypatch.setattr(os, "fchmod", denied_fchmod)
    store = NtripProfileStore(
        tmp_path / "config" / "ntrip_profiles.json",
        tmp_path / "config" / "ntrip.env",
    )
    with pytest.raises(PermissionError):
        store.initialize()
    assert not store.registry_path.exists()
    assert list(store.registry_path.parent.glob(".*.tmp")) == []


def test_legacy_env_migrates_once_without_deleting_source(tmp_path):
    config = tmp_path / "config"
    config.mkdir()
    legacy = config / "ntrip.env"
    legacy.write_text(
        "NTRIP_HOST=caster.example.com\n"
        "NTRIP_PORT=2101\n"
        "NTRIP_MOUNTPT=/ROVER\n"
        "NTRIP_USER=field-user\n"
        "NTRIP_PASS=migrated-secret\n",
        encoding="utf-8",
    )
    legacy.chmod(0o600)
    store = NtripProfileStore(config / "ntrip_profiles.json", legacy)
    assert store.initialize() is True
    assert legacy.exists()

    snapshot = store.snapshot()
    assert snapshot["registry_revision"] == 1
    assert len(snapshot["profiles"]) == 1
    assert snapshot["profiles"][0]["name"] == "Migrated Default"
    assert "password" not in snapshot["profiles"][0]
    profile_id, revision, loaded = store.default_config()
    assert profile_id == snapshot["default_profile_id"]
    assert revision == 1
    assert loaded.password == "migrated-secret"

    reloaded = NtripProfileStore(store.registry_path, legacy)
    assert reloaded.initialize() is False
    assert len(reloaded.snapshot()["profiles"]) == 1


def test_legacy_env_is_made_private_before_migration(tmp_path):
    config = tmp_path / "config"
    config.mkdir()
    legacy = config / "ntrip.env"
    legacy.write_text(
        "NTRIP_HOST=caster.example.com\n"
        "NTRIP_PORT=2101\n"
        "NTRIP_MOUNTPT=/ROVER\n"
        "NTRIP_USER=field-user\n"
        "NTRIP_PASS=migrated-secret\n",
        encoding="utf-8",
    )
    legacy.chmod(0o644)

    store = NtripProfileStore(config / "ntrip_profiles.json", legacy)
    assert store.initialize() is True
    assert legacy.stat().st_mode & 0o777 == 0o600


def test_malformed_legacy_env_creates_empty_repairable_registry_without_secret(
    tmp_path,
):
    config = tmp_path / "config"
    config.mkdir()
    legacy = config / "ntrip.env"
    legacy.write_text(
        "NTRIP_HOST=caster.example.com\n"
        "NTRIP_PORT=not-a-port\n"
        "NTRIP_MOUNTPT=ROVER\n"
        "NTRIP_USER=field-user\n"
        "NTRIP_PASS=do-not-expose\n",
        encoding="utf-8",
    )
    legacy.chmod(0o600)
    store = NtripProfileStore(config / "ntrip_profiles.json", legacy)

    assert store.initialize() is False
    snapshot = store.snapshot()
    assert snapshot["profiles"] == []
    assert snapshot["registry_revision"] == 0
    assert snapshot["migration_warning"] == store.migration_warning
    assert "do-not-expose" not in json.dumps(snapshot)
    assert "do-not-expose" not in store.registry_path.read_text(encoding="utf-8")

    created = store.create(_values(), expected_revision=0)
    assert created["name"] == "Primary Caster"
    assert store.snapshot()["migration_warning"] is None


def test_corrupt_existing_registry_still_fails_closed(tmp_path):
    config = tmp_path / "config"
    config.mkdir()
    registry = config / "ntrip_profiles.json"
    registry.write_text("{not-json", encoding="utf-8")
    store = NtripProfileStore(registry, config / "ntrip.env")

    with pytest.raises(NtripProfileStoreError, match="invalid JSON"):
        store.initialize()
    assert registry.read_text(encoding="utf-8") == "{not-json"


def test_post_replace_failure_keeps_memory_and_disk_on_same_revision(
    monkeypatch, tmp_path
):
    store = _store(tmp_path)

    def post_replace_failure(path):
        raise NtripProfileStoreError("injected post-replace failure")

    monkeypatch.setattr(store, "_ensure_private", post_replace_failure)
    with pytest.raises(NtripProfileStoreError, match="post-replace"):
        store.create(_values(), expected_revision=0)

    memory = store.snapshot()
    disk = json.loads(store.registry_path.read_text(encoding="utf-8"))
    assert memory["registry_revision"] == disk["registry_revision"] == 1
    assert [profile["id"] for profile in memory["profiles"]] == [
        profile["id"] for profile in disk["profiles"]
    ]


def test_create_update_password_is_write_only_and_omission_retains(tmp_path):
    store = _store(tmp_path)
    created = store.create(_values(), expected_revision=0)
    profile_id = created["id"]
    assert "password" not in created
    assert created["password_configured"] is True

    store.set_default(profile_id, expected_revision=1)
    store.update(profile_id, {"name": "Renamed"}, expected_revision=2)
    assert store.default_config()[2].password == "first-secret"

    updated = store.update(
        profile_id,
        {"password": "replacement-secret"},
        expected_revision=3,
    )
    assert "password" not in updated
    assert store.default_config()[2].password == "replacement-secret"
    serialized = json.dumps(store.snapshot())
    assert "first-secret" not in serialized
    assert "replacement-secret" not in serialized


def test_edit_active_profile_sets_pending_apply(tmp_path):
    store = _store(tmp_path)
    created = store.create(_values(), expected_revision=0)
    profile_id = created["id"]
    store.update(profile_id, {"host": "new.example.com"}, expected_revision=1)
    profile = store.snapshot(
        active_profile_id=profile_id,
        active_profile_revision=1,
    )["profiles"][0]
    assert profile["is_active"] is True
    assert profile["revision"] == 2
    assert profile["pending_apply"] is True


def test_stale_revision_does_not_overwrite(tmp_path):
    store = _store(tmp_path)
    store.create(_values(), expected_revision=0)
    with pytest.raises(NtripProfileConflictError):
        store.create(_values(name="Backup"), expected_revision=0)
    assert len(store.snapshot()["profiles"]) == 1


def test_validation_rejects_duplicate_name_bad_host_and_empty_password(tmp_path):
    store = _store(tmp_path)
    store.create(_values(), expected_revision=0)
    with pytest.raises(NtripProfileValidationError):
        store.create(_values(name=" primary caster "), expected_revision=1)
    with pytest.raises(NtripProfileValidationError):
        store.create(_values(name="Bad Host", host="http://caster"), expected_revision=1)
    with pytest.raises(NtripProfileValidationError):
        store.create(_values(name="Bad Secret", password=""), expected_revision=1)


def test_delete_rejects_default_and_active_profiles(tmp_path):
    store = _store(tmp_path)
    first = store.create(_values(), expected_revision=0)
    second = store.create(_values(name="Backup"), expected_revision=1)
    store.set_default(first["id"], expected_revision=2)

    with pytest.raises(NtripProfileProtectedError, match="default"):
        store.delete(first["id"], expected_revision=3, active_profile_id=None)
    with pytest.raises(NtripProfileProtectedError, match="active"):
        store.delete(second["id"], expected_revision=3, active_profile_id=second["id"])
    revision = store.delete(second["id"], expected_revision=3, active_profile_id=None)
    assert revision == 4
