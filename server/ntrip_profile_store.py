"""Durable, backend-owned NTRIP profile registry.

Passwords are intentionally present only in the private on-disk representation
and in :class:`NtripConfig` objects handed to the RTK manager.  Every public
snapshot produced by this module is redacted.
"""

from __future__ import annotations

import datetime as dt
import ipaddress
import json
import os
import re
import secrets
import threading
import uuid
from pathlib import Path
from typing import Any

from rtk_manager import NtripConfig, RTKProcessError, load_ntrip_config


class NtripProfileStoreError(RuntimeError):
    """Base error for profile registry operations."""


class NtripProfileValidationError(NtripProfileStoreError):
    """A profile or registry field is invalid."""


class NtripProfileNotFoundError(NtripProfileStoreError):
    """The requested profile does not exist."""


class NtripProfileConflictError(NtripProfileStoreError):
    """The supplied registry revision is stale."""


class NtripProfileProtectedError(NtripProfileStoreError):
    """A default or active profile cannot be deleted."""


_SCHEMA_VERSION = 1
_HOST_LABEL = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?$")
_LEGACY_MIGRATION_WARNING = (
    "Legacy NTRIP configuration is invalid; add a profile from the tablet."
)


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds").replace(
        "+00:00", "Z"
    )


def _has_control(value: str) -> bool:
    return any(ord(char) < 32 or ord(char) == 127 for char in value)


def _validate_host(value: Any) -> str:
    if not isinstance(value, str):
        raise NtripProfileValidationError("host must be a string")
    host = value.strip()
    if not host or len(host) > 253:
        raise NtripProfileValidationError("host must contain 1..253 characters")
    if "://" in host or any(char.isspace() for char in host) or _has_control(host):
        raise NtripProfileValidationError(
            "host must be a hostname or IP address without a URL scheme"
        )
    candidate = host[1:-1] if host.startswith("[") and host.endswith("]") else host
    try:
        ipaddress.ip_address(candidate)
        return candidate
    except ValueError:
        pass
    dns_name = host[:-1] if host.endswith(".") else host
    if not dns_name or any(not _HOST_LABEL.fullmatch(label) for label in dns_name.split(".")):
        raise NtripProfileValidationError("host must be a valid hostname or IP address")
    return dns_name.lower()


def _validate_text(
    field: str,
    value: Any,
    *,
    maximum: int,
    strip: bool = True,
    allow_whitespace: bool = True,
) -> str:
    if not isinstance(value, str):
        raise NtripProfileValidationError(f"{field} must be a string")
    normalized = value.strip() if strip else value
    if not normalized or len(normalized) > maximum:
        raise NtripProfileValidationError(
            f"{field} must contain 1..{maximum} characters"
        )
    if _has_control(normalized):
        raise NtripProfileValidationError(f"{field} must not contain control characters")
    if not allow_whitespace and any(char.isspace() for char in normalized):
        raise NtripProfileValidationError(f"{field} must not contain whitespace")
    return normalized


def _validate_profile_fields(values: dict[str, Any], *, partial: bool) -> dict[str, Any]:
    allowed = {"name", "host", "port", "mountpoint", "username", "password"}
    unknown = set(values) - allowed
    if unknown:
        raise NtripProfileValidationError(
            "unknown profile fields: " + ", ".join(sorted(unknown))
        )
    if not partial:
        missing = allowed - set(values)
        if missing:
            raise NtripProfileValidationError(
                "missing profile fields: " + ", ".join(sorted(missing))
            )
    if partial and not values:
        raise NtripProfileValidationError("at least one profile field is required")

    clean: dict[str, Any] = {}
    if "name" in values:
        clean["name"] = _validate_text("name", values["name"], maximum=64)
    if "host" in values:
        clean["host"] = _validate_host(values["host"])
    if "port" in values:
        port = values["port"]
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise NtripProfileValidationError("port must be an integer in 1..65535")
        clean["port"] = port
    if "mountpoint" in values:
        mountpoint = values["mountpoint"]
        if isinstance(mountpoint, str):
            mountpoint = mountpoint.strip().lstrip("/")
        clean["mountpoint"] = _validate_text(
            "mountpoint", mountpoint, maximum=128, allow_whitespace=False
        )
    if "username" in values:
        clean["username"] = _validate_text(
            "username", values["username"], maximum=128
        )
    if "password" in values:
        # Preserve leading/trailing spaces: they may be part of a caster secret.
        clean["password"] = _validate_text(
            "password", values["password"], maximum=512, strip=False
        )
    return clean


class NtripProfileStore:
    """Thread-safe profile registry with atomic, private JSON persistence."""

    def __init__(self, registry_path: str | Path, legacy_env_path: str | Path) -> None:
        self.registry_path = Path(registry_path)
        self.legacy_env_path = Path(legacy_env_path)
        self._lock = threading.RLock()
        self._registry: dict[str, Any] | None = None

    def initialize(self) -> bool:
        """Load the registry, migrating the legacy env file when needed.

        Returns ``True`` only when a legacy file was imported.  The legacy file
        is deliberately retained for rollback and is never modified here.
        """
        with self._lock:
            if self.registry_path.exists():
                self._registry = self._read_registry()
                return False

            registry = self._empty_registry()
            migrated = False
            if self.legacy_env_path.exists():
                # Migration deliberately retains the legacy file for rollback,
                # so protect that second copy of the secret before reading it.
                # Permission failures are not a malformed-config case: fail
                # closed and let the operator repair the deployment.
                self._ensure_private(self.legacy_env_path)
                try:
                    config = load_ntrip_config(self.legacy_env_path)
                except RTKProcessError:
                    # Do not brick profile management because an older env
                    # file is malformed.  Persist only a fixed, secret-free
                    # warning and let an authenticated tablet repair it.
                    registry["migration_warning"] = _LEGACY_MIGRATION_WARNING
                else:
                    profile_id = str(uuid.uuid4())
                    now = _utc_now()
                    registry.update(
                        {
                            "registry_revision": 1,
                            "default_profile_id": profile_id,
                            "profiles": [
                                {
                                    "id": profile_id,
                                    "revision": 1,
                                    "name": "Migrated Default",
                                    "host": config.host,
                                    "port": config.port,
                                    "mountpoint": config.mountpoint,
                                    "username": config.user,
                                    "password": config.password,
                                    "created_at": now,
                                    "updated_at": now,
                                }
                            ],
                        }
                    )
                    migrated = True
            self._write_registry(registry)
            return migrated

    @property
    def migration_warning(self) -> str | None:
        with self._lock:
            return self._require_registry().get("migration_warning")

    def snapshot(
        self,
        *,
        active_profile_id: str | None = None,
        active_profile_revision: int | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            registry = self._require_registry()
            default_id = registry["default_profile_id"]
            profiles = [
                self._public_profile(
                    profile,
                    default_id=default_id,
                    active_profile_id=active_profile_id,
                    active_profile_revision=active_profile_revision,
                )
                for profile in registry["profiles"]
            ]
            return {
                "schema_version": _SCHEMA_VERSION,
                "registry_revision": registry["registry_revision"],
                "default_profile_id": default_id,
                "active_profile_id": active_profile_id,
                "migration_warning": registry.get("migration_warning"),
                "profiles": profiles,
            }

    def default_config(self) -> tuple[str, int, NtripConfig]:
        with self._lock:
            registry = self._require_registry()
            profile_id = registry["default_profile_id"]
            if profile_id is None:
                raise NtripProfileStoreError("no default NTRIP profile configured")
            profile = self._find_profile(registry, profile_id)
            return profile_id, int(profile["revision"]), self._as_config(profile)

    def create(self, values: dict[str, Any], *, expected_revision: int) -> dict[str, Any]:
        clean = _validate_profile_fields(values, partial=False)
        with self._lock:
            registry = self._require_registry()
            self._check_revision(registry, expected_revision)
            self._check_unique_name(registry, clean["name"])
            now = _utc_now()
            profile = {
                "id": str(uuid.uuid4()),
                "revision": 1,
                **clean,
                "created_at": now,
                "updated_at": now,
            }
            updated = self._copy_registry(registry)
            updated["profiles"].append(profile)
            updated["migration_warning"] = None
            self._commit_next(updated)
            return self._public_profile(
                profile,
                default_id=updated["default_profile_id"],
                active_profile_id=None,
                active_profile_revision=None,
            )

    def update(
        self,
        profile_id: str,
        values: dict[str, Any],
        *,
        expected_revision: int,
    ) -> dict[str, Any]:
        clean = _validate_profile_fields(values, partial=True)
        with self._lock:
            registry = self._require_registry()
            self._check_revision(registry, expected_revision)
            current = self._find_profile(registry, profile_id)
            if "name" in clean:
                self._check_unique_name(registry, clean["name"], exclude_id=profile_id)
            updated = self._copy_registry(registry)
            profile = self._find_profile(updated, profile_id)
            profile.update(clean)
            profile["revision"] = int(current["revision"]) + 1
            profile["updated_at"] = _utc_now()
            self._commit_next(updated)
            return self._public_profile(
                profile,
                default_id=updated["default_profile_id"],
                active_profile_id=None,
                active_profile_revision=None,
            )

    def set_default(self, profile_id: str, *, expected_revision: int) -> dict[str, Any]:
        with self._lock:
            registry = self._require_registry()
            self._check_revision(registry, expected_revision)
            self._find_profile(registry, profile_id)
            if registry["default_profile_id"] == profile_id:
                return {
                    "registry_revision": registry["registry_revision"],
                    "default_profile_id": profile_id,
                }
            updated = self._copy_registry(registry)
            updated["default_profile_id"] = profile_id
            self._commit_next(updated)
            return {
                "registry_revision": updated["registry_revision"],
                "default_profile_id": profile_id,
            }

    def delete(
        self,
        profile_id: str,
        *,
        expected_revision: int,
        active_profile_id: str | None,
    ) -> int:
        with self._lock:
            registry = self._require_registry()
            self._check_revision(registry, expected_revision)
            self._find_profile(registry, profile_id)
            if registry["default_profile_id"] == profile_id:
                raise NtripProfileProtectedError(
                    "default profile cannot be deleted; select another default first"
                )
            if active_profile_id == profile_id:
                raise NtripProfileProtectedError(
                    "active profile cannot be deleted; stop or switch RTK first"
                )
            updated = self._copy_registry(registry)
            updated["profiles"] = [
                profile for profile in updated["profiles"] if profile["id"] != profile_id
            ]
            self._commit_next(updated)
            return int(updated["registry_revision"])

    @staticmethod
    def _empty_registry() -> dict[str, Any]:
        return {
            "schema_version": _SCHEMA_VERSION,
            "registry_revision": 0,
            "default_profile_id": None,
            "migration_warning": None,
            "profiles": [],
        }

    def _require_registry(self) -> dict[str, Any]:
        if self._registry is None:
            raise NtripProfileStoreError("NTRIP profile registry is not initialized")
        return self._registry

    def _read_registry(self) -> dict[str, Any]:
        try:
            with open(self.registry_path, encoding="utf-8") as stream:
                raw = json.load(stream)
        except json.JSONDecodeError as exc:
            raise NtripProfileStoreError("NTRIP profile registry contains invalid JSON") from exc
        except OSError as exc:
            raise NtripProfileStoreError("NTRIP profile registry cannot be read") from exc
        self._ensure_private(self.registry_path)
        return self._validate_registry(raw)

    def _validate_registry(self, raw: Any) -> dict[str, Any]:
        if not isinstance(raw, dict) or raw.get("schema_version") != _SCHEMA_VERSION:
            raise NtripProfileStoreError("unsupported NTRIP profile registry schema")
        revision = raw.get("registry_revision")
        profiles = raw.get("profiles")
        default_id = raw.get("default_profile_id")
        migration_warning = raw.get("migration_warning")
        if isinstance(revision, bool) or not isinstance(revision, int) or revision < 0:
            raise NtripProfileStoreError("invalid NTRIP profile registry revision")
        if not isinstance(profiles, list):
            raise NtripProfileStoreError("invalid NTRIP profile list")
        if migration_warning is not None:
            if (
                not isinstance(migration_warning, str)
                or not migration_warning
                or len(migration_warning) > 256
                or _has_control(migration_warning)
            ):
                raise NtripProfileStoreError("invalid NTRIP migration warning")
        normalized: list[dict[str, Any]] = []
        ids: set[str] = set()
        names: set[str] = set()
        for raw_profile in profiles:
            if not isinstance(raw_profile, dict):
                raise NtripProfileStoreError("invalid NTRIP profile record")
            try:
                profile_id = str(uuid.UUID(str(raw_profile["id"])))
                profile_revision = int(raw_profile["revision"])
                created_at = str(raw_profile["created_at"])
                updated_at = str(raw_profile["updated_at"])
                clean = _validate_profile_fields(
                    {
                        key: raw_profile[key]
                        for key in ("name", "host", "port", "mountpoint", "username", "password")
                    },
                    partial=False,
                )
            except (KeyError, TypeError, ValueError, NtripProfileValidationError) as exc:
                raise NtripProfileStoreError("invalid NTRIP profile record") from exc
            if profile_revision < 1 or profile_id in ids or clean["name"].casefold() in names:
                raise NtripProfileStoreError("duplicate or invalid NTRIP profile metadata")
            ids.add(profile_id)
            names.add(clean["name"].casefold())
            normalized.append(
                {
                    "id": profile_id,
                    "revision": profile_revision,
                    **clean,
                    "created_at": created_at,
                    "updated_at": updated_at,
                }
            )
        if default_id is not None:
            default_id = str(default_id)
            if default_id not in ids:
                raise NtripProfileStoreError("default NTRIP profile does not exist")
        return {
            "schema_version": _SCHEMA_VERSION,
            "registry_revision": revision,
            "default_profile_id": default_id,
            "migration_warning": migration_warning,
            "profiles": normalized,
        }

    def _write_registry(self, registry: dict[str, Any]) -> None:
        self.registry_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.registry_path.with_name(
            f".{self.registry_path.name}.{os.getpid()}.{secrets.token_hex(4)}.tmp"
        )
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        try:
            fd = os.open(tmp, flags, 0o600)
            try:
                os.fchmod(fd, 0o600)
                if os.fstat(fd).st_mode & 0o777 != 0o600:
                    raise NtripProfileStoreError(
                        "cannot guarantee private NTRIP registry permissions"
                    )
            except Exception:
                os.close(fd)
                raise
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(registry, stream, indent=2, sort_keys=True)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(tmp, self.registry_path)
            # os.replace is the commit point.  Publish the same state in
            # memory immediately so a later chmod/directory-fsync failure can
            # never leave the process serving the superseded revision.
            self._registry = registry
            self._ensure_private(self.registry_path)
            try:
                directory_fd = os.open(self.registry_path.parent, os.O_DIRECTORY)
            except OSError:
                return
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            try:
                tmp.unlink(missing_ok=True)
            except OSError:
                pass

    def _commit_next(self, updated: dict[str, Any]) -> None:
        updated["registry_revision"] = int(updated["registry_revision"]) + 1
        self._write_registry(updated)

    @staticmethod
    def _ensure_private(path: Path) -> None:
        try:
            os.chmod(path, 0o600)
            mode = path.stat().st_mode & 0o777
        except OSError as exc:
            raise NtripProfileStoreError(
                "cannot secure NTRIP profile registry permissions"
            ) from exc
        if mode != 0o600:
            raise NtripProfileStoreError(
                "cannot guarantee private NTRIP profile registry permissions"
            )

    @staticmethod
    def _copy_registry(registry: dict[str, Any]) -> dict[str, Any]:
        return {
            **registry,
            "profiles": [dict(profile) for profile in registry["profiles"]],
        }

    @staticmethod
    def _find_profile(registry: dict[str, Any], profile_id: str) -> dict[str, Any]:
        for profile in registry["profiles"]:
            if profile["id"] == profile_id:
                return profile
        raise NtripProfileNotFoundError("NTRIP profile not found")

    @staticmethod
    def _check_revision(registry: dict[str, Any], expected_revision: int) -> None:
        if expected_revision != registry["registry_revision"]:
            raise NtripProfileConflictError(
                f"profile registry changed; current revision is {registry['registry_revision']}"
            )

    @staticmethod
    def _check_unique_name(
        registry: dict[str, Any], name: str, *, exclude_id: str | None = None
    ) -> None:
        folded = name.casefold()
        if any(
            profile["id"] != exclude_id and profile["name"].casefold() == folded
            for profile in registry["profiles"]
        ):
            raise NtripProfileValidationError("profile name already exists")

    @staticmethod
    def _as_config(profile: dict[str, Any]) -> NtripConfig:
        return NtripConfig(
            host=profile["host"],
            port=profile["port"],
            mountpoint=profile["mountpoint"],
            user=profile["username"],
            password=profile["password"],
        )

    @staticmethod
    def _public_profile(
        profile: dict[str, Any],
        *,
        default_id: str | None,
        active_profile_id: str | None,
        active_profile_revision: int | None,
    ) -> dict[str, Any]:
        is_active = profile["id"] == active_profile_id
        return {
            "id": profile["id"],
            "revision": profile["revision"],
            "name": profile["name"],
            "host": profile["host"],
            "port": profile["port"],
            "mountpoint": profile["mountpoint"],
            "username": profile["username"],
            "password_configured": bool(profile["password"]),
            "is_default": profile["id"] == default_id,
            "is_active": is_active,
            "pending_apply": bool(
                is_active and active_profile_revision != profile["revision"]
            ),
            "created_at": profile["created_at"],
            "updated_at": profile["updated_at"],
        }
