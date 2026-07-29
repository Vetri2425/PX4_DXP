import json

import pytest
from fastapi import HTTPException

import auth


def setup_function():
    auth.reset_for_tests()
    # Tests must exercise real password/session/machine paths even if the
    # process inherited ROVER_DISABLE_AUTH / ROVER_AUTH_DISABLED from the shell.
    auth.AUTH_DISABLED = False
    auth.AUTH_BOOTSTRAP_ENABLED = True


def test_password_hash_is_pbkdf2_and_not_plaintext(tmp_path):
    path = tmp_path / "password.json"

    auth.write_password_hash("correct horse battery", path=path)
    auth.init_auth(str(path), str(tmp_path / "machine.json"))

    text = path.read_text(encoding="utf-8")
    assert "correct horse battery" not in text
    assert '"algorithm": "pbkdf2_hmac_sha256"' in text
    assert oct(path.stat().st_mode & 0o777) == "0o600"
    assert auth.verify_password("correct horse battery")
    assert not auth.verify_password("wrong")


def test_operator_login_returns_random_revocable_session(tmp_path):
    path = tmp_path / "password.json"
    auth.write_password_hash("secret123", path=path)
    auth.init_auth(str(path), str(tmp_path / "machine.json"))

    first = auth.login("secret123")
    second = auth.login("secret123")

    assert first["token"] != second["token"]
    assert first["must_change_password"] is False
    first_ctx = auth.validate_operator_token(first["token"])
    assert first_ctx is not None
    assert first_ctx.kind == "operator"

    assert auth.logout(first["token"]) is True
    assert auth.validate_operator_token(first["token"]) is None
    assert auth.validate_operator_token(second["token"]) is not None


def test_machine_token_is_limited_to_allowed_scopes(tmp_path):
    machine_path = tmp_path / "machine.json"
    token = auth.create_machine_token("bag-autorecord", path=machine_path)
    # Disable bootstrap so a missing password file stays missing (legacy path).
    auth.AUTH_BOOTSTRAP_ENABLED = False
    auth.init_auth(str(tmp_path / "missing-password.json"), str(machine_path))

    status_dep = auth.require_operator_or_machine("mission:status")
    loaded_dep = auth.require_operator_or_machine("mission:loaded-path")
    activity_dep = auth.require_operator_or_machine("activity:read")

    assert status_dep(x_rover_token=token).kind == "machine"
    assert loaded_dep(x_rover_token=token).kind == "machine"
    assert activity_dep(x_rover_token=token).kind == "machine"
    with pytest.raises(HTTPException):
        auth.require_operator_token(x_rover_token=token)


def test_password_change_rotates_requester_and_revokes_others(tmp_path):
    path = tmp_path / "password.json"
    auth.write_password_hash("oldpass123", path=path)
    auth.init_auth(str(path), str(tmp_path / "machine.json"))
    requester = auth.login("oldpass123")
    other = auth.login("oldpass123")
    requester_id = auth.validate_operator_token(requester["token"]).token_id

    auth.set_password_after_verified("oldpass123", "newpass123")
    rotated = auth.rotate_session_token(requester_id)
    revoked = auth.revoke_other_sessions(rotated["token_id"])

    assert len(revoked) == 1
    assert auth.validate_operator_token(requester["token"]) is None
    assert auth.validate_operator_token(other["token"]) is None
    assert auth.validate_operator_token(rotated["token"]) is not None
    assert auth.verify_password("newpass123")


def test_bootstrap_fresh_boot_writes_default_and_flags_login(tmp_path):
    path = tmp_path / "password.json"
    assert not path.exists()

    auth.init_auth(str(path), str(tmp_path / "machine.json"))

    assert path.exists()
    record = json.loads(path.read_text(encoding="utf-8"))
    assert record.get("is_default") is True
    # Password value must never appear on disk.
    assert auth.AUTH_BOOTSTRAP_PASSWORD not in path.read_text(encoding="utf-8")
    assert auth.is_default_password() is True
    assert auth.is_configured() is True

    session = auth.login(auth.AUTH_BOOTSTRAP_PASSWORD)
    assert session["must_change_password"] is True
    assert auth.validate_operator_token(session["token"]) is not None


def test_bootstrap_blocks_operator_routes_until_rotated(tmp_path):
    path = tmp_path / "password.json"
    auth.init_auth(str(path), str(tmp_path / "machine.json"))
    session = auth.login(auth.AUTH_BOOTSTRAP_PASSWORD)
    token = session["token"]

    with pytest.raises(HTTPException) as blocked:
        auth.require_operator_token(x_rover_token=token)
    assert blocked.value.status_code == 403
    assert blocked.value.detail["code"] == "password_change_required"

    # change-password / logout path still authenticates
    ctx = auth.require_operator_token_allow_default(x_rover_token=token)
    assert ctx.kind == "operator"
    assert auth.logout(token) is True
    assert auth.validate_operator_token(token) is None


def test_bootstrap_change_password_clears_default_and_revokes_others(tmp_path):
    path = tmp_path / "password.json"
    auth.init_auth(str(path), str(tmp_path / "machine.json"))
    requester = auth.login(auth.AUTH_BOOTSTRAP_PASSWORD)
    other = auth.login(auth.AUTH_BOOTSTRAP_PASSWORD)
    requester_id = auth.validate_operator_token(requester["token"]).token_id

    auth.set_password_after_verified(auth.AUTH_BOOTSTRAP_PASSWORD, "brand-new-pass")
    record = json.loads(path.read_text(encoding="utf-8"))
    assert "is_default" not in record
    assert auth.is_default_password() is False

    rotated = auth.rotate_session_token(requester_id)
    revoked = auth.revoke_other_sessions(rotated["token_id"])
    assert len(revoked) == 1
    assert auth.validate_operator_token(requester["token"]) is None
    assert auth.validate_operator_token(other["token"]) is None

    fresh = auth.login("brand-new-pass")
    assert fresh["must_change_password"] is False
    assert auth.require_operator_token(x_rover_token=fresh["token"]).kind == "operator"
    assert auth.require_operator_token(x_rover_token=rotated["token"]).kind == "operator"


def test_existing_password_file_without_is_default_is_not_treated_as_bootstrap(
    tmp_path,
):
    """Regression: already-configured rovers must not suddenly 403."""
    path = tmp_path / "password.json"
    auth.write_password_hash("site-password", path=path)
    record = json.loads(path.read_text(encoding="utf-8"))
    assert "is_default" not in record

    auth.init_auth(str(path), str(tmp_path / "machine.json"))
    assert auth.is_default_password() is False

    session = auth.login("site-password")
    assert session["must_change_password"] is False
    assert auth.require_operator_token(x_rover_token=session["token"]).kind == "operator"


def test_bootstrap_disabled_leaves_unconfigured(tmp_path):
    auth.AUTH_BOOTSTRAP_ENABLED = False
    path = tmp_path / "password.json"
    assert not path.exists()

    auth.init_auth(str(path), str(tmp_path / "machine.json"))

    assert not path.exists()
    assert auth.is_configured() is False
    with pytest.raises(HTTPException) as exc:
        auth.login("anything")
    assert exc.value.status_code == 503


def test_bootstrap_refuses_socket_bind_while_default(tmp_path):
    """Drive/joystick is Socket.IO — must not work on the bootstrap password."""
    path = tmp_path / "password.json"
    auth.init_auth(str(path), str(tmp_path / "machine.json"))
    session = auth.login(auth.AUTH_BOOTSTRAP_PASSWORD)

    assert auth.bind_socket_sid("sid-1", session["token"]) is None

    auth.set_password_after_verified(auth.AUTH_BOOTSTRAP_PASSWORD, "rotated-pass1")
    fresh = auth.login("rotated-pass1")
    assert auth.bind_socket_sid("sid-2", fresh["token"]) is not None
