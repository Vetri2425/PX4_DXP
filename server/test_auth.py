import pytest
from fastapi import HTTPException

import auth


def setup_function():
    auth.reset_for_tests()


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
    first_ctx = auth.validate_operator_token(first["token"])
    assert first_ctx is not None
    assert first_ctx.kind == "operator"

    assert auth.logout(first["token"]) is True
    assert auth.validate_operator_token(first["token"]) is None
    assert auth.validate_operator_token(second["token"]) is not None


def test_machine_token_is_limited_to_allowed_scopes(tmp_path):
    machine_path = tmp_path / "machine.json"
    token = auth.create_machine_token("bag-autorecord", path=machine_path)
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
