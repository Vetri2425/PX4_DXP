"""Config load-time invariant test (plan phase J1 gate: "config refuses
illegal timeout ordering").

Uses a subprocess with a fresh interpreter so the env-var override actually
takes effect at *import* time — config.py runs its validation once, at
module load, and `import config` in-process would just reuse the already-
validated module from every other test file in this session.
"""
from __future__ import annotations

import os
import subprocess
import sys


def _import_config_with_env(env_overrides: dict[str, str]) -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env.update(env_overrides)
    return subprocess.run(
        [sys.executable, "-c", "import config"],
        cwd=os.path.dirname(__file__),
        env=env,
        capture_output=True,
        text=True,
        timeout=15,
    )


def test_default_config_imports_cleanly():
    result = _import_config_with_env({})
    assert result.returncode == 0, result.stderr


def test_server_stop_timeout_not_less_than_gateway_stale_rejected():
    result = _import_config_with_env(
        {
            "ROVER_JOYSTICK_SERVER_STOP_TIMEOUT_S": "0.50",
            "ROVER_JOYSTICK_GATEWAY_STALE_TIMEOUT_S": "0.40",
        }
    )
    assert result.returncode != 0
    assert "joystick timeout-ordering invariant violated" in result.stderr
    assert "JOYSTICK_SERVER_STOP_TIMEOUT_S" in result.stderr


def test_gateway_stale_not_less_than_px4_rc_loss_rejected():
    result = _import_config_with_env(
        {
            "ROVER_JOYSTICK_GATEWAY_STALE_TIMEOUT_S": "0.60",
            "ROVER_JOYSTICK_PX4_RC_LOSS_S": "0.50",
        }
    )
    assert result.returncode != 0
    assert "joystick timeout-ordering invariant violated" in result.stderr


def test_px4_rc_loss_not_less_than_lease_revoke_rejected():
    result = _import_config_with_env(
        {
            "ROVER_JOYSTICK_PX4_RC_LOSS_S": "3.0",
            "ROVER_JOYSTICK_LEASE_REVOKE_TIMEOUT_S": "2.0",
        }
    )
    assert result.returncode != 0
    assert "joystick timeout-ordering invariant violated" in result.stderr


def test_equal_timeouts_rejected_not_just_reversed():
    result = _import_config_with_env(
        {
            "ROVER_JOYSTICK_SERVER_STOP_TIMEOUT_S": "0.40",
            "ROVER_JOYSTICK_GATEWAY_STALE_TIMEOUT_S": "0.40",
        }
    )
    assert result.returncode != 0
    assert "joystick timeout-ordering invariant violated" in result.stderr


def test_joystick_manual_enabled_defaults_off():
    result = subprocess.run(
        [sys.executable, "-c", "import config; print(config.JOYSTICK_MANUAL_ENABLED)"],
        cwd=os.path.dirname(__file__),
        env={k: v for k, v in os.environ.items() if k != "ROVER_JOYSTICK_MANUAL_ENABLED"},
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "False"
