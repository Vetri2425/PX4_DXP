"""Task 02 server-side architecture guards that avoid FastAPI app imports."""

from __future__ import annotations

import shutil

import pytest

from path_manager import PathManager
from spray_mission_config import validate_staged_spray_config


def test_path_manager_rejects_compensate_spray_true(tmp_path):
    source = "Simple Demo/square_2x2.dxf"
    shutil.copy(source, tmp_path / "square_2x2.dxf")
    mgr = PathManager(str(tmp_path))

    with pytest.raises(ValueError, match="compensate_spray=True is not permitted"):
        mgr.plan_path("square_2x2.dxf", compensate_spray=True)


def test_staged_calibration_rejects_non_monotonic_speed_table():
    staged = {
        "mission_id": "stg_demo",
        "path_fingerprint": "abc",
        "configuration_revision": 1,
        "waypoints": [[0.0, 0.0], [1.0, 0.0]],
        "spray_flags": [False, True],
        "speed_pwm_table": [
            {"speed_mps": 0.3, "pwm": 1200.0},
            {"speed_mps": 0.2, "pwm": 1500.0},
        ],
    }

    with pytest.raises(ValueError, match="strictly increasing"):
        validate_staged_spray_config(staged)
