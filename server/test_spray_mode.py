#!/usr/bin/env python3
"""Tests for per-path spray-mode sidecar store, route guards, and the
continuous-param propagation fix (continuous config set via the sidecar
endpoints must reach the staged artifact, not silently fall back to defaults).
"""

from __future__ import annotations

import asyncio
import os
import sys
import tempfile

import pytest
from fastapi import HTTPException

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import routes.spray_mode as spray_mode_route
from models import ContinuousModeRequest, MissionState
from spray_config import staged_spray_defaults
from spray_mission_config import spray_fields_from_staged
from spray_mode_store import (
    delete_spray_mode,
    load_spray_mode,
    save_spray_mode,
    sidecar_exists,
)

_CONTINUOUS_KEYS = (
    "solenoid_open_delay_s",
    "solenoid_close_delay_s",
    "on_overspray_margin_m",
    "off_overspray_margin_m",
    "min_spray_speed_mps",
    "max_xtrack_error_m",
    "nozzle_forward_offset_m",
    "nozzle_lateral_offset_m",
)


# ───────────────────────── Regression: the actual bug ────────────────────────

def test_continuous_keys_present_in_staged_defaults():
    """Bug: continuous params were absent from staged_spray_defaults, so they
    never reached the controller. They must now be present."""
    defaults = staged_spray_defaults()
    for key in _CONTINUOUS_KEYS:
        assert key in defaults, f"{key} missing from staged_spray_defaults()"


def test_continuous_sidecar_propagates_to_controller_fields():
    """Continuous params written to a sidecar must survive the staged→controller
    extraction instead of falling back to factory defaults."""
    with tempfile.TemporaryDirectory() as d:
        data = load_spray_mode(d, "demo.dxf")
        data["spray_mode"] = "continuous"
        data["solenoid_open_delay_s"] = 0.42
        data["nozzle_forward_offset_m"] = 0.13
        save_spray_mode(d, "demo.dxf", data)

        merged = load_spray_mode(d, "demo.dxf")
        # _stage_mission writes these into the staged dict; simulate that subset.
        staged = {k: merged[k] for k in ("spray_mode", *_CONTINUOUS_KEYS)}
        out = spray_fields_from_staged(staged)
        assert out["solenoid_open_delay_s"] == 0.42
        assert out["nozzle_forward_offset_m"] == 0.13


# ───────────────────────── Store roundtrip / safety ──────────────────────────

def test_store_roundtrip_and_delete():
    with tempfile.TemporaryDirectory() as d:
        assert sidecar_exists(d, "p.dxf") is False
        assert load_spray_mode(d, "p.dxf") == staged_spray_defaults()  # pure defaults
        data = load_spray_mode(d, "p.dxf")
        data["spray_mode"] = "dash"
        data["dash_on_distance_m"] = 0.5
        save_spray_mode(d, "p.dxf", data)
        assert sidecar_exists(d, "p.dxf") is True
        assert load_spray_mode(d, "p.dxf")["dash_on_distance_m"] == 0.5
        assert delete_spray_mode(d, "p.dxf") is True
        assert delete_spray_mode(d, "p.dxf") is False  # already gone
        assert sidecar_exists(d, "p.dxf") is False


def test_store_corrupt_file_falls_back_to_defaults():
    with tempfile.TemporaryDirectory() as d:
        # Write a corrupt sidecar directly.
        from spray_mode_store import _sidecar_path

        with open(_sidecar_path(d, "bad.dxf"), "w", encoding="utf-8") as f:
            f.write("{ not valid json")
        assert load_spray_mode(d, "bad.dxf") == staged_spray_defaults()


def test_store_validation_rejects_bad_config():
    with tempfile.TemporaryDirectory() as d:
        data = load_spray_mode(d, "p.dxf")
        data["point_default_dwell_s"] = -1.0  # invalid
        with pytest.raises(ValueError):
            save_spray_mode(d, "p.dxf", data)


# ───────────────────────── Route guards ──────────────────────────────────────

def test_safe_name_rejects_empty_and_strips_traversal():
    assert spray_mode_route._safe_name("../../etc/passwd") == "passwd"
    with pytest.raises(HTTPException):
        spray_mode_route._safe_name("")


def test_live_guard_blocks_reconfig_during_active_mission(monkeypatch):
    import main

    class Ctrl:
        state = MissionState.RUNNING

    monkeypatch.setattr(main, "offboard_ctrl", Ctrl(), raising=False)
    with pytest.raises(HTTPException) as ei:
        spray_mode_route._guard_live()
    assert ei.value.status_code == 409


def test_put_continuous_persists_and_round_trips(monkeypatch):
    import main

    class Ctrl:
        state = MissionState.IDLE

    with tempfile.TemporaryDirectory() as d:
        monkeypatch.setattr(spray_mode_route, "MISSION_DIR", d)
        monkeypatch.setattr(main, "offboard_ctrl", Ctrl(), raising=False)
        req = ContinuousModeRequest(solenoid_open_delay_s=0.33, nozzle_lateral_offset_m=0.07)
        resp = asyncio.run(spray_mode_route.set_continuous_mode("road.dxf", req))
        assert resp.spray_mode == "continuous"
        assert resp.has_sidecar is True
        assert resp.config["solenoid_open_delay_s"] == 0.33
        assert resp.config["nozzle_lateral_offset_m"] == 0.07


def main():
    import inspect

    for name, fn in sorted(globals().items()):
        if not name.startswith("test_"):
            continue
        try:
            params = inspect.signature(fn).parameters
        except (TypeError, ValueError):
            continue
        if params:  # skips monkeypatch-dependent tests when run standalone
            continue
        fn()
    print("PASS (standalone subset)")


if __name__ == "__main__":
    main()
