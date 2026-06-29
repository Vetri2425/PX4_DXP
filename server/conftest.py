"""Server test isolation for module-level runtime singletons."""

from __future__ import annotations

import pytest


_MAIN_SINGLETONS = (
    "ros_node",
    "offboard_ctrl",
    "path_mgr",
    "emergency_handler",
    "bridge_health",
    "rtk_manager",
    "mission_capture",
    "point_mission",
    "hold_owner",
    "operation_coordinator",
    "manual_gateway",
    "joystick_ctrl",
    "spray_startup_reconciliation",
)


def _reset_server_globals() -> None:
    try:
        import main

        for name in _MAIN_SINGLETONS:
            setattr(main, name, None)
        main.activity_log.clear()
    except Exception:
        pass

    try:
        from control_arbiter import reset_control_arbiter_for_tests

        reset_control_arbiter_for_tests()
    except Exception:
        pass

    try:
        from point_events import reset_point_event_journal_for_tests

        reset_point_event_journal_for_tests()
    except Exception:
        pass


@pytest.fixture(autouse=True)
def isolate_server_runtime_globals():
    _reset_server_globals()
    yield
    _reset_server_globals()
