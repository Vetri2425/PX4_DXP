#!/usr/bin/env python3
"""B0 anti-drift test: server-built config must parse in the NODE's parser.

Defect #2 (the 409 storm, memory: spray_param_contract_and_degraded_load) was
two independently-maintained schemas drifting apart. This test wires the two
sides together: whatever `server/spray_session_builder.py` emits must survive
`src/spray_session_config.parse_session_config` unchanged. Both are pure (no
rclpy), so this runs anywhere.

Run:  python3 -m pytest -q test_spray_session_builder.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import json

from spray_session_builder import (  # noqa: E402
    SPRAY_SCHEMA_VERSION,
    build_session_config,
    build_session_config_json,
    cleared_config_json,
)
from spray_session_config import (  # noqa: E402  (the NODE's parser, pure)
    SCHEMA_VERSION,
    parse_session_config,
)


def test_schema_versions_match():
    """The builder's version constant must equal the node parser's."""
    assert SPRAY_SCHEMA_VERSION == SCHEMA_VERSION


def test_continuous_round_trips_through_node_parser():
    cfg = parse_session_config(build_session_config("continuous"))
    assert cfg.mode == "continuous"
    assert cfg.dash is None and cfg.points_mode is None


def test_dash_round_trips_through_node_parser():
    cfg = parse_session_config(build_session_config(
        "dash", dash_on_distance_m=6.0, dash_off_distance_m=3.0, dash_start_state="on",
    ))
    assert cfg.mode == "dash"
    assert cfg.dash is not None
    assert cfg.dash.on_distance_m == 6.0
    assert cfg.dash.off_distance_m == 3.0
    assert cfg.dash.start_state == "on"


def test_dash_missing_distances_falls_back_to_continuous():
    # A half-configured dash must never ship a broken sub-config.
    cfg = parse_session_config(build_session_config("dash", dash_on_distance_m=6.0))
    assert cfg.mode == "continuous"


def test_point_round_trips_through_node_parser():
    cfg = parse_session_config(build_session_config(
        "point",
        point_coordinates=[(0.0, 0.0), (1.5, 2.0)],
        point_arrival_tolerance_m=0.05,
        point_arrival_settle_s=0.2,
        point_dwell_s=1.5,
    ))
    assert cfg.mode == "point"
    assert cfg.points_mode is not None
    assert len(cfg.points_mode.coordinates) == 2
    assert cfg.points_mode.dwell_s == 1.5
    assert cfg.points_mode.heading_tolerance_deg is None  # position-only


def test_point_no_coordinates_falls_back_to_continuous():
    cfg = parse_session_config(build_session_config("point", point_coordinates=[]))
    assert cfg.mode == "continuous"


def test_cleared_config_json_parses_continuous_empty():
    cfg = parse_session_config(json.loads(cleared_config_json()))
    assert cfg.mode == "continuous"
    assert cfg.points == () and cfg.flags == ()


def test_json_is_finite_and_valid():
    # allow_nan=False path: build_session_config_json must never emit nan/inf.
    s = build_session_config_json("dash", dash_on_distance_m=6.0, dash_off_distance_m=3.0)
    assert "NaN" not in s and "Infinity" not in s
    parse_session_config(json.loads(s))  # parses without raising


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"PASS: {name}")
    print("ALL BUILDER TESTS PASSED")
