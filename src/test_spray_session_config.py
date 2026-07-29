#!/usr/bin/env python3
"""Unit tests for spray_session_config.py — pure, no ROS/rclpy dependency.

Runnable both as:
    python3 -m pytest src/test_spray_session_config.py
    python3 src/test_spray_session_config.py
"""

import json
import math
import unittest

from spray_session_config import (
    SCHEMA_VERSION,
    ConfigSchemaError,
    DashConfig,
    PointsModeConfig,
    SpraySessionConfig,
    cleared_config,
    continuous_config_from_path,
    parse_session_config,
    to_dict,
)


def _continuous_data(points=None, flags=None):
    return {
        "schema_version": SCHEMA_VERSION,
        "mode": "continuous",
        "points": points if points is not None else [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        "flags": flags if flags is not None else [False, True, True],
    }


def _dash_data(**overrides):
    data = {
        "schema_version": SCHEMA_VERSION,
        "mode": "dash",
        "points": [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        "flags": [False, True, True],
        "dash": {
            "on_distance_m": 0.5,
            "off_distance_m": 0.3,
            "start_state": "off",
        },
    }
    data.update(overrides)
    return data


def _point_data(**overrides):
    data = {
        "schema_version": SCHEMA_VERSION,
        "mode": "point",
        "points": [],
        "flags": [],
        "points_mode": {
            "coordinates": [[0.0, 0.0], [1.0, 1.0]],
            "arrival_tolerance_m": 0.1,
            "heading_tolerance_deg": 5.0,
            "arrival_settle_s": 0.5,
            "dwell_s": 2.0,
        },
    }
    data.update(overrides)
    return data


class ParseValidConfigsTest(unittest.TestCase):
    def test_parse_valid_continuous(self):
        cfg = parse_session_config(_continuous_data())
        self.assertEqual(cfg.mode, "continuous")
        self.assertEqual(cfg.schema_version, SCHEMA_VERSION)
        self.assertEqual(cfg.points, ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)))
        self.assertEqual(cfg.flags, (False, True, True))
        self.assertIsNone(cfg.dash)
        self.assertIsNone(cfg.points_mode)

    def test_parse_valid_dash(self):
        cfg = parse_session_config(_dash_data())
        self.assertEqual(cfg.mode, "dash")
        self.assertIsInstance(cfg.dash, DashConfig)
        self.assertEqual(cfg.dash.on_distance_m, 0.5)
        self.assertEqual(cfg.dash.off_distance_m, 0.3)
        self.assertEqual(cfg.dash.start_state, "off")
        self.assertIsNone(cfg.points_mode)

    def test_parse_valid_point(self):
        cfg = parse_session_config(_point_data())
        self.assertEqual(cfg.mode, "point")
        self.assertIsInstance(cfg.points_mode, PointsModeConfig)
        self.assertEqual(cfg.points_mode.coordinates, ((0.0, 0.0), (1.0, 1.0)))
        self.assertEqual(cfg.points_mode.arrival_tolerance_m, 0.1)
        self.assertEqual(cfg.points_mode.heading_tolerance_deg, 5.0)
        self.assertEqual(cfg.points_mode.arrival_settle_s, 0.5)
        self.assertEqual(cfg.points_mode.dwell_s, 2.0)
        self.assertIsNone(cfg.dash)

    def test_parse_valid_point_null_heading_tolerance(self):
        data = _point_data()
        data["points_mode"]["heading_tolerance_deg"] = None
        cfg = parse_session_config(data)
        self.assertIsNone(cfg.points_mode.heading_tolerance_deg)

    def test_parse_valid_point_empty_coordinates(self):
        data = _point_data()
        data["points_mode"]["coordinates"] = []
        cfg = parse_session_config(data)
        self.assertEqual(cfg.points_mode.coordinates, ())


class SchemaVersionTest(unittest.TestCase):
    def test_schema_version_mismatch_raises(self):
        data = _continuous_data()
        data["schema_version"] = SCHEMA_VERSION + 1
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_schema_version_missing_raises(self):
        data = _continuous_data()
        del data["schema_version"]
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_unknown_top_level_key_is_tolerated(self):
        """Additive schema evolution: unknown top-level keys must not reject.

        R3 / SCHEMA_VERSION trap: if the parser rejected unknowns, a new server
        could not add optional keys without a version bump, and a bump against
        an old node fails-static into the previous mission's mode. Prove the
        additive path is available before relying on it.
        """
        data = _continuous_data()
        data["some_future_key"] = 1
        cfg = parse_session_config(data)
        self.assertEqual(cfg.mode, "continuous")
        self.assertEqual(cfg.schema_version, SCHEMA_VERSION)


class PointsFlagsLengthTest(unittest.TestCase):
    def test_mismatched_length_raises(self):
        data = _continuous_data(points=[[0.0, 0.0], [1.0, 0.0]], flags=[False])
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_non_list_points_raises(self):
        data = _continuous_data()
        data["points"] = "not-a-list"
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_malformed_point_pair_raises(self):
        data = _continuous_data(points=[[0.0, 0.0, 0.0]], flags=[False])
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)


class ModeSubConfigTest(unittest.TestCase):
    def test_unknown_mode_raises(self):
        data = _continuous_data()
        data["mode"] = "bogus"
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_dash_missing_dash_subconfig_raises(self):
        data = _dash_data()
        del data["dash"]
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_dash_with_points_mode_present_raises(self):
        data = _dash_data()
        data["points_mode"] = _point_data()["points_mode"]
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_point_missing_points_mode_subconfig_raises(self):
        data = _point_data()
        del data["points_mode"]
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_point_with_dash_present_raises(self):
        data = _point_data()
        data["dash"] = _dash_data()["dash"]
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_continuous_with_dash_present_raises(self):
        data = _continuous_data()
        data["dash"] = _dash_data()["dash"]
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_continuous_with_points_mode_present_raises(self):
        data = _continuous_data()
        data["points_mode"] = _point_data()["points_mode"]
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)


class DashFieldValidationTest(unittest.TestCase):
    def test_invalid_start_state_raises(self):
        data = _dash_data()
        data["dash"]["start_state"] = "sideways"
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_negative_on_distance_raises(self):
        data = _dash_data()
        data["dash"]["on_distance_m"] = -0.5
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_negative_off_distance_raises(self):
        data = _dash_data()
        data["dash"]["off_distance_m"] = -0.1
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_zero_on_distance_raises(self):
        data = _dash_data()
        data["dash"]["on_distance_m"] = 0.0
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)


class PointsModeFieldValidationTest(unittest.TestCase):
    def test_negative_arrival_tolerance_raises(self):
        data = _point_data()
        data["points_mode"]["arrival_tolerance_m"] = -0.1
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_negative_arrival_settle_raises(self):
        data = _point_data()
        data["points_mode"]["arrival_settle_s"] = -1.0
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_negative_dwell_raises(self):
        data = _point_data()
        data["points_mode"]["dwell_s"] = -2.0
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_negative_heading_tolerance_raises(self):
        data = _point_data()
        data["points_mode"]["heading_tolerance_deg"] = -5.0
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_non_finite_coordinate_raises(self):
        data = _point_data()
        data["points_mode"]["coordinates"] = [[float("inf"), 0.0]]
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_nan_coordinate_raises(self):
        data = _continuous_data(points=[[float("nan"), 0.0]], flags=[False])
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)


class PathFingerprintTest(unittest.TestCase):
    def test_stable_for_identical_geometry(self):
        cfg_a = parse_session_config(_continuous_data())
        cfg_b = parse_session_config(_continuous_data())
        self.assertEqual(cfg_a.path_fingerprint(), cfg_b.path_fingerprint())

    def test_differs_when_point_changes(self):
        cfg_a = parse_session_config(_continuous_data())
        cfg_b = parse_session_config(
            _continuous_data(points=[[0.0, 0.0], [1.0, 0.5], [2.0, 0.0]])
        )
        self.assertNotEqual(cfg_a.path_fingerprint(), cfg_b.path_fingerprint())

    def test_differs_when_flag_changes(self):
        cfg_a = parse_session_config(_continuous_data())
        cfg_b = parse_session_config(_continuous_data(flags=[False, False, True]))
        self.assertNotEqual(cfg_a.path_fingerprint(), cfg_b.path_fingerprint())

    def test_fingerprint_not_part_of_public_parse_contract(self):
        # A caller-supplied "fingerprint" field must simply be ignored --
        # never trusted/validated (plan §3, defect #3).
        data = _continuous_data()
        data["path_fingerprint"] = "bogus-caller-supplied-hash"
        cfg = parse_session_config(data)
        self.assertNotEqual(cfg.path_fingerprint(), "bogus-caller-supplied-hash")


class ToDictRoundTripTest(unittest.TestCase):
    def test_round_trip_continuous(self):
        cfg = parse_session_config(_continuous_data())
        d = to_dict(cfg)
        cfg2 = parse_session_config(d)
        self.assertEqual(cfg, cfg2)

    def test_round_trip_dash(self):
        cfg = parse_session_config(_dash_data())
        d = to_dict(cfg)
        cfg2 = parse_session_config(d)
        self.assertEqual(cfg, cfg2)

    def test_round_trip_point(self):
        cfg = parse_session_config(_point_data())
        d = to_dict(cfg)
        cfg2 = parse_session_config(d)
        self.assertEqual(cfg, cfg2)

    def test_json_dumps_allow_nan_false_succeeds(self):
        for data in (_continuous_data(), _dash_data(), _point_data()):
            cfg = parse_session_config(data)
            d = to_dict(cfg)
            serialized = json.dumps(d, allow_nan=False)
            self.assertIsInstance(serialized, str)
            # And it comes back the same way in.
            reloaded = json.loads(serialized)
            self.assertEqual(reloaded, d)

    def test_to_dict_has_null_dash_and_points_mode_for_continuous(self):
        cfg = parse_session_config(_continuous_data())
        d = to_dict(cfg)
        self.assertIsNone(d["dash"])
        self.assertIsNone(d["points_mode"])
        self.assertNotIn("max_xtrack_error_m", d)


class MaxXtrackErrorMTest(unittest.TestCase):
    """R3: additive optional per-mission xtrack gate (SCHEMA_VERSION unchanged)."""

    def test_absent_parses_as_none(self):
        cfg = parse_session_config(_continuous_data())
        self.assertIsNone(cfg.max_xtrack_error_m)

    def test_null_parses_as_none(self):
        data = _continuous_data()
        data["max_xtrack_error_m"] = None
        cfg = parse_session_config(data)
        self.assertIsNone(cfg.max_xtrack_error_m)

    def test_positive_value_parses(self):
        data = _continuous_data()
        data["max_xtrack_error_m"] = 0.03
        cfg = parse_session_config(data)
        self.assertAlmostEqual(cfg.max_xtrack_error_m, 0.03)

    def test_zero_rejected(self):
        data = _continuous_data()
        data["max_xtrack_error_m"] = 0.0
        with self.assertRaises(ConfigSchemaError) as ctx:
            parse_session_config(data)
        self.assertIn("must be > 0", str(ctx.exception))

    def test_negative_rejected(self):
        data = _continuous_data()
        data["max_xtrack_error_m"] = -0.01
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)

    def test_to_dict_omits_when_none_includes_when_set(self):
        cfg = parse_session_config(_continuous_data())
        self.assertNotIn("max_xtrack_error_m", to_dict(cfg))
        data = _continuous_data()
        data["max_xtrack_error_m"] = 0.025
        cfg2 = parse_session_config(data)
        d2 = to_dict(cfg2)
        self.assertEqual(d2["max_xtrack_error_m"], 0.025)
        self.assertEqual(parse_session_config(d2), cfg2)


class ClearedConfigTest(unittest.TestCase):
    def test_cleared_config_is_continuous_and_empty(self):
        cfg = cleared_config()
        self.assertEqual(cfg.mode, "continuous")
        self.assertEqual(cfg.points, tuple())
        self.assertEqual(cfg.flags, tuple())
        self.assertIsNone(cfg.dash)
        self.assertIsNone(cfg.points_mode)
        self.assertEqual(cfg.schema_version, SCHEMA_VERSION)

    def test_cleared_config_round_trips(self):
        cfg = cleared_config()
        d = to_dict(cfg)
        json.dumps(d, allow_nan=False)
        cfg2 = parse_session_config(d)
        self.assertEqual(cfg, cfg2)

    def test_cleared_config_unloads_previous_mission(self):
        # Simulate a node latching a mission, then a mission-clear
        # publishing an explicit cleared config over it (plan §3).
        latched = parse_session_config(_continuous_data())
        self.assertNotEqual(latched.points, tuple())
        cleared = cleared_config()
        self.assertEqual(cleared.points, tuple())
        self.assertNotEqual(latched.path_fingerprint(), cleared.path_fingerprint())


class ContinuousConfigFromPathTest(unittest.TestCase):
    def test_matches_expected_points_and_flags(self):
        points = [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0)]
        flags = [False, True, True]
        cfg = continuous_config_from_path(points, flags)
        self.assertEqual(cfg.mode, "continuous")
        self.assertEqual(cfg.points, tuple(points))
        self.assertEqual(cfg.flags, tuple(flags))
        self.assertIsNone(cfg.dash)
        self.assertIsNone(cfg.points_mode)
        self.assertEqual(cfg.schema_version, SCHEMA_VERSION)

    def test_degraded_load_is_all_false_flags(self):
        points = [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0)]
        healthy = continuous_config_from_path(points, [False, True, True])
        degraded = continuous_config_from_path(points, [False, False, False])
        self.assertEqual(degraded.points, healthy.points)
        self.assertEqual(degraded.flags, (False, False, False))
        self.assertNotEqual(degraded.path_fingerprint(), healthy.path_fingerprint())

    def test_mismatched_length_raises(self):
        with self.assertRaises(ConfigSchemaError):
            continuous_config_from_path([(0.0, 0.0), (1.0, 0.0)], [False])

    def test_empty_path(self):
        cfg = continuous_config_from_path([], [])
        self.assertEqual(cfg.points, tuple())
        self.assertEqual(cfg.flags, tuple())


class MiscTypeCoercionTest(unittest.TestCase):
    def test_data_not_a_dict_raises(self):
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(["not", "a", "dict"])  # type: ignore[arg-type]

    def test_flags_coerced_to_bool(self):
        data = _continuous_data(flags=[0, 1, 1])
        cfg = parse_session_config(data)
        self.assertEqual(cfg.flags, (False, True, True))

    def test_bool_in_numeric_field_rejected(self):
        data = _dash_data()
        data["dash"]["on_distance_m"] = True
        with self.assertRaises(ConfigSchemaError):
            parse_session_config(data)


if __name__ == "__main__":
    unittest.main()
