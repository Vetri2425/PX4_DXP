#!/usr/bin/env python3
"""Unit tests for the pure spray telemetry/status contract module.

Runnable both as:
    python3 -m pytest src/test_spray_status.py
    python3 src/test_spray_status.py
"""

from __future__ import annotations

import json
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from spray_status import (  # noqa: E402
    STATUS_SCHEMA_VERSION,
    SpraySessionStatus,
    make_status,
    sanitize_optional_float,
    status_to_dict,
    status_to_json_safe,
)


def _base_kwargs(**overrides):
    kwargs = dict(
        mode="continuous",
        fsm_state="ON_CONFIRMED",
        spraying=True,
        desired=True,
        manual_active=False,
        safety_ok=True,
        safety_reason="",
        distance_to_boundary_m=1.5,
        gps_fix_ok=True,
        gps_fix_name="RTK_FIXED",
        xtrack_error_m=0.02,
        mode_state={},
    )
    kwargs.update(overrides)
    return kwargs


class SanitizeOptionalFloatTests(unittest.TestCase):
    def test_inf_becomes_none(self):
        self.assertIsNone(sanitize_optional_float(float("inf")))

    def test_neg_inf_becomes_none(self):
        self.assertIsNone(sanitize_optional_float(float("-inf")))

    def test_nan_becomes_none(self):
        self.assertIsNone(sanitize_optional_float(float("nan")))

    def test_none_stays_none(self):
        self.assertIsNone(sanitize_optional_float(None))

    def test_finite_float_passes_through(self):
        self.assertEqual(sanitize_optional_float(3.5), 3.5)

    def test_zero_passes_through_as_float(self):
        result = sanitize_optional_float(0)
        self.assertEqual(result, 0.0)
        self.assertIsInstance(result, float)

    def test_non_numeric_becomes_none(self):
        self.assertIsNone(sanitize_optional_float("not-a-number"))


class NoneRoundTripTests(unittest.TestCase):
    """A status with Optional[float] fields set to None round-trips cleanly."""

    def test_none_distance_and_xtrack_round_trip_clean_json(self):
        status = make_status(
            **_base_kwargs(
                fsm_state="OFF_CONFIRMED",
                spraying=False,
                desired=False,
                distance_to_boundary_m=None,
                xtrack_error_m=None,
            )
        )
        self.assertIsNone(status.distance_to_boundary_m)
        self.assertIsNone(status.xtrack_error_m)

        payload = status_to_json_safe(status)
        self.assertNotIn("Infinity", payload)
        self.assertNotIn("NaN", payload)

        decoded = json.loads(payload)
        self.assertIsNone(decoded["distance_to_boundary_m"])
        self.assertIsNone(decoded["xtrack_error_m"])


class Defect1RegressionTests(unittest.TestCase):
    """Regression coverage for plan §1 defect #1: raw inf reaching json.dumps."""

    def test_make_status_sanitizes_inf_and_nan_to_none(self):
        status = make_status(
            **_base_kwargs(
                distance_to_boundary_m=float("inf"),
                xtrack_error_m=float("nan"),
            )
        )
        self.assertIsNone(status.distance_to_boundary_m)
        self.assertIsNone(status.xtrack_error_m)

    def test_status_to_json_safe_succeeds_after_sanitization(self):
        status = make_status(
            **_base_kwargs(
                distance_to_boundary_m=float("inf"),
                xtrack_error_m=float("nan"),
            )
        )
        # Must not raise -- this is the exact crash-loop class of bug.
        payload = status_to_json_safe(status)
        self.assertNotIn("Infinity", payload)
        self.assertNotIn("NaN", payload)
        decoded = json.loads(payload)
        self.assertIsNone(decoded["distance_to_boundary_m"])
        self.assertIsNone(decoded["xtrack_error_m"])

    def test_neg_inf_also_sanitized(self):
        status = make_status(**_base_kwargs(distance_to_boundary_m=float("-inf")))
        self.assertIsNone(status.distance_to_boundary_m)


class ModeStateSanitizationTests(unittest.TestCase):
    def test_inf_in_mode_state_sanitized_to_none(self):
        status = make_status(
            **_base_kwargs(
                mode_state={"next_boundary_s": float("inf"), "dash_on": True}
            )
        )
        self.assertIsNone(status.mode_state["next_boundary_s"])
        self.assertTrue(status.mode_state["dash_on"])

        payload = status_to_json_safe(status)
        self.assertNotIn("Infinity", payload)
        self.assertNotIn("NaN", payload)

    def test_nested_dict_in_mode_state_sanitized(self):
        status = make_status(
            **_base_kwargs(
                mode_state={"nested": {"a": float("nan"), "b": 2.0}}
            )
        )
        self.assertIsNone(status.mode_state["nested"]["a"])
        self.assertEqual(status.mode_state["nested"]["b"], 2.0)

    def test_list_in_mode_state_sanitized(self):
        status = make_status(
            **_base_kwargs(mode_state={"waypoints": [1.0, float("inf"), 3.0]})
        )
        self.assertEqual(status.mode_state["waypoints"], [1.0, None, 3.0])

    def test_mode_state_default_is_not_shared_between_calls(self):
        status_a = make_status(**_base_kwargs(mode_state=None))
        status_b = make_status(**_base_kwargs(mode_state=None))
        # Mutating one instance's mode_state must not affect the other --
        # proves the default is a fresh dict per call, not a shared object.
        status_a.mode_state["mutated"] = True
        self.assertNotIn("mutated", status_b.mode_state)

    def test_mode_state_defaults_to_empty_dict_when_omitted(self):
        kwargs = _base_kwargs()
        del kwargs["mode_state"]
        status = make_status(**kwargs)
        self.assertEqual(status.mode_state, {})


class MakeStatusDefaultsTests(unittest.TestCase):
    def test_schema_version_defaults(self):
        kwargs = _base_kwargs()
        status = make_status(**kwargs)
        self.assertEqual(status.schema_version, STATUS_SCHEMA_VERSION)

    def test_explicit_schema_version_respected(self):
        status = make_status(**_base_kwargs(schema_version=99))
        self.assertEqual(status.schema_version, 99)


class StatusToDictAndJsonTests(unittest.TestCase):
    def test_status_to_dict_contains_all_fields(self):
        status = make_status(**_base_kwargs())
        d = status_to_dict(status)
        expected_fields = {
            "schema_version",
            "mode",
            "fsm_state",
            "spraying",
            "desired",
            "manual_active",
            "safety_ok",
            "safety_reason",
            "distance_to_boundary_m",
            "gps_fix_ok",
            "gps_fix_name",
            "xtrack_error_m",
            "mode_state",
        }
        self.assertEqual(set(d.keys()), expected_fields)

    def test_status_to_json_safe_is_valid_json_string(self):
        status = make_status(**_base_kwargs())
        payload = status_to_json_safe(status)
        self.assertIsInstance(payload, str)
        json.loads(payload)  # must not raise


class SprayingFsmStateDocumentationTests(unittest.TestCase):
    """spraying=True only makes semantic sense with fsm_state=ON_CONFIRMED.

    This module does NOT enforce that invariant (it has no way to
    independently verify a caller's FSM state) -- the FSM/node is
    responsible for only ever constructing a status that way. This test
    documents the division of responsibility: the dataclass faithfully
    stores whatever it is given, including a "wrong" combination, rather
    than silently coercing or rejecting it.
    """

    def test_dataclass_faithfully_stores_given_fields_even_if_inconsistent(self):
        # Deliberately inconsistent combination -- this module must not
        # raise or silently "fix" it; that responsibility belongs to the
        # FSM/node layer, not this pure data contract.
        status = make_status(
            **_base_kwargs(fsm_state="OFF_CONFIRMED", spraying=True)
        )
        self.assertEqual(status.fsm_state, "OFF_CONFIRMED")
        self.assertTrue(status.spraying)

    def test_consistent_on_confirmed_combination_stores_cleanly(self):
        status = make_status(
            **_base_kwargs(fsm_state="ON_CONFIRMED", spraying=True)
        )
        self.assertEqual(status.fsm_state, "ON_CONFIRMED")
        self.assertTrue(status.spraying)


class NoInfinityOrNanAcrossAllOutputsTests(unittest.TestCase):
    """Sweep: 'Infinity'/'NaN' must never appear in any status_to_json_safe output."""

    def _all_statuses(self):
        return [
            make_status(**_base_kwargs()),
            make_status(
                **_base_kwargs(distance_to_boundary_m=None, xtrack_error_m=None)
            ),
            make_status(
                **_base_kwargs(
                    distance_to_boundary_m=float("inf"),
                    xtrack_error_m=float("nan"),
                )
            ),
            make_status(**_base_kwargs(distance_to_boundary_m=float("-inf"))),
            make_status(
                **_base_kwargs(mode_state={"a": float("inf"), "b": float("nan")})
            ),
            make_status(
                **_base_kwargs(
                    mode="dash",
                    fsm_state="DISABLED",
                    spraying=False,
                    desired=False,
                    safety_ok=False,
                    safety_reason="gps_fix_degraded",
                    mode_state={"segments": [{"len_m": float("inf")}]},
                )
            ),
        ]

    def test_no_infinity_or_nan_token_in_any_output(self):
        for status in self._all_statuses():
            payload = status_to_json_safe(status)
            self.assertNotIn("Infinity", payload)
            self.assertNotIn("NaN", payload)
            # Also confirm it is loadable JSON, not just token-absent.
            json.loads(payload)

    def test_frozen_dataclass_is_immutable(self):
        status = make_status(**_base_kwargs())
        with self.assertRaises(Exception):
            status.spraying = False  # type: ignore[misc]

    def test_direct_construction_with_inf_is_possible_but_json_dumps_would_raise(self):
        # Demonstrates why make_status (not the raw dataclass constructor)
        # is the load-bearing safety boundary: bypassing it and building
        # SpraySessionStatus directly can still hold inf, and allow_nan=False
        # then raises -- exactly the backstop behavior the plan calls for.
        status = SpraySessionStatus(
            schema_version=STATUS_SCHEMA_VERSION,
            **_base_kwargs(distance_to_boundary_m=float("inf")),
        )
        self.assertTrue(math.isinf(status.distance_to_boundary_m))
        with self.assertRaises(ValueError):
            json.dumps(status_to_dict(status), allow_nan=False)


class TestModeStateKeySanitization(unittest.TestCase):
    def test_nonfinite_float_key_does_not_break_json(self):
        # Regression for defect #1's exact crash class via a dict KEY (not
        # value): a nan/inf float key must not survive to json.dumps.
        for bad in (float("nan"), float("inf"), float("-inf")):
            status = make_status(
                mode="continuous",
                fsm_state="OFF_CONFIRMED",
                spraying=False,
                desired=False,
                manual_active=False,
                safety_ok=True,
                safety_reason="",
                distance_to_boundary_m=None,
                gps_fix_ok=True,
                gps_fix_name="",
                xtrack_error_m=None,
                mode_state={bad: 1.0, "ok": 2.0},
            )
            out = status_to_json_safe(status)
            self.assertNotIn("NaN", out)
            self.assertNotIn("Infinity", out)
            # Round-trips cleanly.
            json.loads(out)

    def test_non_serializable_mode_state_never_crashes_publish(self):
        # Defect-#1 class via a NON-float payload: a set/bytes value and a
        # tuple/nan key must not raise in status_to_json_safe (they'd take
        # down the node at the publish boundary). They are stringified.
        status = make_status(
            mode="dash",
            fsm_state="ON_CONFIRMED",
            spraying=True,
            desired=True,
            manual_active=False,
            safety_ok=True,
            safety_reason="",
            distance_to_boundary_m=None,
            gps_fix_ok=True,
            gps_fix_name="",
            xtrack_error_m=None,
            mode_state={"s": {1, 2}, "b": b"xy", ("t", "k"): 5, float("nan"): 9},
        )
        out = status_to_json_safe(status)  # must not raise
        self.assertNotIn("NaN", out)
        self.assertNotIn("Infinity", out)
        json.loads(out)  # valid JSON

    def test_finite_keys_pass_through(self):
        status = make_status(
            mode="dash",
            fsm_state="ON_CONFIRMED",
            spraying=True,
            desired=True,
            manual_active=False,
            safety_ok=True,
            safety_reason="",
            distance_to_boundary_m=1.5,
            gps_fix_ok=True,
            gps_fix_name="RTK_FIXED",
            xtrack_error_m=0.01,
            mode_state={"phase": "on", "s_at_last_toggle": 3.2},
        )
        out = json.loads(status_to_json_safe(status))
        self.assertEqual(out["mode_state"]["phase"], "on")
        self.assertEqual(out["mode_state"]["s_at_last_toggle"], 3.2)


if __name__ == "__main__":
    unittest.main()
