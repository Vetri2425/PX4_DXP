#!/usr/bin/env python3
"""Tests for deterministic path identity canonicalization."""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from path_identity import path_geometry_fingerprint  # noqa: E402


def test_signed_zero_canonicalizes_to_same_fingerprint():
    a = path_geometry_fingerprint([(0.0, -0.0), (1.0, 0.0)], [False, True])
    b = path_geometry_fingerprint([(-0.0, 0.0), (1.0, -0.0)], [False, True])
    assert a == b


def test_fingerprint_is_deterministic_and_order_sensitive():
    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    flags = [False, True, False]
    assert path_geometry_fingerprint(points, flags) == path_geometry_fingerprint(points, flags)
    assert path_geometry_fingerprint(points, flags) != path_geometry_fingerprint(
        list(reversed(points)),
        list(reversed(flags)),
    )


def test_fingerprint_rejects_nan_inf_malformed_and_bad_flags():
    bad_cases = [
        ([(math.nan, 0.0), (1.0, 0.0)], [False, True]),
        ([(math.inf, 0.0), (1.0, 0.0)], [False, True]),
        ([("north", 0.0), (1.0, 0.0)], [False, True]),
        ([(0.0, 0.0), (1.0, 0.0)], [0, True]),
        ([(0.0, 0.0)], [False]),
    ]
    for points, flags in bad_cases:
        with pytest.raises(ValueError):
            path_geometry_fingerprint(points, flags)
