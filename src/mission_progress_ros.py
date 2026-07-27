#!/usr/bin/env python3
"""ROS adapter for the mission-progress contract (design §4).

Thin rclpy glue over the pure `mission_progress` module: it turns the plain
`QoSSpec` data into real `rclpy` `QoSProfile`s so both nodes build identical
QoS from the single source of truth. This file imports rclpy and therefore is
NOT imported by the Mac unit tests — the pure contract + its tests
(`test_mission_progress.py`) stay rclpy-free. On-robot, the RPP node, the
spray node, and the server import the QoS builder here.
"""

from __future__ import annotations

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

import mission_progress as mp

_RELIABILITY = {
    "BEST_EFFORT": ReliabilityPolicy.BEST_EFFORT,
    "RELIABLE": ReliabilityPolicy.RELIABLE,
}
_DURABILITY = {
    "VOLATILE": DurabilityPolicy.VOLATILE,
    "TRANSIENT_LOCAL": DurabilityPolicy.TRANSIENT_LOCAL,
}


def qos_profile(spec: mp.QoSSpec) -> QoSProfile:
    """Build a QoSProfile from a contract QoSSpec (KEEP_LAST history)."""
    return QoSProfile(
        depth=spec.depth,
        reliability=_RELIABILITY[spec.reliability],
        durability=_DURABILITY[spec.durability],
        history=HistoryPolicy.KEEP_LAST,
    )
