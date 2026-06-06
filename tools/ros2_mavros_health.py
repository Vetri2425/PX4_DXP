#!/usr/bin/env python3
"""Fast rclpy health probes for px4_start_service.sh.

Avoids `ros2 node list` / `ros2 topic echo`, which go through the ROS2 CLI
daemon and can be slow or stale during MAVROS restarts.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from mavros_msgs.msg import State
except ImportError:
    State = None


def _full_node_names(node: Node) -> set[str]:
    names: set[str] = set()
    for name, namespace in node.get_node_names_and_namespaces():
        namespace = namespace or "/"
        if namespace == "/":
            names.add(f"/{name}")
        else:
            names.add(f"{namespace.rstrip('/')}/{name}")
    return names


def check_node(node_name: str, timeout: float) -> int:
    rclpy.init()
    node = Node("px4_dxp_node_probe")
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if node_name in _full_node_names(node):
                return 0
            rclpy.spin_once(node, timeout_sec=0.1)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def check_state(timeout: float, require_connected: bool) -> int:
    if State is None:
        print("mavros_msgs not available", file=sys.stderr)
        return 2

    rclpy.init()
    node = Node("px4_dxp_state_probe")
    seen = {"connected": False, "received": False}
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )

    def _cb(msg: State) -> None:
        seen["received"] = True
        seen["connected"] = bool(msg.connected)

    node.create_subscription(State, "/mavros/state", _cb, qos)
    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if seen["received"] and (
                seen["connected"] or not require_connected
            ):
                return 0
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=5.0)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_node = sub.add_parser("node")
    p_node.add_argument("name")

    p_state = sub.add_parser("state")
    p_state.add_argument("--require-connected", action="store_true")

    args = parser.parse_args()
    if args.cmd == "node":
        return check_node(args.name, args.timeout)
    if args.cmd == "state":
        return check_state(args.timeout, args.require_connected)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
