"""EKF-origin cache invalidation in RosBridgeNode (field bug, 2026-07-27).

The cached origin lives in the SERVER's process state, which outlives both the
FCU and MAVROS. When either restarts, a new EKF session can declare a different
origin — three different origins up to 4.8 m apart were recorded on this rig in
one afternoon — and the server went on serving the dead session's value with no
symptom until the rover drove 2.15 m off the line.

These tests drive the REAL `_cb_state` / `_cb_gp_origin` / `_invalidate_ekf_origin`
methods. Only the ROS plumbing (rclpy, message types) is stubbed, and the node is
built with ``__new__`` so no subscriptions/timers are created — nothing about the
logic under test is replaced.

How each test would fail if the code were wrong
-----------------------------------------------
* reconnect test: if `_cb_state` did not invalidate, `ekf_origin_received` would
  still be True after the link came back and the assertion flips.
* gap test: if the gap threshold were read from the wrong clock or the wrong
  constant, the 6 s gap would not trip and `received` stays True.
* no-false-positive test: a steady 1 Hz /mavros/state stream must NOT invalidate;
  if it did, the field would see spurious refusals every second, so this is the
  guard against over-triggering, which the reconnect test alone cannot give.
* re-arm test: the original bug was `_origin_req_count` only ever being reset at
  process init. If that reset were dropped, the counter stays at its cap.
"""

from __future__ import annotations

import os
import sys
import threading
import types

sys.path.insert(0, os.path.dirname(__file__))


# ── Minimal ROS plumbing stubs (module import only; no node is spun) ─────────
def _install_ros_stubs() -> None:
    if "rclpy" in sys.modules:
        return

    def _mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    class _Node:
        def __init__(self, *a, **k):
            pass

    class _Enum:
        RELIABLE = BEST_EFFORT = TRANSIENT_LOCAL = VOLATILE = KEEP_LAST = 1

    rclpy = _mod("rclpy", ok=lambda: True, init=lambda *a, **k: None)
    _mod("rclpy.callback_groups",
         MutuallyExclusiveCallbackGroup=object, ReentrantCallbackGroup=object)
    _mod("rclpy.executors", MultiThreadedExecutor=object)
    _mod("rclpy.node", Node=_Node)
    _mod("rclpy.qos", DurabilityPolicy=_Enum, HistoryPolicy=_Enum,
         QoSProfile=lambda **k: None, ReliabilityPolicy=_Enum)
    rclpy.node = sys.modules["rclpy.node"]
    _mod("geometry_msgs")
    _mod("geometry_msgs.msg", PoseStamped=object, Vector3Stamped=object)
    _mod("nav_msgs")
    _mod("nav_msgs.msg", Path=object)
    _mod("std_msgs")
    _mod("std_msgs.msg", Bool=object, Float32MultiArray=object, String=object)


_install_ros_stubs()

import ros_node as ros_node_mod  # noqa: E402
from origin_health import INCONSISTENT, NO_ORIGIN, OK, evaluate_origin_health  # noqa: E402


class _FakeState:
    def __init__(self, connected: bool, armed: bool = False, mode: str = "OFFBOARD"):
        self.connected = connected
        self.armed = armed
        self.mode = mode


class _FakeGpOrigin:
    """geographic_msgs/GeoPointStamped shape, only what _cb_gp_origin reads."""

    def __init__(self, lat: float, lon: float, sec: int = 100):
        self.position = types.SimpleNamespace(latitude=lat, longitude=lon, altitude=0.0)
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=sec, nanosec=0)
        )


def _bare_node():
    """A RosBridgeNode with only the fields the origin logic touches."""
    node = ros_node_mod.RosBridgeNode.__new__(ros_node_mod.RosBridgeNode)
    node._lock = threading.Lock()
    node._state = dict(ros_node_mod.RosBridgeNode._DEFAULT_STATE)
    node._state_recv_time = None
    node._origin_req_count = 0
    node._MAVROS_STATE_TIMEOUT_S = 2.0
    return node


ORIGIN_A = (13.0720437, 80.2619664)   # the 14:39 EKF session (from the bags)
ORIGIN_B = (13.0720521, 80.2619705)   # the post-reboot session (15:10 declared)


# ── 1. Reboot / reconnect invalidates the cache ──────────────────────────────

def test_fcu_link_regained_invalidates_the_cached_origin():
    node = _bare_node()
    node._cb_state(_FakeState(connected=True))
    node._cb_gp_origin(_FakeGpOrigin(*ORIGIN_A))
    assert node._state["ekf_origin_received"] is True

    # FCU reboots: MAVROS drops the link, then picks it back up.
    node._cb_state(_FakeState(connected=False))
    assert node._state["ekf_origin_received"] is True, (
        "losing the link alone must not drop the datum — only regaining it can, "
        "because only then could a NEW EKF session exist")
    node._cb_state(_FakeState(connected=True))

    assert node._state["ekf_origin_received"] is False
    assert node._state["ekf_origin_lat"] == 0.0
    assert node._state["ekf_origin_lon"] == 0.0
    assert "link re-established" in node._state["ekf_origin_invalid_reason"]
    assert node._state["ekf_origin_invalidations"] == 1


def test_mavros_restart_gap_invalidates_even_when_connected_never_flips():
    """A MAVROS process restart re-publishes connected=True with no False in
    between (TRANSIENT_LOCAL), so the flip test cannot see it. The /mavros/state
    receive-time gap can."""
    node = _bare_node()
    node._cb_state(_FakeState(connected=True))
    node._cb_gp_origin(_FakeGpOrigin(*ORIGIN_A))

    # Fake a 6 s hole in /mavros/state (> ORIGIN_LINK_GAP_S = 5 s default).
    node._state_recv_time -= 6.0
    node._cb_state(_FakeState(connected=True))

    assert node._state["ekf_origin_received"] is False
    assert "gap" in node._state["ekf_origin_invalid_reason"]


def test_a_steady_state_stream_never_invalidates():
    """False-positive guard: every spurious invalidation is a real refusal
    window in the field. 30 consecutive 1 Hz publishes must change nothing."""
    node = _bare_node()
    node._cb_state(_FakeState(connected=True))
    node._cb_gp_origin(_FakeGpOrigin(*ORIGIN_A))
    for _ in range(30):
        node._state_recv_time -= 1.0        # 1 Hz, the real /mavros/state rate
        node._cb_state(_FakeState(connected=True))

    assert node._state["ekf_origin_received"] is True
    assert node._state["ekf_origin_invalidations"] == 0
    assert node._state["ekf_origin_lat"] == ORIGIN_A[0]


# ── 2. The request path re-arms ──────────────────────────────────────────────

def test_request_budget_rearms_on_invalidation_and_on_a_fresh_origin():
    """The original bug: `_origin_req_count` was reset ONLY at process init, so
    once the cap was reached the server never asked PX4 for the origin again."""
    node = _bare_node()
    node._cb_state(_FakeState(connected=True))
    node._origin_req_count = ros_node_mod.ORIGIN_REQUEST_MAX_TRIES   # exhausted

    node._cb_state(_FakeState(connected=False))
    node._cb_state(_FakeState(connected=True))
    assert node._origin_req_count == 0, "reconnect must re-arm the request budget"

    node._origin_req_count = 7
    node._cb_gp_origin(_FakeGpOrigin(*ORIGIN_B))
    assert node._origin_req_count == 0, "a fresh origin must re-arm it too"
    assert node._state["ekf_origin_invalid_reason"] is None


def test_a_new_origin_after_a_reboot_is_adopted():
    node = _bare_node()
    node._cb_state(_FakeState(connected=True))
    node._cb_gp_origin(_FakeGpOrigin(*ORIGIN_A))
    node._cb_state(_FakeState(connected=False))
    node._cb_state(_FakeState(connected=True))
    node._cb_gp_origin(_FakeGpOrigin(*ORIGIN_B))

    assert node._state["ekf_origin_received"] is True
    assert node._state["ekf_origin_lat"] == ORIGIN_B[0]
    assert node._state["ekf_origin_lon"] == ORIGIN_B[1]


# ── 3. Invalidation and the consistency gate compose ─────────────────────────

def _sample_state(node, lat, lon, pos_n=0.0, pos_e=0.0):
    """A fresh, simultaneous, RTK_FIXED sample laid over the node's state."""
    s = dict(node._state)
    s.update(
        connected=True, pose_received=True, global_position_received=True,
        gps_fix_received=True, gps_fix=6,
        local_pose_age_ms=10.0, global_position_age_ms=10.0,
        gps_fix_age_ms=10.0, pose_global_skew_ms=5.0,
        lat=lat, lon=lon, pos_n=pos_n, pos_e=pos_e,
    )
    return s


def test_reboot_then_stale_relatch_is_still_caught_by_the_gate():
    """Belt and braces. If MAVROS re-latches the OLD origin after the reboot,
    invalidation is undone — the measurement is what catches it."""
    node = _bare_node()
    node._cb_state(_FakeState(connected=True))
    node._cb_gp_origin(_FakeGpOrigin(*ORIGIN_A))
    node._cb_state(_FakeState(connected=False))
    node._cb_state(_FakeState(connected=True))

    assert evaluate_origin_health(_sample_state(node, *ORIGIN_B)).status == NO_ORIGIN

    node._cb_gp_origin(_FakeGpOrigin(*ORIGIN_A))          # stale value comes back
    health = evaluate_origin_health(_sample_state(node, *ORIGIN_B))
    assert health.status == INCONSISTENT
    assert health.delta_m > 0.9, health.detail

    node._cb_gp_origin(_FakeGpOrigin(*ORIGIN_B))          # the live one arrives
    assert evaluate_origin_health(_sample_state(node, *ORIGIN_B)).status == OK
