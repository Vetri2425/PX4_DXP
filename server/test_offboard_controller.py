import asyncio
import os
import sys
from collections import deque

import pytest

sys.path.insert(0, os.path.dirname(__file__))

import offboard_controller as offboard_module
from config import RPP_IDLE, RPP_TRACKING
from mission_placement import PlacementError
from models import MissionState
from offboard_controller import OffboardController


class FakeRppMonitor:
    def reset(self):
        pass


class FakeNode:
    def __init__(self, states):
        self._states = list(states)
        self.calls = []

    def get_state(self):
        if len(self._states) > 1:
            return self._states.pop(0)
        return self._states[0]

    def get_rpp_monitor(self):
        return FakeRppMonitor()

    def publish_path(self, points, frame_id="local_ned", spray_flags=None,
                     must_hit_flags=None):
        self.calls.append(("publish_path", list(points), spray_flags))

    def publish_spray_manual(self, on):
        self.calls.append(("spray_manual", bool(on)))

    async def arm_async(self, arm):
        self.calls.append(("arm", arm))
        return True, ""

    async def set_mode_async(self, mode):
        self.calls.append(("set_mode", mode))
        return True, ""


def run(coro):
    return asyncio.run(coro)


def test_start_publishes_path_before_arm_and_offboard():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        node = FakeNode([
            {"connected": True, "rpp_state": RPP_TRACKING},
            {"connected": True, "rpp_state": RPP_TRACKING},
        ])
        ctrl = OffboardController(node, deque())
        ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="test")

        ok, msg = run(ctrl.start_async())

        assert ok is True
        assert msg == "running"
        assert ctrl.state == MissionState.RUNNING
        assert node.calls == [
            ("publish_path", [(1.0, 2.0), (3.0, 4.0)], None),
            ("arm", True),
            ("set_mode", "OFFBOARD"),
        ]
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_start_disarms_if_rpp_stays_idle_after_path_publish():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        node = FakeNode([
            {"connected": True, "rpp_state": RPP_TRACKING},
            {"connected": True, "rpp_state": RPP_IDLE},
        ])
        ctrl = OffboardController(node, deque())
        ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="test")

        ok, msg = run(ctrl.start_async())

        assert ok is False
        assert "RPP IDLE after path publish" in msg
        assert ctrl.state == MissionState.ERROR
        assert node.calls == [
            ("publish_path", [(1.0, 2.0), (3.0, 4.0)], None),
            ("arm", True),
            ("arm", False),
        ]
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


# The EKF local-frame origin this field sample actually implies. Placement
# refuses outright without a declared origin that agrees with the live
# pose/global pair (origin_health), so a fixture that omits it is not a
# telemetry state the rover can ever be in.
_SURVEY_EKF_ORIGIN = (13.072019284527872, 80.26196407384334)


def _healthy_survey_state(rpp_state=RPP_TRACKING):
    return {
        "connected": True,
        "rpp_state": rpp_state,
        "pose_received": True,
        "global_position_received": True,
        "gps_fix_received": True,
        "local_pose_age_ms": 20.0,
        "global_position_age_ms": 15.0,
        "gps_fix_age_ms": 100.0,
        "pose_global_skew_ms": 5.0,
        "gps_fix": 6,
        "pos_n": 7.4629,
        "pos_e": -0.9070,
        "lat": 13.0720864,
        "lon": 80.2619557,
        "ekf_origin_received": True,
        "ekf_origin_lat": _SURVEY_EKF_ORIGIN[0],
        "ekf_origin_lon": _SURVEY_EKF_ORIGIN[1],
    }


def test_surveyed_start_rejects_auto_origin():
    ctrl = OffboardController(FakeNode([_healthy_survey_state()]), deque())
    ctrl.load_path(
        [(0.0, -0.035), (1.0, -0.035)],
        name="surveyed",
        placement_mode="GPS_SURVEYED",
        origin_gps=(13.072066, 80.261956),
    )
    with pytest.raises(PlacementError, match="incompatible with auto_origin"):
        run(ctrl.start_async(auto_origin=True))


def test_surveyed_start_publishes_live_ekf_points():
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        state = _healthy_survey_state()
        node = FakeNode([state, dict(state)])
        ctrl = OffboardController(node, deque())
        source = [(0.0, -0.035), (1.0, -0.035)]
        ctrl.load_path(
            source,
            name="surveyed",
            placement_mode="GPS_SURVEYED",
            origin_gps=(13.072066, 80.261956),
        )

        ok, msg = run(ctrl.start_async())

        assert ok is True
        # D1: a surveyed start with the rover off the first point drives a
        # spray-OFF entry leg first, so start returns "entry" (not "running").
        assert msg == "entry"
        assert ctrl.state == MissionState.ENTRY
        name, entry_pts, entry_flags = node.calls[0]
        assert name == "publish_path"
        # E1 aligned entry: the rover is parked PAST the line start (N=7.46 vs
        # wp0 N=5.19, mark runs +N), so the entry routes via a staging point
        # 1.2 m behind wp0 along the mark direction and arrives collinear.
        assert len(entry_pts) == 3
        assert entry_pts[0] == pytest.approx((7.4629, -0.9070), abs=1e-3)  # live pose
        assert entry_pts[1] == pytest.approx((3.992, -0.910), abs=0.02)    # staging
        assert entry_pts[2] == pytest.approx((5.192, -0.910), abs=0.02)    # entry target
        assert entry_flags == [False, False, False]
        # The full marking path (live-placed) is stashed for phase 2.
        assert ctrl._entry_marking_pts[0] == pytest.approx((5.192, -0.910), abs=0.02)
        # Source resident path remains anchor-relative (not mutated).
        assert ctrl._loaded_pts == source

        # Phase 2: entry stop confirmed (RPP DONE) → publish the marking path.
        advanced = ctrl.advance_entry_to_marking()
        assert advanced is True
        assert ctrl.state == MissionState.RUNNING
        name2, mark_pts, _flags = node.calls[-1]
        assert name2 == "publish_path"
        assert mark_pts[0] == pytest.approx((5.192, -0.910), abs=0.02)   # placed wp0
        assert ctrl._entry_marking_pts is None                           # stash consumed
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_surveyed_start_skips_entry_when_on_first_point():
    """D1 degenerate: rover already within ENTRY_SKIP_DIST of the first point →
    single publish, state RUNNING, no entry leg (no stash)."""
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        anchor = (13.072066, 80.261956)
        state = _healthy_survey_state()
        # Rover parked AT the survey anchor → R_anchor ≈ 0. Both halves of the
        # sample move together: lat/lon AND the local pose the EKF would report
        # for that lat/lon (computed independently of path_engine.ned, small-angle
        # equirectangular on PX4's sphere, agrees to 0.02 mm).
        state["lat"], state["lon"] = anchor
        state["pos_n"], state["pos_e"] = 5.194523496337718, -0.8745059958907082
        node = FakeNode([state, dict(state)])
        ctrl = OffboardController(node, deque())
        # source[0] is 3.5 cm from the anchor origin — inside ENTRY_SKIP_DIST_M.
        source = [(0.0, -0.035), (1.0, -0.035)]
        ctrl.load_path(source, name="surveyed",
                       placement_mode="GPS_SURVEYED", origin_gps=anchor)

        ok, msg = run(ctrl.start_async())

        assert ok is True
        assert msg == "running"                  # no entry leg
        assert ctrl.state == MissionState.RUNNING
        assert ctrl._entry_marking_pts is None   # nothing stashed
        name, pts, _ = node.calls[0]
        assert name == "publish_path"
        # First publish is the placed MARKING path (wp0 = live pose + source[0]),
        # not a [live_pose, target] entry leg.
        assert pts[0] == pytest.approx((5.1945, -0.9095), abs=0.02)
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


# ── E1: aligned-entry staging geometry (docs/ALIGNED_ENTRY_PLAN.md) ──────────
# _entry_leg_points is pure geometry; these pin the staging placement for
# parked positions all around the mission start, plus every degenerate fallback.

# Mission start at (5.0, -1.0), mark running due north: u = (1, 0).
_E1_PATH = [(5.0, -1.0), (5.05, -1.0), (6.0, -1.0)]
_E1_STAGING = (5.0 - offboard_module.ENTRY_STAGING_DIST_M, -1.0)  # (3.8, -1.0)


@pytest.mark.parametrize(
    "live",
    [
        (7.5, -1.0),    # parked past the line end (anti-parallel arrival today)
        (5.0, 2.0),     # parked to the east, perpendicular approach
        (5.0, -4.0),    # parked to the west, perpendicular approach
        (3.0, 1.5),     # behind but well off-axis (~37 deg > 20 deg skip cone)
    ],
)
def test_entry_leg_routes_via_staging(live):
    pts = offboard_module._entry_leg_points(live, _E1_PATH)
    assert len(pts) == 3
    assert pts[0] == live
    assert pts[1] == pytest.approx(_E1_STAGING, abs=1e-9)
    assert pts[2] == (5.0, -1.0)


def test_entry_leg_skips_staging_when_chord_arrives_aligned():
    # Parked 3 m behind the start, 0.5 m off-axis: chord bearing ~9.5 deg off
    # the mark direction — inside the 20 deg cone, so the plain chord is used.
    pts = offboard_module._entry_leg_points((2.0, -0.5), _E1_PATH)
    assert pts == [(2.0, -0.5), (5.0, -1.0)]


def test_entry_leg_skips_staging_when_parked_at_staging_point():
    live = (_E1_STAGING[0] + 0.1, _E1_STAGING[1] + 0.3)
    pts = offboard_module._entry_leg_points(live, _E1_PATH)
    assert pts == [live, (5.0, -1.0)]


def test_entry_leg_degenerate_path_falls_back_to_chord():
    # All placed points within 1 cm of wp0 → no direction → plain chord.
    degenerate = [(5.0, -1.0), (5.004, -1.0), (5.0, -1.004)]
    pts = offboard_module._entry_leg_points((8.0, 2.0), degenerate)
    assert pts == [(8.0, 2.0), (5.0, -1.0)]


def test_entry_leg_off_switch_restores_plain_chord(monkeypatch):
    monkeypatch.setattr(offboard_module, "ENTRY_STAGING_ENABLED", False)
    pts = offboard_module._entry_leg_points((7.5, -1.0), _E1_PATH)
    assert pts == [(7.5, -1.0), (5.0, -1.0)]


def test_clear_mission_resets_resident_state():
    ctrl = OffboardController(FakeNode([{"connected": True, "rpp_state": RPP_IDLE}]), deque())
    ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="sq")
    assert ctrl._loaded_pts == [(1.0, 2.0), (3.0, 4.0)]

    status = run(ctrl.clear_mission_async())

    assert ctrl.state == MissionState.IDLE
    assert ctrl._loaded_pts is None
    assert ctrl.loaded_path_name is None
    assert status["loaded"] is False
    assert status["num_waypoints"] == 0


def test_clear_mission_rejected_while_running():
    ctrl = OffboardController(FakeNode([{"connected": True, "rpp_state": RPP_IDLE}]), deque())
    ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="sq")
    ctrl.state = MissionState.RUNNING

    with pytest.raises(offboard_module.MissionClearConflict):
        run(ctrl.clear_mission_async())

    # A rejected clear leaves state and resident path untouched.
    assert ctrl.state == MissionState.RUNNING
    assert ctrl._loaded_pts == [(1.0, 2.0), (3.0, 4.0)]


def test_loaded_path_summary_exposes_staged_identity():
    # A staged surveyed load, exactly as POST /load-to-controller does
    # (name == the staged mission_id). The operator app's post-load
    # verifyStagedLoadedMission reads mission_id/is_staged/protected/placement_mode.
    ctrl = OffboardController(FakeNode([{"connected": True, "rpp_state": RPP_IDLE}]), deque())
    ctrl.load_path(
        [(1.0, 2.0), (3.0, 4.0)],
        name="stg_abc123_1700",
        placement_mode="GPS_SURVEYED",
        origin_gps=(12.9716, 80.1946),
        is_staged=True,
    )
    s = ctrl.loaded_path_summary()
    assert s["mission_id"] == "stg_abc123_1700"
    assert s["is_staged"] is True
    assert s["protected"] is True
    assert s["placement_mode"] == "GPS_SURVEYED"

    # A plain (non-staged) path must NOT masquerade as a staged mission.
    ctrl2 = OffboardController(FakeNode([{"connected": True, "rpp_state": RPP_IDLE}]), deque())
    ctrl2.load_path([(0.0, 0.0), (1.0, 1.0)], name="square_2x2")
    s2 = ctrl2.loaded_path_summary()
    assert s2["mission_id"] is None
    assert s2["protected"] is False


# ── Mission↔joystick arbiter wiring (plan §7.4) ────────────────────────────────

from control_arbiter import ControlArbiter, ControlOwner


class _FakeStateHolder:
    def __init__(self, state=MissionState.IDLE):
        self.state = state


def _joystick_owned_arbiter():
    arb = ControlArbiter()
    run(arb.begin_joystick_acquire(_FakeStateHolder(MissionState.IDLE)))
    arb.mark_joystick_active("sess-1", "lease-1")
    assert arb.joystick_owned
    return arb


def test_start_rejected_while_joystick_owns_arbiter():
    """A mission must not begin while the joystick owns manual control, and it
    must be refused BEFORE any FCU I/O (no arm, no mode switch)."""
    arb = _joystick_owned_arbiter()
    node = FakeNode([{"connected": True, "rpp_state": RPP_TRACKING}])
    ctrl = OffboardController(node, deque(), arbiter=arb)
    ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="test")

    ok, msg = run(ctrl.start_async())

    assert ok is False
    assert "joystick" in msg.lower()
    assert ctrl.state != MissionState.RUNNING
    assert ("arm", True) not in node.calls  # refused before touching the FCU


def test_start_relinquishes_arbiter_on_success():
    """owner==MISSION is in-flight-only: a successful start leaves the arbiter
    back at IDLE (the running mission is guarded by state, not a sticky owner)."""
    old_grace = offboard_module.SETPOINT_STREAM_GRACE_S
    offboard_module.SETPOINT_STREAM_GRACE_S = 0.0
    try:
        arb = ControlArbiter()
        node = FakeNode([
            {"connected": True, "rpp_state": RPP_TRACKING},
            {"connected": True, "rpp_state": RPP_TRACKING},
        ])
        ctrl = OffboardController(node, deque(), arbiter=arb)
        ctrl.load_path([(1.0, 2.0), (3.0, 4.0)], name="test")

        ok, _ = run(ctrl.start_async())

        assert ok is True
        assert ctrl.state == MissionState.RUNNING
        assert arb.owner == ControlOwner.IDLE  # bracket relinquished on exit
    finally:
        offboard_module.SETPOINT_STREAM_GRACE_S = old_grace


def test_start_relinquishes_arbiter_on_failure():
    """An early-guard failure (no path loaded) must also relinquish the bracket
    so a stranded owner can never block a later joystick acquire."""
    arb = ControlArbiter()
    node = FakeNode([{"connected": True, "rpp_state": RPP_IDLE}])
    ctrl = OffboardController(node, deque(), arbiter=arb)

    ok, _ = run(ctrl.start_async())  # no load_path → early return

    assert ok is False
    assert arb.owner == ControlOwner.IDLE


# ── B4: spray-OFF + disarm on natural completion ────────────────────────────

def _running_ctrl(node=None):
    node = node or FakeNode([{"connected": True, "rpp_state": RPP_TRACKING}])
    ctrl = OffboardController(node, deque())
    ctrl._state = MissionState.RUNNING
    return ctrl, node


def test_mark_completed_reports_transition_edge():
    ctrl, _ = _running_ctrl()
    assert ctrl.mark_completed() is True          # RUNNING → COMPLETED
    assert ctrl.state == MissionState.COMPLETED
    assert ctrl.mark_completed() is False         # already COMPLETED — no edge


def test_completion_commands_spray_off_then_disarm():
    old = offboard_module.config.DISARM_ON_COMPLETE
    offboard_module.config.DISARM_ON_COMPLETE = True
    try:
        ctrl, node = _running_ctrl()
        assert ctrl.mark_completed() is True
        result = run(ctrl.disarm_on_complete_async())
        assert result["spray_off_sent"] is True
        assert result["disarmed"] is True
        # Spray commanded OFF BEFORE the disarm, and disarm actually called.
        assert node.calls == [("spray_manual", False), ("arm", False)]
        # Mission stays COMPLETED (arm_async, not disarm_async → not IDLE).
        assert ctrl.state == MissionState.COMPLETED
    finally:
        offboard_module.config.DISARM_ON_COMPLETE = old


def test_completion_flag_off_leaves_rover_armed():
    old = offboard_module.config.DISARM_ON_COMPLETE
    offboard_module.config.DISARM_ON_COMPLETE = False
    try:
        ctrl, node = _running_ctrl()
        assert ctrl.mark_completed() is True
        result = run(ctrl.disarm_on_complete_async())
        assert result == {"attempted": False, "spray_off_sent": False,
                          "disarmed": False}
        assert node.calls == []                    # old behaviour: no commands
    finally:
        offboard_module.config.DISARM_ON_COMPLETE = old
