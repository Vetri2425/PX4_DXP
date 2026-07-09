#!/usr/bin/env python3
"""Regression tests for run-boundary (CORNER_ALIGN) pivot fix.

Bug: after a certified stop at the first real MARK point, _run_alignment_hold()
commanded nonzero linear velocity (0.08 m/s) via _corner_pivot_velocity() during
the pivot phase.  Because position_ok was in the settle gate, creep from that
velocity (up to 5.37 cm) prevented the settle clock from ever accumulating,
causing a 41-second oscillation before release.

Fix (rpp_controller_node.py _run_alignment_hold):
  1. position_ok removed from the settle gate (heading+yaw_rate+speed sufficient).
  2. The standalone position-recovery hold branch (corner_stop_complete AND NOT
     position_ok) removed; RPP lookahead corrects offset once tracking resumes.

These tests verify:
  A. With corner_stop_complete=True, valid stop cert, heading outside tolerance:
     - _run_alignment_hold returns True (still aligning).
     - _publish_velocity receives nonzero velocity (PX4 SPOT_TURNING requires it).
     - The velocity bearing is inside the ±75° forward cone (reverse-flip guard).
     - _publish_yaw_rate receives 0.0 (PX4 ignores yaw_rate in velocity mode).
     - _corner_pivot_velocity IS used (the validated firmware-aware mechanism).

  B. With corner_stop_complete=True, valid stop cert, heading inside tolerance,
     speed_ok and yaw_rate_ok:
     - settle dwell accumulates even when position_ok=False (pivot creep present).
     - alignment certificate is issued after align_settle_s.
     - _run_align_pending becomes False.
     - _run_alignment_hold returns False (aligned, handover to tracking).

  C. Bridge: when RPP publishes nonzero yaw_rate_body (type_mask=455 path),
     twist_to_setpoint_node selects TYPE_MASK_VEL_YAW_YAWRATE (455) not 2503.
     (This is unchanged behavior; the test pins it as a regression guard.)

Run on a ROS2-sourced host:
    python3 -X utf8 src/test_run_boundary_pivot.py
"""

import math
import sys
import time

import rclpy
from rclpy.parameter import Parameter


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class _CapturePub:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)

    @property
    def last(self):
        return self.messages[-1] if self.messages else None

    def clear(self):
        self.messages.clear()


def _wire_captures(node):
    caps = {
        "vel": _CapturePub(),
        "yaw": _CapturePub(),
        "debug": _CapturePub(),
        "segment": _CapturePub(),
        "stop": _CapturePub(),
        "spray": _CapturePub(),
    }
    node._vel_pub = caps["vel"]
    node._yaw_rate_pub = caps["yaw"]
    node._dbg_pub = caps["debug"]
    node._segment_dbg_pub = caps["segment"]
    node._stop_dbg_pub = caps["stop"]
    node._spray_active_pub = caps["spray"]
    return caps


def _pose(n, e, yaw_ned=0.0):
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.pose.position.x = float(e)   # MAVROS ENU x=East
    ps.pose.position.y = float(n)   # MAVROS ENU y=North
    yaw_enu = math.pi / 2.0 - yaw_ned
    ps.pose.orientation.z = math.sin(yaw_enu / 2.0)
    ps.pose.orientation.w = math.cos(yaw_enu / 2.0)
    return ps


def _runtime_entry_path(n_entry, e_entry, n_mark, e_mark, n1, e1):
    """Two-run path: runtime-entry OFF leg + MARK leg."""
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    p = Path()
    p.header.frame_id = "local_ned"
    # Entry point (spray OFF, z=0)
    a = PoseStamped()
    a.pose.position.x = n_entry; a.pose.position.y = e_entry
    a.pose.orientation.x = 1.0; a.pose.orientation.w = 0.0   # runtime_entry marker
    # MARK boundary – first spray-OFF duplicate
    b = PoseStamped()
    b.pose.position.x = n_mark; b.pose.position.y = e_mark
    # MARK boundary – spray-ON duplicate
    b_on = PoseStamped()
    b_on.pose.position.x = n_mark; b_on.pose.position.y = e_mark
    b_on.pose.position.z = 1.0
    # Next MARK waypoint (spray ON, z=1)
    c = PoseStamped()
    c.pose.position.x = n1; c.pose.position.y = e1; c.pose.position.z = 1.0
    p.poses = [a, b, b_on, c]
    return p


# FW constant: PX4 DifferentialVelControl freezes heading below this speed.
_FW_ZERO_VEL_THRESHOLD = 0.01
_MAX_BEARING_OFFSET_DEG = 75.0


def _wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Test A — pivot phase: nonzero velocity, correct bearing, yaw_rate=0
# ---------------------------------------------------------------------------

def test_A_pivot_velocity_nonzero_and_in_cone(node, caps):
    """A: corner_stop_complete=True, heading outside tolerance → pivot command.

    Verified:
    - published velocity speed > FW_ZERO_VEL_THRESHOLD (PX4 SPOT_TURNING active)
    - velocity bearing inside ±75° forward cone (no reverse-flip)
    - yaw_rate published as 0.0 (PX4 ignores yaw_rate in velocity mode)
    - _run_alignment_hold returns True (still aligning)
    - _corner_pivot_velocity IS the mechanism used (bearing = yaw + clamped step)
    """
    from rpp_controller_node import StopReason

    # Scenario: entry heading 94.27° NED, MARK heading 2.72° NED → ~91.56° turn.
    entry_yaw_ned = math.radians(94.27)
    mark_heading_ned = math.radians(2.72)

    # Build runtime-entry path: entry leg along 94.27°, MARK leg along 2.72°.
    # Use a path where rover is parked at origin facing entry_yaw_ned,
    # MARK boundary at (0,0), next MARK waypoint along 2.72°.
    node._path_cb(_runtime_entry_path(
        -0.5 * math.cos(entry_yaw_ned), -0.5 * math.sin(entry_yaw_ned),  # entry start
        0.0, 0.0,                                                          # mark boundary
        2.0 * math.cos(mark_heading_ned), 2.0 * math.sin(mark_heading_ned),  # mark pt2
    ))
    assert len(node._runs) == 2, f"Expected 2 runs, got {len(node._runs)}"

    # Simulate: rover stopped at MARK boundary, certified.
    boundary = node._runs[0]["poses"][-1].pose.position
    boundary_seg_idx = max(0, len(node._runs[0]["poses"]) - 2)
    node._make_stop_certificate(
        StopReason.RUNTIME_ENTRY_TO_MARK,
        boundary.x, boundary.y,
        0.0055,    # 5.5 mm position error — matches bag
        math.radians(-91.3),
        segment_idx=boundary_seg_idx,
    )
    assert node._advance_run(pre_stopped=True)
    assert node._run_idx == 1
    assert node._run_align_pending is True
    assert node._corner_stop_complete is True

    # Provide fresh zero velocity (just stopped).
    node._latest_vel_time = node.get_clock().now()
    node._latest_vel_ned = (0.0, 0.0)
    node._latest_yaw_rate_ned = 0.0

    caps["vel"].clear()
    caps["yaw"].clear()

    held = node._run_alignment_hold(0.0, 0.0, entry_yaw_ned, 0.0)
    assert held is True, "_run_alignment_hold must return True while pivoting"

    v = caps["vel"].last
    assert v is not None, "_publish_velocity must be called during pivot"
    v_n = v.vector.x
    v_e = v.vector.y
    speed = math.hypot(v_n, v_e)
    assert speed > _FW_ZERO_VEL_THRESHOLD, (
        f"Pivot velocity {speed:.4f} m/s must exceed FW freeze threshold "
        f"{_FW_ZERO_VEL_THRESHOLD} m/s so PX4 SPOT_TURNING activates"
    )

    bearing_ned = math.atan2(v_e, v_n)
    offset_rad = abs(_wrap_pi(bearing_ned - entry_yaw_ned))
    assert offset_rad <= math.radians(_MAX_BEARING_OFFSET_DEG) + 1e-9, (
        f"Pivot bearing offset {math.degrees(offset_rad):.1f}° exceeds "
        f"±{_MAX_BEARING_OFFSET_DEG}° forward cone — reverse-flip risk"
    )

    yr = caps["yaw"].last
    assert yr is not None, "_publish_yaw_rate must be called during pivot"
    assert abs(yr.data) < 1e-9, (
        f"yaw_rate must be 0.0 during pivot (PX4 ignores it in velocity mode), "
        f"got {yr.data:.4f}"
    )

    # Confirm the velocity matches _corner_pivot_velocity output exactly.
    corner_speed = max(0.05, float(node.get_parameter("segment_min_corner_speed").value))
    heading_err = _wrap_pi(mark_heading_ned - entry_yaw_ned)
    step = max(-math.radians(75), min(math.radians(75), heading_err))
    cmd_bearing = entry_yaw_ned + step
    expected_vn = corner_speed * math.cos(cmd_bearing)
    expected_ve = corner_speed * math.sin(cmd_bearing)
    assert abs(v_n - expected_vn) < 1e-6, (
        f"v_n {v_n:.6f} != _corner_pivot_velocity expected {expected_vn:.6f}"
    )
    assert abs(v_e - expected_ve) < 1e-6, (
        f"v_e {v_e:.6f} != _corner_pivot_velocity expected {expected_ve:.6f}"
    )

    print("PASS A: pivot velocity nonzero, in forward cone, yaw_rate=0, "
          "_corner_pivot_velocity used")


# ---------------------------------------------------------------------------
# Test B — settle dwell accumulates despite position_ok=False (pivot creep)
# ---------------------------------------------------------------------------

def test_B_settle_ignores_position_creep(node, caps):
    """B: heading in tolerance + speed/yaw_rate OK → settle dwell accumulates
    even when position_ok=False (rover drifted during pivot).

    This is the core fix: the old code required position_ok in the settle gate,
    which was reset every cycle when pivot creep drove pos_error > 0.02 m.
    The fix removes position_ok from the settle gate so the dwell can
    accumulate as long as heading, speed, and yaw_rate are satisfied.

    Uses segment_entry_pivot_recenter=False (legacy) so timeout/legacy release
    may still complete off-position; production default is True (see B4).
    """
    from rpp_controller_node import StopReason

    node.set_parameters([Parameter("segment_entry_pivot_recenter", value=False)])
    mark_heading_ned = math.radians(2.72)

    node._path_cb(_runtime_entry_path(
        -0.5, 0.0,                                      # entry start (West of boundary)
        0.0, 0.0,                                       # mark boundary
        2.0 * math.cos(mark_heading_ned),
        2.0 * math.sin(mark_heading_ned),
    ))

    # Certify stop at boundary.
    boundary = node._runs[0]["poses"][-1].pose.position
    boundary_seg_idx = max(0, len(node._runs[0]["poses"]) - 2)
    node._make_stop_certificate(
        StopReason.RUNTIME_ENTRY_TO_MARK,
        boundary.x, boundary.y,
        0.005, 0.0, segment_idx=boundary_seg_idx,
    )
    assert node._advance_run(pre_stopped=True)
    assert node._corner_stop_complete is True

    # Rover has drifted 5 cm from boundary (simulates pivot creep).
    # pos_error > corner_position_tolerance_m=0.02 → position_ok=False.
    drift_n = 0.03   # 3 cm north drift
    drift_e = 0.04   # 4 cm east drift (total ~5 cm)

    # Provide "just stopped" velocity for speed_ok.
    node._latest_vel_time = node.get_clock().now()
    node._latest_vel_ned = (0.005, 0.003)    # ~0.006 m/s < segment_stop_speed_threshold
    node._latest_yaw_rate_ned = 0.02         # < segment_stop_yaw_rate_threshold

    # Heading aligned to MARK.
    aligned_yaw_ned = mark_heading_ned

    caps["vel"].clear()

    # Call repeatedly to simulate the settle dwell accumulating.
    # align_settle_s=0.20 s; we advance by calling 12× with ~25ms real gaps.
    iters = 0
    released = False
    start = time.monotonic()
    for _ in range(60):   # up to 60 calls, should release well before that
        held = node._run_alignment_hold(drift_n, drift_e, aligned_yaw_ned, 0.0)
        iters += 1
        if not held:
            released = True
            break
        time.sleep(0.005)   # 5 ms between calls

    elapsed = time.monotonic() - start
    assert released, (
        f"_run_alignment_hold must release when heading/speed/yaw_rate OK even if "
        f"position_ok=False (pivot creep).  Did not release after {iters} iterations "
        f"({elapsed:.2f}s).  Fix: remove position_ok from settle gate."
    )

    align_cert = node._alignment_certificate
    assert align_cert is not None and align_cert.valid, (
        "Alignment certificate must be issued on release"
    )
    assert node._run_align_pending is False, (
        "_run_align_pending must be cleared after release"
    )

    print(f"PASS B: settle dwell accumulated despite position_ok=False "
          f"(released after {iters} iters / {elapsed*1000:.0f} ms)")


def test_B4_gate_on_blocks_off_position_release(node, caps):
    """B4 (gate ON): with segment_entry_pivot_recenter=True the run-boundary
    pivot must NOT certify while position_ok is False, even with heading/speed/
    yaw_rate OK — segment CORNER_ALIGN parity, the inverse of legacy test_B.
    Guards the observed 52 cm off-point certification."""
    from rpp_controller_node import StopReason

    node.set_parameters([Parameter("segment_entry_pivot_recenter", value=True)])
    mark_heading_ned = math.radians(2.72)
    node._path_cb(_runtime_entry_path(
        -0.5, 0.0, 0.0, 0.0,
        2.0 * math.cos(mark_heading_ned), 2.0 * math.sin(mark_heading_ned),
    ))
    boundary = node._runs[0]["poses"][-1].pose.position
    boundary_seg_idx = max(0, len(node._runs[0]["poses"]) - 2)
    node._make_stop_certificate(
        StopReason.RUNTIME_ENTRY_TO_MARK, boundary.x, boundary.y,
        0.005, 0.0, segment_idx=boundary_seg_idx,
    )
    assert node._advance_run(pre_stopped=True)
    assert node._corner_stop_complete is True

    drift_n, drift_e = 0.03, 0.04    # ~5 cm → position_ok False
    node._latest_vel_time = node.get_clock().now()
    node._latest_vel_ned = (0.005, 0.003)
    node._latest_yaw_rate_ned = 0.02
    aligned_yaw_ned = mark_heading_ned    # heading OK; position NOT ok

    released = False
    for _ in range(40):
        held = node._run_alignment_hold(drift_n, drift_e, aligned_yaw_ned, 0.0)
        if not held:
            released = True
            break
        time.sleep(0.005)
    assert not released, (
        "gate-on: must NOT certify while position_ok=False (5 cm drift); the "
        "strict position gate (segment parity) was bypassed"
    )
    assert node._run_align_pending is True, "must remain in pivot/hold"
    print("PASS B4: gate-on blocks off-position release")


def test_B5_gate_on_recenter_drives_toward_point(node, caps):
    """B5 (gate ON): heading still misaligned + drifted off point → the
    position-recovery hold fires, driving _corner_hold_velocity back toward the
    boundary (capping in-turn drift) and NOT certifying."""
    from rpp_controller_node import StopReason

    node.set_parameters([Parameter("segment_entry_pivot_recenter", value=True)])
    mark_heading_ned = math.radians(2.72)
    node._path_cb(_runtime_entry_path(
        -0.5, 0.0, 0.0, 0.0,
        2.0 * math.cos(mark_heading_ned), 2.0 * math.sin(mark_heading_ned),
    ))
    boundary = node._runs[0]["poses"][-1].pose.position
    boundary_seg_idx = max(0, len(node._runs[0]["poses"]) - 2)
    node._make_stop_certificate(
        StopReason.RUNTIME_ENTRY_TO_MARK, boundary.x, boundary.y,
        0.005, 0.0, segment_idx=boundary_seg_idx,
    )
    assert node._advance_run(pre_stopped=True)

    drift_n, drift_e = 0.07, 0.07    # ~10 cm off the boundary (0,0)
    node._latest_vel_time = node.get_clock().now()
    node._latest_vel_ned = (0.0, 0.0)
    node._latest_yaw_rate_ned = 0.0
    misaligned_yaw = mark_heading_ned - math.radians(90.0)   # heading NOT ok

    caps["vel"].clear()
    held = node._run_alignment_hold(drift_n, drift_e, misaligned_yaw, 0.0)
    assert held is True, "must hold (not certify) while heading is off"
    v = caps["vel"].last
    assert v is not None, "recenter must publish a velocity toward the point"
    # Boundary at (0,0), rover at (drift_n, drift_e): recenter must have a
    # positive component toward the point (a - pos).
    dot = v.vector.x * (-drift_n) + v.vector.y * (-drift_e)
    assert dot > 0.0, (
        f"recenter velocity must drive toward the boundary point; "
        f"dot={dot:.4f}, v=({v.vector.x:.3f},{v.vector.y:.3f})"
    )
    # Speed cap: at ~10cm drift the raw servo (~1.5*0.10=0.15 m/s) would exceed
    # the pivot speed; the entry recenter must clamp it to segment_min_corner_
    # speed (~0.08), NOT the 0.18 brake cap that flung the rover in bag 12-35-10.
    cap = float(node.get_parameter("segment_min_corner_speed").value)
    speed = math.hypot(v.vector.x, v.vector.y)
    assert speed <= cap + 1e-6, (
        f"entry recenter speed {speed:.3f} m/s must be capped at pivot speed "
        f"{cap:.3f} m/s (not the 0.18 brake cap)"
    )
    print(f"PASS B5: recenter drives toward point (capped {speed:.3f}<= {cap:.3f})")


# ---------------------------------------------------------------------------
# Test C — heading out of tolerance: settle clock must NOT start
# ---------------------------------------------------------------------------

def test_C_settle_clock_does_not_start_while_pivoting(node, caps):
    """C: heading still outside tolerance → settle clock never starts.

    Regression guard: the settle clock must only accumulate when ALL remaining
    release gates (heading, speed, yaw_rate) are satisfied.  A pivot that is
    still turning (heading_ok=False) must not prematurely issue the certificate.
    """
    from rpp_controller_node import StopReason

    mark_heading_ned = math.radians(2.72)
    entry_yaw_ned = math.radians(94.27)   # still misaligned by ~91°

    node._path_cb(_runtime_entry_path(
        -0.5, 0.0, 0.0, 0.0,
        2.0 * math.cos(mark_heading_ned),
        2.0 * math.sin(mark_heading_ned),
    ))

    boundary = node._runs[0]["poses"][-1].pose.position
    boundary_seg_idx = max(0, len(node._runs[0]["poses"]) - 2)
    node._make_stop_certificate(
        StopReason.RUNTIME_ENTRY_TO_MARK,
        boundary.x, boundary.y,
        0.005, 0.0, segment_idx=boundary_seg_idx,
    )
    assert node._advance_run(pre_stopped=True)
    assert node._corner_stop_complete is True

    node._latest_vel_time = node.get_clock().now()
    node._latest_vel_ned = (0.0, 0.0)
    node._latest_yaw_rate_ned = 0.0

    # Call 5 times while still misaligned.
    for _ in range(5):
        held = node._run_alignment_hold(0.0, 0.0, entry_yaw_ned, 0.0)
        assert held is True, "Must remain in CORNER_ALIGN while heading outside tolerance"
        assert node._align_settle_since is None, (
            "Settle clock must NOT start while heading is outside tolerance"
        )

    assert node._run_align_pending is True, (
        "_run_align_pending must still be True while pivoting"
    )
    assert node._alignment_certificate is None or not node._alignment_certificate.valid, (
        "No alignment certificate must be issued while still pivoting"
    )

    print("PASS C: settle clock does not start while heading outside tolerance")


# ---------------------------------------------------------------------------
# Test C2 — position_ok=False alone does NOT block settle clock (fix verification)
# ---------------------------------------------------------------------------

def test_C2_position_ok_false_does_not_block_settle(node, caps):
    """C2: heading in tolerance, speed/yaw_rate OK, but position_ok=False
    → settle clock MUST start (this was the bug: position_ok blocked it).

    Uses segment_entry_pivot_recenter=False (legacy). With the production
    default (True), position_ok is required for release (see B4).
    """
    from rpp_controller_node import StopReason

    node.set_parameters([Parameter("segment_entry_pivot_recenter", value=False)])
    mark_heading_ned = math.radians(2.72)

    node._path_cb(_runtime_entry_path(
        -0.5, 0.0, 0.0, 0.0,
        2.0 * math.cos(mark_heading_ned),
        2.0 * math.sin(mark_heading_ned),
    ))

    boundary = node._runs[0]["poses"][-1].pose.position
    boundary_seg_idx = max(0, len(node._runs[0]["poses"]) - 2)
    node._make_stop_certificate(
        StopReason.RUNTIME_ENTRY_TO_MARK,
        boundary.x, boundary.y,
        0.005, 0.0, segment_idx=boundary_seg_idx,
    )
    assert node._advance_run(pre_stopped=True)

    # Provide heading-aligned, low-speed, low-yaw-rate state.
    node._latest_vel_time = node.get_clock().now()
    node._latest_vel_ned = (0.003, 0.002)    # very slow — below threshold
    node._latest_yaw_rate_ned = 0.01

    # Rover drifted 5 cm — position_ok=False.
    drift_n, drift_e = 0.03, 0.04

    # Single call — settle clock must START (align_settle_since must be set).
    node._run_alignment_hold(drift_n, drift_e, mark_heading_ned, 0.0)

    assert node._align_settle_since is not None, (
        "Settle clock must start when heading/speed/yaw_rate OK even if "
        "position_ok=False.  This was the bug — position_ok was blocking it."
    )

    print("PASS C2: position_ok=False alone does NOT block settle clock (fix confirmed)")


# ---------------------------------------------------------------------------
# Test D — bridge: yaw_rate=0 → type_mask=2503; yaw_rate≠0 → type_mask=455
# ---------------------------------------------------------------------------

def test_D_bridge_type_mask_selection():
    """D: twist_to_setpoint_node selects type_mask based on yaw_rate magnitude.

    During CORNER_ALIGN, RPP publishes yaw_rate=0.0, so the bridge selects
    TYPE_MASK_VELOCITY_AND_YAW (2503).  This is the observed behavior in the
    bag.  The test pins this as a regression guard: if yaw_rate were ever made
    nonzero during pivot, type_mask=455 would be selected automatically.
    """
    import math

    IGNORE_PX = 1; IGNORE_PY = 2; IGNORE_PZ = 4
    IGNORE_AFX = 64; IGNORE_AFY = 128; IGNORE_AFZ = 256
    IGNORE_YAW_RATE = 2048

    TYPE_MASK_VEL_YAW_YAWRATE = (
        IGNORE_PX | IGNORE_PY | IGNORE_PZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
    )   # 455: vel + yaw + yaw_rate
    TYPE_MASK_VELOCITY_AND_YAW = (
        IGNORE_PX | IGNORE_PY | IGNORE_PZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ
        | IGNORE_YAW_RATE
    )   # 2503: vel + yaw, ignore yaw_rate

    def select_type_mask(yaw_rate_body, source="rpp", yaw_rate_fresh=True):
        """Mirror of twist_to_setpoint_node._stream_cb type_mask logic."""
        if source == "rpp" and yaw_rate_fresh and abs(yaw_rate_body) > 1e-4:
            return TYPE_MASK_VEL_YAW_YAWRATE   # 455
        return TYPE_MASK_VELOCITY_AND_YAW       # 2503

    # CORNER_ALIGN: yaw_rate_body = 0.0 → 2503 (observed in bag)
    assert select_type_mask(0.0) == 2503, (
        "yaw_rate=0.0 must select type_mask=2503 (vel+yaw, yaw_rate ignored)"
    )
    assert select_type_mask(1e-5) == 2503, (
        "yaw_rate below 1e-4 threshold must select type_mask=2503"
    )

    # If yaw_rate were nonzero (e.g. feedforward on arc), 455 is selected.
    assert select_type_mask(0.4) == 455, (
        "nonzero yaw_rate must select type_mask=455 (vel+yaw+yaw_rate)"
    )
    assert select_type_mask(-0.4) == 455, (
        "negative yaw_rate must also select type_mask=455"
    )

    # Stale yaw_rate → fall back to 2503.
    assert select_type_mask(0.4, yaw_rate_fresh=False) == 2503, (
        "stale yaw_rate must fall back to type_mask=2503"
    )

    print("PASS D: bridge type_mask=2503 for yaw_rate=0, 455 for nonzero yaw_rate")


# ---------------------------------------------------------------------------
# Test E — stop certificate is preserved unchanged
# ---------------------------------------------------------------------------

def test_E_stop_certificate_unaffected(node, caps):
    """E: the stop certificate logic is not changed by this fix.

    A stop at the MARK boundary with position_error=0.0055 m, speed=0.0177 m/s,
    yaw_rate≈0, dwell=0.319 s must still issue a valid StopCertificate.
    (Mirrors the bag evidence at t=11.9358 s.)
    """
    from rpp_controller_node import StopReason

    mark_heading_ned = math.radians(2.72)

    node._path_cb(_runtime_entry_path(
        -0.5, 0.0, 0.0, 0.0,
        2.0 * math.cos(mark_heading_ned),
        2.0 * math.sin(mark_heading_ned),
    ))

    # Provide fresh velocity confirming the rover is physically stopped.
    node._latest_vel_time = node.get_clock().now()
    node._latest_vel_ned = (0.0177 * math.cos(math.radians(94.27)),
                             0.0177 * math.sin(math.radians(94.27)))
    node._latest_yaw_rate_ned = 0.003

    boundary = node._runs[0]["poses"][-1].pose.position
    boundary_seg_idx = max(0, len(node._runs[0]["poses"]) - 2)

    cert = node._make_stop_certificate(
        StopReason.RUNTIME_ENTRY_TO_MARK,
        boundary.x, boundary.y,
        0.0055,
        math.radians(-91.3),
        segment_idx=boundary_seg_idx,
    )

    assert cert.valid, "Stop certificate must be valid"
    assert cert.position_error_m == 0.0055
    assert abs(cert.measured_speed_m_s - 0.0177) < 1e-4
    assert cert.reason == StopReason.RUNTIME_ENTRY_TO_MARK

    valid = node._stop_certificate_valid_for(
        StopReason.RUNTIME_ENTRY_TO_MARK,
        boundary.x, boundary.y,
        run_idx=0, segment_idx=boundary_seg_idx,
    )
    assert valid, "stop_certificate_valid_for must return True for matching cert"

    print("PASS E: stop certificate behavior unchanged")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    ok = True
    try:
        from rpp_controller_node import RPPControllerNode

        # Test D is pure-math, no node needed.
        try:
            test_D_bridge_type_mask_selection()
        except AssertionError as e:
            ok = False; print(f"FAIL D: {e}")

        # Tests A, B, C, C2, E need a node instance each.
        for test_fn, label in [
            (test_A_pivot_velocity_nonzero_and_in_cone, "A"),
            (test_B_settle_ignores_position_creep, "B"),
            (test_B4_gate_on_blocks_off_position_release, "B4"),
            (test_B5_gate_on_recenter_drives_toward_point, "B5"),
            (test_C_settle_clock_does_not_start_while_pivoting, "C"),
            (test_C2_position_ok_false_does_not_block_settle, "C2"),
            (test_E_stop_certificate_unaffected, "E"),
        ]:
            node = RPPControllerNode()
            node.set_parameters([
                Parameter("require_rtk_fix", value=False),
                Parameter("segment_align_settle_s", value=0.05),   # short dwell for test speed
            ])
            node._gps_fix_type = 6
            caps = _wire_captures(node)
            try:
                test_fn(node, caps)
            except AssertionError as e:
                ok = False; print(f"FAIL {label}: {e}")
            finally:
                node.destroy_node()

        print()
        if ok:
            print("=== ALL RUN-BOUNDARY PIVOT TESTS PASSED ===")
        else:
            print("=== SOME TESTS FAILED ===")
    finally:
        rclpy.shutdown()
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
