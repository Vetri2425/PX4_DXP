#!/usr/bin/env python3
"""D2 — runtime-entry run-0 pre-align gate (entry_prealign_enabled).

Run 0 has no prev_run, so _apply_run's alignment block is skipped and the rover
arcs onto the first heading at tracking speed (E2E audit gap 9). When
entry_prealign_enabled is true, run 0 sets _run_align_pending so _run_alignment_hold
pivots in place first (spray OFF). Default off = validated missions unchanged.

Run on a ROS2-sourced host (needs rclpy), isolated domain:
    ROS_DOMAIN_ID=42 python3 -X utf8 src/test_entry_prealign.py
"""
import sys
import math

import rclpy
from rclpy.parameter import Parameter


def _pose(n, e):
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.pose.position.x = float(n)
    ps.pose.position.y = float(e)
    ps.pose.orientation.w = 1.0
    return ps


def main():
    rclpy.init(args=["--ros-args", "-p", "require_rtk_fix:=false"])
    ok = True
    try:
        from rpp_controller_node import RPPControllerNode
        node = RPPControllerNode()
        P = lambda **kw: node.set_parameters([Parameter(k, value=v) for k, v in kw.items()])
        run = {"poses": [_pose(0.0, 0.0), _pose(2.0, 0.0)], "flags": [False, False],
               "profile": "segment", "cum_s": [0.0, 2.0], "closed": False}
        node._runs = [run]

        # ---- default OFF: run 0 does NOT pre-align (validated behaviour) ----
        P(entry_prealign_enabled=False)
        node._apply_run(0)
        assert node._run_align_pending is False, "default must not pre-align run 0"
        print("PASS 1: default off — run 0 does not pre-align (no regression)")

        # ---- enabled: run 0 latches a pre-align pivot ----
        P(entry_prealign_enabled=True)
        node._apply_run(0)
        assert node._run_align_pending is True, "enabled must pre-align run 0"
        assert node._run_align_turn_rad == math.pi, "budget worst-case π when pose unknown"
        print("PASS 2: enabled — run 0 latches pre-align (spray OFF via _run_alignment_hold)")

        # ---- single-point run 0 never pre-aligns (no heading to face) ----
        node._runs = [{"poses": [_pose(0.0, 0.0)], "flags": [False], "profile": "segment",
                       "cum_s": [0.0], "closed": False}]
        P(entry_prealign_enabled=True)
        node._apply_run(0)
        assert node._run_align_pending is False, "single-point run 0 has no heading to align to"
        print("PASS 3: single-point run 0 does not pre-align")

        print("\nALL D2 ENTRY-PREALIGN TESTS PASSED")
    except AssertionError as e:
        ok = False
        print(f"FAIL: {e}")
    except Exception as e:  # noqa: BLE001
        ok = False
        print(f"ERROR: {type(e).__name__}: {e}")
    finally:
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
