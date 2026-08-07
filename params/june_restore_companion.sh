#!/usr/bin/env bash
# JUNE-15 RESTORE — companion (RPP) side.
#
# Run ON THE JETSON, AFTER rpp-pipeline is up. These are RUNTIME params: every
# `systemctl restart rpp-pipeline` wipes them back to the code defaults, so
# re-run this after any restart and BEFORE the run.
#
#   ssh flash@192.168.1.102 'bash -s' < params/june_restore_companion.sh
#
# ⚠ Check `armed` and mission state first — a restart or a param change
#   mid-run in OFFBOARD trips the failsafe (NAV_RCL_ACT/NAV_DLL_ACT = Disarm).
set -euo pipefail

NODE=/rpp_controller          # from super().__init__("rpp_controller")
                              # confirm with: ros2 node list | grep rpp

set_p() { echo "  $1 = $2"; ros2 param set "$NODE" "$1" "$2" >/dev/null; }

echo "June-15 restore -> $NODE"

# --- the seven that actually moved since 2026-06-15 -------------------------
set_p lookahead_time                    1.6      # was 1.0  -> Ld 0.52 -> 0.56 m
set_p approach_velocity_scaling_dist    0.6      # was 0.9  (leftover from an
                                                 #  abandoned 0.70 m/s test)
set_p segment_heading_tolerance_deg     2.0      # was 3.0
set_p segment_align_settle_s            0.10     # was 0.20
set_p use_imu_extrapolation             false    # was true (P2.4, 07-31)
set_p pivot_to_intercept_enabled        false    # D1, still field-unverified
set_p segment_endpoint_lookahead_extend false    # D15

# --- hold at the June operating point ---------------------------------------
set_p mission_speed                     0.35
set_p min_lookahead_dist                0.52     # code default; runtime has
                                                 #  been set to 0.35 before

# --- deliberately NOT reverted ----------------------------------------------
#   entry_prealign_enabled            LEAVE True  — essential, and it is also
#                                                   the spray pivot gate.
#   endpoint_approach_run_remaining    already False (correct)
#   stop_latch_enabled                 already False (its A/B is separate)
#   segment_slowdown_dist / segment_stop_* / a_lat_max / max_yaw_rate_body /
#   xy_goal_tolerance / corner_smooth_radius_m — all already at June values.

echo
echo "verify:"
ros2 param get "$NODE" lookahead_time
ros2 param get "$NODE" approach_velocity_scaling_dist
ros2 param get "$NODE" use_imu_extrapolation
