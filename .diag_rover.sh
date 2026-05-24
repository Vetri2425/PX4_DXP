#!/bin/bash
# Diagnose why rover isn't moving despite RTK FIXED
source /opt/ros/humble/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0

echo "=== MAVROS STATE ==="
timeout 4 ros2 topic echo /mavros/state --once 2>&1 | head -15

echo
echo "=== GPS FIX TYPE ==="
timeout 4 ros2 topic echo /mavros/gpsstatus/gps1/raw --once 2>&1 | grep -E "fix_type|^---"

echo
echo "=== RPP DEBUG (state[7], xtrack[0], speed[3], pose_age[6]) ==="
timeout 4 ros2 topic echo /rpp/debug --once 2>&1 | head -10

echo
echo "=== RPP VELOCITY OUT ==="
timeout 4 ros2 topic echo /rpp/velocity_ned --once 2>&1 | head -8

echo
echo "=== TWIST TO SETPOINT OUTPUT ==="
timeout 4 ros2 topic echo /mavros/setpoint_raw/local --once 2>&1 | head -25

echo
echo "=== MAVROS LOCAL POSITION ==="
timeout 4 ros2 topic echo /mavros/local_position/pose --once 2>&1 | head -15

echo
echo "=== ACTIVE TOPICS WITH HZ ==="
for t in /rpp/velocity_ned /mavros/setpoint_raw/local /mavros/local_position/pose /rpp/debug; do
  echo "--- $t ---"
  timeout 3 ros2 topic hz "$t" 2>&1 | tail -2
done

echo
echo "=== MISSION RUNNER LOG (LAST 30) ==="
sudo -n journalctl --since "5 minutes ago" 2>/dev/null | grep -E "mission_runner|rpp_controller|twist_to_setpoint" | tail -30
