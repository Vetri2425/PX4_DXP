# Task 04 — pkill Pattern Does Not Match Real MAVROS Process Name

**Priority:** HIGH
**File:** `px4_start_service.sh`
**Line:** 118

---

## Problem

The stale-process cleanup before starting MAVROS uses the wrong pattern:

```bash
# px4_start_service.sh:118
pkill -f "mavros px4.launch" 2>/dev/null || true
```

The actual MAVROS process command line on this system (confirmed from `systemctl status`):

```
/usr/bin/python3 /opt/ros/humble/bin/ros2 launch mavros node.launch fcu_url:=/dev/ttyACM0:921600 ...
```

The pattern `"mavros px4.launch"` will **never match** this. The `pkill` silently does nothing (`|| true` suppresses the non-zero exit).

## Impact

Every service restart leaves the previous MAVROS process alive. The watchdog then starts a new `mavros_node`, and you have two instances competing for:
- `/dev/ttyACM0` — the second will fail to open the serial device (`Device or resource busy`) and crash immediately, but only after spewing errors into the journal
- The `/mavros` ROS2 node namespace — duplicate node names cause ROS2 topic graph corruption

In practice the old `mavros_node` likely exits when its serial fd gets stolen, but this is race-dependent. The correct fix eliminates the race entirely.

## Evidence in Codebase

`systemctl status` output shows the real process tree:

```
├─6529 /usr/bin/python3 /opt/ros/humble/bin/ros2 launch mavros node.launch ...
└─6556 /opt/ros/humble/lib/mavros/mavros_node --ros-args ...
```

Two processes to kill: the `ros2 launch` parent and the `mavros_node` child.

## Required Fix (do not apply — analysis only)

```bash
# Kill both the launch wrapper and the actual mavros_node binary
pkill -f "mavros node.launch" 2>/dev/null || true
pkill -f "mavros_node" 2>/dev/null || true
sleep 1   # let the OS reclaim /dev/ttyACM0 before new instance opens it
```

The `sleep 1` after pkill is appropriate here (unlike in the watchdog loop) because we need the serial device to be released before the new MAVROS instance opens it.

---

**Depends on:** None
**Blocks:** Nothing, but should be fixed before any restart testing
