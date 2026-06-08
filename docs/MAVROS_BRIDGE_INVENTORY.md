# MAVROS Bridge — Jetson ↔ PX4 Inventory

> Complete catalog of all files and snippets related to the MAVROS bridge connection between Jetson Orin (`192.168.1.102`) and CubeOrangePlus (PX4 v1.16.2) over `/dev/ttyACM0 @ 921600 baud`.
>
> Generated: 2026-06-06

---

## Architecture Overview

```
┌──────────────────────────────────────────────────────────────────────────┐
│                      Jetson Orin (192.168.1.102)                        │
│                                                                          │
│  systemd: px4-dxp.service                                               │
│  └─ px4_start_service.sh                                                │
│       ├─ MAVROS watchdog (launches ros2 launch mavros node.launch)      │
│       │    └─ MAVROS node (/mavros)                                     │
│       │         ├─ Serial: /dev/ttyACM0 @ 921600 → CubeOrangePlus (PX4) │
│       │         ├─ UDP GCS: :14550 (QGC)                                │
│       │         ├─ Plugin list: px4_pluginlists_rover.yaml              │
│       │         └─ Topics:                                              │
│       │              ├─ /mavros/state           (published ~10 Hz)      │
│       │              ├─ /mavros/local_position/pose                     │
│       │              ├─ /mavros/local_position/velocity_local           │
│       │              ├─ /mavros/battery                                 │
│       │              ├─ /mavros/global_position/global                  │
│       │              ├─ /mavros/gpsstatus/gps1/raw                     │
│       │              ├─ /mavros/statustext                              │
│       │              └─ Services:                                       │
│       │                   ├─ /mavros/cmd/arming                         │
│       │                   ├─ /mavros/set_mode                           │
│       │                   ├─ /mavros/param/get_parameters               │
│       │                   └─ /mavros/param/set_parameters               │
│       └─ NTRIP watchdog                                                │
│            └─ ntrip_rtcm_node.py → injects RTCM into /mavros           │
│                                                                          │
│  ROS2 Nodes (separate processes, ROS_DOMAIN_ID=0):                      │
│  ┌─ server/ros_node.py (RosBridgeNode — FastAPI bridge)                │
│  │    ├─ Sub: /mavros/state, /mavros/local_position/pose,              │
│  │    │       /mavros/battery, /mavros/global_position/global,         │
│  │    │       /mavros/gpsstatus/gps1/raw, /rpp/debug,                 │
│  │    │       /rpp/velocity_ned                                        │
│  │    ├─ Pub: /path                                                     │
│  │    ├─ Svc: /mavros/cmd/arming, /mavros/set_mode,                    │
│  │    │       /mavros/param/get_parameters,                            │
│  │    │       /mavros/param/set_parameters                             │
│  │    └─ Runs: FastAPI (port 5001), Socket.IO telemetry                │
│  │                                                                      │
│  ├─ src/twist_to_setpoint_node.py                                      │
│  │    ├─ Sub: /rpp/velocity_ned, /rpp/yaw_rate_body                    │
│  │    └─ Pub: /mavros/setpoint_raw/local @ 50 Hz                       │
│  │                                                                      │
│  ├─ src/rpp_controller_node.py                                         │
│  │    ├─ Sub: /path, /mavros/local_position/pose,                      │
│  │    │       /mavros/gpsstatus/gps1/raw,                              │
│  │    │       /mavros/local_position/velocity_local                    │
│  │    └─ Pub: /rpp/velocity_ned, /rpp/debug, /rpp/yaw_rate_body       │
│  │                                                                      │
│  ├─ src/mission_runner_node.py (legacy)                                │
│  │    ├─ Sub: /mavros/state, /rpp/debug, /mavros/statustext            │
│  │    └─ Svc: /mavros/set_mode, /mavros/cmd/arming                     │
│  └─ src/launch/rpp_pipeline.launch.py (launches twist + RPP + logger)  │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
                              │
                    /dev/ttyACM0 @ 921600
                              │
                    ┌─────────▼──────────┐
                    │  CubeOrangePlus     │
                    │  (PX4 v1.16.2)      │
                    │  FMUv5 / ArduPilot? │
                    └────────────────────┘
```

---

## 1. Core Bridge Orchestration

### `px4-dxp.service`
- **Path:** `px4-dxp.service`
- **Role:** systemd unit that keeps the MAVROS bridge alive
- **Key details:**
  - Runs `px4_start_service.sh` as user `flash`
  - `After=network.target network-online.target dev-ttyACM0.device`
  - FIFO scheduling on CPU core 4 (latency reduction)
  - `ROS_DOMAIN_ID=0`
  - Loads NTRIP credentials from `config/ntrip.env`
  - `Restart=always`, `RestartSec=10`
  - `PartOf=rpp-pipeline.service` (cascading restart)

### `px4_start_service.sh`
- **Path:** `px4_start_service.sh`
- **Role:** Launches MAVROS with a watchdog (auto-restart on crash)
- **Key MAVROS launch command (line 91-100):**
  ```bash
  ros2 launch mavros node.launch \
      fcu_url:=${FCU_DEVICE}:${FCU_BAUD} \
      gcs_url:=udp://:${GCS_UDP_PORT}@ \
      pluginlists_yaml:=${SCRIPT_DIR}/px4_pluginlists_rover.yaml \
      config_yaml:=/opt/ros/humble/share/mavros/launch/px4_config.yaml \
      fcu_protocol:=v2.0 \
      tgt_system:=1 \
      tgt_component:=1 \
      log_output:=screen \
      respawn_mavros:=false &
  ```
- **FCU validation (line 222):**
  ```bash
  timeout 10 ros2 topic echo /mavros/state --once | grep -q "connected: true"
  ```

### `px4_pluginlists_rover.yaml`
- **Path:** `px4_pluginlists_rover.yaml`
- **Role:** Controls which MAVROS plugins are active
- **Notable denylist entries:** `wheel_odometry`, `odometry`, `actuator_control`, `waypoint`, `geofence`
- **Critically NOT denied:** `gps_rtk` (required for NTRIP RTK via `/mavros`)

---

## 2. Python ROS2 Nodes (MAVROS Clients)

### `server/ros_node.py` — FastAPI Bridge Node
- **Path:** `server/ros_node.py`
- **Class:** `RosBridgeNode(Node)` — node name `"fastapi_bridge"`
- **MAVROS Subscribers:**
  | Topic | Msg Type | Callback | QoS |
  |---|---|---|---|
  | `/mavros/state` | `mavros_msgs/State` | `_cb_state` | RELIABLE + TRANSIENT_LOCAL |
  | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `_cb_pose` (ENU→NED) | BEST_EFFORT |
  | `/mavros/battery` | `sensor_msgs/BatteryState` | `_cb_battery` | BEST_EFFORT |
  | `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | `_cb_global_pos` | BEST_EFFORT |
  | `/mavros/gpsstatus/gps1/raw` | `mavros_msgs/GPSRAW` | `_cb_gps_raw` | BEST_EFFORT |
- **MAVROS Publishers:** `/path` (nav_msgs/Path)
- **MAVROS Service Clients:**
  | Service | Type | Variable |
  |---|---|---|
  | `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | `_arming_cli` |
  | `/mavros/set_mode` | `mavros_msgs/srv/SetMode` | `_set_mode_cli` |
  | `/mavros/param/get_parameters` | `rcl_interfaces/srv/GetParameters` | `_param_get_cli` |
  | `/mavros/param/set_parameters` | `rcl_interfaces/srv/SetParameters` | `_param_set_cli` |
- **Crash detection (lines 185-186, 398-401):**
  ```python
  self._state_recv_time: float | None = None
  self._MAVROS_STATE_TIMEOUT_S = 2.0
  # In get_state():
  if self._state_recv_time is not None:
      age = time.monotonic() - self._state_recv_time
      if age > self._MAVROS_STATE_TIMEOUT_S:
          state["connected"] = False
  ```
- **ENU→NED pose conversion (line 325-337):**
  ```python
  yaw_ned = math.pi / 2.0 - yaw_enu
  self._state["pos_n"] = msg.pose.position.y
  self._state["pos_e"] = msg.pose.position.x
  ```
- **Async API:** `arm_async()`, `set_mode_async()`, `get_param_async()`, `set_param_async()`

### `src/twist_to_setpoint_node.py` — RPP → MAVROS Setpoint Bridge
- **Path:** `src/twist_to_setpoint_node.py`
- **Class:** `TwistToSetpointNode(Node)` — node name `"twist_to_setpoint"`
- **Role:** Converts NED velocity from RPP controller into MAVROS PositionTarget at 50 Hz
- **MAVROS Publisher:** `/mavros/setpoint_raw/local` (mavros_msgs/PositionTarget)
- **Type mask:** `455` = velocity + explicit yaw + yaw_rate (IGNORE_PX, PX, PZ, AFX, AFY, AFZ)
- **Coordinate conversion (line 229-231):**
  ```python
  msg.velocity.x = v_e       # ENU x = East (was NED y)
  msg.velocity.y = v_n       # ENU y = North (was NED x)
  msg.velocity.z = -v_d      # ENU z = Up (negate NED Down)
  ```
- **Yaw computation (line 244-245):**
  ```python
  yaw_enu = math.atan2(v_n, v_e)  # ENU: 0=East, CCW+
  ```
- **Stale-input guard:** streams (0,0,0) when input > 200 ms stale

### `src/rpp_controller_node.py` — Regulated Pure Pursuit Controller
- **Path:** `src/rpp_controller_node.py`
- **Class:** `RPPControllerNode(Node)` — node name `"rpp_controller"`
- **MAVROS Subscribers:**
  | Topic | Use |
  |---|---|
  | `/mavros/local_position/pose` | ENU→NED pose for projection |
  | `/mavros/gpsstatus/gps1/raw` | RTK fix gate (P0.3) |
  | `/mavros/local_position/velocity_local` | Velocity-based pose extrapolation (P2.4) |

### `src/mission_runner_node.py` — Legacy Standalone Mission Runner
- **Path:** `src/mission_runner_node.py`
- **Class:** `MissionRunnerNode(Node)` — node name `"mission_runner"`
- **MAVROS Subscribers:**
  | Topic | Use |
  |---|---|
  | `/mavros/state` | FCU state tracking |
  | `/mavros/statustext` | PX4 diagnostic messages |
- **MAVROS Service Clients:**
  | Service | Use |
  |---|---|
  | `/mavros/set_mode` | OFFBOARD/MANUAL switch |
  | `/mavros/cmd/arming` | Arm/Disarm |
- **Uses `call_async()` + `ReentrantCallbackGroup`** to avoid deadlock

---

## 3. Server-Level Wiring (FastAPI)

### `server/main.py`
- **Path:** `server/main.py`
- **Lifespan startup (line 101-114):** Initialises `rclpy`, creates `RosBridgeNode` + `RosExecutorThread`
- **Telemetry loop (line 234-375):** 10 Hz loop that:
  - Reads `ros_node.get_state()` (which internally polls `/mavros/state`)
  - Pushes telemetry via Socket.IO
  - Detects stale pose/FCU disconnect → triggers `emergency_handler.estop_async()`
  - Monitors RPP state for auto-completion (DONE → COMPLETED)

### `server/offboard_controller.py`
- **Path:** `server/offboard_controller.py`
- **Class:** `OffboardController`
- **MAVROS service calls through RosBridgeNode:**
  - `start_async()` → `arm_async(True)` → `set_mode_async("OFFBOARD")` → publish path
  - `stop_async()` → `publish_stop_path()` (single-point path at current position)
  - `abort_async()` → stop path → `set_mode_async("MANUAL")` → `arm_async(False)`

### `server/emergency.py`
- **Path:** `server/emergency.py`
- **Called by telemetry watchdog when:** consecutive stale pose > grace period, or FCU disconnect detected
- **Action:** Calls `offboard_ctrl.abort_async()`

---

## 4. Launch & Deployment

### `src/launch/rpp_pipeline.launch.py`
- **Path:** `src/launch/rpp_pipeline.launch.py`
- **Launches (assuming MAVROS is already running):**
  `twist_to_setpoint_node.py` → `rpp_controller_node.py` → `xtrack_logger_node.py` → `path_publisher_node.py` → (optional) `mission_runner_node.py`

### `rpp-pipeline.service`
- **Path:** `rpp-pipeline.service`
- **systemd unit** that runs the RPP pipeline
- `PartOf=px4-dxp.service` (restart cascade)

### `rover-server.service`
- **Path:** `rover-server.service`
- **systemd unit** for the FastAPI server
- Independent service (no MAVROS restart cascade)

### `deploy.sh`
- **Path:** `deploy.sh`
- Deployment script: copies service files, runs `systemctl daemon-reload`

---

## 5. Auxiliary Files

| File | Role |
|---|---|
| `ntrip_rtcm_node.py` | NTRIP RTK client — injects RTCM corrections into `/mavros` |
| `src/path_publisher_node.py` | Publishes test paths to `/path` for RPP |
| `src/xtrack_logger_node.py` | Logs cross-track error CSV |
| `src/offboard_test.py` | Direct MAVROS test script |
| `src/test_p05_yaw_setpoint.py` | Tests yaw setpoint via MAVROS |
| `src/test_smoke_rpp_controller.py` | Smoke test for RPP controller |
| `tools/capture_offboard_diag.sh` | Diagnostic capture using ROS2 CLI |
| `tools/benchmark_rpp.py` | RPP benchmark tool |
| `tools/extract_rosbag_to_csv.py` | Rosbag → CSV extraction |
| `tools/inspect_bag.py` | Rosbag inspection |

---

## 6. Architecture Documentation

| File | Topic |
|---|---|
| `docs/Architecture/MAVROS2_ONLY_DECISION.md` | Decision to use MAVROS2 exclusively |
| `docs/Architecture/FINAL_ARCHITECTURE.md` | Overall system architecture |
| `docs/Architecture/PHASE2_BUILD_PLAN.md` | Phase 2 OFFBOARD build plan |
| `docs/Researches/MAVROS_vs_DDS.md` | MAVROS vs DDS comparison |
| `docs/Researches/Pure_DDS.md` | Pure DDS exploration |
| `docs/Researches/Hybride_Archi_Decision.md` | Hybrid architecture decision record |
| `docs/Researches/OFFBOARD_RESEARCHES/OFFBOARD_ACTION_PLAN.md` | OFFBOARD mode plan |
| `docs/Researches/OFFBOARD_RESEARCHES/OFFBOARD_AUDIT_FINDINGS.md` | OFFBOARD audit findings |
| `docs/Researches/OFFBOARD_RESEARCHES/OFFBOARD_PATCHES.md` | OFFBOARD patches applied |
| `docs/Researches/OFFBOARD_RESEARCHES/DeepSeek_review.md` | External review |
| `docs/Architecture/KIRO_OPUS_OFFBOARD_AUDIT_PROMPT.md` | Offboard audit prompt |

---

## 7. Key MAVROS Topics & Services Summary

### Subscribed Topics (consumed by the bridge)

| Topic | Publisher (MAVROS) | Consumer | Rate |
|---|---|---|---|
| `/mavros/state` | MAVROS → PX4 | `ros_node.py`, `mission_runner_node.py` | ~10 Hz |
| `/mavros/local_position/pose` | MAVROS → PX4 | `ros_node.py`, `rpp_controller_node.py` | 30-50 Hz |
| `/mavros/local_position/velocity_local` | MAVROS → PX4 | `rpp_controller_node.py` | 30-50 Hz |
| `/mavros/battery` | MAVROS → PX4 | `ros_node.py` | ~1 Hz |
| `/mavros/global_position/global` | MAVROS → PX4 | `ros_node.py` | ~10 Hz |
| `/mavros/gpsstatus/gps1/raw` | MAVROS → PX4 | `ros_node.py`, `rpp_controller_node.py` | ~10 Hz |
| `/mavros/statustext` | MAVROS → PX4 | `mission_runner_node.py` | event |

### Published Topics (from the bridge to PX4 via MAVROS)

| Topic | Publisher | Consumer | Rate |
|---|---|---|---|
| `/mavros/setpoint_raw/local` | `twist_to_setpoint_node.py` | MAVROS → PX4 | 50 Hz |
| `/path` | `ros_node.py` | `rpp_controller_node.py` | on mission start |

### Service Calls

| Service | Caller | Effect |
|---|---|---|
| `/mavros/cmd/arming` | `ros_node.py`, `mission_runner_node.py` | Arm/Disarm PX4 |
| `/mavros/set_mode` | `ros_node.py`, `mission_runner_node.py` | OFFBOARD/MANUAL switch |
| `/mavros/param/get_parameters` | `ros_node.py` | Read PX4 parameters |
| `/mavros/param/set_parameters` | `ros_node.py` | Write PX4 parameters |

---

## 8. MAVROS Crash Detection Flow

```
MAVROS process dies → TRANSIENT_LOCAL keeps last /mavros/state cached
                          → _state_recv_time stops updating
                          → get_state() detects age > 2.0s
                          → overrides connected to False
                          → telemetry loop sees connected=False while RUNNING
                          → after SAFETY_STALE_GRACE_S → estop_async()
                          → abort sequence: stop_path → MANUAL → disarm
```

---

## 9. Service Restart Dependencies (from AGENTS.md)

| Changed Files | Restart Command | Drops MAVROS? |
|---|---|---|
| `src/*.py` | `sudo systemctl restart rpp-pipeline` | No (~2s) |
| `server/**` | `sudo systemctl restart rover-server` | No (~2s) |
| `px4_start_service.sh`, pluginlist, NTRIP | `sudo systemctl restart px4-dxp` | **Yes (~11s)** |
| New `*.service` files | `./deploy.sh` (daemon-reload) | — |

`rpp-pipeline PartOf=px4-dxp` — px4-dxp restart cascades down to RPP pipeline.