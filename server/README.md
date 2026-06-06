# Drawing Rover Backend Server

The FastAPI backend server for the DYX Autonomous 3WD Marking Rover. It bridges the web interface (React/Vue GCS app) to the ROS2 Humble / MAVROS node ecosystem running locally on the companion computer (Jetson Orin).

---

## 🏗 Architecture & Threading Model

The server handles asynchronous REST calls, Socket.IO connections, and background ROS2 pub/sub execution concurrently using a hybrid asynchronous event loop and multithreaded design:

```
┌───────────────────────────────── FastAPI / Uvicorn (Main Async Loop) ─────────────────────────────────┐
│                                                                                                       │
│   Web Frontend  ◄─── Socket.IO Telemetry / REST Cmds ───►  FastAPI REST Routers (routes/)              │
│                                                                 │                                     │
│                                                                 ▼                                     │
│                                                     OffboardController / Emergency                    │
│                                                                 │ (asyncio.Future wrapper)            │
│                                                                 ▼                                     │
└─────────────────────────────────────────────────────────────────┼─────────────────────────────────────┘
                                                                  │ Thread-Safe IPC
┌─────────────────────────────────────── ROS2 Executor Thread ────┼─────────────────────────────────────┐
│                                                                 ▼                                     │
│   rclpy.spin()  ◄──── MultiThreadedExecutor (4 Threads)  ◄── RosBridgeNode (Reentrant callbacks)      │
│                                                                 │                                     │
│                                                                 ▼                                     │
│                                                    MAVROS & RPP Controller Services                   │
└───────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

1. **FastAPI Loop (Uvicorn)**: Manages incoming REST endpoints and Socket.IO connections. 
2. **Background ROS2 Thread**: Spins a single `RosBridgeNode` inside a `RosExecutorThread` utilizing a ROS2 `MultiThreadedExecutor` (with 4 threads).
3. **Non-Blocking Async Bridging**: All service clients on `RosBridgeNode` use a `ReentrantCallbackGroup`. Public ROS actions use `call_async` and return an `asyncio.Future` resolved via `add_done_callback` inside the ROS executor thread, ensuring the FastAPI event loop is never blocked by synchronous wait calls.
4. **Thread-Safe Shared State**: A thread-safe Python dictionary is guarded by `threading.Lock` inside `RosBridgeNode` to record the latest telemetry updates, which the FastAPI loops read from instantly.

---

## 📁 File Registry

Here is the functional map of the server files:

### Application Foundation

| File | Purpose & Details |
| :--- | :--- |
| [`main.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/main.py) | **App Entry Point & Lifespan Handler**:<br>• Configures logging, mounts REST routes, and binds the ASGI Socket.IO server.<br>• Orchestrates startup (rclpy initialization, daemon executor start, systemd notifications) and clean shutdown.<br>• Hosts the high-frequency (`10Hz`) telemetry broadcast loop, pose watchdogs, and completion monitors. |
| [`config.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/config.py) | **Central Configuration Constants**:<br>• Declares topics (`/path`, `/rpp/debug`, `/mavros/state`, etc.) and service identifiers.<br>• Houses watchdog limits, safety state filters, LAN UDP port assignments, upload thresholds, and CORS settings. |
| [`models.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/models.py) | **Pydantic Data Schemas**:<br>• Houses all request and response structures for REST endpoints, WebSockets, and path-planning payloads. |
| [`auth.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/auth.py) | **Token-Based Authentication Security**:<br>• Generates and reads a secure client token stored at `~/.rover_token` (mode `0600`).<br>• Enforces `X-Rover-Token` verification header checks on REST routes and connection-handshake validation on Socket.IO events. Can be bypassed using `ROVER_DISABLE_AUTH=1`. |
| [`logging_setup.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/logging_setup.py) | Sets up standard unified formatting and console outputs for loggers. |
| [`run.sh`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/run.sh) | Launcher script that sources the ROS2 Humble install folder and boots uvicorn on port `5001`. |

### ROS2 & Mission Controllers

| File | Purpose & Details |
| :--- | :--- |
| [`ros_node.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/ros_node.py) | **MAVROS & RPP ROS2 Bridge**:<br>• Manages subscriptions (`/mavros/*`, `/rpp/debug`, `/rpp/velocity_ned`) and publishers.<br>• Implements ENU-to-NED conversions: `pos_n = pose.y`, `pos_e = pose.x`, `yaw_NED = pi/2 - yaw_ENU`.<br>• Detects MAVROS crashes despite `TRANSIENT_LOCAL` settings using a state-reception freshness timestamp watchdog.<br>• Exposes asynchronous ROS2 service wrapper pipelines (`arm_async`, `set_mode_async`, parameters). |
| [`offboard_controller.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/offboard_controller.py) | **OFFBOARD Mission State Machine**:<br>• Manages transitions: `IDLE` → `ARMING` → `SWITCHING_OFFBOARD` → `RUNNING` → `STOPPING` → `COMPLETED` / `ABORTED`.<br>• Validates pre-conditions before arming: rejects startup if RPP states are unhealthy (`STALE`, `RTK_WAIT`, `JUMP_SKIP`).<br>• Enforces setpoint streaming grace periods (0.5s) prior to switching PX4 modes to satisfy OFFBOARD acceptance gates. |
| [`emergency.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/emergency.py) | **Emergency Stop (E-Stop) Coordinator**:<br>• Immediately overrides linear velocity commands by publishing a current-coordinate single-point path.<br>• Calls services to force mode to `MANUAL` and disarms the vehicle motors. |
| [`path_manager.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/path_manager.py) | **Path Manager & Generator**:<br>• Stores built-in shapes (e.g. `square_2x2`, `straight_5m`) and maps uploads (QGC `.waypoints`, simple `.csv`).<br>• Bridges CAD processing requests to the Python `path_engine` planning library. |
| [`rpp_status.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/rpp_status.py) | Decodes the raw 39-field array published on `/rpp/debug`. Handles settling windows (needs to verify RPP is in `DONE` status for 1 second continuously) before marking a mission as completed. |
| [`beacon.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/beacon.py) | Broadcasts UDP packet heartbeats on port `5002` containing host IP and ID for LAN auto-discovery, and listens for other backend instances nearby. |

### API Endpoints (`routes/` & `sockets/`)

* **`routes/` (REST Routers)**:
  * [`system.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/routes/system.py): Cheap health diagnostics (`/ping`, `/healthz`), LAN active rover discovery, and system activity logs.
  * [`vehicle.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/routes/vehicle.py): Direct manual vehicle commands: `/arm`, `/set_mode`, and `/estop`.
  * [`mission.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/routes/mission.py): Lifecycle commands to load, start, stop, abort, and fetch mission status.
  * [`path.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/routes/path.py): Uploads, deletes, and publishes path geometries. Coordinates the `/parse-dxf` and `/plan` endpoints connecting requests to `path_engine`.
  * [`params.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/routes/params.py): Gets and sets parameters on the PX4 Flight Controller Unit (FCU).
  * [`rpp_params.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/routes/rpp_params.py): Manages parameters for the RPP controller node. Houses the schema describing parameter metadata, limits, and type safety constraints.
  * [`telemetry.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/routes/telemetry.py): Exposes `/latest` telemetry snapshot for poll-based dashboard updates.
* **`sockets/` (WebSocket Sockets)**:
  * [`events.py`](file:///Users/dyx_a1/Vetri/PX4_DXP/server/sockets/events.py): Listens for client Socket.IO connections and dispatches vehicle controls, parameters, and mission transitions.

---

## ⚡ Crucial Implementation Details

### E-Stop Safety Behavior
The upstream RPP node ignores empty path arrays. To stop the vehicle instantly while keeping it armed or during abort procedures, the server:
1. Translates the current rover ENU coordinates into relative NED space.
2. Publishes a single-point path consisting of just this coordinate.
3. RPP registers that the target has been reached within `xy_goal_tolerance`, forcing commanded velocities to zero.
4. If a pose was never received from MAVROS, the server falls back to publishing an empty path and logs a warning while issuing manual mode commands.

### Pre-Stream OFFBOARD Switches
PX4 rejects switches to OFFBOARD mode unless a constant setpoint command stream is active. 
1. The server sleeps for `0.5 seconds` to let the downstream pipeline populate setpoints.
2. The server verifies RPP status is healthy (rejecting switches during `STALE`, `RTK_WAIT`, or `JUMP_SKIP`).
3. Sends the mode change service command to `/mavros/set_mode`.

---

## 🚀 Execution & Logging

Run the server locally using the shell wrapper:

```bash
# Execute launcher
./run.sh
```

To run the backend with token authentication bypassed for local testing:

```bash
ROVER_DISABLE_AUTH=1 ./run.sh
```
