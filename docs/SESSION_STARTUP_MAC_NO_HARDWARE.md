# Session Startup — Mac Dev Flow (No Jetson / No Hardware)

This project (3WD Marking Rover) normally runs on a Jetson Orin with ROS2 + MAVROS + a CubeOrange FCU. On a Mac with none of that hardware, you can still test (1) the entire path_engine library and (2) the FastAPI server + its REST API. ROS2/rclpy does NOT install on macOS Apple Silicon, but the server is designed to boot without it (rclpy.init() is wrapped in try/except in server/main.py), so ROS-dependent endpoints degrade to HTTP 503 instead of crashing.

## One-time Setup

- **Project root:** `/Users/dyx_a1/Vetri/PX4_DXP`
- **Create an isolated venv** (system Python 3.9 is fine):
  ```bash
  python3 -m venv .venv-dev
  ```
- **Install deps + test/runtime shims:**
  ```bash
  .venv-dev/bin/python -m pip install --upgrade pip
  .venv-dev/bin/python -m pip install pytest -r server/requirements.txt
  .venv-dev/bin/python -m pip install eval_type_backport
  ```

**CRITICAL NOTE:** `eval_type_backport` is **REQUIRED**. FastAPI route signatures use PEP-604 `str | None` union syntax, which only evaluates at runtime on Python 3.10+. Without this shim the server crashes on import on Python 3.9.

## Run the Path Engine Tests (No Hardware, No Server)

```bash
cd /Users/dyx_a1/Vetri/PX4_DXP
.venv-dev/bin/python -m pytest path_engine/tests/ -v
```

**Expected:** 111 passed in ~0.14s. The ezdxf PyparsingDeprecationWarning lines are harmless.

## Start the FastAPI Server (Headless, No ROS2)

```bash
cd /Users/dyx_a1/Vetri/PX4_DXP/server
PYTHONPATH=/Users/dyx_a1/Vetri/PX4_DXP ROVER_DISABLE_AUTH=1 ROS_DOMAIN_ID=0 /Users/dyx_a1/Vetri/PX4_DXP/.venv-dev/bin/python -m uvicorn main:app --host 127.0.0.1 --port 5001 --log-level info
```

### Critical Environment Requirements

1. **`PYTHONPATH`** MUST include the project root, or `/api/path/plan` and DXF parsing return HTTP 500 with `No module named 'path_engine'`.
2. **`ROVER_DISABLE_AUTH=1`** bypasses token auth for local dev (safe on localhost).

### Expected Startup Log Lines

- `"auth: DISABLED via ROVER_DISABLE_AUTH=1"`
- `"ROS2 unavailable — server running without MAVROS: No module named 'rclpy'"` (this is EXPECTED and fine)
- `"Application startup complete"`
- `"Uvicorn running on http://127.0.0.1:5001"`

### Stop the Server

```bash
pkill -f "uvicorn main:app"
```

## Quick API Smoke Test

| Endpoint | Method | Expected | Meaning |
|---|---|---|---|
| `/api/ping` | GET | 200 | liveness |
| `/api/healthz` | GET | 200, `ros_node:false` | readiness, shows degraded state |
| `/api/paths` | GET | 200 | lists builtin + uploaded paths |
| `/api/path/plan` | POST | 200 | full path-engine pipeline via API |
| `/api/mission/load` | POST | 200 | loads waypoints (no ROS needed) |
| `/api/arm`, `/api/set_mode` | POST | 503 "ROS node not ready" | clean degrade, no hardware |
| `/api/params/{name}` | GET | 503 | clean degrade, no hardware |

### Concrete Example

```bash
curl -s -X POST -H 'Content-Type: application/json' -d '{"source":"square_2x2","line_spacing":0.1}' http://127.0.0.1:5001/api/path/plan
```

**Note:** Builtin path names are `square_2x2`, `straight_5m`, `lshape_2x2` — use the bare name, NO `builtin:` prefix.

## What This CAN and CANNOT Test

### CAN Test
- path_engine (100%)
- All REST API request/response shapes
- Server startup
- Auth-disabled flow
- Graceful ROS2 degradation

### CANNOT Test (needs Docker + PX4 SITL or real Jetson)
- Actual ROS2 nodes
- MAVROS
- Arm/OFFBOARD/telemetry data flow
- RTK GPS
- Motor/xtrack tuning

For full ROS2 + time-sync integration: use Docker with the `ros:humble` arm64 image + PX4 SITL.

## Bug-Fix Status (2026-06-05)

All review findings from the 2026-06-05 session are **FIXED and verified** (server + path_engine):

- **FIXED (was HIGH):** `mission/start`/`stop`/`abort` and `estop` no longer crash with 500 when `ros_node` is None — they now degrade cleanly (start → 409, stop/abort/estop → 200 with a clear message). Guards added in `offboard_controller.py`, `routes/mission.py`, `emergency.py`.
- **FIXED:** swallowed service-call exceptions in `ros_node.py` (`get_param_async`, `_call_set_param`); emergency state write now under the controller lock.
- **FIXED:** `ROVER_DISABLE_AUTH=1` now binds `127.0.0.1` (run.sh) and restricts CORS to localhost (config.py); DXF parse temp file no longer leaks into `missions/`; ARC length 360°-wrap bug + `math.pi`; `dxf_parser` unit_scale≤0 guard; `ned.py` extra-refpoint warning; `test_engine.py` `_HAS_EZDXF` consistency.

If any mission endpoint returns **500** again, the `ros_node=None` guards regressed — re-check the three files above.

---

## TL;DR for Next Claude

Three commands to get running in under a minute:

```bash
# 1. One-time venv + deps setup
python3 -m venv /Users/dyx_a1/Vetri/PX4_DXP/.venv-dev
/Users/dyx_a1/Vetri/PX4_DXP/.venv-dev/bin/python -m pip install --upgrade pip
/Users/dyx_a1/Vetri/PX4_DXP/.venv-dev/bin/python -m pip install pytest -r /Users/dyx_a1/Vetri/PX4_DXP/server/requirements.txt eval_type_backport

# 2. Run path_engine tests
cd /Users/dyx_a1/Vetri/PX4_DXP && .venv-dev/bin/python -m pytest path_engine/tests/ -v

# 3. Start the FastAPI server
cd /Users/dyx_a1/Vetri/PX4_DXP/server && PYTHONPATH=/Users/dyx_a1/Vetri/PX4_DXP ROVER_DISABLE_AUTH=1 ROS_DOMAIN_ID=0 /Users/dyx_a1/Vetri/PX4_DXP/.venv-dev/bin/python -m uvicorn main:app --host 127.0.0.1 --port 5001 --log-level info
```

Then test with: `curl http://127.0.0.1:5001/api/ping`
