# Socket.IO AsyncAPI

Last updated: 2026-06-29

The canonical machine-readable document is served by the rover backend:

```bash
curl -H "X-Rover-Token: $ROVER_TOKEN" \
  http://192.168.1.102:5001/api/docs/asyncapi
```

The document is generated from `server/socket_contract.py` and uses AsyncAPI
2.6.0 channels named `socket.io/<event>`.

## Client Emits

| Event | Purpose |
|---|---|
| `connect` | Authenticate Socket.IO SID with an operator token. |
| `disconnect` | Socket lifecycle disconnect; releases joystick ownership. |
| `arm` | Arm/disarm request. Disarm uses spray-safe shutdown. |
| `set_mode` | Set a non-OFFBOARD mode. |
| `emergency_stop` | Run the established e-stop path. |
| `joystick_acquire` | Acquire virtual joystick control. |
| `joystick_command` | Send virtual joystick command under a lease. |
| `joystick_release` | Release virtual joystick control. |
| `mission_load` | Load a mission path. |
| `mission_start` | Start resident or named mission. |
| `mission_stop` | Stop mission without hard abort. |
| `mission_abort` | Abort mission with terminal cleanup. |
| `mission_pause` | Pause point mission into hold. |
| `mission_resume` | Resume paused point mission. |
| `point_continue` | Continue manual point mission. |
| `mission_obstacle` | Set obstacle-clear hook. |
| `point_skip` | Skip requested point leg. |
| `mission_restart` | Reset resident mission and optionally restart. |
| `request_params` | Read selected PX4 parameters. |

## Server Emits

| Event | Purpose |
|---|---|
| `arm_result` | Arm/disarm result. |
| `auth_revoked` | Session revoked by password change. |
| `bridge_health` | MAVROS bridge health transition or alert. |
| `bridge_recovery` | Bridge recovery attempt metadata. |
| `estop_result` | E-stop result. |
| `gps_safety_abort` | GPS_SURVEYED runtime safety abort. |
| `joystick_acquired` | Joystick lease acquired. |
| `joystick_error` | Joystick rejection/runtime error. |
| `joystick_released` | Joystick lease released. |
| `mission_abort_result` | Mission abort result. |
| `mission_completed` | Mission completed cleanly. |
| `mission_completion_degraded` | Completion cleanup degraded. |
| `mission_error` | Mission command/service error. |
| `mission_loaded` | Mission load result. |
| `mission_obstacle_result` | Obstacle-clear update result. |
| `mission_pause_result` | Point pause result. |
| `mission_restart_result` | Mission restart result. |
| `mission_resume_result` | Point resume result. |
| `mission_status` | Periodic mission status snapshot. |
| `mission_status_update` | Command-triggered mission status update. |
| `mission_stop_result` | Mission stop result. |
| `mode_result` | Mode request result. |
| `params_result` | Parameter read result. |
| `point_continue_result` | Manual point continue result. |
| `point_mission_event` | Point event journal append. |
| `point_skip_result` | Point skip result. |
| `rover_disconnected` | FCU/MAVROS disconnected transition. |
| `safety_abort` | Telemetry watchdog safety abort. |
| `socket_error` | Unauthenticated socket command rejection. |
| `telemetry` | Periodic telemetry snapshot. |
