# GCS Migration Compatibility Matrix

Last updated: 2026-06-29

This matrix is for migrating the GCS/mobile client against the current
`rover-server` API without changing mission-control, spray, RTK, e-stop, or auth
behavior on the Jetson.

| Surface | Current backend contract | Migration guidance | Status |
|---|---|---|---|
| REST base URL | `http://192.168.1.102:5001/api` | Keep the same LAN target and token header. | Compatible |
| Auth header | `X-Rover-Token` | Operator endpoints require a login token. Bag/autorecord machine tokens remain limited to their existing scopes. | Compatible |
| Activity log JSON | `GET /api/activity` | Existing bounded in-memory records: `timestamp`, `level`, `message`. | Compatible |
| Activity log CSV | `GET /api/activity.csv` | New export of the same bounded activity records. Use for "download logs" UI. | Added |
| Telemetry snapshot | `GET /api/telemetry/latest` | Auth-protected `TelemetryData`; no field removals in this change. | Compatible |
| Socket.IO URL | `/socket.io` | Authenticate once at connect with `{token}`. Event contract is documented at `GET /api/docs/asyncapi`. | Compatible |
| Mission commands | REST `/api/mission/*`, Socket.IO mission events | No command semantics changed. OFFBOARD still goes through mission start only. | Compatible |
| Point mode controls | REST point endpoints, Socket.IO point events | Existing pause/resume/continue/skip/restart surfaces retained. | Compatible |
| Spray controls | `/api/spray/*`, `/api/spray/params/*`, path spray-mode routes | No spray safety or actuator behavior changed. | Compatible |
| RTK/NTRIP status | `/api/rtk/status` and existing RTK manager behavior | No `force_clear` or injection behavior added. | Compatible |
| ROS node monitor | `GET /api/nodes` | New read-only graph snapshot. Reports expected nodes as `present`, `missing`, or `unknown`; degrades safely if ROS graph/CLI is unavailable. | Added |
| Network telemetry | `GET /api/network` | New read-only interface/default-route/Wi-Fi snapshot. Reports unavailable command errors instead of inventing data. | Added |
| Bridge health | `GET /api/health/bridge`, Socket.IO `bridge_health` | Existing observe/recovery reporting retained. | Compatible |
| Discovery | `POST /api/discover` | Existing LAN beacon discovery retained. | Compatible |

## New Read-Only Monitoring Responses

`GET /api/nodes` returns:

- `ok`: true only when the ROS graph is available and all expected nodes are present.
- `source`: `ros_graph`, `ros2_cli`, or `unavailable`.
- `nodes`: discovered graph nodes.
- `expected_nodes`: current backend expectations for rover operation.
- `errors`: bounded diagnostics for unavailable graph/CLI paths.

`GET /api/network` returns:

- `timestamp`, `hostname`, `source`.
- `interfaces`: interface name, operstate, MAC, and parsed IP addresses when available.
- `default_routes`: parsed default routes when `ip` is available.
- `wifi`: Wi-Fi availability, link details, and command errors.
- `errors`: bounded diagnostics for unavailable platform probes.

## Deferred Or Out Of Scope

| Item | Disposition |
|---|---|
| TTS | Not implemented in this repo. |
| Ultrasonic hardware publishing | Hardware-deferred; no synthetic data added. |
| WS2812 LED control | Hardware-deferred. |
| RTK `force_clear` | Not implemented; RTK injection behavior unchanged. |
| PX4 firmware or FCU params | Out of scope for Jetson companion work. |
