Features on **`main` that are not on `test/colinear-fix`** — rebuild list only (no logic):

### Mission / placement
1. GPS-surveyed mission placement into the live EKF frame  
2. Runtime entry transit (spray-OFF lead-in to wp0 for GPS_SURVEYED starts)  
3. Runtime-entry path densified at 5 cm  
4. Align / pre-align before marking and before first-run drive  
5. Affine / ref-point scale gate (stop double-scaling survey refs)  
6. Mission clear API (`POST /api/mission/clear`, including from ABORTED/ERROR)  
7. Mission-mode ops: skip, restart, operation coordinator, terminal cleanup, event journal  

### Point navigation
8. Point-navigation stack (Task_01)  
9. GPS lat/lon point-mission CSV ingest + GPS_SURVEYED staging  
10. Point-mission plan-and-stage end-to-end unblocking  

### Path / geometry
11. Uniform ≤5 cm densification everywhere (PRE/MARK/AFT/connectors + global enforce)  

### Spray
12. Three spray modes: continuous, dash, point (mission-bound config)  
13. Per-path spray-mode API + sidecar store + hot-apply to live controller  
14. Production spray architecture: path identity binding + strict gating  
15. Actuator state machine + terminal safety (Task 17/18/19)  
16. GPS_SURVEYED runtime safety gate for continuous/dash  
17. Controller-owned spray latency  
18. Startup OFF-reconciliation / recovery retries  
19. Continuous-mode crash / param-contract hardening (declare params, no 409 on degraded load, structured timeouts)  

### Joystick / manual
20. Production virtual joystick (V2)  
21. MANUAL_CONTROL float32 path + rejection logging  
22. Throttle/steering tuning caps  
23. Corner-stop / reverse-brake yaw hold in setpoint bridge (joystick+corner harden)  

### Auth / API / telemetry
24. Local password authentication  
25. Auth disable bypass for dev (`ROVER_AUTH_DISABLED`) + Socket.IO honor it  
26. Swagger `X-Rover-Token` security scheme  
27. Read-only rover monitoring telemetry APIs  
28. Activity log CSV export  
29. Socket.IO AsyncAPI contract docs  
30. GCS migration compatibility matrix  
31. Telemetry: `measured_speed_m_s`  
32. Telemetry: GPS lat/lon to 8 decimal places  

### RTK / NTRIP
33. LoRa/NTRIP RTCM injection reliability + self-healing (Task_03)  
34. NTRIP watchdog / reconnect-age fixes  
35. 3D GPS label / plaintext-auth rejection  

### Capture / stability / debug
36. Bag capture race fix (e-stop→retry 503) + GPS nuisance e-stop debounce  
37. Complete mission capture evidence bundle  
38. Fail-closed completion / geometry / RPP-freshness / spray-backend stability pack  
39. Final DONE gated on physical coast-down (not position alone)  
40. Corner-stop param retunes on main (slowdown/brake/hold — **do not copy blindly**; baseline knobs stay frozen)

### Intentionally out of “rebuild as-is”
- Docs-only / rename / backlog refresh commits  
- PWM ON-range bump then revert  
- Any corner-stop **param regression** vs baseline (`0.50` / `0.08` / no hold)

That’s the scratch backlog from baseline → main.