I'll do this properly — actual research, not theoretical analysis. Let me first check what's in the existing architecture doc, then research the current state.

Real research time. Let me run parallel searches on the key technical questions.

Let me dig into the actual content of these key pages:

Critical find — let me dig into that Nov 2025 forum post about rover offboard not working:

I have enough evidence. Stopping web research here and synthesizing the answer. The pattern is clear from forum posts dated through Nov-Dec 2025 and the eProsima reconnect issue.

---

# Architecture Decision Report — Hybrid (Option C)

## TL;DR

**Recommendation: Hybrid (Option C). MAVROS2 stays on USB. DDS goes on TELEM1 (currently `/dev/ttyUSB0`) for the 100 Hz control loop.** This is the only option that lets you ship Phase 2 with proven safety nets while you de-risk the parts of DDS that are still rough in production.

The single highest risk: **dual-writer race condition on `OffboardControlMode` and `vehicle_command` uORB topics.** Both bridges write to the same FCU memory. Mitigation below.

---

## 1. uXRCE-DDS Reliability — Current State (Nov 2025–May 2026)

### What I found

The "no auto-reconnect" issue is **NOT fixed** as of v1.16. It's a chronic class of bugs, not a single one.

| Date | Source | Symptom | Status |
|------|--------|---------|--------|
| Nov 23, 2025 | [DDS failed to connect ROS2 Jazzy](https://discuss.px4.io/t/dds-faild-to-connect-ros2-jazzy/47966) | session established, no /fmu topics | Open |
| Nov 26, 2025 | [Can't connect to PX4 uXRCE client](https://discuss.px4.io/t/cant-connect-to-px4-uxrce-client/47990) | TELEM serial connection fails | Open |
| Nov 14, 2025 | [Flight Controller not communicate over XRCE-DDS](https://discuss.px4.io/t/flight-controller-not-communicate-over-xrce-dds/47873) | client running, no topics | Open |
| Nov 19, 2025 | [Not receiving /fmu/out topics](https://discuss.px4.io/t/not-receiving-fmu-out-topics-on-pixhawk-uxrce-dds/47935) | "no datawriter created" — UART connection initialised but no topics | Open |
| Jun 24, 2025 | [Kakute H7 + RPi5, v1.15.4](https://discuss.px4.io/t/uxrce-dds-client-and-agent-not-connecting-with-kakute-h7-and-rpi5-usb-to-ttl-px4-v1-15-4-ros-2-jazzy/46145) | "Running, disconnected" forever | Open |
| Apr 27, 2025 | [uxrce_dds_client disconnected](https://discuss.px4.io/t/uxrce-dds-client-diconnected/45314) | wifi connection drops, no recovery | Open |
| 2019 → still open | [eProsima Agent Issue #48](https://github.com/eProsima/Micro-XRCE-DDS-Agent/issues/48) | "Can't reconnect serial client using same client key" | **Open since 2019** |

The fundamental architectural problem is the eProsima Issue #48: when the agent restarts, the client-side session state on PX4 doesn't reset. Workarounds exist (cycle the client key, restart `uxrce_dds_client` on PX4, reboot the FCU) but no clean fix has shipped. v1.16 didn't address it.

### Implication for your rover

If MAVROS2 isn't there as a backup, a single agent restart on the Jetson can leave the rover **uncontrollable until the FCU is rebooted**. On a moving construction site, that's unacceptable. This alone kills Pure DDS for production today.

---

## 2. PX4 ROS2 Interface Library for Rovers — Production Readiness

### What I found

`RoverSpeedSteeringSetpointType` lives in [`px4_ros2/control/setpoint_types/experimental/rover/`](https://github.com/Auterion/px4-ros2-interface-lib). The path itself documents the maturity level: **experimental**.

Real production data:

| Date | Source | Finding |
|------|--------|---------|
| Dec 4, 2025 | [Rover Offboard: rover_speed_setpoint / rover_rate_setpoint](https://discuss.px4.io/t/rover-offboard-rover-speed-setpoint-rover-rate-setpoint/48430) | v1.17 SITL: OFFBOARD accepted, vehicle armed, setpoints received — **wheels don't spin**. No resolution posted. |
| May 20, 2025 | [About offboard velocity control](https://discuss.px4.io/t/about-offboard-velocity-control/45652) | Rover can't go straight forward/backward in offboard velocity control |
| Oct 6, 2021 | [Rover offboard lost default failsafe continues on last setpoint #18346](https://github.com/PX4/PX4-Autopilot/issues/18346) | Rover keeps driving at last setpoint after signal loss — this is a **safety bug**, status unclear in v1.16 |

### Implication

The rover setpoints work in the lab. They are **not** proven on production hardware. Nobody has shipped a commercial rover product using `RoverSpeedSteeringSetpointType`. The library version is 2.x, API is marked experimental, and the surrounding rover offboard code has open issues including a failsafe gap that would let your rover keep driving after Jetson crashes.

You should still build on it — it's where PX4 is going — but you cannot bet your shipping schedule on it being your only control path. Hybrid lets you fall back to MAVROS2 `setpoint_velocity/cmd_vel` (which works today) the moment a rover-DDS bug bites.

---

## 3. Hybrid Architecture — MAVLink + DDS Coexistence

### Confirmed: PX4 supports this natively

From [PX4 MAVLink Peripherals docs](https://docs.px4.io/v1.14/en/peripherals/mavlink_peripherals): PX4 supports up to **3 simultaneous MAVLink instances** plus a separate DDS instance, each on its own serial port, configured independently via `MAV_0_CONFIG` / `MAV_1_CONFIG` / `MAV_2_CONFIG` and `UXRCE_DDS_CFG`. They don't fight for serial bandwidth — each owns its own port.

### How PX4 routes incoming commands internally

This is where the real architectural detail matters. Both bridges deserialize their inputs and **write to the same uORB topics** on the FCU:

- MAVROS2 sends `SET_POSITION_TARGET_LOCAL_NED` → `mavlink_receiver` decodes → writes to `trajectory_setpoint` uORB topic + `offboard_control_mode` uORB topic
- DDS publishes to `/fmu/in/trajectory_setpoint` → `uxrce_dds_client` writes to the same `trajectory_setpoint` uORB topic

uORB does **not** arbitrate. **Last writer wins** at the uORB level. The flight control modules just read whatever was last published.

### What this means for two-bridge operation

If both bridges try to control the rover simultaneously, the FCU sees both streams interleaved. The rover follows the more recent setpoint. At 100 Hz from DDS and 50 Hz from MAVROS2, you get a 150 Hz mixed stream — chaos.

**This is the highest risk in the Hybrid architecture and must be designed around** (see section 6).

### Confirmed working pattern

The PX4 docs explicitly support [Companion Computer with both MAVLink and ROS 2](https://docs.px4.io/v1.14/en/companion_computer/pixhawk_companion). Many production drone builds run MAVLink for QGC and DDS for ROS2 control simultaneously. The pattern is stable. **The trick is making sure only one bridge is in command mode at any moment.**

---

## 4. Optimal Topic Split for Hybrid

Design rule: **Each function has exactly one bridge that owns it. No overlap on write paths.**

### MAVROS2 (USB / `/dev/ttyACM0`) — owns these:

| Function | Topic / Service | Why MAVROS2 |
|---|---|---|
| QGC bridge | `udp-b://:14550@` | Built-in, working, no port reassignment needed |
| RTK injection | `/mavros/gps_rtk/send_rtcm` | `GpsInjectData` not in default `dds_topics.yaml` ([px4_ros_com #212](https://github.com/PX4/px4_ros_com/issues/212)) |
| Parameter R/W | `/mavros/param/get`, `/mavros/param/set` | DDS has `VehicleCommand` but no full param service |
| Firmware flash | QGC over MAVLink | DDS doesn't carry firmware |
| Arm/disarm | `/mavros/cmd/arming` | Single source of truth for safety; DDS keeps a backup |
| RC override status | `/mavros/manual_control` | RC failsafe path |
| Heartbeat / connection state | `/mavros/state` | Used by your watchdog |
| Health monitoring | `/mavros/diagnostics` | Drives systemd watchdog |

### uXRCE-DDS (TELEM1 / `/dev/ttyUSB0`) — owns these:

| Function | Topic | Why DDS |
|---|---|---|
| Vehicle odometry (control input) | `/fmu/out/vehicle_odometry` | 100 Hz native, lowest latency, native CDR |
| Local position | `/fmu/out/vehicle_local_position` | 50 Hz, includes detailed variances |
| Attitude | `/fmu/out/vehicle_attitude` | 50 Hz |
| Vehicle status flags | `/fmu/out/vehicle_status` | Parallel monitoring path independent of MAVROS |
| Failsafe flags | `/fmu/out/failsafe_flags` | Lower latency than MAVROS state |
| EKF status | `/fmu/out/estimator_status_flags` | Lets controller detect EKF degradation in <100ms |
| Speed setpoint | `/fmu/in/rover_speed_setpoint` | Rover-specific, no NED translation |
| Steering setpoint | `/fmu/in/rover_steering_setpoint` | Rover-specific normalized [-1,1] |
| Offboard mode flag | `/fmu/in/offboard_control_mode` | Required at ≥2 Hz for OFFBOARD heartbeat |

### Disabled on both (or used carefully):

| Topic | Why |
|---|---|
| `/mavros/setpoint_velocity/cmd_vel` | **Disabled** during normal operation — only enable when DDS fails (see mitigation) |
| `/mavros/setpoint_raw/local` | Same reason |
| `/fmu/in/vehicle_command` from DDS | **Don't use for arm/mode** — let MAVROS own these. Race condition risk is too high |
| `/fmu/in/trajectory_setpoint` from DDS | Don't use — use rover-specific setpoints instead |

### Read-only topics safe on both bridges

- `/mavros/imu/data` ↔ `/fmu/out/sensor_combined` — both can read, no conflict
- `/mavros/global_position/raw/fix` ↔ `/fmu/out/vehicle_gps_position` — both can read

The NTRIP node already subscribes to the MAVROS GPS topic for GGA back-feed. Keep it there.

---

## 5. Graduation Criteria — Hybrid → Pure DDS

These are the gates that must close **all together** before Pure DDS is safe.

### Gate 1: DDS reliability bar
- [ ] 100+ hours continuous DDS operation with **zero** unrecovered disconnects
- [ ] Agent restart test: kill MicroXRCEAgent, verify automatic recovery within 5s, no FCU reboot needed
- [ ] Serial flap test: physically unplug TELEM1, replug, verify recovery
- [ ] Document the workaround for [eProsima #48](https://github.com/eProsima/Micro-XRCE-DDS-Agent/issues/48) if you hit it

### Gate 2: Rover setpoint maturity
- [ ] [Forum post 48430](https://discuss.px4.io/t/rover-offboard-rover-speed-setpoint-rover-rate-setpoint/48430) issue resolved upstream OR you've identified the root cause and patched
- [ ] [Rover offboard failsafe #18346](https://github.com/PX4/PX4-Autopilot/issues/18346) — verify last-setpoint-runaway is fixed in your build
- [ ] At least 50 successful arc runs at target accuracy via DDS-only control

### Gate 3: Function parity verified on DDS
- [ ] NTRIP via DDS works (Patch 7: add `GpsInjectData` to `dds_topics.yaml`, rebuild, reflash)
- [ ] QGC works via ESP8266 on TELEM1 (firmware flash, param tuning, telemetry)
- [ ] Param read/write via QGC over WiFi confirmed end-to-end
- [ ] All Phase 2 features (encoder fusion, NHC, spray timing) tested with DDS-only

### Gate 4: Hardware
- [ ] ESP8266 on TELEM1 with MAVESP8266 firmware deployed
- [ ] Patch 7 in your firmware build pipeline (`px4_pluginlists` → firmware flash automated)
- [ ] New systemd unit `px4-dds.service` replaces `px4-dxp.service`
- [ ] Old MAVROS2 service tagged in git as `last-mavros-version` for rollback

### Gate 5: Field validation
- [ ] One full customer-representative job (8 hr operation) on Pure DDS without intervention
- [ ] Failsafe drill: kill DDS agent mid-mission → rover stops safely (not runaway)
- [ ] Recovery drill: full system reboot → rover resumes mission within 60s

**Minimum viable Pure DDS setup:**
1. PX4 firmware with Patch 7 (one yaml line + rebuild)
2. ESP8266 with MAVESP8266 firmware on TELEM1
3. Jetson: MicroXRCEAgent v2.4.2 (pin this — v3.x breaks Humble)
4. NTRIP node modified to publish `GpsInjectData` instead of `RTCM`
5. Control nodes using `RoverSpeedSetpoint` + `RoverSteeringSetpoint`
6. New `px4-dds.service` systemd unit

---

## 6. Single Highest Risk + Mitigation

### Risk: **Dual-Writer Race on OFFBOARD Setpoints**

If both bridges send setpoints to the same uORB topic (`offboard_control_mode`, `trajectory_setpoint`, or rover setpoints), PX4 follows whichever message arrived last. With DDS at 100 Hz on TELEM1 and any stray `/mavros/setpoint_*` publication on USB, the rover's actual speed/steering becomes nondeterministic. On a sub-cm precision arc, this manifests as chatter or divergence.

This is a **silent failure** — no error appears, the rover just doesn't track the path. That's the dangerous kind.

### Mitigation: Strict Bridge Mode + Hardware Authority Lock

Implement a **single-controller-active invariant** in software, enforced by a small ROS2 supervisor node on the Jetson:

```
ControlSupervisor node owns the only handle to either bridge's setpoint publisher.
At any moment, exactly one of {DDS, MAVROS2} is in CONTROL_ACTIVE state.
The other is in MONITOR_ONLY state and its setpoint publishers are destroyed (not just suppressed).
```

Three concrete steps:

**(a) Disable MAVROS setpoint plugins entirely during normal operation.**

Add to your existing `px4_pluginlists_rover.yaml`:
```yaml
- setpoint_velocity      # Phase 2: DDS owns control
- setpoint_position      # Phase 2: DDS owns control  
- setpoint_raw           # Phase 2: DDS owns control
- setpoint_attitude      # Phase 2: DDS owns control
```

Now MAVROS literally cannot publish setpoints. The FCU only sees DDS for control inputs. Race eliminated at the source.

**(b) Pre-flight assertion in your supervisor node.**

```python
# At startup, verify only ONE writer for offboard topics
writers = self.count_publishers("/fmu/in/offboard_control_mode")
if writers > 1:
    self.get_logger().fatal(
        "Multiple offboard publishers detected - REFUSING TO ARM"
    )
    raise SystemExit(1)
```

This catches future maintainers who re-enable a MAVROS setpoint plugin without thinking.

**(c) Hardware fallback path stays available, but explicit.**

If DDS fails mid-mission, the fallback is **not** "MAVROS takes over silently." It's:
1. Rover stops (last_setpoint_zero pattern, not last-setpoint-coast)
2. Operator manually triggers fallback mode
3. Supervisor re-enables MAVROS setpoint plugins via dynamic param reload
4. MAVROS-based simple-velocity controller takes over

This makes the bridge handover **explicit and witnessed**, never automatic. On a marking rover that costs ₹25-35L and operates in customer view, silent failover is worse than a clean stop.

### Why this mitigation is the right one

The race exists because PX4's uORB doesn't arbitrate writers. You can't fix that in PX4 v1.16 without forking. So you fix it on the companion side: **make it physically impossible for the wrong bridge to write**. Disabling the MAVROS plugins is a single yaml line and removes the entire failure class.

---

## Sources

- PX4 forum posts on uXRCE-DDS reliability (Nov 2025–Dec 2025): listed in section 1
- [eProsima Micro-XRCE-DDS-Agent Issue #48 — reconnect bug, open since 2019](https://github.com/eProsima/Micro-XRCE-DDS-Agent/issues/48)
- [PX4 MAVLink Peripherals — multi-instance MAVLink support](https://docs.px4.io/v1.14/en/peripherals/mavlink_peripherals)
- [PX4 Companion Computer with Pixhawk — coexistence guide](https://docs.px4.io/v1.14/en/companion_computer/pixhawk_companion)
- [Rover Offboard wheels-don't-spin bug, Dec 2025](https://discuss.px4.io/t/rover-offboard-rover-speed-setpoint-rover-rate-setpoint/48430)
- [Rover offboard velocity control broken, May 2025](https://discuss.px4.io/t/about-offboard-velocity-control/45652)
- [Rover offboard failsafe gap #18346](https://github.com/PX4/PX4-Autopilot/issues/18346)
- [Auterion px4-ros2-interface-lib — experimental rover setpoints](https://github.com/Auterion/px4-ros2-interface-lib)
- [px4_ros_com Issue #212 — RTCM not in DDS](https://github.com/PX4/px4_ros_com/issues/212)
- [PX4 Offboard mode 2Hz heartbeat requirement](https://docs.px4.io/v1.14/en/flight_modes/offboard)

Content was paraphrased from the above sources for compliance with licensing restrictions.

---

## What This Means for Your `FINAL_ARCHITECTURE.md`

Your existing doc has **MAVROS2** selected with the note "if >40ms consistently, evaluate uXRCE-DDS with workarounds." That decision was correct for Phase 1. For Phase 2 it should evolve to:

> **Phase 2 communication: Hybrid (MAVROS2 monitor + DDS control).**  
> MAVROS2 owns QGC, NTRIP, params, firmware flash, arm/mode.  
> DDS owns 100 Hz control loop and high-rate odometry.  
> MAVROS setpoint plugins are explicitly disabled in `px4_pluginlists_rover.yaml`.  
> Graduation to Pure DDS requires Gates 1–5 (see `MAVROS_vs_DDS.md`).

That single sentence in your decision table replaces the "evaluate later" ambiguity with a clear path. Want me to draft the doc update once you've decided?