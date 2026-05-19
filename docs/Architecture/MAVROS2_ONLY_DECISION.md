# MAVROS2-Only Decision — Why DDS Is Shelved

**Date:** 2026-05-20
**Status:** Active decision — MAVROS2 only for Phase 2. DDS is a future upgrade, not a current path.

---

## Decision

**Use MAVROS2 exclusively for Phase 2 OFFBOARD control.** No uXRCE-DDS. No hybrid bridge. One bridge, zero race conditions, proven path.

DDS is only reconsidered if:
1. MAVROS2 fails to achieve target accuracy (±3cm on arcs) due to latency/rate limits, OR
2. Phase 2 is fully complete and stable — DDS becomes the next-stage upgrade

---

## Why DDS Is Shelved (Research Findings)

### 1. DDS has chronic reliability bugs with no fix

| Bug | Source | Status | Impact |
|---|---|---|---|
| No auto-reconnect after agent restart | [eProsima #48](https://github.com/eProsima/Micro-XRCE-DDS-Agent/issues/48) | **Open since 2019** | Jetson crash → rover uncontrollable until FCU reboot |
| Session established, no /fmu topics | [PX4 Forum 47966](https://discuss.px4.io/t/dds-faild-to-connect-ros2-jazzy/47966) | Open (Nov 2025) | DDS appears connected but delivers zero data |
| Client connected but "disconnected" forever | [PX4 Forum 46145](https://discuss.px4.io/t/uxrce-dds-client-and-agent-not-connecting-with-kakute-h7-and-rpi5-usb-to-ttl-px4-v1-15-4-ros-2-jazzy/46145) | Open (Jun 2025) | Hardware combo simply doesn't work |
| WiFi connection drops, no recovery | [PX4 Forum 45314](https://discuss.px4.io/t/uxrce-dds-client-diconnected/45314) | Open (Apr 2025) | Field deployment risk |

These aren't edge cases. They're a pattern: **DDS on PX4 does not recover gracefully from disconnections.** On a moving rover at a construction site, that's unacceptable.

### 2. Rover-specific DDS control is broken/unproven

| Bug | Source | Impact |
|---|---|---|
| OFFBOARD accepted, armed, setpoints received — **wheels don't spin** | [PX4 Forum 48430](https://discuss.px4.io/t/rover-offboard-rover-speed-setpoint-rover-rate-setpoint/48430) (Dec 2025) | DDS rover control simply doesn't work in current firmware |
| Can't go straight forward/backward in offboard velocity | [PX4 Forum 45652](https://discuss.px4.io/t/about-offboard-velocity-control/45652) (May 2025) | Basic velocity control broken |
| Rover keeps driving at last setpoint after signal loss | [PX4 #18346](https://github.com/PX4/PX4-Autopilot/issues/18346) | **Safety bug** — rover runaway on DDS disconnect |
| RoverSpeedSteeringSetpointType is in `experimental/` | [px4-ros2-interface-lib](https://github.com/Auterion/px4-ros2-interface-lib) | API not stable, not production-qualified |

**Nobody has shipped a commercial rover using DDS offboard control.** The rover setpoint types are marked experimental. The most basic operation (make wheels turn) has open unresolved bugs.

### 3. RTCM injection doesn't work over DDS

`GpsInjectData` is not in the default `dds_topics.yaml` ([px4_ros_com #212](https://github.com/PX4/px4_ros_com/issues/212)). NTRIP RTK corrections can only flow via MAVROS2. Without RTK, positioning degrades from ±1.5cm to ±1-2m. This alone forces MAVROS2 to stay running regardless of DDS.

### 4. Dual-bridge race condition is a silent killer

Both MAVROS2 and DDS write to the same uORB topics on the FCU (`offboard_control_mode`, `trajectory_setpoint`). PX4's uORB does **not** arbitrate — last writer wins. With two bridges active, setpoints interleave unpredictably. On a ±3cm arc, this manifests as chatter or divergence with **no error indication**.

Mitigation exists (disable MAVROS setpoint plugins, add ControlSupervisor node) but adds complexity and new failure modes to a system that should be getting simpler, not more complex.

---

## MAVROS2 vs DDS — Side-by-Side

### MAVROS2

| Pro | Detail |
|---|---|
| **Proven on rovers** | Working today on our hardware. AUTO mode already functional. |
| **NTRIP RTK native** | `/mavros/gps_rtk/send_rtcm` injects corrections directly. No workaround needed. |
| **QGC built-in** | `udp-b://:14550@` gives live GCS telemetry, param tuning, firmware flash. |
| **RC failsafe** | RC override path works through MAVROS. Hardware fallback exists. |
| **Arm/mode services** | `/mavros/cmd/arming`, `/mavros/set_mode` — stable, documented, tested. |
| **Zero race conditions** | Single bridge, single writer, deterministic behavior. |
| **Param read/write** | Full parameter service via MAVLink. DDS has no equivalent. |
| **Community support** | Most PX4 users run MAVROS2. Forum help is available. |

| Weakness | Detail | Impact |
|---|---|---|
| **50 Hz max rate** | MAVLink serial overhead limits update rate | For a 0.4 m/s rover, 50Hz = 8mm between updates. Likely sufficient. |
| **20-35ms typical latency** | Serialization + USB + parsing overhead | At 0.4 m/s, 35ms = 14mm position lag. RPP lookahead compensates. |
| **NED coordinate translation** | Velocity setpoints go through NED→body transform | Adds ~5ms and potential frame convention bugs. |
| **No rover-specific setpoints** | Uses generic `TwistStamped`, not `RoverSpeedSetpoint` | Less semantic clarity but functionally equivalent. |
| **MAVLink overhead** | ~280 bytes per message vs ~50 bytes CDR | Bandwidth waste at 921600 baud is negligible. |

### uXRCE-DDS

| Pro | Detail |
|---|---|
| **100 Hz native rate** | Direct uORB publish, no serialization overhead |
| **<1ms latency** | CDR serialization, no MAVLink parsing |
| **Rover-specific setpoints** | `RoverSpeedSetpoint`, `RoverSteeringSetpoint` — semantic match |
| **Native CDR efficiency** | ~50 bytes vs ~280 bytes per message |
| **No NED translation** | Setpoints in body frame natively |
| **PX4's intended direction** | Future PX4 versions prioritize DDS over MAVLink |

| Weakness | Detail | Impact |
|---|---|---|
| **Chronic reconnect failure** | eProsima #48 open since 2019 | Agent restart → FCU reboot required. Production-unacceptable. |
| **Rover offboard broken** | Wheels don't spin (forum 48430) | DDS rover control doesn't work today. Period. |
| **No NTRIP/RTCM** | GpsInjectData not in dds_topics.yaml | RTK impossible without MAVROS2 or firmware patch + rebuild. |
| **No QGC support** | No MAVLink = no Ground Control Station | Can't tune params, flash firmware, or monitor in field. |
| **No param service** | DDS has no parameter read/write | Can't adjust PID gains without MAVROS2 or QGC. |
| **Experimental API** | Rover setpoints in `experimental/` path | API may change between versions. Not production-grade. |
| **Dual-writer race** | Two bridges write same uORB topics | Silent failures, nondeterministic control. Requires complex mitigation. |
| **Zero commercial rover deployments** | Nobody ships rover products on DDS | No reference implementations, no field-proven patterns. |
| **Failsafe runaway** | Last setpoint persists after disconnect (#18346) | Safety-critical — rover keeps driving after signal loss. |

---

## The Math: Does MAVROS2's 50Hz Matter?

For a marking rover at **0.4 m/s** on **1.5m radius arcs**:

| Metric | Value |
|---|---|
| Distance per update at 50Hz | 0.4/50 = **8mm** |
| Position error from 35ms latency | 0.4 × 0.035 = **14mm** |
| RPP lookahead distance (min) | **1.0m** (current param) |
| Required arc accuracy | **±30mm** |
| Worst-case MAVROS2 tracking error | ~22mm (8mm + 14mm) |
| **Headroom** | **8mm** — tight but within spec |

At 0.4 m/s, MAVROS2's 50Hz and 35ms latency gives ~22mm worst-case tracking error against a 30mm budget. **It should work.** If it doesn't, speed reduction to 0.3 m/s buys 35% more headroom before we need DDS.

DDS would improve this to ~4mm tracking error (1ms latency + 10mm at 100Hz) — but only if the rover actually responds to DDS setpoints, which it currently doesn't.

---

## Clear Summary

| | MAVROS2 (NOW) | DDS (FUTURE) |
|---|---|---|
| Status | **Working today** | Broken on rovers, experimental |
| NTRIP/RTK | Native | Not available |
| QGC/Monitoring | Built-in | Not available |
| Failsafe | RC override + RTL | Last-setpoint runaway bug |
| Rate | 50 Hz (sufficient at 0.4 m/s) | 100 Hz (overkill at 0.4 m/s) |
| Latency | 20-35 ms | <1 ms |
| Community | Large, active | Small, rover-specific gaps |
| Commercial rover use | Proven | Zero deployments |
| Risk | Low | High |

**Bottom line:** MAVROS2 is sufficient for our performance requirements and has zero blocking bugs. DDS has 4+ blocking bugs including a safety-critical runaway issue. Investing time in DDS now would be engineering speculation — fixing bugs in a system we don't need yet.

**Phase 2 path: MAVROS2 OFFBOARD → straight lines → arcs → ship.** DDS is the Phase 4 upgrade, not the Phase 2 prerequisite.

---

## When to Revisit DDS

Re-evaluate DDS if **any** of these conditions change:

1. MAVROS2 at 50Hz fails to achieve ±3cm on arcs (measured, not theorized)
2. PX4 merges fixes for eProsima #48 (reconnect) AND forum 48430 (rover wheels spin)
3. `RoverSpeedSteeringSetpoint` graduates out of `experimental/`
4. `GpsInjectData` lands in default `dds_topics.yaml`
5. PX4 v1.17+ ships with stable rover DDS offboard control

Until then, DDS remains shelved. All Phase 2 development targets MAVROS2 exclusively.