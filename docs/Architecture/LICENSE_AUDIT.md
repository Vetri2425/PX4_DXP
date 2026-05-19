# License Audit — 3WD Marking Rover Stack

**Date:** 2026-05-20
**Verdict:** FULLY CLEAR for closed commercial proprietary use. No GPL contamination in the core stack.

---

## Executive Summary

Every component in our stack uses permissive licenses (BSD, Apache 2.0, MIT, MPL2, PSF). No component requires us to disclose proprietary source code. The only obligations are attribution and license text inclusion in product documentation.

**One warning:** RTKLIB v2.4.1 and earlier were GPLv3. Use only v2.4.2+ or the demo5 fork (BSD).

---

## Complete Stack License Table

### Firmware Layer (on CubeOrangePlus)

| Component | License | Commercial Use? | Key Obligation |
|---|---|---|---|
| **PX4 v1.16.2 firmware** | BSD 3-Clause | Safe | Include copyright + license text in docs |
| **Our 6 bug fixes** (RoverDifferential, mission_block, RoverLandDetector) | BSD 3-Clause (ours) | Safe | Not required to publish modifications |
| **MAVLink generated C headers** | MIT (explicit exception) | Safe | Include MIT copyright notice |
| **MAVLink generator toolchain** | LGPL v3 | N/A — we don't distribute the generator | Don't distribute modified generators |
| **NuttX RTOS** | Apache 2.0 | Safe | Include Apache notice |
| **CubeOrangePlus bootloader** | Proprietary (Hex/CubePilot) | Safe — not distributed | No obligation (stays on board) |
| **STM32 HAL** (if Plan B PCA9685 bridge) | BSD 3-Clause | Safe | Include copyright + license text |

### Communication Layer (on Jetson)

| Component | License | Commercial Use? | Key Obligation |
|---|---|---|---|
| **MAVROS2** | BSD/GPLv3/LGPLv3 (choose one) | Safe — **choose BSD** | Elect BSD; include BSD license text |
| **libmavconn** (MAVROS2 dependency) | BSD/GPLv3/LGPLv3 | Safe — choose BSD | Same as MAVROS2 |
| **GeographicLib** (MAVROS2 dependency) | MIT/X11 | Safe | Include MIT notice |

### ROS2 Layer (on Jetson)

| Component | License | Commercial Use? | Key Obligation |
|---|---|---|---|
| **ROS2 Humble core** | Apache 2.0 | Safe | Include Apache notice |
| **Fast-DDS** (DDS middleware) | Apache 2.0 | Safe | Include Apache notice |
| **Cyclone DDS** (alternative DDS) | EPL-2.0 OR BSD-3-Clause | Safe — choose BSD | Choose EDL-1.0/BSD-3 path |
| **Nav2** (core packages) | Apache 2.0 | Safe | Include Apache notice |
| **nav2_regulated_pure_pursuit** | Apache 2.0 | Safe | Include Apache notice |
| **nav2_amcl** | **LGPL-2.1+** | Safe with conditions | Dynamic link only; or don't use it |
| **robot_localization** | BSD 3-Clause | Safe | Include BSD notice (package.xml says Apache — it's wrong, real license is BSD) |

### Solver & Math Layer (on Jetson)

| Component | License | Commercial Use? | Key Obligation |
|---|---|---|---|
| **OSQP solver** | Apache 2.0 | Safe | Include Apache notice |
| **Eigen3** | MPL2 | Safe | Attribute; share modifications to Eigen files only |
| **NumPy** | BSD 3-Clause | Safe | Include copyright + license |
| **SciPy** | BSD 3-Clause | Safe | Include copyright + license |
| **Matplotlib** | PSF/BSD-compatible | Safe | Include license; note changes if distributing |
| **PyYAML** | MIT | Safe | Include MIT notice |
| **Python/CPython** | PSF v2 | Safe | Include license stack; trademark notice |

### GNSS & NTRIP Layer (on Jetson)

| Component | License | Commercial Use? | Key Obligation |
|---|---|---|---|
| **NTRIP protocol** | Open (spec costs $215) | Safe | Buy spec from RTCM; write own client |
| **Our ntrip_rtcm_node.py** | Proprietary (ours) | Safe — we own it | N/A |
| **RTKLIB** (if used for positioning) | BSD 2-clause (v2.4.2+) | Safe — **use v2.4.2+ only** | Include BSD notice. DO NOT use v2.4.1 (was GPLv3) |
| **UM982 protocol** | NMEA (open) + Unicore proprietary | Safe with purchased module | Use MIT driver or own parser |
| **u-blox UBX** (NOT in our stack) | Proprietary LULA | N/A — we don't use u-blox | Would require u-blox hardware |

### Platform & Infrastructure (on Jetson)

| Component | License | Commercial Use? | Key Obligation |
|---|---|---|---|
| **NVIDIA Jetson Orin / JetPack** | NVIDIA EULA | Safe | Use production modules (not dev kits); don't reverse-engineer SDK; not certified for safety-critical |
| **systemd** | LGPL-2.1+ (core), GPL-2.0+ (udev) | Safe | Dynamic-link libsystemd; include LGPL notice |
| **Ubuntu/Linux** | GPL/LGPL mixed | Safe | Standard Linux distribution rules apply |
| **Jetson GPIO** | MIT | Safe | Include MIT notice |

### Our Proprietary Code (not based on any GPL code)

| Component | License | Notes |
|---|---|---|
| **ntrip_rtcm_node.py** | Proprietary | Written from scratch. NTRIP protocol is open; our implementation is ours. |
| **OFFBOARD controller node** (Phase 2) | Proprietary | Uses MAVROS2 BSD-licensed API; our code is ours. |
| **RPP/MPC path tracker** (Phase 2) | Proprietary | Uses OSQP Apache 2.0; our code is ours. |
| **NHC virtual sensor** (Phase 2) | Proprietary | Publishes ROS2 topics; our code is ours. |
| **Spray controller** (Phase 3) | Proprietary | Jetson GPIO; our code is ours. |
| **STM32F103 encoder bridge** (Phase 2) | MIT | We can choose MIT or keep proprietary; using STM32 HAL (BSD). |
| **DXF parser / path generator** (Phase 3) | Proprietary | Our code. |

---

## GPL/LGPL Items to AVOID

| Item | License | Why Avoid | Alternative |
|---|---|---|---|
| **RTKLIB v2.4.1 or earlier** | GPLv3 | Full copyleft — must disclose all source | Use v2.4.2+ or demo5 fork (BSD) |
| **nav2_amcl** | LGPL-2.1+ | Must dynamic-link; static linking triggers copyleft | Don't use AMCL; use RPP controller instead |
| **FFmpeg with --enable-gpl** | GPLv2+ | Full copyleft | Use LGPL-only FFmpeg build or don't use FFmpeg |
| **GStreamer bad/ugly plugins** | Various GPL | Pipeline infection | Use only core GStreamer plugins |
| **pymavlink (mavutil.py)** | GPLv3 | Only affects Python tooling, not C headers | Don't include pymavlink in product; use generated C headers only |
| **ArduPilot/ArduRover** | GPLv3 | Full copyleft — cannot use in closed product | **Already abandoned for this reason** |

---

## Attribution Checklist (for product documentation)

Create a `THIRD_PARTY_LICENSES` file including:

1. **PX4** — BSD 3-Clause copyright notice + license text
2. **MAVLink** — MIT copyright notice (for generated headers)
3. **MAVROS2** — BSD 3-Clause copyright notice + license text (elected BSD)
4. **ROS2 Humble** — Apache 2.0 copyright notice + license text
5. **Nav2** — Apache 2.0 copyright notice + license text
6. **robot_localization** — BSD 3-Clause copyright notices (Charles River Analytics + UT Austin)
7. **OSQP** — Apache 2.0 copyright notice + license text
8. **Eigen** — MPL2 copyright notice + license text
9. **NumPy/SciPy** — BSD copyright notices + license texts
10. **PyYAML** — MIT copyright notice + license text
11. **Fast-DDS** — Apache 2.0 copyright notice + license text
12. **NuttX** — Apache 2.0 copyright notice + license text
13. **systemd** — LGPL-2.1 copyright notice + license text
14. **JetPack/NVIDIA** — NVIDIA EULA acknowledgment
15. **STM32 HAL** — BSD 3-Clause copyright notice (STMicroelectronics)
16. **Python** — PSF License Version 2

---

## Verification Sources

| Claim | Source |
|---|---|
| PX4 = BSD 3-Clause | [github.com/PX4/PX4-Autopilot/LICENSE](https://github.com/PX4/PX4-Autopilot/blob/main/LICENSE) |
| MAVLink generated code = MIT | [github.com/mavlink/mavlink/COPYING](https://github.com/mavlink/mavlink/blob/master/COPYING) |
| MAVROS2 = BSD/GPL/LGPL triple | [github.com/mavlink/mavros/LICENSE.md](https://github.com/mavlink/mavros/blob/ros2/LICENSE.md) |
| OSQP = Apache 2.0 | [github.com/osqp/osqp/LICENSE](https://github.com/osqp/osqp/blob/master/LICENSE) |
| RTKLIB v2.4.2+ = BSD 2-clause | [github.com/tomojitakasu/RTKLIB](https://github.com/tomojitakasu/RTKLIB/blob/master/readme.txt) |
| Eigen = MPL2 | [eigen.tuxfamily.org](https://eigen.tuxfamily.org/) |
| Nav2 RPP = Apache 2.0 | [github.com/ros-navigation/navigation2/LICENSE](https://github.com/ros-navigation/navigation2/blob/main/LICENSE) |
| robot_localization = BSD | [github.com/cra-ros-pkg/robot_localization/LICENSE](https://github.com/cra-ros-pkg/robot_localization/blob/ros2/LICENSE) |
| Fast-DDS = Apache 2.0 | [github.com/eProsima/Fast-DDS/LICENSE](https://github.com/eProsima/Fast-DDS/blob/master/LICENSE) |
| Jetson EULA commercial use | [NVIDIA JetPack EULA](https://docs.nvidia.com/jetson/jetpack/eula/) |
| NTRIP spec = open ($215) | [RTCM 10410.1](https://www.rtcm.org/) |