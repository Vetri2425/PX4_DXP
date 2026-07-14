# Commercial Line-Marking Robots — Market Benchmark & Upgrade Path

**Date:** 2026-07-14
**Branch:** `baseline_master` @ `ea47ec8`
**Status:** Research / strategy — no code changes
**Author:** Research pass (web sources, cited inline)

---

## TL;DR

**Our control accuracy is already at or above commercial parity — that is not the gap.**
Sub-2 cm RMS cross-track is competitive with everything selling today outdoors on GNSS
(TinySurveyor markets 1–3 cm, CivDot ~2–3 cm, Turf Tank / TinyLineMarker ~1 cm *positioning*).

The three real gaps are:

1. **Throughput** — we are 3–6× too slow.
2. **Workflow / fleet maturity** — no cloud, no fallback, no fleet layer.
3. **Safety certification** — nothing started; blocks commercial sale.

> ### ⚠ CORRECTION (2026-07-14, same day) — the original P0 in this doc was WRONG
>
> This document originally claimed the UM982's **dual-antenna heading was unused** and proposed
> enabling it as the top-priority "46% free accuracy win."
>
> **That was incorrect. Dual-antenna heading is ALREADY FUSED.** Verified against the FCU param
> dumps in-repo:
>
> - `PX4_params/12-06-2026/init.params:251` → **`EKF2_GPS_CTRL = 15`**. Bit 3 (value 8) =
>   *"Dual antenna heading"* per PX4 `src/modules/ekf2/params_gnss.yaml:9-16`. Default is `7`
>   (heading OFF). We have `15` = 8+4+2+1 → **bit 3 explicitly set.**
> - Deliberately tuned, not accidental: `7` (May) → `11` → settled at **`15`** by 2026-06-08,
>   stable through 06-12.
> - Corroborating: `GPS_1_CONFIG=101` (=TELEM1), `SER_TEL1_BAUD=230400` (UM982's mandated rate,
>   non-default), `GPS_1_PROTOCOL=6` (NMEA), `GPS_YAW_OFFSET=180` + `EKF2_GPS_YAW_OFF=180`
>   (a heading-offset calibration that is only meaningful if heading fusion is live).
>
> **The 46% win described below is already banked.** P0 is struck. See
> `02_PRODUCTION_HARDENING_AUDIT.md` for the real priorities.
>
> **Residual open question (read-only check):** params prove PX4 *intends* to fuse heading if the
> receiver supplies a valid one — not that the UM982 is currently achieving baseline lock.
> Settle in QGC → MAVLink Console: `listener estimator_status_flags -n 1` →
> want `cs_gnss_yaw: True`, `cs_gnss_yaw_fault: False`.

The paper below remains a useful reference for *why* dual-antenna heading matters at our speed —
it just describes a capability we already have, not one we're missing.

---

## 1. The market splits into three tiers

"Marking robot" is really **three different products** with three different physics. The accuracy
numbers are **not comparable across tiers**, and conflating them leads to bad targets.

### Tier A — Indoor construction layout (mm-class, total-station localized)

| Product | Accuracy | Localization |
|---|---|---|
| Dusty Robotics FieldPrinter 2 | **1.6 mm** (1/16″) | Laser tracker (Leica AT500) + IMU + encoders |
| HP SitePrint | **±2 mm** | Robotic total station (Leica TS16/TS60, Topcon LN-150, Trimble S9) + internal prism |

**Neither uses GNSS.** They track a total-station / laser-tracker prism. That is how they reach
mm accuracy — and why they only work indoors on a flat slab, need a surveyor to set the
instrument, and lose the robot on line-of-sight break (Dusty falls back to dead-reckoning
"shadow printing").

> **This tier is not our competition. Their mm numbers should not drive our targets — different
> sensor class entirely.**

### Tier B — Sports turf (cm-class, RTK GNSS)

Turf Tank, TinyLineMarker, Swozi. Quoted **±1 cm**. Differential drive, base-station RTK, tablet
app, pre-loaded sport templates. **Architecturally our exact machine.** Pricing is public
(see §6).

### Tier C — Road / civil pre-marking (1–3 cm, RTK GNSS, outdoor) ← **we are here**

| Product | Accuracy | Notes |
|---|---|---|
| TinySurveyor Plotter | **1–2 cm** | GNSS or robotic total station |
| TinySurveyor Terra | **2–3 cm** | 600 pts/hr, 8 hr battery |
| CivDot | 3 cm | 3,000 pts/day |
| CivDot+ | 8 mm | dual-RTK + IMU + **robotic arm** |
| CivDot Mini | ~2 cm ("sub-inch") | spray paint, **17 mi/day** |
| RoadPrintz | — | truck-mounted IP67 arm, 3000 psi airless — different beast |

Published accuracy here is **worse than turf** — outdoor, longer range, rougher ground, often
needs total-station fallback.

### ⚠ Critical framing point

Vendor "accuracy" is almost always the **RTK positioning spec** (what the receiver does),
**not** the achieved **paint placement error of a moving robot**.

Our "sub-2 cm RMS cross-track at 0.35 m/s" is a **tracking** metric — strictly harder and more
honest. **When we benchmark, insist on this distinction. It is a selling point, not a weakness.**

---

## 2. Where we actually stand

| Dimension | Commercial state of the art | **Our stack** | Verdict |
|---|---|---|---|
| **Placement accuracy** | TinySurveyor 1–3 cm; CivDot 3 cm / CivDot+ 8 mm (arm); CivDot Mini ~2 cm w/ spray | **sub-2 cm RMS x-track** (arc 1.46 / L 0.90 / sq 0.87 / U 1.06 cm) | ✅ **At or above parity** |
| **Marking speed** | Swozi Auto **up to 4.5 mph (~2.0 m/s)**; CivDot Mini **17 mi/day (~27 km)**; TinySurveyor 600 pts/hr | **0.35 m/s (~1.26 km/h)** | 🔴 **3–6× too slow — #1 gap** |
| **Heading source** | Dual-antenna GNSS heading (UM982-class) + IMU + NHC; MPC tracking | **Dual-antenna GNSS heading FUSED** (`EKF2_GPS_CTRL=15`, bit 3) + EKF2 wheel-encoder fusion | ✅ **At parity — already enabled** |
| **Localization redundancy** | Total-station fallback (TinySurveyor); manual mode on GNSS loss (Swozi) | RTK only | 🟠 No graceful degradation |
| **Obstacle / personnel detection** | TinySurveyor: **ultrasound + hi-vis beacon**; HP SitePrint: obstacle avoidance + cliff sensors | **None** | 🔴 **Blocks commercial sale** |
| **Safety certification** | ISO 3691-4 (harmonized w/ Machinery Directive 2006/42 **May 2024**); PLd per ISO 13849 for personnel detection | Not started | 🔴 **Blocks EU/CE sale** |
| **Planning: CAD ingest** | DXF + CSV via tablet or **USB stick** (TinySurveyor); DXF via CivPlan; native Revit/AutoCAD plugins (Dusty) | DXF→path, per-line PRE/MARK/AFT, 5 cm densification, TSP ordering | ✅ **Competitive, arguably richer** |
| **Operator app** | Tablet app + cloud project mgmt; TinyConnectivity license **$1,990/yr** | FastAPI + Socket.IO + Expo RN app | 🟡 Functional; no cloud/fleet layer |
| **Runtime** | TinySurveyor **8 hr**; Turf Tank 650 Wh ≈ 4 full pitches | Unspecified | 🟡 Need to publish a number |
| **Price** | TinyLineMarker Sport **$21,750**; Pro X **$37,750** (or $55k all-in); Turf Tank ~**$43k** + $6–16k/yr | — | 💰 Healthy margin envelope |

---

## 3. Planning flow — nobody is doing anything magic

Vendor sites are marketing-only; TinyMobileRobots and TinySurveyor publish essentially **zero**
engineering detail (repeated dead ends on their own docs). But the workflow is consistent, and
**we already match it**:

> **DXF/CSV in → tablet → robot.**

- **TinySurveyor** — accepts DXF or CSV pushed from the tablet, or literally **plugged in on a
  USB key**. Operator sets projection shifts, marking settings, and robot velocities from the
  tablet.
- **CivDot** — **CivPlan** software, DXF blueprints, real-time reporting (coordinates,
  timestamps, tolerance levels, ground elevation).
- **Dusty / HP** — the sophisticated end: native Revit/AutoCAD plugins, full BIM coordination.

The real control detail lives in the **academic** work, not the vendor sites. The 2025 JACIII
pre-marking robot paper describes:

- **Three-point circle correction** to generate a target path *carrying curvature*
- **Curvature-adaptive pure pursuit** — lookahead scales with speed **and** curvature radius
- **Fuzzy feedback compensation** on the control variable

That is a meaningful refinement over fixed-lookahead RPP, and **directly portable to our
controller** (see [P3](#p3--adopt-curvature-adaptive-lookahead)).

> **Our planner (per-line PRE/MARK/AFT extensions, 5 cm densification, collinear merging, TSP
> ordering, spray-boundary metering) is genuinely more sophisticated than what most of these
> vendors document publicly. Don't under-rate it. The gap is not planning.**

---

## 4. Control flow — our single biggest technical opportunity

### The dual-antenna heading finding

[Scientific Reports, 2025 — *Enhancing navigation control accuracy of guidance line drawing robot
by dual antenna GNSS and MEMS IMU*](https://pmc.ncbi.nlm.nih.gov/articles/PMC12267425/) built a
**two-wheel differential line-drawing robot** using the **Unicore UM982** — the same module on
our TELEM1.

**Why single-antenna heading fails for us specifically:**

> Heading derived from GNSS course-over-ground **diverges at low speed and constant-velocity
> straight-line motion**, because the acceleration signal needed to observe heading error
> vanishes.

**This is precisely our regime: 0.35 m/s, long straight marking runs.** It is the textbook
failure mode for this application.

**UM982 dual-antenna spec (from the paper):**

| Quantity | Spec |
|---|---|
| Heading RMS | **0.1° @ 1.0 m baseline** |
| Position | 0.8 cm + 1 ppm |
| Velocity | 0.03 m/s |

**Their fix:** loosely-coupled **error-state Kalman filter** fusing INS mechanization + GNSS
pos/vel + **dual-antenna heading observation** + **non-holonomic constraints (NHC)**, feeding
gyro/accel bias corrections back.

**Results at 0.1–0.4 m/s (our band):**

| Metric | Baseline | With dual-antenna + NHC | Δ |
|---|---|---|---|
| Heading RMS | 0.4403° | **0.2390°** | **−46%** |
| Lateral RMS | 2.64 mm | **1.42 mm** | **−46%** |
| Max lateral error | 7.97 mm | **4.12 mm** | **−48%** |

Position accuracy improved 15% / 42% / 60% in E / N / U respectively.

> **They reached ~1.4 mm RMS lateral. We are at ~9–15 mm. Same GNSS chip, same drive
> configuration, same speed. The delta is in the estimator, not the plant.**

Their tracking law was **MPC** with constrained control increments — not pure pursuit.

### Guidance law across the field

Pure pursuit (with **adaptive** lookahead) and MPC dominate. Our regulated pure pursuit is a
reasonable choice — but note that **both** best-documented systems moved *past* fixed-lookahead
PP: one to curvature-adaptive lookahead + fuzzy compensation, one to MPC.

### On our known arc limitation

Our `CLAUDE.md` correctly identifies that velocity-mode OFFBOARD discards
`trajectory_setpoint.yawspeed`, giving a structural following error of `≈ ω / RO_YAW_P`.

Worth stating plainly:

> **The commercial machines do not have this constraint** — they don't run a flight-stack
> autopilot in velocity mode; they command wheel velocities directly.

This is **self-inflicted architecture debt from the PX4 choice**, and it caps arc quality until
we either move to position/trajectory setpoints or bypass PX4's rover controller.
See `docs/OFFBOARD_POSITION_MODE_PLAN.md`.

---

## 5. Safety — the thing that will actually stop us selling

The least glamorous and most load-bearing finding.

**ISO 3691-4 was harmonized with Machinery Directive 2006/42 in May 2024**, making it *the*
presumption-of-conformity route for CE marking an autonomous ground vehicle in Europe.

It requires:

- **Personnel detection rated to PLd (ISO 13849-1)** — redundant controllers, fault detection,
  independent monitoring. **A single ultrasonic sensor on a non-safety-rated MCU does not
  satisfy this.**
- Hardware **and** software design requirements, including defined safe-stop behaviour.
- CE marking applies to the vehicle; **the integrator carries residual system risk.**

The commercial players' visible answers are modest — TinySurveyor ships **ultrasound detection +
high-visibility beacon**; HP SitePrint has obstacle avoidance and cliff sensors.

But for **road** work specifically (live or semi-live carriageway, workers present), our exposure
is far higher than a sports pitch. **We currently have no detection layer at all.**

> A rover that can drive itself at 1 m/s on a road with a paint system and no personnel detection
> is not a sellable product in the EU, and is a liability anywhere.

---

## 6. Pricing envelope

| Product | Price |
|---|---|
| TinyLineMarker **Sport** | **$21,750** outright / $302 mo / "Pay As You Spray" $250 mo min |
| TinyLineMarker **Pro X** | **$37,750** standard / **$55,000** all-inclusive / $524–763 mo |
| TinyConnectivity license | **$1,990/yr** |
| Installation | $1,995 |
| Turf Tank | ~**$43,000** outright + **$6–16k/yr** subscription |

**We do not need to win on price.** The margin envelope is generous.

---

## 7. Recommendations — prioritized

### ~~P0 — Exploit the UM982 dual-antenna heading~~ — ✅ STRUCK: ALREADY IN USE

**This recommendation was wrong and is withdrawn.** `EKF2_GPS_CTRL = 15` (bit 3 set) — dual-antenna
heading fusion is already enabled and was deliberately tuned to that value in June 2026. See the
correction block at the top of this document.

**The only residual action** is a read-only confirmation that the receiver is achieving baseline
lock in the field (not just that PX4 is configured to accept it):

```
QGC → Analyze Tools → MAVLink Console:
    listener estimator_status_flags -n 1
Healthy: cs_gnss_yaw: True, cs_gnss_yaw_fault: False
```

**Doc-hygiene follow-up:** `docs/Architecture/FINAL_ARCHITECTURE.md:70` and
`docs/Researches/COMMERCIAL_ROVER_RESEARCH/Tasks/T4_sensor_fusion.md:57` both wrongly state the
UM982 is on `/dev/ttyUSB0`. The params prove **TELEM1**. The `ttyUSB0` claim appears miscopied
from the *proposed* uXRCE-DDS bridge in `docs/Researches/Hybride_Archi_Decision.md`. Fix both —
they will mislead anyone debugging GNSS. Also note `PX4_DXP_Tracker.xlsx` is **not** the "full FCU
set" CLAUDE.md claims it is (it omits `EKF2_GPS_CTRL`, `GPS_1_CONFIG`, `GPS_YAW_OFFSET`); the raw
`PX4_params/*/*.params` dumps are the real source of truth.

Related: `docs/FIELD_POSITION_ERROR_ANALYSIS_20260713.md`.

### P1 — Close the throughput gap (SPD-T1)

0.35 m/s is **not commercially viable** against Swozi's 2.0 m/s and CivDot Mini's 27 km/day.

The existing backlog target (**1.0 m/s line / 0.6 m/s arc** — "SPD-T1" in `CLAUDE.md`) is the
right one, and the prerequisite already noted — **verify RoboClaw top speed against
`RO_MAX_THR_SPEED=0.9`** — is the right first step.

Expect accuracy to degrade with speed. The P0 estimator work is what buys the headroom to spend.

> **Do P0 first, then P1 — in that order, or we spend accuracy we don't have.**

### P2 — Personnel/obstacle detection + start the ISO 3691-4 conversation

Minimum viable:

- Hi-vis beacon
- Forward ultrasonic / ToF
- **Hardware e-stop on a separate safety path from the ROS2 stack**

> **Do not route safety-stop through the FastAPI / Socket.IO layer.**

Table stakes to demo to a highway authority, let alone sell. Treat **PLd personnel detection** as
a funded workstream, not a sprint task.

### P3 — Adopt curvature-adaptive lookahead

Scale RPP lookahead by speed **and** path curvature radius (JACIII method). Directly targets
arc/corner performance; contained change to a controller we already own.

**Lower risk than an MPC rewrite.** Consider MPC only if P0 + P3 leave us short.

### P4 — Localization redundancy

Every serious competitor has a fallback — TinySurveyor supports robotic total stations, Swozi
drops to manual on GNSS loss.

Define what the rover does when **RTK degrades mid-line**. Right now the honest answer appears to
be "nothing good." Even a well-defined **safe stop + resume-on-fix** is a shippable answer.

### P5 — Positioning & pricing

Sell on the two things we can defensibly claim:

1. **Honest tracking-error numbers** (not receiver specs)
2. **Richer DXF planning pipeline** (per-line extensions, densification guarantees, spray-boundary
   metering)

---

## 8. Caveat on all of the above

Vendor accuracy claims are **marketing figures, unaudited**, and — as established in §1 —
frequently conflate receiver positioning spec with achieved marking placement.

**The academic numbers (JACIII, Scientific Reports) are the only ones in this document with
methodology behind them. Weight them accordingly.**

---

## Sources

1. [TinyMobileRobots — How a TinyMobileRobot Works](https://tinymobilerobots.com/how-a-tinymobilerobot-works/) · [Pricing](https://tinymobilerobots.us/robots/prices)
2. [TinySurveyor Plotter — Aptella](https://www.aptella.com/product/tiny-surveyor-plotter/) (2–3 cm, 600 pts/hr, 8 hr, ultrasound, DXF/CSV)
3. [TinySurveyor — Monsen Engineering: Road Pre-Marking & Striping Robot](https://www.monsenengineering.com/survey/tinysurveyor/road-pre-marking-striping-robot/)
4. [Turf Tank Two](https://turftank.com/us/turf-tank-two/) · [Subscription pricing](https://turftank.com/us/subscription/)
5. [Swozi Auto — autonomous line marking robot](https://swozi.com/auto/) (4.5 mph, 30 kg paint)
6. [Civ Robotics — CivDot Mini](https://www.civrobotics.com/robots/layout-robots/civdot-mini) (17 mi/day, sub-inch, DXF, CivPlan) · [CivDot+](https://www.civrobotics.com/robots/layout-robots/civdot-plus) (8 mm)
7. [Dusty Robotics FieldPrinter 2](https://www.dustyrobotics.com/fieldprint-platform) (1.6 mm, laser tracker) · [vs HP SitePrint](https://www.dustyrobotics.com/compare/fieldprinter-vs-siteprint)
8. [HP SitePrint](https://www.hp.com/us-en/printers/site-print/layout-robot.html) (±2 mm, robotic total station, 936 lin.ft/hr)
9. [RoadPrintz](https://roadprintz.com/) (F-550 + IP67 arm, 3000 psi airless)
10. **[Enhancing navigation control accuracy of guidance line drawing robot by dual antenna GNSS and MEMS IMU — Scientific Reports, 2025](https://pmc.ncbi.nlm.nih.gov/articles/PMC12267425/)** ← the key one (UM982, ESKF + NHC, 46% improvement)
11. [Dynamic Sampling and Control for Automated Road Pre-Marking Robot — JACIII 29(4), Jul 2025](https://www.jstage.jst.go.jp/article/jaciii/29/4/29_921/_article/-char/ja/) (curvature-adaptive pure pursuit)
12. [ISO 3691-4 & ISO 13849 for mobile robots](https://jlcrobotics.com/iso-3691-4/) · [Harmonization with Machinery Directive, May 2024](https://www.agvnetwork.com/automated-guided-vehicles-technology/standard-3691-4)
13. [Ohio DOT 640/641 Pavement Marking spec](https://www.dot.state.oh.us/divisions/constructionmgt/onlinedocs/specifications/2008cms/600/641.htm) (premark offset ≤ 1 in / 25 mm)

---

## Next action

> **Check whether the UM982's second antenna is connected and whether `heading` is published
> anywhere in the MAVROS tree.**
>
> If it isn't, that is a peer-reviewed **46% accuracy improvement** sitting unused on hardware we
> already own — the cheapest item on this list by a wide margin.
