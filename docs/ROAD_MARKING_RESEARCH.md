# Road Line Marking Rover — Deep Research Findings

**Date:** 2026-07-22
**Method:** 5-angle web fan-out → 25 sources fetched → 114 claims extracted → 25 adversarially verified (3-vote, 2/3 refutes kills) → synthesis.
**Baseline for comparison:** 3WD sports-field marking rover (PX4 v1.16.2 / MAVROS / ROS 2 Humble, RTK UM982 dual-antenna, regulated pure pursuit, DXF→path engine, FastAPI backend).

**Evidence grading used throughout:**
- **[V]** = survived 3-vote adversarial verification
- **[E]** = extracted from a primary source with a supporting quote, but NOT put through verification (still source-grounded, lower confidence)
- **[R]** = refuted by verification — do not design against without re-confirming from the primary standard yourself
- **[GAP]** = no evidence gathered

---

## 0. The single most important finding

**Commercial road marking is two different machines, not one.** Everything else follows from which one you are building.

| | **Category A — Pre-marking / layout robot** | **Category B — Striping machine** |
|---|---|---|
| Example | TinyMobileRobots TinySurveyor; InfraROB | LimnTech LifeMark-400; RoadPrintz Electra100; Borum/Hofmann/Graco trucks |
| Platform | Standalone 3-wheel rover, ~18 kg **[E]** | Truck-mounted arm or carriage on a driven carrier **[E]** |
| Speed | **4 km/h max marking speed** (~1.1 m/s) **[E]** | **24–32 km/h painting, up to 130 km/h survey** **[E]** |
| Input | **DXF or CSV via USB key** **[E]** | Layout plan import, or machine-vision scan of existing markings **[E]** |
| Output | Reference marks / dots for a following crew — 750 ml paint can **[E]** | The final legal line: paint + glass beads, film-thickness controlled **[E]** |
| Localization | External survey GNSS (Trimble/Leica/Topcon), 1–2 cm **[E]** | Dual-antenna GNSS + MEMS INS, sub-50 mm RTK **[E]** |
| Autonomy | SAE L4, supervised by long-range remote **[E]** | Operator drives; robot positions the paint head only **[E]** |

Your existing rover **is already a Category A machine.** 18 kg three-wheeled GNSS carrier at walking pace, DXF-driven, RTK 1–2 cm — that is a specification-level match to InfraROB, which a 2026 peer-reviewed review names as the most autonomous pavement-maintenance robot published **[E]**.

The strategic consequence: **Category A is a port, Category B is a new product.** Most of what follows is scoped to "what changes for A", with B called out where it diverges.

> A verified caution against over-reading the speed story: a commercial pre-marking robot tops out at 4 km/h **[E]**, so road *pre*-marking is explicitly **not** a high-speed control problem. The highway-speed control literature below applies to Category B, and to the drive-to-site transit phase — not to the marking pass itself.

---

## 1. Regulation — the output becomes legally specified

This is the deepest change from sports-field work. A field line that is 2 cm off is cosmetic. A road line that is off-spec is **non-compliant**, and compliance is *measurable and auditable*.

### 1.1 Dash/gap geometry is context-dependent along the road **[V, 3-0]**

IRC:35-2015 §4.6.1 (India):

| Context | Centre line | Lane line (divided c/way) |
|---|---|---|
| Normal section | **3 m mark + 6 m gap** (LM01/LM02) | 3 m + 9 m |
| Warning section | **6 m mark + 3 m gap** (LM04/LM05) | **3 m + 3 m** (LM11/LM12) |

Every pattern carries a **coded marking type**. US MUTCD Part 3 uses a **10 ft line / 30 ft gap** 3:1 duty cycle for broken centre/lane lines **[E]**.

**Design consequence:** your path engine must emit spray-metering segments **keyed to road context and chainage**, not a fixed duty cycle — and the marking-type code must be a first-class attribute in the mission data model, because retroreflectivity rules exclude certain marking categories entirely (dotted extensions, curb, parking, shared-use path) **[E]**.

### 1.2 Retroreflectivity is a numeric, speed-scaled, time-bounded acceptance target **[V, 3-0]**

IRC:35-2015 §15, Tables 15.1/15.2 — **white markings only** (§15.5):

| Design speed (kmph) | Initial R_L (7 days) | Minimum threshold, warranty to 2 yr |
|---|---|---|
| up to 65 | 200 mcd/m²/lx | 80 |
| 65–100 | 250 | 120 |
| above 100 | 350 | 150 |

Wet R_w: **100 initial / 50 minimum, irrespective of design speed.** Measurement methods are **EN 1436**-based (Annexures D/E).

Daytime is *also* measured **[E]**: luminance coefficient Q_d ≥ 130 mcd/m²/lux on cement, ≥ 100 on asphalt, for the whole service life; plus skid resistance 55 BPN initial / 45 minimum at urban zebra crossings.

**Design consequence:** the backend stops being a path executor and becomes a **spec-driven job + QA system** that records measurable acceptance data with a warranty clock attached. This is a genuinely new backend subsystem, not an extension of the current mission store.

### 1.3 US numbers — **treat as unconfirmed** ⚠️

Verification **killed 0-3** both the MUTCD 50 mcd/m²/lx (≥35 mph) and 100 mcd/m²/lx (≥70 mph) claims **[R]**, despite direct quotes from the Federal Register. Reading the votes, the refutations appear to be about *scope and characterization* — the rule is a **maintenance-method obligation with a 4-year implementation window**, not a per-marking pass/fail at installation **[E]** — rather than about the numbers being wrong. Corroborating (unverified) detail: 50 white / 25 yellow above 35 mph, with an exemption for roads under 6,000 ADT **[E]**.

**Do not design US QA thresholds off this report.** Read MUTCD 11th Edition **§3A.05** (published 2023-12-19) and **FHWA-SA-22-028** directly **[E]**.

### 1.4 Line width — **unpinned** ⚠️

The IRC width claim (100 mm rural ≤100 kmph / 150 mm above, expressways and urban) was **refuted 0-3 [R]**. MUTCD longitudinal widths of 4–6 in (100–150 mm), wide edge lines 6–8 in, stop lines 12–24 in, crosswalks 6–24 in are **[E]** only.

This matters more than it looks: **line width is what sizes your nozzle/die and sets the cross-track error budget.** It is currently the largest unresolved number in the whole design. Transverse markings at 300–600 mm also imply a **variable-width applicator or multi-pass planning** **[E]** — your fixed-width field marker cannot do stop lines.

### 1.5 Material spec lives elsewhere

IRC:35-2015 explicitly does **not** define material formulations or application thickness — it defers to **MoRTH "Specification for Road and Bridge Works" Clause 803** **[E]**. For India, Clause 803 is your source for thermoplastic thickness and bead drop rate, not IRC 35.

### 1.6 Colour carries regulatory semantics **[E]**

Yellow = centreline / no-passing / left edge. White = lane lines, right edge, crosswalks, stop lines. Red/blue/green/purple = transit, disabled parking, bike, toll. **The job model needs a colour + material attribute per geometry element, and the machine needs multi-colour paint circuits.** Sports-field marking is monochrome; road marking is not.

---

## 2. Input geometry — from DXF vertices to linear-referenced alignments

### 2.1 The alignment data model **[V, 3-0, four merged claims]**

**IFC 4.3** (ISO 16739-1:2024) `IfcAlignment`:
- Three **separable** layers: `IfcAlignmentHorizontal` (x/y), `IfcAlignmentVertical` (distance-along/z), `IfcAlignmentCant` (**lateral inclination = superelevation**). "Only Horizontal (H)" is an explicitly valid configuration — **you can read the 2D centreline without touching profile or superelevation.**
- Alignments **nest `IfcReferent`s** — stations / mileage points (`Pset_Stationing`: IncomingStation, Station, HasIncreasingStation), including **broken-chainage** handling.
- Positioning is **explicitly grounded in ISO 19148 linear referencing**. `IfcPointByDistanceExpression` carries `DistanceAlong` + `OffsetLateral`/`Vertical`/`Longitudinal` on a basis curve — **literally the "lane line at fixed offset" construct you need.**
- Alignments aggregate hierarchically: child alignments reuse the parent's horizontal layout as `BaseCurve` **[E]** — exactly right for multi-lane jobs where many marking lines share one centreline.
- Business logic is deliberately separated from geometric representation **[E]** — your ingester handles **two parallel descriptions** of the same alignment, unlike DXF where only geometry exists.

**LandXML / Inframodel 4.0.2**: `Alignment` has mandatory `staStart` and `length`, with per-element `staStart` on Line/Curve/Spiral plus a continuity requirement. `StaEquation` handles labelled discontinuities while `staInternal` preserves a **continuous machine-usable station axis**. Vertical geometry is a separate `<Profile><ProfAlign>` of PVI points + circular vertical curves **[E]**. Cross-section parameters layered on the line-string model are **the mechanism by which lane widths and offsets-from-centreline are derived** **[E]**.

**Design consequence — the big one:** the path engine's native coordinate becomes **(chainage, lateral offset)** rather than XY vertices. That is an architectural change to `path_engine`, not a new importer.

**Caveats [V]:** referents and `Pset_Stationing` are **optional** (fall back to distance-along); `StartDistAlong` was removed in 4.3 RC4 so station is *semantic metadata*, not distance-along; **IFC 4.3 exporter maturity is uneven and actively changing** — re-check before committing to it as primary ingest; and **IFC alignment chainage is 2D-projected**, so true 3D arc-length for dash metering on graded sections needs the vertical layer.

> Practical note: TinySurveyor — the closest commercial analogue to your machine — ingests **plain DXF or CSV via USB key** **[E]**. Alignment ingestion is the *right* long-term architecture, but it is not the entry ticket. DXF gets you to market; alignments get you to highway projects.

### 2.2 Spiral/transition types — line+arc is structurally insufficient **[V, 3-0]**

LandXML's `spiralType` enumeration: biquadratic, bloss, clothoid, cosine, cubic, sinusoid, revBiquadratic, revBloss, revCosine, revSinusoid, sineHalfWave, biquadraticParabola, cubicParabola, japaneseCubic, radioid, weinerBogen. A **2025** commercial SDK (Bentley OpenRoads Designer) adds Viennese, NSWCubic, WACubic, MXCubic, CNCubic, Czech, ITCubic, HalfCosine, POCubic, AREMACubic — **several jurisdiction-specific, so multiplicity is growing, not legacy.**

LandXML `<Spiral>` stores **parametric** fields (spiType, length, radiusStart/End, PI, rot) with **no densified geometry** — an ingester genuinely must branch on type.

Two sharp traps **[E]**:
- **Sinusoidal and cosinusoidal transitions are NOT true spirals** — their rate of change of curvature is not constant. You cannot substitute a clothoid without introducing geometric error against design intent.
- **Cubic parabola is only valid below ~24°** — its radius reaches a minimum at 24°05′41″ then *increases* again. Any alignment validator must enforce this bound.

**Escape hatch [V]:** if the alignment is exported **pre-densified to a polyline**, spiral math is avoided entirely. This is probably your Phase 1.

### 2.3 Clothoid mathematics **[V, 3-0, two merged]**

κ(s) = κ₀ + σs, hence θ(s) = θ₀ + κ₀s + σs²/2 — **closed-form quadratic tangent angle**, arc-length parameterized. That linear-curvature property is *why* road transitions use them: they **bound lateral jerk at constant speed** (they do not eliminate it), and they double as **superelevation runoff** length.

Closed form applies to **tangent angle and curvature, NOT to x/y** — Cartesian position needs Fresnel integrals. Note the strong-form claim "therefore you must tessellate" was **refuted 0-3 [R]**; treat position evaluation as a practical numerical matter, not a proven necessity.

**Off-the-shelf:** `pyclothoids` (MIT, Python) — minimal immutable API, single `Clothoid` class, and crucially exposes **`SolveG2`** for **G2 (curvature-continuous) interpolation** between endpoints, which is the continuity class road alignments need — G1 tangent continuity is not enough **[E]**.

---

## 3. Control — the uncomfortable finding

**The verified literature says the tracking controller is not your accuracy bottleneck.** This is the most decision-relevant result in the whole report, and it argues *against* a rewrite.

### 3.1 Geometric controllers plateau at decimetre scale **[V, medium confidence]**

Jung, *Sensors* 2025, 25(20), 6491 — MORAI simulation, passenger-car bicycle model, 20 m-radius quarter circle:

| Speed | Pure Pursuit RMS | Stanley RMS | IMM hybrid |
|---|---|---|---|
| 30 kph | 0.125 m | 0.302 m | 0.114 m |
| 35 kph | 0.138 | 0.387 | — |
| 40 kph | 0.301 | 0.769 | 0.283 |
| General road | 0.481 | 0.643 | — |

Two results worth internalizing:
- **Pure Pursuit beat Stanley in every scenario**, by ~2.4–2.8× on curves. The paper itself states "it is known that the Stanley method is suitable for high-speed driving" — **and its own data contradicts that** (this reversal carried 2-1, reflecting genuine literature disagreement).
- The **IMM probabilistic blend improved on the better controller by only 6.0–8.8% (curve) and 3.3% (general)**. An 8.8% shave off decimetre error leaves decimetre error.

Note: the *absolute* accuracy claim from this paper was **refuted 0-3 [R]** on scope grounds — cite the **relative** findings as transferable, not the numbers. It is simulation-only, tuning-sensitive, and 30–40 kph passenger car, not a low-speed marking rover.

### 3.2 Clothoid-native MPC buys smoothness, not accuracy **[V, medium]**

Lima et al., ECC 2015 (LTV-MPCC): a controller parameterized directly in clothoid segments produces "very comfortable and smooth driving **while maintaining a tracking accuracy comparable to** that of a regular LTV-MPC, with the ability to use paths described by very sparse waypoints."

The clothoid identity holds because **at low speed** a piecewise-linear steering angle traces clothoid segments — so this **does not by itself support highway-speed claims.** Simulation only, no field validation, 11 years old. **Do not cite for any cm-level cross-track figure.**

### 3.3 What *does* move the needle **[E]**

- **NSGA-II multi-objective tuning of pure-pursuit lookahead + PID gains cut average pose error 55.94%** at 4 m/s with 0.5 m lookahead. That is an order of magnitude more leverage than swapping controllers.
- **Optimal parameters are speed-dependent**: Kp 0.5857–0.8852, Ki 0.0–0.3674 across lookaheads (0.5/1/1.5 m) and speeds (3/4/5 m/s). A road rover needs a **velocity-scheduled lookahead/gain table**, not the single frozen lookahead you use now.
- MUTCD-scale corroboration: lookahead should be speed-scheduled as `d_la = max(5, k_d·v_x)`, k_d = 0.5 s **[E]**.
- **Both geometric controllers destabilize at geometric discontinuities** — Stanley on discontinuous path points, Pure Pursuit immediately after curve exit, severity rising with speed **[E]**. This is a direct argument for **curvature-continuous path generation (clothoid/spline) instead of arc-line stitching** — i.e. fix it in the planner, not the controller.

### 3.4 A real road-marking robot's approach **[V, medium]**

Wang et al., *JACIII* 29(4), 2025-07-20 — autonomous road **pre-marking** robot: **three-point circle correction** to fit arcs through sampled road points, generating a target path that **carries curvature information**, fed to pure pursuit. Independently corroborated as standard practice (US patents 11372414, 8751089).

⚠️ Its headline accuracy figures (<1.5 cm curved, ~2 cm right-angle) and the "curvature-adaptive pure pursuit" characterization were **both refuted 0-3 [R]** — self-reported, unreplicated, modest venue. **Cite the method, not the numbers.**

### 3.5 Bottom line

> **No verified evidence in this pass shows cm-level road-marking accuracy demonstrated on hardware at road speed.** Every hardware accuracy claim failed verification.

Your cm-level budget will come from **localization + speed policy + applicator timing compensation** — the same three levers you already know from the sports rover — **not** from replacing regulated pure pursuit. That is a strong argument for keeping your controller and investing elsewhere.

---

## 4. Localization

### 4.1 The commercial bar **[E]**

LimnTech LifeMark (Advanced Navigation Certus): **sub-50 mm RTK, dual-antenna GNSS for heading + MEMS INS.** Your UM982 dual-antenna setup is architecturally the same class — this is a validation of your existing choice, not a call to change it.

**The INS payoff is availability, not accuracy:** switching to the Certus MEMS INS raised system availability **from 40% to 70%** in tree-covered locations, "enabling automated marking across larger geographical areas previously requiring manual backup operations" **[E]**. That reframes tight INS coupling as a *coverage/uptime* feature, which is a much easier business case than a precision feature.

### 4.2 The GNSS-denied reality is harsher than hoped **[E]**

A tightly-coupled particle filter fusing **vector HD map + LiDAR + RTK GNSS + INS** achieves only **sub-metre** horizontal accuracy in GNSS-challenging environments — **roughly an order of magnitude worse than road-marking tolerance.**

**Conclusion: bridges, tunnels and urban canyons cannot be marked at spec accuracy with this class of fallback.** Design for *detect-and-stop / hand to operator*, not *degrade-and-continue*. (Reported gains over baseline are real — 77/75/64/65% on 3D position and yaw vs GNSS-RTK/INS, vs 16/53/48/51% for LIO-SAM — but they improve a sub-metre answer, not a cm one.)

Useful mechanism: **lane geometry from a vector HD map plus LiDAR constrains the lateral axis** directly — a way to hold a lane offset when GNSS drops. And **longitudinal (along-track) is the harder axis**, resolved via constrained damped LAMBDA ambiguity search **[E]** — which matters specifically for **chainage accuracy and dash-gap metering**.

### 4.3 Raw GNSS eats most of your budget **[E]**

Even with high-precision augmentation (Mitsubishi AQLOC/QZSS) and 15–17 satellites, raw error was **±2–3 cm longitude, ±1–2 cm latitude**. IMU fusion and controller error stack **on top** of that. If your line width tolerance is a few cm, the localization layer alone consumes most of it.

### 4.4 Re-marking over existing lines is a new perception stack **[E]**

Both commercial leaders do this and it has no sports-field analogue:
- **LimnTech**: HD cameras + AI-driven processing layered on RTK; **three modes — Full Auto, Semi-Auto, Manual** — full-auto only viable when existing markings are high-contrast and patterns simple.
- **RoadPrintz**: scan an existing marking ~10 s, reproduce in <3 min.
- **Hong et al.** (*Automation in Construction*, 2008): the primary sensing task is machine-vision recognition of **half-faded** lane marks, which then *defines* the path to paint. Validated on image data only — no closed-loop painting trial.

**Design consequence:** re-marking inverts your pipeline. Geometry comes from *perception*, not from a design file. And note that the market leader ships **three autonomy modes** — full autonomy is explicitly not always achievable. Plan for graceful degradation to operator assist as a *product feature*, not a failure state.

### 4.5 Lever arm is real **[E]**

A research platform mounts the GNSS antenna on a **1 m mast**; antenna-to-tool lever-arm offset is called out as physically significant and requiring compensation. You already know this from `spray_nozzle_offset_plan` — it generalizes, and on a taller road machine with superelevation (roll) it gets worse, because a roll angle times a 1 m lever arm is a centimetre-scale lateral error.

---

## 5. Applicator — the largest genuinely new subsystem

Your sports rover has a solenoid: on/off, one circuit, pre-mixed paint. Essentially none of that survives.

### 5.1 Glass beads are mandatory and physically subtle **[E]**

Retroreflectivity comes from **glass beads dropped into wet paint** — a whole second subsystem (tank, dedicated pneumatic supply, separate applicator) that sports-field marking does not have.

- **Wet-paint window is a hard constraint**: bead gun mounted **no more than one yard (~0.9 m) behind** the paint gun; bead spreading is *simultaneous* with paint application so quick-drying paint is still receptive.
- **Bead-drop physics governs quality, not just placement**: conventional bead guns impart the truck's forward velocity (8–12 mph) to the beads, so they **roll on impact and get partially or completely coated with paint**, destroying retroreflectivity.
- **The fix is closed-loop speed-proportional actuation**: counter-rotating rubber rollers expel beads downward and rearward at a speed matching vehicle forward speed, so beads land at **net-zero horizontal velocity**. An electric motor drives the primary roller and a controller **continuously monitors vehicle velocity**, sending a variable voltage to adjust roller RPM in real time — *automatically*, not by operator preset.
- Patented commercial practice (EZ Liner, priority 2008, issued 2012), not a research proposal.

**Design consequence:** the bead subsystem consumes a **live ground-speed signal** as a control input — exactly the speed-proportional flow architecture in your Spray V2 plan §7.5, but with a hard physical justification and a second independently controllable actuator channel.

### 5.2 Flow control is pressure-based and speed-referenced **[E]**

US 8,880,362 (Epic Solutions, issued 2014, **active to 2031 — an IP constraint on speed-proportional marking flow control**):
- **Inline flow meters fail** in high-temperature thermoplastic, so volume is inferred from **line pressure** instead.
- Film thickness = f(supply pressure, **vehicle distance travelled / speed**), computed **per paint gun** → **flow must be speed-referenced, not time-referenced.**
- Closed loop: recalculate thickness from new pressure samples, **automatically adjust pump speed** to hold thickness in range.
- Modest sensing: 0.1 PSI sensitivity, **1–4 Hz sampling**, optionally + material temperature and pump speed.
- Operational driver: **remaining-material prediction** — running a tank empty on a highway forces backtracking of an entire traffic-control convoy.

RoadPrintz corroborates: **closed-loop computer control of wet film thickness AND bead dispersion** — the applicator meters both, rather than toggling a solenoid **[E]**.

### 5.3 Paint chemistry changes the plumbing **[E]**

Road striping uses **two-component external-mix paint** — components stay separate until the instant of spraying. That means **separate supply lines, separate pumps, no pre-mixed reservoir.** Plus:
- **Airless high-pressure atomization** (Graco compressor + spray tips) for clean edges and transfer efficiency — not the low-pressure solenoid nozzle of field marking.
- Multiple large reservoirs with **source switching** (five 5-gal buckets, two selectable sources), waterborne **and** solvent-based support, onboard **water flush tank**, and an **automated tip-flipper for clog clearing** → material-handling, purge and **clog-recovery logic in the control backend**.

### 5.4 Accuracy benchmark **[E]**

RoadPrintz claims **±2 mm line-width tolerance**, ~3× better than manual. Note this is *line width*, a different quantity from cross-track error, and it is a vendor claim with no disclosed sensor suite, calibration procedure or positioning tolerance.

### 5.5 Regulatory geometry in the planner **[E]**

RoadPrintz drives marking geometry from a **library of MUTCD-compliant symbols** laid out on a computer-aided grid on an operator touchscreen — **the regulatory standard is encoded directly in the path-generation layer, replacing raw DXF import.** That is a concrete, proven design pattern for §1: ship a standards library, not a CAD importer.

---

## 6. ROS 2 / system architecture

**[GAP]** — no claim in this area survived verification, and the verification pass never selected ROS 2 architecture claims. The one architectural reference below is **[E]** from a peer-reviewed review.

### 6.1 A published reference architecture **[E]**

ASCE *J. Constr. Eng. Manage.* (10.1061/JCEMD4.COENG-16623) — autonomous highway maintenance vehicle (AMV):

- **Six-module stack**: sensing → perception/localisation → planning → control → actuation → communication.
- **Control module runs PID or MPC at millisecond resolution.**
- **Actuation couples a 6-DoF arm or XY gantry with a nozzle head plus pump and flow meters** for closed-loop volumetric delivery.
- **Communication is DDS + ROS middleware** linking vehicle → digital twin → manned control centre, with a human operator able to supervise or **teleoperate on complex edge cases**. Remote supervision is an *explicit architectural element*, not an afterthought.
- Stated requirements: SAE Level 5, waypoint-following **<0.1 m error at >80 km/h**, cm-level localization surviving GNSS-denied bridges/tunnels, **>100 TOPS** automotive-grade compute.
- Localization: DGPS/RTK-GNSS + IMU (sub-5 cm at highway speed) + camera/LiDAR fusion + SLAM + **HD map matching** to hold lane-level accuracy through GNSS outages.

Note the internal tension: the review asks for **<0.1 m waypoint following** at highway speed — a *decimetre* spec, consistent with §3 — while separately asking for cm-level localization. **The tight number is on localization, the loose one is on tracking.** That is the correct allocation and it matches your experience.

### 6.2 PX4/MAVROS vs pure ROS 2 — an honest read

The evidence does **not** force a rewrite:
- Your controller is not the accuracy bottleneck (§3.5) — the usual argument for moving to nav2/MPC evaporates.
- The published reference architecture is **DDS + ROS middleware**, which describes both options.
- Nothing verified addresses nav2 vs custom, ros2_control, micro-ROS, lifecycle or QoS design for this application.

The real forcing function is **functional safety** (§7), not control performance. Your current e-stop path is mediated through MAVROS — publishing a single-point path so RPP holds position. Whether that can satisfy a road work-zone functional-safety obligation is **the open architectural question**, and it is unresearched.

**Recommendation:** do not rewrite the stack on control grounds. Re-open the question only when you have the functional-safety requirements in hand.

---

## 7. Safety and work-zone operation

**[GAP]** on the standards themselves — ISO 3691 / ISO 13849 / ISO 12100, e-stop categories and work-zone traffic control returned **zero verified claims and were not searched effectively.** This is the single biggest hole in this report and is a hard blocker for any road deployment.

What the sources do establish **[E]**:

- **Every published pavement-maintenance robot assumes a protected workspace** — rolling traffic control, closed motorway, or car park. **None has real-time perception or obstacle avoidance.** Autonomous navigation, where implemented, relies only on GNSS and dead reckoning. The most autonomous example (InfraROB, SAE L4) is the ~18 kg three-wheeled TinyMobileRobots carrier with 1–2 cm GNSS.
- **Safety is achieved architecturally, by removing people, not by making the robot smart.** RoadPrintz: "Road workers stay in the truck cab — No Boots on the Ground!" LimnTech: eliminates layout crews on active roadways. Hong et al.: one supervising operator in the cab replaces multi-worker crews exposed to live traffic.
- **The commercial value proposition is crew exposure reduction** — LimnTech cuts a half-day, 4–5 person string-line layout to ~20 minutes and 1–2 people; a Missouri DOT RoadPrintz deployment reported up to 40% project time reduction.
- Supervised, not unattended: TinySurveyor runs ~8 h/charge under a **long-range remote controller**.

**Two conclusions that should shape the product:**
1. **You are not expected to solve live-traffic autonomy.** The entire industry operates inside protected work zones. Designing for that is normal practice, not a compromise.
2. **Supervised autonomy is the commercial norm at every price point.** Full autonomy is neither expected nor, per LimnTech's three-mode design, always achievable.

---

## 8. Backend / server functions

**[GAP]** — no verified claims. Requirements inferred from the regulatory and applicator evidence above (marked as inference, not findings):

| Function | Driver |
|---|---|
| Marking-type coded catalogue (LM01/LM02…, MUTCD symbol library) | §1.1, §5.5 |
| Road-network attributes per segment: **design speed, ADT, illumination, urban/rural, warning-section flags** | §1.1, §1.3 — these *select which spec applies* |
| Colour + material attribute per element; multi-circuit paint | §1.6 |
| Alignment ingestion: IFC 4.3 / LandXML → (chainage, offset) | §2.1 |
| Chainage carried end-to-end: job → path → metering → as-built | §2.1 |
| **Retroreflectivity QA store** with warranty clock, EN 1436 measurement records, R_L / R_w / Q_d / BPN | §1.2 |
| Material telemetry: film thickness, bead rate, **remaining-material prediction** | §5.2 |
| Clog/purge/flush state machine | §5.3 |
| Fleet + offline operation | §0, §7 |

Your existing FastAPI mission/job architecture is the right substrate; the QA/asset-lifecycle half is new.

---

## 9. What this means for your project

### Keep (validated by the research)
- **Regulated pure pursuit** — controller choice is not the bottleneck (§3.5), and PP beat Stanley on curvature (§3.1).
- **RTK + dual-antenna heading** — matches the commercial bar (§4.1).
- **DXF ingest** — still the commercial norm for pre-marking (§2.1 note).
- **~18 kg 3-wheel GNSS carrier at walking pace** — this *is* the Category A form factor (§0).
- **Speed-proportional spray + nozzle-offset plans** — both independently confirmed as necessary (§5.1, §4.5).

### Add (ordered by leverage per unit effort)
1. **Standards/spec layer** — marking-type catalogue, context-keyed dash/gap metering, colour+material model. Highest value, lowest technical risk, and it is what makes the machine *legal* rather than merely accurate. (§1)
2. **Velocity-scheduled lookahead/gain table** — ~56% error reduction reported from tuning alone; you already have the bag-analysis harness to A/B it. (§3.3)
3. **Curvature-continuous path generation** (clothoid/spline via `pyclothoids` SolveG2) replacing arc-line stitching — fixes the controller instability at geometric discontinuities in the *planner*, which is also the right fix for your open arc-flow P1 issue. (§2.3, §3.3)
4. **Retroreflectivity/QA backend** with warranty clock. (§1.2)
5. **Alignment ingestion** (chainage, offset) — architecturally the deepest change; defer behind pre-densified polyline export. (§2.1)
6. **Bead subsystem + two-component paint + pressure-based flow** — only if moving to Category B. (§5)

### Do not do yet
- **Do not rewrite off PX4/MAVROS on control grounds.** Re-open only when functional-safety requirements are known. (§6.2)
- **Do not design for marking through GNSS-denied stretches.** Detect and stop. (§4.2)
- **Do not trust any accuracy number in this report as a hardware target.** All of them are simulation or vendor claims. (§3.5)

### Blocking unknowns — resolve before design
1. **Line width tolerance** — refuted, unpinned, and it sizes the nozzle and sets the entire cross-track budget. (§1.4)
2. **Functional safety obligations** for work-zone operation — completely unresearched, and the only real forcing function on stack architecture. (§7)
3. **Which category are you building** — A (port your rover) or B (new machine)? Everything downstream depends on this. (§0)
4. **Target jurisdiction** — India (IRC 35 + MoRTH Cl. 803) is the only regime with verified numbers here. US/EU/AU are unconfirmed. (§1.3)

---

## 10. Research quality — read this before trusting the above

**Coverage was badly uneven against the 8-part question.** Verification produced primary-source evidence for **3 of 8 areas**: Indian regulatory geometry/retroreflectivity, design-file ingestion, and path geometry + controller comparison.

**Five areas returned zero surviving verified claims** — localization, applicator control, ROS 2 stack, backend functions, and safety standards. §4, §5 and §7 above are built from **[E]** extracted claims (source-grounded with quotes, but not adversarially checked). §6 and §8 are largely **[GAP]**.

Specific weaknesses:
- **All US/MUTCD regulatory claims were refuted.** The picture is India-only.
- **Every hardware cm-level accuracy claim was refuted.** Every surviving control number is simulation.
- The ECC 2015 clothoid-MPC result is **explicitly low-speed and 11 years old** — evidence about a formulation, not current SOTA.
- The *Sensors* 2025 comparison is a single tuning-sensitive simulation study; its 2-1 vote reflects real disagreement about Stanley at speed.
- **No commercial-product claim was verified** — Graco, TinyMobileRobots, RoadPrintz, LimnTech, Borum, Hofmann, Titan material in §0/§4/§5/§7 is all **[E]**, much of it from vendor pages.

**Time-sensitivity:** IRC:35-2015 is current (2nd revision, no successor as of 2026-07). IFC 4.3 is current as ISO 16739-1:2024. LandXML 1.2 is frozen but still the working schema. **IFC 4.3 alignment *exporter* maturity is actively changing** — re-check before committing.

**Recommended follow-up research (in priority order):** functional safety for work-zone robots; line-width and cross-track tolerance across MUTCD/EN 1436/RMS; ROS 2 vs PX4 architecture for a safety-rated ground vehicle; whether *any* road-marking robot has demonstrated cm-level accuracy on pavement at working speed.

---

## Sources

**Primary standards / schemas**
- IRC:35-2015 Code of Practice for Road Markings — https://law.resource.org/pub/in/bis/irc/irc.gov.in.035.2015.pdf
- IFC 4.3 `IfcAlignment` (ISO 16739-1:2024) — https://ifc43-docs.standards.buildingsmart.org/IFC/RELEASE/IFC4x3/HTML/lexical/IfcAlignment.htm
- Inframodel 4.0.2 Alignments (LandXML) — https://buildingsmart.fi/en_GB/infra/inframodel/inframodel402/pages/3_Alignments.html
- LandXML 1.2 `spiralType` — https://landxml.org/schema/landxml-1.2/documentation/LandXML-1.2Doc_spiralType.html
- LandXML "Transition curves in Road Design" — http://www.landxml.org/schema/Documentation/Transition%20curves%20in%20Road%20Design.doc
- MUTCD retroreflectivity final rule (2022) — https://www.federalregister.gov/documents/2022/08/05/2022-16781/... ⚠️ claims refuted
- FHWA pavement marking regulations/standards — https://highways.dot.gov/safety/other/visibility/pavement-markings-regulations-standards
- MUTCD Part 3 Markings (secondary) — https://mutcd.info/mutcd-part-3-markings/

**Academic**
- Wang et al., "Dynamic Sampling and Control for Automated Road Pre-Marking Robot", *JACIII* 29(4), 2025 — https://www.fujipress.jp/jaciii/jc/jacii002900040921/
- Lima et al., "Clothoid-based model predictive control for autonomous driving", ECC 2015 — https://ieeexplore.ieee.org/document/7330991/
- Jung, "Model-Based Hybrid Control of Pure Pursuit and Stanley Methods", *Sensors* 25(20):6491, 2025 — https://pmc.ncbi.nlm.nih.gov/articles/PMC12567833/
- HD map + LiDAR + RTK/INS particle-filter localization — https://www.tandfonline.com/doi/full/10.1080/10095020.2024.2377800
- NSGA-II pure-pursuit parameter optimization — https://pmc.ncbi.nlm.nih.gov/articles/PMC11820862/
- Hong et al., "A robotic system for road lane painting", *Automation in Construction* 17(2), 2008 — https://www.sciencedirect.com/science/article/abs/pii/S0926580506001270
- Autonomous highway maintenance vehicle review, *ASCE JCEM* — https://doi.org/10.1061/JCEMD4.COENG-16623

**Patents (applicator engineering)**
- US 8,880,362 B2 — Epic Solutions, pressure-based flow/thickness control (active to 2031)
- US 8,128,313 — EZ Liner, zero-velocity bead dispenser
- US 6,547,158 — two-component paint + co-timed bead gun

**Commercial**
- LimnTech LifeMark-400 — https://limntech.com/lifemark400/
- Advanced Navigation Certus / LimnTech case study — https://www.advancednavigation.com/case-studies/certus-mems-ins-improves-efficiency-of-limntech-scientific-road-marking-solution/
- RoadPrintz — https://roadprintz.com/technology/
- TinyMobileRobots TinySurveyor — https://www.monsenengineering.com/survey/tinysurveyor/road-pre-marking-striping-robot/

**Tooling**
- pyclothoids (MIT) — https://github.com/phillipd94/pyclothoids
