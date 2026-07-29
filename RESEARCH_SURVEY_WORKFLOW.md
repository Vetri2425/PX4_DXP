# Research: Precision Survey Workflow for a Pre-Line Road-Marking Rover

**Date:** 2026-07-27  
**Scope:** How professionals create road-marking survey/layout points, how to push RTK below ~1.5 cm, what agencies require for marking placement, and whether our product’s canonical input should stay surveyed CSV or move to design-file + control.  
**Method:** Web research of primary standards, vendor specs, and commercial prior art. No code changes.  
**Our context (given, not re-derived):** CSV (Emlid Reach RS3 export) → densify to 5 cm → arc-fit → EKF-origin mission → drive/paint. Field: plan ~2 cm of stations; physical track 1.8–3.6 cm/station; survey shots single-epoch Lateral RMS 1.6–1.8 cm — survey is now the floor.

---

## TL;DR

1. **Agency marking placement tolerances are typically ±15–25 mm lateral** (UK / Australia / RSMA) or **~½–2 inch** rate/max limits (US state DOTs). Our rover already beats the **paint-application** specs in many jurisdictions; the tighter real-world constraint is **survey + layout geometry fidelity**, not the paint truck’s ±25 mm clause.
2. **Professionals still set out from design alignments** (Civil 3D / OpenRoads / LandXML), then **premark** with stringline/chalk, RTK stakeout, or layout robots — not from ad-hoc field points alone. Commercial pre-mark robots (TinySurveyor, CivDot Mini) accept **both CSV and DXF**.
3. **Canonical input recommendation: keep surveyed CSV as the production path for as-built / re-line / field-owned geometry; add design DXF/LandXML + field control as a second authoring path** for new construction where the designer owns the alignment. Hybrid site transform is the liability-safe bridge.

---

## Part 1 — How professionals CREATE the survey points

### 1a. How highways / road-marking contractors set out line-marking geometry

**Design → set-out → premark → paint** remains the dominant chain.

| Stage | Typical practice | Notes |
|---|---|---|
| Design | Alignment from Civil 3D / OpenRoads / 12d; plan sheets or digital stakeout files | Centreline / edge-line offsets from design alignment |
| Primary set-out | Robotic total station for structures / kerbs / critical geometry; RTK GNSS for long open corridors | UK highways practice: TS for mm work, RTK ~8–15 mm claimed for earthworks/kerbs when network RTK is good ([AKN Engineering summary](https://www.aknengineering.co.uk/blog/what-is-the-most-accurate-equipment-for-setting-out-large-highways-projects) — **contractor blog, not a standard**) |
| Premark / spotting | Paint spots, chalk, stringline, or GPS-guided layout marks that guide the striper | Ohio DOT: longitudinal premarks at **40 ft (12 m)** intervals, ≤2″ wide × ≤12″ long, offset ≤1″ from theoretical edge ([ODOT CMS 641.06](https://www.dot.state.oh.us/Divisions/ConstructionMgt/OnlineDocs/Specifications/2010CMS/640/641.htm)) |
| Paint | Self-propelled striper follows premark / existing line / joint | Layout robots increasingly replace stringline for safety ([LimnTech Vontz case study](https://limntech.com/wp-content/uploads/2025/05/Vontz-Case-Study-formatted.pdf) — NDOT CSV → GPS layout; **marketing case study**) |

**Point spacing and codes (what is documented vs. field custom):**

- **Longitudinal lines (premark):** agency-documented spacing is often **~40 ft / 12 m** (Ohio). That is a **guide for the paint truck**, not a survey-station density for cm path following.
- **Curves:** traditional stakeout uses shorter chords on sharper curves (US Navy/Seabee guidance: 100 ft chords for mild curves down to 10 ft for sharp; allowable arc–chord discrepancy ~0.02 ft per 100 ft) — see [NAVSEA construction surveying curves](https://engineeringtraining.tpub.com/14071/css/Solving-And-Laying-Out-A-Simple-Curve-246.htm). Modern GNSS stakeout typically uses **coordinate stakeout of design stations** (e.g. 5–25 m) rather than deflection-angle chords ([WSDOT Highway Surveying Manual Ch.11](https://wsdot.wa.gov/publications/manuals/fulltext/m22-97/chapter11.pdf)).
- **Symbols / words / arrows:** templates or schematic forms; premark from plans (Ohio 641.06). Not sparse polylines.
- **Codes:** not standardized globally for road marking. Surveyors use local point codes (CL, EL, LL, STOP, etc.). Our Emlid CSV `Name`/`Code` columns match that culture; agencies rarely mandate a universal code dictionary for striping layout.

**Stringlines:** still common for parking lots / short jobs; on highways they are being displaced by RTK stakeout and GPS layout systems because of **worker exposure to traffic** ([LimnTech / contractor case studies](https://limntech.com/wp-content/uploads/2025/05/Vontz-Case-Study-formatted.pdf)).

**Unverified:** a universal “industry standard point spacing for survey stations on marking jobs” beyond agency premark intervals. Field crews densify curves by judgment or by exporting denser stations from CAD.

---

### 1b. Survey-grade capture methods — ranked by accuracy vs cost

Numbers below mix **manufacturer RMS specs** (optimistic, 1σ / RMS under good conditions) and **agency practice requirements**. Spec ≠ field absolute accuracy.

| Rank (best →) | Method | Published / practiced accuracy | Relative cost / productivity | When it makes sense for road marking |
|---|---|---|---|---|
| 1 | **Robotic total station + prism** (or RTS-tracked robot) | Angular/distance instrument class: mm at short range; HP SitePrint quotes **±2 mm layout** with 1″ RTS under constrained indoor/near setup ([HP SitePrint](https://www.hp.com/us-en/printers/site-print/layout-robot.html) — **vendor claim, RTS-dependent**) | High crew skill; line-of-sight; slow on long corridors | Intersections, symbols, canopy/urban canyons, indoor slabs; QA check of RTK |
| 2 | **Static / fast-static GNSS (post-processed)** | Emlid RS3: Static **H 4 mm + 0.5 ppm**, V 8 mm + 1 ppm ([Emlid RS3](https://emlid.com/reachrs3/)); Trimble R12i high-precision static **H 3 mm + 0.1 ppm** ([Trimble R12i datasheet](https://trl.trimble.com/docushare/dsweb/Get/Document-950413/Datasheet%20-%20Trimble%20R12i%20GNSS%20System%20-%20English%20(USL)%20-%20Screen.pdf)) | Slow; needs office processing | Control densification, base coordinates, liability-critical monuments — **not** every paint station |
| 3 | **PPK (stop-and-go / kinematic post-process)** | Emlid RS3 PPK **H 5 mm + 0.5 ppm** ([Emlid](https://emlid.com/reachrs3/)); typically similar or slightly better than RTK when base is well known | Same field time as RTK + office; no radio dependency | Remote corridors with poor NTRIP; audit trail; QA of RTK campaign |
| 4 | **Network RTK / single-base RTK with multi-epoch occupation** | Vendor RTK: Emlid **H 7 mm + 1 ppm**; Trimble R12i single-baseline **H 8 mm + 1 ppm**, network **H 8 mm + 0.5 ppm** ([Emlid](https://emlid.com/reachrs3/), [Trimble](https://trl.trimble.com/docushare/dsweb/Get/Document-950413/Datasheet%20-%20Trimble%20R12i%20GNSS%20System%20-%20English%20(USL)%20-%20Screen.pdf)). NGS cites industry RT specs ~**1 cm + 1 ppm H / 2 cm + 1 ppm V (1σ)** ([NGS RT guidelines v2.1](https://www.ngs.noaa.gov/PUBS_LIB/NGSRealTimeUserGuidelines.v2.1.pdf)) | Fast; one-person | **Primary tool for long road corridors** when sky is open |
| 5 | **RTK single-epoch / few-second shot (hand-held pole)** | Same vendor RMS *if* FIX + level + no multipath — but field RMS often **1–3 cm H** once pole tilt, bipod absence, and multipath are included. Our own single-epoch Lateral RMS **1.6–1.8 cm** matches this band. | Fastest | Dense topo / premark guides where agency paint tolerance is ±25–50 mm |

**Cost intuition (order of magnitude, not quotes):** Reach-class RTK kit ~$3k/rover; survey-grade Trimble/Leica RTK ~$15–40k+; robotic TS ~$20–60k+. Layout robots (TinySurveyor / CivDot) sit above the rover kit and below full striping-truck automation.

**Road-marking fit:** for **cm-level outdoor long corridors**, **network or local-base RTK** is the workhorse; use **RTS** for symbols/intersections/obstructions; use **static/PPK** to pin the control frame, not every station.

---

### 1c. Best practice to push per-point accuracy **below 1.5 cm** with an RTK rover pole

Goal: **horizontal absolute or relative ≤ 15 mm** at 95% (or at least consistently under 15 mm RMS). Vendor RMS of 7–8 mm does **not** guarantee sub-1.5 cm field points without procedure.

#### Concrete numbers from vendor docs

| Source | Number | What it is |
|---|---|---|
| [Emlid Reach RS3](https://emlid.com/reachrs3/) | RTK H **7 mm + 1 ppm**; Static H **4 mm + 0.5 ppm**; PPK H **5 mm + 0.5 ppm** | Manufacturer RMS under good conditions |
| [Emlid RS3 tilt](https://emlid.com/reachrs3/) | Tilt add-on **RTK + 2 mm + 0.3 mm/°**; marketing “20 mm up to 60° tilt” | **Turn tilt OFF** for sub-1.5 cm control shots unless pole is nearly plumb |
| [Trimble R12i](https://trl.trimble.com/docushare/dsweb/Get/Document-950413/Datasheet%20-%20Trimble%20R12i%20GNSS%20System%20-%20English%20(USL)%20-%20Screen.pdf) | RTK H **8 mm + 1 ppm** (single baseline); network **8 mm + 0.5 ppm**; TIP tilt **RTK + 3 mm + 0.15 mm/°** (≤40°, FW ≥6.43) | Older datasheets show TIP **+5 mm + 0.4 mm/° ≤30°** — **sources disagree by firmware era** |
| [Leica GS18 T](https://leica-geosystems.com/en-us/products/gnss-systems/smart-antennas/leica-gs18-t/store-tilt-functionality-gs18-t) | 2 m pole at **20′ bubble** → **~1 cm** tip error if bubble-limited | Explains why bipod + averaging matters even without tilt IMU |
| ppm term | +1 mm per km of baseline (at 1 ppm) | Keep base/NTRIP effective baseline short; prefer local base or nearby CORS |

#### Agency / NGS procedural gates (actionable)

From [NGS User Guidelines for Classical Real Time Positioning v2.1](https://www.ngs.noaa.gov/PUBS_LIB/NGSRealTimeUserGuidelines.v2.1.pdf) (primary):

- Manufacturer RT specs ~**1 cm + 1 ppm H** (1σ); treat as **precision relative to a correct base**, not absolute datum accuracy.
- For **high-accuracy** work (~**1–2 cm @ 95%**): **≥2 redundant occupations** at **staggered geometry** (guideline text: ~**3–4 hour** stagger), average after discarding outliers; **~180 epochs** (~3 min at 1 Hz) for higher class; **PDOP ≤ 2.0**, **≥7 sats**, solution RMS **≤ 0.01 m**; **fixed-height pole + bipod/tripod**; level before recording.
- For general topo (~few cm): shorter occupations (e.g. **10–15 epochs**) with steady pole; bipod preferred but not always required.

From [WisDOT FDM 9-30 RTK guidelines](https://wisconsindot.gov/rdwy/fdm/fd-09-30.pdf) (primary DOT):

| Application | Desired accuracy (95%) H | Min epochs / observation set |
|---|---|---|
| Primary control | **±0.05 ft (1.5 cm)** | **300 epochs (~5 min)** |
| Secondary control | **±0.05 ft (1.5 cm)** H | **180 epochs (~3 min)** |
| General topo | looser | shorter |

From [Caltrans GPS Survey Specs (LS Manual Ch.6)](https://dot.ca.gov/-/media/dot-media/programs/right-of-way/documents/ls-manual/06-surveys-a11y.pdf):

- RTK observation example gates: **≥30 epochs**, PDOP ≤5, H precision of measurement **≤ 0.03 ft (~9 mm)** (and other session rules for control vs topo).

From Orange County survey field services RTK chapter ([OC GIS Ch.2 RTK](https://www.ocgis.com/documents/OCSurveyFieldServices/Chapter%202%20-%20RTK%20GPS.pdf)):

- New control: **2 independent occupations**, **≥2 hour** time differential; each **180 epochs** or **3×60 epochs**.

#### Practical recipe to beat 1.5 cm (for our Emlid workflow)

1. **Base / datum:** known control or Average FIX from NTRIP for **10–20 minutes** on tripod ([Emlid Average FIX](https://docs.emlid.com/reachrs3/base-setup/setting-up-base-over-known-point/averaged-in-fix/), [Emlid base guide](https://blog.emlid.com/the-ultimate-guide-to-setting-up-a-gnss-base-for-centimeter-accuracy/)). Never Average SINGLE if absolute georeferencing matters.
2. **Rover mount:** fixed-height pole + **bipod**; bubble checked; **tilt compensation OFF** for control-grade shots (tilt adds mm-to-cm).
3. **Gates:** FIX only; reject FLOAT; monitor HRMS/VRMS; prefer HRMS **≪ 1.5 cm** before storing (Caltrans-style internal precision gate ~9 mm is a good target).
4. **Occupation:** for stations that define the paint geometry, **≥60–180 s averaging** (topo vs control); for **control / check points**, use WisDOT/NGS **3–5 min** and **repeat later**.
5. **Repeat & average:** 2 occupations, **≥2–4 h apart**, average if residuals within tolerance; discard outliers (NGS).
6. **Baseline:** shorten effective baseline (local base on site control); every **10 km** at 1 ppm adds **~1 cm** to the error budget.
7. **Environment:** open sky, avoid multipath (guardrails, trucks, building façades); NGS flags multipath as a silent vertical/horizontal killer.

**ASCE study note:** [Impact of Rover Pole Holding on RTK-GNSS](https://doi.org/10.1061/(asce)su.1943-5428.0000404) found bipod vs hand-held differences often **within satellite survey noise** for Trimble R8s under their test — i.e. bipod is still best practice, but **multipath, FIX quality, and base error dominate**.

**Honest floor:** with Emlid-class RTK and excellent procedure, **sub-1.5 cm @ 95%** is achievable for **relative** layout on a short baseline; claiming **absolute** sub-1.5 cm everywhere without redundant occupations + known control is **not** supported by NGS/DOT practice docs. Our current **1.6–1.8 cm single-epoch Lateral RMS** is expected for short occupations; averaging + bipod + repeats is the path down.

---

### 1d. What DOTs / highways agencies REQUIRE — the number we must beat

These are **installation / placement** tolerances for the finished marking (or its spotting), not GNSS survey standards. They define the **acceptance envelope** a paint contractor must meet.

| Jurisdiction | Spec | Lateral / alignment tolerance | Source |
|---|---|---|---|
| **UK (National Highways SHW)** | Series 1200 / Clause on permanent road markings | Longitudinal markings: lateral **±25 mm** from designed position | [MCHW Vol 1](https://www.standardsforhighways.co.uk/tses/attachments/31e1eb76-5906-45e5-bfa9-f024326cf9ef) |
| **UK industry (RSMA)** | STANSPEC 2022 | Lane lines lateral **±25 mm**; also dimensional % tolerances on length/width | [RSMA STANSPEC 2022](https://www.rsma.co.uk/wp-content/uploads/2024/02/20220704-Stanspec-2022-FINAL.pdf) |
| **UK Traffic Signs Manual Ch.5** | Design guidance | “A tolerance of **±25 mm** is normally allowed in the lateral positioning of lane lines” | [TSM Chapter 5](https://assets.publishing.service.gov.uk/government/uploads/system/uploads/attachment_data/file/773421/traffic-signs-manual-chapter-05.pdf) §12.6 |
| **Queensland TMR** | MRTS45 | New longitudinal: **±15 mm** from correct alignment/position; straightness 5 mm in 2000 mm | [MRTS45](https://www.tmr.qld.gov.au/-/media/busind/techstdpubs/Specifications-and-drawings/Specifications/3-Roadworks-Drainage-Culverts-and-Geotechnical/MRTS45.pdf) |
| **Western Australia Main Roads** | Spec 604 | Same band: **±15 mm** alignment/position for new markings | [Spec 604](https://www.mainroads.wa.gov.au/globalassets/technical-commercial/technical-library/specifications/600-series-traffic-facilities/specification-604-pavement-marking.pdf) |
| **City of Sydney** | Streets Tech Spec B9 | Locations **≤20 mm** from drawings; lines H **±20 mm** | [Sydney Streets B9](https://www.cityofsydney.nsw.gov.au/-/media/corporate/files/publications/design-codes-technical-specifications/sydney-streets-technical-specifications-2025/construction/b9-road-pavement-marking-and-road-signage-construction---rev-7.pdf) |
| **Ohio DOT** | CMS 641.07 | Lateral deviation rate **≤2″ per 100′ (50 mm / 30 m)**; **max 3″ (75 mm)** | [ODOT 641](https://www.dot.state.oh.us/Divisions/ConstructionMgt/OnlineDocs/Specifications/2010CMS/640/641.htm) |
| **NCDOT** | Div 12 / 1205 | Lateral deviation from proposed alignment **≤½″ (12.7 mm)** at any point | [NCDOT 1205](https://connect.ncdot.gov/projects/construction/ConstManRefDocs/1205,%202012%20Standard%20Specifications.pdf) |
| **TxDOT / common Item 666 language** | Prefabricated / reflectorized markings practice | Deviation rate **≤1″ per 200′**; **max 2″**; no abrupt deviation | [Example Item 666](https://www.deerparktx.gov/DocumentCenter/View/13376/Spec-666-Prefrabricated-Pavement-Markings); TxDOT handbook points to Item 666.4.1 ([TxDOT PMH](https://www.txdot.gov/manuals/trf/pmh/installation_and_inspection/preinstallation_inspections-i1003348/lateral_placement_guides_for_new_pavement_surfaces.html)) |

**What our rover must beat:**

| Use of number | Value | Interpretation |
|---|---|---|
| **Common international paint acceptance** | **±15 to ±25 mm** | Queensland/WA (±15) and UK (±25) — good product targets |
| **Strict US outlier** | **≤12.7 mm** (NCDOT ½″) | Harder; rare among surveyed US specs |
| **Loose US rate/max specs** | 2″/100′ rate, 3″ max (Ohio); 1″/200′, 2″ max (TxDOT-style) | Easy for a cm rover; not a useful stretch goal |
| **Recommended product bar** | **≤15 mm lateral RMS to design or to surveyed stations** | Beats UK ±25 and matches AU ±15 with margin for spray width |

**Caveat:** these tolerances are usually **relative to the approved layout / drawings / premark**, not relative to a GNSS absolute frame. If the survey/layout is wrong, a “within tolerance” paint job can still be in the wrong place. Liability sits on **who owns the geometry**.

**AS/NZS:** Austroads notes **no single national installation tolerance** — states write their own (AS 1742 covers devices/marking *design*, not always install tolerance) ([Austroads AP-R578-18](https://austroads.gov.au/publications/asset-management/ap-r578-18/media/AP-578-18_Harmonisation_of_Pavement_Markings_and_National_Pavement_Marking_Specification.pdf)). Use state specs (MRTS45, Spec 604) as cited.

---

## Part 2 — From points to drivable path: authoring tool

### 2a. Pipeline comparison

| | **(A) Field-surveyed CSV → densifier** *(today)* | **(B) Design CAD/DXF → georef → rover** | **(C) Hybrid: design alignment + field control** |
|---|---|---|---|
| Who authors geometry | Field surveyor / striping contractor | Design engineer (Civil 3D / OpenRoads) | Designer authors; surveyor pins frame |
| Truth source | Physical marks / measured stations | Design intent | Design intent in a **field-validated** CRS |
| Curve fidelity | Depends on station spacing + densifier/arc-fit | Native arcs/spirals if DXF/LandXML keep them | Same as B after transform |
| Failure mode | Good paint of a **wrong/biased survey** | Perfect paint of **design that doesn’t match pavement** (mill/overlay shift) | Bad control → wrong Helmert → whole job shifts |
| Re-survey cost | High if many stations | Low field if control exists; high if design wrong | Medium — control only |
| Best for | Re-line, as-built match, parking lots, “paint what we shot” | New construction with machine-control / BIM | New roads + mill-and-fill with known control |

**What commercial layout robots use:**

| Product | Input | Positioning | Claimed accuracy | Notes |
|---|---|---|---|---|
| **TinyMobileRobots TinySurveyor** (Plotter/Terra) | **CSV + DXF** | Customer GNSS or total station | **1–2 cm GNSS**; **~10 mm with TS** ([tinysurveyor.com](https://tinysurveyor.com/plotter/), [Monsen](https://www.monsenengineering.com/survey/tinysurveyor/road-pre-marking-striping-robot/), [Geometius PDF](https://www.geometius.nl/wp-content/uploads/2024/06/TinyMobileRobots-Intro-TinySurveyor-Terra-Plotter.pdf)) | Closest peer to our product; highway pre-mark case studies |
| **Civ Robotics CivDot Mini** | **DXF for lines**, **CSV for points** | RTK + dual heading + IMU | **Sub-inch / ~2–2.5 cm** ([Civ FAQ](https://www.civrobotics.com/faq), [CivDot Mini](https://www.civrobotics.com/robots/layout-robots/civdot-mini)) | Explicitly roads / parking / airports; site calibration supported |
| **CivDot / CivDot+** | CSV points (DXF lines in CivPlan) | RTK | **~30 mm** spray / **~8 mm** laser+ ([Civ FAQ](https://www.civrobotics.com/faq)) | Point layout more than continuous striping |
| **HP SitePrint** | **2D DXF** (+ plugin), control points in file | **Robotic total station** + prism | **±2 mm** layout under stated RTS/high-accuracy mode ([HP](https://www.hp.com/us-en/printers/site-print/layout-robot.html)) | Indoor/construction floors — **not** highway GNSS peer; hybrid = design + control |
| **Trimble FieldLink** (+ RTS/GNSS) | **DWG/DXF/IFC/SKP/…**, model-based layout | RTS and/or GNSS | Instrument-class (not a paint robot) | Design-to-field; [FieldLink](https://www.trimble.com/en/products/building-construction-field-systems/fieldlink-software) |
| **LimnTech LifeMark** | CSV / recorded / design coords (contractor cases) | Vehicle RTK + vision | “Centimeter-level” ([LimnTech](https://limntech.com/lifemark400/)) — **marketing** | Truck-mounted layout/restripe assist |
| **Traqnology Stripetraq** | In-app, CAD upload, survey stick, drone | RTK | Claims **1 cm / sub-inch** ([Traqnology](https://www.traqnology-na.com/pavement/)) — **marketing** | Graco/UTV kits; sports + pavement |
| **TrackMaster / “graden”** | — | — | — | **Not verified** under that name in public sources (see §Unverified) |

**Pattern:** commercial pre-mark robots that compete with us are **(A)+(B) dual-input**. Interior layout robots (HP) are **(C)** with RTS. Almost nobody is CSV-only or DXF-only in this niche.

---

### 2b. Civil alignments, LandXML, IFC — what a chord CSV loses

#### How curves are represented

| Format | Straight | Circular arc | Clothoid / spiral | Other |
|---|---|---|---|---|
| **Civil 3D alignment** | Native | Native | Native spirals | PI-based design |
| **LandXML** | `CoordGeom` lines | curves | **spirals / transitions** (Euler clothoid); **not** b-splines / Archimedes ([Civil 3D LandXML support](https://help.autodesk.com/cloudhelp/2025/ENU/Civil3D-UserGuide/files/GUID-4D10ABA5-5EA0-41A8-BB61-C3F446CE7C6B.htm); [Bentley note](https://bentleyopencivil.ideas.aha.io/ideas/BCI-I-261)) | OpenRoads exports LandXML / IFC alignment ([OpenRoads Export Geometry](https://docs.bentley.com/LiveContent/web/OpenRoads%20Designer-v2025.1/Help/en/topics/405971/GUID-2594492C-C48C-4BF7-836A-E5D282D3A79A.html)) |
| **IFC 4.3 alignment** | `LINE` | `CIRCULARARC` | `CLOTHOID`, `CUBIC`, plus Bloss/Helmert/sine/… ([buildingSMART IFC4.3](https://standards.buildingsmart.org/IFC/RELEASE/IFC4_3/HTML/lexical/IfcAlignmentHorizontalSegmentTypeEnum.htm)) | Richer transition set than LandXML |
| **DXF** | LINE | ARC / LWPOLYLINE bulge | Often **approximated** as polylines unless custom entities | Depends on exporter; spirals frequently tessellated |
| **CSV stations** | Implied between points | Lost unless reconstructed | Lost | Only samples |

#### Chord / sample spacing vs 2.5 cm mid-ordinate error

For a circular arc of radius \(R\), chord length \(C\), mid-ordinate (sagitta) ≈ \(C^2 / (8R)\) (small-angle).

Require \(M \le 0.025\,\mathrm{m}\):

\[
C \lesssim \sqrt{0.2\,R}
\]

| Radius \(R\) | Max chord \(C\) for \(M\le 2.5\) cm |
|---|---|
| 30 m (very tight) | ~2.4 m |
| 50 m | ~3.2 m |
| 100 m | ~4.5 m |
| 200 m | ~6.3 m |
| 500 m | ~10 m |
| 1000 m | ~14 m |

**Interpretation for us:**

- Sparse survey stations every **10–20 m** on a **tight** curve can leave **>2.5 cm** geometric error *even if each station is perfect* — unless the densifier **arc-fits** (which we do). Arc-fit recovers circular arcs; it does **not** recover **clothoids** (curvature ≠ constant).
- On highway-scale radii (hundreds of metres), **5–10 m** sampling is often enough for 2.5 cm circular mid-ordinate; **clothoid entry/exit** still needs design parameters or denser samples.
- **5 cm densification** (our pipeline) is far denser than needed for circular mid-ordinate on typical road radii — survey noise dominates geometry tessellation error.

**DXF risk:** if Civil exports spirals as dense polylines, fidelity depends on export chord tolerance. Prefer **LandXML / IFC alignment** when spiral fidelity matters.

---

### 2c. Commercial prior art (detail)

#### TinyMobileRobots — TinySurveyor Plotter / Terra
- **Input:** CSV and DXF via tablet/USB ([Plotter](https://tinysurveyor.com/plotter/), [Monsen road pre-mark](https://www.monsenengineering.com/survey/tinysurveyor/road-pre-marking-striping-robot/)).
- **Authoring:** existing survey/CAD files; operator adjusts projection shift, speed, spray on tablet.
- **Accuracy:** marketed **1–2 cm GNSS**; **10 mm with total station** ([Geometius intro PDF](https://www.geometius.nl/wp-content/uploads/2024/06/TinyMobileRobots-Intro-TinySurveyor-Terra-Plotter.pdf)). Treat as **vendor claim**; GNSS number matches RTK class.
- **Curves:** DXF geometry (arcs/polylines as provided); customer anecdote of arc marks every ~3 m ([Terra page](https://tinysurveyor.com/terra/)).

#### HP SitePrint
- **Input:** prepared **2D DXF** with printables + obstacles + **control points** ([HP FAQ](https://www.hp.com/us-en/printers/site-print/layout-robot/faq.html)).
- **Authoring:** BIM/CAD → HP plugin → robot-ready DXF.
- **Accuracy:** **±2 mm** average layout under RTS + SMR prism + high-accuracy mode, 5–30 m ([HP SitePrint](https://www.hp.com/us-en/printers/site-print/layout-robot.html)) — **conditional vendor claim**.
- **Curves:** CAD arcs/lines printed; positioning is **RTS**, not GNSS corridor RTK.

#### Civ Robotics
- **Input:** CSV (points), DXF (lines); local / state plane + site calibration ([FAQ](https://www.civrobotics.com/faq)).
- **CivDot Mini:** continuous lines/curves/dashes; **~2 cm / sub-inch** spray ([CivDot Mini](https://www.civrobotics.com/robots/layout-robots/civdot-mini)).
- **Curves:** DXF line work; sensor fusion (dual RTK heading + IMU); won’t mark without FIX.

#### Trimble FieldLink (+ robots / RTS)
- **Input:** models/drawings including DXF/DWG/IFC etc. ([Trimble FieldLink](https://www.trimble.com/en/products/building-construction-field-systems/fieldlink-software)).
- **Role:** contractor layout from design — closer to **stakeout OS** than paint robot.
- **Accuracy:** instrument-limited (RTS mm-class indoors; GNSS cm outdoors).

#### TrackMaster / Graden
- **Not found** as a distinct public product matching “TrackMaster/graden line markers.” Possible confusion with **Traqnology** (GPS kits for Graco LineLazer / UTV), **LimnTech LifeMark**, or regional OEM names. **Flagged unverified.**

---

### 2d. Recommendation for OUR product

#### Decision

**Keep surveyed CSV + densifier as the canonical production path for v1/field reality; add design-file (DXF first, LandXML next) + field control points as a parallel path — do not abandon CSV.**

#### Decision criteria

| Criterion | Winner | Why |
|---|---|---|
| Who authors geometry today (our customers) | **CSV** | Emlid export / field surveyors already in the loop |
| Who authors on new highway jobs | **Design file** | Engineers own Civil alignments; robots in market ingest DXF |
| Curve fidelity (clothoids) | **LandXML/IFC > DXF > CSV** | CSV+arc-fit OK for circular; spirals need parametric import |
| Liability for wrong-place marks | **Hybrid (C)** | Design intent + surveyed control separates “wrong design” from “wrong frame” |
| Re-survey cost | **B/C** for long corridors | Reshoot control, not every station |
| Match commercial peers | **Both** | TinySurveyor + CivDot already CSV+DXF |
| Beat agency ±15–25 mm | **Either**, if survey ≤~1 cm and tracking ≤~2 cm | Our tracking already in band; survey is the floor |

#### Migration path (“both”)

1. **Now — harden CSV path (A)**  
   - Document occupation SOP to push survey HRMS toward **≤1.0–1.2 cm** (averaging, bipod, FIX-only, tilt off, repeats).  
   - Keep 5 cm densify + arc-fit; treat sparse stations as **control polyline**, not the drive path.

2. **Next — DXF import (B lite)**  
   - Accept 2D DXF in a documented CRS; tessellate ARC/LWPOLYLINE at mid-ordinate **≤1 cm** (tighter than paint tol).  
   - Require operator confirmation of CRS / units (classic DXF foot/metre footguns).

3. **Then — field control / site transform (C)**  
   - Import ≥3–4 **surveyed control points** also present in the design file; estimate 2D similarity/Helmert; report residuals; refuse job if residual > threshold (e.g. **10–15 mm RMS**).  
   - This mirrors HP SitePrint “control in the DXF” and CivPlan site calibration.

4. **Later — LandXML / IFC alignment**  
   - When customers demand true clothoids from OpenRoads/Civil; densify parametrically instead of arc-fitting chords.

5. **Product positioning**  
   - **CSV mode:** “Paint what you surveyed” (re-line, as-built).  
   - **Design+control mode:** “Paint the design on the ground frame you proved.”  
   - Liability copy: geometry author vs frame author called out in the UI/report.

---

## Comparison tables (quick reference)

### Capture methods vs road-marking use

| Method | Typical H accuracy (good conditions) | Cost | Corridor productivity | Best use |
|---|---|---|---|---|
| RTS + prism | mm–few mm | High | Low | Symbols, canopy, QA |
| Static GNSS | ~5 mm + ppm | Med | Very low | Control |
| PPK | ~5–10 mm + ppm | Med | Med | No radio / audit |
| RTK multi-epoch | ~1–1.5 cm @95% with procedure | Med | High | Station control |
| RTK single-epoch | ~1.5–3 cm field | Low–med | Highest | Dense guides |

### Agency lateral marking tolerances (finished work)

| Spec family | Lateral |
|---|---|
| AU state (QLD/WA) | **±15 mm** |
| UK SHW / RSMA / TSM | **±25 mm** |
| NCDOT | **≤12.7 mm** |
| Ohio | 2″/100′ rate, **3″ max** |
| TxDOT-style Item 666 | 1″/200′, **2″ max** |

### Authoring pipelines

| Pipeline | Peer products | Keep? |
|---|---|---|
| CSV survey → densify | TinySurveyor, CivDot points, our today | **Yes — primary** |
| DXF design → robot | TinySurveyor, CivDot Mini, HP, FieldLink | **Yes — add** |
| Design + control transform | HP SitePrint, Civ site calibration, machine control | **Yes — add for liability** |

---

## Unverified / conflicting / gaps

| Item | Status |
|---|---|
| **TrackMaster / Graden line markers** | No reliable public match found; do not cite as prior art until identified |
| TinySurveyor / Civ / Traqnology / LimnTech **accuracy claims** | Vendor marketing unless independently tested; GNSS “1–2 cm” is plausible RTK-class, not proven for every site |
| AKN “8–15 mm RTK” for UK highways | Contractor blog, not a standard |
| Trimble TIP tilt specs | **3 mm + 0.15 mm/°** (newer FW) vs **5 mm + 0.4 mm/°** (older datasheets) — both appear in Trimble materials |
| Universal premark station spacing for **survey** (vs Ohio 40 ft paint premark) | Not standardized; curve densification is practice-dependent |
| Whether customers will supply clean LandXML vs broken DXF polylines | Unknown — needs customer interviews |
| AS 1742 install lateral tolerance | Not a single national install number; use state specs |
| Exact TxDOT 2024 Item 666 PDF text | Cited via municipal republications + TxDOT handbook pointer; prefer official TxDOT standard specs book for contract work |

---

## Sources

1. Ohio DOT CMS Item 641 — Layout, Premarking, Line Placement Tolerance — https://www.dot.state.oh.us/Divisions/ConstructionMgt/OnlineDocs/Specifications/2010CMS/640/641.htm  
2. NCDOT Standard Specifications Division 12 / 1205 — https://connect.ncdot.gov/projects/construction/ConstManRefDocs/1205,%202012%20Standard%20Specifications.pdf  
3. UK MCHW Volume 1 (Specification for Highway Works) — longitudinal marking lateral ±25 mm — https://www.standardsforhighways.co.uk/tses/attachments/31e1eb76-5906-45e5-bfa9-f024326cf9ef  
4. RSMA STANSPEC 2022 — https://www.rsma.co.uk/wp-content/uploads/2024/02/20220704-Stanspec-2022-FINAL.pdf  
5. UK Traffic Signs Manual Chapter 5 — https://assets.publishing.service.gov.uk/government/uploads/system/uploads/attachment_data/file/773421/traffic-signs-manual-chapter-05.pdf  
6. Queensland TMR MRTS45 Road Surface Delineation — https://www.tmr.qld.gov.au/-/media/busind/techstdpubs/Specifications-and-drawings/Specifications/3-Roadworks-Drainage-Culverts-and-Geotechnical/MRTS45.pdf  
7. Main Roads WA Specification 604 Pavement Marking — https://www.mainroads.wa.gov.au/globalassets/technical-commercial/technical-library/specifications/600-series-traffic-facilities/specification-604-pavement-marking.pdf  
8. City of Sydney Streets Technical Specifications B9 — https://www.cityofsydney.nsw.gov.au/-/media/corporate/files/publications/design-codes-technical-specifications/sydney-streets-technical-specifications-2025/construction/b9-road-pavement-marking-and-road-signage-construction---rev-7.pdf  
9. Austroads AP-R578-18 Harmonisation of Pavement Markings — https://austroads.gov.au/publications/asset-management/ap-r578-18/media/AP-578-18_Harmonisation_of_Pavement_Markings_and_National_Pavement_Marking_Specification.pdf  
10. TxDOT Pavement Marking Handbook — lateral placement guides / Item 666 — https://www.txdot.gov/manuals/trf/pmh/installation_and_inspection/preinstallation_inspections-i1003348/lateral_placement_guides_for_new_pavement_surfaces.html  
11. Example Item 666 construction methods (municipal republication of TxDOT-style language) — https://www.deerparktx.gov/DocumentCenter/View/13376/Spec-666-Prefrabricated-Pavement-Markings  
12. NGS User Guidelines for Classical Real Time Positioning v2.1 — https://www.ngs.noaa.gov/PUBS_LIB/NGSRealTimeUserGuidelines.v2.1.pdf  
13. WisDOT Facilities Development Manual 9-30 (RTK guidelines) — https://wisconsindot.gov/rdwy/fdm/fd-09-30.pdf  
14. Caltrans Land Surveys Manual Ch.6 GPS Survey Specifications — https://dot.ca.gov/-/media/dot-media/programs/right-of-way/documents/ls-manual/06-surveys-a11y.pdf  
15. Orange County Survey Field Services — Chapter 2 RTK GPS — https://www.ocgis.com/documents/OCSurveyFieldServices/Chapter%202%20-%20RTK%20GPS.pdf  
16. Emlid Reach RS3 product / specs — https://emlid.com/reachrs3/  
17. Emlid Averaging base in FIX — https://docs.emlid.com/reachrs3/base-setup/setting-up-base-over-known-point/averaged-in-fix/  
18. Emlid Choosing base setup method — https://docs.emlid.com/reachrs3/base-setup/choosing-base-setup-method/  
19. Emlid tilt compensation guide — https://docs.emlid.com/reachrs3/rtk-quickstart/reachrs3-as-rover/tilt-compensation/  
20. Emlid base setup blog (10–20 min Average FIX) — https://blog.emlid.com/the-ultimate-guide-to-setting-up-a-gnss-base-for-centimeter-accuracy/  
21. Trimble R12i datasheet — https://trl.trimble.com/docushare/dsweb/Get/Document-950413/Datasheet%20-%20Trimble%20R12i%20GNSS%20System%20-%20English%20(USL)%20-%20Screen.pdf  
22. Trimble R12i Customer FAQs (TIP) — https://trl.trimble.com/docushare/dsweb/Get/Document-951881/Trimble%20R12i%20Customer%20FAQs.pdf  
23. Leica GS18 T — store tilt / bubble error note — https://leica-geosystems.com/en-us/products/gnss-systems/smart-antennas/leica-gs18-t/store-tilt-functionality-gs18-t  
24. ASCE — Impact of Rover Pole Holding on RTK-GNSS — https://doi.org/10.1061/(asce)su.1943-5428.0000404  
25. Autodesk Civil 3D — Supported LandXML data — https://help.autodesk.com/cloudhelp/2025/ENU/Civil3D-UserGuide/files/GUID-4D10ABA5-5EA0-41A8-BB61-C3F446CE7C6B.htm  
26. Bentley OpenRoads — Export Geometry (LandXML / IFC) — https://docs.bentley.com/LiveContent/web/OpenRoads%20Designer-v2025.1/Help/en/topics/405971/GUID-2594492C-C48C-4BF7-836A-E5D282D3A79A.html  
27. Bentley idea — LandXML clothoid vs b-spline limits — https://bentleyopencivil.ideas.aha.io/ideas/BCI-I-261  
28. buildingSMART IFC 4.3 — IfcAlignmentHorizontalSegmentTypeEnum — https://standards.buildingsmart.org/IFC/RELEASE/IFC4_3/HTML/lexical/IfcAlignmentHorizontalSegmentTypeEnum.htm  
29. WSDOT Highway Surveying Manual Ch.11 (curves) — https://wsdot.wa.gov/publications/manuals/fulltext/m22-97/chapter11.pdf  
30. NAVSEA / tpub — Solving and laying out a simple curve (chord lengths) — https://engineeringtraining.tpub.com/14071/css/Solving-And-Laying-Out-A-Simple-Curve-246.htm  
31. TinySurveyor Plotter — https://tinysurveyor.com/plotter/  
32. TinySurveyor Terra — https://tinysurveyor.com/terra/  
33. Monsen — TinySurveyor road pre-marking — https://www.monsenengineering.com/survey/tinysurveyor/road-pre-marking-striping-robot/  
34. Geometius TinyMobileRobots intro PDF — https://www.geometius.nl/wp-content/uploads/2024/06/TinyMobileRobots-Intro-TinySurveyor-Terra-Plotter.pdf  
35. Civ Robotics FAQ — https://www.civrobotics.com/faq  
36. CivDot Mini — https://www.civrobotics.com/robots/layout-robots/civdot-mini  
37. Civ Robotics roads solution — https://www.civrobotics.com/solutions/roads  
38. HP SitePrint — https://www.hp.com/us-en/printers/site-print/layout-robot.html  
39. HP SitePrint FAQ — https://www.hp.com/us-en/printers/site-print/layout-robot/faq.html  
40. Trimble FieldLink — https://www.trimble.com/en/products/building-construction-field-systems/fieldlink-software  
41. LimnTech Vontz case study (CSV layout) — https://limntech.com/wp-content/uploads/2025/05/Vontz-Case-Study-formatted.pdf  
42. LimnTech LifeMark-400 — https://limntech.com/lifemark400/  
43. Traqnology pavement GPS marking — https://www.traqnology-na.com/pavement/  
44. AKN Engineering — setting-out equipment (secondary) — https://www.aknengineering.co.uk/blog/what-is-the-most-accurate-equipment-for-setting-out-large-highways-projects  
45. ND DOT survey manual excerpt (check shots / RTS practice) — https://www.dot.nd.gov/manuals/design/surveymanual/total-station.pdf  

---

*End of research report. No repository code was modified for this task.*
