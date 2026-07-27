# How Commercial Pre-Marking Rovers Ingest DXF/CSV

**Date:** 2026-07-22 · **Method:** 3 parallel web-research agents (TinyMobileRobots / adjacent layout robots / survey-format conventions), vendor-primary sources preferred.
**Companion doc:** `docs/ROAD_MARKING_RESEARCH.md`

---

## TL;DR

TinySurveyor's ingest contract is fully documented and **much simpler than survey convention**: DXF must be pre-georeferenced with the operator selecting an EPSG projection in-app; CSV is **only three columns** (`name, Easting, Northing` — everything after column 3 is discarded); and **meaning is carried by DXF entity type, not by layer names or feature codes**. Traversal is **file row order, not an optimiser**. No vendor in the category publishes a layer→action standard.

---

## 1. TinySurveyor — the documented contract

Source: `help.tinysurveyor.com` (vendor Zendesk), 14 articles.

### DXF
> "Make sure the CAD drawing is georeferenced, meaning the lines are placed in the coordinates corresponding to the desired projection. E.g. UTM zone 32."

> "The app will import **spline, line, polyline, circle, arc, ellipse, points and Blocks**. **Text and Hatch will not be imported** and may lead to error if contained in the drawing."

Units: Meter / Feet / US Survey feet / Yard. Guidance is to import *as few lines as possible* (compute cost); AutoCAD `COPYBASE`/`PASTEORIG` to preserve coordinates, `EXPLODE` to inspect polylines.
— [Importing Job Files: How Should a File be Prepared?](https://help.tinysurveyor.com/hc/en-us/articles/19688099776541-Importing-Job-Files-How-Should-a-File-be-Prepared)

### CSV — three columns only
> "The app will read the CSV file with comma as delimiter and the default format is **Pointname, Easting, Northing. Any data after the three first columns will not be read.**"

> "A custom format can be defined in the first line of the CSV file using **name, x and y** to tell which order the values are listed."

Plus a global `Swap NE` toggle. **No elevation, no code column** — architecturally rules out feature codes.

### Layer semantics — ABSENT
Knowledge-base search for "layer" returns **zero results**. No layer→marking-type convention exists.

### Coordinate handling
Operator selects the projection by name or **EPSG/ESRI code** (sourced from epsg.io). Custom CRS via `.prj` import — "We support import of .prj files containing **WKT or WKT 2** proj-string text." Total-station / local-grid work sets projection to `None(m)/None(ft)/None(us-ft)` and trusts raw coordinates verbatim.
**There is no site-calibration ceremony** — file in a known CRS → tell the app which CRS → RTK lines up.
— [Projection article](https://help.tinysurveyor.com/hc/en-us/articles/19688113342877-Importing-Job-Files-How-do-I-Select-the-Right-Projection) · [.prj article](https://help.tinysurveyor.com/hc/en-us/articles/21683044327965-How-to-create-a-basic-prj-projection-file)

### Transfer
USB stick into the tablet, or direct tablet↔PC drag-drop. **No cloud job upload documented.**

### Execution
> "When driving lines, TinySurveyor will drive in a straight line towards the starting point of the line… When driving to a point, TinySurveyor will drive in a straight line towards the **first point in the job list**."

Order is **file row order** (tool-calibration docs confirm ascending/descending traversal). Operator anchors with a **"Near"** function: "Find the nearest point on the nearest line/point. The robot will start marking from this point."

Per-entity meaning is assigned **after import** via four states: **Normal / Ignored / Tree filter / Robot path**.

Runtime params: **Line length / Line space** (dash + gap, a *global* setting), **Paint ramp up / ramp down**, antenna height, tool height, tool side-shift, **Min. position frequency 7.5 Hz** (refuses to drive below) with a **1 s estimation window** so brief GNSS gaps under obstructions don't error.

Accuracy: 1–2 cm GNSS, mm with total station.

### Data logging (= as-built QA)
Logs "Calibrated Tool Coordinates, Antenna Coordinates (RAW + Proj.), Planned Coordinates" plus averaged **SURVEYED POINTS** at stationary waypoints. Note **tool and antenna are logged separately** — the nozzle offset is explicit in the record.

### Three test modes
Fast (path clearance only) · Normal (drives the outline, GNSS-strength check) · Full (entire field as if painting, no paint).

### Corrections to common assumptions
- **"TinyPlanner" / "TinyCommand" do not exist** as TinyMobileRobots products. Authoring happens in the customer's own CAD (AutoCAD named); the tablet app is importer/editor only.
- **TinyLineMarker (sports) has no DXF/CSV import at all** — fields are built in-app from driven GNSS points or on-screen dimensioning. Do not conflate with TinySurveyor.
- ⚠️ **LandXML support is unresolved.** Reseller pages ([Aptella](https://www.aptella.com/product/tiny-surveyor-plotter/), [Monsen](https://www.monsenengineering.com/survey/tinysurveyor/layout-stakeout-robot/)) list it; the official help centre documents only DXF and CSV.

---

## 2. Comparison across the category

| | **TinySurveyor** | **Civ Robotics CivDot** | **HP SitePrint** | **Dusty FieldPrinter** |
|---|---|---|---|---|
| Points input | CSV, 3 cols | CSV `name, northing, easting, elevation, annotation` | — | CSV (electrical trade) |
| Lines input | DXF | DXF | DXF (+DWG) | DWG via Revit/AutoCAD plugin |
| Layer semantics | **None documented** | Not documented | Visibility toggles only | Carries wall/plumbing type; **no published schema** |
| Prep step | Operator trims in CAD | Template given at on-site training | **Plugin-based** conversion required | Plugin + ~1 week human VDC prep |
| Site tie | EPSG / WKT projection pick | State plane + site calibration | **Robotic Total Station**, 2–3 control points | RTS / laser tracker, 3 min / 5–8 recommended |
| Ordering | **File row order** + "Near" anchor | Operator picks start + direction | Undisclosed "flight plan" | Not documented |
| Accuracy | 1–2 cm GNSS; mm w/ TS | 30 mm; 8 mm laser-guided | ±3 mm (2023 claim) | 1/16″ |

- **RoadPrintz has no file import at all** — in-cab touchscreen, 45-symbol MUTCD library, camera preview, press PAINT.
- **LimnTech documents no import format** — its 2024 release describes CAD *export* of camera-collected data only.

---

## 3. The convention you'd inherit if you went richer

TinySurveyor discards codes; the surveying industry does not. Established grammar = base code + instance + control token:

```
Civil 3D   EP1 B              EP=edge-of-pavement, 1=instance, B=begin figure
Carlson    EP BEG … EP END    BLD CLO (close figure), PC/PT (bracket a 3-point arc)
Trimble    "start join sequence" / "end join sequence" control codes in .fxl
```

A **token-per-point state machine** (line open/closed, curve open/closed), not a topology description.

Trimble's documented stakeout CSV: `Point name, First ordinate, Second ordinate, Elevation, Point code`. CivDot's five columns mirror it exactly.

**PNEZD vs PENZD has no formal spec** — de-facto only, which is why good importers let the user remap columns.

**No national CAD layer standard exists for pavement markings.** MUTCD governs appearance; every state DOT ships its own level table (FDOT `PMSIGNAL_ep`, ODOT category "Q" / section 520). NCS/AIA covers general civil layers (`C-ROAD-PVMT-T`, `V-PROP-LINE`) but not marking semantics.

---

## 4. What this means for our rover

### Already ahead
- `791ddd2` (CAD POINT = reference marker, not spray target) is **exactly** TinySurveyor's entity-type-carries-meaning model.
- Our layer mapping (the DIM-layer fix) **exceeds** what any vendor documents.
- `georef.py` auto-detection is better UX than "operator looks up the EPSG code on epsg.io".

### Deliberate divergence
We TSP-optimise entity order; TinySurveyor uses file row order + a "Near" anchor. Neither is wrong — but our saved-manual-entity-order feature is the analogue of their model, and it is what disabled the optimiser and produced the teleport corners. Their design avoids that failure class by never optimising at all.

### Worth adopting
1. **CSV header-defined column mapping** — TinySurveyor's `name, x, y` first-line trick solves PNEZD-vs-PENZD without a settings toggle. Adopt it, but keep **five** columns (name, N, E, elev, code). Three columns is their limitation, not a model: road work needs the code field, and CivDot/Trimble both carry it.
2. **A `Robot path` display state** — their four-state per-entity model surfaces non-painting transit legs explicitly. We have transit preview; making it a first-class per-entity state is cheap.
3. **Min. position frequency gate** — 7.5 Hz refuse-to-drive with a 1 s estimation window is a concrete, field-proven number for Spray V2 Phase B.
4. **Their data-logging schema is our as-built QA format** — planned vs antenna vs calibrated-tool coordinates, plus averaged surveyed points at stationary waypoints. Logging tool and antenna separately independently validates the nozzle-offset plan.
5. **Three graduated test modes** rather than a single dry-run flag.

### The road differentiator
TinySurveyor's dash control is **Line length / Line space as a GLOBAL setting**. No commercial system found encodes a *per-segment* dash pattern. IRC's 3 m+6 m normal vs 6 m+3 m warning sections is beyond what the category currently does — a genuine differentiator if built into `PathSegment.metadata` rather than one global pair. See the Phase C note in `SPRAY_V2_PHASES_BF_IMPLEMENTATION_PLAN.md`.

---

## 5. Where we are weak

Ranked, from this comparison:

1. **No feature/point codes.** Our DXFs carry `Names`/`Codes`/`Heights` MTEXT (a survey export) that we never read. Codes are the industry's channel for intent — which points to mark, where lines start and end.
2. **No CSV ingest at all.** DXF-only. CivDot and TinySurveyor both take point CSVs, and that is the native format for point/stakeout work.
3. **No operator subset control after import.** No per-entity Normal/Ignored equivalent exposed the way TinySurveyor does it.
4. **Ordering is opaque and has bitten us.** TSP is stronger in principle, but our saved-order path silently disabled transit insertion. Vendors keep it dumb and operator-visible.
5. **No graduated pre-flight test modes**, and no position-frequency drive gate.
6. **No as-built log** of planned vs actual vs tool position.

---

## 6. Caveats

- TinySurveyor LandXML support: reseller-claimed, absent from official docs. **Unresolved.**
- The Dusty ~1-week file-prep figure is from a paywalled Springer case study via search snippet — single project, lower confidence.
- No vendor publishes a marking-sequence algorithm. Civ Robotics and TinySurveyor delegate to the operator; HP's is a black box.
- No third-party field accounts of TinySurveyor DXF prep surfaced — everything is vendor-authored.

---

## Sources

TinySurveyor knowledge base (14 articles, linked inline) · [TinySurveyor e-book](https://www.monsenengineering.com/wp-content/uploads/sites/7/2021/07/TinyMobileRobots-TinySurveyor-e-book.pdf) · [TinyLineMarker Pro X manual](https://help.tinylinemarker.com/hc/en-us/articles/22140936565533-User-Manual-TinyLineMarker-Pro-X) · [Civ Robotics FAQ](https://www.civrobotics.com/faq) · [CivDash](https://www.civrobotics.com/products/civdash) · [HP SitePrint FAQ](https://www.hp.com/gb-en/printers/site-print/layout-robot/faq.html) · [AEC Magazine, Oct 2023](https://aecmag.com/construction/hp-siteprint-the-robot-that-prints-11-plans/) · [Dusty control points](https://www.dustyrobotics.com/blog/understanding-control-in-construction-layout-methods-pros-and-cons) · [Dusty for AutoCAD](https://marketplace.autodesk.com/apps?id=1011912448359747061) · [Trimble Access point files](https://help.fieldsystems.trimble.com/trimble-access/latest/en/data-files-points-lines.htm) · [Trimble control codes](https://help.fieldsystems.trimble.com/trimble-access/latest/en/feature-libraries-control-codes.htm) · [Autodesk field codes](https://help.autodesk.com/cloudhelp/2023/ENU/Civil3D-UserGuide/files/GUID-2DC2AA57-057B-41AC-BA2E-C893FF01A300.htm) · [Carlson feature codes](https://help.carlsonsw.com/en/survpc/main-menu/file/feature-code-list/feature-code-list.html) · [NCS layer format](https://www.nationalcadstandard.org/ncs5/pdfs/ncs5_clg_lnf.pdf) · [FDOT CADD level table](https://fdotwww.blob.core.windows.net/sitefinity/docs/default-source/cadd/downloads/publications/caddmanualfdm/levels/fdotcaddstandardleveltable-09302021.pdf) · [Raise Robotics](https://raiserobotics.ai/blog/learning-center/guide-to-automating-construction-layout/) · [ISARC 2018 marking robot](https://www.iaarc.org/publications/fulltext/ISARC2018-Paper074.pdf) · [MDPI Buildings 2023](https://www.mdpi.com/2075-5309/13/9/2212) · [LimnTech 2024 release](https://www.prnewswire.com/news-releases/limntech-scientific-announces-data-export-and-import-ability-of-lifemark-100-automated-layout-system-301624721.html)
