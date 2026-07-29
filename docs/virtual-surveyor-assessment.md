# Virtual Surveyor (virtual-surveyor.com) — Capability & Accuracy Assessment for Autonomous Road-Marking Geometry Authoring

**Prepared for:** DYX Autonomous — pre-line marking rover programme
**Question:** Can drone + Virtual Surveyor replace or complement the RTK-pole survey as the front end that authors drivable marking geometry at ±2.5 cm?
**Date of research:** 27 July 2026
**Method:** vendor primary sources (marketing pages, support/knowledge base, press releases) + peer-reviewed photogrammetry accuracy literature. Marketing claims and independently verifiable numbers are kept in separate columns throughout. Every factual claim carries a URL. Items that could not be verified are explicitly flagged.

---

## 0. Verdict first

**Virtual Surveyor is structurally a 3–8 cm horizontal tool for line work at realistic corridor flight parameters. It cannot author paint-line geometry to ±2.5 cm. It is, however, a genuinely useful complement for context, as-built capture, and planning.**

Three findings drive this:

1. **The vendor publishes no horizontal accuracy number at all.** Their own validation study reports vertical RMSE only (4–5 cm with GCPs). Horizontal accuracy was "verified visually" by eyeballing a check point against a ground marker — no RMSE, no residual table. For a horizontal paint spec, the vendor's central accuracy claim is simply absent.
2. **The best independent, road-specific study achieves RMSExy = 2.6–2.8 cm** at 1.75 cm GSD, 65 m AGL, with 9–11 surveyed GCPs over 2.1 km. That is *the ceiling of careful academic practice on exactly your geometry* — and it is already at your tolerance, as a 1σ statistic, before any rover control error is added.
3. **Virtual Surveyor's road tool doesn't produce marking geometry.** The "6 steps" workflow builds a **TIN surface** of the road at 30 m station spacing and 3 m cross-section point spacing, for CAD/earthworks handoff. It is a topographic deliverable, roughly three orders of magnitude coarser than a paint line, and its CSV export cannot emit polyline vertices at all.

Recommended posture: **drone + Virtual Surveyor as the area/context and as-built layer; RTK pole remains the geometric authority, with a small pole-shot control set used to rigid-fit any drone-authored geometry before it is painted.** Detailed proposal in §3.4.

---

# PART 1 — Who they are

## 1.1 Legal entity, HQ, history

| Fact | Value | Source | Confidence |
|---|---|---|---|
| Legal entity (EU/HQ) | Virtual Surveyor nv, Kleine Mechelsebaan 52, B-3200 Aarschot, Belgium. VAT BE0871935077 | https://www.virtual-surveyor.com/virtual-surveyor-offices-throughout-the-world | **Verified — primary** |
| US entity | Virtual Surveyor North America Inc., 4509 Creedmoor Rd. Suite 201, Raleigh, NC 27612 | same as above | **Verified — primary** |
| Founded | 2015 | https://tracxn.com/d/companies/virtual-surveyor/__-34Mz3zpM6wKwomNDsOM1EkLcJmXLi3U2x3LtFhpyaM | Secondary (database) |
| Founder / CEO | Tom Op 't Eyndt | Tracxn (above); quoted as CEO in https://amerisurv.com/2023/09/25/virtual-surveyor-unveils-photogrammetry-app-in-major-new-release-of-smart-drone-survey-software/ | Corroborated |
| Team size | ~9 employees | https://rocketreach.co/virtual-surveyor-group-nv-profile_b55870a2f6851ec6 | **Unverified — scraped aggregator. Treat as order-of-magnitude only.** |
| Funding | No disclosed rounds | Tracxn; https://www.crunchbase.com/organization/virtual-surveyor | Secondary |

**Flags / discrepancies:**
- Tracxn lists the company as based in **Leuven**; the company's own offices page and its 2023 press releases both say **Aarschot**. Also, a Belgian registry listing shows the VAT number BE0871935077 against a Nieuwlandlaan address in Aarschot (https://www.staatsbladmonitor.be/bedrijfsfiche.html?ondernemingsnummer=0871935077). Aarschot is correct; Leuven is likely a stale or regional entry.
- Op 't Eyndt was described as **"managing director"** in 2021 coverage (https://www.geoweeknews.com/news/virtual-surveyor-6-1-turns-drone-point-clouds-into-cad-models-in-just-a-few-steps) and **"CEO"** from 2023. Title change, not a different person.
- The "used in 46 countries" figure that appears on aggregator pages is recycled vendor marketing copy, not an audited number.

**Product history relevant to you:**
- Virtual Surveyor began as a *downstream* tool: it took orthophotos + DSMs produced elsewhere (Pix4D, Metashape, DroneDeploy) and gave surveyors an interactive environment to digitise points and breaklines into CAD. Explicitly described as bridging "the gap between drone photogrammetric processing applications and engineering design packages" (https://amerisurv.com/2023/09/25/virtual-surveyor-unveils-photogrammetry-app-in-major-new-release-of-smart-drone-survey-software/).
- **v6.1 (≈2021)** added the ability to auto-generate section lines from road surfaces and to change coordinate systems mid-project (https://www.geoweeknews.com/news/virtual-surveyor-6-1-turns-drone-point-clouds-into-cad-models-in-just-a-few-steps).
- **v9.0 (25 Sept 2023)** added the **Terrain Creator** app — their own photogrammetry front end — making the suite end-to-end (photos → ortho/DSM → digitised CAD). Bundled with Ridge and Peak subscriptions (https://amerisurv.com/2023/09/25/virtual-surveyor-unveils-photogrammetry-app-in-major-new-release-of-smart-drone-survey-software/).
- Current major version is **9.x** (a reseller page lists "Virtual Surveyor 9.0 is now available": https://www.aptella.com/product/virtual-surveyor-3/).

## 1.2 Pricing / licensing tiers

Scraped from the live pricing page (https://www.virtual-surveyor.com/pricing):

| Tier | Price shown | Key features |
|---|---|---|
| **Valley** | €0 — free for everyone | Import orthos & DSMs, convert point clouds, profiles, draw points & lines, create surfaces, export to CAD |
| **Ridge** | €100 | All Valley + **Photogrammetry (Terrain Creator)**, Point Grids, **Line Tools**, Advanced Editing, Stockpile Reports |
| **Mountain** | €210 | All Ridge + Timeline, Remove Objects, Compare Profiles, Cut/Fill Maps, Extract Cut/Fill Areas |
| **Peak** | €210 | All Mountain + Offset by Slope, Curve-Based Elevation, Grading, Intersections, Offset Surfaces |

**Flags:**
- **The billing period does not render on the scraped page** (a monthly/annually toggle controls it) and **Mountain and Peak both display €210**, which is almost certainly a toggle-state artefact rather than genuinely identical pricing. **Do not quote these figures to anyone without confirming with their sales.**
- Third-party figures broadly corroborate the order of magnitude: Tracxn lists "$0–$149/user/month"; Commercial UAV News wrote in 2020 that "prices start at around $100 per month" (https://www.commercialuavnews.com/surveying/virtual-surveyor-newest-features-deliver-improved-insights-and-worker-s-safety).
- Licensing is **floating cloud licensing** — you licence concurrent users, not machines; the licence follows the login. Auto-renew is **off by default**. 14-day trial gives full Peak features. After trial you drop to free Valley and retain project access (all from the pricing FAQ, URL above).
- **Which tier you'd need:** the Section Lines road workflow uses "Line Tools", and Terrain Creator photogrammetry is Ridge-and-above. So **Ridge minimum** — free Valley cannot process photos, though it *can* import an ortho+DSM you processed elsewhere and still draw points/lines and export to CAD. That is a real free-tier evaluation path.
- Software is **Windows desktop**; the licensing is cloud. There is a support article on "Running Virtual Surveyor on Mac" (listed at https://support.virtual-surveyor.com/support/solutions), i.e. it is not native — relevant to you given the M5 MacBook.

## 1.3 Positioning vs competitors, and who actually uses it

Their own solution pages target: **Topographic Surveying, Stockpile Inventory, Mining & Quarrying, Construction Earthworks, Water Management, Waste Management** (nav on https://www.virtual-surveyor.com/pricing). Roads appear *inside* topographic surveying as a corridor-survey workflow — not as a standalone road/highway product line, and nowhere as a marking or setting-out product.

| Product | What it is | How it differs from VS |
|---|---|---|
| **PIX4Dsurvey** | Closest direct analogue: vectorise point clouds into CAD-ready geometry | **Vectorises in 3D from the point cloud and lets you verify each vertex against the original images** — "Vectorize in 3D, edit in 2D… survey complex objects only visible in images" (https://www.pix4d.com/product/pix4dsurvey). This is materially better for precise line work than digitising on a flattened raster ortho. Exports DXF/SHP; DTM to LandXML/GeoTIFF (https://support.pix4d.com/hc/en-us/articles/360033317432) |
| **Propeller Aero** | Cloud platform + AeroPoints smart GCPs, earthworks/mining focus | Hardware-coupled ground control, cloud processing, cut/fill and volume analytics |
| **DJI Terra** | DJI-ecosystem 2D/3D reconstruction | Locked to DJI hardware; processing, not digitising |
| **DroneDeploy** | Cloud mapping SaaS, construction-led | Cloud-only, no offline processing |
| **Trimble Stratus / Carlson PhotoCapture** | Processing/analytics in vendor ecosystems | Ecosystem plays; Carlson's is a processing service feeding Carlson CAD |

*Source note:* the competitive-landscape framing above draws on vendor pages plus several 2026 comparison round-ups (e.g. https://www.guideflow.com/blog/best-drone-analytics-software, https://contechfinder.com/blog/best-construction-drone-and-survey-software-for-2026). **These round-ups are SEO content-marketing, not independent benchmarks. Use them for taxonomy, not for accuracy claims.**

**Bottom line on positioning:** Virtual Surveyor's differentiator is *ease and speed of turning drone terrain into a lightweight CAD surface with almost no learning curve*. It optimises for surveyor productivity on earthworks-class deliverables. It does not compete on geometric precision of linear feature extraction — that is PIX4Dsurvey's ground.

---

# PART 2 — Working principle, precisely

## 2.1 The pipeline

### Terrain Creator (photos → ortho + DSM)

Full workflow, from https://support.virtual-surveyor.com/support/solutions/articles/1000318552-from-drone-photos-to-topographic-surveys:

1. **Add photos** — either geotagged JPEGs (positions read from EXIF), or a `.csv`/`.txt` file of camera positions, which is the **PPK path**: "The camera positions in the photos' Exif metadata (if any are included) are ignored in favor of the PPK camera positions that are included in the external .csv file."
2. **Set Project Coordinate System** — horizontal CRS + vertical reference (ellipsoidal or geoidal) from dropdowns; "there is no need to worry about configuring a geoid grid because Terrain Creator handles that for you." Reprojection can be changed before processing.
3. **Review/cull photos** — delete blurry, overexposed, ground-taken, or out-of-area frames.
4. **Add GCPs** — imported from `.csv`/`.txt` with columns **EPSG (optional), Name, X, Y, Z**. If EPSG is used it must be the **first row**, with two numbers: horizontal CRS EPSG, then vertical reference EPSG.
5. **Mark GCPs** — guided, iterative process. Terrain Creator builds a rough initial model, proposes the photo and the zoom window for each GCP, you click the marker centre, press space, repeat. The article is explicit that "the positional accuracy of your camera positions greatly influences the accuracy of the initial model" — with RTK/PPK the tool zooms tight; with plain GPS it zooms out and you hunt. **You decide when to stop marking** — the stated stopping criterion is subjective: "a good indication is when your mouse cursor is landing near the center of focus for each candidate."
6. **Process Photos** → ortho + DSM. "This process can take minutes, hours, or even up to a day."
7. **Inspect GCP residuals**, then **Open in Virtual Surveyor**.

### Which SfM engine?

**Not disclosed.** No vendor page, support article, or the 2023 launch press release names the underlying SfM/MVS engine, nor whether it is in-house or licensed. The press release only says Terrain Creator "removes complexity from the drone photogrammetry process." **Flag: unverifiable from public sources.** Worth asking their support directly — the answer materially affects whether published Pix4D/Metashape accuracy literature transfers.

### Import path (bypassing Terrain Creator)

Virtual Surveyor also accepts externally processed products (https://support.virtual-surveyor.com/support/solutions/articles/1000273543-supported-input-file-formats):

- **Ortho/topomap raster:** GeoTIFF, .jp2, .ecw, .tif+.tfw, .jpg+.jgw, .png+.pgw, .img
- **DSM/DTM raster:** GeoTIFF, .tif+.tfw, .img
- **Point cloud:** .las, .laz — **"Point cloud files are converted to raster elevation models."**
- **Vector:** .dxf, .shp, .xml (LandXML), .txt, .csv

**This is architecturally decisive for your use case.** Virtual Surveyor is a **2.5D raster environment**: an "Image Terrain" (the ortho) draped over an "Elevation Terrain" (the DSM raster). Even a LiDAR point cloud gets rasterised on import. You are always digitising on a **flattened, orthorectified image**, never on the original photographs and never in a 3D point cloud with image verification. Every line you draw inherits the full orthorectification error budget, and you have no way to cross-check a vertex against the raw imagery — which is exactly the check PIX4Dsurvey provides.

### What does "survey-grade" mean in their materials?

**It is never defined numerically.** The phrase appears throughout ("survey-grade terrains", "survey-grade orthomosaics and digital surface models", "a corrected survey-grade model"). The only place it is given operational content is the validation article, where a **vertical RMSE of 0.02 m on asphalt** is called "a definitive survey-grade RMSE accuracy" (https://support.virtual-surveyor.com/support/solutions/articles/1000320871-terrain-creator-validation). So in practice: *survey-grade = vertical RMSE in the 2–5 cm band*. **No horizontal definition exists anywhere in their materials.** Treat the term as marketing until they give you a number in writing.

## 2.2 The accuracy chain, with numbers

### 2.2a — Vendor claims (their own validation study)

Source for all of this section: https://support.virtual-surveyor.com/support/solutions/articles/1000320871-terrain-creator-validation

**Test Site 1 — Texas, USA. RTK. DJI Mavic 3 Enterprise, 20 MP, 442 photos, GSD 0.05 m, 6 GB.** 120 check points, 5 GCPs available.

| GCPs used | Check points | Average error (m) | **Vertical RMSE (m)** |
|---|---|---|---|
| 5 | 120 | 0.01 | **0.04** |
| 1 | 120 | 0.00 | **0.05** |
| 0 | 120 | 0.07 | **0.08** |

Ground-cover breakdown at 5 GCPs:

| Surface | Check points | Avg error (m) | Vertical RMSE (m) |
|---|---|---|---|
| All | 120 | 0.01 | 0.04 |
| **Asphalt** | 30 | 0.00 | **0.02** |
| Natural ground (soft dirt) | 46 | 0.02 | 0.05 |

**Test Site 2 — Port Melbourne, Australia. PPK. DJI Phantom 4 Pro RTK, 20 MP, 155 photos, GSD 0.06 m.** 13 check points.

| GCPs used | Check points | Average error (m) | **Vertical RMSE (m)** |
|---|---|---|---|
| 5 | 13 | 0.00 | **0.04** |
| 1 | 13 | 0.01 | **0.05** |
| 0 | 13 | **−0.45** | **0.45** |

Applying a −0.07 m (Site 1) / +0.45 m (Site 2) `Offset Z` correction against independent check points pulled the 0-GCP datasets back to RMSE 0.05 m and 0.04 m respectively.

**Their conclusion, verbatim in substance:** RTK or PPK with **at least 1 GCP** is "plenty capable of accomplishing survey-grade results"; GPS-only processing "would generally require at least 10 evenly spread out GCPs" to match.

Separately, a worked example in the Terrain Creator walkthrough reports an **overall GCP RMSE of 0.016 m**, with the worst individual GCP at **2.8 cm**, described as "very low" (https://support.virtual-surveyor.com/support/solutions/articles/1000318552-from-drone-photos-to-topographic-surveys).

### 2.2b — What is wrong with those numbers, for your purpose

Five substantive problems:

1. **Every published RMSE is vertical.** Both test sites' "Horizontal Accuracy Results" sections contain no numbers whatsoever. The method described is: use the distance measurement tool and *"visually verify the accuracy of a check point in relation to the ground marker"*, concluding "in all cases the check point is either on or virtually in the center of the ground marker." At a 5 cm GSD, "in the centre of the marker" is a statement with roughly ±5 cm of resolving power. **This is not a horizontal accuracy assessment.**

2. **The 0.016 m figure is a GCP residual, not a check point residual.** GCP residuals measure how well the bundle adjustment fitted the points it was *given*; they systematically understate true accuracy because the model deforms to match them. The corridor literature is explicit that using GCPs rather than withheld CPs "is not a good methodology for estimating the accuracy" (Ferrer-González et al. 2020, §1, https://doi.org/10.3390/rs12152447).

3. **GSD is 5–6 cm in both tests.** That is the practical ceiling of the whole exercise. The widely used rule of thumb is that absolute horizontal accuracy cannot beat roughly 1–2× GSD; the largest systematic study (Sanz-Ablanedo et al. 2018, cited in Ferrer-González §1) found RMSE "converges slowly to a value approximately double the GSD." At 5 cm GSD you are structurally in the **5–10 cm horizontal band**, whatever the vertical numbers say.

4. **Internal inconsistencies reduce the evidential weight.** The narrative bullet says 5 GCPs gave "an average vertical error of 0.00 m" while the adjacent table says 0.01 m; the screenshot column headers read "5 GCPs / **3 GCPs** / 0 GCPs" while the surrounding text and tables describe 5 / **1** / 0. Sloppy, and this is their flagship accuracy document.

5. **N=13 check points at Site 2.** Thirteen points is a thin statistical basis for an RMSE, and the article itself notes "we had no control over where to place the GCPs for this dataset."

### 2.2c — Independent numbers, road-specific

The single most relevant independent study is **Ferrer-González, Agüera-Vega, Carvajal-Ramírez & Martínez-Carricondo (2020), "UAV Photogrammetry Accuracy Assessment for Corridor Mapping Based on the Number and Distribution of Ground Control Points", *Remote Sensing* 12(15):2447, https://doi.org/10.3390/rs12152447**. It is your geometry almost exactly.

**Setup:** A-1051-R3 branch road, Almería, Spain. **2.1 km × 190 m (~40 ha)**. DJI Phantom 4 Pro (20 MP, 1" sensor, 24 mm equiv.). **65 m AGL, GSD 1.75 cm/px**, 80% forward / 60% side overlap, 746 images, four flights. Processing in Pix4Dmapper 4.5.6. 47 targets surveyed with Trimble R6 in PPK mode, base <1 km from all points (manufacturer spec 8 mm + 1 ppm horizontal, 15 mm + 1 ppm vertical). 29 withheld as check points. 13 GCP configurations across 4 distribution patterns.

**Headline results:**

| Configuration | GCPs (per km) | **RMSExy (m)** | RMSEz (m) |
|---|---|---|---|
| Dist. 2 (zigzag both sides), 9 GCPs | 9 (4.3/km) | **0.026** | 0.071 |
| Dist. 4 (zigzag + pair at each end), 11 GCPs | 11 (5.2/km) | **0.028** | 0.055 |
| Dist. 4, 9 GCPs | 9 (4.3/km) | 0.029 | 0.057 |
| Dist. 1 (facing pairs), 18 GCPs | 18 (8.6/km) | **0.027** | 0.055 |
| Dist. 4, 7 GCPs | 7 (3.3/km) | 0.031 | 0.081 |
| Dist. 1, 4 GCPs | 4 (1.9/km) | 0.061 | 0.394 |
| Dist. 3 (one side only), 3 GCPs | 3 (1.4/km) | 0.084 | 0.931 |

**Key conclusions from the paper:**
- "Only five GCPs (approximately 2.4 GCPs km⁻¹) were necessary to achieve an RMSExy less than two times the GSD… and **no fewer than nine GCPs (4.3 GCPs km⁻¹) were required to achieve RMSExy values less than 0.03 m**."
- "The increase in accuracy became insignificant when more than nine GCPs were used." Going from 5 to 18 GCPs bought **less than 1 cm**.
- Optimal distribution for corridors: **alternating (zigzag) on both sides of the road, with a pair of GCPs at each end.** GCPs on one side only were the worst configuration on every metric.
- Planimetric accuracy was always better than vertical.

**Corroborating independent numbers:**
- Skarlatos et al. (2013), 2.2 km × 160 m corridor, **4 cm GSD**, 7 GCPs: **RMSExy = 0.130 m**, RMSEz = 0.170 m — roughly 3× and 4× GSD (as reported and compared in Ferrer-González §4). *This is the closest published proxy for Virtual Surveyor's own 5 cm GSD test conditions, and it lands at 13 cm horizontal.*
- Agüera-Vega et al. (2017): best case across four terrain morphologies and four altitudes was **0.053 m horizontal / 0.049 m vertical**, at 50 m AGL with 10 GCPs (cited in Ferrer-González §1).
- Forlani et al. (2018): RTK-drone DSM accuracy ≈ **2.1 GSD** with GCPs or with camera stations + 1 GCP; **3.7 GSD** with camera stations alone (cited in Ferrer-González §1).
- Sanz-Ablanedo et al. (2018), 3,465 configurations over 1,225 ha: RMSE converges to **~2× GSD** (cited in Ferrer-González §1).

### 2.2d — Can a line digitised on an orthomosaic be trusted to ±2.5 cm?

**No. Here is the arithmetic.**

Digitising error decomposes into (a) the georeferencing error of the ortho and (b) the operator's pointing error on the raster.

**(a) Georeferencing.** Best published corridor result: **RMSExy = 2.6 cm at 1.75 cm GSD with 4.3 GCP/km.** RMSE is a **1σ** statistic — roughly 68% of points fall inside it in 1D, ~39% in 2D radially. A ±2.5 cm *tolerance* is normally read as a bound (95%, or 2σ-ish), which would require **RMSExy ≈ 1.2–1.3 cm**. Nobody in the published corridor literature has achieved that on a 2 km road.

> **This is the first question to settle internally, Vetri:** is your ±2.5 cm a 1σ RMSE or a hard 95% bound? It changes the required RMSE by a factor of two and it changes what you can honestly promise a client. The rest of this analysis assumes the stricter (bound) reading, which is how a paint spec normally reads.

**(b) Operator pointing.** On a raster, best-case manual pointing repeatability is ~0.5 px (a value adopted from the literature in a 2026 processing-parameters study: https://www.sciencedirect.com/science/article/pii/S0263224126000242), and 1 px is more realistic for a soft-edged paint line on textured asphalt. At Virtual Surveyor's own test GSD of 5 cm that is **2.5–5 cm of pointing error alone**, before any georeferencing error. At 1.75 cm GSD it is 0.9–1.8 cm.

**(c) Combined, RSS, at two operating points:**

| Scenario | GSD | Georef RMSExy | Pointing (1 px) | Combined 1σ | Implied 95% |
|---|---|---|---|---|---|
| VS's own validation conditions | 5 cm | ~5–10 cm (inferred, 1–2× GSD) | 5 cm | **7–11 cm** | 14–22 cm |
| Best academic corridor practice | 1.75 cm | 2.6 cm | 1.75 cm | **3.1 cm** | ~6 cm |
| Theoretical best you could push to | 1.0 cm | ~2.0 cm (2× GSD) | 1.0 cm | **2.2 cm** | ~4.5 cm |

**Even the theoretical best — 1 cm GSD, which means flying at ~35–40 m AGL over live traffic, with 8–10 GCPs/km — does not clear a ±2.5 cm bound.** And it says nothing about the rover's own control error, which is additive on top.

**Your full error budget for a painted line would be:**

```
σ_total = √( σ_geometry² + σ_rover_control² + σ_mechanical² )
```

With your rover already running RTK at 2–4 cm HRMS and NAV_ACC_RAD tuned to 5 cm, plus spray-head offset and latency pre-fire residuals, **you have very little budget left to spend on the geometry source.** The pole, at 1.6–1.8 cm/point observed (spec: Emlid Reach RS3 RTK H = 7 mm + 1 ppm, V = 14 mm + 1 ppm, PPK H = 5 mm + 0.5 ppm — https://docs.emlid.com/reachrs3/specifications/specs/), consumes roughly a third of the geometry budget that photogrammetry would consume.

## 2.3 The "6 steps" road-survey workflow, step by step

The video (https://www.youtube.com/watch?v=oy062wAP0o8) corresponds to the knowledge-base article **"Road Survey with Section Lines"** (https://support.virtual-surveyor.com/support/solutions/articles/1000291529-road-survey-with-section-lines, last modified 9 Jan 2026). The KB article is the authoritative and current version — the video dates from ~2019 and a newer walkthrough exists at https://www.youtube.com/watch?v=dcVMeW11LHI (Sept 2025).

**Step 1 — Draw a Centerline.** Switch to 2D View Mode. Home tab → Polyline tool → trace the road centreline; the Arc setting helps on curves. Purpose is explicitly stated: *"The captured centerline is primarily used as a reference for the section line creation."* It is a construction line, not a deliverable — the article later instructs you to switch it **off** before triangulating.

**Step 2 — Create Section Lines.** Select the centreline → TOOLS for Polyline tab → Create group. Set **Along** (spacing between successive section lines) and **Across** (length of each section line = survey width). The article's recommended values: **Along = 100 ft / 30 m; Across = 100 ft / 30 m.** Section lines are also automatically drawn at every vertex of the centreline. A **Boundary** is auto-generated around the whole set. Per the Section Lines reference (https://support.virtual-surveyor.com/support/solutions/articles/1000291621-section-lines), Across should be measured from the outermost feature on one side to the outermost on the other, e.g. edge-of-gravel to edge-of-gravel.

**Step 3 — Remove redundant section lines.** New section lines are created flat and over-dense; you cull them from the Project View layer list, keeping every 30 m / 100 ft.

**Step 4 — Drape and densify onto the terrain.** Move Boundary to a separate layer. Select the Section Lines layer → TOOLS → Densify with **Method = Regular, Distance = 10 ft / 3 m**. This is the step that pulls flat lines onto the elevation terrain and populates them with vertices.

**Step 5 — Edit vertices, then model and clean the boundary.** Vertices landing on trees, jersey barriers, or rails are deleted or Z-corrected via Edit Vertex / Edit Z. Then the Boundary is draped with **To Terrain**, redundant boundary vertices deleted, and boundary ends snapped to the section-line ends (Geometry Snapping, Transparent Lens to see under foliage).

**Step 6 — Create the road surface and export.** Turn the original centreline off. Select the Boundary → **Triangulate Within** → a **TIN** surface is generated inside the boundary. Then EXPORT tab → choose CAD format → **Export Survey**.

**What the output file actually contains:** a **TIN surface** of the road, plus (if left visible) the boundary and section lines, in .dxf / .xml (LandXML) / .shp / .kml.

**Assessment for your use case.** This is a **topographic corridor deliverable for design and earthworks** — the road as a 3D surface, sampled every 30 m longitudinally and every 3 m transversely. It is not, and does not claim to be, marking geometry. Nothing in it captures the position of a paint line. To get marking geometry from Virtual Surveyor you would abandon the Section Lines tool entirely and hand-trace each marking as a polyline on the orthomosaic — which is a different workflow the vendor does not document, and which lands squarely on the accuracy limits of §2.2.

## 2.4 Export formats — and the CSV problem

Primary source: https://support.virtual-surveyor.com/support/solutions/articles/1000305567-export-survey

**Formats:** `.dxf`, `.xml` (LandXML), `.shp`, `.csv`, `.kml`. Export is **WYSIWYG** — "All data that is visible in the Viewport is exported"; you control it via the Layers panel checkboxes. **LandXML cannot be exported without a projected coordinate system.**

### The CSV export

Exact structure (https://support.virtual-surveyor.com/support/solutions/articles/1000271434-export-points-as-a-csv-file):

| Column | Meaning |
|---|---|
| **P** | Point number |
| **X** | Easting/Westing |
| **Y** | Northing/Southing |
| **Z** | Elevation |
| **D** | Descriptor |
| **Layer** | Point layer (optional) |

- Coordinates carry **up to 6 decimal places**.
- **"The column separators used in your exported .csv are formatted according to your Windows regional settings."**

### The blocking limitation

> **"The Virtual Surveyor app only exports single point data — descriptors, numbers, and coordinates — when you choose to export your survey as a .csv file. Vertices that come from polylines, boundaries, and surfaces are not exported in this file type."**
> — https://support.virtual-surveyor.com/support/solutions/articles/1000305567-export-survey

The workaround is the **Extract Points** tool (https://support.virtual-surveyor.com/support/solutions/articles/1000296240-extract-points), which "creates individual points from your surface, drawn geometries, or point grids… A point is created in the Layers panel for each vertex."

### Answering your specific question

**Can it export a polyline as ordered stations with codes — i.e. exactly the CSV your rover already ingests? No, not directly.**

1. **CSV is points-only.** You must first explode every polyline into loose points via Extract Points. Once exploded, the *polyline* — the thing that carries connectivity, direction, and station order — no longer exists in the export. You get an unordered bag of vertices with point numbers and descriptors. Point numbering may happen to follow creation order, but the format guarantees nothing about traversal order, and there is no chainage/station column.
2. **Coordinates are projected (X = easting, Y = northing), not lat/lon.** So your existing pyproj EPSG:32644 → WGS84 step stays in the pipeline regardless. That is fine — you already have it, verified to <0.5 mm round-trip.
3. **The separator follows Windows regional settings.** On an Indian-locale machine this is usually safe, but any European-locale machine will emit semicolon-separated files with comma decimals. Build a sniffing parser; do not assume `,`.
4. **The 6-decimal cap** is harmless on projected metres (µm resolution) but would be **~11 cm** if anyone ever exports geographic coordinates. Never accept lat/lon CSV from this tool.

### The better ingestion route for DYX

**Take the DXF, not the CSV.** You already have a hardened DXF → Karney Transverse Mercator (pyproj EPSG:32644) → WGS84 → Emlid Flow CSV pipeline, and `path_engine` already densifies polylines into waypoints. A DXF preserves:

- polyline connectivity and vertex order,
- layer names, which map naturally onto your IRC:35-2015 marking codes (LM/TM/HM/BM/AM/FM/OM),
- arcs and curve geometry, which a point list loses.

Densification is *your* job anyway and you do it better — arc-length-based, matched to your spray-zone logic and pre-fire latency compensation. Do not let Virtual Surveyor's 3 m Densify tool decide your waypoint spacing.

---

# PART 3 — Fit for the DYX rover (critical assessment)

## 3.1 Failure modes for centimetre-level line work

**1. Raster resolution is a hard floor.** You digitise on the orthomosaic. At the vendor's own test GSDs (5–6 cm) a single pixel is twice your entire tolerance. No amount of GCPs fixes this — GCPs fix *georeferencing*, not *sampling*.

**2. Orthomosaic distortion is a real, documented mechanism.** The ortho is produced by reprojecting images through the DSM. Pix4D documents the consequence plainly: *"errors and noise present in the Densified Point Cloud will be reflected in the orthomosaic. Such errors appear often at building edges or on small details"* (https://support.pix4d.com/hc/en-us/articles/202561099). Every DSM height error at a point becomes a **planimetric displacement** of the pixel at that point, scaled by the local viewing obliquity. On a flat carriageway away from the nadir the effect is small; near kerbs, gantries, medians, parked vehicles, and image seamlines it is not.

**3. Between-GCP drift and the bowl/doming effect.** Corridor geometry is the worst case for SfM self-calibration — camera calibration errors accumulate along a long thin strip. Ferrer-González et al. discuss the mechanism explicitly and note the two mitigations: densify GCPs, or improve exterior orientation estimation (i.e. RTK/PPK camera stations). Their own data shows GCPs **on one side of the road only** was the worst distribution on every metric, with local M3C2 errors reaching **~0.3 m** in one region. **If you ever fly this, GCPs must alternate on both sides with a pair at each end.**

**4. DSM smoothing does not affect paint edges the way you might expect — and that's the problem.** Paint has no height signature, so the DSM contributes nothing to locating it; you are relying entirely on the ortho's radiometry. What DSM smoothing *does* corrupt is the physical breaklines you might want as reference — kerb lines, edge of pavement, joints — which get rounded. So neither channel gives you a crisp geometric handle on markings.

**5. CRS and vertical datum mismatches against your base.** Terrain Creator handles geoid transforms behind a dropdown, and that is convenient — but their own troubleshooting article warns of exactly this failure: *"A common scenario is when you're working in a local coordinate system, which Terrain Creator does not support yet. In this case, the ground points have a scaling factor applied, while the project's coordinate system is defined without that scaling factor, leading to discrepancies in accuracy"* (https://support.virtual-surveyor.com/support/solutions/articles/1000324711-average-error-rmse-accuracy-metrics). Any mismatch between the CRS/geoid the drone project is built in and the frame your CUAV C-RTK 2HP base defines is a **systematic translation of the whole job** — invisible in the software, catastrophic on the ground. This is the failure mode most likely to bite you in production, and it is the one most easily caught by a handful of pole-shot checkpoints.

**6. Temporal drift.** The ortho is a snapshot. Between flight and paint you get: parked and moving vehicles occluding the carriageway, milling/resurfacing, temporary works, sweeping, monsoon washout, chalk-line reference marks erased. On an active Indian carriageway the gap between survey and marking is where geometry silently goes stale. The pole survey has the same problem in principle but a far shorter half-life in practice.

**7. The decisive point: for *pre*-marking there is often nothing to trace.** Digitising from an ortho assumes the feature exists in the image. That works for **re-marking** (the faded old line is visible — genuinely useful) and for **as-built inventory**. It does not work for **virgin marking on fresh asphalt**, where the geometry must come from the design: centreline alignment, offsets, IRC:35-2015 dash/gap patterns, arrow and legend geometry. In that case the imagery contributes a basemap and a centreline reference, and every dimension that matters comes from the standard and your `path_engine` — not from Virtual Surveyor.

**8. Radiometric conditions.** Wet asphalt, low sun angle, hard shadows from trees or gantries, and worn paint all collapse the contrast you need to see a line edge. Your operating environment (Tamil Nadu, high sun, monsoon) has both the good case and the bad case.

## 3.2 Honest comparison for the DYX workflow

| | **RTK pole (today)** | **Drone + Virtual Surveyor** | **Drone RTK/PPK, direct georeferencing, no GCPs** |
|---|---|---|---|
| Point accuracy (H) | **1.6–1.8 cm observed**; spec 7 mm + 1 ppm RTK, 5 mm + 0.5 ppm PPK | **2.6 cm RMSExy best published corridor case**; 5–13 cm at typical 4–5 cm GSD | Typically 2–5 cm H; **their own Site 2 hit −0.45 m vertically at 0 GCPs** |
| Vertical accuracy | 14 mm + 1 ppm RTK | 4–5 cm RMSE with ≥1 GCP (vendor) | Unreliable — the single biggest failure mode |
| Coverage rate | Slow. Point-by-point, in traffic | Fast, area-wide. 40 ha corridor in 4 short flights | Fastest — no ground control at all |
| Ground time in live traffic | High and dangerous | **Low** — only GCP placement/recovery | **Near zero** |
| Authors *new* design geometry? | Yes — you shoot where the line will go | **Only by tracing what already exists** | Same |
| Captures as-built / existing markings? | Point-sampled, slow | **Yes, comprehensively** | Yes |
| Deliverable to client | Point list | **Ortho + TIN + CAD — visually compelling** | Ortho + models |
| Audit / independent verification | Is the reference | Needs withheld checkpoints | Needs withheld checkpoints; no independent check exists by construction |
| Marginal cost per job | Surveyor day-rate | Pilot + processing time + €100+/mo licence | Pilot + processing |

**When each wins, for road pre-marking specifically:**

- **Pole wins** for anything the paint spec touches: stop lines, zebra placement, lane-line offsets at tapers and junctions, tie-ins to existing markings, and final verification. It also wins on trust — it *is* the reference frame your rover flies in.
- **Drone + Virtual Surveyor wins** for: corridor context and basemap for DYX_GCS, as-built inventory of existing markings before a re-marking job, quantity take-off (linear metres of line, paint volume, number of arrows/legends), road and lane width inventory, carriageway condition, cross-slope and drainage for planning, corridor topography feeding the four-wheel-drive rover's slope/traction planning, and client-facing deliverables. All of these are 5–10 cm problems, where the tool is genuinely good.
- **Direct georeferencing with no GCPs** wins only where ground access is impossible or unsafe and 5–10 cm is acceptable. **Their own Site 2 data is the argument against ever doing this**: zero GCPs produced a −0.45 m vertical bias that was completely invisible until independent check points exposed it. Always place at least 2 GCPs and always withhold independent checkpoints.

## 3.3 Verdict on the ±2.5 cm question

**Under what conditions would a Virtual Surveyor-authored CSV meet ±2.5 cm?**

If ±2.5 cm is a **95% bound** (the normal reading of a paint spec): **under no realistic conditions.** You would need RMSExy ≈ 1.2 cm, which is roughly half the best result in the published corridor literature at 1.75 cm GSD with 4.3 GCP/km, and that result did not include operator digitising error.

If ±2.5 cm is an **RMSE (1σ)**: it is *theoretically* approachable but not operationally sensible. You would need, all together:

- **GSD ≤ 1.5 cm** → ~55 m AGL with a 20 MP 1" sensor, ~40 m for a strict 1 cm — over a live carriageway,
- **80% forward / 70% side overlap** (above the 80/60 used in the reference study),
- **RTK or PPK camera stations** (never GPS-only),
- **8–10 GCPs per kilometre**, alternating both sides of the road with a pair at each end (Ferrer-González Distribution 4, densified),
- **independent withheld checkpoints** — not GCP residuals — proving RMSExy on every job,
- careful digitising by an experienced operator, at high zoom, on dry asphalt in even light.

At that point you are placing and surveying **8–10 GCPs/km with the pole anyway**. The pole never left the job; it just got demoted to serving the photogrammetry instead of producing the answer directly. **The economics collapse.**

**So: yes, it is structurally a 3–5 cm tool** (and closer to 5–10 cm at the GSDs the vendor itself tests at) **best used for planning, context, and as-built capture, while the pole remains the authority for final geometry.** That is not a criticism of the product — it is doing exactly what it says on the box. The mismatch is that your box says ±2.5 cm horizontal and theirs says "survey-grade vertical terrain models for earthworks."

## 3.4 Recommended architecture (hybrid)

Rather than replace the pole, use the drone to reduce how much pole work is needed and to make the pole work higher-value.

**A. Drone + Virtual Surveyor produces the *context and design intent* layer**
- Fly the corridor, process to ortho + DSM, export the corridor centreline and any existing markings as **DXF polylines** (not CSV).
- Feed the ortho into DYX_GCS as a raster basemap layer — this is a straightforward win for the MapLibre migration you're already evaluating, and gives the operator real situational awareness at 394+ waypoints.

**B. IRC:35-2015 geometry is generated, not traced**
- The dimensional content of a marking — dash/gap, width, arrow proportions — comes from the standard and your existing DXF marking-code library, never from imagery. Imagery only supplies **placement of the alignment**.
- This is important: it means the imagery's precision only has to be good enough to place the *alignment*, and every dimension downstream of that inherits your DXF geometry exactly.

**C. Pole shots become sparse control, not dense geometry**
- Shoot **2 control points per work block (start and end) plus 1 every 200–300 m**, on identifiable features that also appear crisply in the ortho.
- Fit a **similarity/Helmert transform** (or a rigid 2D transform if you want to preserve scale) from the drone-authored geometry onto those pole shots before painting. This collapses the bulk of the photogrammetric error — which is dominated by low-frequency drift, not high-frequency noise — into a residual you can *measure and report*.
- This is the single highest-leverage change: it turns an unbounded, invisible error into a bounded, logged one.

**D. Verification is mandatory and logged**
- Withhold 5–8 independent checkpoints per job. Run Virtual Surveyor's **Check Points** tool for vertical statistics (https://support.virtual-surveyor.com/support/solutions/articles/1000304572-check-points) and do the horizontal comparison manually — because, as established, the tool does not give you horizontal RMSE.
- Log per-job RMSExy and RMSEz alongside your existing rosbag/RTK logs. Over ten jobs you will have your own empirical accuracy envelope, which is worth more than any vendor claim.

**E. Where the pole stays sovereign**
- Junction geometry, stop lines, zebra crossings, tapers, tie-ins to existing markings, and final acceptance. Anything where a 3 cm error is visible to the eye or fails inspection.

### Suggested pilot (≈1 day of field work, 2–3 days total)

Run this before spending anything on a licence — the 14-day trial gives full Peak features.

1. Pick a **300–500 m straight-plus-curve** section of road with existing, visible markings.
2. Fly **twice**: once at ~60 m AGL (~1.6 cm GSD with a 20 MP 1" sensor) and once at ~100 m AGL (~2.7 cm GSD). This isolates the GSD effect directly.
3. Place **8 GCPs**, zigzag both sides, pair at each end. Survey them with the RS3.
4. Separately survey, with the RS3, **20–30 points along the centreline of an existing painted line**, at 15–20 m spacing. This is your ground truth.
5. Process both flights in Terrain Creator with 6 GCPs; withhold 2 as checkpoints.
6. In Virtual Surveyor, **digitise that same painted line** as a polyline on each ortho. Export DXF.
7. Compute the **perpendicular offset** from each RS3 truth point to the digitised polyline. Report mean, RMSE, and 95th percentile — **for both altitudes**.

**Acceptance criteria:** if RMSE(perpendicular) ≤ 1.5 cm and P95 ≤ 2.5 cm at the low altitude, the hybrid path (§3.4A–D) is viable for alignment authoring. If P95 exceeds ~5 cm at either altitude — which is what the literature predicts — you have a definitive, in-house, defensible answer and you stop spending time on it. Either way you gain a reusable measurement harness for evaluating *any* future geometry source, including LiDAR and mobile mapping.

### Questions to put to Virtual Surveyor support before committing

1. **What SfM/MVS engine does Terrain Creator use — in-house or licensed?** (Determines whether Pix4D/Metashape accuracy literature transfers.)
2. **Do you publish any horizontal RMSE from withheld check points?** (Your validation article reports vertical only.)
3. **Is there any way to export a polyline as an ordered vertex list with station/chainage?**
4. **Can vertices be digitised or verified against the original images, rather than only on the orthomosaic?** (The PIX4Dsurvey capability.)
5. **Is a local/site coordinate system supported yet in Terrain Creator?** (Your own KB flags this as unsupported and as an error source.)
6. **Confirm current pricing, billing period, and exactly which tier includes Section Lines + Terrain Creator.**
7. **What are the linux/macOS options, if any?**

---

## Appendix A — Source list

**Vendor primary (marketing)**
- Terrain Creator product page — https://www.virtual-surveyor.com/terrain-creator-from-drone-photos-to-survey-grade-orthomosaics-and-digital-surface-models
- Pricing — https://www.virtual-surveyor.com/pricing
- Vision & mission — https://www.virtual-surveyor.com/vision-mission
- Offices — https://www.virtual-surveyor.com/virtual-surveyor-offices-throughout-the-world

**Vendor primary (support / knowledge base)**
- Road Survey with Section Lines — https://support.virtual-surveyor.com/support/solutions/articles/1000291529-road-survey-with-section-lines
- Section Lines (reference) — https://support.virtual-surveyor.com/support/solutions/articles/1000291621-section-lines
- From Drone Photos to Topographic Surveys — https://support.virtual-surveyor.com/support/solutions/articles/1000318552-from-drone-photos-to-topographic-surveys
- **Terrain Creator Validation** — https://support.virtual-surveyor.com/support/solutions/articles/1000320871-terrain-creator-validation
- Export Survey — https://support.virtual-surveyor.com/support/solutions/articles/1000305567-export-survey
- Export Points as a CSV File — https://support.virtual-surveyor.com/support/solutions/articles/1000271434-export-points-as-a-csv-file
- Extract Points — https://support.virtual-surveyor.com/support/solutions/articles/1000296240-extract-points
- Supported Input File Formats — https://support.virtual-surveyor.com/support/solutions/articles/1000273543-supported-input-file-formats
- Check Points — https://support.virtual-surveyor.com/support/solutions/articles/1000304572-check-points
- Average Error & RMSE — https://support.virtual-surveyor.com/support/solutions/articles/1000324711-average-error-rmse-accuracy-metrics
- Import Points with a CSV File — https://support.virtual-surveyor.com/support/solutions/articles/1000316164-import-points-with-a-csv-file

**Video**
- A road survey with section lines [6 steps] — https://www.youtube.com/watch?v=oy062wAP0o8
- Road Survey with Section Lines (2025 version) — https://www.youtube.com/watch?v=dcVMeW11LHI

**Press / trade coverage**
- v9.0 + Terrain Creator launch — https://amerisurv.com/2023/09/25/virtual-surveyor-unveils-photogrammetry-app-in-major-new-release-of-smart-drone-survey-software/
- v6.1 section lines — https://www.geoweeknews.com/news/virtual-surveyor-6-1-turns-drone-point-clouds-into-cad-models-in-just-a-few-steps
- v7.2 / pricing note — https://www.commercialuavnews.com/surveying/virtual-surveyor-newest-features-deliver-improved-insights-and-worker-s-safety

**Company databases (secondary, treat with caution)**
- Tracxn — https://tracxn.com/d/companies/virtual-surveyor/__-34Mz3zpM6wKwomNDsOM1EkLcJmXLi3U2x3LtFhpyaM
- Crunchbase — https://www.crunchbase.com/organization/virtual-surveyor
- RocketReach — https://rocketreach.co/virtual-surveyor-group-nv-profile_b55870a2f6851ec6
- Belgian registry — https://www.staatsbladmonitor.be/bedrijfsfiche.html?ondernemingsnummer=0871935077

**Peer-reviewed / independent accuracy sources**
- **Ferrer-González, Agüera-Vega, Carvajal-Ramírez & Martínez-Carricondo (2020)**, *Remote Sensing* 12(15):2447 — https://doi.org/10.3390/rs12152447 *(the key corridor study; also the source for the summarised Skarlatos 2013, Agüera-Vega 2017, Forlani 2018, and Sanz-Ablanedo 2018 comparisons)*
- Impact of processing parameters on high-accuracy UAV photogrammetry (2026), *Measurement* — https://www.sciencedirect.com/science/article/pii/S0263224126000242 *(0.5 px manual marking repeatability)*
- Optimal GCP layout for high-precision 3D mapping — https://www.sciencedirect.com/science/article/abs/pii/S0263224125017026
- Accuracy assessment of low-cost UAV photogrammetry — https://www.sciencedirect.com/science/article/pii/S1110016821002544

**Competitor / mechanism references**
- PIX4Dsurvey product — https://www.pix4d.com/product/pix4dsurvey ; vectorisation docs — https://support.pix4d.com/hc/en-us/articles/360033317432
- Pix4D: Distortions and Artifacts in the Orthomosaic — https://support.pix4d.com/hc/en-us/articles/202561099
- Emlid Reach RS3 specification — https://docs.emlid.com/reachrs3/specifications/specs/

---

## Appendix B — Confidence register

| Claim | Confidence | Basis |
|---|---|---|
| HQ, legal entity, product feature set, workflow steps, export formats & CSV schema | **High** | Vendor primary docs, current |
| Vendor vertical RMSE 4–5 cm with GCPs | **High** (as a vendor claim) | Vendor validation article; internally inconsistent in places |
| Vendor horizontal accuracy | **No number exists** | Vendor's own horizontal section contains no measurements |
| RMSExy 2.6–2.8 cm best corridor case @ 1.75 cm GSD | **High** | Peer-reviewed, withheld check points, methodology stated in full |
| 1–2× GSD horizontal rule of thumb | **Medium-high** | Multiple independent studies converge; Sanz-Ablanedo ~2× GSD |
| Combined digitising budget (§2.2d table) | **Medium** | My arithmetic from cited components; the 1 px pointing assumption is an estimate, not a measurement — **the pilot in §3.4 measures it directly** |
| Terrain Creator's SfM engine | **Unverified** | Not disclosed anywhere public |
| Team size ~9 | **Low** | Scraped aggregator only |
| Exact pricing / billing period | **Low** | Page renders ambiguously; confirm with sales |
| Competitor taxonomy | **Medium** | Vendor pages solid; round-up articles are SEO content, not benchmarks |
