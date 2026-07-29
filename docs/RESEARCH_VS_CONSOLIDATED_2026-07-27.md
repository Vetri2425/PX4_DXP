# Virtual Surveyor — Consolidated Assessment (3 independent reports)

**For:** DYX pre-line marking rover — learning phase, session 2026-07-28
**Sources consolidated:** Opus 5, Grok 4.5, DeepSeek v4 (all researched 2026-07-27,
independently, same brief). Companion doc: `RESEARCH_SURVEY_WORKFLOW.md`
(Cursor, 45 sources — the wider survey-methods study this drills into).
**Question:** can drone + Virtual Surveyor author paint-line geometry at ±2.5 cm,
replacing or complementing the RTK pole?

---

## 1. The unanimous verdict — three models, zero disagreement

**Virtual Surveyor is structurally a 3–5 cm horizontal tool for line work**
(Opus: 3–8 cm, and 5–10 cm at the GSDs the vendor itself tests at). **It cannot
author paint-line geometry to ±2.5 cm. The RTK pole remains the geometric
authority; the drone layer is a complement for planning, context, and as-built
capture.**

All three reports reached this independently. When three different models with
different sources and different reasoning styles converge on the same number,
that is as settled as desk research gets. The remaining uncertainty is not
"is this right" but "exactly how far off ±2.5 cm is it on OUR site" — which only
the §6 pilot measures.

## 2. The five decisive facts (strongest evidence across all three)

1. **The vendor publishes NO horizontal accuracy number. At all.** (Opus —
   verified against the vendor's own validation article.) Their flagship
   accuracy study reports vertical RMSE only (4–5 cm with ≥1 GCP; 2 cm on
   asphalt); horizontal was "verified visually" against a ground marker at
   5 cm GSD. For a *horizontal* paint spec, the central vendor claim is absent.
   "Survey-grade" is never defined numerically anywhere in their materials.

2. **Best published corridor result: RMSExy = 2.6 cm** — Ferrer-González et al.
   2020, *Remote Sensing* 12(15):2447: 2.1 km road, 1.75 cm GSD (65 m AGL),
   9–11 GCPs zigzagged both sides. That is careful academic practice on exactly
   our geometry, and it *already equals our tolerance as a 1σ statistic* —
   before adding operator pointing error (~0.5–1 px) and before the rover's own
   control error. Rule of thumb corroborated by 3,465-configuration study:
   horizontal accuracy converges to **~2× GSD**.

3. **Architecture: it is a 2.5D raster environment.** (Opus) Everything —
   even imported LiDAR — is rasterised. You always digitise on a flattened
   orthomosaic, never against the original photos or a true 3D cloud. Every
   drawn vertex inherits the full orthorectification error budget with no way
   to verify against imagery. (PIX4Dsurvey has exactly that verification —
   if we ever revisit this class of tool, evaluate PIX4Dsurvey first.)

4. **The "6-steps" road workflow does not produce marking geometry.** It builds
   a TIN of the road surface at 30 m stations / 3 m cross-section spacing — an
   earthworks deliverable ~3 orders of magnitude coarser than a paint line.
   Marking geometry would have to be hand-traced as polylines, a workflow the
   vendor does not even document.

5. **CSV export drops polyline vertices** (all three found this). Points-only;
   connectivity, order and chainage are destroyed unless you run Extract
   Points first, and even then order is not guaranteed. **If we ever ingest
   from this tool: take the DXF, never the CSV** — DXF preserves vertex order,
   arcs, and layer names (which map onto IRC:35 marking codes), and our
   path_engine does the densification better than their 3 m Densify tool.

## 3. The one genuinely new decision-question (Opus)

> **Is our ±2.5 cm a 1σ RMSE or a 95% bound?**

A paint spec normally reads as a bound; a bound reading requires RMSE ≈ 1.2 cm
from the geometry source — half the best published corridor result. This
question is not about Virtual Surveyor at all: **it defines what WE promise a
client**, and it should be answered and written into the product spec before
the next customer conversation. Note our own field grading (2026-07-27) reports
per-station miss and RMS — we should state which statistic our "≤2.5 cm"
claims use. (Agency context from `RESEARCH_SURVEY_WORKFLOW.md`: QLD/WA ±15 mm,
UK ±25 mm, NCDOT ½″ — the bar we quote against.)

## 4. Conflict register (resolved)

| Item | Grok | DeepSeek | Opus | Resolution |
|---|---|---|---|---|
| Pricing | € (Ridge 100 / Mtn 210 / Peak 210) | $ (155 / 230 / 270) | Same € page, flags Mtn=Peak as a **billing-toggle render artefact** | Opus explains the disagreement: the page's monthly/annual toggle doesn't render to scrapers. **Do not quote any figure; confirm with sales.** Order of magnitude: ~€100–270/mo, Ridge minimum for our needs |
| Countries | 88 | 78 | notes "46" recycled copy | Three numbers, all unaudited vendor marketing. Ignore |
| Team size | ~7 | ~7 (also cites 2) | ~9 (low confidence) | Small team, <10. Sole material fact: vendor-viability risk for a workflow dependency |
| HQ | Aarschot | Aarschot | Aarschot (resolves Tracxn's "Leuven" as stale) | Aarschot, Belgium. Bootstrapped, ~$770k ARR |
| SfM engine | own (asserted) | own (asserted) | **undisclosed — flagged unverifiable** | Opus is right: nobody found a source. Vendor question #1 |

Lesson for future multi-model research: the two cheaper reports *asserted*
where Opus *verified or flagged* — the pricing artefact and the missing
horizontal RMSE were only caught by reading the primary pages critically.
Convergent verdicts are trustworthy; convergent *details* are not.

## 5. Adopted architecture (if/when a drone layer is added)

The hybrid from Opus §3.4, endorsed by all three in weaker forms:

1. **Drone + VS = context layer**: corridor ortho as a DYX_GCS basemap,
   as-built inventory before re-marking jobs, quantity take-off. All 5–10 cm
   problems where the tool is genuinely good.
2. **Design geometry is GENERATED, never traced**: dash/gap/width/arrows come
   from IRC:35 + our DXF library; imagery only places the alignment.
3. **Pole shots become sparse control**: 2 per work block + 1 per 200–300 m;
   **Helmert/rigid-fit** the drone-authored alignment onto pole shots before
   painting — converts unbounded invisible photogrammetric drift into a
   measured, logged residual.
4. **Withheld checkpoints on every job**, logged next to the rosbags. Horizontal
   comparison done by us — the tool cannot do it.
5. **Pole stays sovereign** for stop lines, zebras, tapers, junctions, tie-ins,
   and acceptance.
6. **Pre-marking on fresh asphalt: the drone contributes nothing but a basemap**
   — there is no line to trace. This is most of our product. (DeepSeek and
   Opus both land on this; it is the quiet killer of the "replace the pole"
   idea independent of accuracy.)

## 6. The pilot that settles it (~1 field day, run before any licence)

From Opus, unmodified — also doubles as a measurement harness for ANY future
geometry source (LiDAR, mobile mapping):

1. 300–500 m road section with existing visible markings.
2. Fly twice: ~60 m AGL (~1.6 cm GSD) and ~100 m (~2.7 cm) — isolates GSD.
3. 8 GCPs, zigzag both sides, pair at each end, surveyed with the RS3.
4. RS3-survey 20–30 points along an existing painted line (15–20 m spacing) = truth.
5. Terrain Creator with 6 GCPs, 2 withheld as checkpoints (14-day Peak trial — free).
6. Digitise the same line on both orthos; export **DXF**.
7. Perpendicular offset truth→polyline: mean, RMSE, P95, both altitudes.

**Accept** the hybrid path if RMSE ≤ 1.5 cm AND P95 ≤ 2.5 cm at low altitude.
The literature predicts failure (P95 ~5 cm+) — either way we get a defensible
in-house answer and a reusable harness.

## 7. Learning-phase takeaways for 2026-07-28 (parallel to field work)

- [ ] **Decide and write down: our ±2.5 cm is RMS or P95?** (§3 — product spec,
      10 minutes, highest leverage per minute of anything in this report)
- [ ] Adopt the **survey SOP upgrades** from `RESEARCH_SURVEY_WORKFLOW.md` for
      the next ground-truth survey (tomorrow's B6 mark re-survey qualifies):
      bipod, tilt OFF, FIX-only, longer occupations, repeat critical stations —
      our 1.6–1.8 cm single-epoch truth is currently our accuracy floor.
- [ ] File the vendor-question list (Opus §3.4) — only if we ever pursue the
      pilot; no licence spend before the pilot passes.
- [ ] Note PIX4Dsurvey as the stronger candidate in this tool class (3D
      vectorisation with image verification) if the drone layer ever matters.
- [ ] No code changes follow from this research. The CSV → densify → drive
      pipeline is confirmed as the right primary input by all three reports
      plus the peer workflow study (TinySurveyor, CivDot take CSV+DXF).

**Status:** research phase CLOSED for Virtual Surveyor unless the pilot is
commissioned. The open thread from `RESEARCH_SURVEY_WORKFLOW.md` that still
matters to the core product is survey SOP (sub-1.5 cm truth), not authoring
tools.
