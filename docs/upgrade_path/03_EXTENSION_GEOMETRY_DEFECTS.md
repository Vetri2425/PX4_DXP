# Extension Geometry — Defect List

**Date:** 2026-07-14
**Branch:** `Upgrade_extensions` @ `fd2d4ed`+ (deployed to Jetson)
**Method:** all 14 sample DXFs planned by the live backend, geometry checked programmatically
**Tools:** `tools/preview_extension_path.py`, `tools/simulate_path_flow.py`, `tools/animate_path_flow.py`

> Scope: this is the *remaining* defect list **after** the transit-routing, densify-idempotence
> and spray-boundary fixes on this branch. The 180° double-back (d82317d failure mode) is
> **gone** — 0 near-reversals across all 14 shapes.

---

## Summary

> ### STATUS — patched in `3e41ba5`, deployed to the Jetson, 485 tests pass
>
> | # | Defect | Status |
> |---|---|---|
> | **E1** | Extensions blind to other geometry | ✅ **FIXED** — 6 crossings → **0** |
> | **E2** | Extension length not proportional | ✅ **FIXED** — worst 7.4× → **0.92×** |
> | **E3** | Connectors not paint-aware | ✅ **FIXED** — 5 crossings → **1** (`99f1c40`) |
> | **S1** | **Spray was DOUBLE-compensated** — paint started ~9 cm early | ✅ **FIXED** (`99f1c40`) |
> | **E4** | Swept footprint never reported | ✅ **REPORTED** — metadata + operator warning |
> | **E5** | Connectors sampled at `transit_spacing` | ✅ **FIXED** — 14.9 cm → **5.0 cm** |
> | **E6** | Overhead | ✅ **IMPROVED** — star +117% → **+92%** |
>
> **Live backend, `star_3x3m.dxf`:** the two 10 cm crosshair lines now carry **no extension at
> all**; swept area 4.00 × 4.00 m vs marked 3.07 × 3.07 m is **reported** (0.47 m overshoot,
> 4 extensions clamped); marked share of travel 46% → **52%**; near-reversals still **0**.
>
> Original findings retained below for the record.

| # | Defect | Severity | Shapes affected |
|---|---|---|---|
| **E1** | Extensions are **blind to other geometry** — they cross lines the rover paints | 🔴 **Paint damage** | star (6), sct, square_circle, square_line |
| **E2** | Extension length is **not proportional to the line** — 10 cm line gets 97 cm of extension | 🔴 Absurd on short lines | star (7.4×), any short entity |
| **E3** | Transit connectors are **not paint-aware** — cross already-laid paint | 🟠 Paint damage | sct (7), square_circle (3), star (1) |
| **E4** | Extensions grow the working footprint by **exactly 1.0 m**, unchecked | 🟠 Out-of-bounds / collision | **all 14** |
| **E5** | Connectors densified at `transit_spacing` (15 cm) — **coarse right at the 135° pivot** | 🟡 Tracking | all multi-entity |
| **E6** | Overhead up to **+157%** — more travel than marking | 🟡 Throughput | plus, star, square_line |

### ✅ Verified GOOD — do not "fix" these

- **ARC/CIRCLE extensions are tangential.** The run-in to `arc_sector`'s ARC is 1.88° off the
  arc's first chord — and that 1.88° *is* the chord-vs-tangent discretisation angle, i.e. the
  extension follows the **true tangent**. Curved geometry is extended correctly.
- **Zero near-reversals (>170°)** across all 14 shapes. The rover never drives back over itself.
- **Spray flags are exact** — 0 spray-ON waypoints outside MARK geometry, every shape.
- **Densification is idempotent** — 0 double-densified gaps, every shape.
- **Every pass has the correct PRE → MARK → AFT structure**, 16/16 on the star.

---

## E1 — 🔴 Extensions are blind to other geometry

**An extension continues its own line's direction past the endpoint, and is laid down wherever
that lands — including on top of another entity that will be (or has been) painted.**

`star_3x3m.dxf` is the proof. The star's **tips physically touch the square's edges**, so each
tip's extension runs straight across the square's line:

```
AFT of LWPOLYLINE_3:edge1   (1.04, 2.93) -> (1.04, 3.43)
    CROSSES square edge2 at (N 1.04, E 3.00)

PRE of LWPOLYLINE_3:edge2   (0.74, 3.33) -> (1.04, 2.93)
    CROSSES square edge2 at (N 0.98, E 3.00)

PRE of LWPOLYLINE_3:edge4   (3.01, 2.79) -> (2.71, 2.38)
    CROSSES square edge1 at (N 3.00, E 2.78)
...6 in total (3 PRE + 3 AFT)
```

The rover drives its run-up/run-out **directly over the square's lines** — six times.

**Nothing in the planner checks this.** `split_mark_segment_with_extensions()` only knows about
the segment it is extending; it has no view of the rest of the drawing.

| Shape | PRE crossing | AFT crossing | CONN crossing |
|---|---|---|---|
| star_3x3m | **3** | **3** | 1 |
| sct 1.5m | 0 | 0 | 2 |
| square_circle | 0 | 0 | 1 |
| square_line | 0 | 0 | 1 |
| *other 10* | 0 | 0 | 0 |

**Fix:** before emitting a PRE/AFT, test it against all other MARK geometry. On a hit, either
shorten the extension to stop short of the obstruction, or drop it for that end.

---

## E2 — 🔴 Extension length is not proportional to the line

`pre_extension_m` / `aft_extension_m` are **absolute**, so a short line gets the same 0.5 m
run-up as a 3 m one.

`star_3x3m` contains two **10 cm** crosshair lines at its centre:

```
MARK  LINE_15   0.131 m       <- 13 cm of paint
  PRE 0.465 m + AFT 0.503 m   =  0.968 m of extension   -> 7.4x the line
```

**The rover drives nearly a metre to paint a hand's width.** Both crosshair passes do this.

Even the star's own edges are marginal: 1.121 m of line, 0.969 m of extension (**0.86×**).

| Shape | worst ext / line |
|---|---|
| **star_3x3m** | **7.4×** |
| plus | 0.9× |
| arc_sector, sct, square+triangle, trepazoid | 0.6× |
| L_2m, line, square_2m, square_circle, square_line | 0.5× |
| pentagon | 0.4× |
| lune, triangle | 0.3× |

**Fix:** clamp to a fraction of the line — `min(pre_extension_m, k * line_length)` — and skip
per-line extensions entirely below a minimum line length (a 10 cm mark cannot benefit from a
run-up longer than itself).

---

## E3 — 🟠 Transit connectors are not paint-aware

The inter-run connector is a **straight line from the AFT tip to the next PRE start**. It will
happily cross geometry the rover has **already painted**.

`star_3x3m`: after painting the square's left edge (E=0, N=−0.03→3.00, pass #11), the rover
travels diagonally from `(0.000, −0.500)` to `(0.697, 0.967)` and **crosses that fresh line at
N = 0.237**.

Crossings over *already-laid* paint: **sct 1.5m = 7**, **square_circle = 3**, **star = 1**.

**Fix:** penalise connectors that cross marked geometry in the TSP cost, so the router prefers a
route around; or route the connector as a 2-leg detour when a straight shot would cross paint.

---

## E4 — 🟠 Extensions push the rover 0.5 m outside the drawing, unchecked

Measured bounding-box growth, extensions ON vs OFF:

```
ALL 14 SHAPES:  bbox grows by exactly 1.00 m
                (0.5 m PRE at one end + 0.5 m AFT at the other)
```

So the rover **always drives up to 0.5 m outside the DXF's own bounds**. On a real site — a wall,
kerb, fence, parked car, or the edge of the pad — that is an out-of-bounds excursion, and
**nothing warns about it or checks it.**

The operator draws a 3×3 m square and gets a 4×4 m swept area, with no indication in the plan.

**Fix:** report the swept bounding box (with extensions) in `planning_metadata`, surface it in the
preview/app, and optionally accept a site-boundary polygon to validate against.

---

## E5 — 🟡 Connectors are sampled coarsely exactly where tracking is hardest

Connectors are TRANSIT and get `transit_spacing` (**0.15 m**), producing gaps up to **14.9 cm** —
and those gaps sit on the diagonal the rover crosses **between two 135° pivots**. The hardest
point to track is the most sparsely sampled.

| Shape | max connector gap |
|---|---|
| sct 1.5m | 14.9 cm |
| star_3x3m | 14.8 cm |
| square_circle / square_line / square+triangle | 14.4 cm |
| square_2m / plus / L_2m / arc_sector / triangle | 14.1 cm |

PRE/AFT run-ups are already correctly sampled at `mark_spacing` (5 cm) — `engine.py:762-765`
special-cases `extension_role in ("pre","aft")`. **The connectors were not given the same
treatment.**

**Fix:** densify `extension_connector` segments at `mark_spacing` too, or at least tighten the
sampling near each end where the pivot happens.

---

## E6 — 🟡 Throughput: more travel than marking

| Shape | marked | driven | overhead |
|---|---|---|---|
| **plus** | 12.03 m | 31.78 m | **+157%** |
| **star_3x3m** | 23.60 m | 51.15 m | **+117%** |
| square+triangle | 12.87 m | 27.70 m | +115% |
| square_line | 10.16 m | 20.28 m | +100% |
| trepazoid | 6.66 m | 12.71 m | +91% |
| sct 1.5m | 22.33 m | 40.55 m | +82% |
| arc_sector | 5.45 m | 9.77 m | +79% |
| square_2m | 8.13 m | 14.12 m | +74% |
| pentagon | 11.06 m | 19.13 m | +73% |

On `star_3x3m` only **46% of the rover's travel is actually marking**, and it accumulates **425°
of cumulative pivot**.

This is the *inherent* cost of per-line — each line is entered clean and exited clean — but E2
(disproportionate extensions) and E3 (blind connectors) inflate it well beyond what the mode
actually requires. **Fixing E2 and E3 reduces E6 as a side-effect; E6 is not a separate fix.**

---

## What shipped (`3e41ba5`)

**E1** — every extension is ray-cast against all other MARK polylines
(`extensions.py::_ray_first_hit`) and shortened to stop `extension_obstacle_clearance_m`
(10 cm) short of a hit, or dropped if nothing useful survives. Hits within **3 cm of the
extension's own origin are ignored** — where two entities genuinely *touch* in the drawing the
rover must cross the junction anyway, and a tangential departure from a discretised curve nicks
its chord polyline at millimetre scale. The star's real obstructions were 7 cm out and are caught.

**E2** — `pre/aft_extension_m` is now a **ceiling**, not a promise
(`extensions.py::_clamp_extension`): capped at `extension_max_line_fraction` (0.5) of the line,
and lines below `extension_min_line_length_m` (0.30 m) get **no extension at all**.

**E5** — `engine.py` Step 5b now densifies `extension_connector` segments at `mark_spacing`,
same as PRE/AFT.

**E4** — `planning_metadata.extensions` carries `marked_bbox` / `swept_bbox` /
`max_overshoot_m` / `clamped_extensions`, and `PathValidator` warns.

New knobs on `PathEngine`: `extension_max_line_fraction`, `extension_min_line_length_m`,
`extension_obstacle_clearance_m`, `extension_min_useful_m`.

## E3 + S1 (`99f1c40`)

### S1 — spray was double-compensated: paint started ~9 cm early

The planner shifted the MARK boundary **3.5 cm** early to cover solenoid open time
(`spray.py` — a *static* shift that assumes `marking_speed`). But the spray controller
**already does this at runtime, from the rover's ACTUAL speed**:

```python
# spray_controller_node.py:276
on_lead = speed_mps * solenoid_open_delay_s + on_overspray_margin_m
        = 0.35 * 0.10 + 0.02   =  5.5 cm at marking speed
```

Both fired. Paint began **~9 cm before the CAD line**, on every line.

The controller is the right place for it — its lead stays correct when the profile slows into a
corner, or when `marking_speed` changes. The planner's does not. **`compensate_spray` now
defaults OFF** (`PathEngine`, `PathPlanRequest`, `PathManager`), so the plan carries the *true*
geometry. Live backend, `square_2m` at API default:

```
MARK lengths : 2.0000  2.0000  2.0000  2.0000   (CAD = 2.000 m)
spray toggles: 0.00 cm from each CAD corner
```

### E3 — paint-aware routing

The fix is **ordering, not rerouting**, and it turns on one observation: **crossing a line that
has not been painted yet is free.** `star_3x3m`'s crosshairs sit inside both the star and the
square, so *some* connector must cross that geometry — but if the crosshairs are marked **first**,
there is no paint there to cross. `_PaintAwareness` penalises only **already-laid** paint (5 m of
equivalent deadhead per crossing) and the optimizer finds that ordering by itself.

Live proof — the star now marks the two centre crosshairs first, unprompted:

```
first 3 marked: ['LINE_14', 'LINE_15', 'LWPOLYLINE_3:edge9']
wet-paint warnings: NONE
```

**Also required:** in per-line mode the chains are now unfused into edges **before** the TSP
(Step 2c), not after. With the square still fused into one composite chain the optimizer only saw
*two* marks on `square_circle`, and every ordering forced a crossing. With the edges visible it
picks the one square edge reachable from the circle's run-out without crossing anything.
Chain-ends mode is untouched — shape-level traversal is the whole point there.

**Bonus: edge-level ordering also finds shorter routes.**

| Shape | before | after |
|---|---|---|
| square_circle | 23.96 m | **22.69 m** |
| square + triangle | 27.70 m | **25.95 m** |
| square_line | 20.28 m | **18.12 m** |
| sct 1.5m | 40.55 m | **38.71 m** |

The penalty **saturates at 5 m** (15 / 30 / 60 m give identical results), so 5 m is the value.

## Still open

**1 crossing remains, on `sct 1.5m`.** It is a 2-opt local minimum, not a weighting problem —
raising the penalty does not shift it. The validator warns with its coordinates. Fixing it needs a
stronger move set than slice-reversal 2-opt (or-opt / segment insertion), which is a separate
change.

---

## Reproduce

```bash
python3 tools/preview_extension_path.py --all --per-line     # raw vs extended, PNG
python3 tools/simulate_path_flow.py  --dxf star_3x3m.dxf --per-line   # step table + flow PNG
python3 tools/animate_path_flow.py   --dxf star_3x3m.dxf --per-line   # MP4 of the run
```
