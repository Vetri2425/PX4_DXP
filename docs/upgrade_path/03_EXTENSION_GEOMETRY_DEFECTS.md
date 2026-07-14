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

## Suggested order

1. **E2** — clamp extension to line length. Cheapest, biggest single win, kills the 7.4× case.
2. **E1** — make extensions geometry-aware (shorten/drop on collision). Highest severity: it is
   the one that actually damages paint.
3. **E3** — make connectors paint-aware.
4. **E5** — densify connectors at `mark_spacing`. One-line change.
5. **E4** — report the swept bbox; add a site-boundary check.
6. **E6** — falls out of 1–3.

---

## Reproduce

```bash
python3 tools/preview_extension_path.py --all --per-line     # raw vs extended, PNG
python3 tools/simulate_path_flow.py  --dxf star_3x3m.dxf --per-line   # step table + flow PNG
python3 tools/animate_path_flow.py   --dxf star_3x3m.dxf --per-line   # MP4 of the run
```
