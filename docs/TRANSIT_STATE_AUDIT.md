# Transit / Mark State — Production-Grade Audit

Scope: how the pipeline determines **MARK (spray ON)** vs **TRANSIT (spray OFF)**
state, and whether the **shape→shape / group→group transition path** is built to
production standard. Verified against the real `path_engine` (identical to the
deployed server) via `tools/audit_transit_state.py`.

---

## A. How MARK/TRANSIT state is actually determined

The spray decision is **1:1 with `SegmentType`**, decided in four stages:

1. **Classification (per DXF entity)** — [`DXFEntity.classify()`](path_engine/core.py:135)
   maps each entity by layer:
   - annotation layers (`DIM/DEFPOINTS/ANNOT/HATCH`) → **ignore** (dropped, never planned),
   - `TRANSIT/TRAVEL/MOVE/RAPID` → **transit**,
   - everything else → **mark**.
   An explicit `layer_mapping` overrides. Ignored entities are filtered out in
   [`entities_to_segments()`](path_engine/parsers/dxf_parser.py:514) before any planning.

2. **Segment typing** — surviving entities become `SegmentType.MARK` or
   `SegmentType.TRANSIT` ([dxf_parser.py:520](path_engine/parsers/dxf_parser.py:520)).

3. **Extensions & connectors are always TRANSIT** — PRE/AFT run-ups and every
   inter-shape connector are emitted as `SegmentType.TRANSIT`
   ([`_insert_transit_connectors_between_segments`](path_engine/engine.py:71)),
   i.e. spray OFF by construction.

4. **Merge → spray flags** — [engine.py:1024‑1040](path_engine/engine.py:1024):
   ```python
   is_mark = seg.segment_type == SegmentType.MARK
   ...
   spray_flags.append(is_mark)          # spray ON  ⇔  MARK
   ```
   Junction de-dup collapses coincident points **only when they share the same
   spray state** (`spray_flags[-1] == is_mark`), so a MARK↔TRANSIT boundary point
   is never removed — the spray ON→OFF edge is preserved. A `close_loop` leg is
   forced spray OFF ([engine.py:1061](path_engine/engine.py:1061)).

**Net:** spray is ON *iff* the point came from CAD mark geometry; run-ups,
run-outs, inter-shape travel, and loop-closing legs are all OFF.

---

## B. How shape→shape / group→group transitions are built

1. **Grouping** — [`group_connected_segments()`](path_engine/optimizers/shape_grouping.py:1)
   chains connected line-like MARK primitives into one composite run per shape
   (shared endpoints within `group_join_tol_m = 0.05`). Curves (circle/arc) stay
   their own group so smooth profiles survive. A **shape = a group.**

2. **Ordering (TSP)** — [`optimize_segment_order()`](path_engine/optimizers/segment_order.py:344)
   orders whole groups with nearest-neighbour + 2-opt + or-opt. A
   [`_PaintAwareness`](path_engine/optimizers/segment_order.py:86) penalty costs any
   connector that drives over **already-laid** paint (crossing not-yet-marked
   geometry is free — this is what lets an enclosed shape be marked first).
   Transit links are **withheld here** when extensions are on.

3. **Extensions** applied per edge/group (PRE→MARK→AFT).

4. **Single routing pass** — [`_insert_transit_connectors_between_segments`](path_engine/engine.py:71)
   inserts one explicit TRANSIT spanning **AFT-tip → next PRE-start** wherever a
   gap exists. Doing this *after* extensions (not before) avoids the old
   double-180°-reversal defect. Connectors are then densified (step 5b).

Result: the `/path` polyline is fully explicit — every metre of inter-shape
travel is a real, spray-OFF TRANSIT segment; there are no implicit jumps.

---

## C. Empirical audit — 5 invariants × 6 multi-shape files

| Invariant | Meaning | Result |
|---|---|---|
| **I1 State integrity** | spray ON ⇔ MARK; all extensions/connectors TRANSIT; Σ MARK-seg length == plan mark length | ✅ all files |
| **I2 Continuity** | zero inter-segment gaps > 0.01 m (no hidden jumps in `/path`) | ✅ all files |
| **I3 Connector routing** | every connector spans prev-tip → next-start and is TRANSIT | ✅ all files |
| **I4 Wet-paint safety** | no connector crosses a MARK line painted earlier | ✅ all files |
| **I5 Transition flips** | spray flips exactly at MARK↔TRANSIT boundaries; boundaries preserved | ✅ all files |

Files: `multi shape 2`, `square and triangle 1.5m`, `star_3x3m`, `square_circle`,
`square_line`, `triangle 3x3`. **All invariants PASS.**

Example — `multi shape 2` (circle+square+triangle, 27 segments, 741 waypoints):
mark length 24.53 m == plan 24.53 m; 0 hidden gaps; 6 inter-shape connectors, 0
mis-routed; 0 wet-paint crossings; 14 spray transitions across 7 mark runs.

**Detector validation (non-vacuous):** on a synthetic small-square-enclosed-in-a-
large-square, naive order produces **1** wet-paint crossing; the production
optimizer reorders (inner first) to **0**. So I4 both detects real crossings and
confirms the optimizer eliminates them.

Robustness also observed: coincident duplicate geometry is detected and dropped
(`LINE_66 coincident with LINE_63 — dropping it`) to avoid double-marking.

---

## Verdict

The MARK/TRANSIT state machine and the shape/group transition path are
**production-grade**: state is a deterministic 1:1 function of segment type, the
spray boundary is preserved through de-dup, all inter-shape travel is explicit
and spray-OFF, connectors route tip-to-tip without wasted reversals, and the
router actively avoids driving over wet paint. No defects found.
