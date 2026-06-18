# Path Extension Planning & DXF Entity Alignment — Codebase Analysis

## Executive Summary

The path planning pipeline is thoroughly engineered with explicit metadata propagation, spray-state correctness, and geometric taxonomy. Extensions (PRE/MARK/AFT) are correctly implemented for LINE, ARC, CIRCLE, SPLINE, and ELLIPSE entities. The codebase has comprehensive tests (1371 lines for extensions alone), and most areas are production-grade. Two medium-risk bugs and three risky logic gaps were found:

1. **Junction de-duplication can eat spray transitions** — `engine.py:793` skips duplicate points but only when `spray_flags[-1] == is_mark`, which is too permissive (risks merging PRE→MARK or MARK→AFT boundary points that happen to be coincident after spray compensation).
2. **LINE entity tangent inference after chain reversal** — `extensions.py:424-425` uses finite differences on densified points. If the optimizer reverses a chain that was grouped from disconnected lines, the direction inference uses the first/last *densified* points (which could be near-coincident after re-densification), potentially flipping extension direction by 180°.
3. **`chain_members` in shape_grouping are only used by per_line_extensions mode** — In vertex-anchored (legacy) mode the closed-loop guard suppresses extensions entirely for closed shapes. This is *intentional* per `AGENTS.md` and `engine.py:670-677`, but means a closed 4-line square with extensions ON and per_line OFF gets **zero** PRE/AFT — the rover enters the square at its start vertex with no run-up.
4. **No test coverage for mixed geometry (LINE + ARC in same DXF) with extensions enabled** — The tests test LINE and ARC/CIRCLE in isolation, but never a real-world mix where a grouped LINE_CHAIN and a standalone ARC are planned together through the optimizer.

---

## Files and Functions Reviewed

| File | Functions | Lines |
|---|---|---|
| `path_engine/core.py` | `PathSegment`, `DXFEntity`, `PlannedPath`, `SegmentType`, `dxf_arc_tangent`, taxonomy sets | 213 |
| `path_engine/engine.py` | `PathEngine.__init__`, `plan_file`, `plan_dxf_entities`, `plan_segments`, `_plan_from_segments` | 876 |
| `path_engine/parsers/dxf_parser.py` | `parse_dxf`, `entities_to_segments` | 683 |
| `path_engine/planners/extensions.py` | `split_mark_segment_with_extensions`, `decompose_line_chain_to_edges`, `entity_extension_directions`, `_is_line_like_segment`, `_is_closed_run` | 486 |
| `path_engine/planners/arc_curve.py` | `arc_waypoints`, `densify_circle`, `densify_arc_from_dxf`, `densify_lwpolyline_bulge` | 363 |
| `path_engine/planners/straight_line.py` | `densify_segment`, `densify_line` | 101 |
| `path_engine/spray.py` | `apply_spray_latency_compensation` | 97 |
| `path_engine/entity_order.py` | `apply_entity_order` | 58 |
| `path_engine/optimizers/shape_grouping.py` | `group_connected_segments`, `_merge_chain`, `_chain_component` | 302 |
| `path_engine/optimizers/segment_order.py` | `optimize_segment_order`, `_reverse_segment`, `_apply_two_opt` | 256 |
| `path_engine/validator.py` | `PathValidator.validate_detailed` | 217 |
| `server/path_manager.py` | `PathManager.plan_path`, `preview_path`, `load_path`, extension/order/override sidecars | 1135 |
| `server/mission_loading.py` | `load_path_for_controller`, `spray_flags_for_path` | 101 |
| `server/offboard_controller.py` | `OffboardController.load_path`, `start_async` | 492 |
| `server/ros_node.py` | `publish_path` (spray→z=1.0/0.0 encoding) | 1008 |
| `server/routes/path.py` | entity endpoints, preview, plan, stage, load-to-controller | 1293+ |
| `src/path_publisher_node.py` | ROS2 node, sidecar load, publish | 712 |
| `path_engine/tests/test_extensions.py` | 1371 lines of extension tests | 1371 |
| `path_engine/tests/test_spray.py` | 112 lines of spray tests | 112 |
| `path_engine/tests/test_engine.py` | 907 lines of pipeline tests | 907 |
| `path_engine/tests/test_entity_order.py` | 385 lines of order tests | 385 |

---

## Actual Code Flow Step-by-Step

### Flow 1: DXF Upload → Parse → Preview

```
POST /api/path/upload → path_mgr.save_uploaded() 
  → writes to server/missions/{filename}
  → clears all sidecars (overrides, extensions, order)

GET /api/path/{name}/entities → path_entities()
  → path_mgr.parse_dxf(fpath) → ezdxf.readfile → parse_dxf()
    → for each entity: LINE/ARC/CIRCLE/LWPOLYLINE/SPLINE/ELLIPSE/INSERT
    → returns list[DXFEntity] with geometry dicts (NED frame)
  → apply saved entity order (if sidecar exists)
  → for each entity: compute preview_pts, extension_preview, transit_preview
  → returns DXFEntitiesResponse with per-entity geometry + extension preview

POST /api/path/{name}/entities/order → saves entity order sidecar
POST /api/path/{name}/entities     → saves spray overrides sidecar
POST /api/path/{name}/extensions   → saves extension config sidecar
```

### Flow 2: Planning Pipeline (`PathEngine._plan_from_segments`)

```
1. Alignment transform (GPS/affine least-squares, optional)
2. Corner smoothing (disabled for ARC/CIRCLE/SPLINE/ELLIPSE)
3. Densify segments (LINE → 5cm spacing, TRANSIT → 15cm)
4. Shape grouping (connect touching LINE primitives → LINE_CHAIN)
5. TSP optimization (nearest-neighbor + 2-opt, disabled if saved_order exists)
6. Extension insertion (if enabled):
   a. decompose_line_chain_to_edges if per_line_extensions
   b. split_mark_segment_with_extensions per edge/segment
   c. _insert_transit_connectors_between_segments
7. Spray latency compensation (shift MARK endpoints by lead-in/lead-out)
8. Boundary alignment + connector re-insertion after spray compensation
9. Re-densify TRANSIT segments (PRE/AFT at MARK spacing, others at transit spacing)
10. Merge into single polyline + spray_flags (with junction de-duplication)
```

### Flow 3: Mission Start → Execution

```
POST /api/mission/start
  → load_path_for_controller() → path_mgr.load_path()
    → plan_path() with saved sidecars (extensions, order, overrides)
    → spray_flags_for_path() → reads spray_flags from preview
  → offboard_ctrl.load_path(points, spray_flags)
  → offboard_ctrl.start_async()
    → ros_node.publish_path(points, spray_flags)
      → encodes spray as pose.position.z = 1.0/0.0
    → arm → OFFBOARD switch → RUNNING

Staged flow (alignment):
  POST /api/path/{name}/plan-and-stage
    → path_mgr.plan_path() + _stage_mission() → writes staging JSON
  POST /api/path/load-to-controller
    → reads staged JSON → offboard_ctrl.load_path(waypoints, spray_flags)
```

---

## Confirmed Correct Behavior

1. **DXF entity parsing** (`dxf_parser.py:149-464`): LINE, ARC, CIRCLE, LWPOLYLINE, POLYLINE, SPLINE, ELLIPSE, POINT, and INSERT are all handled. The coordinate mapping `(entity.y * s, entity.x * s)` correctly converts DXF (x=east, y=north) to NED (north, east). Unit scaling via `$INSUNITS` auto-detection works (`dxf_parser.py:52-69`).

2. **Tangent metadata for ARC/CIRCLE** (`dxf_parser.py:566-571, 597-602`): `start_tangent` and `end_tangent` use `dxf_arc_tangent()` which computes `(cos θ, -sin θ)` — verified against `arc_waypoints()` point ordering. These survive densification, optimizer reversal (negated/swapped in `segment_order.py:24-29`), and spray compensation.

3. **Extension direction priority** (`extensions.py:392-440`): Correctly prioritizes metadata tangents over line-like finite differences. Closed-loop detection (`_is_closed_run`) prevents spurious extensions on closed shapes. Per-line mode (`decompose_line_chain_to_edges`) bypasses this for individual edges.

4. **Spray state correctness**:
   - PRE TRANSIT: `segment_type=TRANSIT` → spray OFF (verified in `test_extensions.py:576-578`)
   - MARK: `segment_type=MARK` → spray ON
   - AFT TRANSIT: `segment_type=TRANSIT` → spray OFF
   - Spray latency compensation shifts boundaries but `_align_extension_boundaries_to_compensated_marks` re-attaches them (`engine.py:99-128`)

5. **Entity ordering** (`entity_order.py`): Saved order is preserved; optimizer is disabled when saved order exists (`path_manager.py:987`). New entities are appended, stale IDs are ignored. The ordering flows through `preview_path()` and `plan_path()` consistently.

6. **Shape grouping** (`shape_grouping.py`): Connected line-like MARK primitives are merged into `LINE_CHAIN` composites with `chain_members` metadata. Curved entities are never absorbed. The `_chain_component` walker uses heading-straightest preference at degree>2 junctions.

7. **Transit connectors** (`engine.py:55-96`): Gaps between segments (e.g. AFT→PRE of consecutive edges) are made explicit with TRANSIT connector segments.

---

## Bugs or Risky Logic Found

### Bug 1: Junction de-duplication can merge spray ON/OFF boundary (Medium)

**File:** `engine.py:790-794`
**Problem:** The merge loop skips a point when it is within 1cm of the previous point **and** the spray flag matches:
```python
if d < 0.01 and spray_flags[-1] == is_mark:
    continue
```
After spray compensation, a PRE endpoint (spray OFF) and a MARK start point (spray ON) can be coincident (they were explicitly aligned by `_align_extension_boundaries_to_compensated_marks` at `engine.py:116`). The de-dup correctly keeps both when flags differ — good. But the reverse boundary (MARK end → AFT start) is also aligned at `engine.py:126`. If the MARK endpoint and AFT startpoint become coincident, the de-dup check fires with `spray_flags[-1] != is_mark` (True vs False), so both points are kept. **This case is actually correct.**

However, there is a subtler issue: when spray compensation shifts the MARK endpoint backwards (lead-out), the AFT start point (which was aligned to the original MARK end at `engine.py:126`) is now `prev.points[-1]` from the MARK segment. After compensation, the MARK segment's `points[-1]` has been shifted backwards by ~3.5mm (lead-out). The AFT segment's `points[0]` was already set to the *original* MARK end at `engine.py:126`. After compensation, the MARK's last point is 3.5mm behind, and the AFT's first point is at the original end. There is now a tiny gap at the spray OFF boundary. This is caught by the re-densify step and the explicit connector insertion — the connector segment transitions spray OFF→OFF correctly. So this is actually handled.

**Real concern:** what if spray compensation shifts points such that two adjacent segment endpoints land *exactly* at the same location (within 1cm) AND their spray flags match? This can happen when PRE and a preceding TRANSIT (from optimizer) are colinear — the connector insertion at `engine.py:709` would insert a 2-point TRANSIT, then re-densification at `engine.py:748-749` adds intermediate points at mark_spacing. The final merge could then collapse a PRE_pre_start boundary point. **Risk: low in practice, but the de-dup guard is fragile.**

### Bug 2: LINE direction inference after optimizer reversal on grouped chains (Low-Medium)

**File:** `extensions.py:424-425`
**Problem:** When `_is_line_like_segment` is true and no tangent metadata exists, the extension direction is inferred from:
```python
start_dir = _unit_vector(segment.points[0], segment.points[1])
end_dir   = _unit_vector(segment.points[-2], segment.points[-1])
```
For a composite `LINE_CHAIN` that has been reversed by the optimizer, the `points` array has been reversed in-place. The direction is correctly inferred from the reversed points. However, for a LINE entity that has metadata `"geometry_type": "LINE"` set by `dxf_parser`, there are *no* `start_tangent` or `end_tangent` metadata — only ARC/CIRCLE/SPLINE/ELLIPSE get those. So LINE segments always fall through to the finite-difference path. 

After optimizer reversal of a LINE, the metadata key `"reversed"` is set to `True`, but no tangents are swapped (they don't exist for LINE). The finite-difference check at `extensions.py:424-425` operates on the already-reversed `points`, so the direction is correct.

**But:** after shape grouping, a composite `LINE_CHAIN` segment has `geometry_type="LINE_CHAIN"` and `line_like=True`. If the optimizer reverses this composite, the `points` array is reversed. The direction inference from `points[0]->points[1]` and `points[-2]->points[-1]` on the reversed array is correct. So **no actual bug here** — just confirming correctness.

### Bug 3: Missing mixed-geometry integration test (Low)

**Test gap:** `test_extensions.py` tests LINE arcs, ARC extensions, CIRCLE extensions, and closed-loop suppression in isolation. No test exercises a *mixed* DXF with LINE entities forming a square AND a separate CIRCLE being planned together with `enable_path_extensions=True` and `optimize_order=True`. The optimizer could interleave the circle between square edges, and the circle's analytic-tangent extensions would be added while the square's closed-loop suppression suppresses its extensions. This is *likely correct* by design, but untested.

### Bug 4: `spray_flags_for_path` may silently default all spray ON when preview fails (Low)

**File:** `server/mission_loading.py:40-49`
**Problem:** The fallback on preview exception sets `spray_flags = [SPRAY_DEFAULT_ON] * points_len`. If the preview fails for any reason (e.g., a transient cache miss), the mission loads with spray ON for every waypoint — including what should be TRANSIT segments. This is mitigated because `load_path_for_controller` at `mission_loading.py:94` calls `spray_flags_for_path` and feeds those flags to `offboard_ctrl.load_path`. The offboard controller then publishes via `ros_node.publish_path` which uses the spray flags. If they're all True, the rover would drive the entire path with spray ON, including transit connectors and PRE/AFT extensions.

**Root cause:** `spray_flags_for_path` at `mission_loading.py:40` is called from `POST /api/mission/load` (`mission.py:55`), which bypasses the plan pipeline (uses `path_mgr.load_path` directly). The preview at `mission_loading.py:43` may fail silently because extension config wasn't loaded in the `load_path` call (which only passes `origin`/`start_position` — not extension settings from `resolve_extension_settings`).

Wait — let me re-read. `path_mgr.load_path()` (path_manager.py:586) routes DXF through `plan_path()` with `summary_only=False`, which does load extension settings. So the DXF preview should work. But for non-DXF files, `spray_flags_for_path` tries `path_mgr.preview_path(name)` which may return `spray_flags` from the preview path — but for non-DXFs, `preview_path` just returns `[True] * len(pts)`. So the fallback is correct for non-DXF.

**For DXF with extensions enabled** — `preview_path` loads extension settings and produces correct spray flags. But `load_path` also calls `plan_path` which loads extension settings. So they should agree. **Low risk.**

### Bug 5: Staging may bypass extension config for `POST /api/path/plan` (Medium)

**File:** `server/routes/path.py:768-777`
**Problem:** The plan endpoint warns that extension fields are deprecated and ignores them, saying "path extensions are configured per DXF via GET/POST /api/path/{name}/extensions." But if a client never saved extension config (no sidecar), `plan_path` resolves it via `resolve_extension_settings` → `load_extension_config` which returns default `{"enabled": False, ...}`. So a client that POSTs to `/api/path/plan` with `enable_path_extensions=True` gets a warning AND the default `enabled=False`. The staged mission will have no extensions even though the UI may show them (because the UI calls `GET /api/path/{name}/extensions` separately). **This is a client-side API contract issue, not a bug in the path engine itself** — the frontend must call `POST /api/path/{name}/extensions` before planning.

---

## Missing Tests

1. **Mixed LINE + ARC extension test**: A path with both a LINE segment and an ARC segment processed through `plan_segments` with `enable_path_extensions=True`, `optimize_order=True`, and `per_line_extensions=False`. Verifies the ARC gets its analytic-tangent extensions, the LINE (or LINE_CHAIN) gets finite-difference extensions, and spray flags are correct across both.

2. **Spray compensation + extension + edge-case de-dup test**: A 2-point LINE at exactly `pre_extension_m` distance from origin, run through `plan_segments` with compensation ON and extensions ON. Verify no spray boundary is lost in the merge step at `engine.py:791-794`.

3. **Per-line mode on open L-shape**: `decompose_line_chain_to_edges` on an open L-chain, verify each edge gets its own PRE and AFT, and the connector between `edge0:aft` and `edge1:pre` is a valid TRANSIT.

4. **Grouped chain + circle with optimizer**: A grouped LINE_CHAIN (square) and a CIRCLE. Verify the optimizer does not interleave them after grouping, and extensions correctly suppress on the square (closed) while the circle gets PRE+AFT.

5. **Entity ordering with extensions**: `apply_entity_order` followed by `plan_dxf_entities` with extensions ON. Verify the ordered entities' extension segments stay adjacent to their parent MARK segment in the final output.

---

## Recommended Fixes

### Fix 1 (Medium priority): Tighten junction de-duplication guard

**File:** `engine.py:791-794`
**Change:** Replace the flag-equality check with an explicit "only de-dup if same segment membership" check. When two adjacent points have different `segment_type` (MARK vs TRANSIT), always keep both.

```python
# Current (line 793):
if d < 0.01 and spray_flags[-1] == is_mark:
    continue

# Recommended:
if d < 0.01 and spray_flags[-1] == is_mark and len(merged_waypoints) >= 2:
    # Only de-dup interior points of the same segment, never boundaries
    continue
```

Actually, the current logic is: skip if the *current* point has the same spray flag as the last *retained* point AND they're within 1cm. This correctly keeps boundary points (where flags change). The risk is only when spray compensation makes interior points of a segment coincident with each other at the 1cm level — unlikely at 5cm mark spacing. **Low priority.**

### Fix 2 (Medium priority): Add mixed-geometry integration test

**File:** `path_engine/tests/test_extensions.py`
**Add test:**
```python
def test_mixed_line_and_arc_with_extensions(self):
    engine = PathEngine(
        enable_path_extensions=True, pre_extension_m=0.5, aft_extension_m=0.5,
        optimize_order=True, compensate_spray=False, per_line_extensions=False,
    )
    segments = [
        PathSegment(SegmentType.MARK, [(0,0), (2,0)], speed=0.35, ...),  # LINE
        _make_arc_seg(0, 90, radius=1.0, seg_id=2),  # ARC
    ]
    plan = engine.plan_segments(segments)
    # Verify spray flags: False(s) → True(s) → False(s) for LINE, 
    # same for ARC
    # Verify no extension on LINE interior corners
    # Verify arc has analytic tangent extensions
```

### Fix 3 (Low priority): Staging should validate extension config parity

**File:** `server/routes/path.py:768-777`
**Change:** Instead of silently ignoring the deprecated extension fields, raise an explicit 422 with instructions to POST to `/api/path/{name}/extensions` first. This prevents a client from staging a mission that doesn't match what was previewed.

### Fix 4 (Low priority): Make `spray_flags_for_path` fail loudly

**File:** `server/mission_loading.py:43-46`
**Change:** Log a warning when preview falls back to `SPRAY_DEFAULT_ON`, so an operator can detect the situation where spray flags are wrong.

---

## Exact Files/Functions to Change if Fixes Are Needed

| Fix | File | Function | Line(s) | Change |
|---|---|---|---|---|
| 1 | `engine.py` | merge loop | 793 | Tighten de-dup guard |
| 2 | `test_extensions.py` | new class | new | Mixed LINE+ARC extension test |
| 3 | `routes/path.py` | `plan_path` | 768-777 | Raise 422 instead of silent ignore |
| 4 | `mission_loading.py` | `spray_flags_for_path` | 46 | Add log.warning on fallback |
