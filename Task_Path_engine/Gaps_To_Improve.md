# 🗺️ Path Engine & Mission Planning: Real Gaps Analysis

> **Audit of the path planning pipeline, isolating real gaps from RPP-controller noise.**

---

## 🔍 DISCOVERED GAPS (Critical → Minor)

---

## 🚨 CRITICAL GAPS

---

### GAP 1: No GPS ↔ NED Coordinate Transform in PathEngine

**Problem:**

```python
# path_engine/engine.py - plan_file()
def plan_file(self, filepath: str, origin: tuple[float, float] = (0.0, 0.0), ...):
    # origin is just a NED offset, NOT a GPS reference!
    # There's NO lat/lon → NED conversion anywhere
```

**What's Missing:**

- No `origin_gps: tuple[float, float]` parameter (lat, lon)
- No `geographiclib` integration for GPS → NED transform
- No way to align DXF to real-world GPS coordinates

**Impact:**

- ❌ Cannot use GPS survey points to align templates
- ❌ Manual NED offset guessing required
- ❌ No field deployment workflow exists

**Fix Required:**

```python
# Add to engine.py
def plan_file(
    self,
    filepath: str,
    origin_gps: tuple[float, float] | None = None,  # (lat, lon)
    origin_ned: tuple[float, float] = (0.0, 0.0),   # fallback
    rotation_deg: float = 0.0,  # DXF north → true north
    ...
):
    """
    Args:
        origin_gps: GPS reference (lat, lon). If provided, all waypoints
                    are computed relative to this GPS origin.
        origin_ned: NED offset (meters). Used if origin_gps is None.
        rotation_deg: Rotation to align DXF north with true north.
    """
    if origin_gps:
        # Transform all DXF points through GPS → NED
        from geographiclib.geodesic import Geodesic
        # ... implement transform
```

---

### GAP 2: No Multi-Point GPS Alignment (Rotation + Scale)

**Problem:**

Even if you add `origin_gps`, you still can't handle:

- 🔄 **Rotation**: DXF north ≠ true north
- 📐 **Scale errors**: DXF says "100m" but field is 99.8m
- ✅ **Validation**: No way to check if alignment is correct

**What's Missing:**

```python
# Should exist but doesn't:
def align_template_to_survey(
    dxf_points: list[tuple[float, float]],  # DXF coords
    gps_points: list[tuple[float, float]],  # (lat, lon) survey
) -> tuple[float, float, float, float]:
    """
    Returns: (origin_lat, origin_lon, rotation_deg, scale_factor)
    
    Uses least-squares fit to compute:
    - Best GPS origin
    - Rotation angle (DXF → true north)
    - Scale factor (if DXF units are wrong)
    """
    # Kabsch algorithm or similar
    pass
```

**Impact:**

- ❌ Cannot align rotated templates (e.g., field at 15° to grid north)
- ❌ Cannot detect scale errors (DXF in feet, thought it was meters)
- ❌ No validation (did alignment work? how much error?)

---

### GAP 3: No Closed-Loop Path Handling

**Problem:**

```python
# path_engine/engine.py - _plan_from_segments()
# Merges segments into a single polyline, but:
merged_waypoints: list[tuple[float, float]] = []
for seg in ordered:
    for pt in seg.points:
        merged_waypoints.append(pt)  # Just appends!
```

**Issues:**

- 🔁 Duplicate start/end points on closed loops (square, circle)
- 🔍 No loop closure detection (is last point ≈ first point?)
- 🚫 No "return to start" insertion for open paths

**Impact:**

```python
# Square path: (0,0) → (2,0) → (2,2) → (0,2) → (0,0)
# merged_waypoints has (0,0) TWICE (start and end)
# RPP sees this as "goal reached" at start!
```

**Fix Required:**

```python
def _merge_segments(self, segments, close_loop: bool = False):
    merged = []
    for seg in segments:
        if merged and seg.points:
            # Skip duplicate junction points
            if _dist(merged[-1], seg.points[0]) < 0.01:
                merged.extend(seg.points[1:])
            else:
                merged.extend(seg.points)
        else:
            merged.extend(seg.points)
    
    # Close loop if requested and not already closed
    if close_loop and merged:
        if _dist(merged[0], merged[-1]) > 0.01:
            merged.append(merged[0])
    
    return merged
```

---

### GAP 4: DXF Layer Mapping is Fragile

**Problem:**

```python
# path_engine/parsers/dxf_parser.py - entities_to_segments()
def is_mark(self, layer_mapping: dict[str, str] | None = None) -> bool:
    if layer_mapping:
        upper = self.layer.upper()
        for pattern, seg_type in layer_mapping.items():
            if pattern.upper() in upper:  # Substring match!
                return seg_type.upper() != "TRANSIT"
    # Default: everything is MARK except "TRANSIT" keyword
```

**Issues:**

| Issue | Example | Problem |
|-------|---------|---------|
| Substring matching | `"MARK_OLD"` matches `"MARK"` | Treated as MARK |
| Substring matching | `"REMARK"` matches `"MARK"` | Treated as MARK |
| No regex support | Can't do `MARK_\d+` | No numbered layers |
| No "ignore" layer handling | Construction lines, dimensions | Clutters path |
| No color-based classification | AutoCAD convention: red=mark, blue=transit | Missed convention |

**Impact:**

- ❌ Wrong spray control (transit lines get marked)
- ❌ Clutter in path (dimension lines, text, hatches)
- ❌ Manual layer cleanup required before export

**Fix Required:**

```python
def entities_to_segments(
    entities: list[DXFEntity],
    layer_rules: list[dict] | None = None,  # Priority-ordered rules
    default_type: str = "ignore",
) -> list[PathSegment]:
    """
    layer_rules example:
    [
        {"pattern": r"^MARK_\d+$", "type": "mark", "method": "regex"},
        {"pattern": "TRANSIT", "type": "transit", "method": "substring"},
        {"pattern": "CONSTRUCTION", "type": "ignore", "method": "exact"},
        {"color": 1, "type": "mark"},  # AutoCAD red
        {"color": 5, "type": "transit"},  # AutoCAD blue
    ]
    """
    pass
```

---

### GAP 5: No Path Validation / Sanity Checks

**Problem:**

PathEngine blindly processes any input. No checks for:

| Missing Check | Risk |
|---------------|------|
| Self-intersecting paths | Path crosses itself |
| Disconnected segments | Gap > threshold |
| Zero-length segments | Duplicate points |
| Excessive curvature | R < vehicle turning radius |
| Out-of-bounds waypoints | Negative coordinates, huge values |
| Segment count explosion | 1000+ segments from bad DXF |

**Impact:**

```python
# Bad DXF with 10,000 tiny LINE entities (CAD export bug)
plan = engine.plan_file("bad.dxf")
# Result: 500,000 waypoints, 50 MB /path message, ROS2 crashes
```

**Fix Required:**

```python
class PathValidator:
    def validate(self, plan: PlannedPath) -> list[str]:
        """Returns list of warnings/errors."""
        issues = []
        
        # Check 1: Waypoint count
        if plan.num_waypoints > 10000:
            issues.append(f"Too many waypoints: {plan.num_waypoints}")
        
        # Check 2: Self-intersection
        if self._has_self_intersection(plan.merged_waypoints):
            issues.append("Path crosses itself")
        
        # Check 3: Disconnected segments
        gaps = self._find_gaps(plan.segments, threshold=0.5)
        if gaps:
            issues.append(f"Found {len(gaps)} gaps > 0.5m")
        
        # Check 4: Minimum radius
        min_r = self._compute_min_radius(plan.merged_waypoints)
        if min_r < 0.3:  # Vehicle can't turn tighter than 30cm
            issues.append(f"Minimum radius {min_r:.2f}m too tight")
        
        return issues
```

---

## ⚠️ MAJOR GAPS

---

### GAP 6: No Path Smoothing in PathEngine

**Problem:**

```python
# path_engine/planners/straight_line.py
def densify_line(start, end, spacing):
    # Just linear interpolation!
    # No smoothing, no corner rounding
```

**What's Missing:**

- 🔄 Corner smoothing (inscribed arcs at vertices)
- 📈 Bezier / spline fitting (G2 continuity)
- ⛓️ Curvature limiting (bound κ_max)

**Current State:**

```
DXF polyline → densify_line → piecewise linear path
                                ↓
                         κ = 0 on straights
                         κ = ∞ at vertices  ← BAD!
```

**Impact:**

- ❌ RPP sees infinite curvature at every corner
- ❌ Speed regulation fails (predictive κ → ∞ → speed → 0)
- ❌ Jerky motion at corners

> **Note:** You mentioned this in `rpp_controller_node.py` as P1.3:
> ```python
> # P1.3 — Path conditioning on receipt
> def _smooth_corners(self, pts, radius, arc_pts, min_bend_rad):
>     # This is in RPP controller, should be in PathEngine!
> ```

**Fix Required — Move smoothing to PathEngine:**

```python
# path_engine/planners/smooth.py
def smooth_path_g2(
    waypoints: list[tuple[float, float]],
    corner_radius: float = 0.3,
    method: str = "arc",  # or "bezier", "spline"
) -> list[tuple[float, float]]:
    """
    Replace sharp corners with smooth curves.
    Guarantees G2 continuity (continuous curvature).
    """
    pass
```

---

### GAP 7: No Multi-Path Mission Support

**Problem:**

```python
# path_engine/engine.py
class PathEngine:
    def plan_file(self, filepath: str) -> PlannedPath:
        # Returns ONE path
        # No support for multiple paths in a mission
```

**What's Missing:**

- 🗂️ Multi-path missions (mark field 1 → transit to field 2 → mark field 2)
- 🔢 Path sequencing (which order to execute?)
- 🔗 Inter-path transitions (auto-generate transit between paths)
- ⚡ Conditional paths (if battery < 20%, skip path 3)

**Real-World Need:**

```
Mission: Mark 3 soccer fields in a complex
├── Field A:    100m × 50m
├── Transit:    200m to Field B
├── Field B:    100m × 50m
├── Transit:    150m to Field C
├── Field C:    100m × 50m
└── Return:     300m to start
```

**Current Workaround:**

```python
# Manually merge 3 DXF files with transit lines drawn in CAD
# OR
# Run 3 separate missions (requires re-ARM between each)
```

**Fix Required:**

```python
class Mission:
    paths: list[PlannedPath]
    transitions: list[PathSegment]  # Auto-generated transits
    
def plan_mission(
    path_files: list[str],
    start_position: tuple[float, float],
    return_to_start: bool = True,
) -> Mission:
    """
    Plans multi-path mission with optimal sequencing.
    """
    pass
```

---

### GAP 8: No Dynamic Replanning

**Problem:**

PathEngine is static — plan once, execute once.

**What's Missing:**

| Feature | Purpose |
|---------|---------|
| 🚧 Obstacle avoidance | Detect obstacle → replan around it |
| 📍 Coverage replanning | Missed a spot → add patch path |
| 🔋 Battery-aware replanning | Low battery → shortest path home |
| 🌧️ Weather replanning | Rain detected → pause, resume later |

**Impact:**

```
Scenario: Rover is 50% through marking a field
├── Obstacle detected (person walks onto field)
├── Current: STOP, wait for manual intervention
└── Needed: Pause, replan around obstacle, resume
```

**Fix Required:**

```python
class DynamicPlanner:
    def replan_around_obstacle(
        self,
        current_path: PlannedPath,
        current_position: tuple[float, float],
        obstacle_position: tuple[float, float],
        obstacle_radius: float,
    ) -> PlannedPath:
        """
        Generates detour path that:
        1. Avoids obstacle
        2. Rejoins original path
        3. Minimizes added distance
        """
        pass
```

---

### GAP 9: No Path Optimization Beyond TSP

**Problem:**

```python
# path_engine/optimizers/segment_order.py
def optimize_segment_order(segments, start_position):
    # Nearest-neighbor TSP (greedy)
    # No global optimization
```

**Issues:**

- 🐢 Greedy TSP is suboptimal (can be 25% longer than optimal)
- 📐 No coverage optimization (back-and-forth vs spiral)
- 🔄 No turn minimization (prefers straight transits)
- 🔋 No battery consideration (doesn't account for elevation)

**Better Algorithms:**

| Algorithm | Improvement | Use Case |
|-----------|-------------|----------|
| 2-opt / 3-opt TSP | 10-15% shorter | Segment ordering |
| Coverage path planning | Optimal area coverage | Boustrophedon, spiral |
| Dubins paths | Minimum-turn paths | Differential drive |

**Fix Required:**

```python
# path_engine/optimizers/coverage.py
def optimize_coverage(
    boundary: list[tuple[float, float]],
    line_spacing: float,
    pattern: str = "boustrophedon",  # or "spiral", "zigzag"
) -> list[PathSegment]:
    """
    Generates optimal coverage path for area marking.
    """
    pass
```

---

## ⚙️ MODERATE GAPS

---

### GAP 10: No Unit Handling

**Problem:**

```python
# path_engine/parsers/dxf_parser.py
def parse_dxf(filepath, unit_scale: float | None = None):
    if unit_scale is None:
        # Auto-detect from $INSUNITS
        unit_scale = _detect_units(doc)
    # But what if $INSUNITS is wrong?
```

**Issues:**

- ❌ No unit validation (is 100 really 100 meters or 100 feet?)
- ❌ No unit conversion UI (user can't override)
- ❌ No scale preview (can't see if path is 10m or 10km)

**Fix Required:**

```python
def parse_dxf_with_validation(filepath):
    plan = parse_dxf(filepath)
    
    # Sanity check: typical field is 10-200m
    bbox = compute_bounding_box(plan.merged_waypoints)
    if bbox.width > 1000:  # 1km
        raise ValueError(
            f"Path is {bbox.width:.0f}m wide. "
            f"Check DXF units (might be cm or inches)."
        )
```

---

### GAP 11: No Spray Latency Calibration

**Problem:**

```python
# path_engine/spray.py
def apply_spray_latency_compensation(
    segment,
    spray_on_latency_s: float = 0.10,  # Hardcoded!
    spray_off_latency_s: float = 0.01,
):
```

**Issues:**

| Issue | Detail |
|-------|--------|
| 🏎️ Speed-dependent | 100ms at 0.4 m/s ≠ 100ms at 0.2 m/s |
| 🔧 Per-nozzle variation | Left/right nozzles differ |
| 🌡️ Temperature sensitivity | Cold paint is slower |
| ✅ No validation | Is compensation working? |

**Fix Required:**

```python
class SprayCalibrator:
    def calibrate_latency(
        self,
        test_speeds: list[float] = [0.2, 0.3, 0.4, 0.5],
    ) -> dict[float, tuple[float, float]]:
        """
        Returns: {speed: (on_latency, off_latency)}
        
        Procedure:
        1. Drive straight line at each speed
        2. Trigger spray ON at known position
        3. Measure actual spray start position (camera)
        4. Compute latency = distance / speed
        """
        pass
```

---

### GAP 12: No Path Preview / Visualization

**Problem:**

PathEngine has no built-in visualization.

**What's Missing:**

| Feature | Purpose |
|---------|---------|
| 🖼️ 2D plot (matplotlib) | Visualize planned path |
| 🎨 Segment coloring | MARK=red, TRANSIT=blue |
| 🌡️ Waypoint density heatmap | Where is spacing tight? |
| 📊 Curvature plot | Where are tight turns? |
| 🌍 Export to KML | View in Google Earth |

**Current Workaround:**

```python
# User must manually plot in Jupyter:
import matplotlib.pyplot as plt
plt.plot([p[1] for p in plan.merged_waypoints],
         [p[0] for p in plan.merged_waypoints])
```

**Fix Required:**

```python
# path_engine/viz.py
def plot_path(plan: PlannedPath, show_waypoints=True, show_spray=True):
    """Interactive matplotlib plot with zoom, pan."""
    pass

def export_kml(plan: PlannedPath, origin_gps: tuple[float, float], filename: str):
    """Export to Google Earth KML."""
    pass
```

---

### GAP 13: No Path Editing / Modification

**Problem:**

Once planned, path is immutable.

**What's Missing:**

| Operation | Use Case |
|-----------|----------|
| ➕ Insert waypoint | Add point between two existing |
| 🗑️ Delete segment | Remove a line |
| 🔄 Reverse segment | Flip direction |
| ✂️ Split path | Break into two paths |
| 🔗 Merge paths | Combine two paths |

**Real-World Need:**

```
Scenario: Planned path has a line through a tree
├── Current: Re-export DXF, re-plan entire path
└── Needed: Delete that segment, add detour, re-merge
```

**Fix Required:**

```python
class PathEditor:
    def insert_waypoint(self, path, index, point):
        pass
    
    def delete_segment(self, path, segment_id):
        pass
    
    def reverse_segment(self, path, segment_id):
        pass
```

---

## 🔧 MINOR GAPS

---

### GAP 14: No Error Recovery

**Problem:**

```python
# path_engine/parsers/dxf_parser.py
def parse_dxf(filepath):
    doc = ezdxf.readfile(filepath)  # Can raise!
    # No try/except, no error messages
```

**Issues:**

- 💥 Corrupt DXF → crash
- ❓ Missing file → crash
- 🔇 Unsupported entity → silent skip (no warning)

**Fix Required:**

```python
def parse_dxf_safe(filepath):
    try:
        doc = ezdxf.readfile(filepath)
    except IOError:
        raise FileNotFoundError(f"DXF not found: {filepath}")
    except ezdxf.DXFStructureError as e:
        raise ValueError(f"Corrupt DXF: {e}")
    
    # Warn on unsupported entities
    for entity in doc.modelspace():
        if entity.dxftype() not in SUPPORTED_TYPES:
            log.warning(f"Skipping unsupported: {entity.dxftype()}")
```

---

### GAP 15: No Performance Optimization

**Problem:**

```python
# path_engine/optimizers/segment_order.py
def optimize_segment_order(segments):
    # O(n²) nearest-neighbor search
    for i in range(len(segments)):
        for j in range(i+1, len(segments)):
            # Compute distance...
```

**Issues:**

- 🐌 Slow on large paths (1000 segments = 1M comparisons)
- 📦 No spatial indexing (R-tree, KD-tree)
- 🗑️ No caching (recomputes distances every call)

**Fix Required:**

```python
from scipy.spatial import KDTree

def optimize_segment_order_fast(segments):
    # Build KD-tree of segment start/end points
    tree = KDTree([s.points[0] for s in segments])
    # Nearest-neighbor in O(log n)
    pass
```

---

### GAP 16: No Logging / Telemetry

**Problem:**

PathEngine has minimal logging.

**What's Missing:**

| Metric | Purpose |
|--------|---------|
| ⏱️ Planning time | How long did it take? |
| 🔢 Waypoint count | Before/after densification |
| 📏 Path length | Total meters |
| 📊 Optimization stats | TSP improvement % |

**Fix Required:**

```python
import logging
log = logging.getLogger("path_engine")

def plan_file(self, filepath):
    t0 = time.time()
    log.info(f"Planning {filepath}...")
    
    plan = self._plan_from_segments(...)
    
    log.info(
        f"Planned in {time.time()-t0:.2f}s: "
        f"{plan.num_waypoints} waypoints, "
        f"{plan.total_length:.1f}m total"
    )
    return plan
```

---

## 📊 GAP PRIORITY MATRIX

| # | Gap | Severity | Effort | Impact | Priority |
|---|-----|----------|--------|--------|----------|
| 1 | GPS Transform | 🔴 Critical | 2 days | Blocks field deployment | **P0** |
| 2 | Multi-Point Alignment | 🔴 Critical | 3 days | Blocks rotated templates | **P0** |
| 3 | Closed-Loop Handling | 🔴 Critical | 4 hours | Breaks square/circle paths | **P0** |
| 4 | Layer Mapping | 🟠 Major | 1 day | Wrong spray control | **P1** |
| 5 | Path Validation | 🟠 Major | 2 days | Prevents bad paths | **P1** |
| 6 | Path Smoothing | 🟠 Major | 3 days | Jerky motion at corners | **P1** |
| 7 | Multi-Path Missions | 🟠 Major | 1 week | Limits scalability | **P2** |
| 8 | Dynamic Replanning | 🟠 Major | 2 weeks | No obstacle avoidance | **P3** |
| 9 | Coverage Optimization | 🟡 Moderate | 1 week | Suboptimal paths | **P2** |
| 10 | Unit Handling | 🟡 Moderate | 1 day | User confusion | **P2** |
| 11 | Spray Calibration | 🟡 Moderate | 2 days | Marking accuracy | **P2** |
| 12 | Visualization | 🟡 Moderate | 3 days | Debugging difficulty | **P2** |
| 13 | Path Editing | 🟡 Moderate | 1 week | Manual rework needed | **P3** |
| 14 | Error Recovery | 🟢 Minor | 1 day | Better UX | **P3** |
| 15 | Performance | 🟢 Minor | 2 days | Slow on large paths | **P3** |
| 16 | Logging | 🟢 Minor | 4 hours | Debugging aid | **P3** |

---

## 🎯 RECOMMENDED ACTION PLAN

---

### 🏃 Sprint 1: Field Deployment Blockers *(1 week)*

| Gap | Task | Deliverable |
|-----|------|-------------|
| ✅ GAP 1 | Add GPS → NED transform to PathEngine | |
| ✅ GAP 2 | Implement `align_template_to_survey()` | |
| ✅ GAP 3 | Fix closed-loop path handling | |
| ✅ GAP 5 | Add basic path validation | |

**🎯 Deliverable:** Can deploy DXF template at a new site with GPS survey.

---

### 🏃 Sprint 2: Path Quality *(1 week)*

| Gap | Task | Deliverable |
|-----|------|-------------|
| ✅ GAP 6 | Move corner smoothing from RPP to PathEngine | |
| ✅ GAP 4 | Improve layer mapping (regex, color-based) | |
| ✅ GAP 12 | Add matplotlib visualization | |

**🎯 Deliverable:** Smooth paths with correct spray control.

---

### 🏃 Sprint 3: Scalability *(2 weeks)*

| Gap | Task | Deliverable |
|-----|------|-------------|
| ✅ GAP 7 | Multi-path mission support | |
| ✅ GAP 9 | Coverage path planning (boustrophedon) | |
| ✅ GAP 10 | Unit validation and conversion | |

**🎯 Deliverable:** Can plan complex multi-field missions.

---

### 🏃 Sprint 4: Polish *(1 week)*

| Gap | Task | Deliverable |
|-----|------|-------------|
| ✅ GAP 11 | Spray latency calibration | |
| ✅ GAP 13 | Basic path editing | |
| ✅ GAP 14-16 | Error handling, performance, logging | |

**🎯 Deliverable:** Production-ready path planning system.

---

## 🔥 CRITICAL NEXT STEPS

> **Before your next field deployment, you MUST fix:**

| Priority | Gap | Reason |
|----------|-----|--------|
| ✅ | GPS alignment (GAP 1+2) | Otherwise you're guessing offsets |
| ✅ | Closed-loop handling (GAP 3) | Otherwise squares don't work |
| ✅ | Path validation (GAP 5) | Otherwise bad DXF crashes system |

> Everything else can wait.