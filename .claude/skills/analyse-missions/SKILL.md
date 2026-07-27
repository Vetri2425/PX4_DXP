---
name: analyse-missions
description: Analyse PX4_DXP rover mission bags — decode the bundle, verify geometry fidelity, and separate PLANNING offset from DRIVING offset across runs. Use when the user says "analyse missions", "check the bags", "read the bundle", asks about tracking/xtrack accuracy, whether the rover hit surveyed points, or why runs differ from each other.
---

# analyse-missions — read rover bags honestly

The rover can track its own belief perfectly while painting the wrong shape in the
wrong place. This skill exists to stop that going unnoticed. **Never report a
single cross-track number as "the accuracy".**

## 0. The four error budgets — keep them separate

| # | Question | Source | Fails silently if you skip it |
|---|---|---|---|
| 1 | Did the controller follow its own path? | pose vs `/rpp/conditioned_path` | — |
| 2 | Was that path the right SHAPE? | `/path` vs `/rpp/conditioned_path` | vertices deleted by conditioning |
| 3 | Was the path in the right PLACE on Earth? | pose→geo vs SURVEYED lat/lon | whole mission shifted, 1 & 2 still clean |
| 4 | Did the PAINT land there? | physical re-survey | nozzle offset, spray latency |

Budget 4 cannot be answered from any log. Say so rather than implying otherwise.

## 1. Bundle anatomy

A recorder bundle (`~/bags_jet/<mission_id>_<timestamp>/` on the Jetson) holds:

```
bag/            rosbag2 sqlite3 (*.db3 + metadata.yaml)
manifest.json   identity, staged-mission provenance, as-run config, outcome
report.txt      previous analyze_mission output (if it was run)
analysis.json   previous analyze_mission output, machine-readable
```

Pull with `rsync -a -e ssh flash@192.168.1.102:"bags_jet/<name>" .`
(macOS rsync is ancient — **no `--info=` flag**.) Land them in `bags/<YYYY-MM-DD>/`.

`manifest.outcome.reason == "COMPLETE"` is **NOT** proof of full traversal —
verify coverage geometrically (§4). Two of five runs on 2026-07-22 reported
COMPLETE having covered 24/64 and 76/86 waypoints.

## 2. Decoding — use `tools/analyze_mission.py` as a library

It is stdlib-only (direct sqlite3 + hand-rolled CDR reader), so it runs on the
Mac with no ROS. Import it rather than reimplementing:

```python
import importlib.util
spec = importlib.util.spec_from_file_location("am", "tools/analyze_mission.py")
am = importlib.util.module_from_spec(spec); spec.loader.exec_module(am)

bag, manifest = am._find_bag_dir("bags/2026-07-22/<bundle>/")
s = am.collect(bag)              # Series: pose, global_fix, path, cond_paths, rpp, seg, ...
for topic, msg, t in am.read_bag(bag):   # raw iteration when you need position.z
    ...
```

Useful helpers already in there: `am._metres_per_degree(lat)`,
`am._geodesic_m()`, `am._geodesic_ne_m()`, `am._perp_from_span()`.

### CDR decoding traps that have already bitten

- **`sensor_msgs/NavSatFix`**: `NavSatStatus` is `int8 status` + **`uint16 service`**.
  Reading `service` as uint8 shifts every following float64 by one slot —
  latitude lands in longitude and the fix is garbage. Sanity-check that the
  derived origin is near the site (13.072 N, 80.262 E), not near (0, 13.07).
- **`mavros_msgs/State`** has **no std_msgs header** on this stack.
- `/path` and `/rpp/conditioned_path` are **TRANSIENT_LOCAL**; the recorder needs
  `config/rosbag_qos_overrides.yaml` or the bag has no `/path` at all.

### `/path` position.z is a BITFIELD, not a boolean

```
bit0 (1) = spray ON
bit1 (2) = must-hit (surveyed vertex / declared control point)
```

`z == 2` occurs in real bags (spray-OFF must-hit, on extension legs). Always
bit-test `int(round(z)) & 1`; a `z > 0.5` test reads those as spray ON.

## 3. Measurement traps — never use these as "accuracy"

- **`/rpp/debug[0]`** is cross-track against the **CONDITIONED** path — the
  controller grading its own homework. It structurally hides vertex deletion.
- **`/spray/debug[5]`** is NOZZLE cross-track, carrying the uncalibrated
  `nozzle_lateral_offset_m`. Not tracking error.
- **The honest number is pose vs raw `/path`** (`analysis.geometry.xtrack_vs_planned`).

## 4. Procedure

### 4.1 Per-bundle
Run `python3 tools/analyze_mission.py <bundle> --quiet`, then read §7 (geometry
fidelity — `dropped_total` must be 0), §1 (tracking), §8 (absolute).

**Verify coverage** — how much of the path the rover actually drove:
```python
near = sum(1 for q in path if min(math.hypot(q[0]-p[1], q[1]-p[2]) for p in s.pose) < 0.25)
```
Report `near/len(path)`. Anything under 100% invalidates per-vertex misses at the
uncovered end (they read as metres, which is "not visited", not "missed").

### 4.2 Identify the drawing — by SHAPE, not by name
Manifests carry staged IDs, not filenames, and GPS_SURVEYED placement puts each
run at different absolute coordinates. Match on a translation/rotation-invariant
signature: consecutive segment lengths + interior bend angles.
Extensions wrap the same core, so look for the core inside a longer must-hit list.

### 4.3 Cross-mission comparison — THE method

This is what found the offset problem. Everything is done in ONE common geo frame
anchored on the surveyed coordinates, so runs are compared to each other and to
ground truth, not each to its own path.

1. **Ground truth** = surveyed lat/lon, read from the source DXF `Points` layer or
   the survey CSV `Latitude`/`Longitude` columns. **Never** derive it from the
   mission's own anchor — placement computes `local = T(global)` once, so
   converting back through `T` is circular and detects nothing.
2. **Derive each run's local→WGS84 transform from the bag's OWN pose/global pairs**
   (independent of any mission metadata):
   ```python
   lat0 = mean(global_fix.lat - pose.n / m_per_deg_north)   # pairs within 0.2 s
   lon0 = mean(global_fix.lon - pose.e / m_per_deg_east)
   ```
3. Convert planned `/path` AND travelled pose to a common ENU frame about the
   surveyed centroid. Export both as CSV (`idx, latitude, longitude, spray_on,
   must_hit` / `t_s, latitude, longitude`).
4. **Decompose the signed offset** — this is the key step:
   ```
   A = mean signed xtrack of the PLANNED marked span vs the SURVEYED line
   B = mean signed xtrack of the DRIVEN pose vs its OWN planned path
   total = A + B          (verify it reconciles with the direct measurement)
   ```
   `+` = right of travel. **A is planning error. B is driving error.** Their
   spreads across runs tell you which subsystem to fix.
5. Report the pairwise matrix of run-to-run mean-offset differences — that is
   literally how far apart two painted lines would sit.

### 4.4 Reading the result
- **Same value on every vertex within a run** → pure translation. The SHAPE is
  right; only the placement floats. Do not report it as a shape error.
- **Large bias + small scatter** → placement.
- **Small bias + large scatter** → tracking/localisation noise.
- **One-sided bias across runs** (e.g. 4 of 5 driving left) → a systematic vehicle
  problem, not noise. Noise straddles zero.

## 5. Known-good reference (2026-07-22, `tes_cross_line`, 5 runs)

Use these as the yardstick for a regression:

```
plan vs surveyed truth   0.26 – 1.13 cm   (identical on all 4 vertices = pure translation)
run-to-run plan spread   ≤ 1.53 cm
driven vs plan (bias)    −2.00 … +1.74 cm     ← dominant term, 2.5x the planning spread
vertex hits, extensions  0.33 – 0.88 cm
vertex hits, bare        0.67 – 3.66 cm       ← v0 is worst; no run-up to settle on
endpoint rest            0.9 – 2.0 cm short, overshoot never > +0.4 cm
```

**Extensions move the entry transient off the painted line** (3.66 cm → 0.42 cm at v0).
Recommend them on every marking mission.

## 6. Reporting rules

- Always give pose-vs-raw-`/path`, never only `/rpp/debug[0]`.
- Always state coverage; never quote a vertex miss for an untraversed vertex.
- Always separate planning from driving before blaming a subsystem.
- If ground truth is missing, say the absolute check is unavailable. Do not
  substitute the mission's own anchor.
- No log answers "did the paint land there". Only a physical re-survey does.
