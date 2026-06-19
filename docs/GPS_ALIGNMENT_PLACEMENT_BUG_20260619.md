# GPS‑Alignment Placement Bug — Field Investigation

**Date:** 2026-06-19
**Reporter symptom:** "Rover not going to the first point" after aligning a DXF from the frontend and starting the mission.
**Mission under investigation:** `square_2x2.dxf_20260619_185015` (started 18:50:15, RTK_FIXED).
**Evidence sources:** Jetson `rover-server` journal, ROS2 bag `square_2x2.dxf_20260619_185015`, server/companion source.
**Status:** Root cause identified and quantified. No fix applied yet (diagnostic only).

---

## 1. TL;DR

The reported failure is **NOT** a scaling error — the [ref-point double-scaling fix](REFPOINT_DOUBLE_SCALING_AUDIT.md) deployed earlier today is working (the published path is a correct 2.0 × 2.0 m square, scale = 1.0).

The real defect is **placement**: a GPS-aligned mission's path is published with its origin pinned to the rover's **EKF local origin (NED 0,0)**, and the alignment's `origin_gps` anchor is **never reconciled with the rover's live local frame**. With `auto_origin` off (as it was for this run), no rover-relative shift is applied either. Net effect: the first waypoint landed **~7.5 m** from the rover instead of at the surveyed reference location (~2.3 m away). The rover correctly drove toward the misplaced first point (~2.9 m south) before the run was aborted.

Because the EKF origin is arbitrary (wherever PX4 set home/origin at boot/arm), the misplacement magnitude varies run-to-run — the diagnostic tell that absolute GPS placement is being dropped.

---

## 2. Operator inputs (as reported)

Two reference points used for alignment:

| Ref | DXF (preview, m) | GPS lat | GPS lon |
|---|---|---|---|
| A | `0.0` | 13.072066 | 80.261956 |
| B | `0.2` | 13.072066 | 80.261945 |

- GPS A→B: Δlat = 0, Δlon = −0.000011° → **B is ~1.19 m west of A**, 0 m north (baseline is purely east-west).
- This is a **short, single-axis baseline** (~1.2 m) for a 2 m square — noise-sensitive for 2-point least-squares (secondary risk, not the placement bug).

---

## 3. Evidence from the bag

Bag: `bags/19-06-2026/path_Engine /square_2x2.dxf_20260619_185015/`
Duration **11.2 s** (incomplete — aborted), 5536 msgs. Decoded on Mac via sqlite3 + manual CDR (no ROS2 needed; 8-byte CDR alignment is relative to the post-encapsulation byte, i.e. offset 4).

### 3.1 `/path` (nav_msgs/Path, frame `local_ned`)

| Msg | Count | First wp | Geometry |
|---|---|---|---|
| path[0] | 120 | (0.0, −0.035) ≈ (0,0) | **2.0 × 2.0 m square**, x∈[0,2.0], y∈[−0.035,2.0], axis-aligned |
| path[1] | 1 | (4.601, −0.244) | single-point **E-stop / abort** path |

`publish_path` maps points as `pose.position.x = north`, `pose.position.y = east`, `z = spray_flag`. So path[0] is north 0..2, east −0.035..2 → a correct square. **Scale = 1.0, rotation ≈ 0 — geometry is right.**

### 3.2 Rover state (`/mavros/local_position/pose`, ENU `map`; `/mavros/global_position/global`)

| | ENU x (E) | ENU y (N) | → NED (n, e) | GPS |
|---|---|---|---|---|
| start | −0.907 | 7.463 | (7.463, −0.907) | 13.0720864, 80.2619557 |
| end (11 s) | −0.231 | 4.551 | (4.551, −0.231) | 13.0720602, 80.2619619 |

Rover drove **~2.9 m south** (N 7.46→4.55), toward the path origin at N=0, then stopped.

### 3.3 The mismatch

- First waypoint: NED **(0, −0.035)**.
- Rover start: NED **(7.463, −0.907)**.
- Distance rover → first waypoint = **~7.51 m** (7.46 m south, 0.87 m east).
- Rover got ~2.9 m before abort → "not reaching the first point."

---

## 4. Evidence from the logs

`journalctl -u rover-server` around the run:

```
18:50:15.120  Path loaded: square_2x2.dxf (120 pts)
18:50:15.134  arming…
18:50:15.220  switching to OFFBOARD…
18:50:15.747  mission running: square_2x2.dxf
```

- **No `auto_origin offset: …` line** → `auto_origin` was **OFF** for this start (that log is emitted only when auto_origin shifts the path — [offboard_controller.py:225](../server/offboard_controller.py:225)).
- Earlier in the day the DXF parsed with `$INSUNITS is 0 (unspecified) — using fallback scale 0.01` — i.e. a non-metre (cm-fallback) drawing, the exact case the scaling fix targets. The clean 2×2 m output confirms the fix held.
- Staged-mission anchors during the session flip-flopped between real coords (`13.07206…, 80.26194…`) and the **placeholder `13.0, 80.0`** (the test fixture coordinate), and least-squares `scale` ranged 1.02–1.11 — see §6.

---

## 5. Evidence from the code

### 5.1 `publish_path` drops the GPS anchor — [server/ros_node.py:905](../server/ros_node.py:905)

```python
def publish_path(self, points, frame_id="local_ned", spray_flags=None):
    ...
    for (n, e), spray in zip(points, flags):
        ps.pose.position.x = float(n)
        ps.pose.position.y = float(e)
        ps.pose.position.z = 1.0 if spray else 0.0
```

No `origin_gps` parameter. Points are published verbatim in `local_ned` with origin (0,0). `origin_gps` does not appear anywhere in the ROS publishing layer (`src/*.py`, `server/ros_node.py`).

### 5.2 Start applies only the optional auto_origin shift — [server/offboard_controller.py:212](../server/offboard_controller.py:212)

```python
pts_to_publish = self._loaded_pts
if auto_origin:
    off_n, off_e = pose_origin           # rover's current local NED
    pts_to_publish = [(n + off_n, e + off_e) for n, e in self._loaded_pts]
    self._log_entry("info", f"auto_origin offset: +{off_n:.3f}N +{off_e:.3f}E")
...
self._node.publish_path(pts_to_publish, spray_flags=spray_flags_to_publish)
```

- `auto_origin = True` → first waypoint snapped to the rover's current position (discards surveyed absolute location).
- `auto_origin = False` (this run) → path published as-is → origin lands on the **EKF origin (0,0)**.
- **Neither branch uses `origin_gps`.** There is no GPS-origin → EKF-origin reconciliation anywhere.

---

## 6. Quantified root cause

The path engine builds waypoints in NED **relative to `origin_gps`**, but nothing converts that frame into the rover's **live EKF/local frame**. The GPS anchor is lost at publish time.

Reconstructing what *should* have happened, using the bag's live GPS↔pose correspondence
(rover GPS `13.0720864, 80.2619557` ↔ EKF-NED `(7.463, −0.907)`):

- Ref point **A** (`13.072066, 80.261956`) maps into the EKF frame at **NED (5.19, −0.87)** — i.e. ~2.3 m south of the rover (matches expectation).
- The path origin was instead published at **EKF-NED (0, 0)** — **~5.2 m too far south**.
- So the first waypoint landed at ~7.5 m from the rover instead of ~2.3 m. The missing ~5.2 m **is exactly the EKF-origin → `origin_gps` offset that is never applied.**

Because the EKF origin is arbitrary, this error is non-deterministic across runs.

---

## 7. Fix options

### Option 1 — Immediate workaround: `auto_origin = true`
Start the mission with `auto_origin = true`. The path's first waypoint is shifted onto the rover's current position ([offboard_controller.py:214](../server/offboard_controller.py:214)); the rover starts marking where it stands.
**Caveat:** ignores the surveyed absolute location — acceptable when you only need the *shape* on the ground at the rover, not at specific real-world coordinates.

### Option 2 — Proper fix: reconcile GPS anchor with the live frame
At mission start, translate the loaded path by the NED offset between the **EKF-origin GPS and `origin_gps`**, derived from the live GPS↔local-pose pair:

```
shift = rover_local_NED − latlon_to_ned(rover_gps, origin_gps)
pts_to_publish = [(n + shift_n, e + shift_e) for n, e in loaded_pts]
```

Requires plumbing `origin_gps` (already in the staged mission's `alignment_metadata` / `anchor`) through load → start → publish. This honors the survey: the DXF point clicked as ref A lands at GPS A on the ground.

> Mutually exclusive with auto_origin — auto_origin pins to the rover, option 2 pins to the survey. The UI should make the operator choose intent.

---

## 8. Secondary findings (not the placement bug, worth tracking)

1. **Short alignment baseline.** The two ref points share latitude and are ~1.2 m apart. For a 2 m feature this is noise-sensitive; least-squares `scale` was seen drifting to 1.02–1.11 on other staged missions today. Prefer ref points far apart and not collinear.
2. **Placeholder anchor leakage.** Several staged missions used `origin_gps = [13.0, 80.0]` — the test-fixture coordinate ~29.5 km from the rover. If a mission with that anchor were ever placed via a correct GPS reconciliation (Option 2), the rover would be commanded 29.5 km away. The frontend should reject/replace the `13.0, 80.0` default and require real coordinates.
3. **Frame label mismatch (benign).** `/path` is `local_ned`; `/mavros/local_position/pose` is `map`. The RPP controller assumes they share an origin (validated historically). Not a cause here, but worth a sanity assert if frames ever diverge.

---

## 9. Reproduction / analysis notes

- Bag decoded with a standalone sqlite3 + CDR reader (Mac has no ROS2). Key gotcha: **float64 CDR fields are 8-byte aligned relative to the byte after the 4-byte encapsulation header** (`aligned = 4 + ((off-4 + 7) & ~7)`); 4-byte alignment is identical absolute-vs-relative because the base (4) is already 4-aligned.
- Reusable decoders live in `tools/extract_rosbag_direct.py` (note: its `parse_Path` truncates to the first 3 poses and lacks the 8-byte alignment fix — extend it before reuse on full paths).
- Live telemetry confirmation: `tools/capture_telemetry.py -n 1 --host localhost` (RTK_FIXED, lat/lon).
