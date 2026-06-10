#!/usr/bin/env python3
"""Quick bag analysis for arc_fix_03 and arc_fix_04."""
import sqlite3, struct, math, sys
import numpy as np
from pathlib import Path

def decode_f32_multiarray(data):
    """Decode CDR-serialized std_msgs/Float32MultiArray.
    Scan for uint32==39 followed by 39 valid floats."""
    data = bytes(data)
    for start in range(4, len(data) - 4 - 39*4, 1):
        try:
            n = struct.unpack_from('<I', data, start)[0]
            if n == 39 and start + 4 + 39*4 <= len(data):
                vals = struct.unpack_from('<39f', data, start + 4)
                if all(math.isfinite(v) for v in vals[:5]):
                    return list(vals)
        except Exception:
            pass
    return []

def read_bag(db3_path):
    conn = sqlite3.connect(db3_path)
    c = conn.cursor()
    c.execute("SELECT id, name FROM topics")
    topics = {row[1]: row[0] for row in c.fetchall()}

    debug_id  = topics.get('/rpp/debug')
    pose_id   = topics.get('/mavros/local_position/pose')
    path_id   = topics.get('/path')

    debug_rows = []
    if debug_id:
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (debug_id,))
        rows = c.fetchall()
        t0 = rows[0][0] if rows else 0
        for ts, raw in rows:
            vals = decode_f32_multiarray(raw)
            if len(vals) >= 39:
                debug_rows.append([(ts - t0) * 1e-9] + vals[:39])

    pose_rows = []
    if pose_id:
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (pose_id,))
        for ts, raw in c.fetchall():
            raw = bytes(raw)
            # ENU PoseStamped: x=east, y=north in position
            try:
                # CDR header 4 bytes, then Header (seq u32, stamp sec+nsec u32+u32, frame_id string),
                # then position (3 float64) + orientation (4 float64)
                # Quickest: scan for 7 consecutive float64 (position + orientation)
                for off in range(4, len(raw) - 56, 4):
                    try:
                        vals = struct.unpack_from('<7d', raw, off)
                        # Sanity: position within 500m of origin, orientation quaternion ~unit
                        px, py, pz, qx, qy, qz, qw = vals
                        if (abs(px) < 500 and abs(py) < 500 and abs(pz) < 100 and
                                0.9 < abs(qx)**2+abs(qy)**2+abs(qz)**2+abs(qw)**2 < 1.1):
                            t0p = pose_rows[0][0] if pose_rows else (ts * 1e-9)
                            n = py   # ENU y = North
                            e = px   # ENU x = East
                            pose_rows.append((ts * 1e-9, n, e))
                            break
                    except Exception:
                        pass
            except Exception:
                pass

    conn.close()
    return debug_rows, pose_rows

for bag_name in ["arc_fix_03_20260609_173922", "arc_fix_04_20260609_174210"]:
    db3 = Path(f"bags/{bag_name}/{bag_name}_0.db3")
    print(f"\n{'='*65}")
    print(f"BAG: {bag_name}")
    debug_rows, pose_rows = read_bag(db3)

    if not debug_rows:
        print("  ERROR: no debug data decoded")
        continue

    arr = np.array(debug_rows)
    # t at col 0, fields[0..38] at cols 1..39
    # field[7]=state_code -> col 8
    state = arr[:, 8]
    active = (state == 1) | (state == 2)
    print(f"  Total debug rows: {len(arr)},  Active (tracking): {int(active.sum())}")
    if active.sum() < 5:
        active = np.ones(len(arr), dtype=bool)

    d = arr[active]
    cte  = d[:, 1]   # cross_track_error_m (signed)
    hdg  = d[:, 2]   # heading_error_rad
    spd  = d[:, 4]   # speed_cmd_m_s
    kap  = d[:, 5]   # curvature_kappa
    yaw  = d[:, 11]  # yaw_rate_cmd_rad_s  (field[10] -> col 11)
    ksp  = d[:, 10]  # kappa_speed          (field[9]  -> col 10)

    last = d[-1]
    # param cols: field[11..38] -> cols 12..39
    print(f"\n  ── PARAMS (live from /rpp/debug telemetry) ─────────────────")
    print(f"  mission_speed          = {last[39]:.3f} m/s")
    print(f"  max_linear_vel         = {last[12]:.3f} m/s")
    print(f"  min_linear_vel         = {last[13]:.3f} m/s")
    print(f"  a_lat_max              = {last[17]:.4f} m/s²")
    print(f"  min_lookahead_dist     = {last[14]:.3f} m")
    print(f"  max_lookahead_dist     = {last[15]:.3f} m")
    print(f"  lookahead_time         = {last[16]:.3f} s")
    print(f"  corner_smooth_radius   = {last[30]:.3f} m")
    print(f"  corner_smooth_arc_pts  = {last[31]:.0f}")
    print(f"  use_feedforward_yaw    = {bool(last[34]>0.5)}")
    print(f"  yaw_rate_feedback_gain = {last[35]:.4f}")
    print(f"  max_yaw_rate_body      = {last[36]:.4f} rad/s")
    print(f"  preview_curvature_n    = {last[27]:.0f}")
    print(f"  xtrack_lookahead_gain  = {last[28]:.4f}")
    print(f"  path_resample_spacing  = {last[29]:.4f} m")
    print(f"  max_linear_accel       = {last[37]:.4f} m/s²")
    print(f"  max_linear_decel       = {last[38]:.4f} m/s²")
    print(f"  require_rtk_fix        = {bool(last[26]>0.5)}")
    print(f"  regulated_min_speed    = {last[18]:.3f} m/s")

    print(f"\n  ── CROSS-TRACK ERROR ────────────────────────────────────────")
    abs_cte = np.abs(cte)
    print(f"  median  |CTE| = {np.median(abs_cte)*100:.2f} cm")
    print(f"  mean    |CTE| = {np.mean(abs_cte)*100:.2f} cm")
    print(f"  p95     |CTE| = {np.percentile(abs_cte,95)*100:.2f} cm")
    print(f"  max     |CTE| = {np.max(abs_cte)*100:.2f} cm")
    pos_pct = np.mean(cte > 0) * 100
    print(f"  sign: {pos_pct:.0f}% positive (right of path), {100-pos_pct:.0f}% negative")

    print(f"\n  ── SPEED ────────────────────────────────────────────────────")
    print(f"  median spd = {np.median(spd):.3f} m/s")
    print(f"  mean   spd = {np.mean(spd):.3f} m/s")
    print(f"  max    spd = {np.max(spd):.3f} m/s")
    moving = spd > 0.01
    if moving.any():
        print(f"  min(moving) = {np.min(spd[moving]):.3f} m/s")

    print(f"\n  ── CURVATURE ────────────────────────────────────────────────")
    nonzero_kap = kap[np.abs(kap) > 0.05]
    if len(nonzero_kap):
        mk = np.median(np.abs(nonzero_kap))
        print(f"  median |kappa| (active) = {mk:.4f} 1/m  → R = {1/mk:.2f} m")
        print(f"  max    |kappa|          = {np.max(np.abs(kap)):.4f} 1/m  → R_min = {1/max(np.max(np.abs(kap)),0.001):.2f} m")
    else:
        print(f"  kappa ≈ 0 (straight)")
    print(f"  median kappa_speed = {np.median(ksp):.4f} 1/m")

    print(f"\n  ── YAW RATE ─────────────────────────────────────────────────")
    abs_yr = np.abs(yaw)
    print(f"  median |yaw_rate| = {np.median(abs_yr):.4f} rad/s")
    print(f"  max    |yaw_rate| = {np.max(abs_yr):.4f} rad/s")
    max_yr_param = last[36]
    if max_yr_param > 0:
        sat_frac = np.mean(abs_yr >= max_yr_param * 0.99)
        print(f"  saturation        = {sat_frac*100:.1f}%  (limit={max_yr_param:.3f} rad/s)")

    print(f"\n  ── HEADING ERROR ────────────────────────────────────────────")
    abs_hdg = np.abs(hdg)
    print(f"  median |hdg_err| = {np.degrees(np.median(abs_hdg)):.2f}°")
    print(f"  max    |hdg_err| = {np.degrees(np.max(abs_hdg)):.2f}°")

    print(f"\n  ── PATH TYPE (inferred) ─────────────────────────────────────")
    all_kap = np.abs(kap)
    high_kap_pct = np.mean(all_kap > 0.3) * 100
    print(f"  % samples with |kappa|>0.3: {high_kap_pct:.0f}%")
    if len(nonzero_kap) and high_kap_pct > 30:
        R = 1.0 / np.median(np.abs(nonzero_kap))
        if R < 2.5 and R > 0.5:
            print(f"  → ARC path  (R≈{R:.2f} m, consistent curvature)")
        elif R < 0.8:
            print(f"  → TIGHT ARC or CORNER sweep (R≈{R:.2f} m)")
        else:
            print(f"  → CURVED path (R≈{R:.2f} m)")
    else:
        print(f"  → STRAIGHT or SQUARE (low/intermittent curvature = corners only)")

    # Duration
    print(f"\n  Duration: {arr[-1,0]:.1f} s")

print("\nDone.")
