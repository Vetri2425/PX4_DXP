#!/usr/bin/env python3
"""
RPP path-tracking analysis for square_2x2.dxf bags (17-06-2026).
Field layout confirmed by probe of raw CDR bytes:
  47 floats at offset 40 in each /rpp/debug message
  field[0]  = cte_m (signed cross-track error)
  field[1]  = hdg_err_rad
  field[2]  = lookahead_dist_m
  field[3]  = speed_cmd_m_s
  field[4]  = always 0 (unused)
  field[5]  = speed/kappa_speed (same range as f[2])
  field[6]  = kappa (curvature, 1/m)
  field[7]  = state_code (1=TRACKING in 192326)
  field[8]  = yaw_body_rad
  field[9]  = always 0
  field[10] = yaw_rate_cmd_rad_s (-max..+max)
  field[11] = max_yaw_rate (param) = 0.8 rad/s
  field[12] = min_lookahead (param) = 0.15 m
  field[13] = max_lookahead (param) = 0.52 m
  field[14] = la_time (param) = 1.0 s
  field[15] = max_lin_vel (param) = 1.6 m/s
  field[16] = a_lat_max (param) = 0.3 m/s²
  field[17] = corner_smooth_radius (param) = 0.3 m
  ...
  field[35] = max_yaw_rate_body (param) = 0.45 rad/s  ← from AGENTS.md validated
"""
import sqlite3, struct, math
import numpy as np
from pathlib import Path

def decode_debug(data):
    """Decode /rpp/debug: 47 floats at offset 40."""
    data = bytes(data)
    try:
        n = struct.unpack_from('<I', data, 36)[0]
        if n == 47 and len(data) >= 40 + 47*4:
            vals = struct.unpack_from('<47f', data, 40)
            if all(math.isfinite(v) for v in vals):
                return list(vals)
    except:
        pass
    # fallback scan for 47-float block
    for start in range(4, len(data)-4-47*4, 4):
        try:
            n = struct.unpack_from('<I', data, start)[0]
            if n == 47 and start+4+47*4 <= len(data):
                vals = struct.unpack_from('<47f', data, start+4)
                if all(math.isfinite(v) for v in vals):
                    return list(vals)
        except:
            pass
    return []

def decode_seg_debug(data):
    """Decode /rpp/segment_debug (variable length)."""
    data = bytes(data)
    # From probe: 10 floats, data count at @32, floats at @40
    for count_off in [32, 36]:
        try:
            n = struct.unpack_from('<I', data, count_off)[0]
            if 4 <= n <= 20 and count_off+4+n*4 <= len(data):
                vals = struct.unpack_from(f'<{n}f', data, count_off+4)
                if all(math.isfinite(v) and abs(v) < 1e4 for v in vals):
                    return list(vals), n
        except:
            pass
    return [], 0

def decode_pose(data):
    """PoseStamped: 7 float64 after CDR+header."""
    data = bytes(data)
    for off in range(4, len(data)-56, 4):
        try:
            px,py,pz,qx,qy,qz,qw = struct.unpack_from('<7d', data, off)
            if abs(px)<500 and abs(py)<500 and abs(pz)<100 and 0.9<qx**2+qy**2+qz**2+qw**2<1.1:
                return px, py
        except:
            pass
    return None

# Field indices (0-based into the 47-float array)
F_CTE   = 0    # cross-track error (m, signed)
F_HDG   = 1    # heading error (rad)
F_LD    = 2    # lookahead distance (m)
F_SPD   = 3    # speed command (m/s)
F_KAP   = 6    # curvature kappa (1/m)
F_STATE = 7    # state code (0=IDLE, 1=TRACKING, 2=?, 3=CORNER, etc.)
F_YAW   = 8    # yaw body (rad)
F_YRCD  = 10   # yaw rate command (rad/s)

# Param fields (constant across run)
P_MAXYR  = 11  # max_yaw_rate (likely controller limit used)
P_MINLA  = 12  # min_lookahead (m)
P_MAXLA  = 13  # max_lookahead (m)
P_LATIME = 14  # lookahead_time (s)
P_MAXV   = 15  # max_linear_vel (m/s)
P_ALAT   = 16  # a_lat_max (m/s²)
P_CRAD   = 17  # corner_smooth_radius (m)

def read_bag(db3_path):
    conn = sqlite3.connect(str(db3_path))
    c = conn.cursor()
    c.execute("SELECT id, name FROM topics")
    topics = {row[1]: row[0] for row in c.fetchall()}

    debug_id  = topics.get('/rpp/debug')
    seg_id    = topics.get('/rpp/segment_debug')
    pose_id   = topics.get('/mavros/local_position/pose')

    debug_rows, seg_rows, pose_rows = [], [], []
    seg_n = 0

    if debug_id:
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (debug_id,))
        rows = c.fetchall()
        t0 = rows[0][0]*1e-9 if rows else 0
        for ts, raw in rows:
            v = decode_debug(raw)
            if v:
                debug_rows.append([ts*1e-9-t0]+v)

    if seg_id:
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (seg_id,))
        rows = c.fetchall()
        t0s = rows[0][0]*1e-9 if rows else 0
        for ts, raw in rows:
            v, n = decode_seg_debug(raw)
            if v:
                seg_rows.append([ts*1e-9-t0s]+v)
                seg_n = max(seg_n, n)

    if pose_id:
        c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (pose_id,))
        rows = c.fetchall()
        t0p = rows[0][0]*1e-9 if rows else 0
        for ts, raw in rows:
            r = decode_pose(raw)
            if r:
                pose_rows.append((ts*1e-9-t0p, r[0], r[1]))

    conn.close()
    return debug_rows, seg_rows, seg_n, pose_rows

# ─────────────────────────────────────────────────────────────────────────────

BAGS = [
    ("192326", Path("/Users/dyx_a1/Vetri/PX4_DXP/bags/17-06-2026 /square_2x2.dxf_20260617_192326/square_2x2.dxf_20260617_192326_0.db3")),
    ("193104", Path("/Users/dyx_a1/Vetri/PX4_DXP/bags/17-06-2026 /square_2x2.dxf_20260617_193104/square_2x2.dxf_20260617_193104_0.db3")),
]

results = {}

for tag, db3 in BAGS:
    print(f"\n{'='*68}")
    print(f"  BAG: square_2x2.dxf_{tag}")
    print(f"{'='*68}")

    debug_rows, seg_rows, seg_n, pose_rows = read_bag(db3)
    ok_pct = len(debug_rows) / 5866 * 100 if tag == '192326' else len(debug_rows) / 2441 * 100
    print(f"  /rpp/debug : {len(debug_rows)} decoded")
    print(f"  /rpp/seg   : {len(seg_rows)} decoded  (n_fields={seg_n})")
    print(f"  /pose      : {len(pose_rows)} decoded")

    if not debug_rows:
        print("  ERROR: no debug data"); continue

    arr  = np.array(debug_rows)     # col0=t, cols 1..47 = fields[0..46]
    t    = arr[:, 0]
    cte  = arr[:, F_CTE  + 1]
    hdg  = arr[:, F_HDG  + 1]
    ld   = arr[:, F_LD   + 1]
    spd  = arr[:, F_SPD  + 1]
    kap  = arr[:, F_KAP  + 1]
    state= arr[:, F_STATE+ 1]
    yaw  = arr[:, F_YAW  + 1]
    yrcd = arr[:, F_YRCD + 1]

    # Params (from last message)
    last = arr[-1, 1:]
    max_yr   = last[P_MAXYR]
    min_la   = last[P_MINLA]
    max_la   = last[P_MAXLA]
    la_time  = last[P_LATIME]
    max_v    = last[P_MAXV]
    a_lat    = last[P_ALAT]
    c_rad    = last[P_CRAD]
    # max_yaw_rate_body from param field 35 (0.45 in AGENTS.md)
    yr_body  = last[35] if len(last) > 35 else float('nan')

    print(f"\n  PARAMS from telemetry:")
    print(f"    max_yaw_rate (ctrl) = {max_yr:.3f} rad/s")
    print(f"    max_yaw_rate_body   = {yr_body:.3f} rad/s")
    print(f"    min/max_lookahead   = {min_la:.3f} / {max_la:.3f} m")
    print(f"    lookahead_time      = {la_time:.3f} s")
    print(f"    max_linear_vel      = {max_v:.3f} m/s")
    print(f"    a_lat_max           = {a_lat:.4f} m/s²")
    print(f"    corner_smooth_rad   = {c_rad:.3f} m")

    # State distribution
    unique_states, cnts = np.unique(state, return_counts=True)
    print(f"\n  State distribution ({len(arr)} total samples):")
    for sv, cnt in zip(unique_states, cnts):
        print(f"    state={sv:.0f}: {cnt:5d} ({cnt/len(arr)*100:5.1f}%)")

    # Use tracking state if available
    tracking = (state == 1) | (state == 2) | (state == 3)
    if tracking.sum() >= 50:
        d = arr[tracking]
        slabel = f"TRACKING (state 1|2|3): {int(tracking.sum())} samples"
    else:
        d = arr
        slabel = f"ALL states ({len(d)} samples)"
    print(f"  Using: {slabel}")

    cte_d  = d[:, F_CTE  + 1]
    hdg_d  = d[:, F_HDG  + 1]
    ld_d   = d[:, F_LD   + 1]
    spd_d  = d[:, F_SPD  + 1]
    kap_d  = d[:, F_KAP  + 1]
    yrcd_d = d[:, F_YRCD + 1]

    abs_cte  = np.abs(cte_d)
    abs_hdg  = np.abs(hdg_d)
    abs_yrcd = np.abs(yrcd_d)

    print(f"\n  ── CROSS-TRACK ERROR ──────────────────────────────────────")
    print(f"  median |CTE|  = {np.median(abs_cte)*100:.2f} cm")
    print(f"  mean   |CTE|  = {np.mean(abs_cte)*100:.2f} cm")
    print(f"  RMS    CTE    = {np.sqrt(np.mean(cte_d**2))*100:.2f} cm")
    print(f"  p75    |CTE|  = {np.percentile(abs_cte,75)*100:.2f} cm")
    print(f"  p90    |CTE|  = {np.percentile(abs_cte,90)*100:.2f} cm")
    print(f"  p95    |CTE|  = {np.percentile(abs_cte,95)*100:.2f} cm")
    print(f"  max    |CTE|  = {np.max(abs_cte)*100:.2f} cm")
    pos_bias = np.mean(cte_d > 0)*100
    print(f"  bias: {pos_bias:.0f}% pos (right of path), {100-pos_bias:.0f}% neg")
    print(f"  >5cm: {np.mean(abs_cte>0.05)*100:.1f}%  |  >10cm: {np.mean(abs_cte>0.10)*100:.1f}%  |  >20cm: {np.mean(abs_cte>0.20)*100:.1f}%")

    print(f"\n  ── HEADING ERROR ──────────────────────────────────────────")
    print(f"  median |hdg|  = {np.degrees(np.median(abs_hdg)):.2f}°")
    print(f"  p75    |hdg|  = {np.degrees(np.percentile(abs_hdg,75)):.2f}°")
    print(f"  p90    |hdg|  = {np.degrees(np.percentile(abs_hdg,90)):.2f}°")
    print(f"  max    |hdg|  = {np.degrees(np.max(abs_hdg)):.2f}°")

    print(f"\n  ── LOOKAHEAD DISTANCE ─────────────────────────────────────")
    print(f"  median ld     = {np.median(ld_d):.3f} m")
    print(f"  range         = [{np.min(ld_d):.3f} .. {np.max(ld_d):.3f}] m")
    print(f"  at min clamp  = {np.mean(ld_d <= min_la+0.001)*100:.1f}%  (min={min_la:.3f}m)")
    print(f"  at max clamp  = {np.mean(ld_d >= max_la-0.001)*100:.1f}%  (max={max_la:.3f}m)")

    print(f"\n  ── SPEED COMMAND ──────────────────────────────────────────")
    print(f"  median spd    = {np.median(spd_d):.3f} m/s")
    print(f"  mean   spd    = {np.mean(spd_d):.3f} m/s")
    print(f"  max    spd    = {np.max(spd_d):.3f} m/s")
    moving = spd_d > 0.01
    if moving.any():
        print(f"  min (moving)  = {np.min(spd_d[moving]):.3f} m/s")
    print(f"  stopped <5cm/s= {np.mean(spd_d<0.05)*100:.1f}%")
    print(f"  speed reduct  = {np.mean(spd_d < max_v*0.7)*100:.1f}%  (samples below 70% max)")

    print(f"\n  ── CURVATURE (kappa) ──────────────────────────────────────")
    nonzero = np.abs(kap_d[np.abs(kap_d)>0.05])
    if len(nonzero):
        mk, maxk = np.median(nonzero), np.max(nonzero)
        print(f"  median |kappa|= {mk:.4f} 1/m  (R={1/max(mk,1e-4):.2f} m)")
        print(f"  max    |kappa|= {maxk:.4f} 1/m  (R_min={1/max(maxk,1e-4):.2f} m)")
    print(f"  |kappa|>0.3   = {np.mean(np.abs(kap_d)>0.3)*100:.1f}% (turning)")
    print(f"  |kappa|>1.0   = {np.mean(np.abs(kap_d)>1.0)*100:.1f}% (tight turn)")

    print(f"\n  ── YAW RATE CMD ───────────────────────────────────────────")
    print(f"  median |yr|   = {np.median(abs_yrcd):.4f} rad/s")
    print(f"  p90    |yr|   = {np.percentile(abs_yrcd,90):.4f} rad/s")
    print(f"  max    |yr|   = {np.max(abs_yrcd):.4f} rad/s")
    if yr_body > 0:
        sat = np.mean(abs_yrcd >= yr_body*0.98)*100
        print(f"  sat @ {yr_body:.3f}r/s= {sat:.1f}%  (yaw_rate_body limit)")
    if max_yr > 0:
        sat2 = np.mean(abs_yrcd >= max_yr*0.98)*100
        print(f"  sat @ {max_yr:.3f}r/s= {sat2:.1f}%  (controller limit)")

    print(f"\n  ── SEGMENT DEBUG ──────────────────────────────────────────")
    if seg_rows and seg_n > 0:
        sarr = np.array(seg_rows)
        # From probe: [1.0, 2.0, 0.0, 0.452, 0.452, NaN, 1.573, -0.002, -0.097, -0.088]
        # field[0]=seg_id, [1]=next_wp, [2]=progress, [3]=seg_len, [4]=dist_next
        # field[6]=seg_heading_rad, [7]=xtrack_signed, [8]=xtrack_lookahead, [9]=?
        print(f"  rows={len(sarr)}, n_fields={seg_n}")
        seg_labels = ['seg_id','next_wp','progress','seg_len_m','dist_next_m',
                      'f5','seg_hdg_rad','xtrack_m','xtrack_la_m','f9']
        for fi in range(min(seg_n, 10)):
            col = sarr[:, fi+1]
            valid = col[np.isfinite(col)]
            if len(valid) == 0:
                continue
            lbl = seg_labels[fi] if fi < len(seg_labels) else f'f{fi}'
            print(f"    {lbl:<16}: med={np.median(valid):8.4f}  "
                  f"std={np.std(valid):7.4f}  "
                  f"p95={np.percentile(np.abs(valid),95)*100:7.2f}cm  "
                  f"max={np.max(np.abs(valid))*100:7.2f}cm")
        # Focus on xtrack (field 7)
        if seg_n >= 8:
            xt = sarr[:, 8]  # field[7]
            xt = xt[np.isfinite(xt)]
            print(f"\n  Segment xtrack (field[7]) — main xtrack metric:")
            print(f"    median |xt|  = {np.median(np.abs(xt))*100:.2f} cm")
            print(f"    RMS    xt    = {np.sqrt(np.mean(xt**2))*100:.2f} cm")
            print(f"    p90    |xt|  = {np.percentile(np.abs(xt),90)*100:.2f} cm")
            print(f"    p95    |xt|  = {np.percentile(np.abs(xt),95)*100:.2f} cm")
            print(f"    max    |xt|  = {np.max(np.abs(xt))*100:.2f} cm")
    else:
        print(f"  No segment_debug decoded")

    print(f"\n  ── TRAJECTORY ─────────────────────────────────────────────")
    if pose_rows:
        parr = np.array(pose_rows)
        east, north = parr[:,1], parr[:,2]
        dur = parr[-1,0] - parr[0,0]
        dist = np.sum(np.sqrt(np.diff(east)**2+np.diff(north)**2))
        print(f"  Pose samples = {len(parr)}")
        print(f"  Duration     = {dur:.1f} s ({dur/60:.1f} min)")
        print(f"  East  span   = {east.max()-east.min():.3f} m  [{east.min():.3f}..{east.max():.3f}]")
        print(f"  North span   = {north.max()-north.min():.3f} m  [{north.min():.3f}..{north.max():.3f}]")
        print(f"  Total dist~  = {dist:.2f} m")
        print(f"  Avg speed    = {dist/dur:.3f} m/s")
    else:
        print(f"  No pose decoded (check CDR offset)")

    dur = arr[-1,0]
    print(f"\n  Duration (debug) = {dur:.1f} s ({dur/60:.1f} min)")

    results[tag] = {
        'cte_median': np.median(abs_cte)*100,
        'cte_rms':    np.sqrt(np.mean(cte_d**2))*100,
        'cte_p75':    np.percentile(abs_cte,75)*100,
        'cte_p90':    np.percentile(abs_cte,90)*100,
        'cte_p95':    np.percentile(abs_cte,95)*100,
        'cte_max':    np.max(abs_cte)*100,
        'cte_gt5':    np.mean(abs_cte>0.05)*100,
        'cte_gt10':   np.mean(abs_cte>0.10)*100,
        'hdg_med':    np.degrees(np.median(abs_hdg)),
        'hdg_p90':    np.degrees(np.percentile(abs_hdg,90)),
        'hdg_max':    np.degrees(np.max(abs_hdg)),
        'spd_med':    np.median(spd_d),
        'spd_max':    np.max(spd_d),
        'stopped_pct':np.mean(spd_d<0.05)*100,
        'yr_max':     np.max(abs_yrcd),
        'yr_sat_pct': np.mean(abs_yrcd>=yr_body*0.98)*100 if yr_body>0 else 0,
        'ld_med':     np.median(ld_d),
        'kap_p90':    np.percentile(np.abs(kap_d),90),
        'n_debug':    len(debug_rows),
        'n_seg':      len(seg_rows),
        'n_pose':     len(pose_rows),
        'duration':   dur,
        'max_v':      max_v,
        'a_lat':      a_lat,
        'c_rad':      c_rad,
        'yr_body':    yr_body,
    }

print(f"\n\n{'='*68}")
print("  SIDE-BY-SIDE COMPARISON")
print(f"{'='*68}")
if len(results) == 2:
    r1, r2 = results['192326'], results['193104']
    metrics = [
        ("CTE median (cm)",     'cte_median'),
        ("CTE RMS (cm)",        'cte_rms'),
        ("CTE p75 (cm)",        'cte_p75'),
        ("CTE p90 (cm)",        'cte_p90'),
        ("CTE p95 (cm)",        'cte_p95'),
        ("CTE max (cm)",        'cte_max'),
        ("CTE >5cm (%)",        'cte_gt5'),
        ("CTE >10cm (%)",       'cte_gt10'),
        ("Hdg med (°)",         'hdg_med'),
        ("Hdg p90 (°)",         'hdg_p90'),
        ("Hdg max (°)",         'hdg_max'),
        ("Speed median (m/s)",  'spd_med'),
        ("Speed max (m/s)",     'spd_max'),
        ("Stopped <5cm/s (%)",  'stopped_pct'),
        ("YawRate max (rad/s)", 'yr_max'),
        ("YawRate sat (%)",     'yr_sat_pct'),
        ("Lookahead med (m)",   'ld_med'),
        ("Kappa p90 (1/m)",     'kap_p90'),
        ("Duration (s)",        'duration'),
        ("Max vel param (m/s)", 'max_v'),
        ("a_lat_max (m/s²)",    'a_lat'),
        ("corner_radius (m)",   'c_rad'),
        ("yr_body limit (r/s)", 'yr_body'),
        ("N debug decoded",     'n_debug'),
        ("N seg decoded",       'n_seg'),
        ("N pose decoded",      'n_pose'),
    ]
    print(f"  {'Metric':<30} {'192326':>10} {'193104':>10}")
    print(f"  {'-'*52}")
    for label, key in metrics:
        v1 = r1.get(key, float('nan'))
        v2 = r2.get(key, float('nan'))
        print(f"  {label:<30} {v1:>10.3f} {v2:>10.3f}")

print("\nDone.")
