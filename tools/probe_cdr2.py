#!/usr/bin/env python3
"""
Detailed field-by-field probe of /rpp/debug for square bags.
Goal: correctly identify CTE, hdg_err, speed, state, kappa, yaw_rate fields.
"""
import sqlite3, struct, math
import numpy as np
from pathlib import Path

DB = Path("/Users/dyx_a1/Vetri/PX4_DXP/bags/17-06-2026 /square_2x2.dxf_20260617_192326/square_2x2.dxf_20260617_192326_0.db3")

conn = sqlite3.connect(str(DB))
c = conn.cursor()
c.execute("SELECT id, name FROM topics")
topics = {row[1]: row[0] for row in c.fetchall()}

debug_id = topics['/rpp/debug']

# Get ALL debug messages
c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (debug_id,))
rows = c.fetchall()
print(f"Total /rpp/debug messages: {len(rows)}")

def decode(data):
    data = bytes(data)
    # From probe: floats found @20 (offset into data array starting at @20)
    # The CDR Float32MultiArray layout:
    #   [0..3]   CDR encapsulation header (00 01 00 00)
    #   [4..7]   dim_count (uint32)
    #   [8..11]  dim[0].label length (uint32) = 10 ("rpp_debug\0")
    #   [12..21] "rpp_debug\0" (10 bytes)
    #   [22..23] padding to 4-byte align → offset 24
    #   [24..27] dim[0].size (uint32)
    #   [28..31] dim[0].stride (uint32)
    #   [32..35] data_offset (uint32)
    #   [36..39] data_count (uint32) = 47
    #   [40..227] 47 floats (47*4 = 188 bytes)
    # Total = 40 + 188 = 228 ✓

    try:
        data_count = struct.unpack_from('<I', data, 36)[0]
        if data_count == 47:
            vals = struct.unpack_from('<47f', data, 40)
            if all(math.isfinite(v) for v in vals):
                return list(vals)
    except:
        pass
    return []

# Decode all
all_vals = []
t0 = rows[0][0] * 1e-9
for ts, raw in rows:
    v = decode(raw)
    if v:
        all_vals.append([ts*1e-9 - t0] + v)

print(f"Decoded: {len(all_vals)}/{len(rows)}")
arr = np.array(all_vals)
# arr[:,0] = time, arr[:,1..47] = fields[0..46]

print(f"\nAll 47 field statistics (median, std, min, max):")
print(f"{'Idx':>4} {'Median':>10} {'Std':>10} {'Min':>10} {'Max':>10}  Note")
print("-" * 65)

# Known param values from the validated RPP config in AGENTS.md:
# max_yaw_rate_body=0.45, a_lat_max=0.3, corner_smooth_radius_m=0.5
# From fields 15..46 we saw: [0.15, 0.52, 1.0, 1.6, 0.3, 0.3, 0.02, 0.5, 0.6, 0.1, 0.02, 0.5, 0.05, 1.0, 4.0, 0.05, 0.08, 0.5, 6.0, 0.0, 0.1, 1.0, 0.0, 0.45, 0.35, 0.5, 0.35, 1.0, 1.0, 45.0, 0.5, 0.08]
# max_yaw_rate=0.45 is at field[38] in 193104 output = arr[:,39]

# Annotate known params
known = {
    # indices are 0-based field indices (arr col = idx+1)
    # will be filled after we identify them
}

for i in range(47):
    col = arr[:, i+1]
    med = np.median(col)
    std = np.std(col)
    mn  = np.min(col)
    mx  = np.max(col)

    note = ""
    # Heuristic annotations
    if std < 1e-5 and abs(med) > 0:
        note = "CONST param"
    elif std < 0.001 and abs(med) < 0.001:
        note = "always ~0"
    elif 0 <= med <= 5 and std > 0.01 and std < 1:
        note = "← dynamic small"
    elif abs(med) > 10:
        note = "← large value"

    print(f"  [{i:2d}] {med:10.4f} {std:10.4f} {mn:10.4f} {mx:10.4f}  {note}")

# Look at time series of first 20 fields to find state transitions
print(f"\nTime series of fields 0..14 for first 30 messages:")
print("t     " + "  ".join(f"f{i:02d}" for i in range(15)))
for row in all_vals[:30]:
    t = row[0]
    vals = row[1:16]
    print(f"{t:5.2f} " + "  ".join(f"{v:5.3f}" for v in vals))

# Find where state changes (look for fields that jump discretely)
print(f"\nLooking for state-like fields (values in {{0,1,2,3,4}}, integer):")
for i in range(47):
    col = arr[:, i+1]
    unique_vals = np.unique(np.round(col, 2))
    if len(unique_vals) <= 6 and all(abs(v - round(v)) < 0.01 for v in unique_vals) and np.max(col) <= 5:
        print(f"  field[{i}]: unique values = {sorted(unique_vals)}")

# Look for CTE: should have both positive and negative values, small range
print(f"\nLooking for CTE-like fields (signed, |median|<0.5m, min<0):")
for i in range(47):
    col = arr[:, i+1]
    if col.min() < -0.001 and col.max() > 0.001 and abs(np.median(col)) < 0.5:
        print(f"  field[{i}]: med={np.median(col):.4f}  std={np.std(col):.4f}  min={col.min():.4f}  max={col.max():.4f}")

conn.close()
