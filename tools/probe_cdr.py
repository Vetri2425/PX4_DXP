#!/usr/bin/env python3
"""Probe the raw CDR bytes of /rpp/debug and /mavros/local_position/pose to understand the layout."""
import sqlite3, struct, math
from pathlib import Path

DB = Path("/Users/dyx_a1/Vetri/PX4_DXP/bags/17-06-2026 /square_2x2.dxf_20260617_192326/square_2x2.dxf_20260617_192326_0.db3")

conn = sqlite3.connect(str(DB))
c = conn.cursor()
c.execute("SELECT id, name FROM topics")
topics = {row[1]: row[0] for row in c.fetchall()}
print("Topics:", topics)

for topic, tid in topics.items():
    c.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (tid,))
    cnt = c.fetchone()[0]
    print(f"  {topic} (id={tid}): {cnt} messages")

print()

def show_msg(name, tid, n_msgs=1):
    c.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT ?", (tid, n_msgs))
    rows = c.fetchall()
    for i, (ts, raw) in enumerate(rows):
        raw = bytes(raw)
        print(f"\n── {name} msg[{i}]  len={len(raw)} bytes  ts={ts} ──")
        # hex dump first 128 bytes
        for off in range(0, min(128, len(raw)), 16):
            chunk = raw[off:off+16]
            hexpart = ' '.join(f'{b:02x}' for b in chunk)
            print(f"  {off:4d}: {hexpart}")
        # Try to find float32 run
        print(f"  Scanning for float32 sequences...")
        for start in range(0, len(raw)-4, 4):
            try:
                vals = []
                for j in range(min(50, (len(raw)-start)//4)):
                    v = struct.unpack_from('<f', raw, start + j*4)[0]
                    if not math.isfinite(v) or abs(v) > 1e6:
                        break
                    vals.append(v)
                if len(vals) >= 8:
                    print(f"    @{start}: {len(vals)} floats: {[f'{v:.4f}' for v in vals[:15]]}")
                    break
            except:
                pass
        # Try uint32 scan
        print(f"  Scanning for uint32 size fields...")
        for start in range(0, min(len(raw)-4, 200), 4):
            n = struct.unpack_from('<I', raw, start)[0]
            if 10 <= n <= 100:
                print(f"    @{start}: uint32={n} (possible array len)")

debug_id = topics.get('/rpp/debug')
pose_id  = topics.get('/mavros/local_position/pose')
segdbg_id = topics.get('/rpp/segment_debug')

if debug_id:
    show_msg('/rpp/debug', debug_id, 2)
if segdbg_id:
    show_msg('/rpp/segment_debug', segdbg_id, 1)
if pose_id:
    show_msg('/mavros/local_position/pose', pose_id, 1)

conn.close()
