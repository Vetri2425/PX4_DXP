#!/usr/bin/env python3
"""Analyze a bag for extension-path tracking: planned /path vs actual pose, xtrack."""
import sqlite3, sys, math, os
sys.path.insert(0, os.path.dirname(__file__))
import extract_rosbag_direct as E

BAG = sys.argv[1]
db = [f for f in os.listdir(BAG) if f.endswith('.db3')][0]
con = sqlite3.connect(os.path.join(BAG, db))

tid = {n: i for i, n, _ in con.execute('select id,name,type from topics')}

def rows(name):
    return con.execute(
        'select timestamp,data from messages where topic_id=? order by timestamp',
        (tid[name],)).fetchall()

# ---- full Path parser via per-pose frame_id markers (robust to CDR padding) ----
import re, struct
def parse_full_path(data):
    # poses_count
    off = E.skip_cdr_header(data, 0)
    _, off = E.read_int32(data, off); _, off = E.read_uint32(data, off)
    _, off = E.read_string(data, off)            # path frame_id
    n, off = E.read_uint32(data, off)
    # each PoseStamped carries its own frame_id 'local_ned'; the FIRST match is
    # the path-level frame_id, so skip it.
    marks = [m.start() for m in re.finditer(b'local_ned', data)][1:]
    pts = []
    for L in marks[:n]:
        end = (L + 10 + 3) & ~3               # past 'local_ned\0', align4
        x, y, z = struct.unpack_from('<ddd', data, end)
        # /path frame_id == 'local_ned': x=north, y=east. Convert to ENU (E,N).
        pts.append((y, x))
    return pts

# planned path (ENU x=east,y=north)
path_msgs = rows('/path')
path = parse_full_path(path_msgs[0][1])
print(f"/path: {len(path)} waypoints")

# actual pose (ENU x=east, y=north)
poses = []
for ts, data in rows('/mavros/local_position/pose'):
    m = E.parse_PoseStamped(data)
    poses.append((ts, m['position_x'], m['position_y']))
print(f"poses: {len(poses)}")

# rpp/debug xtrack (m, signed) + rpp_state
dbg = []
for ts, data in rows('/rpp/debug'):
    m = E.parse_Float32MultiArray(data)
    d = [m.get(f'data_{i}', float('nan')) for i in range(m['data_count'])]
    dbg.append((ts, d))
print(f"rpp/debug: {len(dbg)}")

# spray active
spray = []
for ts, data in rows('/spray/active'):
    off = E.skip_cdr_header(data, 0)
    b, _ = E.read_bool(data, off)
    spray.append((ts, b))

# ---- geometric xtrack: dist from pose to planned polyline ----
def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx-ax, by-ay
    L2 = dx*dx+dy*dy
    if L2 == 0: return math.hypot(px-ax, py-ay), 0.0
    t = max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/L2))
    fx, fy = ax+t*dx, ay+t*dy
    return math.hypot(px-fx, py-fy), t

def min_dist_to_path(px, py, pts):
    best, bi = 1e9, -1
    for i in range(len(pts)-1):
        d, t = seg_dist(px, py, pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1])
        if d < best:
            best, bi = d, i
    return best, bi

# path bbox + length
xs = [p[0] for p in path]; ys = [p[1] for p in path]
plen = sum(math.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1]) for i in range(len(path)-1))
print(f"path bbox x[{min(xs):.2f},{max(xs):.2f}] y[{min(ys):.2f},{max(ys):.2f}] len={plen:.2f}m")
print("waypoints (east,north):")
for i, p in enumerate(path):
    print(f"  [{i}] {p[0]:+.3f}, {p[1]:+.3f}")

# geometric xtrack over trajectory
gxt = []
for ts, x, y in poses:
    d, bi = min_dist_to_path(x, y, path)
    gxt.append((ts, d*100.0, bi))
vals = [v for _, v, _ in gxt]
print(f"\nGEOMETRIC xtrack (cm): mean={sum(vals)/len(vals):.2f} "
      f"rms={math.sqrt(sum(v*v for v in vals)/len(vals)):.2f} "
      f"max={max(vals):.2f} p95={sorted(vals)[int(len(vals)*0.95)]:.2f}")

# rpp-reported xtrack
rxt = [abs(d[0])*100 for _, d in dbg if d and math.isfinite(d[0])]
if rxt:
    print(f"RPP-reported |xtrack| (cm): mean={sum(rxt)/len(rxt):.2f} "
          f"rms={math.sqrt(sum(v*v for v in rxt)/len(rxt)):.2f} max={max(rxt):.2f}")

# per-segment worst tracking — which segments deviate most
seg_err = {}
for ts, v, bi in gxt:
    seg_err.setdefault(bi, []).append(v)
print("\nper-segment xtrack (cm)  [seg i = path[i]->path[i+1]]:")
for i in sorted(seg_err):
    e = seg_err[i]
    a, b = path[i], path[i+1]
    seglen = math.hypot(b[0]-a[0], b[1]-a[1])
    print(f"  seg[{i:2d}] len={seglen:5.2f}m  n={len(e):4d}  "
          f"mean={sum(e)/len(e):6.2f}  max={max(e):6.2f}")

# where is the worst deviation in time/space
worst = max(gxt, key=lambda r: r[1])
wts, wv, wbi = worst
wp = next((x, y) for ts, x, y in poses if ts == wts)
t0 = poses[0][0]
print(f"\nWORST point: t={ (wts-t0)/1e9:.1f}s  xtrack={wv:.1f}cm  "
      f"at (E={wp[0]:.2f},N={wp[1]:.2f})  nearest seg[{wbi}]")

# ---- plot ----
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig, ax = plt.subplots(figsize=(11, 9))
pe = [p[0] for p in path]; pn = [p[1] for p in path]
ax.plot(pe, pn, '-', color='tab:blue', lw=1.2, label='planned /path (incl. extensions)', zorder=2)
ax.plot(pe, pn, '.', color='tab:blue', ms=3, zorder=3)
ax.plot(path[0][0], path[0][1], 'o', color='green', ms=12, label='path start', zorder=5)
# actual trajectory colored by geometric xtrack
ae = [x for _, x, _ in poses]; an = [y for _, _, y in poses]
sc = ax.scatter(ae, an, c=[v for _, v, _ in gxt], cmap='inferno', s=6, vmin=0, vmax=10,
                label='actual (color=xtrack cm)', zorder=4)
plt.colorbar(sc, label='cross-track error (cm)')
# mark seg[0] (the long transit/extension)
ax.plot([path[0][0], path[1][0]], [path[0][1], path[1][1]], '-', color='red', lw=2.5,
        alpha=0.5, label='seg[0] transit/extension (12.7m)', zorder=1)
ax.set_xlabel('East (m)'); ax.set_ylabel('North (m)'); ax.axis('equal')
ax.legend(loc='best', fontsize=9); ax.grid(alpha=0.3)
ax.set_title('square_2x2 20260617_192326 — planned vs actual (extension tracking)')
out = '/tmp/ext_traj_192326.png'
plt.savefig(out, dpi=130, bbox_inches='tight')
print('saved', out)
