#!/usr/bin/env python3
"""Analyze the square corner-fix bag without ROS2 (manual CDR parse).

Answers one question: did the velocity-vector corner pivot work — i.e. did the
rover advance through all corners of the square instead of deadlocking at the
first one (the bug in bag square_20260611_170539)?
"""
import sqlite3
import struct
import math
import sys

DB = "bags/square_cornerfix_20260611_174508/square_cornerfix_20260611_174508_0.db3"

SEG_STATE = {0: "INACTIVE", 1: "TRACK_SEGMENT", 2: "PRE_CORNER_SLOWDOWN",
             3: "CORNER_ALIGN", 4: "DONE"}


def rows(cur, name):
    tid = cur.execute("SELECT id FROM topics WHERE name=?", (name,)).fetchone()[0]
    return cur.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid,)).fetchall()


def align(off, n):
    return off + (-off % n)


def f32(d, o):
    return struct.unpack_from("<f", d, o)[0]


def f64arr_header_skip(d):
    # CDR: 4-byte encapsulation header
    return 4


def parse_pose(d):
    # PoseStamped ends with Pose{position 3xf64, orientation 4xf64} = 56 bytes.
    px, py, pz, ox, oy, oz, ow = struct.unpack_from("<7d", d, len(d) - 56)
    yaw_enu = math.atan2(2*(ow*oz + ox*oy), 1 - 2*(oy*oy + oz*oz))
    return px, py, pz, yaw_enu  # ENU x=East,y=North


def parse_vec3(d):
    # Vector3Stamped ends with Vector3(3xf64) — read the trailing 24 bytes.
    x, y, z = struct.unpack_from("<3d", d, len(d) - 24)
    return x, y, z  # NED: x=N, y=E


def parse_f32multiarray(d):
    # Float32MultiArray: layout(dim seq of {label string,size u32,stride u32}, data_offset u32) + data seq f32
    o = 4
    o = align(o, 4)
    ndim = struct.unpack_from("<I", d, o)[0]; o += 4
    for _ in range(ndim):
        o = align(o, 4)
        slen = struct.unpack_from("<I", d, o)[0]; o += 4 + slen
        o = align(o, 4); o += 4  # size
        o += 4                   # stride
    o = align(o, 4); o += 4      # data_offset
    o = align(o, 4)
    n = struct.unpack_from("<I", d, o)[0]; o += 4
    o = align(o, 4)
    return list(struct.unpack_from("<%df" % n, d, o))


def parse_state(d):
    # mavros State: header + connected(bool) guided(bool)... mode(string) — we only need armed+mode
    o = 4
    o = align(o, 4); o += 4; o += 4
    o = align(o, 4)
    slen = struct.unpack_from("<I", d, o)[0]; o += 4 + slen  # frame_id
    connected = d[o]; o += 1
    armed = d[o]; o += 1
    guided = d[o]; o += 1
    manual_input = d[o]; o += 1
    o = align(o, 4)
    mlen = struct.unpack_from("<I", d, o)[0]; o += 4
    mode = d[o:o+mlen-1].decode("ascii", "replace")
    return bool(armed), mode


def parse_path(d):
    # nav_msgs/Path: header + poses seq of PoseStamped. Extract (x,y) NED.
    o = 4
    o = align(o, 4); o += 4; o += 4
    o = align(o, 4)
    slen = struct.unpack_from("<I", d, o)[0]; o += 4 + slen
    o = align(o, 4)
    npose = struct.unpack_from("<I", d, o)[0]; o += 4
    pts = []
    for _ in range(npose):
        o = align(o, 4); o += 4; o += 4              # stamp
        o = align(o, 4)
        sl = struct.unpack_from("<I", d, o)[0]; o += 4 + sl
        o = align(o, 8)
        x, y, z = struct.unpack_from("<3d", d, o); o += 24
        o += 32                                       # orientation 4xf64
        pts.append((x, y))                            # NED x=N,y=E
    return pts


def main():
    c = sqlite3.connect(DB); cur = c.cursor()
    t0 = cur.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]

    # ---- state window ----
    states = [(ts, *parse_state(d)) for ts, d in rows(cur, "/mavros/state")]
    offb = [(ts-t0)/1e9 for ts, a, m in states if m == "OFFBOARD"]
    armed_t = [(ts-t0)/1e9 for ts, a, m in states if a]
    modes = sorted(set(m for _, _, m in states))
    print("=== STATE ===")
    print("modes seen:", modes)
    if offb:
        print(f"OFFBOARD: {offb[0]:.1f}s .. {offb[-1]:.1f}s")
    if armed_t:
        print(f"armed from ~{armed_t[0]:.1f}s")

    # ---- desired square ----
    paths = rows(cur, "/path")
    pts = []
    for _, d in reversed(paths):
        try:
            pts = parse_path(d)
            if pts:
                break
        except Exception as ex:
            pts = []
    print("\n=== DESIRED PATH ===")
    if pts:
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
        print(f"{len(pts)} waypoints; N(x) {min(xs):.2f}..{max(xs):.2f}  E(y) {min(ys):.2f}..{max(ys):.2f}")
    else:
        print("(path parse failed — continuing; xtrack will fall back to pose extent)")

    # ---- segment_debug: the key signal ----
    seg = [( (ts-t0)/1e9, parse_f32multiarray(d)) for ts, d in rows(cur, "/rpp/segment_debug")]
    # transitions of (state, seg_idx)
    print("\n=== SEGMENT STATE / INDEX TIMELINE (transitions only) ===")
    prev = None
    corner_aligns = 0
    max_idx = 0
    for t, a in seg:
        if len(a) < 9:
            continue
        st = int(a[1]); idx = int(a[2]); herr = a[7]
        max_idx = max(max_idx, idx)
        key = (st, idx)
        if key != prev:
            print(f"  t={t:5.1f}s  {SEG_STATE.get(st,st):<20} seg_idx={idx}  heading_err={math.degrees(herr):+6.1f}°")
            if st == 3:
                corner_aligns += 1
            prev = key
    print(f"max seg_idx reached = {max_idx}; CORNER_ALIGN entries = {corner_aligns}")

    # ---- velocity magnitude during CORNER_ALIGN ----
    # build time-indexed velocity list
    velrows = [((ts-t0)/1e9, parse_vec3(d)) for ts, d in rows(cur, "/rpp/velocity_ned")]
    # poses for actual-yaw correlation (NED yaw)
    poses_raw = [((ts-t0)/1e9, parse_pose(d)) for ts, d in rows(cur, "/mavros/local_position/pose")]
    yawrows = [(t, math.pi/2 - yenu) for t,(px,py,pz,yenu) in poses_raw]

    # ---- detailed CORNER_ALIGN window dump (every ~1s) ----
    print("\n=== CORNER_ALIGN WINDOW (cmd vel + commanded vs actual heading) ===")
    print("  t     segHerr  cmd_vN  cmd_vE  |cmd|   tgtHead  actualYaw")
    last_print = -999
    for t, a in seg:
        if len(a) < 9 or int(a[1]) != 3:
            continue
        if t - last_print < 1.0:
            continue
        last_print = t
        bv = min(velrows, key=lambda vr: abs(vr[0]-t))[1]
        by = min(yawrows, key=lambda yr: abs(yr[0]-t))[1]
        mag = math.hypot(bv[0], bv[1])
        print(f"  {t:5.1f}  {math.degrees(a[7]):+6.1f}°  {bv[0]:+.3f}  {bv[1]:+.3f}  "
              f"{mag:.3f}  {math.degrees(a[6]):+6.1f}°  {math.degrees(by):+6.1f}°")
    # correlate: for each seg sample in CORNER_ALIGN, nearest velocity
    ca_speeds = []
    vi = 0
    for t, a in seg:
        if len(a) < 9 or int(a[1]) != 3:
            continue
        # nearest velocity sample by time
        best = min(velrows, key=lambda vr: abs(vr[0]-t))
        vn, ve, vd = best[1]
        ca_speeds.append(math.hypot(vn, ve))
    print("\n=== VELOCITY DURING CORNER_ALIGN ===")
    if ca_speeds:
        print(f"  samples={len(ca_speeds)}  min={min(ca_speeds):.3f}  "
              f"max={max(ca_speeds):.3f}  mean={sum(ca_speeds)/len(ca_speeds):.3f} m/s")
        print(f"  zero-velocity (<0.01) samples: {sum(1 for s in ca_speeds if s < 0.01)}  "
              f"(>0 ⇒ firmware would freeze)")
    else:
        print("  (no CORNER_ALIGN samples)")

    # ---- actual motion + yaw sweep ----
    poses = [((ts-t0)/1e9, parse_pose(d)) for ts, d in rows(cur, "/mavros/local_position/pose")]
    # ENU->NED: pos_n=y, pos_e=x ; yaw_ned = pi/2 - yaw_enu
    P = [(t, py, px, (math.pi/2 - yenu)) for t,(px,py,pz,yenu) in poses]
    n0 = P[0]; nf = P[-1]
    travel = sum(math.hypot(P[i][1]-P[i-1][1], P[i][2]-P[i-1][2]) for i in range(1,len(P)))
    print("\n=== ACTUAL MOTION ===")
    print(f"start N={n0[1]:.2f} E={n0[2]:.2f}   end N={nf[1]:.2f} E={nf[2]:.2f}")
    print(f"travelled distance = {travel:.2f} m  (square perimeter ~8 m)")
    # yaw sweep: unwrap total absolute heading change
    yaws = [p[3] for p in P]
    tot = 0.0
    for i in range(1, len(yaws)):
        dd = (yaws[i]-yaws[i-1]+math.pi) % (2*math.pi) - math.pi
        tot += abs(dd)
    print(f"total |yaw| change = {math.degrees(tot):.0f}°  (full square ~360°)")

    # ---- xtrack vs desired polyline ----
    def seg_dist(p, a, b):
        ax,ay=a; bx,by=b; px,py=p
        dx,dy=bx-ax,by-ay
        L2=dx*dx+dy*dy
        if L2<1e-9: return math.hypot(px-ax,py-ay)
        t=max(0,min(1,((px-ax)*dx+(py-ay)*dy)/L2))
        return math.hypot(px-(ax+t*dx), py-(ay+t*dy))
    poly=pts
    errs=[]
    if poly and len(poly) >= 2:
      for t,n,e,y in P:
        if offb and t<offb[0]: continue
        d=min(seg_dist((n,e),poly[i],poly[i+1]) for i in range(len(poly)-1))
        errs.append(d)
    if errs:
        errs_sorted=sorted(errs)
        print("\n=== XTRACK vs desired square (after OFFBOARD) ===")
        print(f"  max={max(errs):.3f}  mean={sum(errs)/len(errs):.3f}  "
              f"median={errs_sorted[len(errs)//2]:.3f}  rms={math.sqrt(sum(e*e for e in errs)/len(errs)):.3f} m")

    return P, pts, offb


if __name__ == "__main__":
    main()
