#!/usr/bin/env python3
"""End-of-line-segment behaviour analysis for the triangle mission bag.

Uses the rosbags typestore (correct CDR decode) to read /path,
/rpp/segment_debug, /rpp/debug, /mavros/local_position/pose, /spray/active.

Goal: characterise what the controller does as it reaches the END of each
straight line segment (the triangle vertices) — stop / oscillate / fail to
advance / spray mistiming — since the same end-of-segment logic governs the
PRE→MARK→AFT extension boundaries.
"""
import math
from collections import defaultdict
from pathlib import Path as P
from rosbags.highlevel import AnyReader

BAG = P("/Users/dyx_a1/Vetri/PX4_DXP/bags/18-06-2026/triangle_202m.DXF_20260618_121813")

SEG_STATE = {0: "INACTIVE", 1: "TRACK", 2: "SLOWDOWN", 3: "ALIGN", 4: "DONE", 5: "STOP"}
RPP_STATE = {-1: "STALE", 0: "IDLE", 1: "TRACK", 2: "APPROACH", 3: "DONE", 4: "RTK_WAIT", 5: "JUMP_SKIP"}


def main():
    seg, dbg, pose = [], [], []
    path_poses = None
    with AnyReader([BAG]) as reader:
        conns = {c.topic: c for c in reader.connections}
        t0 = None
        for con, ts, raw in reader.messages():
            if t0 is None:
                t0 = ts
            t = (ts - t0) / 1e9
            topic = con.topic
            if topic == "/path" and path_poses is None:
                m = reader.deserialize(raw, con.msgtype)
                path_poses = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in m.poses]
            elif topic == "/rpp/segment_debug":
                v = reader.deserialize(raw, con.msgtype).data
                if len(v) >= 10:
                    seg.append({"t": t, "state": int(v[1]), "seg": int(v[2]),
                                "dseg": v[3], "dcorner": v[4], "cangle": v[5],
                                "herr": v[7], "yawcmd": v[8], "yawact": v[9]})
            elif topic == "/rpp/debug":
                v = reader.deserialize(raw, con.msgtype).data
                if len(v) >= 40:
                    dbg.append({"t": t, "xtrack": v[0], "speed": v[3], "dgoal": v[5],
                                "state": int(v[7]), "spray": v[39]})
            elif topic == "/mavros/local_position/pose":
                m = reader.deserialize(raw, con.msgtype)
                pose.append({"t": t, "e": m.pose.position.x, "n": m.pose.position.y})

    # ---- /path geometry ----
    pts = [(x, y) for x, y, z in path_poses]
    flags = [z > 0.5 for x, y, z in path_poses]
    print(f"=== /path: {len(pts)} waypoints ===")
    corners = [0]
    for i in range(1, len(pts) - 1):
        a, b, d = pts[i-1], pts[i], pts[i+1]
        h0 = math.atan2(b[1]-a[1], b[0]-a[0]); h1 = math.atan2(d[1]-b[1], d[0]-b[0])
        if math.degrees(abs(math.atan2(math.sin(h1-h0), math.cos(h1-h0)))) > 20:
            corners.append(i)
    corners.append(len(pts)-1)
    print(f"corners (>20deg) at wp {corners}:")
    for ci in corners:
        print(f"   wp[{ci}] = ({pts[ci][0]:.2f}N, {pts[ci][1]:.2f}E) spray={int(flags[ci])}")
    print(f"spray ON: {sum(flags)}/{len(flags)} ; flag transitions at wp "
          f"{[i for i in range(1,len(flags)) if flags[i]!=flags[i-1]]}")

    print(f"\n=== segment_debug: {len(seg)} samples over {seg[-1]['t']:.1f}s ===")
    print("\n--- state / seg-index change timeline ---")
    prev = None
    for s in seg:
        key = (s["state"], s["seg"])
        if key != prev:
            print(f"  t={s['t']:6.2f}s seg={s['seg']} {SEG_STATE.get(s['state']):8} "
                  f"dcorner={s['dcorner']:.3f} herr={math.degrees(s['herr']):+6.1f}d "
                  f"yawcmd={s['yawcmd']:+.3f} yawact={s['yawact']:+.3f}")
            prev = key

    dwell = defaultdict(float); cnt = defaultdict(int)
    for a, b in zip(seg, seg[1:]):
        dwell[a["state"]] += b["t"]-a["t"]; cnt[a["state"]] += 1
    print("\n--- time per segment-state ---")
    for st in sorted(dwell):
        print(f"  {SEG_STATE.get(st):8}: {dwell[st]:6.2f}s ({cnt[st]} samp)")

    print("\n--- CORNER STOP/ALIGN/SLOWDOWN episodes (end-of-line events) ---")
    i = 0
    while i < len(seg):
        if seg[i]["state"] in (2, 3, 5):
            j = i
            while j < len(seg) and seg[j]["state"] in (2, 3, 5):
                j += 1
            w = seg[i:j]; dur = w[-1]["t"]-w[0]["t"]
            ya = [x["yawact"] for x in w]
            flips = sum(1 for a, b in zip(ya, ya[1:]) if abs(a) > 0.05 and abs(b) > 0.05 and (a > 0) != (b > 0))
            print(f"  t={w[0]['t']:6.2f}->{w[-1]['t']:6.2f}s ({dur:4.1f}s) seg{w[0]['seg']}->{w[-1]['seg']} "
                  f"states={sorted({SEG_STATE.get(x['state']) for x in w})} "
                  f"herr {math.degrees(w[0]['herr']):+.0f}->{math.degrees(w[-1]['herr']):+.0f}d yaw-reversals={flips}")
            i = j
        else:
            i += 1

    print("\n--- speed/spray sampled ~2s ---")
    last = -100
    for d in dbg:
        if d["t"]-last >= 2.0:
            print(f"  t={d['t']:6.2f}s speed={d['speed']:.3f} xtrack={d['xtrack']*100:+6.1f}cm "
                  f"{RPP_STATE.get(d['state']):8} spray={d['spray']:.0f} dgoal={d['dgoal']:.2f}")
            last = d["t"]

    print("\n--- stalls (speed<0.02, state!=DONE, >1.0s) ---")
    i = 0; total = 0.0
    while i < len(dbg):
        if dbg[i]["speed"] < 0.02 and dbg[i]["state"] != 3:
            j = i
            while j < len(dbg) and dbg[j]["speed"] < 0.02 and dbg[j]["state"] != 3:
                j += 1
            dur = dbg[j-1]["t"]-dbg[i]["t"]
            if dur > 1.0:
                total += dur
                print(f"  STALL t={dbg[i]['t']:6.2f}->{dbg[j-1]['t']:6.2f}s ({dur:4.1f}s) "
                      f"{RPP_STATE.get(dbg[i]['state'])} spray={dbg[i]['spray']:.0f}")
            i = j
        else:
            i += 1
    print(f"  total non-DONE stall: {total:.1f}s of {dbg[-1]['t']:.1f}s")

    print(f"\n--- mission end ---")
    print(f"  seg_debug: seg={seg[-1]['seg']} {SEG_STATE.get(seg[-1]['state'])} t={seg[-1]['t']:.1f}s")
    print(f"  rpp_debug: {RPP_STATE.get(dbg[-1]['state'])} speed={dbg[-1]['speed']:.3f} dgoal={dbg[-1]['dgoal']:.2f}")
    if pose:
        ns = [p["n"] for p in pose]; es = [p["e"] for p in pose]
        print(f"  pose samples={len(pose)} N[{min(ns):.2f},{max(ns):.2f}] E[{min(es):.2f},{max(es):.2f}]")


if __name__ == "__main__":
    main()
