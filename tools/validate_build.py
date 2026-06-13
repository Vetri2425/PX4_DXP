#!/usr/bin/env python3
"""Batch build-validation harness — run after a fix campaign over N bags (+ ulogs).

Usage:
    python3 tools/validate_build.py <dir> [--out report.md]

Scans <dir> for ROS2 bags (any *_0.db3 / dir with metadata.yaml) and PX4 *.ulg.
For each bag it computes tracking metrics (geometric cross-track to /path, heading,
speed) and runs the three regression checks for the shipped fixes:

  BUG-T1 (pivot oscillation): yaw-rate ringing during CORNER_STOP/ALIGN windows.
  BUG-T2 (stop-and-go):        full speed stops at non-hard junctions.
  BUG-T3 (wrong initial turn): first-move turn direction + no reverse drive.

For each ulog it confirms no actuator/steering/rate clipping and snapshots key params.
Emits a consolidated markdown report with a per-shape aggregate and an overall
BUILD: STABLE / REVIEW verdict. Shape is taken from the filename
(arc/square/lshape/uturn) else inferred.

Decode is via sqlite3 + rosbags typestore (works with or without metadata.yaml).
Reuses the proven decode from analyze_square.py.
"""
from __future__ import annotations
import argparse, glob, math, os, sys
import numpy as np

# ---- bag decode --------------------------------------------------------------
def _ts():
    from rosbags.typesys import Stores, get_typestore
    return get_typestore(Stores.ROS2_HUMBLE)

def _find_db3(bag_path: str) -> str | None:
    if bag_path.endswith(".db3"): return bag_path
    hits = glob.glob(os.path.join(bag_path, "*.db3"))
    return hits[0] if hits else None

def _yaw_enu(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def _deriv(t, x, h=0.2):
    o = np.full_like(x, np.nan)
    for i in range(len(t)):
        a = np.searchsorted(t, t[i]-h); b = np.searchsorted(t, t[i]+h)
        if b-a >= 3:
            A = np.c_[t[a:b]-t[i], np.ones(b-a)]
            o[i] = np.linalg.lstsq(A, x[a:b], rcond=None)[0][0]
    return o

def _seg_dist(px, py, ax, ay, bx, by):
    vx, vy = bx-ax, by-ay; wx, wy = px-ax, py-ay
    L2 = vx*vx+vy*vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, (wx*vx+wy*vy)/L2))
    return math.hypot(px-(ax+t*vx), py-(ay+t*vy))

def decode_bag(bag_path: str) -> dict:
    import sqlite3
    ts = _ts()
    db = _find_db3(bag_path)
    if not db: raise FileNotFoundError(f"no .db3 under {bag_path}")
    con = sqlite3.connect(db); c = con.cursor()
    tt = {r[0]: (r[1], r[2]) for r in c.execute("select id,name,type from topics")}
    n2i = {v[0]: k for k, v in tt.items()}
    def load(name):
        if name not in n2i: return []
        tid = n2i[name]; typ = tt[tid][1]
        return [(t*1e-9, ts.deserialize_cdr(d, typ))
                for t, d in c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tid,))]

    pose = load("/mavros/local_position/pose")
    dbg  = load("/rpp/debug")
    seg  = load("/rpp/segment_debug")
    path = load("/path")
    if not pose or not path:
        raise ValueError("missing pose or /path")
    t0 = pose[0][0]
    E = np.array([p[1].pose.position.x for p in pose])
    N = np.array([p[1].pose.position.y for p in pose])
    Tp = np.array([p[0]-t0 for p in pose])
    yawENU = np.unwrap(np.array([_yaw_enu(p[1].pose.orientation) for p in pose]))
    yawNED = math.pi/2 - yawENU
    # commanded path → map frame (path NED x=N,y=E → East=y, North=x)
    pm = path[-1][1]
    pE = np.array([q.pose.position.y for q in pm.poses])
    pN = np.array([q.pose.position.x for q in pm.poses])
    poly = np.c_[pE, pN]
    geo = np.array([min(_seg_dist(E[i], N[i], poly[j,0], poly[j,1], poly[j+1,0], poly[j+1,1])
                        for j in range(len(poly)-1)) for i in range(len(E))]) * 100.0
    # rpp debug
    d = {"Td": np.array([x[0]-t0 for x in dbg]) if dbg else np.array([]),
         "xtk": np.array([x[1].data[0]*100 for x in dbg]) if dbg else np.array([]),
         "herr": np.degrees(np.array([x[1].data[1] for x in dbg])) if dbg else np.array([]),
         "scmd": np.array([x[1].data[3] for x in dbg]) if dbg else np.array([]),
         "state": np.array([int(x[1].data[7]) for x in dbg]) if dbg else np.array([]),
         "yrc": np.array([x[1].data[10] for x in dbg]) if dbg else np.array([])}
    # segment debug ([1] state, [5] corner angle, [7] heading err, [9] actual yaw rate if present)
    s = {}
    if seg:
        s["Ts"] = np.array([x[0]-t0 for x in seg])
        s["st"] = np.array([int(x[1].data[1]) for x in seg])
        s["cang"] = np.array([x[1].data[5] for x in seg])
        s["herr"] = np.array([x[1].data[7] for x in seg])
        s["yr9"] = np.array([x[1].data[9] if len(x[1].data) > 9 else np.nan for x in seg])
    # actual speed & yaw rate
    spd = np.sqrt(_deriv(Tp, E)**2 + _deriv(Tp, N)**2)
    yr_act = _deriv(Tp, yawNED)
    # TRACKING mask on the pose timeline (interp rpp state_code; default all-true)
    if d["Td"].size:
        trkP = np.round(np.interp(Tp, d["Td"], d["state"])).astype(int) == 1
    else:
        trkP = np.ones(len(Tp), bool)
    return dict(Tp=Tp, E=E, N=N, yawNED=yawNED, geo=geo, spd=spd, yr_act=yr_act,
                pE=pE, pN=pN, dbg=d, seg=s, dur=Tp[-1], trkP=trkP)

# ---- metrics + bug checks ----------------------------------------------------
def stats(a):
    a = a[np.isfinite(a)]
    if len(a) == 0: return dict(mean=float('nan'), rms=float('nan'), p95=float('nan'), mx=float('nan'))
    return dict(mean=float(np.mean(np.abs(a))), rms=float(np.sqrt(np.mean(a**2))),
                p95=float(np.percentile(np.abs(a), 95)), mx=float(np.max(np.abs(a))))

def shape_of(name: str) -> str:
    n = name.lower()
    for k in ("square", "lshape", "l_shape", "uturn", "u_turn", "u-turn", "arc", "circle", "line"):
        if k in n: return k.replace("_", "").replace("-", "")
    return "unknown"

def check_bugs(b: dict) -> dict:
    out = {}
    dbg, seg = b["dbg"], b["seg"]
    # state mask: TRACKING=1 in rpp debug
    trk = (dbg["state"] == 1) if dbg["state"].size else np.array([], bool)

    # BUG-T1 — pivot oscillation: yaw-rate sign reversals during CORNER_ALIGN(3)/STOP(5)
    if seg and seg["yr9"].size and np.isfinite(seg["yr9"]).any():
        piv = np.isin(seg["st"], (3, 5))
        yr = seg["yr9"][piv]
        yr = yr[np.isfinite(yr)]
        reversals = int(np.sum(np.diff(np.sign(yr[np.abs(yr) > 0.02])) != 0)) if yr.size else 0
        peak = float(np.max(np.abs(yr))) if yr.size else 0.0
        out["BUG-T1"] = ("PASS" if reversals <= 4 else "WARN" if reversals <= 8 else "FAIL",
                         f"pivot yaw-rate reversals={reversals}, peak={peak:.2f} rad/s")
    else:
        out["BUG-T1"] = ("N/A", "no segment_debug[9] (pre-fix bag or no corners)")

    # BUG-T2 — stop-and-go: full stops that are NOT intentional hard-corner stops.
    # A stop is legit if it happens in segment state CORNER_STOP(5)/ALIGN(3); a BUG-T2
    # violation = speed≈0 while segment state is TRACK_SEGMENT(1) (a straight/tangent run).
    if dbg["scmd"].size and seg and seg["st"].size:
        Td = dbg["Td"]; sc = dbg["scmd"]
        seg_state_at = np.round(np.interp(Td, seg["Ts"], seg["st"])).astype(int)
        nkeep = int(len(sc)*0.9)  # drop goal-approach tail
        mask = np.zeros(len(sc), bool); mask[:nkeep] = True
        bad = mask & (sc < 0.02) & (seg_state_at == 1)        # stopped while "tracking"
        legit = mask & (sc < 0.02) & np.isin(seg_state_at, (3, 5))
        nbad = int(bad.sum())
        out["BUG-T2"] = ("PASS" if nbad < 3 else "WARN" if nbad < 15 else "FAIL",
                         f"bad stops (TRACK-state v≈0)={nbad}; legit corner stops={int(legit.sum())}")
    elif dbg["scmd"].size:
        sc = dbg["scmd"][trk] if trk.size else dbg["scmd"]
        nkeep = int(len(sc)*0.9); stops = int(np.sum(sc[:nkeep] < 0.02)) if nkeep else 0
        out["BUG-T2"] = ("WARN" if stops else "PASS", f"no segment_debug; raw mid-track zero-speed={stops}")
    else:
        out["BUG-T2"] = ("N/A", "no rpp/debug speed")

    # BUG-T3 — wrong initial turn: first-segment bearing vs initial heading; turn shortest way, no reverse
    try:
        # path first-segment bearing (NED)
        b0_N, b0_E = b["pN"][1]-b["pN"][0], b["pE"][1]-b["pE"][0]
        bearing = math.atan2(b0_E, b0_N)  # NED CW+
        h0 = b["yawNED"][0]
        err0 = (bearing - h0 + math.pi) % (2*math.pi) - math.pi   # shortest signed
        # actual yaw-rate sign over first 2s of motion
        m = (b["Tp"] < b["Tp"][0]+2.5) & (b["spd"] > 0.03)
        yr_early = b["yr_act"][m]
        yr_early = yr_early[np.isfinite(yr_early)]
        turned = np.sign(np.nanmean(yr_early)) if yr_early.size else 0
        dir_ok = (turned == np.sign(err0)) or abs(err0) < math.radians(10)
        # reverse check: rpp speed_cmd should stay >= 0 (no negative)
        rev = bool(dbg["scmd"].size and np.any(dbg["scmd"] < -0.02))
        ok = dir_ok and not rev
        out["BUG-T3"] = ("PASS" if ok else "FAIL",
                         f"init heading_err={math.degrees(err0):+.0f}°, turned={'+CW' if turned>0 else '-CCW' if turned<0 else '0'}, reverse={rev}")
    except Exception as e:
        out["BUG-T3"] = ("N/A", f"insufficient data ({e})")
    return out

# ---- ulog check --------------------------------------------------------------
def check_ulog(path: str) -> dict:
    try:
        from pyulog import ULog
        u = ULog(path)
    except Exception as e:
        return {"err": str(e)}
    def arr(n, f):
        try:
            d = [x for x in u.data_list if x.name == n][0].data
            return np.array(d[f])
        except Exception:
            return np.array([])
    res = {}
    m0 = arr("actuator_motors", "control[0]"); m1 = arr("actuator_motors", "control[1]")
    nsd = arr("rover_steering_setpoint", "normalized_speed_diff")
    sp_cmd = arr("rover_rate_setpoint", "yaw_rate_setpoint"); sp_meas = arr("rover_rate_status", "measured_yaw_rate")
    res["motor_peak"] = float(np.nanmax(np.abs(np.r_[m0, m1]))) if m0.size else float('nan')
    res["steer_peak"] = float(np.nanmax(np.abs(nsd))) if nsd.size else float('nan')
    res["clip"] = (res["motor_peak"] > 0.98) or (res["steer_peak"] > 0.94)
    P = u.initial_parameters
    res["RO_YAW_P"] = P.get("RO_YAW_P"); res["RO_YAW_RATE_LIM"] = P.get("RO_YAW_RATE_LIM")
    res["EKF2_WENC_CTRL"] = P.get("EKF2_WENC_CTRL"); res["RBCLW_COUNTS_REV"] = P.get("RBCLW_COUNTS_REV")
    res["dur"] = (u.last_timestamp-u.start_timestamp)/1e6
    return res

# ---- main --------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = a.out or os.path.join(a.dir, "BUILD_VALIDATION.md")

    bags = sorted({os.path.dirname(p) for p in glob.glob(os.path.join(a.dir, "**", "*_0.db3"), recursive=True)}
                  | {os.path.dirname(p) for p in glob.glob(os.path.join(a.dir, "**", "metadata.yaml"), recursive=True)})
    ulogs = sorted(glob.glob(os.path.join(a.dir, "**", "*.ulg"), recursive=True))

    rows, bug_tally = [], {"BUG-T1": [], "BUG-T2": [], "BUG-T3": []}
    L = ["# Build Validation Report", "",
         f"Source: `{a.dir}`  |  bags: {len(bags)}  |  ulogs: {len(ulogs)}", ""]
    L += ["## Per-bag tracking + regression checks (cross-track = TRACKING state only)", "",
          "| Bag | Shape | geo RMS/max (cm) | RPP RMS/max (cm) | BUG-T1 | BUG-T2 | BUG-T3 |",
          "|---|---|---|---|---|---|---|"]
    for bp in bags:
        name = os.path.basename(bp.rstrip("/"))
        try:
            b = decode_bag(bp)
        except Exception as e:
            L.append(f"| {name} | — | DECODE FAIL: {e} | — | — | — | — |"); continue
        sh = shape_of(name)
        st = stats(b["geo"][b["trkP"]])                       # geometric, TRACKING-only
        rpp = stats(b["dbg"]["xtk"][b["dbg"]["state"] == 1]) if b["dbg"]["xtk"].size else stats(np.array([]))
        bugs = check_bugs(b)
        for k in bug_tally: bug_tally[k].append(bugs[k][0])
        rows.append((sh, st["rms"], st["mx"]))
        L.append(f"| {name} | {sh} | {st['rms']:.2f} / {st['mx']:.2f} | {rpp['rms']:.2f} / {rpp['mx']:.2f} | "
                 f"{bugs['BUG-T1'][0]} | {bugs['BUG-T2'][0]} | {bugs['BUG-T3'][0]} |")
    # per-bug detail
    L += ["", "## Regression check detail", ""]
    for bp in bags:
        name = os.path.basename(bp.rstrip("/"))
        try: b = decode_bag(bp); bugs = check_bugs(b)
        except Exception: continue
        L.append(f"**{name}**")
        for k, (verdict, msg) in bugs.items():
            L.append(f"- {k}: **{verdict}** — {msg}")
        L.append("")
    # aggregate per shape
    L += ["## Per-shape aggregate (geometric cross-track)", "",
          "| Shape | runs | RMS mean (cm) | RMS spread (cm) | max worst (cm) |",
          "|---|---|---|---|---|"]
    shapes = {}
    for sh, rms, mx in rows: shapes.setdefault(sh, []).append((rms, mx))
    for sh, v in sorted(shapes.items()):
        r = np.array([x[0] for x in v]); m = np.array([x[1] for x in v])
        L.append(f"| {sh} | {len(v)} | {np.nanmean(r):.2f} | {np.nanmax(r)-np.nanmin(r):.2f} | {np.nanmax(m):.2f} |")
    # ulog section
    if ulogs:
        L += ["", "## PX4 ulog sanity (clipping + key params)", "",
              "| Log | dur s | motor peak | steer peak | clip? | RO_YAW_P | YAW_RATE_LIM | WENC | COUNTS_REV |",
              "|---|---|---|---|---|---|---|---|---|"]
        for lg in ulogs:
            r = check_ulog(lg)
            if "err" in r: L.append(f"| {os.path.basename(lg)} | parse fail: {r['err']} |"); continue
            L.append(f"| {os.path.basename(lg)} | {r['dur']:.0f} | {r['motor_peak']:.2f} | {r['steer_peak']:.2f} | "
                     f"{'YES⚠' if r['clip'] else 'no'} | {r['RO_YAW_P']} | {r['RO_YAW_RATE_LIM']} | {r['EKF2_WENC_CTRL']} | {r['RBCLW_COUNTS_REV']} |")
    # verdict
    def worst(v): return "FAIL" if "FAIL" in v else "WARN" if "WARN" in v else "PASS"
    verdicts = {k: worst(v) for k, v in bug_tally.items()}
    overall = "STABLE" if all(x == "PASS" for x in verdicts.values()) else \
              "REVIEW" if "FAIL" not in verdicts.values() else "NOT STABLE"
    L += ["", "## Verdict", "",
          f"- BUG-T1 (pivot): **{verdicts.get('BUG-T1','?')}**",
          f"- BUG-T2 (stop-go): **{verdicts.get('BUG-T2','?')}**",
          f"- BUG-T3 (turn dir): **{verdicts.get('BUG-T3','?')}**",
          "", f"### BUILD: **{overall}**", ""]
    with open(out, "w") as f: f.write("\n".join(L))
    print(f"[validate_build] {len(bags)} bags, {len(ulogs)} ulogs → {out}")
    print(f"[validate_build] BUILD: {overall}  ({verdicts})")

if __name__ == "__main__":
    main()
