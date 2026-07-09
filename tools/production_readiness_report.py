#!/usr/bin/env python3
"""Production Readiness Report for 3WD Marking Rover mission bags.

Analyzes every bag bundle under a directory and produces a comprehensive markdown
report covering: entry behaviour, pivot behaviour, speed/steering oscillation,
cross-track error, stop accuracy, spray system, and an overall production verdict
for drag-mission drawing readiness.

Usage:
    python3 tools/production_readiness_report.py <bags_dir> [--out report.md]

Topic field references (verified from live bags):
  /rpp/debug (47 floats):
    [0]  cte_m (signed cross-track error)
    [1]  heading_err_rad (relative to current segment — large during pivots)
    [2]  lookahead_dist_m
    [3]  speed_cmd_m_s
    [6]  kappa (curvature 1/m)
    [7]  state_code (0=IDLE, 1=TRACKING/DECEL, 2=ACCEL, 3=CORNER_ALIGN, 5=CORNER_STOP)
    [10] yaw_rate_cmd_rad_s
    [11] max_yaw_rate (param)
    [15] max_lin_vel (param)
    [16] a_lat_max (param)
    [17] corner_smooth_radius (param)
    [35] max_yaw_rate_body (param)

  /rpp/segment_debug (10 floats):
    [0]  run_idx
    [1]  state (0=IDLE,1=RUN/ACCEL,2=DECEL,3=CORNER_ALIGN,4=DONE,5=CORNER_STOP)
    [2]  seg_idx
    [3]  seg_len_m (nan during pivot)
    [4]  dist_to_end_m
    [5]  corner_angle_deg (NOT radians — e.g. 90.0 for square corner)
    [6]  seg_heading_rad
    [7]  heading_err_rad (delta from seg heading)
    [8]  unused
    [9]  actual_yaw_rate_rad_s

  /rpp/stop_debug (20 floats):
    [0]  phase (0=INACTIVE,1=HOLDING,2=STOP_CERTIFIED,3=ALIGNING,
              4=ALIGN_CERTIFIED,5=FINAL_CERTIFIED,6=RELEASED,7=BLOCKED)
    [1]  reason (1=INTRA_RUN_CORNER,2=RUN_BOUNDARY,
               3=RUNTIME_ENTRY_TO_MARK,4=FINAL_ENDPOINT)
    [4]  target_n (NED)
    [5]  target_e (NED)
    [6]  position_error_m
    [7]  measured_speed_m_s
    [8]  measured_yaw_rate_rad_s
    [10] dwell_time_s

Frame convention:
  /mavros/local_position/pose is ENU (x=East, y=North)
  /path and mission waypoints are NED (x=North, y=East)
  Path NED(n,e) -> ENU(e,n) for geometric comparison with pose
"""
from __future__ import annotations
import argparse, glob, json, math, os, sys
from dataclasses import dataclass, field
import numpy as np

# ── Helpers ───────────────────────────────────────────────────────────────────

def _ts():
    from rosbags.typesys import Stores, get_typestore
    return get_typestore(Stores.ROS2_HUMBLE)

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

def _rms(a):
    a = a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a**2))) if a.size else float('nan')

def _pct(a, p):
    a = a[np.isfinite(a)]
    return float(np.percentile(np.abs(a), p)) if a.size else float('nan')

def _median(a):
    a = a[np.isfinite(a)]
    return float(np.median(np.abs(a))) if a.size else float('nan')

def shape_of(name: str) -> str:
    n = name.lower()
    if "square" in n: return "square"
    if "circle" in n: return "circle"
    if "line" in n: return "line"
    if "l_2m" in n or "lshape" in n or "l_shape" in n: return "lshape"
    if "triangle" in n: return "triangle"
    if "uturn" in n or "u_turn" in n: return "uturn"
    if "arc" in n: return "arc"
    return "unknown"


# ── Bag decode ────────────────────────────────────────────────────────────────

def load_staged_json(bundle_path: str) -> dict | None:
    """Load mission/staged.json for waypoints and anchor."""
    p = os.path.join(bundle_path, "mission", "staged.json")
    if os.path.isfile(p):
        with open(p) as f:
            return json.load(f)
    # Fallback: search recursively
    hits = glob.glob(os.path.join(bundle_path, "**", "staged.json"), recursive=True)
    if hits:
        with open(hits[0]) as f:
            return json.load(f)
    return None


def reconstruct_path_from_topic(path_msgs: list) -> tuple[np.ndarray, np.ndarray]:
    """Collect all path poses from /path messages (absolute NED) and convert to ENU.

    /path publishes latched messages; the first usually has the full first-run path.
    We collect unique points from all messages to get the complete mission geometry.
    NED(n,e) -> ENU(e,n)
    """
    all_n, all_e = [], []
    for _, pm in path_msgs:
        for q in pm.poses:
            all_n.append(q.pose.position.x)  # NED x = North
            all_e.append(q.pose.position.y)  # NED y = East
    if not all_n:
        return np.array([]), np.array([])
    # Deduplicate (path topic may re-send same points)
    pts = list(zip(all_n, all_e))
    seen = set()
    uniq_n, uniq_e = [], []
    for n, e in pts:
        key = (round(n, 4), round(e, 4))
        if key not in seen:
            seen.add(key)
            uniq_n.append(n)
            uniq_e.append(e)
    # Convert to ENU: East=e, North=n
    return np.array(uniq_e), np.array(uniq_n)


def decode_bag(bundle_path: str) -> dict:
    """Decode all relevant topics from a mission bundle."""
    import sqlite3
    ts = _ts()

    # Find db3
    db3s = glob.glob(os.path.join(bundle_path, "**", "*.db3"), recursive=True)
    if not db3s:
        raise FileNotFoundError(f"no .db3 under {bundle_path}")
    db = db3s[0]

    con = sqlite3.connect(db); c = con.cursor()
    tt = {r[0]: (r[1], r[2]) for r in c.execute("select id,name,type from topics")}
    n2i = {v[0]: k for k, v in tt.items()}

    def load(name, typename):
        if name not in n2i: return []
        tid = n2i[name]
        return [(t*1e-9, ts.deserialize_cdr(d, typename))
                for t, d in c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tid,))]

    pose = load("/mavros/local_position/pose", "geometry_msgs/msg/PoseStamped")
    dbg  = load("/rpp/debug", "std_msgs/msg/Float32MultiArray")
    seg  = load("/rpp/segment_debug", "std_msgs/msg/Float32MultiArray")
    stop = load("/rpp/stop_debug", "std_msgs/msg/Float32MultiArray")
    path_msgs = load("/path", "nav_msgs/msg/Path")
    spray_active = load("/spray/active", "std_msgs/msg/Bool")

    if not pose:
        raise ValueError("missing /mavros/local_position/pose")
    con.close()

    t0 = pose[0][0]

    # Pose (ENU)
    Tp = np.array([p[0]-t0 for p in pose])
    E = np.array([p[1].pose.position.x for p in pose])
    N = np.array([p[1].pose.position.y for p in pose])
    yawENU = np.unwrap(np.array([_yaw_enu(p[1].pose.orientation) for p in pose]))
    yawNED = math.pi/2 - yawENU

    # Velocity: compute actual speed from position derivatives (MAVROS velocity
    # topic may report zero due to EKF config; position-derived is reliable)
    Tv = Tp.copy()
    act_spd = np.sqrt(_deriv(Tp, E)**2 + _deriv(Tp, N)**2)

    # Path reconstruction from /path topic (absolute NED → ENU)
    staged = load_staged_json(bundle_path)
    pE, pN = reconstruct_path_from_topic(path_msgs)

    # Geometric cross-track (ENU)
    if len(pE) > 1:
        poly = np.c_[pE, pN]
        geo = np.array([min(_seg_dist(E[i], N[i], poly[j,0], poly[j,1], poly[j+1,0], poly[j+1,1])
                            for j in range(len(poly)-1)) for i in range(len(E))]) * 100.0
    else:
        geo = np.full(len(E), np.nan)

    # RPP debug
    d = {}
    if dbg:
        d["Td"] = np.array([x[0]-t0 for x in dbg])
        d["cte"] = np.array([x[1].data[0]*100 for x in dbg])  # cm
        d["herr"] = np.array([x[1].data[1] for x in dbg])     # rad
        d["scmd"] = np.array([x[1].data[3] for x in dbg])     # m/s
        d["yrcmd"] = np.array([x[1].data[10] for x in dbg])   # rad/s
        d["state"] = np.array([int(x[1].data[7]) for x in dbg])
        d["kappa"] = np.array([x[1].data[6] for x in dbg])
        last = dbg[-1][1].data
        d["max_v_param"] = float(last[15])
        d["a_lat_param"] = float(last[16])
        d["crad_param"] = float(last[17])
        d["yr_body_param"] = float(last[35]) if len(last) > 35 else 0.0
        d["max_yr_param"] = float(last[11])
    else:
        for k in "Td cte herr scmd yrcmd state kappa".split(): d[k] = np.array([])
        d.update(max_v_param=0, a_lat_param=0, crad_param=0, yr_body_param=0, max_yr_param=0)

    # Segment debug
    s = {}
    if seg:
        s["Ts"] = np.array([x[0]-t0 for x in seg])
        s["st"] = np.array([int(x[1].data[1]) for x in seg])
        s["cang"] = np.array([x[1].data[5] for x in seg])       # degrees
        s["seg_hdg"] = np.array([x[1].data[6] for x in seg])    # rad
        s["herr"] = np.array([x[1].data[7] for x in seg])       # rad
        s["yr_act"] = np.array([x[1].data[9] for x in seg])     # rad/s
    else:
        for k in "Ts st cang seg_hdg herr yr_act".split(): s[k] = np.array([])

    # Stop debug
    sd = {}
    if stop:
        sd["Ts"] = np.array([x[0]-t0 for x in stop])
        sd["phase"] = np.array([int(x[1].data[0]) for x in stop])
        sd["reason"] = np.array([int(x[1].data[1]) for x in stop])
        sd["tn"] = np.array([x[1].data[4] for x in stop])
        sd["te"] = np.array([x[1].data[5] for x in stop])
        sd["perr"] = np.array([x[1].data[6]*100 for x in stop])
        sd["spd"] = np.array([x[1].data[7] for x in stop])
        sd["yr"] = np.array([x[1].data[8] for x in stop])
        sd["dwell"] = np.array([x[1].data[10] for x in stop])
    else:
        for k in "Ts phase reason tn te perr spd yr dwell".split(): sd[k] = np.array([])

    # Spray active timeline
    sa = {}
    if spray_active:
        sa["Ts"] = np.array([x[0]-t0 for x in spray_active])
        sa["val"] = np.array([1 if x[1].data else 0 for x in spray_active])
    else:
        sa["Ts"] = sa["val"] = np.array([])

    # Define cruising mask on rpp/debug timeline:
    # Use ACTUAL speed > 0.15 m/s (from position derivative) AND RPP state==1
    # This reliably identifies genuine driving periods, excluding stop/pivot/transition
    if d["Td"].size and act_spd.size:
        act_spd_at_dbg = np.interp(d["Td"], Tv, act_spd)
        cruise_mask = (act_spd_at_dbg > 0.15) & (d["state"] == 1)
    else:
        cruise_mask = np.array([], dtype=bool)

    return dict(Tp=Tp, E=E, N=N, yawNED=yawNED, geo=geo,
                Tv=Tv, act_spd=act_spd,
                pE=pE, pN=pN, staged=staged,
                dbg=d, seg=s, stop=sd, spray_act=sa,
                cruise_mask=cruise_mask,
                dur=float(Tp[-1]) if len(Tp) else 0.0)


# ── Analysis: Entry Behaviour ─────────────────────────────────────────────────

@dataclass
class EntryAnalysis:
    time_to_first_motion_s: float
    initial_heading_err_deg: float
    turn_direction_correct: bool
    entry_xtrack_cm: float
    entry_converge_s: float


def analyze_entry(b: dict) -> EntryAnalysis:
    Tp, E, N, yawNED = b["Tp"], b["E"], b["N"], b["yawNED"]
    # Actual speed via derivative for entry phase
    spd_drv = np.sqrt(_deriv(Tp, E)**2 + _deriv(Tp, N)**2)

    # Time to first motion (speed > 0.05 m/s sustained for 0.5s)
    first_motion = float('nan')
    for i in range(len(Tp)):
        if spd_drv[i] > 0.05:
            j = np.searchsorted(Tp, Tp[i]+0.5)
            if np.nanmean(spd_drv[i:min(j,len(Tp))]) > 0.05:
                first_motion = Tp[i]
                break

    # Initial heading error: path first-segment bearing vs initial yaw
    # Note: for missions with runtime-entry stop, the rover may start facing any direction
    # The entry pivot handles alignment, so we report the error but the turn_direction
    # check should look at the actual pivot that happens during entry
    init_hdg_err = float('nan')
    turn_ok = True
    pE, pN = b["pE"], b["pN"]
    if len(pE) > 1:
        # Path in ENU: bearing from North (NED convention: atan2(E, N))
        dE = pE[1] - pE[0]
        dN = pN[1] - pN[0]
        bearing = math.atan2(dE, dN)  # NED bearing
        h0 = yawNED[0]
        err0 = (bearing - h0 + math.pi) % (2*math.pi) - math.pi
        init_hdg_err = math.degrees(err0)

        # Turn direction: check yaw rate during first sustained motion
        # (skip entry stop period - look for when speed first exceeds 0.15)
        act_drv = np.sqrt(_deriv(Tp, E)**2 + _deriv(Tp, N)**2)
        first_move_end = None
        for i in range(len(Tp)):
            if act_drv[i] > 0.15:
                j = np.searchsorted(Tp, Tp[i]+3.0)
                first_move_end = min(j, len(Tp))
                break
        if first_move_end:
            i_start = np.searchsorted(Tp, Tp[np.nanargmax(act_drv > 0.15)])
            yr_hdg = _deriv(Tp, yawNED)
            yr_m = yr_hdg[i_start:first_move_end]
            yr_m = yr_m[np.isfinite(yr_m)]
            if yr_m.size:
                turn_sign = float(np.sign(np.nanmean(yr_m)))
                turn_ok = (turn_sign == np.sign(err0)) or abs(err0) < math.radians(10)

    # Entry xtrack (RPP cte in first 5s of cruise)
    dbg = b["dbg"]
    entry_xtrack = float('nan')
    converge = float('nan')
    if dbg["cte"].size and dbg["Td"].size:
        # First 5s of tracking
        t5 = dbg["Td"][0] + 5.0
        end_idx = np.searchsorted(dbg["Td"], t5)
        early_cte = dbg["cte"][:end_idx]
        early_cte = early_cte[np.isfinite(early_cte)]
        if early_cte.size:
            entry_xtrack = _rms(early_cte)

        # Convergence: first sustained < 5cm for 1s window
        cte = dbg["cte"]; Td = dbg["Td"]
        for i in range(len(cte)):
            if np.isfinite(cte[i]) and abs(cte[i]) < 5.0:
                j = min(np.searchsorted(Td, Td[i]+1.0), len(cte))
                window = cte[i:j]
                window = window[np.isfinite(window)]
                if window.size and np.all(np.abs(window) < 5.0):
                    converge = Td[i]
                    break

    return EntryAnalysis(
        time_to_first_motion_s=first_motion,
        initial_heading_err_deg=init_hdg_err,
        turn_direction_correct=turn_ok,
        entry_xtrack_cm=entry_xtrack,
        entry_converge_s=converge,
    )


# ── Analysis: Pivot Behaviour ─────────────────────────────────────────────────

@dataclass
class PivotEvent:
    t_start: float
    duration_s: float
    corner_angle_deg: float
    peak_yaw_rate_rad_s: float
    significant_reversals: int   # sign changes > 0.10 rad/s
    settle_yaw_rate_rad_s: float


@dataclass
class PivotAnalysis:
    total_pivots: int
    events: list = field(default_factory=list)
    avg_duration_s: float = 0.0
    avg_peak_yr: float = 0.0
    total_sig_reversals: int = 0
    worst_reversals: int = 0


def analyze_pivots(b: dict) -> PivotAnalysis:
    """Pivot analysis from segment_debug states 3 (ALIGN) + 5 (STOP).

    Groups contiguous ALIGN+STOP periods into single pivot events.
    """
    seg = b["seg"]
    if not seg["Ts"].size:
        return PivotAnalysis(total_pivots=0)

    st = seg["st"]; Ts = seg["Ts"]
    pivot_mask = np.isin(st, (3, 5))

    events = []
    i = 0
    while i < len(Ts):
        if pivot_mask[i]:
            j = i
            while j < len(Ts) and pivot_mask[j]:
                j += 1
            yr = seg["yr_act"][i:j]
            yr_f = yr[np.isfinite(yr)]
            cang = seg["cang"][i:j]
            cang_f = cang[np.isfinite(cang)]

            if yr_f.size > 2:
                # Significant reversals: sign changes in yaw rate > 0.10 threshold
                sig = yr_f[np.abs(yr_f) > 0.10]
                reversals = int(np.sum(np.diff(np.sign(sig)) != 0)) if sig.size > 1 else 0
                peak = float(np.max(np.abs(yr_f)))
                settle = float(abs(yr_f[-1]))
                corner_deg = float(np.nanmean(np.abs(cang_f))) if cang_f.size else 0.0

                events.append(PivotEvent(
                    t_start=float(Ts[i]),
                    duration_s=float(Ts[j-1]-Ts[i]),
                    corner_angle_deg=corner_deg,
                    peak_yaw_rate_rad_s=peak,
                    significant_reversals=reversals,
                    settle_yaw_rate_rad_s=settle,
                ))
            i = j
        else:
            i += 1

    if not events:
        return PivotAnalysis(total_pivots=0)

    total_sig = sum(e.significant_reversals for e in events)
    worst = max((e.significant_reversals for e in events), default=0)

    return PivotAnalysis(
        total_pivots=len(events),
        events=events,
        avg_duration_s=float(np.mean([e.duration_s for e in events])),
        avg_peak_yr=float(np.mean([e.peak_yaw_rate_rad_s for e in events])),
        total_sig_reversals=total_sig,
        worst_reversals=worst,
    )


# ── Analysis: Speed & Steering ────────────────────────────────────────────────

@dataclass
class SpeedSteerAnalysis:
    target_speed: float
    cmd_speed_cruise_mean: float
    actual_speed_cruise_mean: float
    actual_speed_cruise_max: float
    speed_tracking_err_pct: float
    speed_cv_pct: float
    speed_overshoot_events: int
    yaw_rate_cmd_std: float
    yaw_rate_reversals_cruise: int
    heading_err_p90_deg: float
    heading_err_rms_deg: float
    heading_err_median_deg: float
    heading_err_max_deg: float
    steer_saturation_pct: float


def analyze_speed_steering(b: dict) -> SpeedSteerAnalysis:
    """Speed and steering analysis, filtered to CRUISE phase only (actual speed > 0.15)."""
    dbg = b["dbg"]
    staged = b.get("staged") or {}
    target = staged.get("marking_speed_mps", 0.35)

    cruise = b["cruise_mask"]

    # Speed during cruise — use position-derived actual speed interpolated to debug timeline
    if cruise.any():
        cmd_vals = dbg["scmd"][cruise]
        act_interp = np.interp(dbg["Td"][cruise], b["Tv"], b["act_spd"])
        cmd_mean = float(np.nanmean(cmd_vals))
        act_mean = float(np.nanmean(act_interp))
        act_max = float(np.nanmax(act_interp))
        act_std = float(np.nanstd(act_interp))
        track_err = abs(act_mean - target) / target * 100 if target > 0 else 0
        cv = act_std / act_mean * 100 if act_mean > 0 else 0
        overshoot_events = int(np.sum(act_interp > target * 1.3))
    else:
        cmd_mean = act_mean = act_max = act_std = 0.0
        track_err = cv = 0.0
        overshoot_events = 0

    # Heading error during cruise only — use P90 (robust to transition spikes)
    # RMS is inflated by brief segment-switch transients (single-sample spikes to ~180°
    # when RPP swaps target segment at corner pass). P90 captures the steady-state quality.
    if cruise.any():
        herr_cruise = np.degrees(dbg["herr"][cruise])
        herr_cruise = herr_cruise[np.isfinite(herr_cruise)]
        herr_cruise = ((herr_cruise + 180) % 360) - 180  # wrap to [-180, 180]
        herr_rms = _rms(herr_cruise)
        herr_p90 = float(np.percentile(np.abs(herr_cruise), 90))
        herr_max = float(np.nanmax(np.abs(herr_cruise))) if herr_cruise.size else 0
        # Also compute median for reference
        herr_median = float(np.median(np.abs(herr_cruise))) if herr_cruise.size else 0
    else:
        herr_rms = herr_p90 = herr_max = herr_median = float('nan')

    # Yaw rate command during cruise
    if cruise.any():
        yr_cruise = dbg["yrcmd"][cruise]
        yr_std = float(np.nanstd(yr_cruise))
        sig_yr = yr_cruise[np.abs(yr_cruise) > 0.02]
        yr_rev = int(np.sum(np.diff(np.sign(sig_yr)) != 0)) if sig_yr.size > 1 else 0

        yr_body = dbg["yr_body_param"]
        sat = float(np.mean(np.abs(yr_cruise) >= yr_body * 0.95)) * 100 if yr_body > 0 else 0
    else:
        yr_std = 0.0; yr_rev = 0; sat = 0.0

    return SpeedSteerAnalysis(
        target_speed=target,
        cmd_speed_cruise_mean=cmd_mean,
        actual_speed_cruise_mean=act_mean,
        actual_speed_cruise_max=act_max,
        speed_tracking_err_pct=track_err,
        speed_cv_pct=cv,
        speed_overshoot_events=overshoot_events,
        yaw_rate_cmd_std=yr_std,
        yaw_rate_reversals_cruise=yr_rev,
        heading_err_p90_deg=herr_p90,
        heading_err_rms_deg=herr_rms,
        heading_err_median_deg=herr_median,
        heading_err_max_deg=herr_max,
        steer_saturation_pct=sat,
    )


# ── Analysis: Cross-Track ─────────────────────────────────────────────────────

@dataclass
class CrossTrackAnalysis:
    rpp_rms_cm: float
    rpp_mean_cm: float
    rpp_p95_cm: float
    rpp_max_cm: float
    geo_rms_cm: float
    geo_mean_cm: float
    geo_p95_cm: float
    geo_max_cm: float
    pct_above_5cm: float
    pct_above_10cm: float


def analyze_xtrack(b: dict) -> CrossTrackAnalysis:
    """Cross-track error, both RPP internal (cruise only) and geometric."""
    dbg = b["dbg"]
    cruise = b["cruise_mask"]

    # RPP internal CTE during cruise only (excludes pivot/stop false errors)
    if cruise.any():
        cte_cruise = dbg["cte"][cruise]
        cte_cruise = cte_cruise[np.isfinite(cte_cruise)]
        rpp_rms = _rms(cte_cruise)
        rpp_mean = float(np.mean(np.abs(cte_cruise)))
        rpp_p95 = _pct(cte_cruise, 95)
        rpp_max = float(np.nanmax(np.abs(cte_cruise)))
        p5 = float(np.mean(np.abs(cte_cruise) > 5.0)) * 100
        p10 = float(np.mean(np.abs(cte_cruise) > 10.0)) * 100
    else:
        rpp_rms = rpp_mean = rpp_p95 = rpp_max = p5 = p10 = float('nan')

    # Geometric cross-track: only during cruise (interp cruise mask to pose timeline)
    if dbg["Td"].size and b["Tp"].size:
        cruise_on_pose = np.interp(b["Tp"], dbg["Td"], cruise.astype(float)) > 0.5
    else:
        cruise_on_pose = np.ones(len(b["Tp"]), bool)

    geo_cruise = b["geo"][cruise_on_pose]
    geo_cruise = geo_cruise[np.isfinite(geo_cruise)]
    if geo_cruise.size:
        geo_rms = _rms(geo_cruise)
        geo_mean = float(np.mean(np.abs(geo_cruise)))
        geo_p95 = _pct(geo_cruise, 95)
        geo_max = float(np.nanmax(np.abs(geo_cruise)))
    else:
        geo_rms = geo_mean = geo_p95 = geo_max = float('nan')

    return CrossTrackAnalysis(
        rpp_rms_cm=rpp_rms, rpp_mean_cm=rpp_mean, rpp_p95_cm=rpp_p95, rpp_max_cm=rpp_max,
        geo_rms_cm=geo_rms, geo_mean_cm=geo_mean, geo_p95_cm=geo_p95, geo_max_cm=geo_max,
        pct_above_5cm=p5, pct_above_10cm=p10,
    )


# ── Analysis: Stop Accuracy ───────────────────────────────────────────────────

STOP_REASON_MAP = {0: "UNKNOWN", 1: "CORNER", 2: "BOUNDARY", 3: "ENTRY", 4: "ENDPOINT"}

@dataclass
class StopEvent:
    t_start: float
    reason: str
    target_n: float
    target_e: float
    certified_error_cm: float
    settle_duration_s: float

@dataclass
class StopAnalysis:
    total_stops: int
    events: list = field(default_factory=list)
    rms_error_cm: float = 0.0
    max_error_cm: float = 0.0
    avg_settle_s: float = 0.0


def analyze_stops(b: dict) -> StopAnalysis:
    sd = b["stop"]
    if not sd["Ts"].size:
        return StopAnalysis(total_stops=0)

    phase, reason = sd["phase"], sd["reason"]
    tn, te, perr = sd["tn"], sd["te"], sd["perr"]

    # Group by unique (reason, target_n, target_e)
    by_target = {}
    for i in range(len(sd["Ts"])):
        key = (int(reason[i]), round(float(tn[i]), 3), round(float(te[i]), 3))
        by_target.setdefault(key, []).append(i)

    events = []
    for (r, tn_v, te_v), indices in by_target.items():
        cert_idx = None; hold_idx = None
        for idx in indices:
            if hold_idx is None and int(phase[idx]) in (1, 3):
                hold_idx = idx
            if cert_idx is None and int(phase[idx]) in (2, 4, 5):
                cert_idx = idx; break
        if cert_idx is None:
            continue
        err = float(perr[cert_idx])
        t_s = float(sd["Ts"][hold_idx]) if hold_idx is not None else float(sd["Ts"][indices[0]])
        t_c = float(sd["Ts"][cert_idx])
        events.append(StopEvent(
            t_start=t_s, reason=STOP_REASON_MAP.get(r, f"?({r})"),
            target_n=tn_v, target_e=te_v,
            certified_error_cm=err, settle_duration_s=t_c - t_s,
        ))

    if not events:
        return StopAnalysis(total_stops=0)

    errs = np.array([e.certified_error_cm for e in events])
    settles = np.array([e.settle_duration_s for e in events])
    return StopAnalysis(
        total_stops=len(events), events=events,
        rms_error_cm=_rms(errs), max_error_cm=float(np.max(errs)),
        avg_settle_s=float(np.mean(settles)),
    )


# ── Analysis: Spray ───────────────────────────────────────────────────────────

@dataclass
class SprayAnalysis:
    active_duration_s: float
    mission_duration_s: float
    coverage_pct: float
    transitions: int
    has_data: bool


def analyze_spray(b: dict) -> SprayAnalysis:
    sa = b["spray_act"]
    dur = b["dur"]
    if not sa["Ts"].size:
        return SprayAnalysis(0, dur, 0, 0, False)
    active = sa["val"] == 1
    spray_dur = float(sa["Ts"][active][-1] - sa["Ts"][active][0]) if active.any() and active.sum() > 1 else 0
    transitions = int(np.sum(np.diff(sa["val"]) != 0))
    coverage = spray_dur / dur * 100 if dur > 0 else 0
    return SprayAnalysis(spray_dur, dur, coverage, transitions, True)


# ── Verdict ───────────────────────────────────────────────────────────────────

@dataclass
class VerdictCheck:
    name: str
    status: str  # PASS / WARN / FAIL
    threshold: str
    actual: str
    detail: str


def compute_verdict(entry, pivots, ss, stops, xtrack, spray) -> list[VerdictCheck]:
    checks = []

    # 1. Cross-track RMS (geometric, cruise)
    ct = xtrack.geo_rms_cm
    checks.append(VerdictCheck(
        "Cross-track RMS (geometric)", "PASS" if ct <= 5 else "WARN" if ct <= 10 else "FAIL",
        "<= 5 cm", f"{ct:.2f} cm",
        f"RPP RMS {xtrack.rpp_rms_cm:.2f} cm, geo max {xtrack.geo_max_cm:.1f} cm"))

    # 2. Cross-track max
    mx = xtrack.geo_max_cm
    checks.append(VerdictCheck(
        "Cross-track max excursion", "PASS" if mx <= 15 else "WARN" if mx <= 30 else "FAIL",
        "<= 15 cm", f"{mx:.1f} cm",
        f"{xtrack.pct_above_5cm:.1f}% > 5cm, {xtrack.pct_above_10cm:.1f}% > 10cm"))

    # 3. Pivot oscillation
    checks.append(VerdictCheck(
        "Pivot oscillation", "PASS" if pivots.total_sig_reversals <= 3 else "WARN" if pivots.total_sig_reversals <= 8 else "FAIL",
        "<= 3 sig reversals", f"{pivots.total_sig_reversals} sig (worst single: {pivots.worst_reversals})",
        f"{pivots.total_pivots} pivots, avg {pivots.avg_duration_s:.1f}s"))

    # 4. Entry turn direction
    checks.append(VerdictCheck(
        "Entry turn direction", "PASS" if entry.turn_direction_correct else "FAIL",
        "correct shortest-way", "OK" if entry.turn_direction_correct else "WRONG",
        f"init err {entry.initial_heading_err_deg:+.0f}°, first motion {entry.time_to_first_motion_s:.1f}s"))

    # 5. Entry convergence
    cv = entry.entry_converge_s
    checks.append(VerdictCheck(
        "Entry convergence", "PASS" if np.isfinite(cv) and cv <= 5 else "WARN" if np.isfinite(cv) and cv <= 10 else "FAIL",
        "<= 5 s to < 5cm", f"{cv:.1f} s" if np.isfinite(cv) else "N/A",
        f"entry RMS {entry.entry_xtrack_cm:.1f} cm"))

    # 6. Speed tracking (cruise)
    ste = ss.speed_tracking_err_pct
    checks.append(VerdictCheck(
        "Speed tracking (cruise)", "PASS" if ste <= 15 else "WARN" if ste <= 30 else "FAIL",
        "<= 15% from target", f"{ste:.1f}%",
        f"target {ss.target_speed:.2f} m/s, actual {ss.actual_speed_cruise_mean:.2f} m/s, CV {ss.speed_cv_pct:.1f}%"))

    # 7. Heading error (cruise) — use P90 (robust to segment-switch transients)
    herr = ss.heading_err_p90_deg
    checks.append(VerdictCheck(
        "Heading error P90 (cruise)", "PASS" if herr <= 10 else "WARN" if herr <= 20 else "FAIL",
        "<= 10 deg", f"{herr:.1f} deg",
        f"median {ss.heading_err_median_deg:.1f} deg, RMS {ss.heading_err_rms_deg:.1f} deg, max {ss.heading_err_max_deg:.0f} deg"))

    # 8. Stop accuracy
    if stops.total_stops > 0:
        se = stops.rms_error_cm
        checks.append(VerdictCheck(
            "Stop position accuracy", "PASS" if se <= 3 else "WARN" if se <= 5 else "FAIL",
            "<= 3 cm RMS", f"{se:.2f} cm",
            f"max {stops.max_error_cm:.1f} cm, {stops.total_stops} stops, avg settle {stops.avg_settle_s:.1f}s"))

    # 9. Steering oscillation
    yr_rev = ss.yaw_rate_reversals_cruise
    checks.append(VerdictCheck(
        "Steering oscillation (cruise)", "PASS" if yr_rev <= 10 else "WARN" if yr_rev <= 25 else "FAIL",
        "<= 10 reversals", f"{yr_rev}",
        f"yr_cmd std {ss.yaw_rate_cmd_std:.3f} rad/s, sat {ss.steer_saturation_pct:.1f}%"))

    return checks


# ── Main ──────────────────────────────────────────────────────────────────────

def find_bundles(root: str) -> list[tuple[str, str]]:
    """Find mission bundle directories containing a .db3 and manifest.json.
    Skips nested duplicates (e.g. 'Untitled') and incomplete bags."""
    db3s = sorted(glob.glob(os.path.join(root, "**", "*.db3"), recursive=True))
    bundles = {}
    for db in db3s:
        d = os.path.dirname(db)
        for _ in range(5):
            if os.path.isfile(os.path.join(d, "manifest.json")):
                break
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
        name = os.path.basename(d.rstrip("/"))
        # Skip nested/artifact directories
        if name in ("Untitled", "bag", "rosbag"):
            continue
        if name not in bundles:
            bundles[name] = d
    return sorted(bundles.items())


def main():
    ap = argparse.ArgumentParser(description="Production readiness report for mission bags")
    ap.add_argument("dir", help="Directory containing mission bag bundles")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    root = args.dir.rstrip("/")
    out_path = args.out or os.path.join(root, "PRODUCTION_READINESS_REPORT.md")
    bundles = find_bundles(root)

    L = ["# Production Readiness Report", "",
         f"**Source:** `{root}`  ",
         f"**Bags found:** {len(bundles)}  ",
         f"**Generated:** 2026-07-08  ",
         "**Purpose:** Production drag-mission drawing readiness assessment  ", ""]

    all_results = []
    all_checks = []

    for bname, bpath in bundles:
        sh = shape_of(bname)
        L += [f"---", f"## {bname}", f"**Shape:** {sh}  ", ""]

        try:
            b = decode_bag(bpath)
        except Exception as e:
            L += [f"DECODE ERROR: {e}", ""]
            continue

        entry = analyze_entry(b)
        pivots = analyze_pivots(b)
        ss = analyze_speed_steering(b)
        stops = analyze_stops(b)
        xtrack = analyze_xtrack(b)
        spray = analyze_spray(b)
        checks = compute_verdict(entry, pivots, ss, stops, xtrack, spray)
        all_checks.append((bname, sh, checks))
        all_results.append((bname, sh, b, entry, pivots, ss, stops, xtrack, spray))

        # Verdict table
        L += ["### Verdict Summary", "",
              "| Check | Status | Threshold | Actual | Detail |",
              "|---|---|---|---|---|"]
        for chk in checks:
            L.append(f"| {chk.name} | **{chk.status}** | {chk.threshold} | {chk.actual} | {chk.detail} |")
        L.append("")

        # Entry
        L += ["### Entry Behaviour", "",
              "| Metric | Value |", "|---|---|",
              f"| Time to first motion | {entry.time_to_first_motion_s:.1f} s |",
              f"| Initial heading error | {entry.initial_heading_err_deg:+.1f}° |" if np.isfinite(entry.initial_heading_err_deg) else "| Initial heading error | N/A |",
              f"| Turn direction correct | {'YES' if entry.turn_direction_correct else 'NO'} |",
              f"| Entry xtrack RMS (first 5s) | {entry.entry_xtrack_cm:.1f} cm |",
              f"| Convergence time (< 5cm) | {entry.entry_converge_s:.1f} s |" if np.isfinite(entry.entry_converge_s) else "| Convergence time | N/A |",
              ""]

        # Pivot
        L += ["### Pivot Behaviour", ""]
        if pivots.total_pivots == 0:
            L += ["No pivot events detected (straight-line mission).", ""]
        else:
            L += [f"Total pivots: **{pivots.total_pivots}**, avg duration: **{pivots.avg_duration_s:.1f}s**, "
                  f"total sig reversals: **{pivots.total_sig_reversals}**, worst single event: **{pivots.worst_reversals}**", "",
                  "| # | t_start (s) | Duration (s) | Corner° | Peak YR (rad/s) | Sig Reversals | Settle YR |",
                  "|---|---|---|---|---|---|---|"]
            for i, ev in enumerate(pivots.events):
                L.append(f"| {i+1} | {ev.t_start:.1f} | {ev.duration_s:.1f} | {ev.corner_angle_deg:.0f} | "
                         f"{ev.peak_yaw_rate_rad_s:.2f} | {ev.significant_reversals} | {ev.settle_yaw_rate_rad_s:.3f} |")
            L.append("")

        # Speed & Steering
        L += ["### Speed & Steering (cruise phase only)", "",
              "| Metric | Value |", "|---|---|",
              f"| Target speed | {ss.target_speed:.3f} m/s |",
              f"| Cmd speed (cruise mean) | {ss.cmd_speed_cruise_mean:.3f} m/s |",
              f"| Actual speed (cruise mean) | {ss.actual_speed_cruise_mean:.3f} m/s |",
              f"| Actual speed (cruise max) | {ss.actual_speed_cruise_max:.3f} m/s |",
              f"| Speed tracking error | {ss.speed_tracking_err_pct:.1f}% |",
              f"| Speed CV (cruise) | {ss.speed_cv_pct:.1f}% |",
              f"| Speed overshoot events (>130% target) | {ss.speed_overshoot_events} |",
              f"| Yaw rate cmd std | {ss.yaw_rate_cmd_std:.4f} rad/s |",
              f"| Yaw rate reversals (cruise) | {ss.yaw_rate_reversals_cruise} |",
              f"| Heading error P90 | {ss.heading_err_p90_deg:.1f}° |",
              f"| Heading error median | {ss.heading_err_median_deg:.1f}° |",
              f"| Heading error RMS | {ss.heading_err_rms_deg:.1f}° |",
              f"| Heading error max | {ss.heading_err_max_deg:.1f}° |",
              f"| Yaw rate saturation | {ss.steer_saturation_pct:.1f}% |",
              ""]

        # Cross-track
        L += ["### Cross-Track Error", "",
              "| Source | RMS | Mean | P95 | Max |", "|---|---|---|---|---|",
              f"| RPP internal (cruise) | {xtrack.rpp_rms_cm:.2f} cm | {xtrack.rpp_mean_cm:.2f} cm | {xtrack.rpp_p95_cm:.2f} cm | {xtrack.rpp_max_cm:.1f} cm |",
              f"| Geometric (pose-to-path) | {xtrack.geo_rms_cm:.2f} cm | {xtrack.geo_mean_cm:.2f} cm | {xtrack.geo_p95_cm:.2f} cm | {xtrack.geo_max_cm:.1f} cm |",
              "",
              f"RPP samples > 5cm: **{xtrack.pct_above_5cm:.1f}%**, > 10cm: **{xtrack.pct_above_10cm:.1f}%**", ""]

        # Stops
        if stops.total_stops > 0:
            L += ["### Stop Accuracy", "",
                  f"Total stops: **{stops.total_stops}**, RMS certified error: **{stops.rms_error_cm:.2f} cm**, "
                  f"max: **{stops.max_error_cm:.1f} cm**, avg settle: **{stops.avg_settle_s:.1f}s**", "",
                  "| # | t (s) | Reason | Target (N,E) | Cert Err (cm) | Settle (s) |",
                  "|---|---|---|---|---|---|"]
            for i, ev in enumerate(stops.events[:20]):
                L.append(f"| {i+1} | {ev.t_start:.1f} | {ev.reason} | ({ev.target_n:.2f}, {ev.target_e:.2f}) | "
                         f"{ev.certified_error_cm:.2f} | {ev.settle_duration_s:.1f} |")
            L.append("")

        # Spray
        if spray.has_data:
            L += ["### Spray System", "",
                  "| Metric | Value |", "|---|---|",
                  f"| Spray active duration | {spray.active_duration_s:.1f} s |",
                  f"| Mission duration | {spray.mission_duration_s:.1f} s |",
                  f"| Coverage | {spray.coverage_pct:.1f}% |",
                  f"| On/off transitions | {spray.transitions} |", ""]

        # Trajectory
        dist = float(np.sum(np.sqrt(np.diff(b['E'])**2 + np.diff(b['N'])**2)))
        L += ["### Trajectory Summary", "",
              "| Metric | Value |", "|---|---|",
              f"| Duration | {b['dur']:.1f} s ({b['dur']/60:.1f} min) |",
              f"| Total distance | {dist:.2f} m |"]
        if b['dur'] > 0:
            L.append(f"| Avg speed | {dist/b['dur']:.3f} m/s |")
        L += [f"| East span | {b['E'].max()-b['E'].min():.2f} m |",
               f"| North span | {b['N'].max()-b['N'].min():.2f} m |", ""]

    # Cross-bag aggregate
    L += ["---", "## Cross-Bag Aggregate", "",
          "### Per-shape summary", "",
          "| Shape | Runs | Geo RMS avg (cm) | Geo RMS worst (cm) | RPP RMS avg (cm) | Pivot sig rev (total) | Stop RMS (cm) |",
          "|---|---|---|---|---|---|---|"]
    shape_agg = {}
    for item in all_results:
        shape_agg.setdefault(item[1], []).append(item)
    for sh in sorted(shape_agg.keys()):
        runs = shape_agg[sh]
        geo_vals = [r[7].geo_rms_cm for r in runs if np.isfinite(r[7].geo_rms_cm)]
        rpp_vals = [r[7].rpp_rms_cm for r in runs if np.isfinite(r[7].rpp_rms_cm)]
        all_rev = sum(r[4].total_sig_reversals for r in runs)
        stop_vals = [r[6].rms_error_cm for r in runs if r[6].total_stops > 0]
        geo_avg = float(np.mean(geo_vals)) if geo_vals else float('nan')
        geo_worst = float(np.max(geo_vals)) if geo_vals else float('nan')
        rpp_avg = float(np.mean(rpp_vals)) if rpp_vals else float('nan')
        stop_avg = float(np.mean(stop_vals)) if stop_vals else float('nan')
        L.append(f"| {sh} | {len(runs)} | {geo_avg:.2f} | {geo_worst:.2f} | {rpp_avg:.2f} | {all_rev} | {stop_avg:.2f} |")
    L.append("")

    # Verdict
    total_p = sum(1 for _, _, checks in all_checks for c in checks if c.status == "PASS")
    total_w = sum(1 for _, _, checks in all_checks for c in checks if c.status == "WARN")
    total_f = sum(1 for _, _, checks in all_checks for c in checks if c.status == "FAIL")
    overall = "READY FOR PRODUCTION" if total_f == 0 and total_w <= 3 else \
              "READY WITH MINOR CONCERNS" if total_f == 0 else "NOT READY"

    L += ["## Overall Production Verdict", "",
          f"**Checks:** {total_p} PASS, {total_w} WARN, {total_f} FAIL  ",
          f"### VERDICT: {overall}", ""]

    if total_f > 0:
        L += ["### Failed Checks", ""]
        for name, sh, checks in all_checks:
            for c in checks:
                if c.status == "FAIL":
                    L.append(f"- **{name}** ({sh}): {c.name} -- {c.actual} (threshold {c.threshold}) -- {c.detail}")
        L.append("")

    if total_w > 0:
        L += ["### Warnings", ""]
        for name, sh, checks in all_checks:
            for c in checks:
                if c.status == "WARN":
                    L.append(f"- **{name}** ({sh}): {c.name} -- {c.actual} (threshold {c.threshold}) -- {c.detail}")
        L.append("")

    L += ["### Production Criteria Reference", "",
          "| Criterion | Target | Rationale |", "|---|---|---|",
          "| Cross-track RMS (geometric) | <= 5 cm | Paint line accuracy |",
          "| Cross-track max | <= 15 cm | No visible path deviation |",
          "| Pivot oscillation | <= 3 sig reversals | Clean corner execution |",
          "| Entry turn direction | Correct shortest-way | No wrong-way marking |",
          "| Entry convergence | <= 5 s | Fast path lock-on |",
          "| Speed tracking (cruise) | <= 15% from target | Consistent paint density |",
          "| Heading error P90 (cruise) | <= 10 deg | Line straightness |",
          "| Stop position RMS | <= 3 cm | Endpoint precision |",
          "| Steering oscillation | <= 10 reversals | Smooth drawing |", ""]

    report = "\n".join(L)
    with open(out_path, "w") as f:
        f.write(report)
    print(f"\n[production_report] {len(bundles)} bundles, {len(all_results)} decoded -> {out_path}")
    print(f"[production_report] VERDICT: {overall} ({total_p}P/{total_w}W/{total_f}F)")


if __name__ == "__main__":
    main()
