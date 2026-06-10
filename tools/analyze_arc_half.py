#!/usr/bin/env python3
"""Standalone arc_half bag analyzer — reads sqlite3 db3 directly, decodes CDR
for /rpp/debug (Float32MultiArray) without ROS. Emits xtrack/heading/goal
stats + full RPP param snapshot ([11..38]).
"""
import sqlite3, struct, sys, math
from pathlib import Path

# ---- debug array field map (matches src/rpp_controller_node.py docstring) ----
PARAM_NAMES = {
    11: "max_linear_vel", 12: "min_linear_vel", 13: "min_lookahead_dist",
    14: "max_lookahead_dist", 15: "lookahead_time", 16: "a_lat_max",
    17: "regulated_linear_scaling_min_speed", 18: "xy_goal_tolerance",
    19: "min_goal_travel_m", 20: "approach_velocity_scaling_dist",
    21: "min_approach_linear_velocity", 22: "p4_zero_vel_threshold",
    23: "pose_max_age_s", 24: "ekf_jump_threshold_m", 25: "require_rtk_fix",
    26: "preview_curvature_n", 27: "xtrack_lookahead_gain",
    28: "path_resample_spacing_m", 29: "corner_smooth_radius_m",
    30: "corner_smooth_arc_pts", 31: "use_imu_extrapolation",
    32: "imu_max_extrap_age_s", 33: "use_feedforward_yaw_rate",
    34: "yaw_rate_feedback_gain", 35: "max_yaw_rate_body",
    36: "max_linear_accel", 37: "max_linear_decel", 38: "mission_speed",
}
STATE = {-1:"STALE",0:"IDLE",1:"TRACKING",2:"APPROACH",3:"DONE",4:"RTK_WAIT",5:"JUMP_SKIP"}


class CDR:
    """Minimal little-endian CDR reader (alignment relative to body start)."""
    def __init__(self, buf):
        # 4-byte encapsulation header; assume CDR_LE
        self.b = buf[4:]
        self.p = 0
    def align(self, n):
        m = self.p % n
        if m: self.p += (n - m)
    def u32(self):
        self.align(4); v = struct.unpack_from("<I", self.b, self.p)[0]; self.p += 4; return v
    def f32(self):
        self.align(4); v = struct.unpack_from("<f", self.b, self.p)[0]; self.p += 4; return v
    def string(self):
        n = self.u32()
        s = self.b[self.p:self.p+n]; self.p += n
        return s


def decode_f32multiarray(buf):
    """std_msgs/Float32MultiArray -> list[float]."""
    c = CDR(buf)
    ndim = c.u32()
    for _ in range(ndim):
        c.string()   # label
        c.u32()      # size
        c.u32()      # stride
    c.u32()          # data_offset
    n = c.u32()
    return [c.f32() for _ in range(n)]


def analyze(bagdir):
    bagdir = Path(bagdir)
    db = next(bagdir.glob("*.db3"))
    con = sqlite3.connect(str(db))
    tid = con.execute("SELECT id FROM topics WHERE name='/rpp/debug'").fetchone()
    if not tid:
        print(f"  no /rpp/debug in {bagdir.name}"); return None
    rows = con.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid[0],)
    ).fetchall()
    con.close()

    samples, t0 = [], None
    params = None
    for ts, data in rows:
        arr = decode_f32multiarray(data)
        if len(arr) < 8:
            continue
        if t0 is None: t0 = ts
        samples.append(((ts - t0) / 1e9, arr))
        if params is None and len(arr) >= 39:
            params = arr  # snapshot identical every cycle; grab first full one

    if not samples:
        print(f"  no decodable samples in {bagdir.name}"); return None

    n = len(samples)
    dur = samples[-1][0]
    # metrics only while actually tracking/approach (state 1 or 2) to exclude idle ramp
    track = [a for (_, a) in samples if int(a[7]) in (1, 2)]
    use = track if track else [a for (_, a) in samples]

    xs = [a[0] for a in use]
    he = [a[1] for a in use]
    goals = [a[5] for a in use]
    speeds = [a[3] for a in use]

    ax = [abs(x) for x in xs]
    x_max = max(ax); x_mean = sum(ax)/len(ax)
    x_rms = math.sqrt(sum(x*x for x in xs)/len(xs))
    x_med = sorted(ax)[len(ax)//2]
    he_deg = [math.degrees(h) for h in he]
    ahe = [abs(h) for h in he_deg]
    he_max = max(ahe); he_mean = sum(ahe)/len(ahe)
    he_rms = math.degrees(math.sqrt(sum(h*h for h in he)/len(he)))
    # goal error = min dist_to_goal reached (closest approach to final WP)
    goal_min = min(goals)
    # final dist_to_goal among DONE samples if any, else last sample
    done = [a[5] for (_, a) in samples if int(a[7]) == 3]
    goal_final = done[0] if done else samples[-1][1][5]
    v_max = max(speeds) if speeds else float("nan")

    states_seen = sorted({STATE.get(int(a[7]), str(int(a[7]))) for (_, a) in samples})

    return {
        "name": bagdir.name, "n": n, "dur": dur, "n_track": len(use),
        "x_max": x_max, "x_mean": x_mean, "x_rms": x_rms, "x_med": x_med,
        "he_max": he_max, "he_mean": he_mean, "he_rms": he_rms,
        "goal_min": goal_min, "goal_final": goal_final, "v_max": v_max,
        "states": states_seen, "params": params,
    }


def main():
    bags = sys.argv[1:]
    results = [analyze(b) for b in bags]
    results = [r for r in results if r]

    for r in results:
        print("="*68)
        print(f"BAG: {r['name']}")
        print(f"  samples={r['n']}  duration={r['dur']:.1f}s  tracking_samples={r['n_track']}")
        print(f"  states seen: {', '.join(r['states'])}")
        print(f"  --- CROSS-TRACK ERROR (m, +=right of path) ---")
        print(f"      max={r['x_max']*100:.2f}cm  mean={r['x_mean']*100:.2f}cm  "
              f"median={r['x_med']*100:.2f}cm  rms={r['x_rms']*100:.2f}cm")
        print(f"  --- HEADING ERROR (deg) ---")
        print(f"      max={r['he_max']:.2f}  mean={r['he_mean']:.2f}  rms={r['he_rms']:.2f}")
        print(f"  --- GOAL / WAYPOINT ERROR (m) ---")
        print(f"      closest_approach={r['goal_min']*100:.2f}cm  final_dist_to_goal={r['goal_final']*100:.2f}cm")
        print(f"  --- SPEED ---")
        print(f"      max_speed_cmd={r['v_max']:.3f} m/s")

    # params (from first bag that has a full snapshot)
    psrc = next((r for r in results if r["params"]), None)
    if psrc:
        print("="*68)
        print(f"RPP PARAM SNAPSHOT (from {psrc['name']}, debug[11..38])")
        for i in range(11, 39):
            print(f"      [{i:2d}] {PARAM_NAMES[i]:38s} = {psrc['params'][i]:.4g}")

    # side-by-side
    if len(results) > 1:
        print("="*68)
        print("COMPARISON")
        hdr = f"{'metric':28s}" + "".join(f"{r['name'][-15:]:>18s}" for r in results)
        print(hdr)
        def row(label, key, scale=1.0, unit=""):
            print(f"{label:28s}" + "".join(f"{r[key]*scale:>16.2f}{unit:>2s}" for r in results))
        row("xtrack max (cm)", "x_max", 100)
        row("xtrack mean (cm)", "x_mean", 100)
        row("xtrack median (cm)", "x_med", 100)
        row("xtrack rms (cm)", "x_rms", 100)
        row("heading err rms (deg)", "he_rms", 1)
        row("heading err max (deg)", "he_max", 1)
        row("goal closest (cm)", "goal_min", 100)
        row("goal final (cm)", "goal_final", 100)


if __name__ == "__main__":
    main()
