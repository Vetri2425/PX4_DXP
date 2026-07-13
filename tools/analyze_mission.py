#!/usr/bin/env python3
"""Post-mission behaviour analyser (G6) — reads ONE finalised bag/bundle and
emits `analysis.json` + a human-readable `report.txt`.

Generalises the D0/D3 single-event checkers to the whole mission: tracking,
stops, pivots, speed, spray, health/anomalies, as-run config, and an overall
PASS/FAIL verdict.

Design (per plan §4):
  * PURE READER — never touches the robot.
  * Dependency-light — a self-contained sqlite3 + CDR reader means it runs
    OFFLINE on any `.db3` bag, on the Mac GCS as well as the Jetson. If
    `rosbag2_py` is importable (Jetson) and the bag is not sqlite3 (e.g. mcap)
    it falls back to that.
  * Tolerant of missing topics — a missing topic is a WARN, never a crash.
  * No shape assumptions — works for line / L / square / arc / point.

Usage:
    python3 tools/analyze_mission.py <bundle-or-bag-dir> [-o OUTDIR] [--json-only]

<bundle-or-bag-dir> may be either a recorder bundle (dir with `bag/` +
`manifest.json`) or a bare rosbag2 directory (dir with `metadata.yaml`).
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sqlite3
import statistics
import struct
import sys
from datetime import datetime, timezone

# ── segment-state codes (rpp_controller_node SegmentStateCode) ───────────────
S_INACTIVE, S_TRACK, S_PRECORNER, S_ALIGN, S_DONE, S_STOP = 0, 1, 2, 3, 4, 5

# ── production thresholds (cm / deg / m·s⁻¹) ─────────────────────────────────
XTRACK_PROD_CM = 2.0        # production tracking class: RMS ≤ this
COAST_MAX_CM = 15.0         # coast-past a stop must stay under this
FINAL_STOP_MAX_CM = 12.0    # resting distance from a stop point
STOP_APPROACH_CM = 10.0     # "arrived" band around a stop point
DEPART_M = 0.30             # left the stop once beyond this (bounds coast to the halt)
SETTLE_TOL_DEG = 3.0        # |heading_err| at pivot release
OSC_BAND_DEG = 8.0          # a |heading_err| rise beyond this = one oscillation
FWD_EPS = -0.02             # commanded forward component while turning (reverse-flip gate)
TURNING_BAND_DEG = 5.0      # below this |heading_err| the turn is done (align-brake, not a flip)
SETPOINT_GAP_S = 0.5        # setpoint-stream gap beyond this risks OFFBOARD failsafe
POSE_STALE_MS = 300.0       # pose age beyond this is a staleness event
RTK_MIN_FIX = 6             # GPSRAW fix_type below this during drive = RTK degraded
EKF_JUMP_M = 0.5            # pose position jump between consecutive samples


# ═══════════════════════════════════════════════════════════════════════════
# CDR reader — classic little-endian XCDR1 with proper member alignment.
# ═══════════════════════════════════════════════════════════════════════════
class _CDR:
    __slots__ = ("d", "o")

    def __init__(self, data: bytes):
        self.d = data
        self.o = 4  # skip 4-byte encapsulation header (representation id + options)

    def _align(self, n: int) -> None:
        # alignment is relative to the start of the payload (after the 4-byte header)
        rel = self.o - 4
        self.o += (-rel) % n

    def u8(self):
        v = self.d[self.o]; self.o += 1; return v

    def i8(self):
        v = struct.unpack_from("<b", self.d, self.o)[0]; self.o += 1; return v

    def u16(self):
        self._align(2); v = struct.unpack_from("<H", self.d, self.o)[0]; self.o += 2; return v

    def u32(self):
        self._align(4); v = struct.unpack_from("<I", self.d, self.o)[0]; self.o += 4; return v

    def i32(self):
        self._align(4); v = struct.unpack_from("<i", self.d, self.o)[0]; self.o += 4; return v

    def f32(self):
        self._align(4); v = struct.unpack_from("<f", self.d, self.o)[0]; self.o += 4; return v

    def f64(self):
        self._align(8); v = struct.unpack_from("<d", self.d, self.o)[0]; self.o += 8; return v

    def boolean(self):
        return bool(self.u8())

    def string(self):
        n = self.u32()  # CDR string length includes the null terminator
        raw = self.d[self.o:self.o + n]; self.o += n
        return raw.split(b"\x00", 1)[0].decode("utf-8", "replace")

    def header(self):
        sec = self.i32(); nsec = self.u32(); fid = self.string()
        return (sec + nsec * 1e-9, fid)


# ── per-type parsers → normalized dicts ──────────────────────────────────────
def _p_pose(d):
    r = _CDR(d); r.header()
    px, py, pz = r.f64(), r.f64(), r.f64()
    ox, oy, oz, ow = r.f64(), r.f64(), r.f64(), r.f64()
    return {"x": px, "y": py, "z": pz, "qx": ox, "qy": oy, "qz": oz, "qw": ow}


def _p_twist(d):
    r = _CDR(d); r.header()
    lx, ly, lz = r.f64(), r.f64(), r.f64()
    return {"lx": lx, "ly": ly, "lz": lz}


def _p_vec3(d):
    r = _CDR(d); r.header()
    return {"x": r.f64(), "y": r.f64(), "z": r.f64()}


def _p_postarget(d):
    r = _CDR(d); r.header()
    cf = r.u16(); tm = r.u32()
    r.f64(); r.f64(); r.f64()                       # position (unused, vel setpoints)
    vx, vy, vz = r.f64(), r.f64(), r.f64()
    r.f64(); r.f64(); r.f64()                       # accel
    yaw = r.f64(); yaw_rate = r.f64()
    return {"coordinate_frame": cf, "type_mask": tm,
            "vx": vx, "vy": vy, "vz": vz, "yaw": yaw, "yaw_rate": yaw_rate}


def _p_state(d):
    # mavros_msgs/State on this stack has NO std header (validated on real bags).
    r = _CDR(d)
    connected = r.boolean(); armed = r.boolean(); guided = r.boolean()
    manual = r.boolean(); mode = r.string(); system_status = r.u8()
    return {"connected": connected, "armed": armed, "guided": guided,
            "manual_input": manual, "mode": mode, "system_status": system_status}


def _p_f32ma(d):
    r = _CDR(d)
    dim_len = r.u32()
    for _ in range(dim_len):
        r.string(); r.u32(); r.u32()
    r.u32()                                          # data_offset
    n = r.u32()
    return {"data": [r.f32() for _ in range(n)]}


def _p_f32(d):
    return {"data": _CDR(d).f32()}


def _p_bool(d):
    return {"data": _CDR(d).boolean()}


def _p_path(d):
    r = _CDR(d); r.header()
    n = r.u32()
    poses = []
    for _ in range(n):
        r.header()
        px, py, pz = r.f64(), r.f64(), r.f64()
        r.f64(); r.f64(); r.f64(); r.f64()           # orientation
        poses.append((px, py, pz))
    return {"poses": poses}


def _p_statustext(d):
    r = _CDR(d); r.header()
    sev = r.u8(); text = r.string()
    return {"severity": sev, "text": text}


def _p_gpsraw(d):
    # header + uint8 fix_type is all we need; deeper fields vary by version.
    r = _CDR(d); r.header()
    return {"fix_type": r.u8()}


PARSERS = {
    "geometry_msgs/msg/PoseStamped": _p_pose,
    "geometry_msgs/msg/TwistStamped": _p_twist,
    "geometry_msgs/msg/Vector3Stamped": _p_vec3,
    "mavros_msgs/msg/PositionTarget": _p_postarget,
    "mavros_msgs/msg/State": _p_state,
    "std_msgs/msg/Float32MultiArray": _p_f32ma,
    "std_msgs/msg/Float32": _p_f32,
    "nav_msgs/msg/Path": _p_path,
    "std_msgs/msg/Bool": _p_bool,
    "mavros_msgs/msg/StatusText": _p_statustext,
    "mavros_msgs/msg/GPSRAW": _p_gpsraw,
}


# ═══════════════════════════════════════════════════════════════════════════
# Bag reading
# ═══════════════════════════════════════════════════════════════════════════
def _find_bag_dir(root: str):
    """Return (bag_dir, manifest_or_None). Accepts a bundle or a bare bag dir."""
    manifest = None
    mpath = os.path.join(root, "manifest.json")
    if os.path.isfile(mpath):
        try:
            manifest = json.load(open(mpath))
        except Exception:
            manifest = None
    # a bundle keeps the bag under bag/
    if os.path.isdir(os.path.join(root, "bag")) and os.path.isfile(
        os.path.join(root, "bag", "metadata.yaml")
    ):
        return os.path.join(root, "bag"), manifest
    if os.path.isfile(os.path.join(root, "metadata.yaml")) or glob.glob(
        os.path.join(root, "*.db3")
    ):
        return root, manifest
    # search one level down for any db3
    for db in glob.glob(os.path.join(root, "**", "*.db3"), recursive=True):
        return os.path.dirname(db), manifest
    return None, manifest


def read_bag(bag_dir: str):
    """Yield (topic_name, msg_dict, t_seconds) in time order.

    Uses a direct sqlite3 + CDR reader (pure stdlib, runs anywhere). Falls back
    to rosbag2_py only when the storage is not sqlite3 (e.g. mcap).
    """
    dbs = sorted(glob.glob(os.path.join(bag_dir, "*.db3")))
    if dbs:
        yield from _read_sqlite(dbs)
        return
    yield from _read_rosbag2(bag_dir)


def _read_sqlite(dbs):
    for db in dbs:
        conn = sqlite3.connect(db)
        conn.row_factory = sqlite3.Row
        try:
            topics = {r["id"]: (r["name"], r["type"])
                      for r in conn.execute("SELECT id,name,type FROM topics")}
            cur = conn.execute(
                "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp ASC"
            )
            for row in cur:
                name, typ = topics.get(row["topic_id"], (None, None))
                parser = PARSERS.get(typ)
                if name is None or parser is None:
                    continue
                try:
                    msg = parser(bytes(row["data"]))
                except Exception:
                    continue
                yield name, msg, row["timestamp"] * 1e-9
        finally:
            conn.close()


def _read_rosbag2(bag_dir: str):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        sys.exit("ERROR: bag is not sqlite3 and rosbag2_py is unavailable — "
                 "run on the Jetson or point at a .db3 bag.")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    # rosbag2 path yields raw ros msgs; adapt the few fields we use.
    msgcls = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in msgcls:
            continue
        yield topic, _adapt_ros(deserialize_message(data, msgcls[topic])), t * 1e-9


def _adapt_ros(m):
    """Map a live ROS msg object into the same normalized dict the CDR path emits."""
    out = {}
    if hasattr(m, "pose") and hasattr(m.pose, "position"):
        p, o = m.pose.position, m.pose.orientation
        out.update({"x": p.x, "y": p.y, "z": p.z, "qx": o.x, "qy": o.y, "qz": o.z, "qw": o.w})
    if hasattr(m, "twist") and hasattr(m.twist, "linear"):
        lin = m.twist.linear
        out.update({"lx": lin.x, "ly": lin.y, "lz": lin.z})
    if hasattr(m, "vector"):
        out.update({"x": m.vector.x, "y": m.vector.y, "z": m.vector.z})
    if hasattr(m, "data"):
        out["data"] = list(m.data) if hasattr(m.data, "__len__") and not isinstance(m.data, str) else m.data
    if hasattr(m, "poses"):
        out["poses"] = [(ps.pose.position.x, ps.pose.position.y, ps.pose.position.z) for ps in m.poses]
    if hasattr(m, "mode"):
        out.update({"connected": getattr(m, "connected", None), "armed": getattr(m, "armed", None),
                    "guided": getattr(m, "guided", None), "mode": m.mode,
                    "system_status": getattr(m, "system_status", None)})
    if hasattr(m, "severity") and hasattr(m, "text"):
        out.update({"severity": m.severity, "text": m.text})
    if hasattr(m, "fix_type"):
        out["fix_type"] = m.fix_type
    if hasattr(m, "yaw_rate") and hasattr(m, "type_mask"):
        v = getattr(m, "velocity", None)
        if v is not None:
            out.update({"vx": v.x, "vy": v.y, "vz": v.z})
        out.update({"yaw": m.yaw, "yaw_rate": m.yaw_rate, "type_mask": m.type_mask,
                    "coordinate_frame": getattr(m, "coordinate_frame", None)})
    return out


# ═══════════════════════════════════════════════════════════════════════════
# math + stats helpers
# ═══════════════════════════════════════════════════════════════════════════
def _yaw_ned_from_quat(qx, qy, qz, qw):
    yaw_enu = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return math.pi / 2.0 - yaw_enu


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _pctl(sorted_vals, q):
    if not sorted_vals:
        return None
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    idx = q * (len(sorted_vals) - 1)
    lo = int(math.floor(idx)); hi = int(math.ceil(idx))
    if lo == hi:
        return sorted_vals[lo]
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (idx - lo)


def _stat_block(vals):
    """RMS / median / p95 / max / mean over |vals| in cm, plus signed bias."""
    if not vals:
        return None
    absv = sorted(abs(v) for v in vals)
    rms = math.sqrt(sum(v * v for v in vals) / len(vals))
    left = sum(1 for v in vals if v < 0)
    right = sum(1 for v in vals if v > 0)
    return {
        "n": len(vals),
        "rms_cm": round(rms * 100, 2),
        "median_cm": round(_pctl(absv, 0.5) * 100, 2),
        "p95_cm": round(_pctl(absv, 0.95) * 100, 2),
        "max_cm": round(absv[-1] * 100, 2),
        "mean_signed_cm": round((sum(vals) / len(vals)) * 100, 2),
        "left_frac": round(left / len(vals), 3),
        "right_frac": round(right / len(vals), 3),
    }


def _nearest(series, t):
    """series = sorted [(t, payload)]; return payload nearest time t (or None)."""
    if not series:
        return None
    lo, hi = 0, len(series) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    best = series[lo]
    if lo > 0 and abs(series[lo - 1][0] - t) < abs(best[0] - t):
        best = series[lo - 1]
    return best[1]


# ═══════════════════════════════════════════════════════════════════════════
# collection
# ═══════════════════════════════════════════════════════════════════════════
class Series:
    """All time-ordered signals we pull from the bag (bag-receive time as clock)."""
    def __init__(self):
        self.pose = []       # (t, n, e, yaw_ned)
        self.vel_meas = []   # (t, speed)
        self.vel_cmd = []    # (t, v_n, v_e)
        self.setpoint = []   # (t, type_mask)
        self.state = []      # (t, mode, armed)
        self.rpp = []        # (t, [data...])
        self.seg = []        # (t, state, heading_err)
        self.yaw_rate = []   # (t, val)
        self.path = None     # [(n, e)] NED
        self.spray_active = []    # (t, bool) desired MARK
        self.spray_desired = []   # (t, bool)
        self.spray_commanded = [] # (t, bool)
        self.spray_state = []     # (t, bool)
        self.statustext = []      # (t, severity, text)
        self.gps = []             # (t, fix_type)
        self.topics_seen = {}     # name -> count


def collect(bag_dir: str) -> Series:
    s = Series()
    for topic, m, t in read_bag(bag_dir):
        s.topics_seen[topic] = s.topics_seen.get(topic, 0) + 1
        if topic == "/mavros/local_position/pose":
            s.pose.append((t, m["y"], m["x"], _yaw_ned_from_quat(m["qx"], m["qy"], m["qz"], m["qw"])))
        elif topic == "/mavros/local_position/velocity_local":
            s.vel_meas.append((t, math.hypot(m["lx"], m["ly"])))
        elif topic == "/rpp/velocity_ned":
            s.vel_cmd.append((t, m["x"], m["y"]))
        elif topic == "/mavros/setpoint_raw/local":
            s.setpoint.append((t, m.get("type_mask")))
        elif topic == "/mavros/state":
            s.state.append((t, m["mode"], m.get("armed")))
        elif topic == "/rpp/debug":
            s.rpp.append((t, m["data"]))
        elif topic == "/rpp/segment_debug":
            dat = m["data"]
            if len(dat) >= 8:
                s.seg.append((t, int(round(dat[1])), float(dat[7])))
        elif topic == "/rpp/yaw_rate_body":
            s.yaw_rate.append((t, m["data"]))
        elif topic == "/path":
            if m["poses"]:
                s.path = [(p[0], p[1]) for p in m["poses"]]   # /path is NED direct (x=N, y=E)
        elif topic == "/spray/active":
            s.spray_active.append((t, m["data"]))
        elif topic == "/spray/desired":
            s.spray_desired.append((t, m["data"]))
        elif topic == "/spray/commanded":
            s.spray_commanded.append((t, m["data"]))
        elif topic == "/spray/state":
            s.spray_state.append((t, m["data"]))
        elif topic == "/mavros/statustext":
            s.statustext.append((t, m["severity"], m["text"]))
        elif topic == "/mavros/gpsstatus/gps1/raw":
            s.gps.append((t, m["fix_type"]))
    for lst in (s.pose, s.vel_meas, s.vel_cmd, s.setpoint, s.state, s.rpp, s.seg,
                s.yaw_rate, s.spray_active, s.spray_desired, s.spray_commanded,
                s.spray_state, s.statustext, s.gps):
        lst.sort(key=lambda r: r[0])
    return s


# ═══════════════════════════════════════════════════════════════════════════
# analysis sections
# ═══════════════════════════════════════════════════════════════════════════
def analyze_tracking(s: Series) -> dict:
    """Cross-track from /rpp/debug[0] (signed, m). Overall + marking-only."""
    if not s.rpp:
        return {"available": False, "reason": "no /rpp/debug"}
    xt_all = [d[0] for (_t, d) in s.rpp if d and len(d) > 0 and math.isfinite(d[0])]
    out = {"available": True, "overall": _stat_block(xt_all)}
    # marking-only: xtrack while spray desired is ON (if we have that signal)
    spray = s.spray_active or s.spray_desired
    if spray:
        xt_mark = []
        for (t, d) in s.rpp:
            if not d or not math.isfinite(d[0]):
                continue
            on = _nearest(spray, t)
            if on:
                xt_mark.append(d[0])
        out["marking_only"] = _stat_block(xt_mark) if xt_mark else None
    ov = out["overall"]
    out["verdict"] = ("PASS" if ov and ov["rms_cm"] <= XTRACK_PROD_CM else "FAIL") if ov else "WARN"
    return out


def _pivot_windows(seg):
    """Yield (i_stop, i_align, i_rel) index triples for each STOP→ALIGN→TRACK pivot."""
    wins = []
    i = 0
    n = len(seg)
    while i < n:
        if seg[i][1] == S_STOP:
            i_stop = i
            i_align = next((j for j in range(i_stop + 1, n) if seg[j][1] == S_ALIGN), None)
            if i_align is None:
                break
            i_rel = next((j for j in range(i_align + 1, n) if seg[j][1] == S_TRACK), n - 1)
            wins.append((i_stop, i_align, i_rel))
            i = i_rel + 1
        else:
            i += 1
    return wins


def analyze_pivots(s: Series) -> dict:
    if not s.seg:
        return {"available": False, "reason": "no /rpp/segment_debug"}
    wins = _pivot_windows(s.seg)
    if not wins:
        return {"available": True, "count": 0, "pivots": [],
                "note": "no CORNER_STOP→ALIGN→TRACK pivots in this mission"}
    pivots = []
    worst_settle = 0.0
    any_flip = False
    for k, (i_stop, i_align, i_rel) in enumerate(wins):
        t_stop, t_align, t_rel = s.seg[i_stop][0], s.seg[i_align][0], s.seg[i_rel][0]
        herr0 = math.degrees(abs(s.seg[i_align][2]))
        # turn magnitude — pose-yaw swept between stop and release
        yaws = [y for (t, _n, _e, y) in s.pose if t_stop <= t <= t_rel]
        swept = 0.0
        for a, b in zip(yaws, yaws[1:]):
            swept += _wrap(b - a)
        turn_deg = abs(math.degrees(swept))
        # reverse-flip — min commanded forward component while ACTIVELY turning
        min_fwd = math.inf
        vc_series = [(t, (vn, ve)) for (t, vn, ve) in s.vel_cmd]
        yw_series = [(pt, py) for (pt, _n, _e, py) in s.pose]
        for (t, st, he) in s.seg[i_align:i_rel + 1]:
            if abs(math.degrees(he)) <= TURNING_BAND_DEG:
                continue  # align-brake (intentional reverse) — not a flip
            vc = _nearest(vc_series, t)
            yw = _nearest(yw_series, t)
            if vc is None or yw is None:
                continue
            fwd = vc[0] * math.cos(yw) + vc[1] * math.sin(yw)
            min_fwd = min(min_fwd, fwd)
        flip = (min_fwd is not math.inf) and (min_fwd < FWD_EPS)
        any_flip = any_flip or flip
        # settle — |heading_err| at release
        settle_deg = math.degrees(abs(s.seg[i_rel][2] if s.seg[i_rel][1] == S_TRACK
                                       else s.seg[i_rel - 1][2]))
        worst_settle = max(worst_settle, settle_deg)
        # oscillation count during ALIGN
        herr = [abs(s.seg[j][2]) for j in range(i_align, i_rel + 1)]
        rises = 0
        if herr:
            run_min = herr[0]
            for h in herr[1:]:
                if h > run_min + math.radians(OSC_BAND_DEG):
                    rises += 1
                    run_min = h
                else:
                    run_min = min(run_min, h)
        # settle time
        settle_s = round(t_rel - t_align, 2)
        pivots.append({
            "index": k,
            "initial_heading_err_deg": round(herr0, 1),
            "turn_magnitude_deg": round(turn_deg, 1),
            "settle_time_s": settle_s,
            "settle_err_deg": round(settle_deg, 2),
            "min_fwd_component_m_s": (None if min_fwd is math.inf else round(min_fwd, 3)),
            "reverse_flip": flip,
            "oscillations": rises,
        })
    verdict = "PASS"
    if any_flip or worst_settle > SETTLE_TOL_DEG:
        verdict = "FAIL"
    return {"available": True, "count": len(pivots), "pivots": pivots,
            "any_reverse_flip": any_flip, "worst_settle_deg": round(worst_settle, 2),
            "verdict": verdict}


CORNER_MIN_DEG = 20.0   # interior direction change beyond this = a corner (a stop)


def _stop_vertices(verts) -> list[int]:
    """Indices of vertices where the rover is meant to halt: corners + endpoint.

    Excludes the start (idx 0) and pass-through densified vertices. Collapses
    near-duplicate consecutive points so densification doesn't spawn phantom
    corners.
    """
    n = len(verts)
    if n <= 1:
        return [n - 1] if n == 1 else []
    stops = []
    for i in range(1, n - 1):
        a, b, c = verts[i - 1], verts[i], verts[i + 1]
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        m1 = math.hypot(*v1); m2 = math.hypot(*v2)
        if m1 < 1e-3 or m2 < 1e-3:
            continue
        cosang = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (m1 * m2)))
        if math.degrees(math.acos(cosang)) >= CORNER_MIN_DEG:
            stops.append(i)
    stops.append(n - 1)  # endpoint is always a stop
    return stops


def analyze_stops(s: Series) -> dict:
    if s.path is None:
        return {"available": False, "reason": "no /path — stop points unknown"}
    if not s.pose:
        return {"available": False, "reason": "no pose samples"}
    rover = [(n, e) for (_t, n, e, _y) in s.pose]
    stops = []
    worst_coast = 0.0
    verts = s.path
    # A "stop" is a point where the rover is meant to halt: a corner (path
    # direction change) or the endpoint. Start + pass-through densified vertices
    # are NOT stops — the coast-past metric is meaningless there. This keeps the
    # analysis shape-agnostic (line has 1 stop, square has 4, etc.).
    times = [t for (t, _n, _e, _y) in s.pose]
    stop_idx = _stop_vertices(verts)
    last_idx = len(verts) - 1
    for idx in stop_idx:
        bn, be = verts[idx]
        dists = [math.hypot(n - bn, e - be) for (n, e) in rover]
        i_arrive = next((i for i, d in enumerate(dists) if d <= STOP_APPROACH_CM / 100.0), None)
        if i_arrive is None:
            continue  # never got close enough to call it a stop
        is_endpoint = (idx == last_idx)
        # incoming travel direction (unit) into this stop — "past the corner" is
        # the forward projection along it. At a 90° corner the perpendicular
        # departure contributes ~0, so this isolates the true overshoot (unlike a
        # euclidean max, which just picks up the rover driving on to the next side).
        pn, pe = verts[idx - 1]
        un, ue = (bn - pn), (be - pe)
        um = math.hypot(un, ue) or 1.0
        un, ue = un / um, ue / um
        # local window: from arrival until the rover clearly departs (> DEPART_M).
        j = i_arrive
        while j < len(dists) and dists[j] <= DEPART_M:
            j += 1
        coast = 0.0
        for i in range(i_arrive, max(i_arrive + 1, j)):
            n, e = rover[i]
            coast = max(coast, (n - bn) * un + (e - be) * ue)  # forward projection past corner
        coast = max(coast, 0.0)
        min_d = min(dists)
        dwell_s = round(times[min(j, len(times) - 1)] - times[i_arrive], 2)
        entry = {
            "vertex": idx,
            "is_endpoint": is_endpoint,
            "closest_cm": round(min_d * 100, 1),
            "coast_past_cm": round(coast * 100, 1),
            "dwell_s": dwell_s,
        }
        worst_coast = max(worst_coast, coast)
        if is_endpoint:
            # euclidean max-distance-after-arrival too (the D3 completion metric):
            # the endpoint must be a true final stop, so this counts toward verdict.
            entry["resting_cm"] = round(dists[-1] * 100, 1)
            endpoint_coast = max(dists[i_arrive:])
            entry["max_dist_after_arrival_cm"] = round(endpoint_coast * 100, 1)
            worst_coast = max(worst_coast, endpoint_coast)
        stops.append(entry)
    saw_done = any(st == S_DONE for (_t, st, _h) in s.seg)
    verdict = "PASS" if worst_coast <= COAST_MAX_CM / 100.0 else "FAIL"
    return {"available": True, "count": len(stops), "stops": stops,
            "worst_coast_cm": round(worst_coast * 100, 1),
            "reached_done": saw_done, "verdict": verdict}


def analyze_speed(s: Series) -> dict:
    if not s.vel_meas and not s.rpp:
        return {"available": False, "reason": "no speed signals"}
    meas = [v for (_t, v) in s.vel_meas]
    cmd = [d[3] for (_t, d) in s.rpp if d and len(d) > 3 and math.isfinite(d[3])]
    def _blk(vals):
        if not vals:
            return None
        sv = sorted(vals)
        return {"n": len(vals), "mean": round(statistics.fmean(vals), 3),
                "p95": round(_pctl(sv, 0.95), 3), "max": round(sv[-1], 3)}
    return {"available": True, "measured_m_s": _blk(meas), "commanded_m_s": _blk(cmd)}


def _edges(series):
    """(t, bool) series → list of (t, 'on'|'off') transitions."""
    out = []
    prev = None
    for (t, v) in series:
        b = bool(v)
        if prev is None or b != prev:
            out.append((t, "on" if b else "off"))
            prev = b
    return out


def analyze_spray(s: Series) -> dict:
    if not (s.spray_active or s.spray_desired or s.spray_commanded or s.spray_state):
        return {"available": False, "reason": "no /spray topics"}
    desired = s.spray_desired or s.spray_active
    de, ce, se = _edges(desired), _edges(s.spray_commanded), _edges(s.spray_state)
    # command latency: for each desired edge, nearest same-direction state edge
    lat = []
    for (t, d) in de:
        cand = [ts - t for (ts, dd) in se if dd == d and abs(ts - t) < 2.0]
        if cand:
            lat.append(min(cand, key=abs))
    misfire = 0
    # spray while clearly in transit (desired OFF but state ON) — sampled
    for (t, v) in s.spray_state:
        if v and desired and not _nearest(desired, t):
            misfire += 1
    return {"available": True,
            "desired_edges": len(de), "commanded_edges": len(ce), "state_edges": len(se),
            "state_vs_desired_latency_s": (round(statistics.fmean(lat), 3) if lat else None),
            "misfire_samples": misfire}


def analyze_health(s: Series) -> dict:
    events = []
    # OFFBOARD drops — mode leaves OFFBOARD while armed
    for (t, mode, armed) in s.state:
        if armed and mode and mode != "OFFBOARD":
            events.append({"t": round(t, 2), "kind": "offboard_drop", "detail": f"mode={mode}"})
    # setpoint-stream gaps > 0.5 s
    max_gap = 0.0
    gaps = 0
    for (a, _), (b, _) in zip(s.setpoint, s.setpoint[1:]):
        gap = b - a
        if gap > SETPOINT_GAP_S:
            gaps += 1
            max_gap = max(max_gap, gap)
    # RTK degradation during drive
    rtk_bad = sum(1 for (_t, fx) in s.gps if fx < RTK_MIN_FIX)
    # EKF position jumps
    jumps = 0
    for (t1, n1, e1, _), (t2, n2, e2, _) in zip(s.pose, s.pose[1:]):
        if math.hypot(n2 - n1, e2 - e1) > EKF_JUMP_M and (t2 - t1) < 0.5:
            jumps += 1
    # pose staleness from rpp/debug[6] (pose_age_ms)
    stale = sum(1 for (_t, d) in s.rpp if d and len(d) > 6 and d[6] > POSE_STALE_MS)
    # statustext failsafe / reject lines
    st_flags = [{"t": round(t, 2), "severity": sev, "text": txt}
                for (t, sev, txt) in s.statustext
                if any(k in txt.lower() for k in
                       ("fail", "reject", "denied", "lost", "error", "critical", "emergency"))]
    verdict = "PASS"
    if events or gaps or jumps or st_flags:
        verdict = "WARN"
    return {"available": True,
            "offboard_drops": len(events),
            "setpoint_gaps_over_0p5s": gaps,
            "max_setpoint_gap_s": round(max_gap, 3),
            "rtk_degraded_samples": rtk_bad,
            "ekf_position_jumps": jumps,
            "pose_stale_samples": stale,
            "statustext_flags": st_flags[:20],
            "events": events[:20],
            "verdict": verdict}


def analyze_config(s: Series, manifest) -> dict:
    """As-run params: manifest (FCU + RPP snapshot) plus a live RPP block from the bag."""
    out = {"from_manifest": None, "rpp_from_bag": None}
    if manifest:
        cfg = manifest.get("as_run_config", {})
        out["from_manifest"] = {
            "fcu_params": cfg.get("fcu_params"),
            "rpp_params": cfg.get("rpp_params"),
            "git_sha": (manifest.get("environment") or {}).get("git_sha"),
            "services": (manifest.get("environment") or {}).get("services"),
        }
    # labelled RPP block from a mid-run /rpp/debug sample (indices 11..38)
    labels = {
        11: "max_linear_vel", 12: "min_linear_vel", 16: "a_lat_max",
        18: "xy_goal_tolerance", 29: "corner_smooth_radius_m",
        35: "max_yaw_rate_body", 36: "max_linear_accel", 37: "max_linear_decel",
        38: "mission_speed",
    }
    if s.rpp:
        d = s.rpp[len(s.rpp) // 2][1]
        out["rpp_from_bag"] = {lbl: (round(d[i], 4) if i < len(d) else None)
                               for i, lbl in labels.items()}
    return out


# ═══════════════════════════════════════════════════════════════════════════
# report + main
# ═══════════════════════════════════════════════════════════════════════════
def _fmt_report(a: dict) -> str:
    L = []
    def line(x=""):
        L.append(x)
    line("=" * 72)
    line("MISSION BEHAVIOUR REPORT")
    line("=" * 72)
    line(f"bag        : {a['input']}")
    ident = a.get("identity") or {}
    if ident:
        line(f"mission    : {ident.get('name')}  (mode={ident.get('placement_mode')}, "
             f"staged={ident.get('is_staged')})")
    line(f"generated  : {a['generated_utc']}")
    line(f"topics     : {a['topic_count']} recorded, {a['pose_samples']} pose samples, "
         f"{a['duration_s']}s")
    line("")

    tr = a["tracking"]
    line("1. TRACKING (cross-track error)")
    if tr.get("available") and tr.get("overall"):
        o = tr["overall"]
        line(f"   overall : RMS {o['rms_cm']}  median {o['median_cm']}  "
             f"p95 {o['p95_cm']}  max {o['max_cm']} cm   (n={o['n']})")
        line(f"   bias    : {o['mean_signed_cm']:+} cm  "
             f"(L {o['left_frac']*100:.0f}% / R {o['right_frac']*100:.0f}%)")
        if tr.get("marking_only"):
            mo = tr["marking_only"]
            line(f"   marking : RMS {mo['rms_cm']}  p95 {mo['p95_cm']}  max {mo['max_cm']} cm")
        line(f"   verdict : {tr['verdict']}  (production class RMS ≤ {XTRACK_PROD_CM} cm)")
    else:
        line(f"   WARN — {tr.get('reason', 'unavailable')}")
    line("")

    st = a["stops"]
    line("2. STOPS (per waypoint / endpoint)")
    if st.get("available"):
        for e in st["stops"]:
            tag = "ENDPOINT" if e["is_endpoint"] else f"corner@{e['vertex']}"
            extra = (f"  resting {e['resting_cm']}cm  final-coast {e['max_dist_after_arrival_cm']}cm"
                     if "resting_cm" in e else "")
            line(f"   {tag:11s} closest {e['closest_cm']}cm  coast-past {e['coast_past_cm']}cm"
                 f"  dwell {e['dwell_s']}s{extra}")
        line(f"   worst coast-past {st['worst_coast_cm']}cm  DONE={st['reached_done']}  "
             f"verdict {st['verdict']}  (coast ≤ {COAST_MAX_CM}cm)")
    else:
        line(f"   WARN — {st.get('reason', 'unavailable')}")
    line("")

    pv = a["pivots"]
    line("3. PIVOTS (per corner / run boundary)")
    if pv.get("available") and pv.get("count"):
        for p in pv["pivots"]:
            line(f"   pivot{p['index']}: turn {p['turn_magnitude_deg']}°  "
                 f"init-err {p['initial_heading_err_deg']}°  settle {p['settle_err_deg']}° "
                 f"in {p['settle_time_s']}s  osc {p['oscillations']}  "
                 f"min-fwd {p['min_fwd_component_m_s']}  flip={p['reverse_flip']}")
        line(f"   any reverse-flip {pv['any_reverse_flip']}  worst settle "
             f"{pv['worst_settle_deg']}°  verdict {pv['verdict']}")
    elif pv.get("available"):
        line(f"   none — {pv.get('note', 'no pivots')}")
    else:
        line(f"   WARN — {pv.get('reason', 'unavailable')}")
    line("")

    sp = a["speed"]
    line("4. SPEED")
    if sp.get("available"):
        m, c = sp.get("measured_m_s"), sp.get("commanded_m_s")
        if m:
            line(f"   measured : mean {m['mean']}  p95 {m['p95']}  max {m['max']} m/s")
        if c:
            line(f"   commanded: mean {c['mean']}  p95 {c['p95']}  max {c['max']} m/s")
    else:
        line(f"   WARN — {sp.get('reason', 'unavailable')}")
    line("")

    spr = a["spray"]
    line("5. SPRAY")
    if spr.get("available"):
        line(f"   desired {spr['desired_edges']} edges  commanded {spr['commanded_edges']}  "
             f"state {spr['state_edges']}")
        line(f"   state↔desired latency {spr['state_vs_desired_latency_s']}s  "
             f"misfire samples {spr['misfire_samples']}")
    else:
        line(f"   WARN — {spr.get('reason', 'unavailable')}")
    line("")

    h = a["health"]
    line("6. HEALTH / ANOMALIES")
    if h.get("available"):
        line(f"   OFFBOARD drops {h['offboard_drops']}  setpoint gaps>0.5s "
             f"{h['setpoint_gaps_over_0p5s']} (max {h['max_setpoint_gap_s']}s)")
        line(f"   RTK degraded {h['rtk_degraded_samples']}  EKF jumps {h['ekf_position_jumps']}  "
             f"pose-stale {h['pose_stale_samples']}")
        for f in h["statustext_flags"]:
            line(f"   ! statustext[{f['severity']}] {f['text']}")
        line(f"   verdict {h['verdict']}")
    line("")

    cfg = a["config"]
    line("7. AS-RUN CONFIG")
    fm = cfg.get("from_manifest")
    if fm:
        line(f"   git {fm.get('git_sha')}  services {fm.get('services')}")
        fcu = (fm.get("fcu_params") or {}).get("values") or {}
        if fcu:
            line("   FCU: " + "  ".join(f"{k}={v}" for k, v in fcu.items() if v is not None))
    if cfg.get("rpp_from_bag"):
        line("   RPP: " + "  ".join(f"{k}={v}" for k, v in cfg["rpp_from_bag"].items()))
    if not fm and not cfg.get("rpp_from_bag"):
        line("   (no manifest and no /rpp/debug params)")
    line("")

    line("8. VERDICT")
    line(f"   ===> {a['verdict']} <===")
    if a["worst_offenders"]:
        line("   worst offenders:")
        for w in a["worst_offenders"]:
            line(f"     - {w}")
    line("=" * 72)
    return "\n".join(L) + "\n"


def analyze(root: str) -> dict:
    bag_dir, manifest = _find_bag_dir(root)
    if bag_dir is None:
        sys.exit(f"ERROR: no rosbag2 (.db3 / metadata.yaml) found under {root}")
    s = collect(bag_dir)
    dur = round(s.pose[-1][0] - s.pose[0][0], 1) if len(s.pose) > 1 else 0.0

    tracking = analyze_tracking(s)
    stops = analyze_stops(s)
    pivots = analyze_pivots(s)
    speed = analyze_speed(s)
    spray = analyze_spray(s)
    health = analyze_health(s)
    config = analyze_config(s, manifest)

    # overall verdict + worst offenders
    offenders = []
    fails = []
    for name, sec in (("tracking", tracking), ("stops", stops), ("pivots", pivots)):
        v = sec.get("verdict")
        if v == "FAIL":
            fails.append(name)
    if tracking.get("overall") and tracking["overall"]["rms_cm"] > XTRACK_PROD_CM:
        offenders.append(f"tracking RMS {tracking['overall']['rms_cm']}cm > {XTRACK_PROD_CM}cm")
    if stops.get("available") and stops["worst_coast_cm"] > COAST_MAX_CM:
        offenders.append(f"coast-past {stops['worst_coast_cm']}cm > {COAST_MAX_CM}cm")
    if pivots.get("available") and pivots.get("any_reverse_flip"):
        offenders.append("reverse-flip detected during a pivot")
    if pivots.get("available") and pivots.get("worst_settle_deg", 0) > SETTLE_TOL_DEG:
        offenders.append(f"pivot settle {pivots['worst_settle_deg']}° > {SETTLE_TOL_DEG}°")
    if health.get("offboard_drops"):
        offenders.append(f"{health['offboard_drops']} OFFBOARD drop(s)")
    verdict = "FAIL" if fails else ("WARN" if (offenders or health.get("verdict") == "WARN") else "PASS")

    return {
        "schema": "analyze_mission@1",
        "input": root,
        "bag_dir": bag_dir,
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "identity": (manifest or {}).get("identity"),
        "topic_count": len(s.topics_seen),
        "topics_seen": s.topics_seen,
        "pose_samples": len(s.pose),
        "duration_s": dur,
        "tracking": tracking,
        "stops": stops,
        "pivots": pivots,
        "speed": speed,
        "spray": spray,
        "health": health,
        "config": config,
        "worst_offenders": offenders,
        "verdict": verdict,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Post-mission behaviour analyser (G6)")
    ap.add_argument("bundle", help="recorder bundle dir OR a bare rosbag2 dir")
    ap.add_argument("-o", "--outdir", default=None,
                    help="where to write analysis.json + report.txt (default: the bundle dir)")
    ap.add_argument("--json-only", action="store_true", help="skip report.txt")
    ap.add_argument("--quiet", action="store_true", help="do not print the report to stdout")
    args = ap.parse_args()
    if not os.path.isdir(args.bundle):
        sys.exit(f"ERROR: not a directory: {args.bundle}")

    result = analyze(args.bundle)
    outdir = args.outdir or args.bundle
    try:
        os.makedirs(outdir, exist_ok=True)
        with open(os.path.join(outdir, "analysis.json"), "w") as f:
            json.dump(result, f, indent=2)
    except OSError as e:
        print(f"WARN: could not write analysis.json: {e}", file=sys.stderr)
    report = _fmt_report(result)
    if not args.json_only:
        try:
            with open(os.path.join(outdir, "report.txt"), "w") as f:
                f.write(report)
        except OSError as e:
            print(f"WARN: could not write report.txt: {e}", file=sys.stderr)
    if not args.quiet:
        print(report)
    return 0 if result["verdict"] != "FAIL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
