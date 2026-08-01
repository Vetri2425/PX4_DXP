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

# ── geometry fidelity (§9) ───────────────────────────────────────────────────
# The controller conditions /path before tracking it (rpp_controller_node
# _simplify_path_for_profile): collinear resample points are dropped so segment
# mode sees real segments instead of 5 cm crumbs. That is correct for generated
# geometry — but it also silently deletes surveyed vertices that bend the line
# only slightly, and nothing in this report could see it, because §1 TRACKING
# reads /rpp/debug[0], which is the controller's error against its OWN
# conditioned path. The rover graded its own homework and always passed.
#
# 2026-07-18 field case (tes_cross_line.dxf, 4 surveyed points): interior
# vertices bent the line 1.45° / 2.79° and sat 3.4 / 4.4 cm off the end-to-end
# chord. Both were dropped, /rpp/conditioned_path came out as TWO points, and
# the rover drove a straight line past them — while §1 reported RMS 0.73 cm and
# verdict PASS.
#
# This section compares the two recorded paths directly and reports what was
# removed. It is diagnostic, not a controller change: dropping a vertex is
# legitimate when the deviation is survey noise. The point is that it must be
# VISIBLE, with the number attached, so the operator can judge.
# DEFAULT only — override per mission (operator, staged with the plan) or per
# run (--survey-tol-cm). It is a property of the SURVEY, not of the analyser:
# a 1.7 cm single-epoch RS3 shot and a 5 mm averaged one cannot share a
# threshold. 2.5 cm is empirical, from the 2026-07-22 Emlid export — lateral
# RMS 1.7 cm at Samples=1 against vertex intent of 3.4/4.4 cm. Because this
# number decides a FAIL, every report states which source it came from.
SURVEY_TOL_CM = 2.5         # deviations below this read as survey noise, above = intent
# S8 ABSOLUTE ACCURACY. Separate budget from S1 (tracking) and S9 (geometry):
# both of those live entirely inside the local frame, so a wrong ANCHOR shifts the
# whole shape on the ground while every local metric still reads perfect.
ABS_MISS_WARN_CM = 5.0      # per-vertex absolute miss above this = WARN
ABS_MISS_FAIL_CM = 15.0     # ...above this = FAIL
ABS_BIAS_FAIL_CM = 10.0     # a consistent mean offset this large is a placement error
VERTEX_BEND_DEG = 0.8       # heading change that marks a /path point as a real vertex
# S9 TRAVERSAL. manifest.outcome.status is hardcoded "COMPLETE" by the recorder
# whenever it shuts down in an orderly way — a clean abort at 40% and a full run
# are indistinguishable there. Runs covering 24/64 and 76/86 waypoints both read
# COMPLETE, so filtering bags on that field silently mixes partial runs into an
# error budget. Coverage is the fraction of /path the pose actually reached.
COVERAGE_RADIUS_M = 0.25    # a path point counts as reached within this distance
COVERAGE_COMPLETE = 0.98    # at/above this the path was traversed end to end
COVERAGE_PARTIAL = 0.75     # below this it is not a full run at all

# P0-3 truncation guard. A recording whose FIRST pose is already moving faster
# than this began mid-mission: the recorder was still finalising the previous
# bundle when the mission started. 2026-07-30: two bags opened at 0.349 and
# 0.293 m/s, 48% along the path — three independent reviews then derived wrong
# root causes from the missing half. A rover genuinely at rest reads ≤~0.02.
# Deliberately NOT a distance-to-path[0] check: a complete run may legally
# start AT REST far from path[0] and drive a dry entry leg (the 191918 control
# run did exactly that from 40% up the path).
TRUNC_START_SPEED_MPS = 0.10


_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)


def _metres_per_degree(lat_deg: float) -> tuple[float, float]:
    """(north, east) metres per degree on the WGS84 ellipsoid.

    North uses the MERIDIONAL radius of curvature, east the prime vertical.
    Using the semi-major axis for north is a +0.62% scale error at 13 deg — the
    bug that was fixed in path_engine/parsers/georef.py on 2026-07-22. Kept
    duplicated here on purpose: this tool must stay a stdlib-only, independent
    check on the pipeline, not import the code it is auditing.
    """
    lat = math.radians(lat_deg)
    sn = math.sin(lat)
    w2 = 1.0 - _WGS84_E2 * sn * sn
    w = math.sqrt(w2)
    m_merid = _WGS84_A * (1.0 - _WGS84_E2) / (w2 * w)
    n_prime = _WGS84_A / w
    per = math.radians(1.0)
    return (m_merid * per, n_prime * per * math.cos(lat))


def _geodesic_ne_m(lat1, lon1, lat2, lon2) -> tuple[float, float]:
    """(north, east) metres from point 1 to point 2. Local-tangent, exact enough
    over a marking site (sub-mm below a few hundred metres)."""
    mn, me = _metres_per_degree((lat1 + lat2) * 0.5)
    return ((lat2 - lat1) * mn, (lon2 - lon1) * me)


def _geodesic_m(lat1, lon1, lat2, lon2) -> float:
    dn, de = _geodesic_ne_m(lat1, lon1, lat2, lon2)
    return math.hypot(dn, de)


def _perp_from_span(p, a, b) -> float:
    """Perpendicular distance of p from the infinite line a->b (metres).

    Measured against the SPAN BEING COLLAPSED, deliberately. The controller's
    own guard measures against the next raw sample ~5 cm away, which reads ~1 mm
    at a vertex sitting 4 cm off the retained chord — that is why its
    max_offset_m test never fires on a densified path.
    """
    dx, dy = b[0] - a[0], b[1] - a[1]
    h = math.hypot(dx, dy)
    if h < 1e-9:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    return abs(dx * (a[1] - p[1]) - dy * (a[0] - p[0])) / h


def _xtrack_to_polyline(p, poly) -> float:
    """Shortest distance from p to a polyline (metres)."""
    best = float("inf")
    for j in range(len(poly) - 1):
        a, b = poly[j], poly[j + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        s2 = dx * dx + dy * dy
        t = 0.0 if s2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / s2))
        d = math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))
        if d < best:
            best = d
    return best


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
    # mavros_msgs/State CDR layout drifted across MAVROS builds: older bags have
    # NO std header (validated on 2026-06/07 bags), the 2026-07-30 bags carry a
    # std_msgs/Header. P2-9: the headerless-only parser raised on every message
    # of the new layout and read_bag silently dropped them — 89/83 recorded
    # State samples decoded as zero and HEALTH graded PASS from an empty list.
    # Try both layouts; accept the first that yields a plausible mode string.
    last_err = None
    for with_header in (False, True):
        try:
            r = _CDR(d)
            if with_header:
                r.header()
            connected = r.boolean(); armed = r.boolean(); guided = r.boolean()
            manual = r.boolean(); mode = r.string(); system_status = r.u8()
            if len(mode) <= 32 and all(32 <= ord(c) < 127 for c in mode):
                return {"connected": connected, "armed": armed, "guided": guided,
                        "manual_input": manual, "mode": mode,
                        "system_status": system_status}
        except Exception as e:
            last_err = e
    raise ValueError(f"State CDR layout unrecognised: {last_err}")


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
    """mavros_msgs/GPSRAW — the receiver's OWN lat/lon, upstream of the EKF.

    This is the only position in the bag that is NOT downstream of the EKF:
    pose-derived geo is the EKF grading its own homework (it IS ekf_origin +
    local_NED), so run-to-run physical separation must be measured here.
    Field layout validated against the 2026-07-25 bags
    (bags/25_07_2026/Analysis/scripts/gpsraw.py): lat/lon land at the site,
    fix_type 6, h_acc 1.4-2.1 cm — reading anything shifted produces garbage
    coordinates, which is the sanity check.
    """
    r = _CDR(d); r.header()
    fix = r.u8()
    lat = r.i32(); lon = r.i32(); alt = r.i32()          # degE7 / degE7 / mm
    eph = r.u16(); epv = r.u16(); vel = r.u16(); cog = r.u16()
    sats = r.u8()
    r.i32()                                              # alt_ellipsoid (mm)
    h_acc = r.u32(); v_acc = r.u32()                     # mm
    return {"fix_type": fix, "lat": lat * 1e-7, "lon": lon * 1e-7,
            "alt": alt * 1e-3, "eph": eph, "epv": epv, "vel": vel, "cog": cog,
            "sats": sats, "h_acc": h_acc * 1e-3, "v_acc": v_acc * 1e-3}


def _p_navsatfix(d):
    """sensor_msgs/NavSatFix — the rover's own lat/lon, for absolute accuracy (S8).

    NavSatStatus is `int8 status` + `uint16 service` — service is SIXTEEN bits.
    Reading it as uint8 shifts every following float64 by one slot, so latitude
    lands in the longitude field and the whole fix is garbage. Validated against
    a real bag: the origin must come out near the site, not near (0, lat).
    """
    r = _CDR(d); r.header()
    r.i8()                               # NavSatStatus.status
    r.u16()                              # NavSatStatus.service  (uint16, not uint8)
    lat = r.f64(); lon = r.f64(); alt = r.f64()
    return {"lat": lat, "lon": lon, "alt": alt}


def _p_geopoint(d):
    """geographic_msgs/GeoPointStamped — the EKF local-frame origin (gp_origin).

    header + GeoPoint{float64 latitude, longitude, altitude}. This is the datum
    the local /path is expressed against; used to render /path back into lat/lon.
    """
    r = _CDR(d); r.header()
    lat = r.f64(); lon = r.f64(); alt = r.f64()
    return {"lat": lat, "lon": lon, "alt": alt}


PARSERS = {
    "sensor_msgs/msg/NavSatFix": _p_navsatfix,
    "geographic_msgs/msg/GeoPointStamped": _p_geopoint,
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
    """RMS / median / p95 / max / mean over |vals| in cm, plus signed bias.

    ``zero_frac`` makes left + right + zero sum to 1: the controller publishes
    hardcoded cross_track=0.0 while braking/aligning (not tracking), and those
    placeholder samples used to vanish from the L/R split — the missing 32%
    in the 2026-07-25 bags (B7).
    """
    if not vals:
        return None
    absv = sorted(abs(v) for v in vals)
    rms = math.sqrt(sum(v * v for v in vals) / len(vals))
    left = sum(1 for v in vals if v < 0)
    right = sum(1 for v in vals if v > 0)
    zero = len(vals) - left - right
    return {
        "n": len(vals),
        "rms_cm": round(rms * 100, 2),
        "median_cm": round(_pctl(absv, 0.5) * 100, 2),
        "p95_cm": round(_pctl(absv, 0.95) * 100, 2),
        "max_cm": round(absv[-1] * 100, 2),
        "mean_signed_cm": round((sum(vals) / len(vals)) * 100, 2),
        "left_frac": round(left / len(vals), 3),
        "right_frac": round(right / len(vals), 3),
        "zero_frac": round(zero / len(vals), 3),
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
        self.paths = []      # (t, [(n, e)]) EVERY /path message — one per run
        self.cond_paths = [] # (t, [(n, e)]) EVERY /rpp/conditioned_path message
        self.spray_active = []    # (t, bool) desired MARK
        self.spray_desired = []   # (t, bool)
        self.spray_commanded = [] # (t, bool)
        self.spray_state = []     # (t, bool)
        self.statustext = []      # (t, severity, text)
        self.gps = []             # (t, fix_type)
        self.gps_raw = []         # (t, lat, lon, sats, h_acc) — receiver's own fix (GPSRAW, ~5 Hz, EKF-independent)
        self.raw_fix = []         # (t, lat, lon, alt) — receiver's own fix (raw/fix NavSatFix, denser, EKF-independent)
        self.global_fix = []      # (t, lat, lon, alt) — EKF WGS84 position (= ekf_origin + local NED; NOT independent)
        self.path_z = None        # z bitfield of the kept /path (bit1 = must-hit)
        self.ekf_origin = None    # (lat, lon) — EKF local-frame datum (gp_origin)
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
                # /path is NED direct (x=N, y=E). A staged mission publishes SEVERAL
                # /path messages — a 2-pt transit hop, the mark run, then a 1-pt
                # endpoint marker. Taking the LAST one (the old behaviour) left
                # s.path as that single endpoint point, so analyze_stops ran against
                # a 1-point path: _stop_vertices returned [0], verts[idx-1] wrapped
                # to the same point, and every stop metric was meaningless. Keep the
                # longest — that is the mark run, the geometry actually driven.
                pts = [(p[0], p[1]) for p in m["poses"]]
                s.paths.append((t, pts))
                if s.path is None or len(pts) > len(s.path):
                    s.path = pts
                    # position.z is a bitfield: bit0 spray, bit1 must-hit vertex.
                    s.path_z = [int(round(p[2])) for p in m["poses"]]
        elif topic == "/rpp/conditioned_path":
            if m["poses"]:
                s.cond_paths.append((t, [(p[0], p[1]) for p in m["poses"]]))
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
            # receiver's own position — guard the (0,0) no-fix placeholder
            if abs(m["lat"]) <= 90.0 and not (m["lat"] == 0.0 and m["lon"] == 0.0):
                s.gps_raw.append((t, m["lat"], m["lon"], m["sats"], m["h_acc"]))
        elif topic == "/mavros/global_position/raw/fix":
            if m["lat"] == m["lat"] and abs(m["lat"]) <= 90.0 \
                    and not (m["lat"] == 0.0 and m["lon"] == 0.0):
                s.raw_fix.append((t, m["lat"], m["lon"], m["alt"]))
        elif topic == "/mavros/global_position/global":
            if m["lat"] == m["lat"] and abs(m["lat"]) <= 90.0:   # skip NaN / unset
                s.global_fix.append((t, m["lat"], m["lon"], m["alt"]))
        elif topic == "/mavros/global_position/gp_origin":
            # Latched EKF datum; one message. Guard NaN / (0,0) placeholder.
            if (m["lat"] == m["lat"] and abs(m["lat"]) <= 90.0
                    and not (m["lat"] == 0.0 and m["lon"] == 0.0)):
                s.ekf_origin = (m["lat"], m["lon"])
    for lst in (s.pose, s.vel_meas, s.vel_cmd, s.setpoint, s.state, s.rpp, s.seg,
                s.yaw_rate, s.spray_active, s.spray_desired, s.spray_commanded,
                s.spray_state, s.statustext, s.gps, s.gps_raw, s.raw_fix,
                s.global_fix):
        lst.sort(key=lambda r: r[0])
    return s


# ═══════════════════════════════════════════════════════════════════════════
# analysis sections
# ═══════════════════════════════════════════════════════════════════════════
def analyze_tracking(s: Series) -> dict:
    """Cross-track from /rpp/debug[0] (signed, m). Overall + marking-only.

    `marking_only` grades PAINT, so it must be gated on the topic that means
    "the valve is open" — /spray/state — and nothing earlier in the chain.

    Field 2026-07-30, bag stg_1cad4f00_..._132302: gating on /spray/active
    reported marking RMS 4.05 cm / max 9.70 and a FAIL verdict, while the
    valve-gated truth was 1.33 cm. /spray/active is published by the RPP from
    `_spray_flags[seg] and _spray_flags[seg+1]` — it is the path's geometric
    INTENT ("this segment should be painted"), and it went true 1.8 s before
    the valve. The spray controller holds the valve shut through its own gates
    (cross-track, pivot state, RTK), so the intent-gated window counts exactly
    the samples those gates were built to exclude: in that bag the 9.70 cm
    excursion was the rover recovering from a pivot that released 17.47° off,
    entirely dry. Over-reporting was 3.0x there, 1.7-1.8x on two other runs
    the same day, and negligible whenever the pivot was clean.

    The intent-vs-valve gap is not noise — it is a real defect signal — so it
    is reported separately as `approach_dry`, and never mixed into the verdict.
    """
    if not s.rpp:
        return {"available": False, "reason": "no /rpp/debug"}
    xt_all = [d[0] for (_t, d) in s.rpp if d and len(d) > 0 and math.isfinite(d[0])]
    out = {"available": True, "overall": _stat_block(xt_all)}

    # Valve truth, best available. state == actual valve; commanded == what the
    # spray node asked the FCU for (equals state absent an actuator fault);
    # desired == pre-gate want; active == RPP geometric intent (earliest, and
    # the one that caused the over-report). Degrade in that order, and record
    # which we used so a reader can tell a valve-gated number from a fallback.
    for name, sig in (("spray_state", s.spray_state),
                      ("spray_commanded", s.spray_commanded),
                      ("spray_desired", s.spray_desired),
                      ("spray_active", s.spray_active)):
        if sig:
            valve, valve_src = sig, name
            break
    else:
        valve, valve_src = None, None

    if valve:
        out["marking_gate"] = valve_src
        out["marking_gate_is_valve"] = valve_src in ("spray_state", "spray_commanded")
        xt_mark = []
        for (t, d) in s.rpp:
            if not d or not math.isfinite(d[0]):
                continue
            if _nearest(valve, t):
                xt_mark.append(d[0])
        out["marking_only"] = _stat_block(xt_mark) if xt_mark else None

        # Intended-to-paint but valve still shut: the approach transient the
        # spray gate suppressed. Large values here mean a bad run entry (e.g.
        # a pivot that released off-heading), NOT bad paint.
        if s.spray_active and valve_src != "spray_active":
            xt_dry = []
            for (t, d) in s.rpp:
                if not d or not math.isfinite(d[0]):
                    continue
                if _nearest(s.spray_active, t) and not _nearest(valve, t):
                    xt_dry.append(d[0])
            out["approach_dry"] = _stat_block(xt_dry) if xt_dry else None

    # B7: the verdict grades the PAINTED span when a spray signal exists. The
    # overall block averages in pivot/idle placeholder zeros (32% of samples in
    # the 2026-07-25 bags) and can dilute a 5 cm marking error below the gate.
    basis = out.get("marking_only") or out["overall"]
    out["verdict_basis"] = "marking_only" if out.get("marking_only") else "overall"
    out["verdict"] = (("PASS" if basis["rms_cm"] <= XTRACK_PROD_CM else "FAIL")
                      if basis else "WARN")
    return out


def _pivot_windows(seg):
    """Yield (i_stop, i_align, i_rel) index triples for each STOP→ALIGN→TRACK pivot.

    ``i_rel`` is None when no S_TRACK ever follows the ALIGN (e.g. the topic
    goes silent because the next run is the smooth profile — B3). B12a: the old
    fallback to n-1 silently measured ALIGN-start → end-of-bag as a "settle
    time" and let post-mission DONE samples (heading_err = NaN) leak into the
    reverse-flip scan.
    """
    wins = []
    i = 0
    n = len(seg)
    while i < n:
        if seg[i][1] == S_STOP:
            i_stop = i
            i_align = next((j for j in range(i_stop + 1, n) if seg[j][1] == S_ALIGN), None)
            if i_align is None:
                break
            i_rel = next((j for j in range(i_align + 1, n) if seg[j][1] == S_TRACK), None)
            wins.append((i_stop, i_align, i_rel))
            i = (i_rel if i_rel is not None else i_align) + 1
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
    settles: list[float] = []      # finite settle errors only (B12b)
    any_flip = False
    any_unreleased = False
    for k, (i_stop, i_align, i_rel) in enumerate(wins):
        released = i_rel is not None
        i_end = i_rel if released else len(s.seg) - 1
        t_stop, t_align = s.seg[i_stop][0], s.seg[i_align][0]
        t_end = s.seg[i_end][0]
        herr0_raw = s.seg[i_align][2]
        herr0 = math.degrees(abs(herr0_raw)) if math.isfinite(herr0_raw) else None
        # turn magnitude — pose-yaw swept between stop and release
        yaws = [y for (t, _n, _e, y) in s.pose if t_stop <= t <= t_end]
        swept = 0.0
        for a, b in zip(yaws, yaws[1:]):
            swept += _wrap(b - a)
        turn_deg = abs(math.degrees(swept))
        # reverse-flip — min commanded forward component while ACTIVELY turning
        min_fwd = math.inf
        vc_series = [(t, (vn, ve)) for (t, vn, ve) in s.vel_cmd]
        yw_series = [(pt, py) for (pt, _n, _e, py) in s.pose]
        for (t, st, he) in s.seg[i_align:i_end + 1]:
            if not math.isfinite(he):
                continue  # B12c: NaN heartbeat (DONE/IDLE) — not a turning sample
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
        # settle — |heading_err| at release. Only meaningful when the pivot
        # actually released; a NaN there is "unknown", never 0.0 (B12b).
        settle_deg = None
        if released:
            he_rel = (s.seg[i_rel][2] if s.seg[i_rel][1] == S_TRACK
                      else s.seg[i_rel - 1][2])
            if math.isfinite(he_rel):
                settle_deg = math.degrees(abs(he_rel))
                settles.append(settle_deg)
        # oscillation count during ALIGN (finite samples only)
        herr = [abs(s.seg[j][2]) for j in range(i_align, i_end + 1)
                if math.isfinite(s.seg[j][2])]
        rises = 0
        if herr:
            run_min = herr[0]
            for h in herr[1:]:
                if h > run_min + math.radians(OSC_BAND_DEG):
                    rises += 1
                    run_min = h
                else:
                    run_min = min(run_min, h)
        any_unreleased = any_unreleased or not released
        pivots.append({
            "index": k,
            "released": released,
            "initial_heading_err_deg": (round(herr0, 1) if herr0 is not None else None),
            "turn_magnitude_deg": round(turn_deg, 1),
            "settle_time_s": (round(t_end - t_align, 2) if released else None),
            "settle_err_deg": (round(settle_deg, 2) if settle_deg is not None else None),
            "min_fwd_component_m_s": (None if min_fwd is math.inf else round(min_fwd, 3)),
            "reverse_flip": flip,
            "oscillations": rises,
        })
    worst_settle = max(settles) if settles else None
    verdict = "PASS"
    if any_flip or (worst_settle is not None and worst_settle > SETTLE_TOL_DEG):
        verdict = "FAIL"
    return {"available": True, "count": len(pivots), "pivots": pivots,
            "any_reverse_flip": any_flip,
            "worst_settle_deg": (round(worst_settle, 2) if worst_settle is not None else None),
            "unreleased_pivots": any_unreleased,
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
    unmeasured = []
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
        is_endpoint = (idx == last_idx)
        # Arrival index = the START of the LAST contiguous run of samples inside
        # the arrival band. For the ENDPOINT this must be the arrival on the FINAL
        # approach, not the first time the rover was ever near the point: on a
        # there-and-back shape the rover starts parked beside its own endpoint, so
        # a first-ever match lands at t≈0 and every window below spans the whole
        # mission.
        #
        # 2026-08-01: this used to anchor on "the last sample beyond DEPART_M",
        # which SILENTLY DROPPED the stop whenever the rover came to rest further
        # than DEPART_M past the point — i.e. exactly when the overshoot was
        # worst. The final sample then satisfied the backward scan, search_from
        # ran off the end of the array, i_arrive was None, and `continue` deleted
        # the measurement. With the empty list the section reported
        # "worst coast-past 0.0cm ... PASS". Measured on 2026-08-01: every run
        # overshooting more than DEPART_M (+54.6, +56.2, -26.9 cm) reported
        # count=0/PASS, while every run under it (+11.8, +9.7, +6.0 cm) reported
        # the correct value. The analyser was blind in proportion to the fault.
        #
        # Taking the last contiguous in-band run keeps the there-and-back intent
        # (the final pass wins) and also handles a pass-through: the rover enters
        # the band, sails past, and `coast` is measured forward from that entry.
        band = STOP_APPROACH_CM / 100.0
        i_arrive = None
        for i in range(len(dists) - 1, -1, -1):
            if dists[i] <= band:
                i_arrive = i
            elif i_arrive is not None:
                break  # walked off the front of the last in-band run
        if i_arrive is None:
            # Genuinely never entered the band (e.g. an aborted run that stopped
            # short). Record it as unmeasured rather than dropping it silently —
            # a missing stop must not read as a clean one.
            unmeasured.append({"vertex": idx, "is_endpoint": is_endpoint,
                               "closest_cm": round(min(dists) * 100, 1),
                               "reason": f"never within {STOP_APPROACH_CM:.0f} cm"})
            continue
        # incoming travel direction (unit) into this stop — "past the corner" is
        # the forward projection along it. At a 90° corner the perpendicular
        # departure contributes ~0, so this isolates the true overshoot (unlike a
        # euclidean max, which just picks up the rover driving on to the next side).
        pn, pe = verts[idx - 1]
        un, ue = (bn - pn), (be - pe)
        um = math.hypot(un, ue) or 1.0
        un, ue = un / um, ue / um
        # local window: from arrival until the rover clearly departs (> DEPART_M).
        #
        # A CORNER legitimately drives on to the next segment, so the window must
        # close or `coast` just measures the next side. The ENDPOINT has no next
        # segment — the rover is supposed to halt — so closing the window at
        # DEPART_M silently CAPS the reported overshoot near 30 cm: bag
        # stg_9ecf2985 coasted 54.8 cm and reported 29.8. Run the endpoint window
        # to the end of the data so the headline number matches reality.
        j = i_arrive
        if is_endpoint:
            j = len(dists)
        else:
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
            # Where the rover actually came to rest. This is the D3 completion
            # metric and the trustworthy one — see FINAL_STOP_MAX_CM below.
            entry["resting_cm"] = round(dists[-1] * 100, 1)
            #
            # `max_dist_after_arrival_cm` used to live here as a second gate and is
            # GONE on purpose. It was a euclidean max, which cannot tell "still
            # 10 cm short of the point" from "10 cm past it" — every window you can
            # anchor it to has its own floor built in (DEPART_M gives ~30 cm,
            # STOP_APPROACH_CM gives ~10 cm), so it reported a constant artifact and
            # failed missions that rested 1.1 cm from their endpoint. Overshoot is
            # already measured correctly above, as the SIGNED forward projection
            # along the incoming direction (`coast`), which is what a euclidean
            # distance can never be.
        stops.append(entry)
    saw_done = any(st == S_DONE for (_t, st, _h) in s.seg)
    # Endpoint settling is its own criterion. FINAL_STOP_MAX_CM has been declared
    # since this analyser was written but was never wired to anything.
    resting_cm = next((e["resting_cm"] for e in stops if e.get("is_endpoint")), None)
    resting_bad = resting_cm is not None and resting_cm > FINAL_STOP_MAX_CM
    verdict = ("FAIL" if (worst_coast > COAST_MAX_CM / 100.0 or resting_bad) else "PASS")
    # Absence of evidence is not evidence of absence (same rule as §6 health).
    # A stop the analyser could not measure must never render as a stop it
    # measured and liked: with an empty list, worst_coast stays 0.0 and
    # resting_cm is None, so the expression above produced a confident PASS.
    if not stops:
        verdict = "UNAVAILABLE"
    elif unmeasured:
        verdict = "PARTIAL" if verdict == "PASS" else verdict
    return {"available": True, "count": len(stops), "stops": stops,
            "unmeasured": unmeasured,
            "worst_coast_cm": round(worst_coast * 100, 1),
            "endpoint_resting_cm": resting_cm,
            "endpoint_measured": resting_cm is not None,
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
    # OFFBOARD drops — a TRANSITION out of OFFBOARD while armed.
    #
    # 2026-08-01: this counted every SAMPLE whose mode != OFFBOARD, but MAVROS
    # republishes the latched State, so a pre-mission MANUAL burst scored one
    # "drop" per republish. Bag stg_9ecf2985 reported "OFFBOARD drops 9" from
    # nine duplicate MANUAL samples at t=0.31 — all BEFORE the rover ever entered
    # OFFBOARD at t=0.32, after which it held for the whole 28 s run. Those nine
    # phantoms were the sole `worst_offenders` entry and the only reason the run
    # graded WARN, while a real 54.6 cm endpoint overshoot graded PASS.
    #
    # Only a real OFFBOARD -> other edge counts, and only once armed: leaving
    # OFFBOARD while disarmed is not a failsafe event, it is the operator.
    prev_mode = None
    for (t, mode, armed) in s.state:
        if mode and armed and prev_mode == "OFFBOARD" and mode != "OFFBOARD":
            events.append({"t": round(t, 2), "kind": "offboard_drop",
                           "detail": f"OFFBOARD -> {mode}"})
        if mode:
            prev_mode = mode
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
    # pose staleness from rpp/debug[6] (pose_age_ms). B12: `nan > thresh` is
    # False, so NaN heartbeats used to count as HEALTHY — count them apart.
    stale = sum(1 for (_t, d) in s.rpp
                if d and len(d) > 6 and math.isfinite(d[6]) and d[6] > POSE_STALE_MS)
    stale_unknown = sum(1 for (_t, d) in s.rpp
                        if d and len(d) > 6 and not math.isfinite(d[6]))
    # statustext failsafe / reject lines
    st_flags = [{"t": round(t, 2), "severity": sev, "text": txt}
                for (t, sev, txt) in s.statustext
                if any(k in txt.lower() for k in
                       ("fail", "reject", "denied", "lost", "error", "critical", "emergency"))]
    verdict = "PASS"
    if events or gaps or jumps or st_flags:
        verdict = "WARN"
    # P2-9: absence of evidence must never grade as evidence of absence. With
    # zero decoded State samples, "0 OFFBOARD drops" is manufactured — the
    # 2026-07-30 bags recorded 89/83 State messages that a stale CDR layout
    # silently dropped, and this section printed PASS from an empty list.
    if not s.state:
        verdict = "UNAVAILABLE"
    return {"available": True,
            "state_samples": len(s.state),
            "offboard_drops": len(events),
            "setpoint_gaps_over_0p5s": gaps,
            "max_setpoint_gap_s": round(max_gap, 3),
            "rtk_degraded_samples": rtk_bad,
            "ekf_position_jumps": jumps,
            "pose_stale_samples": stale,
            "pose_age_unknown_samples": stale_unknown,
            "statustext_flags": st_flags[:20],
            "events": events[:20],
            "verdict": verdict}


def _path_vertices(poly, bend_deg=VERTEX_BEND_DEG):
    """Interior points of *poly* where the heading turns by more than bend_deg.

    On a densified /path these are exactly the CAD-authored vertices: every
    other point is a resample sitting dead on its own leg.
    """
    out = []
    for i in range(1, len(poly) - 1):
        h0 = math.atan2(poly[i][1] - poly[i - 1][1], poly[i][0] - poly[i - 1][0])
        h1 = math.atan2(poly[i + 1][1] - poly[i][1], poly[i + 1][0] - poly[i][0])
        d = abs(_wrap(h1 - h0))
        if math.degrees(d) > bend_deg:
            out.append((i, poly[i], math.degrees(d)))
    return out


def resolve_survey_tol_cm(manifest, cli_cm: float | None = None) -> tuple[float, str]:
    """Return (tolerance_cm, where_it_came_from).

    Precedence: explicit --survey-tol-cm, then the value the operator staged
    with the mission, then the documented default. The provenance string is
    returned rather than logged because this threshold decides whether §7
    FAILs — a verdict that turns on an unattributed constant is not a verdict.

    An unusable staged value (non-numeric, zero, negative, absurd) falls back
    to the default and SAYS SO, instead of silently judging the run by a
    number nobody chose.
    """
    if cli_cm is not None:
        return float(cli_cm), "--survey-tol-cm"

    # B9: the recorder writes "plan_provenance"; "staged_mission" never existed
    # in a real manifest (only in test fixtures). Accept both, prefer the real one.
    staged = ((manifest or {}).get("plan_provenance")
              or (manifest or {}).get("staged_mission") or {})
    raw = staged.get("survey_tolerance_m")
    if raw is not None:
        try:
            val_m = float(raw)
        except (TypeError, ValueError):
            return SURVEY_TOL_CM, f"default (staged value {raw!r} is not a number)"
        if 0.0 < val_m <= 1.0:
            return val_m * 100.0, "staged with the mission (operator-set)"
        return SURVEY_TOL_CM, f"default (staged value {val_m} m out of range)"

    return SURVEY_TOL_CM, "built-in default — not set for this survey"


def analyze_recording_integrity(s: Series) -> dict:
    """Was the recorder actually running when the mission started? (P0-3)

    Every downstream section silently assumes the bag covers the whole
    mission. When the recorder's blind window eats the opening, coverage
    metrics report SKIPPED/STARTED_LATE for geometry that was driven but not
    recorded — and reviews then build root causes on the gap. The signature is
    unambiguous: a bag that begins mid-mission begins with the rover already
    at speed. One is flagged here so no consumer has to re-derive it.
    """
    path = (s.paths[0][1] if s.paths else None) or s.path
    if not s.pose:
        return {"available": False, "reason": "no pose samples"}
    t0, n0, e0, _ = s.pose[0]
    speed0 = _nearest(s.vel_meas, t0) if s.vel_meas else None
    if speed0 is None and len(s.pose) > 1:
        # fall back to pose differencing over the opening half-second
        w = [p for p in s.pose if p[0] <= t0 + 0.5]
        dt = w[-1][0] - w[0][0] if len(w) > 1 else 0.0
        if dt > 0:
            speed0 = math.hypot(w[-1][1] - w[0][1], w[-1][2] - w[0][2]) / dt
    out = {"available": True,
           "first_pose_speed_mps": round(speed0, 3) if speed0 is not None else None,
           "truncated": bool(speed0 is not None and speed0 > TRUNC_START_SPEED_MPS)}
    if path:
        ni = min(range(len(path)),
                 key=lambda i: math.hypot(path[i][0] - n0, path[i][1] - e0))
        out["first_pose_nearest_path_index"] = ni
        out["first_pose_dist_to_path0_m"] = round(
            math.hypot(path[0][0] - n0, path[0][1] - e0), 3)
        out["path_points"] = len(path)
    return out


def analyze_traversal(s: Series, radius_m: float = COVERAGE_RADIUS_M) -> dict:
    """Did the rover actually GO everywhere it was told to? (report §9)

    Every other section measures how well the rover drove the part of the path
    it drove. None of them notice that it stopped a third of the way through:
    an aborted run just yields fewer samples, and its statistics are biased —
    it never reaches the far corners, so its error budget reads low. That is
    what makes a partial run dangerous when it is averaged in with full ones.

    Coverage = fraction of /path points that the pose came within radius_m of.
    Deliberately NOT cross-track: a rover that stops dead on the line has zero
    xtrack error for the rest of the mission it never drove.
    """
    if not s.path:
        return {"available": False, "reason": "no /path in bag"}
    if len(s.pose) < 2:
        return {"available": False, "reason": "no pose samples in bag"}

    # Uniform grid over the poses, cell = radius, so each path point tests only
    # its own cell and the 8 around it. The naive all-pairs scan is O(path x
    # pose), which is fine for a 160-point square and minutes of wall-clock for
    # a 17k-waypoint plan against 10k pose samples.
    cell = max(radius_m, 1e-6)
    grid: dict[tuple[int, int], list[tuple[float, float]]] = {}
    for rec in s.pose:
        n, e = rec[1], rec[2]
        grid.setdefault((int(n // cell), int(e // cell)), []).append((n, e))

    r2 = radius_m * radius_m
    covered: list[bool] = []
    for (pn, pe) in s.path:
        gn, ge = int(pn // cell), int(pe // cell)
        hit = False
        for dn in (-1, 0, 1):
            for de in (-1, 0, 1):
                for (n, e) in grid.get((gn + dn, ge + de), ()):
                    if (n - pn) ** 2 + (e - pe) ** 2 <= r2:
                        hit = True
                        break
                if hit:
                    break
            if hit:
                break
        covered.append(hit)

    total = len(covered)
    num_covered = sum(covered)
    coverage = num_covered / total if total else 0.0

    # WHERE the misses sit matters more than how many there are, and the 07-22
    # field bags proved a two-way split (abort vs skip) was too coarse: the
    # 24/64 run covered the LAST 24 points, not the first — it never drove the
    # beginning at all. Measure the leading and trailing runs of misses and
    # classify from those.
    first_covered = next((i for i, c in enumerate(covered) if c), None)
    last_covered = max((i for i, c in enumerate(covered) if c), default=-1)
    lead_missing = first_covered if first_covered is not None else total
    trail_missing = (total - 1 - last_covered) if last_covered >= 0 else 0
    interior_missing = (
        total - num_covered - lead_missing - trail_missing if num_covered else 0
    )

    if num_covered == 0:
        shape = "NONE"
    elif lead_missing == trail_missing == interior_missing == 0:
        shape = "FULL"
    elif interior_missing:
        # Missed geometry with driven path on both sides of it.
        shape = "INTERIOR_GAP"
    elif lead_missing and trail_missing:
        shape = "MIDDLE_ONLY"
    elif lead_missing:
        # Drove to the end, but the start was never reached: a late start, a
        # resumed mission, or a recorder that began after the rover did.
        shape = "STARTED_LATE"
    else:
        shape = "STOPPED_EARLY"

    if coverage >= COVERAGE_COMPLETE:
        status = "COMPLETE"
    elif coverage >= COVERAGE_PARTIAL:
        status = "MOSTLY"
    else:
        status = "PARTIAL"

    return {
        "available": True,
        "status": status,
        "shape": shape,
        "coverage": round(coverage, 4),
        "points_total": total,
        "points_covered": num_covered,
        "radius_cm": round(radius_m * 100, 1),
        "first_covered_index": first_covered,
        "last_covered_index": last_covered,
        "missing_leading": lead_missing,
        "missing_trailing": trail_missing,
        "missing_interior": interior_missing,
        # Kept for the one question most callers ask. NOTE: false on a run that
        # never started, which is why `shape` exists — do not read this alone.
        "stopped_early": shape == "STOPPED_EARLY",
        "verdict": "PASS" if status == "COMPLETE" else "FAIL",
    }


def _write_traversal_to_manifest(root: str, traversal: dict) -> str | None:
    """Fold the traversal verdict back into the bundle's manifest.

    The whole point is to be machine-detectable WITHOUT opening the bag, so the
    number has to live next to outcome. Rewrites only this one key, atomically,
    and never raises — a bundle whose manifest cannot be updated is still a
    valid bundle with a valid analysis.json.
    """
    mpath = os.path.join(root, "manifest.json")
    if not os.path.isfile(mpath):
        return None
    try:
        with open(mpath) as f:
            manifest = json.load(f)
        if not isinstance(manifest, dict):
            return None
        manifest["traversal"] = traversal
        tmp = mpath + ".tmp"
        with open(tmp, "w") as f:
            json.dump(manifest, f, indent=2)
        os.replace(tmp, mpath)
        return mpath
    except Exception as exc:
        return f"ERROR: {type(exc).__name__}: {exc}"


def analyze_geometry_fidelity(s: Series, survey_tol_cm: float = SURVEY_TOL_CM,
                              survey_tol_source: str = "caller-supplied") -> dict:
    """Did the rover track the geometry it was GIVEN? (§9)

    Independent of §1: §1 asks "how well did the controller follow its own
    conditioned path"; this asks "does that conditioned path still contain the
    surveyed geometry". A mission can pass §1 perfectly while having driven a
    different shape — that is the failure mode this exists to catch.
    """
    if not s.paths:
        return {"available": False, "reason": "no /path in bag"}
    if not s.cond_paths:
        return {"available": False,
                "reason": "no /rpp/conditioned_path in bag (recorder predates it, or "
                          "the controller never conditioned a run)"}

    # Pair each /path run with the conditioned path published for it. Both are
    # latched and emitted per run in the same order, so index pairing holds; the
    # endpoint guard keeps a mismatched count from silently mis-pairing.
    runs = []
    for k, (t_in, pin) in enumerate(s.paths):
        if k >= len(s.cond_paths):
            break
        pout = s.cond_paths[k][1]
        if len(pin) < 3:
            continue                      # transit hop / endpoint marker: nothing to drop
        if math.dist(pin[0], pout[0]) > 0.5 or math.dist(pin[-1], pout[-1]) > 0.5:
            continue                      # not the same run — refuse to guess

        dropped = []
        for idx, v, bend in _path_vertices(pin):
            # nearest retained node: if the vertex survived it is there exactly
            near = min((math.dist(v, q), q) for q in pout)
            if near[0] <= 0.01:
                continue
            # How far off the path the controller ACTUALLY tracked did this
            # vertex end up? That is the whole question, and it is just the
            # vertex's distance to the conditioned polyline.
            dev = _xtrack_to_polyline(v, pout)
            rec = {
                "path_index": idx,
                "n": round(v[0], 4), "e": round(v[1], 4),
                "bend_deg": round(bend, 2),
                "deviation_cm": round(dev * 100, 2),
                "nearest_tracked_node_cm": round(near[0] * 100, 1),
                "intent": "INTENT" if dev * 100 >= survey_tol_cm else "noise",
            }
            dropped.append(rec)
        runs.append({
            "run": k,
            "planned_points": len(pin),
            "tracked_points": len(pout),
            "vertices_in_plan": len(_path_vertices(pin)),
            "dropped": dropped,
        })

    if not runs:
        return {"available": False, "reason": "no comparable mark run"}

    # rover closest approach to each dropped vertex — the number that matters
    track = [(n, e) for (_t, n, e, _y) in s.pose]
    for r in runs:
        for d in r["dropped"]:
            v = (d["n"], d["e"])
            d["rover_closest_cm"] = (round(min(math.dist(v, p) for p in track) * 100, 2)
                                     if track else None)

    # independent cross-track: driven vs the PLANNED geometry, not the conditioned one
    xt_true = None
    mark = max((p for _t, p in s.paths), key=len, default=None)
    if track and mark and len(mark) >= 2:
        i0 = min(range(len(track)), key=lambda i: math.dist(track[i], mark[0]))
        i1 = min(range(len(track)), key=lambda i: math.dist(track[i], mark[-1]))
        seg = track[min(i0, i1):max(i0, i1) + 1]
        if len(seg) >= 2:
            errs = [_xtrack_to_polyline(p, mark) for p in seg]
            xt_true = {
                "rms_cm": round(math.sqrt(sum(e * e for e in errs) / len(errs)) * 100, 2),
                "max_cm": round(max(errs) * 100, 2),
                "n": len(errs),
            }

    all_dropped = [d for r in runs for d in r["dropped"]]
    intent = [d for d in all_dropped if d["intent"] == "INTENT"]
    worst = max((d["deviation_cm"] for d in all_dropped), default=0.0)
    return {
        "available": True,
        "survey_tol_cm": survey_tol_cm,
        "survey_tol_source": survey_tol_source,
        "runs": runs,
        "dropped_total": len(all_dropped),
        "dropped_above_tolerance": len(intent),
        "worst_deviation_cm": worst,
        "xtrack_vs_planned": xt_true,
        "verdict": "FAIL" if intent else ("WARN" if all_dropped else "PASS"),
    }


def _surveyed_latlon_from_source(manifest) -> tuple[list, str]:
    """The INDEPENDENT ground truth: surveyed lat/lon straight from the source file.

    This must NOT come from the mission's own anchor. Placement computes
    local = T(global) once, from a single pose/global correspondence pair; if T
    is wrong (a skewed pair — what POSE_GLOBAL_MAX_SKEW_MS guards), converting
    the driven local position back through T reproduces the intended lat/lon
    exactly and detects nothing. Re-reading the source file sidesteps T entirely.

    Returns (points, provenance) where points is [(lat, lon), ...].
    """
    # B9: the recorder writes "plan_provenance"; accept the legacy fixture key
    # too so old test bundles keep working.
    staged = ((manifest or {}).get("plan_provenance")
              or (manifest or {}).get("staged_mission") or {})

    # Preferred: ground truth staged INSIDE the mission artifact. An app-planned
    # trajectory (POST /api/path/plan-trajectory) has no source file at all —
    # the geometry was fitted in the app and posted as NED runs — so there is
    # nothing for the file branch below to re-read and §8 would report "source
    # file unavailable" on every such mission. It is still INDEPENDENT of the
    # mission's own anchor, which is the property that makes §8 mean anything:
    # these lat/lon came off the surveyor's receiver, not from inverting T.
    gt = staged.get("survey_ground_truth")
    if isinstance(gt, list) and gt:
        pts = []
        for row in gt:
            if not isinstance(row, dict):
                continue
            try:
                lat, lon = float(row["lat"]), float(row["lon"])
            except (KeyError, TypeError, ValueError):
                continue
            if abs(lat) <= 90.0 and abs(lon) <= 180.0:
                pts.append((lat, lon))
        if pts:
            return pts, f"staged survey_ground_truth ({len(pts)} point(s))"

    src = staged.get("source_file")
    if not src or not os.path.isfile(src):
        return [], f"source file unavailable ({src or 'not recorded'})"
    ext = os.path.splitext(src)[1].lower()
    try:
        if ext == ".csv":
            import csv as _csv
            with open(src, encoding="utf-8-sig", errors="replace") as f:
                rows = list(_csv.DictReader(f))
            hdr = {k.strip().lower(): k for k in (rows[0].keys() if rows else {})}
            klat = next((hdr[k] for k in ("latitude", "lat") if k in hdr), None)
            klon = next((hdr[k] for k in ("longitude", "lon", "long") if k in hdr), None)
            if not (klat and klon):
                return [], "CSV has no Latitude/Longitude columns"
            pts = []
            for r in rows:
                try:
                    pts.append((float(r[klat]), float(r[klon])))
                except (TypeError, ValueError):
                    continue
            return pts, f"survey CSV {os.path.basename(src)}"
        if ext == ".dxf":
            import ezdxf
            doc = ezdxf.readfile(src)
            pts = [(p.dxf.location.y, p.dxf.location.x)
                   for p in doc.modelspace() if p.dxftype() == "POINT"]
            # A georeferenced DXF stores lat in y and lon in x (parser maps
            # DXF y->north, x->east and leaves the values unscaled).
            pts = [(a, b) for a, b in pts if abs(a) <= 90.0 and abs(b) <= 180.0]
            return pts, f"DXF POINT layer of {os.path.basename(src)}"
    except Exception as exc:
        return [], f"{type(exc).__name__} reading {os.path.basename(src)}: {exc}"
    return [], f"unsupported source extension {ext!r}"


def analyze_absolute(s: Series, manifest) -> dict:
    """Did the rover reach the real-world coordinates? (S8)

    S1 asks "did the controller follow its own path" and S9 asks "was that path
    the right shape". Neither can see a PLACEMENT error, because both work in the
    local frame — if the anchor is off by 40 cm the whole shape moves on the
    ground and both still report centimetres.

    Method: for each surveyed point, find the rover's closest approach in the
    LOCAL frame, take the CONCURRENT global fix, and measure the geodesic to the
    surveyed lat/lon. Local closest-approach is only used to pick the instant;
    the comparison itself is global-to-global, so the anchor never enters it.
    """
    out = {"available": False}
    if not s.global_fix:
        out["reason"] = "no /mavros/global_position/global in bag"
        return out
    if not s.pose:
        out["reason"] = "no pose"
        return out

    truth, provenance = _surveyed_latlon_from_source(manifest)
    out["provenance"] = provenance
    if not truth:
        out["reason"] = f"no independent ground truth ({provenance})"
        out["fixes"] = len(s.global_fix)
        return out

    # Local-frame targets to time the closest approach against: prefer the
    # must-hit vertices the planner declared, else every /path vertex.
    path = s.path or []
    if not path:
        out["reason"] = "no /path"
        return out
    if s.path_z and len(s.path_z) == len(path):
        targets = [p for p, z in zip(path, s.path_z) if z & 2]
        target_kind = "must-hit vertices"
    else:
        targets = []
        target_kind = ""
    if not targets:
        targets = path
        target_kind = "all /path vertices (no must-hit flags in bag)"

    # Pair each surveyed point with a local target. Counts usually match (both
    # are the surveyed vertex set); if not, say so rather than guessing.
    out["n_truth"] = len(truth)
    out["n_targets"] = len(targets)
    out["target_kind"] = target_kind
    if len(truth) != len(targets):
        out["reason"] = (f"cannot pair {len(truth)} surveyed point(s) with "
                         f"{len(targets)} local target(s)")
        return out

    gf = s.global_fix
    rows = []
    for i, (tgt_n, tgt_e) in enumerate(targets):
        best_t, best_d = None, float("inf")
        for (t, n, e, _yaw) in s.pose:
            d = math.hypot(n - tgt_n, e - tgt_e)
            if d < best_d:
                best_d, best_t = d, t
        if best_t is None:
            continue
        # nearest global fix in time
        j = min(range(len(gf)), key=lambda k: abs(gf[k][0] - best_t))
        t_fix, lat_r, lon_r, _alt = gf[j]
        skew_ms = abs(t_fix - best_t) * 1000.0
        lat_s, lon_s = truth[i]
        miss_m = _geodesic_m(lat_s, lon_s, lat_r, lon_r)
        dn, de = _geodesic_ne_m(lat_s, lon_s, lat_r, lon_r)
        rows.append({"i": i, "miss_cm": miss_m * 100.0,
                     "dn_cm": dn * 100.0, "de_cm": de * 100.0,
                     "local_approach_cm": best_d * 100.0, "skew_ms": skew_ms})

    if not rows:
        out["reason"] = "no usable closest-approach samples"
        return out

    misses = [r["miss_cm"] for r in rows]
    mean_dn = sum(r["dn_cm"] for r in rows) / len(rows)
    mean_de = sum(r["de_cm"] for r in rows) / len(rows)
    bias_cm = math.hypot(mean_dn, mean_de)
    # Scatter about the mean separates a PLACEMENT shift (large bias, small
    # scatter) from noise/tracking (small bias, large scatter).
    scatter = math.sqrt(sum((r["dn_cm"] - mean_dn) ** 2 + (r["de_cm"] - mean_de) ** 2
                            for r in rows) / len(rows))
    max_skew = max(r["skew_ms"] for r in rows)

    verdict = "PASS"
    notes = []
    if max(misses) > ABS_MISS_FAIL_CM:
        verdict = "FAIL"; notes.append(f"worst miss {max(misses):.1f} cm > {ABS_MISS_FAIL_CM:.0f} cm")
    elif max(misses) > ABS_MISS_WARN_CM:
        verdict = "WARN"; notes.append(f"worst miss {max(misses):.1f} cm > {ABS_MISS_WARN_CM:.0f} cm")
    if bias_cm > ABS_BIAS_FAIL_CM:
        verdict = "FAIL"
        notes.append(f"systematic {bias_cm:.1f} cm offset with only {scatter:.1f} cm scatter "
                     f"— this is PLACEMENT, not tracking")
    if max_skew > 500.0:
        notes.append(f"global fix up to {max_skew:.0f} ms from closest approach — "
                     f"at speed that is itself centimetres; treat as indicative")

    out.update({"available": True, "verdict": verdict, "rows": rows,
                "max_cm": max(misses), "mean_cm": sum(misses) / len(misses),
                "bias_cm": bias_cm, "bias_n_cm": mean_dn, "bias_e_cm": mean_de,
                "scatter_cm": scatter, "max_skew_ms": max_skew, "notes": notes})
    return out


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
        # B12: a midpoint frame can be a stop-debug heartbeat whose whole RPP
        # block is NaN — pick the first frame whose config indices are finite.
        d = next((d for (_t, d) in s.rpp
                  if d and len(d) > 38 and all(math.isfinite(d[i]) for i in labels)),
                 s.rpp[len(s.rpp) // 2][1])
        out["rpp_from_bag"] = {lbl: (round(d[i], 4) if i < len(d) and math.isfinite(d[i])
                                     else None)
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
             f"(L {o['left_frac']*100:.0f}% / R {o['right_frac']*100:.0f}% / "
             f"zero {o.get('zero_frac', 0)*100:.0f}%)")
        if tr.get("marking_only"):
            mo = tr["marking_only"]
            gate = tr.get("marking_gate", "?")
            note = "" if tr.get("marking_gate_is_valve") else "  ⚠ NOT valve-gated"
            line(f"   marking : RMS {mo['rms_cm']}  p95 {mo['p95_cm']}  max {mo['max_cm']} cm"
                 f"   (n={mo['n']}, gate={gate}){note}")
        if tr.get("approach_dry"):
            ad = tr["approach_dry"]
            line(f"   approach: RMS {ad['rms_cm']}  max {ad['max_cm']} cm  (n={ad['n']}) — "
                 f"intended-to-paint but valve SHUT; not paint error, but a large")
            line(f"             value flags a bad run entry (e.g. pivot released off-heading)")
        line(f"   verdict : {tr['verdict']}  on {tr.get('verdict_basis', 'overall')} "
             f"(production class RMS ≤ {XTRACK_PROD_CM} cm)")
    else:
        line(f"   WARN — {tr.get('reason', 'unavailable')}")
    line("")

    st = a["stops"]
    line("2. STOPS (per waypoint / endpoint)")
    if st.get("available"):
        for e in st["stops"]:
            tag = "ENDPOINT" if e["is_endpoint"] else f"corner@{e['vertex']}"
            extra = f"  resting {e['resting_cm']}cm" if "resting_cm" in e else ""
            line(f"   {tag:11s} closest {e['closest_cm']}cm  coast-past {e['coast_past_cm']}cm"
                 f"  dwell {e['dwell_s']}s{extra}")
        for u in st.get("unmeasured", []):
            tag = "endpoint" if u["is_endpoint"] else f"vertex {u['vertex']}"
            line(f"   {tag:11s} NOT MEASURED — {u['reason']} (closest {u['closest_cm']}cm)")
        if not st.get("stops"):
            line(f"   ⚠ NO STOP WAS MEASURED — this is NOT a pass. The rover never")
            line(f"     entered the {STOP_APPROACH_CM:.0f} cm band around any stop point,")
            line(f"     or the run was aborted. Decode the bag before trusting anything here.")
        rest = st.get("endpoint_resting_cm")
        rest_s = f"{rest}cm" if rest is not None else "NOT MEASURED"
        line(f"   worst coast-past {st['worst_coast_cm']}cm  "
             f"endpoint resting {rest_s}  DONE={st['reached_done']}  "
             f"verdict {st['verdict']}  (coast ≤ {COAST_MAX_CM}cm, resting ≤ {FINAL_STOP_MAX_CM}cm)")
    else:
        line(f"   WARN — {st.get('reason', 'unavailable')}")
    line("")

    pv = a["pivots"]
    line("3. PIVOTS (per corner / run boundary)")
    if pv.get("available") and pv.get("count"):
        for p in pv["pivots"]:
            if p.get("released", True):
                tail = (f"settle {p['settle_err_deg']}° in {p['settle_time_s']}s")
            else:
                tail = "NEVER RELEASED (no S_TRACK followed — topic went silent, see B3)"
            line(f"   pivot{p['index']}: turn {p['turn_magnitude_deg']}°  "
                 f"init-err {p['initial_heading_err_deg']}°  {tail}  "
                 f"osc {p['oscillations']}  "
                 f"min-fwd {p['min_fwd_component_m_s']}  flip={p['reverse_flip']}")
        ws = pv.get("worst_settle_deg")
        line(f"   any reverse-flip {pv['any_reverse_flip']}  worst settle "
             f"{ws if ws is not None else 'n/a (no released pivot)'}°  verdict {pv['verdict']}")
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
    if h.get("available") and h.get("verdict") == "UNAVAILABLE":
        line(f"   ! zero /mavros/state samples decoded ({h.get('state_samples', 0)}) — ")
        line("   ! OFFBOARD continuity is UNVERIFIED, not verified-clean. If the bag")
        line("   ! contains State messages, the CDR layout drifted past the parser.")
        line(f"   verdict {h['verdict']}")
    elif h.get("available"):
        line(f"   OFFBOARD drops {h['offboard_drops']}  setpoint gaps>0.5s "
             f"{h['setpoint_gaps_over_0p5s']} (max {h['max_setpoint_gap_s']}s)")
        line(f"   RTK degraded {h['rtk_degraded_samples']}  EKF jumps {h['ekf_position_jumps']}  "
             f"pose-stale {h['pose_stale_samples']}")
        for f in h["statustext_flags"]:
            line(f"   ! statustext[{f['severity']}] {f['text']}")
        line(f"   verdict {h['verdict']}")
    line("")

    g = a.get("geometry") or {}
    line("7. GEOMETRY FIDELITY (planned /path vs tracked /rpp/conditioned_path)")
    if not g.get("available"):
        line(f"   unavailable — {g.get('reason')}")
    else:
        for r in g["runs"]:
            line(f"   run {r['run']}: planned {r['planned_points']} pts "
                 f"({r['vertices_in_plan']} CAD vertices) -> tracked {r['tracked_points']} pts")
        xt = g.get("xtrack_vs_planned")
        if xt:
            line(f"   driven vs PLANNED geometry : RMS {xt['rms_cm']} cm  max {xt['max_cm']} cm  "
                 f"(n={xt['n']})")
            line("     ^ independent of §1, which measures against the CONDITIONED path")
        # State the threshold and where it came from even when nothing was
        # dropped: a PASS at a tolerance nobody chose is not evidence.
        line(f"   survey tolerance {g['survey_tol_cm']:.2f} cm  "
             f"[{g.get('survey_tol_source', '?')}]")
        if not g["dropped_total"]:
            line("   no surveyed vertex was dropped — the rover tracked the full geometry")
        else:
            line(f"   {g['dropped_total']} vertex/vertices removed by conditioning:")
            for r in g["runs"]:
                for d in r["dropped"]:
                    tag = "INTENT — should have been driven" if d["intent"] == "INTENT" else "within survey noise"
                    line(f"     - idx {d['path_index']:>3} ({d['n']:+.3f}N,{d['e']:+.3f}E)  "
                         f"bend {d['bend_deg']:>5.2f}deg  {d['deviation_cm']:>5.2f} cm off the "
                         f"driven path  [{tag}]")
                    if d.get("rover_closest_cm") is not None:
                        line(f"       rover closest approach {d['rover_closest_cm']} cm   "
                             f"nearest tracked node {d['nearest_tracked_node_cm']} cm away")
        line(f"   verdict : {g['verdict']}")
    line("")

    ab = a.get("absolute") or {}
    line("8. ABSOLUTE ACCURACY (rover's own lat/lon vs the SURVEYED lat/lon)")
    line("   asks the one question §1 and §7 structurally cannot: is the shape in the")
    line("   RIGHT PLACE ON EARTH? Both of those live in the local frame, so a wrong")
    line("   anchor moves the whole mission and they still report centimetres.")
    if not ab.get("available"):
        line(f"   unavailable — {ab.get('reason')}")
        if ab.get("provenance"):
            line(f"   ground truth : {ab['provenance']}")
    else:
        line(f"   ground truth : {ab['provenance']}  ({ab['n_truth']} surveyed point(s))")
        line(f"   timed against: {ab['target_kind']}")
        for r in ab["rows"]:
            line(f"     - pt {r['i'] + 1}: miss {r['miss_cm']:>6.2f} cm "
                 f"(N {r['dn_cm']:+6.2f}, E {r['de_cm']:+6.2f})   "
                 f"local approach {r['local_approach_cm']:>5.2f} cm, "
                 f"fix skew {r['skew_ms']:.0f} ms")
        line(f"   mean miss {ab['mean_cm']:.2f} cm   max {ab['max_cm']:.2f} cm")
        line(f"   systematic bias {ab['bias_cm']:.2f} cm "
             f"(N {ab['bias_n_cm']:+.2f}, E {ab['bias_e_cm']:+.2f})   "
             f"scatter about it {ab['scatter_cm']:.2f} cm")
        line("     ^ large bias + small scatter = PLACEMENT (the whole shape is shifted).")
        line("       small bias + large scatter = tracking/localisation noise.")
        for n in ab.get("notes", []):
            line(f"   NOTE: {n}")
        line(f"   verdict : {ab['verdict']}")
    line("")

    rc = a.get("recording") or {}
    if rc.get("truncated"):
        line("!" * 72)
        line(f"!! RECORDING TRUNCATED — first pose already moving at "
             f"{rc.get('first_pose_speed_mps')} m/s")
        if rc.get("first_pose_nearest_path_index") is not None:
            line(f"!! bag opens at path index {rc['first_pose_nearest_path_index']}"
                 f"/{rc.get('path_points', '?')} — the mission opening was NOT recorded")
        line("!! every section in this report describes the RECORDING, not the")
        line("!! mission. Do not root-cause geometry that is merely unrecorded.")
        line("!" * 72)
    tv = a.get("traversal") or {}
    line("9. TRAVERSAL (did the rover reach the whole path, or stop part way?)")
    line("   every section above describes only the part that WAS driven — an abort")
    line("   just yields fewer samples, and biases the error budget low.")
    if not tv.get("available"):
        line(f"   unavailable — {tv.get('reason')}")
    else:
        line(f"   reached {tv['points_covered']}/{tv['points_total']} path points "
             f"({tv['coverage']:.1%}) within {tv['radius_cm']} cm")
        if tv["status"] != "COMPLETE":
            _shape = {
                "STOPPED_EARLY": f"STOPPED EARLY — drove indices 0..{tv['last_covered_index']} "
                                 f"of {tv['points_total'] - 1}, then never resumed",
                "STARTED_LATE": f"NEVER DROVE THE START — first {tv['missing_leading']} "
                                f"point(s) unreached; ran from index "
                                f"{tv['first_covered_index']} to the end",
                "MIDDLE_ONLY": f"drove only the middle — missed {tv['missing_leading']} "
                               f"at the start and {tv['missing_trailing']} at the end",
                "INTERIOR_GAP": f"SKIPPED {tv['missing_interior']} point(s) mid-path and "
                                f"came back — not a clean abort",
                "NONE": "the rover never came within range of ANY path point",
            }.get(tv["shape"], tv["shape"])
            line(f"   {_shape}")
            line("   ! manifest.outcome.status does NOT encode this: the recorder writes")
            line("     COMPLETE whenever it shut down cleanly, abort or not.")
        line(f"   verdict : {tv['verdict']}  ({tv['status']})")
    line("")

    cfg = a["config"]
    line("10. AS-RUN CONFIG")
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

    line("11. VERDICT")
    line(f"   ===> {a['verdict']} <===")
    if a["worst_offenders"]:
        line("   worst offenders:")
        for w in a["worst_offenders"]:
            line(f"     - {w}")
    line("")

    geo = a.get("geo") or {}
    line("12. GEO OVERLAY (surveyed vs commanded /path vs driven — all in lat/lon)")
    if not geo.get("available"):
        line(f"   unavailable — {geo.get('reason', 'no geo layers')}")
    else:
        o = geo.get("ekf_origin")
        line(f"   EKF origin : {o[0]:.7f}, {o[1]:.7f}" if o
             else "   EKF origin : (none — /path could not be geo-referenced)")
        line(f"   layers     : surveyed {geo['n_surveyed']}, commanded {geo['n_commanded']}, "
             f"driven {geo['n_driven']}")
        if geo.get("placement_mean_cm") is not None:
            line(f"   placement  : commanded-geo vs surveyed-geo   "
                 f"mean {geo['placement_mean_cm']:.2f} cm   max {geo['placement_max_cm']:.2f} cm")
            line("     ^ placement + projection only (§8 = surveyed vs DRIVEN = this + tracking).")
        if geo.get("commanded_error"):
            line(f"   NOTE: commanded layer skipped — {geo['commanded_error']}")
        line(f"   files      : {', '.join(geo.get('files', []))}"
             "   (open in geojson.io / Google Earth / QGIS)")
    line("=" * 72)
    return "\n".join(L) + "\n"


# ── §12 GEO OVERLAY — surveyed vs commanded /path vs driven, all in lat/lon ────
def _import_ned_to_latlon():
    import sys as _sys
    from pathlib import Path as _Path
    root = _Path(__file__).resolve().parents[1]
    if str(root) not in _sys.path:
        _sys.path.insert(0, str(root))
    from path_engine.ned import ned_to_latlon
    return ned_to_latlon


def _downsample(pts, cap=2000):
    if len(pts) <= cap:
        return list(pts)
    step = len(pts) / cap
    return [pts[int(i * step)] for i in range(cap)]


def _geo_feature(name, pts, geom, color):
    coords = [[lon, lat] for (lat, lon) in pts]          # GeoJSON is [lon, lat]
    g = ({"type": "LineString", "coordinates": coords} if geom == "line"
         else {"type": "MultiPoint", "coordinates": coords})
    return {"type": "Feature",
            "properties": {"layer": name, "count": len(pts),
                           "stroke": color, "marker-color": color},
            "geometry": g}


def analyze_geo(s: Series, manifest, out_dir: str) -> dict:
    """Render the mission into GEO coordinates and write a map overlay (§12).

    Three lat/lon layers → geo_overlay.geojson + geo_overlay.csv:
      * surveyed  — the intent, straight from the source file (independent).
      * commanded — the local /path converted back to lat/lon via the EKF origin.
      * driven    — the rover's own /mavros/global_position/global trace.

    Also a PLACEMENT-only miss (commanded-geo vs surveyed-geo at the must-hit
    vertices): isolates placement + projection residual, the half §8 folds into
    the total (§8 = surveyed vs DRIVEN = placement + tracking).
    """
    out = {"available": False}
    truth, provenance = _surveyed_latlon_from_source(manifest)
    origin = s.ekf_origin
    driven = [(lat, lon) for (_t, lat, lon, _a) in s.global_fix]

    commanded, commanded_musthit = [], []
    if origin and s.path:
        try:
            n2ll = _import_ned_to_latlon()
            commanded = [n2ll(n, e, origin[0], origin[1]) for (n, e) in s.path]
            if s.path_z and len(s.path_z) == len(s.path):
                commanded_musthit = [ll for ll, z in zip(commanded, s.path_z) if z & 2]
        except Exception as exc:                     # geographiclib missing etc.
            out["commanded_error"] = f"{type(exc).__name__}: {exc}"

    out["ekf_origin"] = list(origin) if origin else None
    out["provenance"] = provenance
    out["n_surveyed"], out["n_commanded"], out["n_driven"] = \
        len(truth), len(commanded), len(driven)
    if not origin:
        out["reason"] = ("no EKF origin in bag (gp_origin) — cannot geo-reference "
                         "/path; driven + surveyed layers still exported")
    if not (truth or commanded or driven):
        out["reason"] = "nothing to export (no surveyed truth, /path, or global fix)"
        return out

    # placement-only miss: commanded must-hit vertices vs surveyed truth
    tgt = commanded_musthit or commanded
    if truth and tgt and len(truth) == len(tgt):
        rows = [{"i": i, "miss_cm": _geodesic_m(a, b, c, d) * 100.0}
                for i, ((a, b), (c, d)) in enumerate(zip(truth, tgt))]
        out["placement_rows"] = rows
        out["placement_mean_cm"] = round(sum(r["miss_cm"] for r in rows) / len(rows), 2)
        out["placement_max_cm"] = round(max(r["miss_cm"] for r in rows), 2)

    features = []
    if truth:
        features.append(_geo_feature("surveyed", truth, "points", "#2ca02c"))
    if commanded:
        features.append(_geo_feature("commanded_path", _downsample(commanded), "line", "#1f77b4"))
    if driven:
        features.append(_geo_feature("driven", _downsample(driven), "line", "#d62728"))

    files = []
    try:
        gj = os.path.join(out_dir, "geo_overlay.geojson")
        with open(gj, "w") as f:
            json.dump({"type": "FeatureCollection", "features": features}, f)
        files.append(os.path.basename(gj))
        cf = os.path.join(out_dir, "geo_overlay.csv")
        with open(cf, "w") as f:
            f.write("layer,index,lat,lon\n")
            for name, pts in (("surveyed", truth),
                              ("commanded_path", _downsample(commanded)),
                              ("driven", _downsample(driven))):
                for i, (lat, lon) in enumerate(pts):
                    f.write(f"{name},{i},{lat:.8f},{lon:.8f}\n")
        files.append(os.path.basename(cf))
    except OSError as exc:
        out["write_error"] = str(exc)

    out["available"] = bool(files)
    out["files"] = files
    return out


def analyze(root: str, survey_tol_cm: float | None = None,
            out_dir: str | None = None) -> dict:
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
    tol_cm, tol_source = resolve_survey_tol_cm(manifest, survey_tol_cm)
    geometry = analyze_geometry_fidelity(s, survey_tol_cm=tol_cm,
                                         survey_tol_source=tol_source)
    absolute = analyze_absolute(s, manifest)
    traversal = analyze_traversal(s)
    recording = analyze_recording_integrity(s)
    config = analyze_config(s, manifest)
    geo = analyze_geo(s, manifest, out_dir or root)

    # overall verdict + worst offenders
    offenders = []
    fails = []
    if recording.get("truncated"):
        offenders.append(
            f"RECORDING TRUNCATED — first pose already moving at "
            f"{recording['first_pose_speed_mps']} m/s (rest reads ≤~0.02): the bag "
            f"missed the mission opening. Coverage/traversal below describe the "
            f"RECORDING, not the mission — do not root-cause the missing span")
    for name, sec in (("tracking", tracking), ("stops", stops), ("pivots", pivots),
                      ("geometry", geometry), ("absolute", absolute),
                      ("traversal", traversal)):
        v = sec.get("verdict")
        if v == "FAIL":
            fails.append(name)
    if traversal.get("available") and traversal.get("status") != "COMPLETE":
        _end = {
            "STOPPED_EARLY": "stopped early and never resumed",
            "STARTED_LATE": "never drove the start of the path",
            "MIDDLE_ONLY": "drove only the middle of the path",
            "INTERIOR_GAP": "skipped geometry mid-path and came back",
            "NONE": "never came within range of the path at all",
        }.get(traversal["shape"], traversal["shape"])
        offenders.append(
            f"only {traversal['points_covered']}/{traversal['points_total']} path points "
            f"reached ({traversal['coverage']:.0%}) — {_end}. Every other metric here "
            f"describes only the part that was driven"
        )
    if geometry.get("available") and geometry.get("dropped_above_tolerance"):
        offenders.append(
            f"{geometry['dropped_above_tolerance']} surveyed vertex/vertices dropped by path "
            f"conditioning (worst {geometry['worst_deviation_cm']}cm off the driven path, "
            f"tolerance {geometry['survey_tol_cm']}cm, {geometry['survey_tol_source']}) — "
            f"the rover did not drive the "
            f"surveyed shape"
        )
    # B7: grade the painted span when a spray signal exists — the overall block
    # is diluted by pivot/idle placeholders and can PASS a bad marking run.
    _xt_block = tracking.get("marking_only") or tracking.get("overall")
    _xt_label = "marking" if tracking.get("marking_only") else "overall"
    if _xt_block and _xt_block["rms_cm"] > XTRACK_PROD_CM:
        offenders.append(
            f"tracking RMS ({_xt_label}) {_xt_block['rms_cm']}cm > {XTRACK_PROD_CM}cm")
    # An unmeasured stop is an offender in its own right — silence here is what
    # let a 54.6 cm overshoot grade PASS on 2026-08-01.
    if stops.get("available") and not stops.get("stops"):
        offenders.append("NO stop measured — endpoint accuracy is UNKNOWN, not good")
    elif stops.get("available") and stops.get("unmeasured"):
        offenders.append(f"{len(stops['unmeasured'])} stop(s) not measured")
    if stops.get("available") and stops["worst_coast_cm"] > COAST_MAX_CM:
        offenders.append(f"coast-past {stops['worst_coast_cm']}cm > {COAST_MAX_CM}cm")
    if stops.get("available") and (stops.get("endpoint_resting_cm") or 0) > FINAL_STOP_MAX_CM:
        offenders.append(
            f"endpoint resting {stops['endpoint_resting_cm']}cm > {FINAL_STOP_MAX_CM}cm")
    if pivots.get("available") and pivots.get("any_reverse_flip"):
        offenders.append("reverse-flip detected during a pivot")
    if pivots.get("available") and (pivots.get("worst_settle_deg") or 0) > SETTLE_TOL_DEG:
        offenders.append(f"pivot settle {pivots['worst_settle_deg']}° > {SETTLE_TOL_DEG}°")
    if pivots.get("available") and pivots.get("unreleased_pivots"):
        offenders.append(
            "pivot never released (no S_TRACK after ALIGN — segment_debug went "
            "silent; settle unmeasurable)")
    if health.get("offboard_drops"):
        offenders.append(f"{health['offboard_drops']} OFFBOARD drop(s)")
    if health.get("verdict") == "UNAVAILABLE":
        offenders.append(
            "health UNAVAILABLE — zero /mavros/state samples decoded; OFFBOARD "
            "continuity is unverified, not verified-clean")
    if absolute.get("available") and absolute.get("bias_cm", 0) > ABS_BIAS_FAIL_CM:
        offenders.append(
            f"the whole mission sits {absolute['bias_cm']:.1f}cm off its surveyed position "
            f"(scatter only {absolute['scatter_cm']:.1f}cm) — a PLACEMENT error, which no "
            f"local-frame metric can see")
    elif absolute.get("available") and absolute.get("max_cm", 0) > ABS_MISS_FAIL_CM:
        offenders.append(
            f"worst absolute miss {absolute['max_cm']:.1f}cm vs the surveyed lat/lon")
    verdict = "FAIL" if fails else ("WARN" if (offenders or health.get("verdict") != "PASS") else "PASS")

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
        "geometry": geometry,
        "absolute": absolute,
        "traversal": traversal,
        "recording": recording,
        "config": config,
        "geo": geo,
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
    ap.add_argument("--survey-tol-cm", type=float, default=None,
                    help="vertex deviation above which a dropped point counts as "
                         "INTENT rather than survey noise. Overrides the value staged "
                         f"with the mission; default {SURVEY_TOL_CM} cm. Set this from "
                         "the survey's own lateral RMS, not by taste.")
    args = ap.parse_args()
    if args.survey_tol_cm is not None and not (0.0 < args.survey_tol_cm <= 100.0):
        sys.exit(f"ERROR: --survey-tol-cm must be in (0, 100]; got {args.survey_tol_cm}")
    if not os.path.isdir(args.bundle):
        sys.exit(f"ERROR: not a directory: {args.bundle}")

    result = analyze(args.bundle, survey_tol_cm=args.survey_tol_cm,
                     out_dir=(args.outdir or args.bundle))

    # A4: fold the traversal verdict back next to outcome, so a partial run is
    # detectable by reading manifest.json alone — no bag, no analysis.json.
    tv = result.get("traversal") or {}
    if tv.get("available"):
        _rc = result.get("recording") or {}
        written = _write_traversal_to_manifest(args.bundle, {
            "status": ("TRUNCATED_RECORDING" if _rc.get("truncated") else tv["status"]),
            "recording_truncated": bool(_rc.get("truncated")),
            "shape": tv["shape"],
            "coverage": tv["coverage"],
            "points_covered": tv["points_covered"],
            "points_total": tv["points_total"],
            "radius_cm": tv["radius_cm"],
            "first_covered_index": tv["first_covered_index"],
            "last_covered_index": tv["last_covered_index"],
            "source": "analyze_mission",
        })
        if written and written.startswith("ERROR"):
            print(f"WARN: could not update manifest traversal: {written}",
                  file=sys.stderr)

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
