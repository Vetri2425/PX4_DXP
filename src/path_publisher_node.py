#!/usr/bin/env python3
"""Path publisher — emits test paths in LOCAL_NED for SITL/hardware bring-up.

Pipeline position:
  [THIS NODE] → /path → rpp_controller_node → /rpp/velocity_ned → twist_to_setpoint_node

Input modes
-----------
  1. Hardcoded paths (path_name parameter) — for SITL and quick testing
  2. QGC .waypoints file (mission_file parameter) — lat/lon converted to NED
  3. CSV/DXF file (mission_file parameter) — planned through path_engine when needed

Hardcoded paths
---------------
  straight_5m       — 5 m straight north, 50 cm point spacing
  arc_quarter_1m5   — quarter circle, R=1.5 m, north then east
  lshape_2x2        — 2 m north then 2 m east (90° corner)
  square_2x2        — 2 m × 2 m square, 4 corners, closed loop
  rectangle_3x2     — 3 m north × 2 m east rectangle
  circle_1m5        — full circle, R=1.5 m, closed loop

QGC .waypoints file format
---------------------------
  Standard QGC WPL 110 format with WGS84 lat/lon columns.
  The home waypoint (current=1) is used as the NED origin.
  All other waypoints are converted to metres North/East from home
  using Karney geodesic (geographiclib, same method as arc generators).

  Requires: pip install geographiclib

Simple CSV format
-----------------
  Two-column CSV with no header:
    north_m,east_m
    0.0,0.0
    1.0,0.0
    1.0,1.0
    ...

Frame
-----
  All paths published in LOCAL_NED (x=North, y=East, z=Down=0).
  header.frame_id = "local_ned" — must match rpp_controller's path_frame_id param.

Usage
-----
  # Hardcoded path:
  ros2 run ... path_publisher --ros-args -p path_name:=square_2x2

  # QGC waypoints file (lat/lon → NED via Karney):
  ros2 run ... path_publisher --ros-args -p mission_file:=/path/to/mission.waypoints

  # Simple CSV (NED metres, no conversion) or DXF (sidecars honored):
  ros2 run ... path_publisher --ros-args -p mission_file:=/path/to/path.csv
"""

import csv
import json
import logging
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32

try:
    from path_engine import PathEngine as _PathEngine
    from path_engine.validator import PathValidator as _PathValidator
    _HAS_PATH_ENGINE = True
except ImportError:
    _PathEngine = None  # type: ignore[assignment,misc]
    _PathValidator = None  # type: ignore[assignment,misc]
    _HAS_PATH_ENGINE = False

_logger = logging.getLogger("path_publisher")


def _sidecar_path(filepath: str, suffix: str) -> str:
    dirname = os.path.dirname(filepath)
    basename = os.path.basename(filepath)
    return os.path.join(dirname, f".{basename}.{suffix}.json")


def _load_extension_sidecar(filepath: str) -> tuple[bool, float, float]:
    default = (False, 0.5, 0.5)
    try:
        with open(_sidecar_path(filepath, "extensions"), encoding="utf-8") as f:
            payload = json.load(f)
        pre = float(payload.get("pre_extension_m", default[1]))
        aft = float(payload.get("aft_extension_m", default[2]))
        if pre < 0.0 or aft < 0.0:
            return default
        return (bool(payload.get("enabled", default[0])), pre, aft)
    except (FileNotFoundError, OSError, ValueError, TypeError):
        return default


def _load_entity_overrides_sidecar(filepath: str) -> dict[str, bool]:
    try:
        with open(_sidecar_path(filepath, "entities"), encoding="utf-8") as f:
            payload = json.load(f)
    except (FileNotFoundError, OSError, ValueError):
        return {}

    raw = payload.get("overrides", {})
    if not isinstance(raw, dict):
        return {}
    result: dict[str, bool] = {}
    for entity_id, value in raw.items():
        if isinstance(value, dict):
            value = value.get("is_mark")
        if isinstance(value, bool):
            result[str(entity_id)] = value
    return result


def _load_entity_order_sidecar(filepath: str) -> list[str]:
    try:
        with open(_sidecar_path(filepath, "entity_order"), encoding="utf-8") as f:
            payload = json.load(f)
    except (FileNotFoundError, OSError, ValueError):
        return []
    raw = payload.get("entity_order", [])
    if not isinstance(raw, list):
        return []
    return [str(v) for v in raw if isinstance(v, str)]


def _plan_file_with_engine(
    filepath: str,
    *,
    origin: tuple[float, float] = (0.0, 0.0),
    start_position: tuple[float, float] | None = None,
):
    """Plan a mission file through PathEngine, honoring DXF sidecars when present."""
    if _PathEngine is None:
        raise ImportError("path_engine is required for PathEngine planning")

    ext = os.path.splitext(filepath)[1].lower()
    if ext != ".dxf":
        return _PathEngine().plan_file(
            filepath,
            origin=origin,
            start_position=start_position,
        )

    enabled, pre_m, aft_m = _load_extension_sidecar(filepath)
    overrides = _load_entity_overrides_sidecar(filepath)
    saved_order = _load_entity_order_sidecar(filepath)
    engine = _PathEngine(
        enable_path_extensions=enabled,
        pre_extension_m=pre_m,
        aft_extension_m=aft_m,
        optimize_order=not bool(saved_order),
    )

    if overrides or saved_order:
        from path_engine.entity_order import apply_entity_order
        from path_engine.parsers.dxf_parser import parse_dxf

        entities = parse_dxf(filepath)
        for ent in entities:
            if ent.entity_id in overrides:
                ent.is_mark_override = bool(overrides[ent.entity_id])
        if saved_order:
            entities = apply_entity_order(entities, saved_order)
        return engine.plan_dxf_entities(
            entities,
            origin=origin,
            start_position=start_position,
        )

    return engine.plan_file(
        filepath,
        origin=origin,
        start_position=start_position,
    )


# ---------------------------------------------------------------------------
# Path generators (hardcoded shapes for SITL)
# ---------------------------------------------------------------------------
def gen_straight_5m(spacing: float = 0.1) -> list[tuple[float, float]]:
    """5 m straight north, points at `spacing` intervals."""
    n_steps = int(5.0 / spacing) + 1
    return [(i * spacing, 0.0) for i in range(n_steps)]


def gen_straight_3m(spacing: float = 0.1) -> list[tuple[float, float]]:
    """3 m straight north, points at `spacing` intervals."""
    n_steps = int(3.0 / spacing) + 1
    return [(i * spacing, 0.0) for i in range(n_steps)]


def gen_arc_quarter_1m5(radius: float = 1.5, arc_spacing: float = 0.05) \
        -> list[tuple[float, float]]:
    """Quarter circle, radius 1.5 m. Starts heading north at origin,
    sweeps to the east (right turn). Centre of circle is at (0, +R)."""
    arc_len = radius * (math.pi / 2.0)
    n_steps = max(2, int(arc_len / arc_spacing) + 1)
    pts = []
    for i in range(n_steps):
        theta = (math.pi / 2.0) * (i / (n_steps - 1))
        n = radius * math.sin(theta)
        e = radius * (1.0 - math.cos(theta))
        pts.append((n, e))
    return pts


def gen_arc_half_1m5(radius: float = 1.5, arc_spacing: float = 0.05) \
        -> list[tuple[float, float]]:
    """Half circle, radius 1.5 m. Starts heading north at origin,
    sweeps to the east (right turn). Centre of circle is at (0, +R)."""
    arc_len = radius * math.pi
    n_steps = max(2, int(arc_len / arc_spacing) + 1)
    pts = []
    for i in range(n_steps):
        theta = math.pi * (i / (n_steps - 1))
        n = radius * math.sin(theta)
        e = radius * (1.0 - math.cos(theta))
        pts.append((n, e))
    return pts


def gen_lshape_2x2(spacing: float = 0.15) -> list[tuple[float, float]]:
    """2 m north, then 2 m east. Sharp 90° corner."""
    pts = []
    n_steps_1 = int(2.0 / spacing) + 1
    for i in range(n_steps_1):
        pts.append((i * spacing, 0.0))
    n_steps_2 = int(2.0 / spacing)
    for i in range(1, n_steps_2 + 1):
        pts.append((2.0, i * spacing))
    return pts


def gen_square_2x2(spacing: float = 0.15) -> list[tuple[float, float]]:
    """2 m × 2 m square starting at origin, clockwise, closed loop."""
    side = 2.0
    pts = []
    n_steps = int(side / spacing) + 1
    for i in range(n_steps):
        pts.append((i * spacing, 0.0))
    n_steps = int(side / spacing)
    for i in range(1, n_steps + 1):
        pts.append((side, i * spacing))
    for i in range(1, n_steps + 1):
        pts.append((side - i * spacing, side))
    for i in range(1, n_steps + 1):
        pts.append((0.0, side - i * spacing))
    return pts


def gen_rectangle_3x2(spacing: float = 0.15) -> list[tuple[float, float]]:
    """3 m north × 2 m east rectangle, clockwise."""
    len_n, len_e = 3.0, 2.0
    pts = []
    n_steps = int(len_n / spacing) + 1
    for i in range(n_steps):
        pts.append((i * spacing, 0.0))
    n_steps = int(len_e / spacing)
    for i in range(1, n_steps + 1):
        pts.append((len_n, i * spacing))
    n_steps = int(len_n / spacing)
    for i in range(1, n_steps + 1):
        pts.append((len_n - i * spacing, len_e))
    n_steps = int(len_e / spacing)
    for i in range(1, n_steps + 1):
        pts.append((0.0, len_e - i * spacing))
    return pts


def gen_circle_1m5(radius: float = 1.5, arc_spacing: float = 0.05) \
        -> list[tuple[float, float]]:
    """Full circle, radius 1.5 m, starts north at origin, closed loop."""
    circ_len = radius * 2.0 * math.pi
    n_steps = max(4, int(circ_len / arc_spacing) + 1)
    pts = []
    for i in range(n_steps):
        theta = (2.0 * math.pi) * (i / n_steps)
        n = radius * math.sin(theta)
        e = radius * (1.0 - math.cos(theta))
        pts.append((n, e))
    pts.append((0.0, 0.0))
    return pts


PATH_GENERATORS = {
    "straight_5m":     gen_straight_5m,
    "straight_3m":     gen_straight_3m,
    "arc_quarter_1m5": gen_arc_quarter_1m5,
    "arc_half_1m5":    gen_arc_half_1m5,
    "lshape_2x2":      gen_lshape_2x2,
    "square_2x2":      gen_square_2x2,
    "rectangle_3x2":   gen_rectangle_3x2,
    "circle_1m5":      gen_circle_1m5,
}


# ---------------------------------------------------------------------------
# File readers
# ---------------------------------------------------------------------------
def read_qgc_waypoints(filepath: str) -> list[tuple[float, float]]:
    """Read QGC WPL 110 .waypoints file and convert lat/lon to NED metres.

    Uses the home waypoint (current=1) as the NED origin.
    All mission waypoints converted to metres North/East from home
    using Karney geodesic on WGS84 ellipsoid.
    """
    try:
        from geographiclib.geodesic import Geodesic
    except ImportError:
        raise ImportError(
            "geographiclib is required for QGC .waypoints files. "
            "Install: pip install geographiclib"
        )

    geod = Geodesic.WGS84
    wps = []  # (lat, lon) pairs, skipping home
    home_lat = home_lon = None

    with open(filepath, "r") as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("QGC"):
                continue
            fields = line.split("\t")
            if len(fields) < 11:
                continue

            try:
                current = int(fields[1])
                lat = float(fields[8])
                lon = float(fields[9])
            except (ValueError, IndexError):
                continue

            if current == 1:
                # Home waypoint — becomes NED origin
                home_lat, home_lon = lat, lon
            else:
                wps.append((lat, lon))

    if home_lat is None:
        # No explicit home — use first waypoint as origin
        if wps:
            home_lat, home_lon = wps[0]
            wps = wps[1:]
        else:
            raise ValueError(f"No waypoints found in {filepath}")

    # Convert each lat/lon to NED metres from home using Karney geodesic
    pts = []
    for lat, lon in wps:
        # Bearing from home to waypoint
        result = geod.Inverse(home_lat, home_lon, lat, lon)
        dist = result["s12"]  # metres
        bearing_rad = math.radians(result["azi1"])
        north = dist * math.cos(bearing_rad)
        east = dist * math.sin(bearing_rad)
        pts.append((north, east))

    return pts


def read_ned_csv(filepath: str) -> list[tuple[float, float]]:
    """Read simple CSV with north_m,east_m columns (no header).

    Lines starting with # are ignored.
    """
    pts = []
    with open(filepath, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            # Skip empty or comment lines
            if not row or row[0].strip().startswith("#"):
                continue
            try:
                n = float(row[0].strip())
                e = float(row[1].strip()) if len(row) > 1 else 0.0
                pts.append((n, e))
            except ValueError:
                continue
    return pts


def load_mission_file(filepath: str, start_position: tuple[float, float] | None = None) -> list[tuple[float, float]]:
    """Auto-detect file format and load waypoints.

    .waypoints → QGC WPL 110 (lat/lon → NED via Karney)
    .csv       → simple NED metres (north, east) or enhanced 6-col
    .dxf       → DXF CAD file (via path_engine, with start_position for TSP)
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"Mission file not found: {filepath}")

    ext = os.path.splitext(filepath)[1].lower()
    if ext == ".waypoints":
        return read_qgc_waypoints(filepath)
    elif ext == ".csv":
        return read_ned_csv(filepath)
    elif ext == ".dxf":
        if _PathEngine is None:
            raise ImportError("path_engine is required for .dxf files")
        plan = _plan_file_with_engine(filepath, start_position=start_position)
        return plan.merged_waypoints
    else:
        # Try QGC format first, fall back to CSV
        try:
            return read_qgc_waypoints(filepath)
        except Exception:
            return read_ned_csv(filepath)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class PathPublisherNode(Node):
    """Publishes a path on /path from hardcoded shapes or mission files."""

    def __init__(self):
        super().__init__("path_publisher")

        self.declare_parameter("path_name", "straight_5m")
        self.declare_parameter("mission_file", "")  # empty = use path_name
        self.declare_parameter("frame_id", "local_ned")
        self.declare_parameter("publish_delay_s", 1.0)
        self.declare_parameter("max_waypoints", 10000)
        # auto_origin: offset path to start at rover's current EKF position.
        # Waits for /mavros/local_position/pose before publishing.
        self.declare_parameter("auto_origin", False)

        # TRANSIENT_LOCAL so late-joining subscribers (rpp_controller) get it
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        be_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(Path, "/path", path_qos)

        # /dyx/mission/progress — 0.0→1.0 completion (1Hz)
        self._progress_pub = self.create_publisher(Float32, "/dyx/mission/progress", be_qos)

        self._auto_origin = self.get_parameter("auto_origin").value
        self._origin_ned: tuple[float, float] | None = None  # (North, East)
        self._start_position: tuple[float, float] | None = None  # For TSP optimization

        # Progress tracking state. Spray state is encoded in /path.pose.position.z
        # and consumed by rpp_controller -> /spray/active.
        self._spray_flags: list[bool] | None = None
        self._path_pts: list[tuple[float, float]] = []  # offset pts for pose lookup
        self._total_waypoints: int = 0
        self._waypoints_visited: int = 0
        self._mission_active: bool = False

        # Subscribe to pose for auto-origin and progress tracking.
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self._pose_cb, be_qos)

        if self._auto_origin:
            # Timer checks if origin is ready, then publishes
            self._timer = self.create_timer(0.2, self._try_publish_with_origin)
        else:
            # One-shot timer to publish after a brief settle
            delay = self.get_parameter("publish_delay_s").value
            self._timer = self.create_timer(delay, self._publish_once)

        mission_file = self.get_parameter("mission_file").value
        path_name = self.get_parameter("path_name").value
        source = f"file={mission_file}" if mission_file else f"path_name={path_name}"
        mode = "auto_origin" if self._auto_origin else "origin_at_zero"
        self.get_logger().info(
            f"path_publisher started — will publish {source} after "
            f"{'pose received' if self._auto_origin else f'{delay:.1f}s'} "
            f"(mode={mode})"
        )

    def _pose_cb(self, msg: PoseStamped):
        """Capture current EKF position for auto-origin, TSP, spray, and progress."""
        # MAVROS pose is ENU: x=East, y=North → NED: N=y, E=x
        pn = msg.pose.position.y
        pe = msg.pose.position.x

        if self._auto_origin and self._origin_ned is None:
            ned = (pn, pe)
            self._origin_ned = ned
            self._start_position = ned
            self.get_logger().info(
                f"Auto-origin captured: N={ned[0]:.3f}, "
                f"E={ned[1]:.3f} (from ENU pose)"
            )

        # Pose-driven progress tracking
        if self._mission_active and self._path_pts:
            self._update_progress(pn, pe)

    def _try_publish_with_origin(self):
        """Called every 200ms when auto_origin=True. Publishes once origin is known."""
        if self._origin_ned is None:
            return  # Still waiting for pose
        self._timer.cancel()
        self._publish_once()

    def _publish_once(self):
        """Load path, apply offset if auto_origin, and publish."""
        if not self._auto_origin:
            self._timer.cancel()

        frame_id = self.get_parameter("frame_id").value
        mission_file = self.get_parameter("mission_file").value
        path_name = self.get_parameter("path_name").value

        # Load path points — pass origin/start_position for TSP if using path_engine
        spray_flags: list[bool] | None = None
        pts: list[tuple[float, float]] = []
        origin_applied_by_engine = False

        if mission_file:
            try:
                ext = os.path.splitext(mission_file)[1].lower()
                if ext in (".dxf", ".csv"):
                    # Use path_engine for DXF and enhanced CSV
                    if _PathEngine is None:
                        self.get_logger().error(
                            "path_engine not installed — refusing .dxf mission"
                        )
                        return
                    origin = (
                        self._origin_ned
                        if self._auto_origin and self._origin_ned
                        else (0.0, 0.0)
                    )
                    plan = _plan_file_with_engine(
                        mission_file,
                        origin=origin,
                        start_position=self._start_position,
                    )
                    if _PathValidator is not None:
                        warnings = _PathValidator(
                            max_waypoints=int(self.get_parameter("max_waypoints").value)
                        ).validate_or_raise(plan)
                        for warning in warnings:
                            self.get_logger().warning(f"Path validation warning: {warning}")
                    pts = plan.merged_waypoints
                    spray_flags = plan.spray_flags
                    origin_applied_by_engine = origin != (0.0, 0.0)
                    source = mission_file
                else:
                    pts = load_mission_file(mission_file, start_position=self._start_position)
                    source = mission_file
            except Exception as e:
                self.get_logger().error(f"Failed to load mission file: {e}")
                return
        elif path_name in PATH_GENERATORS:
            pts = PATH_GENERATORS[path_name]()
            source = path_name
        else:
            self.get_logger().error(
                f"Unknown path_name {path_name!r}. "
                f"Available: {list(PATH_GENERATORS.keys())}"
            )
            return

        if not pts:
            self.get_logger().error("No waypoints loaded — nothing to publish")
            return

        max_waypoints = int(self.get_parameter("max_waypoints").value)
        if len(pts) > max_waypoints:
            self.get_logger().error(
                f"Refusing to publish {len(pts)} waypoints; max_waypoints={max_waypoints}. "
                "Increase spacing, fix units, or simplify the mission."
            )
            return

        # Apply auto-origin offset if enabled
        if self._auto_origin and self._origin_ned is not None and not origin_applied_by_engine:
            offset_n, offset_e = self._origin_ned
            pts = [(n + offset_n, e + offset_e) for (n, e) in pts]
            self.get_logger().info(
                f"Auto-origin offset applied: +{offset_n:.3f}N, +{offset_e:.3f}E"
            )

        # Build Path message
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        if spray_flags is None or len(spray_flags) != len(pts):
            path_spray_flags = [True] * len(pts)
        else:
            path_spray_flags = [bool(f) for f in spray_flags]

        for (n, e), spray in zip(pts, path_spray_flags):
            ps = PoseStamped()
            ps.header.stamp = path.header.stamp
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(n)
            ps.pose.position.y = float(e)
            ps.pose.position.z = 1.0 if spray else 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self._pub.publish(path)

        # Store offset waypoints for pose-driven spray+progress tracking
        self._path_pts = list(pts)
        self._total_waypoints = len(pts)
        self._waypoints_visited = 0
        self._spray_flags = path_spray_flags
        self._mission_active = True

        if path_spray_flags:
            self.get_logger().info(
                f"Spray schedule loaded: {sum(path_spray_flags)} MARK / "
                f"{len(path_spray_flags) - sum(path_spray_flags)} TRANSIT waypoints"
            )

        # Start 1Hz progress timer
        self._progress_timer = self.create_timer(1.0, self._progress_cb)

        self.get_logger().info(
            f"Published {source!r}: {len(path.poses)} waypoints "
            f"first=({pts[0][0]:.3f}N,{pts[0][1]:.3f}E) "
            f"last=({pts[-1][0]:.3f}N,{pts[-1][1]:.3f}E) "
            f"frame={frame_id!r}"
        )

    def _update_progress(self, pn: float, pe: float):
        """Project pose onto path segments and update progress.

        I2 fix: previously this used nearest-WAYPOINT Euclidean distance, which
        matches RPP only for densely spaced paths. RPP projects the pose onto
        path SEGMENTS (closest point on each segment), so for sparse waypoints
        (0.5–2 m apart) the spray boundary could fire up to half the spacing
        early/late. We now use the same segment-projection scheme:
        continuous path index c = i + t (segment i, projection parameter t),
        and the spray flag of the last *crossed* waypoint floor(c) is applied.
        """
        # Search forward from last index in a small window (avoids backtracking)
        search_ahead = 50
        start_i = self._waypoints_visited
        end_i = min(start_i + search_ahead, self._total_waypoints - 1)

        best_c = float(start_i)   # continuous index = segment_idx + t
        best_d = float("inf")

        if self._total_waypoints == 1:
            best_c = 0.0
        else:
            for i in range(start_i, end_i):
                ax, ay = self._path_pts[i]
                bx, by = self._path_pts[i + 1]
                dx, dy = bx - ax, by - ay
                seg_len_sq = dx * dx + dy * dy
                if seg_len_sq < 1e-12:
                    # Zero-length segment (dwell point) — treat as a point
                    t = 0.0
                    cx, cy = ax, ay
                else:
                    t = ((pn - ax) * dx + (pe - ay) * dy) / seg_len_sq
                    t = max(0.0, min(1.0, t))
                    cx, cy = ax + t * dx, ay + t * dy
                d = (cx - pn) ** 2 + (cy - pe) ** 2
                if d < best_d:
                    best_d = d
                    best_c = i + t

        # Last crossed waypoint = floor(continuous index), matching where RPP
        # places the rover on the path rather than the nearest discrete waypoint.
        last_crossed = min(int(best_c), self._total_waypoints - 1)
        self._waypoints_visited = last_crossed

    def _progress_cb(self):
        """1Hz progress publisher — reports waypoint completion fraction."""
        progress = Float32()
        if self._total_waypoints > 0:
            progress.data = float(self._waypoints_visited) / float(self._total_waypoints)
        else:
            progress.data = 0.0
        self._progress_pub.publish(progress)


def main():
    rclpy.init()
    node = None
    try:
        node = PathPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
