#!/usr/bin/env python3
"""Path publisher — emits test paths in LOCAL_NED for SITL/hardware bring-up.

Pipeline position:
  [THIS NODE] → /path → rpp_controller_node → /rpp/velocity_ned → twist_to_setpoint_node

Input modes
-----------
  1. Hardcoded paths (path_name parameter) — for SITL and quick testing
  2. QGC .waypoints file (mission_file parameter) — lat/lon converted to NED
  3. Simple CSV file (mission_file parameter) — already in NED metres

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

  # Simple CSV (NED metres, no conversion):
  ros2 run ... path_publisher --ros-args -p mission_file:=/path/to/path.csv
"""

import csv
import logging
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32

try:
    from path_engine import PathEngine as _PathEngine
    _HAS_PATH_ENGINE = True
except ImportError:
    _PathEngine = None  # type: ignore[assignment,misc]
    _HAS_PATH_ENGINE = False

_logger = logging.getLogger("path_publisher")


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
        engine = _PathEngine()
        plan = engine.plan_file(filepath, start_position=start_position)
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

        # /dyx/spray_cmd — spray ON/OFF control (edge-triggered from spray_flags)
        spray_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._spray_pub = self.create_publisher(Bool, "/dyx/spray_cmd", spray_qos)

        # /dyx/mission/progress — 0.0→1.0 completion (1Hz)
        self._progress_pub = self.create_publisher(Float32, "/dyx/mission/progress", be_qos)

        self._auto_origin = self.get_parameter("auto_origin").value
        self._origin_ned: tuple[float, float] | None = None  # (North, East)
        self._start_position: tuple[float, float] | None = None  # For TSP optimization

        # Spray tracking state
        self._spray_flags: list[bool] | None = None
        self._path_pts: list[tuple[float, float]] = []  # offset pts for pose lookup
        self._total_waypoints: int = 0
        self._waypoints_visited: int = 0
        self._last_spray_state: bool | None = None
        self._mission_active: bool = False

        # Subscribe to pose for spray edge-detection + progress tracking
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

        if self._origin_ned is None:
            ned = (pn, pe)
            self._origin_ned = ned
            self._start_position = ned
            self.get_logger().info(
                f"Auto-origin captured: N={ned[0]:.3f}, "
                f"E={ned[1]:.3f} (from ENU pose)"
            )

        # Pose-driven spray edge-detection and progress tracking
        if self._mission_active and self._spray_flags and self._path_pts:
            self._update_spray_and_progress(pn, pe)

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

        # Load path points — pass start_position for TSP if using path_engine
        spray_flags: list[bool] | None = None
        pts: list[tuple[float, float]] = []

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
                    engine = _PathEngine()
                    plan = engine.plan_file(
                        mission_file, start_position=self._start_position,
                    )
                    pts = plan.merged_waypoints
                    spray_flags = plan.spray_flags
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

        # Apply auto-origin offset if enabled
        if self._origin_ned is not None:
            offset_n, offset_e = self._origin_ned
            pts = [(n + offset_n, e + offset_e) for (n, e) in pts]
            self.get_logger().info(
                f"Auto-origin offset applied: +{offset_n:.3f}N, +{offset_e:.3f}E"
            )

        # Build Path message
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        for (n, e) in pts:
            ps = PoseStamped()
            ps.header.stamp = path.header.stamp
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(n)
            ps.pose.position.y = float(e)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self._pub.publish(path)

        # Store offset waypoints for pose-driven spray+progress tracking
        self._path_pts = list(pts)
        self._total_waypoints = len(pts)
        self._waypoints_visited = 0
        self._spray_flags = spray_flags
        self._last_spray_state = None
        self._mission_active = True

        if spray_flags:
            # Publish initial spray command
            initial_spray = spray_flags[0]
            spray_msg = Bool()
            spray_msg.data = initial_spray
            self._spray_pub.publish(spray_msg)
            self._last_spray_state = initial_spray

            self.get_logger().info(
                f"Spray schedule loaded: {sum(spray_flags)} MARK / "
                f"{len(spray_flags) - sum(spray_flags)} TRANSIT waypoints"
            )

        # Start 1Hz progress timer
        self._progress_timer = self.create_timer(1.0, self._progress_cb)

        self.get_logger().info(
            f"Published {source!r}: {len(path.poses)} waypoints "
            f"first=({pts[0][0]:.3f}N,{pts[0][1]:.3f}E) "
            f"last=({pts[-1][0]:.3f}N,{pts[-1][1]:.3f}E) "
            f"frame={frame_id!r}"
        )

    def _update_spray_and_progress(self, pn: float, pe: float):
        """Find closest waypoint from current pose, edge-detect spray, update progress."""
        # Search forward from last index in a small window (avoids backtracking)
        search_ahead = 50
        best = self._waypoints_visited
        best_d = float("inf")
        for i in range(self._waypoints_visited,
                       min(self._waypoints_visited + search_ahead, self._total_waypoints)):
            wp = self._path_pts[i]
            d = (wp[0] - pn) ** 2 + (wp[1] - pe) ** 2
            if d < best_d:
                best_d = d
                best = i
        self._waypoints_visited = best

        # Edge-detect spray state change
        if self._spray_flags:
            state = self._spray_flags[min(best, len(self._spray_flags) - 1)]
            if state != self._last_spray_state:
                spray_msg = Bool()
                spray_msg.data = bool(state)
                self._spray_pub.publish(spray_msg)
                self._last_spray_state = state

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