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
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


# ---------------------------------------------------------------------------
# Path generators (hardcoded shapes for SITL)
# ---------------------------------------------------------------------------
def gen_straight_5m(spacing: float = 0.5) -> list[tuple[float, float]]:
    """5 m straight north, points at `spacing` intervals."""
    n_steps = int(5.0 / spacing) + 1
    return [(i * spacing, 0.0) for i in range(n_steps)]


def gen_arc_quarter_1m5(radius: float = 1.5, arc_spacing: float = 0.1) \
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


def gen_lshape_2x2(spacing: float = 0.25) -> list[tuple[float, float]]:
    """2 m north, then 2 m east. Sharp 90° corner."""
    pts = []
    n_steps_1 = int(2.0 / spacing) + 1
    for i in range(n_steps_1):
        pts.append((i * spacing, 0.0))
    n_steps_2 = int(2.0 / spacing)
    for i in range(1, n_steps_2 + 1):
        pts.append((2.0, i * spacing))
    return pts


def gen_square_2x2(spacing: float = 0.25) -> list[tuple[float, float]]:
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


def gen_rectangle_3x2(spacing: float = 0.25) -> list[tuple[float, float]]:
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


def gen_circle_1m5(radius: float = 1.5, arc_spacing: float = 0.1) \
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
    "arc_quarter_1m5": gen_arc_quarter_1m5,
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


def load_mission_file(filepath: str) -> list[tuple[float, float]]:
    """Auto-detect file format and load waypoints.

    .waypoints → QGC WPL 110 (lat/lon → NED via Karney)
    .csv       → simple NED metres (north, east)
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"Mission file not found: {filepath}")

    ext = os.path.splitext(filepath)[1].lower()
    if ext == ".waypoints":
        return read_qgc_waypoints(filepath)
    elif ext == ".csv":
        return read_ned_csv(filepath)
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

        # TRANSIENT_LOCAL so late-joining subscribers (rpp_controller) get it
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(Path, "/path", path_qos)

        # One-shot timer to publish after a brief settle
        delay = self.get_parameter("publish_delay_s").value
        self._timer = self.create_timer(delay, self._publish_once)

        mission_file = self.get_parameter("mission_file").value
        path_name = self.get_parameter("path_name").value
        source = f"file={mission_file}" if mission_file else f"path_name={path_name}"
        self.get_logger().info(
            f"path_publisher started — will publish {source} after {delay:.1f}s"
        )

    def _publish_once(self):
        self._timer.cancel()

        frame_id = self.get_parameter("frame_id").value
        mission_file = self.get_parameter("mission_file").value
        path_name = self.get_parameter("path_name").value

        # Load path points
        if mission_file:
            try:
                pts = load_mission_file(mission_file)
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
        self.get_logger().info(
            f"Published {source!r}: {len(path.poses)} waypoints "
            f"first=({pts[0][0]:.3f}N,{pts[0][1]:.3f}E) "
            f"last=({pts[-1][0]:.3f}N,{pts[-1][1]:.3f}E) "
            f"frame={frame_id!r}"
        )


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