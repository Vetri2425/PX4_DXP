"""Path validation and sanity checks for PlannedPath."""

from __future__ import annotations

import math
from .core import PlannedPath


class PathValidationError(ValueError):
    """Raised when a planned path is unsafe to publish or execute."""

    def __init__(self, errors: list[str]):
        self.errors = list(errors)
        super().__init__("; ".join(self.errors))


class PathValidator:
    """Validator for verifying the physical and geometric safety of PlannedPaths."""

    def __init__(
        self,
        min_turn_radius_m: float = 0.3,
        max_gap_m: float = 0.5,
        max_bbox_size_m: float = 1000.0,
        max_waypoints: int = 100000,
        max_segments: int = 2000,
    ):
        self.min_turn_radius_m = min_turn_radius_m
        self.max_gap_m = max_gap_m
        self.max_bbox_size_m = max_bbox_size_m
        self.max_waypoints = max_waypoints
        self.max_segments = max_segments

    def validate(self, plan: PlannedPath) -> list[str]:
        """Run all safety and sanity checks. Returns a list of warning strings."""
        warnings, _ = self.validate_detailed(plan)
        return warnings

    def validate_detailed(self, plan: PlannedPath) -> tuple[list[str], list[str]]:
        """Run sanity checks and return (warnings, hard_errors)."""
        warnings: list[str] = []
        errors: list[str] = []
        if not plan.merged_waypoints:
            return ["Path contains no waypoints."], errors

        # 1. Publication size checks
        self._check_counts(plan, warnings, errors)

        # 2. Bounding Box Check
        self._check_bounding_box(plan.merged_waypoints, warnings)

        # 3. Turning Radius / Curvature Check
        self._check_turn_radius(plan.merged_waypoints, warnings)

        # 4. Gap Check
        self._check_gaps(plan.merged_waypoints, warnings)

        # 5. Self-Intersection Check
        self._check_self_intersections(plan.merged_waypoints, warnings)

        # 6. Does the rover drive over paint it has already laid? (E3)
        self._check_drives_over_wet_paint(plan, warnings)

        # 7. How far outside the drawing do the extensions reach? (E4)
        self._check_extension_overshoot(plan, warnings)

        # 8. Duplicate geometry dropped from the source drawing.
        self._check_duplicate_geometry(plan, warnings)

        return warnings, errors

    def _check_duplicate_geometry(self, plan: PlannedPath, warnings: list[str]) -> None:
        """Tell the operator when the DRAWING contained lines on top of each other.

        The planner drops them (marking a line twice paints it double-thick and forces a
        180 deg reverse between the two passes), but the operator should know their CAD
        file has them — the next export will have them too.
        """
        dup = (plan.planning_metadata or {}).get("duplicate_geometry") or {}
        n = dup.get("removed", 0)
        if not n:
            return
        srcs = ", ".join(dup.get("sources", [])[:3])
        more = f" (+{n - 3} more)" if n > 3 else ""
        warnings.append(
            f"Dropped {n} duplicate line(s) from the drawing: {srcs}{more}. They sit on "
            f"top of geometry already being marked — check the source DXF."
        )

    @staticmethod
    def _seg_cross(p1, p2, p3, p4) -> bool:
        """True if the OPEN segments p1p2 and p3p4 properly cross."""
        d = (p2[0] - p1[0]) * (p4[1] - p3[1]) - (p2[1] - p1[1]) * (p4[0] - p3[0])
        if abs(d) < 1e-12:
            return False
        t = ((p3[0] - p1[0]) * (p4[1] - p3[1]) - (p3[1] - p1[1]) * (p4[0] - p3[0])) / d
        u = ((p3[0] - p1[0]) * (p2[1] - p1[1]) - (p3[1] - p1[1]) * (p2[0] - p1[0])) / d
        return 0.05 < t < 0.95 and 0.05 < u < 0.95

    def _check_drives_over_wet_paint(self, plan: PlannedPath, warnings: list[str]) -> None:
        """Warn when a spray-OFF move crosses a line the rover has ALREADY painted.

        The inter-run connector is a straight shot from one run's run-out to the next
        run's run-up, and nothing stops it crossing finished geometry. The rover then
        drives its wheels through wet paint.

        This is a *report*, not a fix: making the router paint-aware means costing
        crossings in the segment ordering, which is a separate change. Surfacing it here
        at least means the operator is not the one who discovers it, on the ground.
        """
        wp, fl = plan.merged_waypoints, plan.spray_flags
        if len(wp) < 4 or len(fl) != len(wp):
            return

        # Spatially indexed instead of all-pairs. The naive double loop is
        # O(n^2): on a 2.4 km road survey (48560 waypoints) it ran 67.7 MILLION
        # segment tests and took 35 s on a laptop, 100 s on the Jetson — the
        # single dominant cost of planning that mission. Painted segments are
        # bucketed into a uniform grid and each spray-OFF move is tested only
        # against the buckets it actually overlaps.
        #
        # The grid is filled INCREMENTALLY as i advances, so a segment is only
        # ever tested against paint laid EARLIER — identical semantics to the
        # `for j in range(i)` it replaces. Which j matches first can differ, but
        # the recorded crossing is wp[i], so the result is unchanged either way.
        cell = 5.0
        grid: dict[tuple[int, int], list[int]] = {}
        oversized: list[int] = []             # segments spanning too many cells

        def _cells(a, b):
            n0, n1 = (a[0], b[0]) if a[0] <= b[0] else (b[0], a[0])
            e0, e1 = (a[1], b[1]) if a[1] <= b[1] else (b[1], a[1])
            return (int(n0 // cell), int(n1 // cell),
                    int(e0 // cell), int(e1 // cell))

        crossings: list[tuple[float, float]] = []
        for i in range(len(wp) - 1):
            # Publish segment i-1 before testing i: only earlier paint is wet.
            if i > 0 and fl[i - 1]:
                cn0, cn1, ce0, ce1 = _cells(wp[i - 1], wp[i])
                if (cn1 - cn0 + 1) * (ce1 - ce0 + 1) > 64:
                    oversized.append(i - 1)
                else:
                    for cn in range(cn0, cn1 + 1):
                        for ce in range(ce0, ce1 + 1):
                            grid.setdefault((cn, ce), []).append(i - 1)

            if fl[i]:
                continue                      # only spray-OFF moves

            cn0, cn1, ce0, ce1 = _cells(wp[i], wp[i + 1])
            if (cn1 - cn0 + 1) * (ce1 - ce0 + 1) > 64:
                candidates = range(i)         # huge transit: fall back to all
            else:
                seen: set[int] = set(oversized)
                for cn in range(cn0, cn1 + 1):
                    for ce in range(ce0, ce1 + 1):
                        seen.update(grid.get((cn, ce), ()))
                candidates = seen

            for j in candidates:
                if j >= i or not fl[j]:
                    continue                  # only against paint already laid
                if self._seg_cross(wp[i], wp[i + 1], wp[j], wp[j + 1]):
                    crossings.append(wp[i])
                    break

        if crossings:
            where = ", ".join(f"({p[0]:.2f}, {p[1]:.2f})" for p in crossings[:3])
            more = f" (+{len(crossings) - 3} more)" if len(crossings) > 3 else ""
            warnings.append(
                f"Rover drives over already-painted lines at {len(crossings)} point(s): "
                f"{where}{more}. Transit routing is not paint-aware — the wheels will "
                f"cross wet paint."
            )

    def _check_extension_overshoot(self, plan: PlannedPath, warnings: list[str]) -> None:
        """Warn how far the run-ups/run-outs reach beyond the drawing itself.

        With 0.5 m extensions a 3x3 m square becomes a 4x4 m swept area. On a bounded
        site — wall, kerb, pad edge, parked vehicle — that is an out-of-bounds excursion,
        and nothing else in the pipeline mentions it.
        """
        ext = (plan.planning_metadata or {}).get("extensions")
        if not ext:
            return
        over = ext.get("max_overshoot_m", 0.0)
        if over <= 0.01:
            return
        swept = ext.get("swept_bbox") or {}
        marked = ext.get("marked_bbox") or {}
        warnings.append(
            f"Extensions reach {over:.2f} m beyond the marked geometry: swept area is "
            f"{swept.get('width_m', 0):.2f} x {swept.get('height_m', 0):.2f} m vs marked "
            f"{marked.get('width_m', 0):.2f} x {marked.get('height_m', 0):.2f} m. "
            f"Confirm the site has this clearance."
        )

    def validate_or_raise(self, plan: PlannedPath) -> list[str]:
        """Return warnings or raise PathValidationError for hard safety failures."""
        warnings, errors = self.validate_detailed(plan)
        if errors:
            raise PathValidationError(errors)
        return warnings

    def _check_counts(self, plan: PlannedPath, warnings: list[str], errors: list[str]) -> None:
        n_waypoints = plan.num_waypoints
        n_segments = len(plan.segments)

        if n_waypoints > self.max_waypoints:
            errors.append(
                f"Too many waypoints: {n_waypoints} exceeds limit {self.max_waypoints}. "
                f"Increase spacing, fix units, or simplify the drawing before publishing."
            )

        if n_segments > self.max_segments:
            errors.append(
                f"Too many path segments: {n_segments} exceeds limit {self.max_segments}. "
                f"Check for a bad CAD export or split the job into smaller missions."
            )

        warn_at = int(self.max_waypoints * 0.8)
        if n_waypoints > warn_at and n_waypoints <= self.max_waypoints:
            warnings.append(
                f"High waypoint count: {n_waypoints}/{self.max_waypoints}. "
                f"Large /path messages can slow ROS2 and mobile clients."
            )

    def _check_bounding_box(self, pts: list[tuple[float, float]], warnings: list[str]) -> None:
        norths = [p[0] for p in pts]
        easts = [p[1] for p in pts]
        min_n, max_n = min(norths), max(norths)
        min_e, max_e = min(easts), max(easts)
        width = max_e - min_e
        height = max_n - min_n

        if width > self.max_bbox_size_m or height > self.max_bbox_size_m:
            warnings.append(
                f"Path bounding box is very large ({width:.1f}m x {height:.1f}m). "
                f"Check DXF/CSV units; template might be in centimetres or inches instead of metres."
            )

    def _check_turn_radius(self, pts: list[tuple[float, float]], warnings: list[str]) -> None:
        n_pts = len(pts)
        if n_pts < 3:
            return

        violations = 0
        min_radius_found = float("inf")
        worst_idx = -1

        for i in range(1, n_pts - 1):
            a, b, c = pts[i - 1], pts[i], pts[i + 1]
            ab = math.hypot(b[0] - a[0], b[1] - a[1])
            bc = math.hypot(c[0] - b[0], c[1] - b[1])
            ca = math.hypot(a[0] - c[0], a[1] - c[1])

            if ab < 1e-5 or bc < 1e-5 or ca < 1e-5:
                continue

            # Area of triangle via cross product
            area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
            # Menger curvature = 4 * Area / (ab * bc * ca)
            kappa = (2.0 * area2) / (ab * bc * ca)

            if kappa > 1e-3:
                radius = 1.0 / kappa
                if radius < self.min_turn_radius_m:
                    violations += 1
                    if radius < min_radius_found:
                        min_radius_found = radius
                        worst_idx = i

        if violations > 0:
            warnings.append(
                f"Found {violations} tight corners violating the minimum turning radius of {self.min_turn_radius_m}m. "
                f"Worst corner at waypoint index {worst_idx} with radius {min_radius_found:.2f}m."
            )

    def _check_gaps(self, pts: list[tuple[float, float]], warnings: list[str]) -> None:
        violations = 0
        max_gap_found = 0.0
        worst_idx = -1

        for i in range(1, len(pts)):
            d = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            if d > self.max_gap_m:
                violations += 1
                if d > max_gap_found:
                    max_gap_found = d
                    worst_idx = i

        if violations > 0:
            warnings.append(
                f"Found {violations} waypoint gaps larger than {self.max_gap_m}m. "
                f"Largest gap is {max_gap_found:.2f}m between index {worst_idx - 1} and {worst_idx}. "
                f"Ensure the path is fully connected with TRANSIT segments."
            )

    def _check_self_intersections(self, pts: list[tuple[float, float]], warnings: list[str]) -> None:
        # Cap segment comparisons to keep computation time low
        n_segs = len(pts) - 1
        if n_segs < 3:
            return

        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        def intersect(p1, p2, p3, p4):
            # Check bounding boxes first
            if max(p1[0], p2[0]) < min(p3[0], p4[0]) or min(p1[0], p2[0]) > max(p3[0], p4[0]):
                return False
            if max(p1[1], p2[1]) < min(p3[1], p4[1]) or min(p1[1], p2[1]) > max(p3[1], p4[1]):
                return False
            return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

        # A closed loop legitimately shares its start/end vertex: the first and
        # last segments meet there and must not be flagged as a self-intersection.
        is_closed = math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-6

        intersections = []
        # To avoid O(N^2) explosion on large files, we limit check to max 1000 segments
        step = max(1, n_segs // 1000)
        # V1 fix: when step > 1 only a subset of segment pairs is tested, so a
        # clean result is NOT a guarantee. Tell the operator instead of giving
        # false reassurance.
        if step > 1:
            warnings.append(
                f"Self-intersection check is SAMPLED (1 in {step} segments tested "
                f"of {n_segs}) because the path is large — a clean result does "
                f"not guarantee the path is intersection-free."
            )
        for i in range(0, n_segs, step):
            p1, p2 = pts[i], pts[i + 1]
            for j in range(i + 2, n_segs, step):
                if j == i + 1 or j == i - 1:
                    continue
                # Skip the first/last segment pair that shares the loop-closure vertex
                if is_closed and i == 0 and j == n_segs - 1:
                    continue
                p3, p4 = pts[j], pts[j + 1]
                if intersect(p1, p2, p3, p4):
                    intersections.append((i, j))
                    if len(intersections) >= 5:
                        break
            if len(intersections) >= 5:
                break

        if intersections:
            warnings.append(
                f"Path self-intersects at {len(intersections)} or more locations "
                f"(e.g., segment near index {intersections[0][0]} crosses segment near index {intersections[0][1]})."
            )
