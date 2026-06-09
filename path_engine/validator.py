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
        max_waypoints: int = 10000,
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

        return warnings, errors

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
