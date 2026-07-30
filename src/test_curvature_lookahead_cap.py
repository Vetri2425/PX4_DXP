#!/usr/bin/env python3
"""P5.1 — arc-cut lookahead cap + robust curvature estimator.

ROS-FREE by design: stubs rclpy/geometry_msgs so it runs on the Mac dev box
(no ROS2 there), because these two pieces are pure geometry and are exactly
where a silent numerical error cost a day of field time.

Background (2026-07-30): `_path_curvature_at` used ADJACENT vertices. On a
conditioned path densified at 4 cm that is noise-dominated — it reported
kappa_max 1.251 (R = 0.80 m) on an arc whose true radius was 2.3-2.4 m. The
bogus number produced a confident wrong diagnosis (imagined yaw-rate
saturation) that a field test then disproved. Hence: baseline convergence is
pinned here, and so is the cap arithmetic against the measured field result.

Run:  python3 -m pytest src/test_curvature_lookahead_cap.py
"""
import math
import sys
import types
import unittest

# ---------------------------------------------------------------------------
# Minimal ROS stubs — enough to import the module, nothing more.
# ---------------------------------------------------------------------------
_ROOTS = ("rclpy", "geometry_msgs", "nav_msgs", "std_msgs", "sensor_msgs",
          "mavros_msgs", "rcl_interfaces")


def _stub(name, **attrs):
    m = types.ModuleType(name)
    m.__path__ = []          # make it a package so `import x.y` resolves
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class _AnyMeta(type):
    """Class-level attribute access too: ROS enums are read as ReliabilityPolicy.BEST_EFFORT."""
    def __getattr__(cls, _n):
        return _Any()


class _Any(metaclass=_AnyMeta):
    """Accepts any attribute / call — stands in for ROS message and QoS types."""
    def __init__(self, *a, **k):
        pass

    def __getattr__(self, _n):
        return _Any()

    def __call__(self, *a, **k):
        return _Any()

    def __hash__(self):
        return 0

    def __eq__(self, _o):
        return True


def _real_ros_available() -> bool:
    """True only for a genuinely INSTALLED rclpy (filesystem-backed package).

    Neither `"rclpy" in sys.modules` nor an attribute probe is sufficient:
    other suites in this repo import rclpy inside try/except and can leave a
    bare ModuleType behind, which satisfies both checks but is not a package,
    so `import rclpy.time` then fails with "'rclpy' is not a package" — and
    only when the suites run TOGETHER, which is the confusing way to find out.
    A real install has both __file__ and __path__; a stub has neither.
    """
    m = sys.modules.get("rclpy")
    if m is None:
        try:
            import rclpy as m  # noqa: F811
        except Exception:
            return False
    return hasattr(m, "__path__") and getattr(m, "__file__", None) is not None


if not _real_ros_available():
    # Drop any half-built stub another module left behind, so ours is coherent.
    for _k in [k for k in sys.modules if k.split(".")[0] in _ROOTS]:
        del sys.modules[_k]
    _stub("rclpy", init=lambda *a, **k: None, shutdown=lambda *a, **k: None,
          spin=lambda *a, **k: None, ok=lambda: True)
    _stub("rclpy.node", Node=_Any)
    _stub("rclpy.qos", QoSProfile=_Any, ReliabilityPolicy=_Any, HistoryPolicy=_Any,
          DurabilityPolicy=_Any, qos_profile_sensor_data=_Any())
    _stub("rclpy.callback_groups", MutuallyExclusiveCallbackGroup=_Any,
          ReentrantCallbackGroup=_Any)
    _stub("rclpy.executors", MultiThreadedExecutor=_Any, SingleThreadedExecutor=_Any,
          ExternalShutdownException=type("ExternalShutdownException", (Exception,), {}))
    _stub("rclpy.parameter", Parameter=_Any)
    _stub("rclpy.duration", Duration=_Any)
    _stub("rclpy.time", Time=_Any)
    _stub("geometry_msgs", msg=_Any())
    _stub("geometry_msgs.msg", PoseStamped=_Any, Twist=_Any, TwistStamped=_Any,
          Vector3Stamped=_Any, Point=_Any, Quaternion=_Any, Pose=_Any)
    _stub("nav_msgs", msg=_Any())
    _stub("nav_msgs.msg", Path=_Any, Odometry=_Any)
    _stub("std_msgs", msg=_Any())
    _stub("std_msgs.msg", Bool=_Any, Float32=_Any, String=_Any,
          Float32MultiArray=_Any, MultiArrayDimension=_Any)
    _stub("sensor_msgs", msg=_Any())
    _stub("sensor_msgs.msg", Imu=_Any, NavSatFix=_Any)
    _stub("mavros_msgs", msg=_Any(), srv=_Any())
    _stub("mavros_msgs.msg", State=_Any, PositionTarget=_Any, GPSRAW=_Any)
    _stub("mavros_msgs.srv", CommandBool=_Any, SetMode=_Any, CommandLong=_Any)
    _stub("rcl_interfaces", msg=_Any())
    _stub("rcl_interfaces.msg", SetParametersResult=_Any, ParameterDescriptor=_Any,
          FloatingPointRange=_Any, IntegerRange=_Any)

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import rpp_controller_node as rc  # noqa: E402

RPP = rc.RPPControllerNode


# ---------------------------------------------------------------------------
class _Pt:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _Pose:
    def __init__(self, x, y):
        self.position = _Pt(x, y)


class _PS:
    def __init__(self, x, y):
        self.pose = _Pose(x, y)


class _FakePathHolder:
    """Just enough of the node for _path_curvature_at (reads only self._path)."""
    _path_curvature_at = RPP._path_curvature_at

    def __init__(self, pts):
        self._path = [_PS(n, e) for n, e in pts]


def _arc(radius, total_deg=120.0, spacing=0.04, jitter=0.0, seed=1):
    """Constant-radius arc sampled at `spacing` metres, optional coord noise."""
    pts = []
    n = max(3, int(radius * math.radians(total_deg) / spacing))
    # Deterministic pseudo-noise: no Math.random equivalent needed, and a fixed
    # sequence keeps the assertions reproducible.
    st = seed
    for i in range(n + 1):
        th = math.radians(total_deg) * i / n
        jx = jy = 0.0
        if jitter:
            st = (st * 1103515245 + 12345) & 0x7FFFFFFF
            jx = ((st / 0x7FFFFFFF) - 0.5) * 2 * jitter
            st = (st * 1103515245 + 12345) & 0x7FFFFFFF
            jy = ((st / 0x7FFFFFFF) - 0.5) * 2 * jitter
        pts.append((radius * math.cos(th) + jx, radius * math.sin(th) + jy))
    return pts


class TestCurvatureBaseline(unittest.TestCase):
    R = 2.35          # the 2026-07-30 curve_6_points-1 arc
    SPACING = 0.04    # its measured waypoint spacing (4.1 cm)

    def test_clean_arc_recovers_radius_at_any_baseline(self):
        h = _FakePathHolder(_arc(self.R, spacing=self.SPACING))
        mid = len(h._path) // 2
        for bl in (0.0, 0.10, 0.15, 0.30):
            k = h._path_curvature_at(mid, baseline_m=bl)
            self.assertAlmostEqual(1.0 / k, self.R, delta=0.05,
                                   msg=f"baseline {bl} on a noiseless arc")

    def test_adjacent_vertices_blow_up_under_realistic_noise(self):
        """The actual 2026-07-30 failure: 1 cm coord noise on 4 cm spacing."""
        h = _FakePathHolder(_arc(self.R, spacing=self.SPACING, jitter=0.01))
        mid = len(h._path) // 2
        naive = h._path_curvature_at(mid, baseline_m=0.0)
        robust = h._path_curvature_at(mid, baseline_m=0.15)
        true_k = 1.0 / self.R
        self.assertGreater(naive, 2.0 * true_k,
                           "adjacent-vertex estimate should be wildly inflated")
        self.assertAlmostEqual(robust, true_k, delta=0.5 * true_k,
                               msg="0.15 m baseline should stay near truth")

    def test_baseline_walks_arc_length_not_index(self):
        """Same geometry, 4x denser sampling → same curvature for a given baseline."""
        mid_a = len(_arc(self.R, spacing=0.04)) // 2
        ka = _FakePathHolder(_arc(self.R, spacing=0.04))._path_curvature_at(
            mid_a, baseline_m=0.20)
        mid_b = len(_arc(self.R, spacing=0.01)) // 2
        kb = _FakePathHolder(_arc(self.R, spacing=0.01))._path_curvature_at(
            mid_b, baseline_m=0.20)
        self.assertAlmostEqual(ka, kb, delta=0.05)

    def test_straight_line_is_zero_curvature(self):
        h = _FakePathHolder([(0.0, i * 0.05) for i in range(40)])
        self.assertLess(h._path_curvature_at(20, baseline_m=0.15), 1e-6)

    def test_degenerate_paths_return_zero(self):
        self.assertEqual(_FakePathHolder([(0, 0), (1, 0)])._path_curvature_at(0, 0.15), 0.0)
        self.assertEqual(_FakePathHolder([])._path_curvature_at(0, 0.15), 0.0)


class TestArcCutCap(unittest.TestCase):
    """The cap law itself: L = sqrt(8*e_target/kappa), floored.

    Mirrors the expression in the smooth tracker. Anchored on field data:
    kappa=0.43, e_target=0.005 -> 0.305 m, which is the L the 2026-07-30 curve
    runs flew: valve-gated marking RMS 1.51 -> 1.23/1.24/1.30 cm, inside-cut
    +1.17 -> -0.39 cm.

    NOTE the earlier "2.31 -> 1.34 cm" figure is RETRACTED -- both sides of it
    came from the pre-`ff3a9bb` analyser, which folded DRY samples into
    marking_only and over-reported the baseline. Re-scored consistently on the
    valve, the same runs give 1.51 -> 1.26 (a 17% gain, not 42%).
    """
    E_TARGET = 0.005
    FLOOR = 0.25
    RAW = 0.56

    def _l_d(self, kappa):
        if kappa <= 1e-6:
            return self.RAW
        cap = math.sqrt(8.0 * self.E_TARGET / kappa)
        return max(min(self.RAW, cap), self.FLOOR)

    def test_reproduces_the_field_validated_lookahead(self):
        self.assertAlmostEqual(self._l_d(0.43), 0.305, places=2)

    def test_straights_and_gentle_arcs_are_untouched(self):
        self.assertEqual(self._l_d(0.0), self.RAW)
        for k in (0.05, 0.10):
            self.assertEqual(self._l_d(k), self.RAW,
                             f"kappa {k} (R={1/k:.0f} m) should not be capped")

    def test_predicted_cut_is_bounded_at_the_target_where_cap_binds(self):
        for k in (0.20, 0.43):
            L = self._l_d(k)
            self.assertAlmostEqual(L * L * k / 8.0, self.E_TARGET, delta=1e-4)

    def test_floor_wins_on_tight_arcs_so_l_d_cannot_collapse(self):
        for k in (0.80, 1.5, 3.0):
            self.assertGreaterEqual(self._l_d(k), self.FLOOR)
        # ...and the floor keeps us clear of the 0.12-0.21 m band that went
        # unstable on 2026-07-29.
        self.assertGreater(self.FLOOR, 0.21)

    def test_cut_scales_as_L_squared(self):
        """Pure algebra: e = L^2*kappa/8, so the cut ratio is the L ratio squared.

        The "L=1.00 arm confirmed this, 3.30 measured vs 3.10 predicted" claim
        is RETRACTED -- that arm's valve was open for only 3.5-5.3 s of a 13.5 s
        line, and the lookahead DURING paint was 0.174 m, not 1.00. It measured
        a selection effect, not the law. The law itself still holds; it is just
        not field-confirmed at L=1.00.
        """
        k = 0.43
        cut = lambda L: L * L * k / 8.0
        self.assertAlmostEqual(cut(0.982) / cut(0.558), (0.982 / 0.558) ** 2, places=6)


class TestParamDefaultsArePinned(unittest.TestCase):
    def test_cap_default_is_the_field_validated_5mm(self):
        """P5.1 shipped ON at 5 mm after the 2026-07-30 curve runs.

        It was declared 0.0 (a named A/B) until that field pass: 3 runs took
        valve-gated marking RMS 1.51 -> 1.23/1.24/1.30 cm with the inside-cut
        signature nulled (+1.17 -> -0.39 cm) and full coverage in one spray
        interval. The default is pinned here so it cannot drift silently in
        either direction -- flipping it is a controller change and must come
        with its own field evidence.
        """
        src = open(rc.__file__).read()
        self.assertIn('declare_parameter("smooth_max_arc_cut_m",                0.005)', src)
        self.assertIn("curvature_baseline_m", src)
        self.assertIn("smooth_min_arc_ld_m", src)

    def test_zero_still_restores_the_pre_p5_1_geometry(self):
        """The A/B arm must survive as a real escape hatch, not just a comment.

        Mirrors the controller's branch: cut_target = 0 skips the cap entirely
        and falls through to the legacy coeff FLOOR, which is what the frozen
        controller did before P5.1.
        """
        raw, kappa, ld_coeff = 0.56, 0.43, 0.20

        def l_d(cut_target):
            if kappa > 1e-6 and cut_target > 0.0:
                return max(min(raw, math.sqrt(8.0 * cut_target / kappa)), 0.25)
            if kappa > 1e-6 and ld_coeff > 0.0:
                return max(raw, ld_coeff / kappa)
            return raw

        # At the field kappa the legacy coeff floor (0.465) sits BELOW the
        # velocity-scaled raw, so pre-P5.1 the rover ran the full 0.56 m.
        self.assertAlmostEqual(l_d(0.0), raw, places=6)
        self.assertAlmostEqual(l_d(0.005), 0.305, places=3)
        self.assertLess(l_d(0.005), l_d(0.0),
                        "the cap must bind BELOW the pre-P5.1 lookahead or it is inert")


if __name__ == "__main__":
    unittest.main(verbosity=2)
