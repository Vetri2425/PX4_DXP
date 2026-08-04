#!/usr/bin/env python3
"""Tests for analyze_bag_ulog.py — the closed-loop tuning diagnostic.

pytest is the authoritative runner:

    python3 -m pytest tools/test_analyze_bag_ulog.py

Running the file directly executes only the unittest.TestCase classes.

No bags, no .ulg files, no ROS. Every fixture is a synthetic numpy signal built
in-test, so the suite says something about the MODEL rather than about whichever
log happened to be on disk.
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analyze_bag_ulog as abu  # noqa: E402


# ----------------------------------------------------------------------
# fixtures
# ----------------------------------------------------------------------
LIVE_PARAMS = {
    "RD_WHEEL_TRACK": 0.47,
    "RD_MAX_THR_YAW_R": 0.95,
    "RO_MAX_THR_SPEED": 1.28,
    "RO_YAW_P": 1.5,
    "RO_YAW_RATE_P": 0.13,
    "RO_YAW_RATE_I": 0.01,
    "RO_YAW_RATE_LIM": 22.0,
    "RO_YAW_RATE_TH": 1.0,
    "RO_SPEED_TH": 0.10,
    "RO_SPEED_LIM": 0.70,
    "RD_TRANS_DRV_TRN": 0.70,
    "RD_TRANS_TRN_DRV": 0.0349,
}


def fw(**over):
    p = dict(LIVE_PARAMS)
    p.update(over)
    return abu.Firmware(p)


# ----------------------------------------------------------------------
# alignment
# ----------------------------------------------------------------------
class TestAlignment(unittest.TestCase):
    def _pair(self, true_off, rate=0.02):
        t = np.arange(0, 40, rate)
        y = np.sin(2 * np.pi * 0.33 * t) + 0.3 * np.sin(2 * np.pi * 1.1 * t)
        rng = np.random.default_rng(7)
        bag_t, bag_y = t + 1e6, y + rng.normal(0, 0.02, len(t))
        ul_t = t[::2]
        ul_y = np.interp(ul_t + true_off, t, y) + rng.normal(0, 0.02, len(ul_t))
        return bag_t, bag_y, ul_t, ul_y

    def test_alignment_recovers_a_known_injected_offset(self):
        for true_off in (0.0, 0.24, -0.37, 1.05):
            bt, by, ut, uy = self._pair(true_off)
            got, corr, _ = abu.refine_offset(bt, by, ut, uy, 1e6)
            self.assertAlmostEqual(got - 1e6, true_off, delta=0.021,
                                   msg=f"offset {true_off}: got {got - 1e6}")
            self.assertGreater(corr, 0.9)

    def test_alignment_reports_weak_lock_when_the_signals_are_unrelated(self):
        rng = np.random.default_rng(3)
        t = np.arange(0, 40, 0.02)
        bt, by = t + 1e6, rng.normal(0, 1, len(t))
        ut, uy = t, rng.normal(0, 1, len(t))
        _, corr, _ = abu.refine_offset(bt, by, ut, uy, 1e6)
        self.assertLess(corr, 0.7, "uncorrelated noise must not report a lock")

    def test_alignment_returns_coarse_unchanged_when_windows_do_not_overlap(self):
        got, _, _ = abu.refine_offset(np.arange(0, 10, .02) + 5000, np.zeros(500),
                                      np.arange(0, 10, .02), np.zeros(500), 0.0)
        self.assertEqual(got, 0.0)


# ----------------------------------------------------------------------
# Link discipline
# ----------------------------------------------------------------------
class TestLink(unittest.TestCase):
    def test_link_gain_is_one_for_an_identical_signal(self):
        t = np.arange(0, 10, 0.1)
        x = np.sin(t)
        lk = abu.Link("id", t, x, x.copy(), "u", np.ones(len(t), bool))
        self.assertAlmostEqual(lk.gain, 1.0, places=6)
        self.assertAlmostEqual(lk.rms, 0.0, places=9)

    def test_link_recovers_a_known_scale(self):
        t = np.arange(0, 20, 0.05)
        x = np.sin(t)
        lk = abu.Link("id", t, x, 1.25 * x, "u", np.ones(len(t), bool))
        self.assertAlmostEqual(lk.gain, 1.25, places=6)

    def test_link_reports_a_zero_source_channel_instead_of_a_gain(self):
        t = np.arange(0, 10, 0.1)
        lk = abu.Link("id", t, np.zeros(len(t)), np.sin(t), "u",
                      np.ones(len(t), bool))
        self.assertTrue(lk.degenerate)
        self.assertFalse(lk.ok)
        self.assertIn("identically zero", lk.row())
        self.assertNotIn("nan", lk.row().lower())

    def test_link_refuses_a_gain_from_too_few_samples(self):
        t = np.arange(0, 0.5, 0.1)
        lk = abu.Link("id", t, np.sin(t), np.sin(t), "u", np.ones(len(t), bool))
        self.assertFalse(lk.ok)


# ----------------------------------------------------------------------
# firmware model
# ----------------------------------------------------------------------
class TestFirmware(unittest.TestCase):
    def test_feedforward_matches_the_v1_16_2_formula(self):
        f = fw()
        omega = 0.20
        self.assertAlmostEqual(float(f.ff_speed_diff(omega)),
                               omega * 0.47 / 2 / 0.95, places=9)

    def test_feedforward_saturates_at_the_interpolate_clamp(self):
        f = fw()
        self.assertEqual(float(f.ff_speed_diff(100.0)), 1.0)
        self.assertEqual(float(f.ff_speed_diff(-100.0)), -1.0)

    def test_steering_is_feedforward_plus_pid(self):
        f = fw()
        adj, meas, integ = 0.20, 0.15, 0.004
        want = adj * 0.47 / 2 / 0.95 + 0.13 * (adj - meas) + integ
        self.assertAlmostEqual(float(f.steering(adj, meas, integ)), want, places=9)

    def test_steering_drops_the_pid_when_the_setpoint_is_zero(self):
        """A dead-banded setpoint resets the integrator (RoverControl.cpp:209)."""
        f = fw()
        self.assertEqual(float(f.steering(0.0, 0.9, 0.5)), 0.0)

    def test_inverse_kinematics_uses_the_fork_sign_convention(self):
        """Fork: left = thr + d, right = thr - d. Positive d = RIGHT turn."""
        f = fw()
        left, right = f.inverse_kinematics(0.4, 0.1)
        self.assertAlmostEqual(float(left), 0.5, places=9)
        self.assertAlmostEqual(float(right), 0.3, places=9)

    def test_stock_sign_convention_would_give_the_opposite_wheels(self):
        """Pairs with the test above: upstream v1.16.2 returns (thr-d, thr+d).

        If this ever matches the fork model, the overlay has been lost.
        """
        f = fw()
        left, right = f.inverse_kinematics(0.4, 0.1)
        stock_left, stock_right = 0.4 - 0.1, 0.4 + 0.1
        self.assertNotAlmostEqual(float(left), stock_left, places=6)
        self.assertNotAlmostEqual(float(right), stock_right, places=6)

    def test_inverse_kinematics_prioritises_yaw_when_the_sum_exceeds_one(self):
        f = fw()
        left, right = f.inverse_kinematics(0.9, 0.3)
        self.assertAlmostEqual(float(left) - float(right), 0.6, places=9,
                               msg="the differential must survive saturation")
        self.assertLessEqual(abs(float(left)), 1.0 + 1e-9)

    def test_the_actuator_differential_is_exactly_the_steering_command(self):
        """Why R4 reconstructs (left-right)/2 and not the wheels themselves."""
        f = fw()
        for thr, d in ((0.4, 0.1), (0.9, 0.3), (-0.2, 0.5)):
            left, right = f.inverse_kinematics(thr, d)
            self.assertAlmostEqual(0.5 * (float(left) - float(right)), d, places=9)

    def test_yaw_rate_setpoint_clamps_at_RO_YAW_RATE_LIM(self):
        f = fw()
        got = float(f.yaw_rate_setpoint(np.array([math.pi / 2]), np.array([0.0]))[0])
        self.assertAlmostEqual(got, math.radians(22.0), places=9)

    def test_yaw_rate_setpoint_wraps_across_pi(self):
        f = fw()
        sp, yaw = np.array([math.pi - 0.05]), np.array([-math.pi + 0.05])
        self.assertLess(float(f.yaw_rate_setpoint(sp, yaw)[0]), 0.0,
                        "the short way round is negative, not +2pi")

    def test_driving_state_is_hysteretic_not_a_single_threshold(self):
        f = fw()
        err = np.array([0.0, 0.8, 0.5, 0.2, 0.10, 0.02, 0.0])
        spot = f.driving_state(err)
        self.assertFalse(spot[0])
        self.assertTrue(spot[1], "0.8 > RD_TRANS_DRV_TRN enters spot-turn")
        self.assertTrue(spot[3], "0.2 is below entry but above exit: still spot")
        self.assertFalse(spot[5], "0.02 < RD_TRANS_TRN_DRV leaves spot-turn")

    def test_closed_loop_gain_is_one_at_the_optimal_R(self):
        f = fw()
        A = 4.7
        f_opt = fw(RD_MAX_THR_YAW_R=f.R_optimal(A))
        self.assertAlmostEqual(f_opt.closed_loop_gain(A), 1.0, places=9)

    def test_R_optimal_equals_RO_MAX_THR_SPEED_when_the_plant_is_ideal(self):
        f = fw()
        self.assertAlmostEqual(f.R_optimal(f.A_ideal), f.K_spd, places=9)

    def test_R_optimal_moves_down_when_the_plant_slips(self):
        """Pairs with the test above: a lossy plant needs a smaller R."""
        f = fw()
        self.assertLess(f.R_optimal(0.88 * f.A_ideal), f.K_spd)


# ----------------------------------------------------------------------
# dead-bands
# ----------------------------------------------------------------------
class TestDeadbands(unittest.TestCase):
    def test_yaw_rate_deadband_zeroes_below_the_threshold(self):
        f = fw()
        x = np.array([0.0, 0.01, 0.017, 0.05])   # rad/s; TH = 1 deg/s = 0.01745
        got = f.rate_deadband(x)
        self.assertEqual(float(got[1]), 0.0)
        self.assertEqual(float(got[2]), 0.0)
        self.assertAlmostEqual(float(got[3]), 0.05, places=9)

    def test_speed_deadband_leaves_signals_above_the_threshold_untouched(self):
        """Pairs with the test above: the guard must not blind the detector."""
        f = fw()
        x = np.array([0.05, 0.32])
        got = f.speed_deadband(x)
        self.assertEqual(float(got[0]), 0.0)
        self.assertAlmostEqual(float(got[1]), 0.32, places=9)

    def test_heading_deadband_floor_is_TH_over_RO_YAW_P(self):
        f = fw()
        self.assertAlmostEqual(math.degrees(f.heading_deadband_floor_rad),
                               1.0 / 1.5, places=6)

    def test_a_smaller_deadband_lowers_the_heading_floor(self):
        self.assertLess(fw(RO_YAW_RATE_TH=0.2).heading_deadband_floor_rad,
                        fw(RO_YAW_RATE_TH=1.0).heading_deadband_floor_rad)


# ----------------------------------------------------------------------
# multi-anchor alignment
# ----------------------------------------------------------------------
class _StubU:
    utc_offset = 1000.0


class _StubB:
    """Serves a fixed anchor list; align() must not care where they came from."""

    def __init__(self, anchors):
        self._a = anchors

    def anchors(self, U):
        return self._a


def _anchor(name, true_off, noise=0.02, seed=1):
    rng = np.random.default_rng(seed)
    t = np.arange(0, 40, 0.02)
    y = np.sin(2 * np.pi * 0.33 * t) + 0.3 * np.sin(2 * np.pi * 1.1 * t)
    bag = (t + 1000.0, y + rng.normal(0, noise, len(t)))
    ul = (t, np.interp(t + true_off, t, y) + rng.normal(0, noise, len(t)))
    return (name, bag, ul)


class TestAlign(unittest.TestCase):
    def test_alignment_picks_the_anchor_with_the_highest_correlation(self):
        rng = np.random.default_rng(5)
        t = np.arange(0, 40, 0.02)
        junk = ("junk", (t + 1000.0, rng.normal(0, 1, len(t))),
                (t, rng.normal(0, 1, len(t))))
        good = _anchor("good", 0.20)
        _, lines, ok = abu.align(_StubB([junk, good]), _StubU())
        self.assertTrue(ok)
        self.assertTrue(any("best anchor" in ln and "good" in ln for ln in lines))

    def test_anchors_that_agree_report_a_small_spread(self):
        a = [_anchor("a", 0.20, seed=1), _anchor("b", 0.20, seed=2)]
        off, lines, ok = abu.align(_StubB(a), _StubU())
        self.assertTrue(ok)
        self.assertAlmostEqual(off - 1000.0, 0.20, delta=0.03)
        self.assertFalse(any("disagree" in ln for ln in lines))

    def test_anchors_that_disagree_by_more_than_50ms_are_flagged(self):
        """Pairs with the test above: agreement must be a real check."""
        a = [_anchor("a", 0.20, seed=1), _anchor("b", 0.90, seed=2)]
        _, lines, _ = abu.align(_StubB(a), _StubU())
        self.assertTrue(any("disagree" in ln for ln in lines),
                        "a 0.7 s disagreement must be reported")

    def test_alignment_with_no_anchors_is_not_ok(self):
        _, lines, ok = abu.align(_StubB([]), _StubU())
        self.assertFalse(ok)
        self.assertTrue(any("no anchor" in ln for ln in lines))


# ----------------------------------------------------------------------
# Layer A geometry
# ----------------------------------------------------------------------
def _straight(n=60, step=0.1):
    """A path heading due north in NED (n, e)."""
    return np.column_stack([np.arange(n) * step, np.zeros(n)])


class TestGeometry(unittest.TestCase):
    def test_true_xtrack_is_zero_for_a_pose_on_the_path(self):
        p = _straight()
        xy = np.column_stack([np.linspace(0.5, 4.0, 20), np.zeros(20)])
        xt, _, _, off, _ = abu.Geometry.signed_xtrack(p, xy)
        self.assertLess(np.nanmax(np.abs(xt)), 1e-6)
        self.assertFalse(off.any())

    def test_true_xtrack_is_positive_to_the_right_of_travel(self):
        """NED: heading north, right is east (+e). No negation."""
        p = _straight()
        xy = np.column_stack([np.linspace(0.5, 4.0, 20), np.full(20, 0.03)])
        xt, _, _, _, _ = abu.Geometry.signed_xtrack(p, xy)
        self.assertGreater(np.nanmin(xt), 0, "east of a northbound path is RIGHT")
        self.assertAlmostEqual(float(np.nanmean(xt)), 0.03, places=6)

    def test_an_enu_style_negation_would_invert_the_sign(self):
        """Pairs with the test above: this is the bug the perception gap caught.

        Mixing an ENU (e, n) convention into an NED (n, e) frame flips the sign
        and shows up as a perception gap of exactly twice the RMS.
        """
        p = _straight()
        xy = np.column_stack([np.linspace(0.5, 4.0, 20), np.full(20, 0.03)])
        xt, _, _, _, _ = abu.Geometry.signed_xtrack(p, xy)
        self.assertNotAlmostEqual(float(np.nanmean(-xt)), 0.03, places=6)

    def test_progressive_projection_stays_on_the_outbound_leg(self):
        """A there-and-back path is exactly self-overlapping."""
        out = np.column_stack([np.arange(40) * 0.1, np.zeros(40)])
        back = out[::-1].copy()
        p = np.vstack([out, back])
        # driving outbound (north) at 2 cm to the east
        xy = np.column_stack([np.linspace(0.2, 3.5, 25), np.full(25, 0.02)])
        yaw = np.zeros(25)                       # heading north
        xt, s, _, _, _ = abu.Geometry.signed_xtrack(p, xy, yaw)
        self.assertTrue(np.all(np.diff(s) >= -1e-9), "arc length must not go back")
        self.assertLess(s[-1], 4.0, "must stay on the outbound leg, not the return")
        self.assertAlmostEqual(float(np.nanmean(xt)), 0.02, places=6)

    def test_samples_past_the_leg_end_are_flagged_not_clamped(self):
        p = _straight(n=20, step=0.1)            # ends at n = 1.9
        xy = np.array([[1.0, 0.0], [2.5, 0.0]])  # second is 0.6 m past the end
        _, _, _, off, _ = abu.Geometry.signed_xtrack(p, xy)
        self.assertFalse(off[0])
        self.assertTrue(off[1], "along-track overshoot must not be called xtrack")

    def test_a_constant_offset_reports_no_oscillation(self):
        s = np.linspace(0, 3, 100)
        x = np.full(100, 0.02)
        osc, why = abu.Geometry.oscillation(s, x)
        self.assertIsNone(osc)
        self.assertIsNotNone(why)

    def test_wavelength_is_recovered_from_a_synthetic_sinusoid(self):
        """Pairs with the test above: the detector must still fire on a real one."""
        s = np.linspace(0, 6, 400)
        x = 0.01 * np.sin(2 * np.pi * s / 1.5)
        osc, why = abu.Geometry.oscillation(s, x)
        self.assertIsNotNone(osc, why)
        self.assertAlmostEqual(osc["wavelength_m"], 1.5, delta=0.1)


# ----------------------------------------------------------------------
# recommendations
# ----------------------------------------------------------------------
class _StubRecon:
    def __init__(self, ok=True, resid=0.0):
        self._ok = ok
        self.results = {"R3": type("R", (), {"resid": resid})()}

    def valid(self, ident):
        return self._ok


class _StubPlant:
    def __init__(self, **out):
        self.out = out


class _StubUlog:
    path = "stub.ulg"

    def __init__(self, params=None):
        self.params = dict(params or LIVE_PARAMS)
        self.d = {}

    def has(self, *names):
        return False


def _recommender(plant_out, r3_ok=True, params=None):
    return abu.Recommender(_StubUlog(params), _StubRecon(r3_ok),
                           _StubPlant(**plant_out))


class TestRecommender(unittest.TestCase):
    def test_no_yaw_recommendation_is_emitted_when_its_reconstructor_is_broken(self):
        recs = _recommender({"R_opt": 1.12, "G": 1.11, "A_mid": 4.8},
                            r3_ok=False).build()
        self.assertFalse(any(r.param == "RD_MAX_THR_YAW_R" for r in recs),
                         "a broken model must not produce a param value")

    def test_the_yaw_recommendation_is_emitted_when_the_model_is_valid(self):
        """Pairs with the test above: the gate must not block everything."""
        recs = _recommender({"R_opt": 1.12, "G": 1.11, "A_mid": 4.8,
                             "R_opt_lo": 1.10, "R_opt_hi": 1.14}).build()
        got = [r for r in recs if r.param == "RD_MAX_THR_YAW_R"]
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(got[0].rec, 1.12, places=6)

    def test_a_nan_recommendation_never_reaches_the_params_file(self):
        import tempfile
        rc = _recommender({})
        rc.recs = [abu.Rec("RO_SPEED_TH", 0.1, float("nan")),
                   abu.Rec("RD_MAX_THR_YAW_R", 0.95, 1.12)]
        with tempfile.NamedTemporaryFile("r+", suffix=".params") as fh:
            n = rc.emit_params(fh.name)
            body = open(fh.name).read()
        self.assertEqual(n, 1)
        self.assertNotIn("nan", body.lower())
        self.assertIn("RD_MAX_THR_YAW_R", body)

    def test_an_unchanged_value_is_not_written_as_a_change(self):
        import tempfile
        rc = _recommender({})
        rc.recs = [abu.Rec("RO_MAX_THR_SPEED", 1.28, 1.28)]
        with tempfile.NamedTemporaryFile("r+", suffix=".params") as fh:
            self.assertEqual(rc.emit_params(fh.name), 0)

    def test_a_rec_with_no_value_renders_as_no_change_not_as_nan(self):
        row = abu.Rec("RO_SPEED_TH", 0.1, float("nan")).rows()[0]
        self.assertIn("no change proposed", row)
        self.assertNotIn("nan", row.lower())


class TestSpeedCalibrationGate(unittest.TestCase):
    """RO_MAX_THR_SPEED is the FULL-throttle speed, used through the origin.

    A slope fitted over a narrow low-throttle band with a friction intercept
    does not extrapolate to it, and a direct full-throttle field measurement
    outranks the extrapolation.
    """

    def test_a_narrow_throttle_fit_withholds_the_speed_recommendation(self):
        recs = _recommender({"K_thr": 1.18, "K_thr_intercept": -0.01,
                             "thr_lo": 0.08, "thr_hi": 0.29,
                             "K_thr_extrapolated": 1.17}).build()
        r = [x for x in recs if x.param == "RO_MAX_THR_SPEED"][0]
        self.assertIn("WITHHELD", r.conf)
        self.assertAlmostEqual(r.rec, r.now, places=9)
        self.assertIn("29% throttle", r.why.replace("%.0f", ""))

    def test_a_full_range_fit_does_produce_a_speed_recommendation(self):
        """Pairs with the test above: the gate must not block a valid fit."""
        recs = _recommender({"K_thr": 1.10, "K_thr_intercept": -0.02,
                             "thr_lo": 0.10, "thr_hi": 0.95,
                             "K_thr_extrapolated": 1.08}).build()
        r = [x for x in recs if x.param == "RO_MAX_THR_SPEED"][0]
        self.assertNotIn("WITHHELD", r.conf)
        self.assertAlmostEqual(r.rec, 1.08, places=6)

    def test_a_withheld_speed_value_never_reaches_the_params_file(self):
        import tempfile
        rc = _recommender({"K_thr": 1.18, "K_thr_intercept": -0.01,
                           "thr_lo": 0.08, "thr_hi": 0.29,
                           "K_thr_extrapolated": 1.17,
                           "R_opt": 1.12, "G": 1.11, "A_mid": 4.8})
        rc.build()
        with tempfile.NamedTemporaryFile("r+", suffix=".params") as fh:
            rc.emit_params(fh.name)
            body = open(fh.name).read()
        self.assertNotIn("RO_MAX_THR_SPEED", body)
        self.assertIn("RD_MAX_THR_YAW_R", body)


class TestDependencyCheck(unittest.TestCase):
    def test_a_missing_dependency_names_the_working_interpreter(self):
        real = abu._DEPS
        try:
            abu._DEPS = (("a_module_that_does_not_exist", "reads .ulg files"),)
            with self.assertRaises(SystemExit) as cm:
                abu.check_deps()
            msg = str(cm.exception)
        finally:
            abu._DEPS = real
        self.assertIn("/opt/homebrew/bin/python3", msg)
        self.assertIn("ros-replay", msg)

    def test_the_check_passes_when_the_dependency_is_present(self):
        """Pairs with the test above: it must not fire on a good environment."""
        real = abu._DEPS
        try:
            abu._DEPS = (("math", "always present"),)
            abu.check_deps()
        finally:
            abu._DEPS = real


class TestGyroNoiseGate(unittest.TestCase):
    """The dead-band recommendation is only as good as its 'at rest' gate."""

    def _u(self, wheels_still, gz):
        n = len(gz)
        t = np.arange(n) * 0.02

        class U(_StubUlog):
            def has(self, *names):
                return all(x in ("vehicle_angular_velocity", "actuator_motors")
                           for x in names)

        u = U()
        cmd = np.where(wheels_still, 0.0, 0.5)
        u.d = {"vehicle_angular_velocity": {"timestamp": t * 1e6, "xyz[2]": gz},
               "actuator_motors": {"timestamp": t * 1e6,
                                   "control[0]": cmd, "control[1]": cmd}}
        u.t = lambda name: u.d[name]["timestamp"] / 1e6
        u.mode_mask = lambda tt: np.ones(len(tt), bool)
        return u

    def test_gyro_noise_is_measured_only_where_the_wheels_are_commanded_still(self):
        n = 400
        still = np.zeros(n, bool)
        still[:200] = True
        gz = np.concatenate([np.random.default_rng(1).normal(0, 0.01, 200),
                             np.full(200, 0.35)])       # a pivot at 20 deg/s
        rc = abu.Recommender(self._u(still, gz), _StubRecon(), _StubPlant())
        sigma = rc._gyro_noise_at_rest()
        self.assertLess(sigma, 2.0, "the pivot must not be counted as noise")

    def test_gating_on_motion_instead_of_command_would_capture_the_pivot(self):
        """Pairs with the test above: this is the bug that recommended 32 deg/s.

        measured_speed_body_x is dead-banded, so it reads zero during a spot
        turn; gating on it samples the gyro mid-pivot.
        """
        n = 400
        never_still = np.zeros(n, bool)
        gz = np.concatenate([np.random.default_rng(1).normal(0, 0.01, 200),
                             np.full(200, 0.35)])
        rc = abu.Recommender(self._u(never_still, gz), _StubRecon(), _StubPlant())
        self.assertTrue(math.isnan(rc._gyro_noise_at_rest()),
                        "with no commanded-still samples it must refuse, not guess")

    def test_a_noise_estimate_above_the_current_threshold_is_withheld(self):
        rc = _recommender({})
        rc._gyro_noise_at_rest = lambda: 5.0        # 3 sigma = 15 > current 1.0
        recs = rc.build()
        th = [r for r in recs if r.param == "RO_YAW_RATE_TH"]
        self.assertEqual(len(th), 1)
        self.assertEqual(th[0].conf, "WITHHELD")
        self.assertAlmostEqual(th[0].rec, th[0].now, places=9)

    def test_a_credible_noise_estimate_does_produce_a_lower_threshold(self):
        """Pairs with the test above: the withhold must not block a real fix."""
        rc = _recommender({})
        rc._gyro_noise_at_rest = lambda: 0.10       # 3 sigma = 0.3 < current 1.0
        th = [r for r in rc.build() if r.param == "RO_YAW_RATE_TH"]
        self.assertEqual(len(th), 1)
        self.assertLess(th[0].rec, th[0].now)
        self.assertNotEqual(th[0].conf, "WITHHELD")


# ----------------------------------------------------------------------
# CSV export
# ----------------------------------------------------------------------
class TestCsvExport(unittest.TestCase):
    """The CSVs are only useful if they can be joined back together."""

    def _u(self):
        t = np.arange(0, 20, 0.1)

        class U(_StubUlog):
            path = "stub.ulg"

            def has(self, *names):
                return all(n in self.d for n in names)

        u = U()
        n = len(t)
        u.d = {
            "rover_rate_setpoint": {"timestamp": t * 1e6,
                                    "yaw_rate_setpoint": np.sin(t)},
            "rover_rate_status": {"timestamp": t * 1e6,
                                  "adjusted_yaw_rate_setpoint": np.sin(t),
                                  "measured_yaw_rate": 1.1 * np.sin(t),
                                  "pid_yaw_rate_integral": np.zeros(n)},
            "rover_steering_setpoint": {"timestamp": t * 1e6,
                                        "normalized_speed_diff": 0.1 * np.sin(t)},
            "actuator_motors": {"timestamp": t * 1e6,
                                "control[0]": 0.3 + 0.1 * np.sin(t),
                                "control[1]": 0.3 - 0.1 * np.sin(t)},
        }
        u.t = lambda name: u.d[name]["timestamp"] / 1e6
        u.mode_mask = lambda tt: np.ones(len(tt), bool)
        u.yaw_rate_chain = lambda: abu.UlogSide.yaw_rate_chain(u)
        u.speed_chain = lambda: {}
        return u

    def test_every_ulog_csv_carries_an_epoch_column_for_joining(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            abu.export_csv(d, self._u(), None, offset=1_000_000.0)
            for f in os.listdir(d):
                if not f.startswith("ulog__"):
                    continue
                with open(os.path.join(d, f)) as fh:
                    head = fh.readline().strip().split(",")
                self.assertEqual(head[:2], ["t_boot_s", "t_epoch_s"], f)

    def test_epoch_timestamps_keep_full_precision(self):
        """%g would render 1785830534.5 as 1.78583e+09 and destroy the join key."""
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            abu.export_csv(d, self._u(), None, offset=1_785_830_527.0)
            with open(os.path.join(d, "joint_50hz.csv")) as fh:
                fh.readline()
                first = fh.readline().split(",")[1]
        self.assertNotIn("e+", first)
        self.assertGreater(float(first), 1.7e9)
        self.assertIn(".", first)

    def test_the_joint_table_carries_the_whole_chain(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            abu.export_csv(d, self._u(), None, offset=1_000_000.0)
            with open(os.path.join(d, "joint_50hz.csv")) as fh:
                head = fh.readline().strip().split(",")
        for col in ("px4_yaw_rate_sp", "px4_yaw_rate_measured",
                    "px4_speed_diff_cmd", "motor_left", "motor_right",
                    "motor_diff", "motor_common"):
            self.assertIn(col, head, "the chain must reach the motors")

    def test_the_motor_differential_equals_the_steering_command(self):
        """Round-trips the fork sign convention through the export."""
        import tempfile
        import csv as _csv
        with tempfile.TemporaryDirectory() as d:
            abu.export_csv(d, self._u(), None, offset=1_000_000.0)
            rows = list(_csv.DictReader(open(os.path.join(d, "joint_50hz.csv"))))
        got = [(float(r["motor_diff"]), float(r["px4_speed_diff_cmd"]))
               for r in rows if r["motor_diff"] and r["px4_speed_diff_cmd"]]
        self.assertGreater(len(got), 50)
        for diff, cmd in got:
            self.assertAlmostEqual(diff, cmd, places=6)

    def test_an_unaligned_export_leaves_the_epoch_column_empty(self):
        """No offset means no join key -- it must be blank, not a fake number."""
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            abu.export_csv(d, self._u(), None, offset=None)
            with open(os.path.join(d, "ulog__actuator_motors.csv")) as fh:
                fh.readline()
                row = fh.readline().strip().split(",")
        self.assertEqual(row[1], "")


# ----------------------------------------------------------------------
# regression guards against claims the tool used to make
# ----------------------------------------------------------------------
def _source():
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "analyze_bag_ulog.py")) as fh:
        return fh.read()


def test_speed_body_x_setpoint_is_never_read():
    """It is never assigned by the firmware — the logged value is garbage.

    Reading it produced a confident, false claim about the speed controller.
    """
    src = _source()
    # Documenting why we avoid it is fine; indexing it is not.
    for read in ('["speed_body_x_setpoint"]', "['speed_body_x_setpoint']"):
        assert read not in src, "the uninitialised field is being read again"


def test_the_speed_controller_not_engaged_banner_is_gone():
    """Pairs with the test above: the false conclusion must not come back."""
    src = _source()
    assert "not engaged" not in src
    assert "speed controller is not" not in src


def test_the_model_is_documented_as_fork_plus_overlay_not_stock():
    """ver_sw reports the base hash and cannot identify the deployed build."""
    src = _source()
    assert "06309e41a7" in src, "the fork commit must be recorded in the model"
    assert "cannot identify a fork build" in src


if __name__ == "__main__":
    unittest.main(verbosity=2)
