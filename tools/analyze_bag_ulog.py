#!/usr/bin/env python3
"""Joint bag <-> ulog analyser: find WHERE the command chain loses fidelity.

The companion-side bag and the FCU-side ulog each answer half the question.
Alone, neither can say whether a tracking error is the controller, the rate
loop, or the plant.  Joined on a common clock they can:

    S0  companion cmd     bag   /rpp/yaw_rate_body        (NED body, CW+)
    S1  PX4 received      ulog  rover_rate_setpoint
    S2  after slew/limit  ulog  rover_rate_status.adjusted_yaw_rate_setpoint
    S3  loop measured     ulog  rover_rate_status.measured_yaw_rate
    S4  raw gyro          ulog  vehicle_angular_velocity.xyz[2]
    S5  wheels            ulog  wheel_encoders -> (v_l - v_r) / RD_WHEEL_TRACK
    S6  estimate          bag   /mavros/imu/data + local_position

and the same for speed.  Every link reports gain, bias, RMS and lag, so the
stage that introduces the error is the stage where the gain stops being 1.

Clock alignment
---------------
ulog timestamps are PX4 boot microseconds; the bag is unix epoch.  The anchor
is vehicle_gps_position.time_utc_usec, which gives an exact boot->UTC map with
no correlation needed.  A cross-correlation of the two yaw-rate signals then
refines it and, more importantly, VERIFIES it: if the refinement is large the
anchor was wrong and the report says so instead of quietly reporting nonsense.

Sign conventions (asserted, not assumed)
----------------------------------------
    /mavros/imu/data angular_velocity.z   FLU  (z up)    = -(PX4 FRD xyz[2])
    /rpp/yaw_rate_body                    NED body, CW+  = +rover_rate_setpoint

Interpreter
-----------
This needs pyulog, which on this Mac exists ONLY in /opt/homebrew/bin/python3.
The ros-replay mamba env can run the TESTS (pure numpy) but NOT the tool; it
will raise ModuleNotFoundError. check_deps() turns that into a message naming
the right interpreter. Do not pip-install pyulog into ros-replay.

    /opt/homebrew/bin/python3 tools/analyze_bag_ulog.py ...

Usage
-----
    ... --bag <bundle_dir> --ulog <file.ulg>       joint (both layers)
    ... --ulog <file.ulg>                          ulog only (Layer B)
    ... --bag <dir> --ulog-dir <dir>               auto-match by time overlap
    ... --sweep --ulog-dir <dir> [--bag-dir <dir>] one row per run + aggregate
    ... --validate-model --ulog <file.ulg>         is the firmware model valid?
    ... --json out.json                            machine-readable results
    ... --emit-params out.params                   QGC file of accepted changes
    ... --selftest                                 alignment recovery check

A recommendation is emitted ONLY from a stage whose reconstruction is VALID and
whose evidence actually covers the operating range; everything else is reported
as WITHHELD with the reason, and never reaches the .params file.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

np.seterr(invalid="ignore", divide="ignore")

# ----------------------------------------------------------------------
# INTERPRETER: this needs pyulog, which on this Mac lives ONLY in
#   /opt/homebrew/bin/python3
# The ros-replay environment can run the TESTS (they are pure numpy) but not
# the tool itself. Fail with that sentence rather than a bare ImportError, so
# nobody starts installing packages into the wrong environment.
# ----------------------------------------------------------------------
_DEPS = (("pyulog", "reads .ulg files"),)


def check_deps():
    missing = []
    for mod, why in _DEPS:
        try:
            __import__(mod)
        except ImportError:
            missing.append((mod, why))
    if not missing:
        return
    lines = ["missing Python package(s) for this interpreter (%s):" % sys.executable]
    for mod, why in missing:
        lines.append("  - %s   (%s)" % (mod, why))
    lines.append("")
    lines.append("On this Mac the tool runs under the Homebrew interpreter:")
    lines.append("    /opt/homebrew/bin/python3 tools/analyze_bag_ulog.py ...")
    lines.append("The ros-replay env can run the TESTS (pure numpy) but not the")
    lines.append("tool. Do NOT pip-install into ros-replay to work around this.")
    raise SystemExit("\n".join(lines))

R2D = 180.0 / math.pi


# ----------------------------------------------------------------------
# small helpers
# ----------------------------------------------------------------------
def wrap(a):
    """Wrap an angle (or array) to +-pi."""
    return (np.asarray(a, float) + np.pi) % (2 * np.pi) - np.pi


def rms(a) -> float:
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a ** 2))) if len(a) else float("nan")


def gain_through_origin(x, y) -> float:
    """Least-squares slope with the intercept pinned at 0 (a pure scale error)."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    d = float(np.sum(x * x))
    return float(np.sum(x * y) / d) if d > 1e-12 else float("nan")


def lag_seconds(t, x, y, max_lag_s=1.5):
    """Lag of y behind x, by cross-correlation on the shared uniform grid."""
    if len(t) < 8:
        return float("nan"), float("nan")
    dt = float(np.median(np.diff(t)))
    n = int(max_lag_s / dt)
    xc = x - np.mean(x)
    yc = y - np.mean(y)
    if np.std(xc) < 1e-9 or np.std(yc) < 1e-9:
        return float("nan"), float("nan")
    best_k, best_c = 0, -2.0
    for k in range(-n, n + 1):
        a = xc[max(0, -k):len(xc) - max(0, k)]
        b = yc[max(0, k):len(yc) - max(0, -k)]
        if len(a) < 8:
            continue
        c = float(np.corrcoef(a, b)[0, 1])
        if c > best_c:
            best_c, best_k = c, k
    return best_k * dt, best_c


class Link:
    """One stage-to-stage comparison."""

    def __init__(self, name, t, x, y, unit, gate):
        m = np.isfinite(x) & np.isfinite(y) & gate
        self.name = name
        self.unit = unit
        self.n = int(m.sum())
        # a channel that is identically zero carries no information: a gain
        # against it is 0/0, and printing nan invites reading it as a result
        self.degenerate = self.n >= 8 and float(np.max(np.abs(x[m]))) < 1e-6
        if self.n < 8 or self.degenerate:
            self.ok = False
            return
        self.ok = True
        self.gain = gain_through_origin(x[m], y[m])
        self.bias = float(np.mean(y[m] - x[m]))
        self.rms = rms(y[m] - x[m])
        self.corr = float(np.corrcoef(x[m], y[m])[0, 1])
        self.lag, self.lag_corr = lag_seconds(t[m], x[m], y[m])
        self.xmean = float(np.mean(np.abs(x[m])))

    def row(self, scale=1.0):
        if getattr(self, "degenerate", False):
            return f"  {self.name:<38} source channel is identically zero — skipped"
        if not self.ok:
            return f"  {self.name:<38} (insufficient overlap)"
        flag = ""
        if abs(self.gain - 1.0) > 0.05:
            flag = "   <-- gain off by %+.0f%%" % (100 * (self.gain - 1.0))
        return ("  %-38s n=%5d  gain %6.3f  bias %+8.3f  RMS %7.3f %-6s"
                "  lag %+6.3f s  r %5.2f%s"
                % (self.name, self.n, self.gain, scale * self.bias,
                   scale * self.rms, self.unit, self.lag, self.corr, flag))


# ----------------------------------------------------------------------
# ulog side
# ----------------------------------------------------------------------
class UlogSide:
    NAV = {0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
           4: "AUTO_LOITER", 5: "AUTO_RTL", 14: "OFFBOARD", 15: "STAB"}

    def __init__(self, path):
        from pyulog import ULog
        self.path = path
        self.u = ULog(path)
        self.d = {}
        for ds in self.u.data_list:
            # multi-instance topics keep the first instance
            self.d.setdefault(ds.name, ds.data)
        self.params = dict(self.u.initial_parameters)
        self.changed_params = list(self.u.changed_parameters)
        self._utc_map()

    def has(self, *names):
        return all(n in self.d for n in names)

    def t(self, topic):
        return self.d[topic]["timestamp"] / 1e6      # boot seconds

    def _utc_map(self):
        """boot seconds -> unix epoch seconds, from the GPS UTC field."""
        self.utc_offset = None
        self.utc_source = "none"
        g = self.d.get("vehicle_gps_position")
        if g is None or "time_utc_usec" not in g:
            return
        utc = g["time_utc_usec"] / 1e6
        boot = g["timestamp"] / 1e6
        good = utc > 1.0e9                     # 2001+; 0 means "no GPS time yet"
        if good.sum() < 5:
            return
        off = utc[good] - boot[good]
        self.utc_offset = float(np.median(off))
        self.utc_source = "vehicle_gps_position.time_utc_usec"
        self.utc_jitter = float(np.std(off))

    def window_utc(self):
        if self.utc_offset is None:
            return None
        return (self.u.start_timestamp / 1e6 + self.utc_offset,
                self.u.last_timestamp / 1e6 + self.utc_offset)

    # ---- signals, all on boot seconds -------------------------------
    def yaw_rate_chain(self):
        out = {}
        # --- the head of the chain: PX4 steers from the velocity VECTOR ---
        # offboard_control_mode.velocity is the live path, so the companion's
        # yaw_rate never enters. Heading is atan2(vE,vN) -> yaw setpoint ->
        # RO_YAW_P -> rate setpoint. Starting the chain at rover_rate_setpoint
        # would hide the two stages where a heading error is actually born.
        if self.has("differential_velocity_setpoint"):
            out["B3_bearing"] = (self.t("differential_velocity_setpoint"),
                                 self.d["differential_velocity_setpoint"]["bearing"])
        if self.has("rover_attitude_setpoint"):
            out["B4_yaw_sp"] = (self.t("rover_attitude_setpoint"),
                                self.d["rover_attitude_setpoint"]["yaw_setpoint"])
        if self.has("rover_attitude_status"):
            a = self.d["rover_attitude_status"]
            ta = self.t("rover_attitude_status")
            out["B5_yaw_adj"] = (ta, a["adjusted_yaw_setpoint"])
            out["B5_yaw_meas"] = (ta, a["measured_yaw"])
        if self.has("rover_rate_setpoint"):
            out["S1_cmd_rx"] = (self.t("rover_rate_setpoint"),
                                self.d["rover_rate_setpoint"]["yaw_rate_setpoint"])
        if self.has("rover_rate_status"):
            s = self.d["rover_rate_status"]
            tt = self.t("rover_rate_status")
            out["S2_adjusted"] = (tt, s["adjusted_yaw_rate_setpoint"])
            out["S3_measured"] = (tt, s["measured_yaw_rate"])
            if "pid_yaw_rate_integral" in s:
                out["integral"] = (tt, s["pid_yaw_rate_integral"])
        if self.has("rover_steering_setpoint"):
            # B9: the differential CHANNEL command, normalised [-1, 1].
            out["B9_speed_diff"] = (
                self.t("rover_steering_setpoint"),
                self.d["rover_steering_setpoint"]["normalized_speed_diff"])
        if self.has("actuator_motors"):
            # B10: what actually left the mixer. Fork convention:
            # control[0] = LEFT = thr + d, control[1] = RIGHT = thr - d.
            a = self.d["actuator_motors"]
            ta = self.t("actuator_motors")
            out["B10_motor_left"] = (ta, a["control[0]"])
            out["B10_motor_right"] = (ta, a["control[1]"])
            out["B10_motor_diff"] = (ta, 0.5 * (a["control[0]"] - a["control[1]"]))
            out["B10_motor_common"] = (ta, 0.5 * (a["control[0]"] + a["control[1]"]))
        if self.has("vehicle_angular_velocity"):
            a = self.d["vehicle_angular_velocity"]
            out["S4_gyro"] = (self.t("vehicle_angular_velocity"), a["xyz[2]"])
        if self.has("wheel_encoders"):
            w = self.d["wheel_encoders"]
            track = float(self.params.get("RD_WHEEL_TRACK", float("nan")))
            rad = float(self.params.get("EKF2_WENC_RAD", float("nan")))
            # wheel_speed is rad/s on this build; convert with the wheel radius.
            # NED/FRD yaw rate is CW+, so a right turn runs the LEFT wheel
            # faster => omega = (v_left - v_right) / track.
            vl, vr = w["wheel_speed[0]"] * rad, w["wheel_speed[1]"] * rad
            out["S5_wheels"] = (self.t("wheel_encoders"), (vl - vr) / track)
            out["S5_wheel_speed"] = (self.t("wheel_encoders"), 0.5 * (vr + vl))
        return out

    def speed_chain(self):
        out = {}
        K = float(self.params.get("RO_MAX_THR_SPEED", float("nan")))
        if self.has("rover_velocity_status"):
            v = self.d["rover_velocity_status"]
            tt = self.t("rover_velocity_status")
            # NOTE: rover_velocity_status.speed_body_x_setpoint is deliberately
            # NOT read. The firmware never assigns it (it is a local variable in
            # DifferentialVelControl, and the published struct is not
            # zero-initialised), so the logged value is uninitialised stack
            # content - measured constant 1.1e-19. Reading it once produced a
            # confident and false claim that PX4's speed controller was not
            # engaged. The real setpoint is adjusted_speed_body_x_setpoint.
            out["S2_adjusted"] = (tt, v["adjusted_speed_body_x_setpoint"])
            out["S3_measured"] = (tt, v["measured_speed_body_x"])
        if self.has("rover_throttle_setpoint"):
            th = self.d["rover_throttle_setpoint"]["throttle_body_x"]
            out["S2_throttle_implied"] = (self.t("rover_throttle_setpoint"), th * K)
        if self.has("vehicle_local_position"):
            p = self.d["vehicle_local_position"]
            out["S6_estimate"] = (self.t("vehicle_local_position"),
                                  np.hypot(p["vx"], p["vy"]))
        return out

    def mode_mask(self, t):
        """OFFBOARD and armed, sampled at t (boot seconds)."""
        m = np.ones(len(t), bool)
        if self.has("vehicle_status"):
            ns = np.interp(t, self.t("vehicle_status"),
                           self.d["vehicle_status"]["nav_state"].astype(float))
            m &= np.abs(ns - 14.0) < 0.5
        if self.has("actuator_armed"):
            ar = np.interp(t, self.t("actuator_armed"),
                           self.d["actuator_armed"]["armed"].astype(float))
            m &= ar > 0.5
        return m

    def mode_summary(self):
        if not self.has("vehicle_status"):
            return "unknown"
        from collections import Counter
        c = Counter(self.d["vehicle_status"]["nav_state"].tolist())
        return ", ".join("%s %d" % (self.NAV.get(k, k), v)
                         for k, v in sorted(c.items(), key=lambda kv: -kv[1]))


# ----------------------------------------------------------------------
# firmware model — the deployed build, not upstream
# ----------------------------------------------------------------------
#
# Deployed firmware = fork Vetri2425/PX4-Autopilot @ 06309e41a7
#                   = stock v1.16.2 + a 26-file overlay (build_rover.yml).
#
# The ULog's ver_sw reports the BASE hash (54f0455ffcd7 = v1.16.2) and cannot
# identify a fork build. Do not use it to decide which source to model from.
#
# Overlay REPLACES: RoverDifferential, module.yaml, DifferentialVelControl,
#   Roboclaw, EKF2 wheel-encoder fusion, logged_topics, RoverLandDetector,
#   mission_block, rover.px4board.
# Overlay does NOT touch: RoverControl.cpp, DifferentialRateControl,
#   DifferentialPosControl, rovercontrol_params.c  -> stock v1.16.2.
#
# So the yaw-rate feedforward and rate loop below are stock v1.16.2, while the
# inverse kinematics come from the FORK and carry the opposite sign to upstream.
# ----------------------------------------------------------------------
class Firmware:
    """Pure functions. No I/O. Parameterised by the log's own parameters."""

    def __init__(self, params):
        p = params
        self.p = p
        g = lambda k, d=float("nan"): float(p.get(k, d))    # noqa: E731
        self.WT = g("RD_WHEEL_TRACK")
        self.R_yaw = g("RD_MAX_THR_YAW_R")
        self.K_spd = g("RO_MAX_THR_SPEED")
        self.yaw_p = g("RO_YAW_P")
        self.rate_p = g("RO_YAW_RATE_P")
        self.rate_i = g("RO_YAW_RATE_I")
        self.rate_lim = math.radians(g("RO_YAW_RATE_LIM"))
        self.rate_th = math.radians(g("RO_YAW_RATE_TH"))
        self.speed_th = g("RO_SPEED_TH")
        self.speed_lim = g("RO_SPEED_LIM")
        self.trans_drv_trn = g("RD_TRANS_DRV_TRN")
        self.trans_trn_drv = g("RD_TRANS_TRN_DRV")

    # ---- stage models -------------------------------------------------
    @staticmethod
    def bearing_from_velocity(vn, ve):
        """DifferentialVelControl.cpp:141 (fork) — NED bearing of the command."""
        return np.arctan2(ve, vn)

    def yaw_rate_setpoint(self, yaw_sp, yaw):
        """DifferentialAttControl: P on yaw error, clamped to RO_YAW_RATE_LIM."""
        return np.clip(self.yaw_p * wrap(yaw_sp - yaw), -self.rate_lim, self.rate_lim)

    def rate_deadband(self, x):
        """DifferentialRateControl.cpp:69-74 and :149-151 — RO_YAW_RATE_TH.

        Applied to BOTH the measurement and the incoming setpoint, so below the
        threshold the loop is fully open in both directions.
        """
        return np.where(np.abs(x) > self.rate_th, x, 0.0)

    def speed_deadband(self, x):
        """DifferentialVelControl.cpp:103-110 — RO_SPEED_TH on measured speed."""
        return np.where(np.abs(x) > self.speed_th, x, 0.0)

    def ff_speed_diff(self, adj_yaw_rate):
        """RoverControl.cpp:197-202 (stock v1.16.2).

        speed_diff = adj * WT/2 ; then interpolate(-R, R -> -1, 1) == /R clamped.
        """
        return np.clip(adj_yaw_rate * self.WT / 2.0 / self.R_yaw, -1.0, 1.0)

    def steering(self, adj_yaw_rate, meas_yaw_rate, integral=0.0):
        """RoverControl.cpp:194-214 — FF + PID, output clamped to +-1.

        The PID is skipped entirely (and the integral reset) when the adjusted
        setpoint is zero, which is what the dead-band produces.
        """
        ff = self.ff_speed_diff(adj_yaw_rate)
        fb = self.rate_p * (adj_yaw_rate - meas_yaw_rate) + integral
        fb = np.where(np.abs(adj_yaw_rate) > 1e-9, fb, 0.0)
        return np.clip(ff + fb, -1.0, 1.0)

    def inverse_kinematics(self, throttle, d):
        """RoverDifferential.cpp:166-180 — FORK sign convention.

        Fork:  control[0]=left = thr + d,  control[1]=right = thr - d
        Stock: the opposite.  Positive d = RIGHT turn on this vehicle.
        Yaw is prioritised: excess is taken out of throttle, not out of d.
        """
        throttle = np.asarray(throttle, float)
        d = np.asarray(d, float)
        excess = np.maximum(np.abs(throttle) + np.abs(d) - 1.0, 0.0)
        thr = throttle - np.sign(throttle) * excess
        return thr + d, thr - d

    def driving_state(self, heading_err):
        """DifferentialVelControl.cpp:172-176 — hysteresis, not a threshold.

        Returns a bool array, True = SPOT_TURNING. Must be scanned in order.
        """
        he = np.abs(np.asarray(heading_err, float))
        out = np.zeros(len(he), bool)
        spot = False
        for i, e in enumerate(he):
            if not spot and e > self.trans_drv_trn:
                spot = True
            elif spot and e < self.trans_trn_drv:
                spot = False
            out[i] = spot
        return out

    def wheel_yaw_rate(self, v_left, v_right):
        """Yaw rate implied by the wheels, in the fork's convention (CW+)."""
        return (np.asarray(v_left, float) - np.asarray(v_right, float)) / self.WT

    # ---- derived constants -------------------------------------------
    @property
    def A_ideal(self):
        """Plant gain if the wheels lost nothing: rad/s per unit speed-diff."""
        return 2.0 * self.K_spd / self.WT

    @property
    def heading_deadband_floor_rad(self):
        """Heading error that produces a yaw-rate setpoint below RO_YAW_RATE_TH.

        Anything smaller receives ZERO correction: RO_YAW_RATE_TH / RO_YAW_P.
        """
        return self.rate_th / self.yaw_p

    @property
    def yaw_rate_sat_heading_err_rad(self):
        """Heading error at which the yaw-rate setpoint saturates."""
        return self.rate_lim / self.yaw_p

    def closed_loop_gain(self, A):
        """G = omega_measured / omega_setpoint for a plant of gain A."""
        return A * (self.WT / (2.0 * self.R_yaw) + self.rate_p) / (1.0 + A * self.rate_p)

    def R_optimal(self, A):
        """Solving closed_loop_gain(A) == 1 gives exactly R = A * WT / 2."""
        return A * self.WT / 2.0


class Recon:
    """One reconstruction of a logged stage from its inputs."""

    def __init__(self, ident, what, resid, signal_rms, n, floor=0.0, note=""):
        self.id = ident
        self.what = what
        self.resid = resid
        self.signal_rms = signal_rms
        self.n = n
        self.floor = max(floor, 1e-6)
        self.note = note
        self.tol = max(self.floor, 0.02 * signal_rms) if np.isfinite(signal_rms) else self.floor
        self.valid = bool(n >= 30 and np.isfinite(resid) and resid <= self.tol)

    @property
    def verdict(self):
        if not np.isfinite(self.resid):
            return "NO DATA"
        return "VALID" if self.valid else "BROKEN"

    def row(self):
        return ("  %-3s %-42s resid %9.5f  tol %9.5f  n=%5d  %s%s"
                % (self.id, self.what, self.resid, self.tol, self.n,
                   self.verdict, ("  (%s)" % self.note) if self.note else ""))


def _interp_to(src, t_target):
    """Interpolate a (t, y) pair onto t_target, NaN outside the source span."""
    t, y = src
    return np.interp(t_target, t, y, left=np.nan, right=np.nan)


def _step_floor(y):
    """Interpolation-error bound: half the median sample-to-sample change."""
    if len(y) < 3:
        return 0.0
    return 0.5 * float(np.nanmedian(np.abs(np.diff(y))))


class Reconstruction:
    """Rebuild each logged stage from its inputs; residual gates Layer C."""

    def __init__(self, U):
        self.U = U
        self.fw = Firmware(U.params)
        self.results = {}

    def run(self):
        for fn in (self._r3_steering, self._r2_rate_setpoint, self._r4_actuators,
                   self._r1_bearing):
            try:
                r = fn()
            except Exception as exc:                       # noqa: BLE001
                r = Recon(fn.__name__[1:3].upper(), fn.__doc__.splitlines()[0],
                          float("nan"), float("nan"), 0, note=str(exc)[:40])
            if r is not None:
                self.results[r.id] = r
        return self.results

    def valid(self, ident):
        r = self.results.get(ident)
        return bool(r and r.valid)

    # -- R3: the one that matters. Gates every yaw recommendation. ------
    def _r3_steering(self):
        """rate status -> normalized_speed_diff  (FF + PID)"""
        U = self.U
        if not U.has("rover_rate_status", "rover_steering_setpoint"):
            return None
        s = U.d["rover_rate_status"]
        ts = U.t("rover_rate_status")
        adj = s["adjusted_yaw_rate_setpoint"]
        meas = s["measured_yaw_rate"]
        integ = s.get("pid_yaw_rate_integral", np.zeros(len(ts)))
        pred = self.fw.steering(adj, meas, integ)
        td = U.t("rover_steering_setpoint")
        got = _interp_to((td, U.d["rover_steering_setpoint"]["normalized_speed_diff"]), ts)
        m = np.isfinite(got) & np.isfinite(pred) & U.mode_mask(ts)
        if m.sum() < 30:
            return Recon("R3", "rate status -> normalized_speed_diff",
                         float("nan"), float("nan"), int(m.sum()))
        return Recon("R3", "rate status -> normalized_speed_diff",
                     rms(pred[m] - got[m]), rms(got[m]), int(m.sum()),
                     floor=_step_floor(got[m]))

    def _r2_rate_setpoint(self):
        """attitude -> rover_rate_setpoint  (RO_YAW_P)"""
        U = self.U
        if not U.has("rover_rate_setpoint", "rover_attitude_status"):
            return None
        a = U.d["rover_attitude_status"]
        ta = U.t("rover_attitude_status")
        pred = self.fw.yaw_rate_setpoint(a["adjusted_yaw_setpoint"], a["measured_yaw"])
        tr = U.t("rover_rate_setpoint")
        got = _interp_to((tr, U.d["rover_rate_setpoint"]["yaw_rate_setpoint"]), ta)
        m = np.isfinite(got) & np.isfinite(pred) & U.mode_mask(ta)
        if m.sum() < 30:
            return Recon("R2", "attitude -> rate setpoint",
                         float("nan"), float("nan"), int(m.sum()))
        return Recon("R2", "attitude -> rate setpoint",
                     rms(pred[m] - got[m]), rms(got[m]), int(m.sum()),
                     floor=_step_floor(got[m]))

    def _r4_actuators(self):
        """steering -> actuator differential  (FORK inverse kinematics sign)"""
        U = self.U
        if not U.has("actuator_motors", "rover_steering_setpoint"):
            return None
        tm = U.t("actuator_motors")
        d = _interp_to((U.t("rover_steering_setpoint"),
                        U.d["rover_steering_setpoint"]["normalized_speed_diff"]), tm)
        a = U.d["actuator_motors"]
        left, right = a["control[0]"], a["control[1]"]
        # Only the DIFFERENTIAL is reconstructed. RoverDifferential.cpp:128-131
        # applies a SECOND throttle slew (RoverControl::throttleControl) between
        # rover_throttle_setpoint and allocation, so the common-mode term is not
        # a function of the logged throttle setpoint alone. The differential is
        # untouched by that slew and by the saturation trim (which is taken out
        # of throttle, never out of d), so (left-right)/2 == d exactly.
        got = 0.5 * (left - right)
        m = np.isfinite(d) & np.isfinite(got) & U.mode_mask(tm)
        if m.sum() < 30:
            return Recon("R4", "steering -> actuator differential",
                         float("nan"), float("nan"), int(m.sum()))
        resid = rms(d[m] - got[m])
        # A stock-sign build would fit the NEGATED differential instead. Report
        # which one wins so the sign convention is measured, never assumed.
        alt = rms(d[m] + got[m])
        note = ("fork sign (stock would be %.4f)" % alt if resid <= alt
                else "STOCK SIGN: negated fits better (%.5f)" % alt)
        return Recon("R4", "steering -> actuator differential", resid,
                     rms(got[m]), int(m.sum()),
                     floor=_step_floor(got[m]), note=note)

    def _r1_bearing(self):
        """trajectory_setpoint -> differential_velocity_setpoint"""
        U = self.U
        if not U.has("differential_velocity_setpoint", "trajectory_setpoint"):
            return None
        td = U.t("differential_velocity_setpoint")
        dv = U.d["differential_velocity_setpoint"]
        tj = U.d["trajectory_setpoint"]
        tt = U.t("trajectory_setpoint")
        vn = _interp_to((tt, tj["velocity[0]"]), td)
        ve = _interp_to((tt, tj["velocity[1]"]), td)
        pred = self.fw.bearing_from_velocity(vn, ve)
        got = dv["bearing"]
        err = np.abs(wrap(pred - got))
        # the fork flips the bearing by pi when reversing; count, do not fail
        rev = err > math.radians(150)
        err = np.where(rev, np.abs(wrap(pred - got + math.pi)), err)
        m = np.isfinite(err) & U.mode_mask(td) & (np.hypot(vn, ve) > 0.05)
        if m.sum() < 30:
            return Recon("R1", "trajectory -> velocity setpoint",
                         float("nan"), float("nan"), int(m.sum()))
        return Recon("R1", "trajectory -> velocity setpoint", rms(err[m]),
                     rms(got[m]), int(m.sum()),
                     floor=math.radians(5.0),   # 5 Hz vs 10 Hz sampling
                     note="reverse-flipped %.0f%%" % (100 * rev[m].mean()))


class PlantID:
    """Identify the plant gain A: rad/s of yaw per unit normalised speed-diff.

    Three independent estimators, because closed-loop identification is biased
    and a single number would be false precision:

      A_direct     regress measured yaw rate on the commanded speed-diff. Biased
                   low: the yaw rate appears in the loop that produced d.
      A_closedloop back-solve A from the observed setpoint->measured gain G.
                   Biased by the integral term this model neglects.
      A_encoder    same as A_direct but using the WHEELS' own yaw rate, which
                   separates 'the body did not turn' from 'the wheels did not spin'.
      A_ideal      2*RO_MAX_THR_SPEED/RD_WHEEL_TRACK, i.e. no losses at all.
    """

    def __init__(self, U, fw=None):
        self.U = U
        self.fw = fw or Firmware(U.params)
        self.out = {}

    def fit(self):
        U, fw = self.U, self.fw
        o = self.out
        o["A_ideal"] = fw.A_ideal
        if not U.has("rover_rate_status", "rover_steering_setpoint"):
            return o
        ts = U.t("rover_rate_status")
        s = U.d["rover_rate_status"]
        adj, meas = s["adjusted_yaw_rate_setpoint"], s["measured_yaw_rate"]
        d = _interp_to((U.t("rover_steering_setpoint"),
                        U.d["rover_steering_setpoint"]["normalized_speed_diff"]), ts)
        gate = U.mode_mask(ts) & np.isfinite(d)
        # steady = the setpoint is not moving; that is where a scale error lives
        dsp = np.gradient(np.nan_to_num(adj), ts)
        steady = gate & (np.abs(dsp) < 0.05) & (np.abs(adj) > 0.05)
        o["n_steady"] = int(steady.sum())
        if steady.sum() >= 30:
            big = steady & (np.abs(d) > 0.02)
            if big.sum() >= 30:
                o["A_direct"] = gain_through_origin(d[big], meas[big])
            G = gain_through_origin(adj[steady], meas[steady])
            o["G"] = G
            # G = A(WT/2R + Kp)/(1 + A Kp)  ->  A = G / (WT/2R + Kp - G Kp)
            den = fw.WT / (2 * fw.R_yaw) + fw.rate_p - G * fw.rate_p
            o["A_closedloop"] = G / den if abs(den) > 1e-9 else float("nan")
        if U.has("wheel_encoders"):
            w = U.d["wheel_encoders"]
            rad = float(U.params.get("EKF2_WENC_RAD", float("nan")))
            tw = U.t("wheel_encoders")
            vl, vr = w["wheel_speed[0]"] * rad, w["wheel_speed[1]"] * rad
            wyaw = _interp_to((tw, fw.wheel_yaw_rate(vl, vr)), ts)
            m = steady & np.isfinite(wyaw) & (np.abs(d) > 0.02)
            if m.sum() >= 30:
                o["A_encoder"] = gain_through_origin(d[m], wyaw[m])
            # is the wheel radius itself trustworthy? if not, "slip" is a
            # radius error wearing a disguise and the split must be refused
            if U.has("vehicle_local_position"):
                p = U.d["vehicle_local_position"]
                spd = _interp_to((U.t("vehicle_local_position"),
                                  np.hypot(p["vx"], p["vy"])), tw)
                wspd = 0.5 * (vl + vr)
                mm = np.isfinite(spd) & (spd > 0.15) & U.mode_mask(tw)
                if mm.sum() >= 30:
                    o["wheel_radius_scale"] = gain_through_origin(spd[mm], wspd[mm])
        # A_encoder is deliberately NOT averaged in: it is the gain from command
        # to WHEEL yaw rate, so it diagnoses scrub (wheels turned, body did not)
        # but it is not an estimator of the body plant gain that sets R_opt.
        ests = [o[k] for k in ("A_direct", "A_closedloop")
                if k in o and np.isfinite(o[k])]
        if ests:
            o["A_lo"], o["A_hi"] = min(ests), max(ests)
            o["A_mid"] = float(np.mean(ests))
            o["R_opt"] = fw.R_optimal(o["A_mid"])
            o["R_opt_lo"] = fw.R_optimal(o["A_lo"])
            o["R_opt_hi"] = fw.R_optimal(o["A_hi"])
        # open-loop speed calibration, same estimator shape as the yaw one
        if U.has("rover_throttle_setpoint", "rover_velocity_status"):
            tv = U.t("rover_velocity_status")
            meas_v = U.d["rover_velocity_status"]["measured_speed_body_x"]
            thr = _interp_to((U.t("rover_throttle_setpoint"),
                              U.d["rover_throttle_setpoint"]["throttle_body_x"]), tv)
            m = U.mode_mask(tv) & np.isfinite(thr) & (thr > 0.05) & (meas_v > 0.05)
            if m.sum() >= 30:
                p = np.polyfit(thr[m], meas_v[m], 1)
                o["K_thr"] = float(p[0])
                o["K_thr_intercept"] = float(p[1])
                # RO_MAX_THR_SPEED is the speed at FULL throttle, and the
                # firmware uses it through the origin. This fit is a LOCAL slope
                # over whatever throttle the mission happened to use, and it
                # carries a rolling-resistance intercept. Record the span so the
                # recommender can refuse to extrapolate.
                o["thr_lo"], o["thr_hi"] = float(thr[m].min()), float(thr[m].max())
                o["K_thr_extrapolated"] = float(p[0] + p[1])
        # A_ideal is 2*K/WT, so an over-stated RO_MAX_THR_SPEED makes the plant
        # LOOK lossy. If A recomputed from the MEASURED full-throttle speed lands
        # on the fitted A, the yaw path has no fault of its own: one calibration
        # error is inflating the yaw feedforward and slowing the rover at once.
        if "K_thr" in o and "A_mid" in o:
            o["A_from_K_thr"] = 2.0 * o["K_thr"] / fw.WT
            o["speed_yaw_agreement"] = abs(o["A_from_K_thr"] - o["A_mid"]) / o["A_mid"]
            o["speed_yaw_unified"] = bool(o["speed_yaw_agreement"] < 0.05)
        return o

    def report(self):
        o = self.out
        print("\n10. PLANT IDENTIFICATION")
        if "G" not in o:
            print("   not enough steady data to identify the plant")
            return
        parts = ["A_ideal %.3f" % o["A_ideal"]]
        for k, lbl in (("A_direct", "A_direct"), ("A_closedloop", "A_closedloop"),
                       ("A_encoder", "A_encoder")):
            if k in o and np.isfinite(o[k]):
                parts.append("%s %.3f" % (lbl, o[k]))
        print("   " + " | ".join(parts))
        if "A_mid" in o:
            print("   -> A = %.3f  [%.3f .. %.3f]   (%.0f%% of ideal)"
                  % (o["A_mid"], o["A_lo"], o["A_hi"],
                     100 * o["A_mid"] / o["A_ideal"]))
        print("   steady setpoint->measured gain G = %.4f   (n=%d)"
              % (o["G"], o["n_steady"]))
        if "wheel_radius_scale" in o:
            wrs = o["wheel_radius_scale"]
            print("   wheel-speed / EKF-speed scale %.3f" % wrs)
            if abs(wrs - 1.0) > 0.03:
                print("     ** off by >3%: EKF2_WENC_RAD is suspect, so the")
                print("        scrub-vs-motor-scale split is NOT separable **")
            elif "A_encoder" in o and "A_direct" in o:
                scrub = 1 - o["A_direct"] / max(o["A_encoder"], 1e-9)
                motor = 1 - o["A_encoder"] / o["A_ideal"]
                print("     deficit split: tyre scrub %.0f%%  motor scale %.0f%%"
                      % (100 * scrub, 100 * motor))
        if "K_thr" in o:
            print("   open-loop speed: measured = %.3f*throttle %+.3f  "
                  "(RO_MAX_THR_SPEED = %.3f)"
                  % (o["K_thr"], o["K_thr_intercept"], self.fw.K_spd))
            print("     fit spans throttle %.2f..%.2f (%.0f%% of range); at full "
                  "throttle this extrapolates to %.3f m/s"
                  % (o.get("thr_lo", float("nan")), o.get("thr_hi", float("nan")),
                     100 * o.get("thr_hi", float("nan")),
                     o.get("K_thr_extrapolated", float("nan"))))
            if "A_from_K_thr" in o:
                print("   cross-check: A implied by the local speed slope = %.3f "
                      "vs fitted A = %.3f (%.0f%% apart)"
                      % (o["A_from_K_thr"], o["A_mid"],
                         100 * o["speed_yaw_agreement"]))
                if o["speed_yaw_unified"]:
                    print("     => consistent. Note WHY: the friction intercept is")
                    print("        common-mode, so it cancels in the wheel-speed")
                    print("        DIFFERENCE that generates yaw. The yaw path")
                    print("        therefore depends on the local slope, which is")
                    print("        what is measured here -- so R_opt stands even")
                    print("        though the same slope must NOT be extrapolated")
                    print("        to full throttle for RO_MAX_THR_SPEED.")
                else:
                    print("     => the yaw deficit is NOT explained by the speed")
                    print("        slope alone; real scrub/slip remains.")


def deadband_report(U, fw):
    """Section 9: where the loop is switched off, saturated, or spot-turning."""
    print("\n9. DEAD-BANDS, SATURATION, STATE MACHINE")
    if U.has("rover_rate_status", "rover_rate_setpoint"):
        ts = U.t("rover_rate_status")
        s = U.d["rover_rate_status"]
        sp = _interp_to((U.t("rover_rate_setpoint"),
                         U.d["rover_rate_setpoint"]["yaw_rate_setpoint"]), ts)
        gate = U.mode_mask(ts) & np.isfinite(sp)
        n = max(int(gate.sum()), 1)
        sp_zero = gate & (np.abs(sp) < 1e-9)
        meas_zero = gate & (np.abs(s["measured_yaw_rate"]) < 1e-9)
        both = sp_zero & meas_zero
        # longest continuous stretch with the loop fully open
        run = best = 0
        for i, v in enumerate(both):
            run = run + 1 if v else 0
            best = max(best, run)
        dt = float(np.median(np.diff(ts))) if len(ts) > 2 else 0.1
        print("   RO_YAW_RATE_TH %.2f deg/s : setpoint zeroed %.1f%% | "
              "measurement zeroed %.1f%%"
              % (math.degrees(fw.rate_th), 100 * sp_zero.sum() / n,
                 100 * meas_zero.sum() / n))
        print("     both zeroed (loop fully open) %.1f%%, longest dwell %.2f s"
              % (100 * both.sum() / n, best * dt))
        print("     -> heading dead-band floor RO_YAW_RATE_TH/RO_YAW_P = %.3f deg"
              % math.degrees(fw.heading_deadband_floor_rad))
        print("        any heading error below this receives ZERO correction")
        sat = gate & (np.abs(sp) >= fw.rate_lim - 1e-6)
        print("   RO_YAW_RATE_LIM %.0f deg/s : saturates above heading error "
              "%.1f deg; hit %.1f%%"
              % (math.degrees(fw.rate_lim),
                 math.degrees(fw.yaw_rate_sat_heading_err_rad),
                 100 * sat.sum() / n))
    if U.has("rover_velocity_status"):
        tv = U.t("rover_velocity_status")
        v = U.d["rover_velocity_status"]
        g = U.mode_mask(tv)
        nz = max(int(g.sum()), 1)
        mz = g & (np.abs(v["measured_speed_body_x"]) < 1e-9)
        adj = v["adjusted_speed_body_x_setpoint"]
        moving = g & (np.abs(adj) > 0.05)
        print("   RO_SPEED_TH %.2f m/s : measurement zeroed %.1f%% of ticks"
              % (fw.speed_th, 100 * mz.sum() / nz))
        stuck = moving & (np.abs(v["measured_speed_body_x"]) < 1e-9)
        if stuck.sum():
            print("     ** %.1f%% of ticks command motion but read exactly zero speed:"
                  % (100 * stuck.sum() / nz))
            print("        the speed integrator is winding against a phantom error **")
    if U.has("rover_attitude_status", "rover_velocity_status"):
        ta = U.t("rover_attitude_status")
        a = U.d["rover_attitude_status"]
        he = wrap(a["adjusted_yaw_setpoint"] - a["measured_yaw"])
        spot = fw.driving_state(he)
        g = U.mode_mask(ta)
        entries = int(np.sum(np.diff(spot.astype(int)) > 0))
        print("   RD_TRANS_DRV_TRN %.0f deg : SPOT_TURNING %.1f%% of ticks, "
              "%d entries"
              % (math.degrees(fw.trans_drv_trn),
                 100 * (spot & g).sum() / max(int(g.sum()), 1), entries))
    if U.has("rover_throttle_setpoint", "rover_steering_setpoint"):
        tt = U.t("rover_throttle_setpoint")
        thr = U.d["rover_throttle_setpoint"]["throttle_body_x"]
        d = _interp_to((U.t("rover_steering_setpoint"),
                        U.d["rover_steering_setpoint"]["normalized_speed_diff"]), tt)
        g = U.mode_mask(tt) & np.isfinite(d)
        clamp = g & (np.abs(thr) + np.abs(d) > 1.0)
        if g.sum():
            worst = float(np.max((np.abs(thr) + np.abs(d) - 1.0)[clamp])) if clamp.sum() else 0.0
            print("   infeasibility clamp (yaw steals throttle) active %.1f%%, "
                  "worst cut %.3f normalised"
                  % (100 * clamp.sum() / int(g.sum()), worst))


class Geometry:
    """Layer A: did the companion command the right thing?

    Cross-track is measured from the rover's own pose to the PUBLISHED /path,
    which is the only reference the controller cannot grade itself against.
    """

    def __init__(self, B):
        self.B = B
        self.s = B.s

    # -- projection -----------------------------------------------------
    @staticmethod
    def signed_xtrack(path, XY, yaw=None, win_back=0.6, win_fwd=3.0):
        """Signed cross-track, walking forward in arc length. + = RIGHT.

        A nearest-segment search is wrong on these missions: a there-and-back
        path is exactly self-overlapping, so the nearest point flips to the
        return leg. Progressing in arc length and breaking ties with the rover's
        own heading keeps it on the leg actually being driven. Samples whose foot
        is pinned past a leg end are flagged: their residual is along-track
        overshoot, not cross-track error.
        """
        P = np.asarray(path, float)
        S = np.zeros(len(P))
        S[1:] = np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))
        seg = np.diff(P, axis=0)
        L = np.linalg.norm(seg, axis=1)
        keep = L > 1e-9
        A, U_, L2, Sa = P[:-1][keep], seg[keep], L[keep], S[:-1][keep]
        Uh = U_ / L2[:, None]
        n = len(A)
        m = len(XY)
        xt = np.full(m, np.nan)
        ss = np.full(m, np.nan)
        si = np.zeros(m, int)
        off_end = np.zeros(m, bool)
        cur = 0.0
        for k, p in enumerate(XY):
            lo = int(np.searchsorted(Sa, cur - win_back) - 1)
            hi = int(np.searchsorted(Sa, cur + win_fwd))
            lo = max(0, lo)
            hi = min(n - 1, max(hi, lo))
            idx = np.arange(lo, hi + 1)
            w = p - A[idx]
            tt = np.clip(np.einsum("ij,ij->i", w, Uh[idx]), 0.0, L2[idx])
            foot = A[idx] + Uh[idx] * tt[:, None]
            dv = p - foot
            dist = np.linalg.norm(dv, axis=1)
            if yaw is not None:
                hv = np.array([math.cos(yaw[k]), math.sin(yaw[k])])
                aligned = Uh[idx] @ hv > 0.0
                if aligned.any():
                    dist = np.where(aligned, dist, dist + 1e3)
            j = int(np.argmin(dist))
            g = idx[j]
            # NED (n, e): with u = north, right-of-travel is east, giving
            # cross = u_n*dv_e - u_e*dv_n > 0. So + = RIGHT with NO negation.
            # (An ENU (e, n) frame flips this; the scratch version negated
            # because it worked in ENU. Mixing the two silently inverts the
            # sign and shows up as a perception gap of exactly 2x the RMS.)
            cross = Uh[g, 0] * dv[j, 1] - Uh[g, 1] * dv[j, 0]
            xt[k] = np.sign(cross) * dist[j] if cross != 0 else 0.0
            ss[k] = Sa[g] + tt[j]
            si[k] = g
            proj = float((p - A[g]) @ Uh[g])
            if proj > L2[g] + 0.02 or proj < -0.02:
                off_end[k] = True
            cur = ss[k]
        return xt, ss, si, off_end, Uh

    # -- phases ---------------------------------------------------------
    def marking_gate(self):
        """Valve truth, in the documented degrade order.

        /spray/active is geometric INTENT, not the valve; gating on it has
        over-reported marking RMS by ~3x in the field, so the source actually
        used is returned alongside and flagged when it is not a valve signal.
        """
        for name, series, is_valve in (("spray_state", self.s.spray_state, True),
                                       ("spray_commanded", self.s.spray_commanded, True),
                                       ("spray_desired", self.s.spray_desired, False),
                                       ("spray_active", self.s.spray_active, False)):
            if series:
                return name, series, is_valve
        return None, [], False

    def phase_mask(self, t):
        """PIVOT/STOP from segment state; MARK from the valve; else TRANSIT."""
        out = np.array(["TRANSIT"] * len(t), dtype=object)
        if self.B.seg_t is not None:
            st = np.interp(t, self.B.seg_t, self.B.seg_state)
            out[np.abs(st - 3) < 0.5] = "PIVOT"       # CORNER_ALIGN
            out[np.abs(st - 5) < 0.5] = "STOP"        # CORNER_STOP
        name, series, _ = self.marking_gate()
        if series:
            st = np.asarray([r[0] for r in series], float)
            sv = np.asarray([bool(r[1]) for r in series])
            i = np.clip(np.searchsorted(st, t, side="right") - 1, 0, len(sv) - 1)
            mark = sv[i]
            out[mark & (out == "TRANSIT")] = "MARK"
        return out

    @staticmethod
    def oscillation(s_along, x, min_peaks=3):
        """Spatial wavelength and amplitude of the wobble, or None with a reason."""
        if len(x) < 20:
            return None, "too few samples"
        xc = x - np.mean(x)
        sg = np.sign(xc)
        sg[sg == 0] = 1
        idx = np.where(np.diff(sg) != 0)[0]
        if len(idx) < min_peaks:
            return None, "fewer than %d zero crossings" % min_peaks
        half = np.diff(s_along[idx])
        half = half[np.isfinite(half) & (half > 0)]
        if len(half) < 2:
            return None, "no usable half-cycles"
        amps = [np.max(np.abs(xc[a:b + 1])) for a, b in zip(idx[:-1], idx[1:])]
        return dict(wavelength_m=2 * float(np.mean(half)),
                    cycles=len(idx) / 2.0,
                    half_amp_cm=100 * float(np.mean(amps)),
                    pk2pk_cm=100 * float(np.max(xc) - np.min(xc)),
                    detrended_rms_cm=100 * rms(xc)), None

    # -- report ---------------------------------------------------------
    def report(self, fw):
        s = self.s
        if not s.pose or not s.paths:
            print("\n2. TRUE CROSS-TRACK — no pose or no /path in this bag")
            return None
        pose = np.asarray(s.pose, float)
        t, N, E, yaw = pose[:, 0], pose[:, 1], pose[:, 2], pose[:, 3]
        XY = np.column_stack([N, E])
        # the path live at each instant, never "the longest one"
        ptimes = np.asarray([p[0] for p in s.paths], float)
        which = np.clip(np.searchsorted(ptimes, t, side="right") - 1, 0, len(ptimes) - 1)
        xt = np.full(len(t), np.nan)
        s_al = np.full(len(t), np.nan)
        off = np.zeros(len(t), bool)
        tang = np.full(len(t), np.nan)
        for pi in np.unique(which):
            poly = np.asarray(s.paths[pi][1], float)
            if len(poly) < 2:
                continue
            m = which == pi
            a, b, si, oe, Uh = self.signed_xtrack(poly, XY[m], yaw[m])
            xt[m], s_al[m], off[m] = a, b, oe
            tang[m] = np.arctan2(Uh[si, 1], Uh[si, 0])
        ph = self.phase_mask(t)
        gate, gname, is_valve = self.marking_gate()[0], None, self.marking_gate()[2]

        print("\n2. TRUE CROSS-TRACK   pose vs the PUBLISHED /path   (cm, + = RIGHT)")
        print("   marking gate: %s%s" % (self.marking_gate()[0],
                                         "" if is_valve else "  ** NOT a valve "
                                         "signal - this is INTENT and over-reports **"))
        print("   %-10s %6s %7s %7s %7s %8s %8s %7s"
              % ("phase", "n", "RMS", "mean", "p95", "LEFT", "RIGHT", "band"))
        stats = {}
        for name in ("TRANSIT", "MARK", "PIVOT", "STOP"):
            m = (ph == name) & np.isfinite(xt) & ~off
            if m.sum() < 20:
                continue
            v = 100 * xt[m]
            stats[name] = dict(n=int(m.sum()), rms=rms(v), mean=float(np.mean(v)),
                               p95=float(np.percentile(np.abs(v), 95)),
                               left=float(np.min(v)), right=float(np.max(v)))
            print("   %-10s %6d %7.2f %+7.2f %7.2f %8.2f %8.2f %7.2f"
                  % (name, m.sum(), stats[name]["rms"], stats[name]["mean"],
                     stats[name]["p95"], stats[name]["left"], stats[name]["right"],
                     stats[name]["right"] - stats[name]["left"]))
        if off.sum():
            print("   %d samples excluded: foot pinned past a leg end (that residual"
                  " is along-track overshoot, not cross-track)" % off.sum())

        # what the controller BELIEVED, vs the truth above
        if self.B.dbg is not None and "rpp_xtrack" in self.B.sig:
            dt_, dv_ = self.B.sig["rpp_xtrack"]
            believed = np.interp(t, dt_, dv_)
            m = (ph == "MARK") & np.isfinite(xt) & ~off & np.isfinite(believed)
            if m.sum() > 20:
                print("   RPP-reported (debug[0], vs its CONDITIONED path) MARK RMS "
                      "%.2f cm -> perception gap %.2f cm"
                      % (100 * rms(believed[m]), 100 * rms(believed[m] - xt[m])))

        print("\n3. HEADING (deg)   nose vs the path tangent it is tracking")
        herr = wrap(yaw - tang)
        for name in ("TRANSIT", "MARK"):
            m = (ph == name) & np.isfinite(herr) & ~off
            if m.sum() < 20:
                continue
            v = R2D * herr[m]
            print("   %-10s n=%5d  RMS %6.2f  mean %+6.2f  p95 %6.2f"
                  % (name, m.sum(), rms(v), np.mean(v),
                     np.percentile(np.abs(v), 95)))
        if s.gps_yaw:
            g = np.asarray([(r[0], r[1]) for r in s.gps_yaw], float)
            print("   INDEPENDENT dual-antenna GPS yaw present (n=%d) — usable as a"
                  " non-circular heading reference" % len(g))
        else:
            print("   no dual-antenna GPS yaw in this bag: heading noise CANNOT be")
            print("   graded independently (comparing the EKF to itself is circular)")

        print("\n4. WOBBLE (MARK span)")
        m = (ph == "MARK") & np.isfinite(xt) & ~off
        if m.sum() > 20:
            osc, why = self.oscillation(s_al[m], xt[m])
            if osc:
                v = float(np.mean(np.hypot(np.gradient(N[m], t[m]),
                                           np.gradient(E[m], t[m]))))
                print("   detrended RMS %.2f cm   pk-pk %.2f cm   %.1f cycles"
                      % (osc["detrended_rms_cm"], osc["pk2pk_cm"], osc["cycles"]))
                print("   spatial wavelength %.2f m -> %.3f Hz at v=%.2f m/s"
                      % (osc["wavelength_m"],
                         v / osc["wavelength_m"] if osc["wavelength_m"] > 0 else 0, v))
                omega = 2 * math.pi * v / osc["wavelength_m"] if osc["wavelength_m"] else float("nan")
                sep = fw.yaw_p / omega if omega else float("nan")
                print("   bandwidth separation RO_YAW_P/omega_obs = %.2f" % sep)
                if sep < 3:
                    print("     ** below 3: the inner heading loop and the outer")
                    print("        lateral loop are NOT separable, so the textbook")
                    print("        lookahead formula does not apply here. Treat any")
                    print("        single-knob lookahead advice as unsupported. **")
            else:
                print("   no oscillation measured: %s" % why)
        return stats


class Rec:
    """One parameter recommendation, with its evidence and its limits."""

    def __init__(self, param, now, rec, lo=None, hi=None, model="", why="",
                 effect="", risk="", conf="MEDIUM", rank=0.0):
        self.param, self.now, self.rec = param, now, rec
        self.lo, self.hi = lo, hi
        self.model, self.why, self.effect = model, why, effect
        self.risk, self.conf, self.rank = risk, conf, rank

    def band(self):
        if self.lo is None or self.hi is None:
            return ""
        return "  [%.3f .. %.3f]" % (self.lo, self.hi)

    def rows(self):
        if not np.isfinite(self.rec) or abs(self.rec - self.now) < 1e-9:
            out = ["  %-18s %8.3f -> (no change proposed)   %s"
                   % (self.param, self.now, self.conf)]
        else:
            out = ["  %-18s %8.3f -> %8.3f%s   %s"
                   % (self.param, self.now, self.rec, self.band(), self.conf)]
        out.append("      evidence : %s" % self.why)
        out.append("      model    : %s" % self.model)
        if self.effect:
            out.append("      effect   : %s" % self.effect)
        if self.risk:
            out.append("      risk     : %s" % self.risk)
        return out


class Recommender:
    """Turn measured defects into parameter changes, gated on model validity."""

    def __init__(self, U, rec, plant, fw=None):
        self.U, self.rec, self.plant = U, rec, plant
        self.fw = fw or Firmware(U.params)
        self.recs = []

    def build(self):
        U, fw, o = self.U, self.fw, self.plant.out

        # D1 — yaw-rate feedforward scale.
        if self.rec.valid("R3") and "R_opt" in o:
            g = o.get("G", float("nan"))
            self.recs.append(Rec(
                "RD_MAX_THR_YAW_R", fw.R_yaw, o["R_opt"],
                o.get("R_opt_lo"), o.get("R_opt_hi"),
                model="G = A(WT/2R + Kp)/(1 + A*Kp); G=1 gives R = A*WT/2",
                why="steady setpoint->measured gain %.4f, fitted plant A = %.3f"
                    % (g, o.get("A_mid", float("nan"))),
                effect="removes the %.0f%% steady yaw over-rotation"
                       % (100 * (g - 1)) if np.isfinite(g) else "",
                risk="NOT RO_YAW_RATE_P: raising Kp also reaches G=1 but moves "
                     "loop bandwidth. R scales the feedforward only.",
                conf="HIGH (R3 residual %.5f)" % self.rec.results["R3"].resid,
                rank=3.0))

        # D5 — speed calibration. Only proposable if the fit actually reaches
        # full throttle; otherwise it is an extrapolation and must be withheld.
        if "K_thr" in o and np.isfinite(o["K_thr"]):
            hi = o.get("thr_hi", 0.0)
            span_ok = hi >= 0.60
            why = ("local slope %.3f over throttle %.2f-%.2f with a %+.3f m/s "
                   "friction intercept" % (o["K_thr"], o.get("thr_lo", 0.0), hi,
                                           o.get("K_thr_intercept", 0.0)))
            if span_ok:
                self.recs.append(Rec(
                    "RO_MAX_THR_SPEED", fw.K_spd, o["K_thr_extrapolated"],
                    model="v = K*throttle + b measured up to full throttle",
                    why=why, conf="MEDIUM", rank=2.5,
                    effect="fixes the speed shortfall",
                    risk="moves the throttle slew (RO_ACCEL_LIM/K) and the "
                         "normalised infeasibility clamp."))
            else:
                self.recs.append(Rec(
                    "RO_MAX_THR_SPEED", fw.K_spd, fw.K_spd,
                    model="RO_MAX_THR_SPEED is the speed at FULL throttle, used "
                          "through the origin; this fit is a LOCAL slope",
                    why=why + ("; the fit reaches only %.0f%% throttle, so "
                               "extrapolating it to 1.0 (%.3f m/s) is not "
                               "evidence about full throttle. A direct "
                               "full-throttle measurement outranks it."
                               % (100 * hi, o.get("K_thr_extrapolated", float("nan")))),
                    effect="none: measure full-throttle speed directly instead",
                    risk="changing this rescales EVERY throttle command and the "
                         "slew rate, so it is not a cheap A/B.",
                    conf="WITHHELD (extrapolation)", rank=0.2))

        # D3 — yaw-rate dead-band. Fit the replacement from the gyro at rest.
        floor_deg = math.degrees(fw.heading_deadband_floor_rad)
        sigma = self._gyro_noise_at_rest()
        rec_th = 3 * sigma if np.isfinite(sigma) else float("nan")
        # Only recommend LOWERING it. A fitted value above the current setting
        # means the noise estimate is contaminated (the rover was not actually
        # still), not that the dead-band should grow.
        if np.isfinite(rec_th) and rec_th >= math.degrees(fw.rate_th):
            self.recs.append(Rec(
                "RO_YAW_RATE_TH", math.degrees(fw.rate_th),
                math.degrees(fw.rate_th),
                model="3 sigma of the gyro while the wheels are commanded still",
                why="floor is %.2f deg of UNCORRECTED heading error, but the "
                    "fitted noise (%.2f deg/s) is NOT below the current setting "
                    "-- no still period long enough to trust. NOT CHANGED."
                    % (floor_deg, sigma),
                effect="none: measure a proper stationary armed segment first",
                conf="WITHHELD", rank=0.1))
        elif np.isfinite(floor_deg) and floor_deg > 0.2 and np.isfinite(rec_th):
            self.recs.append(Rec(
                "RO_YAW_RATE_TH", math.degrees(fw.rate_th), rec_th,
                model="heading floor = RO_YAW_RATE_TH / RO_YAW_P; replacement is "
                      "3 sigma of the gyro at rest",
                why="floor is %.2f deg of UNCORRECTED heading error; measured "
                    "gyro noise at rest sigma = %s"
                    % (floor_deg,
                       "%.3f deg/s" % sigma if np.isfinite(sigma) else "not measurable"),
                effect="lowers the heading floor to %.2f deg"
                       % (rec_th / fw.yaw_p if np.isfinite(rec_th) else float("nan")),
                risk="the dead-band exists to stop motor chatter at standstill; "
                     "3 sigma is what preserves that.",
                conf="HIGH for the floor arithmetic, MEDIUM for the cm effect",
                rank=2.0 if floor_deg > 0.5 else 1.0))

        # D4 — speed dead-band, only when it actually bites.
        stuck = self._speed_deadband_bite()
        if stuck is not None and stuck > 0.02:
            # No value is proposed: the replacement must come from the speed
            # estimate's own noise at rest, which needs a stationary armed
            # segment this log may not contain. A diagnostic without a number
            # is honest; a nan in a .params file is not.
            self.recs.append(Rec(
                "RO_SPEED_TH", fw.speed_th, float("nan"),
                model="measured speed reads exactly 0 below the threshold",
                why="%.1f%% of ticks command motion but read zero speed, so the "
                    "speed integrator winds against a phantom error" % (100 * stuck),
                effect="restores speed feedback at marking speed",
                risk="set from 3 sigma of the speed estimate at rest; too low "
                     "re-admits noise into the integrator.",
                conf="MEDIUM", rank=1.5))
        self.recs.sort(key=lambda r: -r.rank)
        return self.recs

    def _gyro_noise_at_rest(self):
        """Gyro sigma while the wheels are commanded to stand still.

        The gate MUST be the commanded actuator output, not measured speed:
        measured_speed_body_x is dead-banded by RO_SPEED_TH and reads exactly
        zero during a spot turn, so gating on it samples the gyro mid-pivot and
        reports the turn rate as 'noise' (measured: 10.6 deg/s, which would
        recommend raising the threshold to 32 deg/s).
        """
        U = self.U
        if not U.has("vehicle_angular_velocity", "actuator_motors"):
            return float("nan")
        tg = U.t("vehicle_angular_velocity")
        gz = U.d["vehicle_angular_velocity"]["xyz[2]"]
        a = U.d["actuator_motors"]
        ta = U.t("actuator_motors")
        l = _interp_to((ta, a["control[0]"]), tg)
        r = _interp_to((ta, a["control[1]"]), tg)
        rest = (np.isfinite(l) & np.isfinite(r)
                & (np.abs(l) < 0.02) & (np.abs(r) < 0.02))
        if U.has("actuator_armed"):
            rest &= np.interp(tg, U.t("actuator_armed"),
                              U.d["actuator_armed"]["armed"].astype(float)) > 0.5
        if rest.sum() < 50:
            return float("nan")
        return float(np.degrees(np.std(gz[rest])))

    def _speed_deadband_bite(self):
        U = self.U
        if not U.has("rover_velocity_status"):
            return None
        v = U.d["rover_velocity_status"]
        g = U.mode_mask(U.t("rover_velocity_status"))
        if not g.sum():
            return None
        moving = g & (np.abs(v["adjusted_speed_body_x_setpoint"]) > 0.05)
        stuck = moving & (np.abs(v["measured_speed_body_x"]) < 1e-9)
        return float(stuck.sum()) / float(g.sum())

    def report(self):
        print("\n11. RECOMMENDATIONS   (ranked; apply in order, re-measure between)")
        if not self.recs:
            print("   no supported recommendation from this log")
            return
        for i, r in enumerate(self.recs, 1):
            print("\n  #%d" % i)
            for ln in r.rows():
                print(ln)
        blocked = [k for k in ("R3",) if not self.rec.valid(k)]
        if blocked:
            print("\n   SUPPRESSED: %s BROKEN, so yaw recommendations are withheld."
                  % ", ".join(blocked))

    def emit_params(self, path):
        """QGC-loadable file containing ONLY the recommended changes."""
        rows = [r for r in self.recs if np.isfinite(r.rec) and
                abs(r.rec - r.now) > 1e-6]
        with open(path, "w") as fh:
            fh.write("# Recommended parameter changes\n")
            fh.write("# Generated by tools/analyze_bag_ulog.py from %s\n"
                     % os.path.basename(self.U.path))
            fh.write("# REVIEW BEFORE LOADING. QGC on the Mac is source of truth;\n")
            fh.write("# never push these from the Jetson.\n")
            fh.write("# vehicle_id\tcomponent_id\tname\tvalue\ttype\n")
            for r in rows:
                fh.write("# %s: %s -> %s   %s\n"
                         % (r.param, r.now, r.rec, r.why))
                fh.write("1\t1\t%s\t%.6f\t9\n" % (r.param, r.rec))
        return len(rows)


def export_csv(out_dir, U, B, offset, hz=50.0):
    """Dump everything, twice: per-topic at native rate, and one joint table.

    Per-topic files are what `ulog2csv` gives you, with one addition that makes
    them worth having here: every ULog file carries a `t_epoch` column computed
    from the measured clock offset, so it can be joined to the bag directly
    without redoing the alignment.

    The joint table is the whole chain on one uniform grid: what the RPP
    commanded, what PX4 made of it, what left the mixer, and what the vehicle
    actually did. Resampled with a zero-order hold, which is what a control
    loop actually sees between samples.
    """
    os.makedirs(out_dir, exist_ok=True)
    written = []

    def hold(sig, grid_t):
        t, y = sig
        t = np.asarray(t, float)
        if len(t) == 0:
            return np.full(len(grid_t), np.nan)
        i = np.clip(np.searchsorted(t, grid_t, side="right") - 1, 0, len(y) - 1)
        out = np.asarray(y, float)[i]
        out[grid_t < t[0]] = np.nan
        return out

    # ---- per-topic, native rate -------------------------------------
    for name, data in sorted(U.d.items()):
        keys = [k for k in data.keys() if k != "timestamp"]
        if not keys:
            continue
        t_boot = data["timestamp"] / 1e6
        path = os.path.join(out_dir, "ulog__%s.csv" % name)
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_boot_s", "t_epoch_s"] + keys)
            cols = [np.asarray(data[k], float) for k in keys]
            for i in range(len(t_boot)):
                w.writerow(["%.6f" % t_boot[i],
                            "%.6f" % (t_boot[i] + offset) if offset else "",
                            *["%.6g" % c[i] for c in cols]])
        written.append(path)

    if B is not None:
        for name, series, cols in (
                ("pose", B.s.pose, ["n", "e", "yaw_ned"]),
                ("vel_cmd_ned", B.s.vel_cmd, ["v_n", "v_e"]),
                ("vel_meas", B.s.vel_meas, ["speed"]),
                ("yaw_rate_cmd", B.s.yaw_rate, ["yaw_rate_body"]),
                ("segment_debug", B.s.seg, ["state", "heading_err"]),
                ("spray_state", B.s.spray_state, ["on"]),
                ("setpoint", B.s.setpoint, ["type_mask"])):
            if not series:
                continue
            path = os.path.join(out_dir, "bag__%s.csv" % name)
            with open(path, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t_epoch_s"] + cols)
                for row in series:
                    w.writerow(["%.6f" % row[0]]
                               + ["%.6g" % float(v) for v in row[1:1 + len(cols)]])
            written.append(path)
        if B.dbg is not None:
            path = os.path.join(out_dir, "bag__rpp_debug.csv")
            with open(path, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["t_epoch_s"] + ["d%02d_%s" % (i, RPP_DEBUG_NAMES.get(i, ""))
                                            for i in range(B.dbg.shape[1])])
                for i in range(len(B.dbg_t)):
                    w.writerow(["%.6f" % B.dbg_t[i]]
                               + ["%.6g" % v for v in B.dbg[i]])
            written.append(path)

    # ---- the joint table --------------------------------------------
    yc = U.yaw_rate_chain()
    sc = U.speed_chain()
    lo = max(s[0][0] for s in yc.values())
    hi = min(s[0][-1] for s in yc.values())
    gb = np.arange(lo, hi, 1.0 / hz)                 # boot seconds
    ge = gb + offset if offset else np.full(len(gb), np.nan)

    cols = [("t_boot_s", gb), ("t_epoch_s", ge)]
    # --- what the RPP commanded (outside loop, companion side) ---
    if B is not None and offset:
        for key, label in (("S0_vel_n", "rpp_cmd_vel_n"),
                           ("S0_vel_e", "rpp_cmd_vel_e"),
                           ("S0_speed_cmd", "rpp_cmd_speed"),
                           ("S0_bearing_cmd", "rpp_cmd_bearing_ned"),
                           ("S0_yaw_rate_cmd", "rpp_cmd_yaw_rate_UNUSED_BY_PX4"),
                           ("rpp_xtrack", "rpp_xtrack_own"),
                           ("rpp_lookahead", "rpp_lookahead")):
            if key in B.sig:
                cols.append((label, hold(B.sig[key], ge)))
        if B.s.pose:
            p = np.asarray(B.s.pose, float)
            for j, label in ((1, "pose_n"), (2, "pose_e"), (3, "pose_yaw_ned")):
                cols.append((label, hold((p[:, 0], p[:, j]), ge)))
        if B.seg_t is not None:
            cols.append(("seg_state", hold((B.seg_t, B.seg_state), ge)))
        if B.s.spray_state:
            sp = np.asarray([(r[0], float(bool(r[1]))) for r in B.s.spray_state], float)
            cols.append(("spray_on", hold((sp[:, 0], sp[:, 1]), ge)))
    # --- what PX4 made of it (outer heading loop) ---
    for key, label in (("B3_bearing", "px4_bearing_sp"),
                       ("B4_yaw_sp", "px4_yaw_sp"),
                       ("B5_yaw_adj", "px4_yaw_sp_adj"),
                       ("B5_yaw_meas", "px4_yaw_measured"),
                       # --- inner rate loop ---
                       ("S1_cmd_rx", "px4_yaw_rate_sp"),
                       ("S2_adjusted", "px4_yaw_rate_sp_adj"),
                       ("S3_measured", "px4_yaw_rate_measured"),
                       ("S4_gyro", "gyro_yaw_rate"),
                       # --- actuator ---
                       ("B9_speed_diff", "px4_speed_diff_cmd"),
                       ("B10_motor_left", "motor_left"),
                       ("B10_motor_right", "motor_right"),
                       ("B10_motor_diff", "motor_diff"),
                       ("B10_motor_common", "motor_common"),
                       # --- achieved ---
                       ("S5_wheels", "wheel_implied_yaw_rate"),
                       ("S5_wheel_speed", "wheel_speed_mean")):
        if key in yc:
            cols.append((label, hold(yc[key], gb)))
    for key, label in (("S2_adjusted", "px4_speed_sp_adj"),
                       ("S3_measured", "px4_speed_measured"),
                       ("S2_throttle_implied", "throttle_implied_speed"),
                       ("S6_estimate", "ekf_speed")):
        if key in sc:
            cols.append((label, hold(sc[key], gb)))
    if U.has("rover_throttle_setpoint"):
        cols.append(("px4_throttle_sp",
                     hold((U.t("rover_throttle_setpoint"),
                           U.d["rover_throttle_setpoint"]["throttle_body_x"]), gb)))
    cols.append(("offboard_armed", U.mode_mask(gb).astype(float)))

    path = os.path.join(out_dir, "joint_%dhz.csv" % int(hz))
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        names = [c[0] for c in cols]
        w.writerow(names)
        arrs = [np.asarray(c[1], float) for c in cols]
        # epoch seconds need ~10 significant digits before the decimal point;
        # %g would render them as 1.78583e+09 and destroy the join key.
        fmt = ["%.6f" if n.startswith("t_") else "%.6g" for n in names]
        for i in range(len(gb)):
            w.writerow(["" if not np.isfinite(a[i]) else f % a[i]
                        for a, f in zip(arrs, fmt)])
    written.append(path)

    with open(os.path.join(out_dir, "README.txt"), "w") as fh:
        fh.write(_CSV_README % dict(
            ulog=U.path, offset=("%.6f" % offset) if offset else "NOT ALIGNED",
            hz=int(hz), n=len(written)))
    return written


RPP_DEBUG_NAMES = {
    0: "xtrack_m", 1: "heading_err_rad", 2: "lookahead_m", 3: "speed_cmd",
    4: "kappa", 5: "dist_to_goal", 6: "pose_age_ms", 7: "state_code",
    8: "ld_raw_m", 9: "kappa_speed", 10: "yaw_rate_cmd", 38: "mission_speed",
    39: "spray_active", 40: "profile_code",
}

_CSV_README = """CSV export from tools/analyze_bag_ulog.py
=========================================
source ulog : %(ulog)s
clock offset: t_epoch = t_boot + %(offset)s
files       : %(n)d

ulog__<topic>.csv   one per uORB topic, NATIVE rate (this is what ulog2csv
                    gives you). The extra t_epoch_s column is computed from the
                    measured clock offset above, so these join to the bag files
                    directly -- no need to redo the alignment.
bag__<name>.csv     companion-side topics, native rate, already in epoch time.
joint_%(hz)dhz.csv      the whole chain on one uniform grid, zero-order hold.

THE CHAIN, in joint order
-------------------------
  rpp_cmd_vel_n/e         what the companion commanded (NED velocity vector)
  rpp_cmd_bearing_ned     its bearing -- THIS is what PX4 steers from
  rpp_cmd_yaw_rate_...    published, but PX4 DISCARDS it in velocity mode
  px4_bearing_sp          PX4's decode of the commanded bearing
  px4_yaw_sp / _adj       heading setpoint, before and after limiting
  px4_yaw_measured        achieved heading      -> OUTER LOOP error is
                                                   px4_yaw_sp_adj - px4_yaw_measured
  px4_yaw_rate_sp         = RO_YAW_P * that error, clamped
  px4_yaw_rate_sp_adj     after the accel/decel slew
  px4_yaw_rate_measured   achieved yaw rate     -> INNER LOOP error is
                                                   px4_yaw_rate_sp_adj - measured
  px4_speed_diff_cmd      the differential CHANNEL command, normalised
  motor_left / motor_right   what left the mixer (fork: L = thr+d, R = thr-d)
  motor_diff              (L-R)/2, equals px4_speed_diff_cmd unless clipped
  motor_common            (L+R)/2, the throttle after the second slew
  wheel_implied_yaw_rate  (v_L - v_R)/RD_WHEEL_TRACK from the encoders
  gyro_yaw_rate           the IMU, pre dead-band

CONVENTIONS
-----------
  all angles rad, NED, CW positive. Yaw rates rad/s, CW positive.
  cross-track + = RIGHT of travel.
  motor commands normalised [-1, 1]; positive diff = RIGHT turn (fork sign).
  px4_yaw_rate_measured is DEAD-BANDED by RO_YAW_RATE_TH; gyro_yaw_rate is not.
  px4_speed_measured is DEAD-BANDED by RO_SPEED_TH.
  Empty cell = no sample yet on that channel at that time.
"""


def targets_report(geo_stats, U, fw, B):
    """Section 12: are we at target, and is the target even verifiable here?"""
    print("\n12. TARGETS")
    if not geo_stats or "MARK" not in geo_stats:
        print("   no MARK span measured — cross-track targets cannot be graded")
    else:
        m = geo_stats["MARK"]
        for label, val, tgt in (("true xtrack RMS, MARK", m["rms"], 2.0),
                                ("  stretch target", m["rms"], 1.0)):
            print("   %-26s %6.2f cm   target <= %.1f   %s"
                  % (label, val, tgt, "PASS" if val <= tgt else "FAIL"))
        print("   %-26s %6.2f cm   (L %+.2f .. R %+.2f)"
              % ("L/R band, MARK", m["right"] - m["left"], m["left"], m["right"]))

    hdg = _tracking_heading_rms(U)
    floor = math.degrees(fw.heading_deadband_floor_rad)
    independent = bool(B is not None and B.s.gps_yaw)
    if np.isfinite(hdg):
        verdict = "PASS" if hdg <= 1.0 else "FAIL"
        if not independent:
            verdict = "UNVERIFIABLE (EKF graded against itself is circular)"
        print("   %-26s %6.2f deg  target <= 1.0   %s"
              % ("heading RMS, tracking", hdg, verdict))
        print("     of which the dead-band floor RO_YAW_RATE_TH/RO_YAW_P is %.2f deg"
              % floor)
        if floor > 0.5:
            print("     -> %.0f%% of the 1 deg budget is spent before any noise is"
                  % (100 * floor / 1.0))
            print("        considered. The target is not reachable until"
                  " RO_YAW_RATE_TH comes down.")
    if not independent:
        print("   no independent heading reference (dual-antenna GPS yaw) in this")
        print("   bag, so the heading target is reported as UNVERIFIABLE, not PASS.")
    print("   NOTE: these are ONE run. A single pass is not a passing vehicle --")
    print("   use --sweep for the run-to-run distribution before claiming a target")
    print("   is met; the best run and the mean are usually different verdicts.")


def validate_model_report(U):
    """Section 6 standalone: is the firmware model trustworthy for this log?"""
    fw = Firmware(U.params)
    print("=" * 100)
    print("FIRMWARE MODEL VALIDATION")
    print("=" * 100)
    print("ulog        : %s" % U.path)
    print("ver_sw      : %s  <- BASE hash only; cannot identify a fork build"
          % U.u.msg_info_dict.get("ver_sw", "?"))
    print("modelled as : stock v1.16.2 rate/FF  +  fork overlay inverse kinematics")
    print("params      : RD_MAX_THR_YAW_R=%.3f  RD_WHEEL_TRACK=%.3f  RO_MAX_THR_SPEED=%.3f"
          % (fw.R_yaw, fw.WT, fw.K_spd))
    print("              RO_YAW_P=%.2f  RO_YAW_RATE_P=%.3f  RO_YAW_RATE_I=%.3f"
          % (fw.yaw_p, fw.rate_p, fw.rate_i))
    print("              RO_YAW_RATE_TH=%.2f deg/s  RO_SPEED_TH=%.2f m/s"
          % (math.degrees(fw.rate_th), fw.speed_th))
    print("\nderived:")
    print("  A_ideal (rad/s per unit speed-diff)      %.3f" % fw.A_ideal)
    print("  heading dead-band floor RO_YAW_RATE_TH/RO_YAW_P   %.3f deg"
          % math.degrees(fw.heading_deadband_floor_rad))
    print("  yaw-rate saturates above heading error            %.1f deg"
          % math.degrees(fw.yaw_rate_sat_heading_err_rad))
    print("\nreconstructions (a recommendation is emitted only from a VALID stage):")
    rec = Reconstruction(U)
    res = rec.run()
    for ident in ("R1", "R2", "R3", "R4"):
        if ident in res:
            print(res[ident].row())
        else:
            print("  %-3s (topics absent)" % ident)
    if not rec.valid("R3"):
        print("\n  ** R3 BROKEN -> the yaw-rate model does not describe this log. **")
        print("     Every yaw recommendation downstream is void. Stop and re-check")
        print("     the firmware overlay before trusting anything else.")
    return rec


# ----------------------------------------------------------------------
# bag side
# ----------------------------------------------------------------------
def _load_am():
    """Import analyze_mission as a library (the established tools/ idiom)."""
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
    import analyze_mission as am           # noqa: PLC0415
    return am


def _register_imu_parser(am):
    """/mavros/imu/data is not in analyze_mission's table; add it locally.

    Do NOT edit analyze_mission.py — it is a shared library with its own tests.
    """
    if "sensor_msgs/msg/Imu" in am.PARSERS:
        return

    def _p_imu(d):
        r = am._CDR(d)
        r.header()
        for _ in range(4):                 # orientation quaternion
            r.f64()
        for _ in range(9):                 # orientation_covariance
            r.f64()
        wx, wy, wz = r.f64(), r.f64(), r.f64()
        return {"wx": wx, "wy": wy, "wz": wz}

    am.PARSERS["sensor_msgs/msg/Imu"] = _p_imu


class BagSide:
    """Companion-side signals, read through analyze_mission's decoder.

    One CDR implementation and one clock definition shared with the mission
    analyser, and it brings /path, /rpp/segment_debug and the spray series that
    Layer A needs and the old rosbags reader never loaded.
    """

    def __init__(self, bundle):
        am = _load_am()
        _register_imu_parser(am)
        bag_dir, self.manifest = am._find_bag_dir(bundle)
        self.bag_dir = bag_dir
        s = am.collect(bag_dir)
        self.s = s
        self.am = am
        self.sig = {}

        def put(name, rows, *cols):
            if not rows:
                return
            a = np.asarray(rows, float)
            self.sig[name] = (a[:, 0], a[:, cols[0]] if len(cols) == 1 else None)

        if s.pose:
            p = np.asarray(s.pose, float)
            self.pose_t, self.pose_n, self.pose_e = p[:, 0], p[:, 1], p[:, 2]
            self.pose_yaw = p[:, 3]                       # NED, CW+
            self.sig["yaw_ned"] = (self.pose_t, self.pose_yaw)
        if s.vel_meas:
            v = np.asarray(s.vel_meas, float)
            self.sig["S6_speed_est"] = (v[:, 0], v[:, 1])
        if s.vel_cmd:
            v = np.asarray(s.vel_cmd, float)
            self.sig["S0_vel_n"] = (v[:, 0], v[:, 1])
            self.sig["S0_vel_e"] = (v[:, 0], v[:, 2])
            self.sig["S0_speed_cmd"] = (v[:, 0], np.hypot(v[:, 1], v[:, 2]))
            self.sig["S0_bearing_cmd"] = (v[:, 0], np.arctan2(v[:, 2], v[:, 1]))
        if s.yaw_rate:
            y = np.asarray(s.yaw_rate, float)
            self.sig["S0_yaw_rate_cmd"] = (y[:, 0], y[:, 1])
        if s.rpp:
            self.dbg_t = np.asarray([r[0] for r in s.rpp], float)
            self.dbg = np.asarray([r[1] for r in s.rpp], float)
            if self.dbg.ndim == 2 and self.dbg.shape[1] > 10:
                self.sig["S0_yaw_rate_dbg"] = (self.dbg_t, self.dbg[:, 10])
                self.sig["S0_speed_dbg"] = (self.dbg_t, self.dbg[:, 3])
                self.sig["rpp_xtrack"] = (self.dbg_t, self.dbg[:, 0])
                self.sig["rpp_lookahead"] = (self.dbg_t, self.dbg[:, 2])
        else:
            self.dbg_t = self.dbg = None
        if s.seg:
            g = np.asarray(s.seg, float)
            self.seg_t, self.seg_state = g[:, 0], g[:, 1]
        else:
            self.seg_t = self.seg_state = None

        # /mavros/imu/data, when the recorder captured it (not all runs do)
        gt, gz = [], []
        for topic, m, t in am.read_bag(bag_dir):
            if topic == "/mavros/imu/data":
                gt.append(t)
                gz.append(-m["wz"])        # FLU z-up -> NED/FRD CW+
        if len(gt) > 10:
            self.sig["gyro_ned"] = (np.asarray(gt, float), np.asarray(gz, float))

        if not self.sig:
            raise SystemExit("bag has none of the expected topics")
        t0 = min(v[0][0] for v in self.sig.values())
        t1 = max(v[0][-1] for v in self.sig.values())
        self.window = (t0, t1)

    # ---- alignment anchors -------------------------------------------
    def anchors(self, U):
        """Measured quantities present on BOTH sides, best candidate first.

        The IMU gyro is the sharpest but is not always recorded, so pose-yaw and
        speed are offered as alternates rather than assumed. Yaw is unwrapped
        before correlating so a wrap does not masquerade as a time shift.
        """
        out = []
        yc = U.yaw_rate_chain()
        if "gyro_ned" in self.sig and "S4_gyro" in yc:
            out.append(("imu-gyro", self.sig["gyro_ned"], yc["S4_gyro"]))
        if "yaw_ned" in self.sig and U.has("rover_attitude_status"):
            a = U.d["rover_attitude_status"]
            out.append(("pose-yaw",
                        (self.sig["yaw_ned"][0], np.unwrap(self.sig["yaw_ned"][1])),
                        (U.t("rover_attitude_status"), np.unwrap(a["measured_yaw"]))))
        if "S6_speed_est" in self.sig and U.has("vehicle_local_position"):
            p = U.d["vehicle_local_position"]
            out.append(("speed", self.sig["S6_speed_est"],
                        (U.t("vehicle_local_position"), np.hypot(p["vx"], p["vy"]))))
        return out


# ----------------------------------------------------------------------
# alignment
# ----------------------------------------------------------------------
def refine_offset(bag_t, bag_y, ulog_t, ulog_y, coarse, search_s=3.0, hz=25.0):
    """Refine boot->epoch offset by correlating the two yaw-rate signals."""
    lo = max(bag_t[0], ulog_t[0] + coarse)
    hi = min(bag_t[-1], ulog_t[-1] + coarse)
    if hi - lo < 3.0:
        return coarse, float("nan"), 0.0
    grid = np.arange(lo, hi, 1.0 / hz)
    b = np.interp(grid, bag_t, bag_y)
    best, best_c = 0.0, -2.0
    step = 1.0 / hz
    for k in np.arange(-search_s, search_s + step, step):
        u = np.interp(grid, ulog_t + coarse + k, ulog_y)
        if np.std(u) < 1e-9 or np.std(b) < 1e-9:
            continue
        c = float(np.corrcoef(b, u)[0, 1])
        if c > best_c:
            best_c, best = c, float(k)
    return coarse + best, best_c, best


def resample(sig, grid):
    t, y = sig
    return np.interp(grid, t, y, left=np.nan, right=np.nan)


def align(B, U):
    """Lock the two clocks using every anchor available, and cross-check them.

    Returns (offset, report_lines, ok). Agreement between independent anchors is
    the evidence that the lock is real; a single correlation is not.
    """
    lines = []
    cands = []
    for name, bs, us in B.anchors(U):
        off, corr, shift = refine_offset(bs[0], bs[1], us[0], us[1], U.utc_offset)
        cands.append((name, off, corr, shift))
    if not cands:
        return None, ["  no anchor signal is present on both sides"], False
    cands.sort(key=lambda c: -c[2])
    name, off, corr, shift = cands[0]
    lines.append("  GPS-UTC anchor         %.3f s" % U.utc_offset)
    lines.append("  best anchor            %-9s shift %+.3f s  r %.4f"
                 % (name, shift, corr))
    if len(cands) > 1:
        lines.append("  alternates             " + " | ".join(
            "%s %+.3f s r %.3f" % (c[0], c[3], c[2]) for c in cands[1:]))
        spread = max(c[1] for c in cands) - min(c[1] for c in cands)
        lines.append("  anchors agree to       %.3f s" % spread)
        if spread > 0.05:
            lines.append("  ** anchors disagree by more than 50 ms — the lock is "
                         "not trustworthy **")
    ok = corr >= 0.7
    if not ok:
        lines.append("  ** WEAK LOCK (r < 0.7) — treat every joint number as "
                     "unverified **")
    elif abs(shift) > 0.5:
        lines.append("  ** refinement > 0.5 s: the GPS anchor and the signals "
                     "disagree; check the pairing **")
    else:
        lines.append("  lock is good: anchor and signal agree to within %.0f ms"
                     % (1000 * abs(shift)))
    return off, lines, ok


# Companion parameters that shape /rpp/yaw_rate_body only. PX4 consumes an
# offboard body-rate command exclusively when body_rate && !position &&
# !velocity && !attitude (DifferentialRateControl.cpp:127-135), so on a
# velocity-vector mission every one of these is inert.
YAW_RATE_ONLY_KNOBS = ("yaw_rate_feedback_gain", "max_yaw_rate_body",
                       "use_feedforward_yaw_rate", "segment_yaw_rate_gain")
RPP_DEBUG_KNOB_IDX = {"use_feedforward_yaw_rate": 33, "yaw_rate_feedback_gain": 34,
                      "max_yaw_rate_body": 35, "segment_yaw_rate_gain": 46}


def routing_report(U, B):
    """Section 1: which command path is live, and which knobs cannot act."""
    print("\n1. COMMAND ROUTING")
    o = U.d.get("offboard_control_mode")
    if o is None:
        print("   offboard_control_mode absent — cannot establish the path")
        return None
    n = len(o["timestamp"])
    fields = ("position", "velocity", "acceleration", "attitude", "body_rate")
    occ = {k: 100.0 * float(np.mean(o[k])) for k in fields if k in o}
    print("   offboard_control_mode  " + "  ".join(
        "%s %.0f%%" % (k, v) for k, v in occ.items()) + "   (n=%d)" % n)
    velocity_path = occ.get("velocity", 0) > 50 and occ.get("body_rate", 100) < 50
    if velocity_path:
        print("   VERDICT: PX4 is on the VELOCITY-VECTOR path.")
        print("            Heading comes from atan2(vE,vN); the yaw_rate field is")
        print("            populated by the companion but PX4 never reads it.")
        knobs = []
        if B is not None and getattr(B, "dbg", None) is not None:
            for k, i in RPP_DEBUG_KNOB_IDX.items():
                if B.dbg.shape[1] > i:
                    v = np.nanmedian(B.dbg[:, i])
                    knobs.append("%s=%g" % (k, v))
        else:
            knobs = list(YAW_RATE_ONLY_KNOBS)
        print("\n   DEAD KNOBS — companion params that cannot act in this mode")
        print("     " + "   ".join(knobs))
        print("     All of these shape /rpp/yaw_rate_body only. Tuning them cannot")
        print("     change the trajectory while the mode occupancy above holds.")
    else:
        print("   VERDICT: NOT the velocity-vector path — the yaw-rate model below")
        print("            may not apply. Check offboard_control_mode occupancy.")
    if B is not None and B.s.setpoint:
        tm = np.asarray([r[1] for r in B.s.setpoint], float)
        from collections import Counter
        c = Counter(tm.astype(int).tolist())
        print("   setpoint type_mask     " + ", ".join(
            "%d: %.0f%%" % (k, 100.0 * v / len(tm))
            for k, v in sorted(c.items(), key=lambda kv: -kv[1])[:3]))
    return velocity_path


# ----------------------------------------------------------------------
# report
# ----------------------------------------------------------------------
def analyse(ulog_path, bag_path=None, fig_path=None, emit_params=None,
            json_path=None, csv_dir=None):
    U = UlogSide(ulog_path)
    print("=" * 108)
    print("ROVER CLOSED-LOOP TUNING DIAGNOSTIC")
    print("=" * 108)
    print("\n0. PROVENANCE & TRUST")
    print("   ulog        : %s" % ulog_path)
    print("   duration    : %.1f s   mode %s"
          % ((U.u.last_timestamp - U.u.start_timestamp) / 1e6, U.mode_summary()))
    print("   ver_sw      : %s" % U.u.msg_info_dict.get("ver_sw", "?"))
    print("                 ^ BASE hash only. The deployed build is fork")
    print("                   Vetri2425/PX4-Autopilot @06309e41a7 = v1.16.2 + a")
    print("                   26-file overlay; ver_sw cannot show that.")
    print("   modelled as : stock v1.16.2 rate/FF + fork overlay inverse kinematics")
    rates = []
    for t_name, label in (("rover_rate_status", "rover_*"),
                          ("vehicle_angular_velocity", "angular_velocity"),
                          ("wheel_encoders", "wheel_encoders")):
        if U.has(t_name):
            tt = U.t(t_name)
            if len(tt) > 2:
                rates.append("%s %.0f Hz" % (label, len(tt) / (tt[-1] - tt[0])))
    if rates:
        lag_floor = 1.5 / (len(U.t("rover_rate_status"))
                           / (U.t("rover_rate_status")[-1] - U.t("rover_rate_status")[0])) \
            if U.has("rover_rate_status") else float("nan")
        print("   logged rates: " + "  ".join(rates) + "   (control loop 100 Hz)")
        print("                 -> lag floor %.2f s; anything faster is NOT"
              " resolvable here" % lag_floor)
    w = U.window_utc()
    if not w:
        print("   UTC anchor  : NONE (no GPS time in this log) — joint mode impossible")

    B = None
    offset = None
    if bag_path:
        B = BagSide(bag_path)
        print("   bag         : %s" % bag_path)
        print("                 %.1f s, %d topics decoded"
              % (B.window[1] - B.window[0], len(B.s.topics_seen)))
        if U.utc_offset is None:
            print("\n   ALIGNMENT FAILED: no UTC anchor in the ulog.")
            B = None
        else:
            ov = min(B.window[1], w[1]) - max(B.window[0], w[0])
            if ov <= 1.0:
                print("\n   ALIGNMENT FAILED: bag and ulog windows do not overlap "
                      "(%.0f s apart). These are not the same run." %
                      (max(B.window[0], w[0]) - min(B.window[1], w[1])))
                B = None
            else:
                print("\n   ALIGNMENT   (raw overlap %.1f s)" % ov)
                offset, lines, ok = align(B, U)
                for ln in lines:
                    print("  " + ln)
                if not ok:
                    B = None

    routing_report(U, B)

    geo_stats = None
    if B is not None:
        geo_stats = Geometry(B).report(Firmware(U.params))

    # ---------------- yaw-rate chain ----------------
    yc = U.yaw_rate_chain()
    if not yc:
        print("\nno rover_rate_* topics in this ulog — nothing to chain")
        return U

    _rate_only = {k: v for k, v in yc.items() if k.startswith("S")}
    t_lo = max(s[0][0] for s in _rate_only.values())
    t_hi = min(s[0][-1] for s in _rate_only.values())
    grid = np.arange(t_lo, t_hi, 0.02)                    # 50 Hz, boot seconds
    R = {k: resample(v, grid) for k, v in yc.items()}
    # angles must be unwrapped before interpolation or a wrap becomes a ramp
    for k in ("B3_bearing", "B4_yaw_sp", "B5_yaw_adj", "B5_yaw_meas"):
        if k in yc:
            R[k] = resample((yc[k][0], np.unwrap(yc[k][1])), grid)
    if B is not None:
        for k in ("S0_yaw_rate_cmd", "S0_yaw_rate_dbg"):
            if k in B.sig:
                R[k] = resample(B.sig[k], grid + offset)
        R["S6_gyro_bag"] = resample(B.sig["gyro_ned"], grid + offset)

    gate = U.mode_mask(grid)
    drive = gate & (np.abs(R.get("S1_cmd_rx", R["S3_measured"])) > 0.05)

    print("\n7. STEERING CHAIN")
    print("   PX4 steers from the velocity VECTOR, so the chain starts at the")
    print("   commanded bearing, not at a yaw-rate command.")
    print("\n   7a. HEADING STAGE (deg) — angles, so these are ERRORS not gains")
    # Split on whether the COMMANDED heading is slewing. During a pivot the
    # setpoint sweeps tens of degrees and the limiter lags by design, so mixing
    # the two makes a well-behaved rover look broken. The TRACKING row is the
    # one that maps to the marking-accuracy and heading-noise targets.
    # "Not slewing" alone is not enough: after a pivot the setpoint stops moving
    # while the limiter is still catching up. Add a non-circular gate on actually
    # driving forward — during a spot turn the fork forces the speed setpoint to
    # zero, so a real forward speed means the rover is tracking a line.
    fwd = np.ones(len(grid), bool)
    if U.has("rover_velocity_status"):
        fwd = resample((U.t("rover_velocity_status"),
                        U.d["rover_velocity_status"]["adjusted_speed_body_x_setpoint"]),
                       grid)
        fwd = np.nan_to_num(np.abs(fwd)) > 0.15
    if "B4_yaw_sp" in R:
        dyaw = np.gradient(np.nan_to_num(R["B4_yaw_sp"]), grid)
        track_m = gate & fwd & (np.abs(dyaw) < 0.10)
        turn_m = gate & ~(fwd & (np.abs(dyaw) < 0.10))
    else:
        track_m, turn_m = gate & fwd, gate & ~fwd
    # B3->B4 is NOT in this table: the firmware assigns the attitude setpoint
    # from the commanded bearing, so their difference is identically zero by
    # construction. A 0.00 there is an identity, not a measurement, and it reads
    # like a result when it sits among error statistics. It is verified in
    # section 6 (R1) where identities belong.
    for a, b, label in (("B4_yaw_sp", "B5_yaw_adj",
                         "attitude setpoint -> after limiting"),
                        ("B5_yaw_adj", "B5_yaw_meas",
                         "attitude setpoint -> MEASURED yaw (HEADING ERROR)")):
        if a not in R or b not in R:
            continue
        e = R2D * wrap(R[b] - R[a])
        for tag, msk in (("TRACKING", track_m), ("turning ", turn_m)):
            m = msk & np.isfinite(e)
            if m.sum() < 30:
                continue
            v = e[m]
            print("     %-50s %s n=%5d  RMS %6.2f  mean %+6.2f  p95 %6.2f"
                  % (label if tag == "TRACKING" else "", tag, m.sum(),
                     rms(v), np.mean(v), np.percentile(np.abs(v), 95)))
    fwl = Firmware(U.params)
    print("     (RO_YAW_P=%.2f turns this error into the rate setpoint; it is the"
          % fwl.yaw_p)
    print("      ONLY place a heading error becomes a correction.)")

    print("\n   7b. RATE STAGE (rad/s; gain = y/x through the origin; "
          "bias and RMS in deg/s)")
    print("   gated on OFFBOARD + armed + |setpoint| > 0.05 rad/s   n=%d of %d ticks"
          % (drive.sum(), len(grid)))
    # S0->S1 is handled separately below: it needs its own gate and resampling.
    order = [("S1_cmd_rx", "S2_adjusted", "setpoint -> after slew/limit"),
             ("S2_adjusted", "S3_measured", "after slew -> loop measured"),
             ("S1_cmd_rx", "S3_measured", "setpoint -> measured  (END-TO-END)"),
             ("S3_measured", "S4_gyro", "loop measured -> raw gyro"),
             ("S4_gyro", "S5_wheels", "gyro -> wheel-implied yaw rate"),
             ("S4_gyro", "S6_gyro_bag", "PX4 gyro -> bag gyro (sign/clock check)")]
    for a, b, label in order:
        if a in R and b in R:
            print(Link(label, grid, R[a], R[b], "d/s", drive).row(R2D))

    # 7c: the actuator end of the chain, in its own units. These are normalised
    # motor commands [-1, 1], not rates, so they get a value table rather than a
    # gain table -- a gain from rad/s into a normalised command is not a number
    # anyone can act on.
    if "B9_speed_diff" in R:
        print("\n   7c. DIFFERENTIAL CHANNEL AND MOTOR OUTPUT (normalised [-1, 1])")
        print("   %-34s %8s %8s %8s %8s" % ("", "mean", "sd", "min", "max"))
        for key, label in (("B9_speed_diff", "commanded speed-diff (FF+PID)"),
                           ("B10_motor_diff", "achieved diff (L-R)/2"),
                           ("B10_motor_common", "common-mode throttle (L+R)/2"),
                           ("B10_motor_left", "motor LEFT  control[0]"),
                           ("B10_motor_right", "motor RIGHT control[1]")):
            if key not in R:
                continue
            v = R[key][drive & np.isfinite(R[key])]
            if len(v) < 20:
                continue
            print("     %-32s %8.4f %8.4f %8.4f %8.4f"
                  % (label, np.mean(v), np.std(v), np.min(v), np.max(v)))
        if "B9_speed_diff" in R and "B10_motor_diff" in R:
            m = drive & np.isfinite(R["B9_speed_diff"]) & np.isfinite(R["B10_motor_diff"])
            if m.sum() > 20:
                d = R["B10_motor_diff"][m] - R["B9_speed_diff"][m]
                print("     commanded -> achieved differential: max |error| %.5f"
                      % np.max(np.abs(d)))
                print("       (the mixer takes saturation out of THROTTLE, never out")
                print("        of the differential, so these agree unless clipped)")
        sat = drive & (np.abs(R.get("B10_motor_left", np.zeros(len(grid)))) >= 0.999)
        sat |= drive & (np.abs(R.get("B10_motor_right", np.zeros(len(grid)))) >= 0.999)
        if drive.sum():
            print("     motor rail saturation: %.1f%% of driving ticks"
                  % (100 * sat.sum() / drive.sum()))

    # A scale error and a transient overshoot both show up as gain > 1 over the
    # whole run, but they mean different things and have different fixes. Split
    # on how fast the setpoint is moving.
    if "S1_cmd_rx" in R and "S3_measured" in R:
        dsp = np.gradient(np.nan_to_num(R["S1_cmd_rx"]), grid)
        steady = drive & (np.abs(dsp) < 0.05)
        trans = drive & (np.abs(dsp) >= 0.05)
        print("  --- setpoint -> measured, split by how fast the setpoint moves ---")
        for label, m in (("STEADY  (|d sp/dt| < 0.05 rad/s^2)", steady),
                         ("TRANSIENT", trans)):
            print(Link("  " + label, grid, R["S1_cmd_rx"], R["S3_measured"],
                       "d/s", m).row(R2D))
        if "S2_adjusted" in R:
            print(Link("  STEADY: setpoint -> after slew", grid, R["S1_cmd_rx"],
                       R["S2_adjusted"], "d/s", steady).row(R2D))
        print("      a gain > 1 that SURVIVES in steady state is a scale error in the"
              " feedforward,\n      not overshoot: the slew limiter is inactive there"
              " by construction.")

    # The companion->PX4 link needs its own gate and its own resampling.
    # /rpp/yaw_rate_body is the RPP feedforward only and is 0 DURING PIVOTS
    # (rpp_controller_node.py:4931) — the pivot command reaches PX4 by another
    # route. Comparing it against rover_rate_setpoint on the pivot plateaus
    # measures the routing, not the link. It is also published ~5x faster than
    # the ulog samples it, so it is held to the ulog's own stamps rather than
    # interpolated onto a finer grid.
    if B is not None and "S0_yaw_rate_cmd" in B.sig and "S1_cmd_rx" in yc:
        t1, y1 = yc["S1_cmd_rx"]
        t0, y0 = B.sig["S0_yaw_rate_cmd"]
        idx = np.searchsorted(t0 - offset, t1, side="right") - 1
        ok = (idx >= 0) & (idx < len(y0))
        s0 = np.full(len(t1), np.nan)
        s0[ok] = y0[idx[ok]]
        fresh = np.zeros(len(t1), bool)
        fresh[ok] = (t1[ok] - (t0 - offset)[idx[ok]]) < 0.3   # companion not silent
        s0[~fresh] = np.nan
        act = fresh & (np.abs(s0) > 0.02) & U.mode_mask(t1)
        piv = fresh & (np.abs(s0) < 1e-6) & (np.abs(y1) > 0.1)
        print("  --- companion -> PX4 (held at the ulog's own stamps) ---")
        print(Link("  driving phase: /rpp/yaw_rate_body -> rover_rate_setpoint",
                   t1, s0, y1, "d/s", act).row(R2D))
        print("      pivot-route ticks (companion 0 while PX4 setpoint > 0.1 rad/s): "
              "%d of %d (%.0f%%) — excluded above, they do not travel on this topic"
              % (piv.sum(), int(fresh.sum()), 100 * piv.mean()))

    if "integral" in R:
        i = R["integral"][drive]
        i = i[np.isfinite(i)]
        if len(i):
            print("  PID integral: mean %+.4f  max |.| %.4f  "
                  "(large => the loop is fighting a steady bias)"
                  % (i.mean(), np.abs(i).max()))

    # ---------------- speed chain ----------------
    sc = U.speed_chain()
    if sc:
        t_lo = max(s[0][0] for s in sc.values())
        t_hi = min(s[0][-1] for s in sc.values())
        g2 = np.arange(t_lo, t_hi, 0.02)
        S = {k: resample(v, g2) for k, v in sc.items()}
        if B is not None:
            for k in ("S0_speed_cmd", "S0_speed_dbg", "S6_speed_est"):
                if k in B.sig:
                    S[k] = resample(B.sig[k], g2 + offset)
        if "S5_wheel_speed" in yc:
            S["S5_wheels"] = resample(yc["S5_wheel_speed"], g2)
        gate2 = U.mode_mask(g2)
        ref = S.get("S2_throttle_implied", S.get("S3_measured"))
        drive2 = gate2 & (np.abs(ref) > 0.05)

        print("\n8. SPEED CHAIN   (m/s)")
        print("   setpoint reference = adjusted_speed_body_x_setpoint (post-slew).")
        print("   The speed loop IS closed: RO_SPEED_P=%s RO_SPEED_I=%s act on top"
              % (U.params.get("RO_SPEED_P"), U.params.get("RO_SPEED_I")))
        print("   of the RO_MAX_THR_SPEED feedforward.")
        for a, b, label in [
                ("S0_speed_cmd", "S2_throttle_implied", "companion cmd -> throttle-implied"),
                ("S0_speed_dbg", "S2_throttle_implied", "rpp/debug[3]  -> throttle-implied"),
                ("S2_adjusted", "S3_measured", "PX4 adjusted setpoint -> measured"),
                ("S2_throttle_implied", "S3_measured", "throttle-implied -> measured  (FF path)"),
                ("S3_measured", "S5_wheels", "measured -> wheel speed"),
                ("S3_measured", "S6_estimate", "measured -> local_position |v|"),
                ("S6_estimate", "S6_speed_est", "PX4 |v| -> bag |v| (clock check)")]:
            if a in S and b in S:
                print(Link(label, g2, S[a], S[b], "m/s", drive2).row())
        if "S2_throttle_implied" in S and "S3_measured" in S:
            m = drive2 & np.isfinite(S["S2_throttle_implied"]) & np.isfinite(S["S3_measured"])
            if m.sum() > 20:
                p = np.polyfit(S["S2_throttle_implied"][m], S["S3_measured"][m], 1)
                print("  open-loop fit: measured = %.3f * (throttle * RO_MAX_THR_SPEED) "
                      "%+.3f   (intercept = rolling-resistance offset)" % (p[0], p[1]))

    # ---------------- model validation, dead-bands, plant ID ----------------
    fw = Firmware(U.params)
    print("\n6. MODEL VALIDATION")
    rec = Reconstruction(U)
    res = rec.run()
    for ident in ("R1", "R2", "R3", "R4"):
        if ident in res:
            print(res[ident].row())
    # identity checks live here, not among the error statistics in 7a
    if "B3_bearing" in yc and "B4_yaw_sp" in yc:
        ta = yc["B4_yaw_sp"][0]
        ident_err = wrap(np.unwrap(yc["B4_yaw_sp"][1])
                         - _interp_to((yc["B3_bearing"][0],
                                       np.unwrap(yc["B3_bearing"][1])), ta))
        mm = np.isfinite(ident_err) & U.mode_mask(ta)
        if mm.sum() > 30:
            print("  ID  attitude setpoint == commanded bearing        "
                  "resid %9.5f  (identity: assigned in firmware, not a measurement)"
                  % math.degrees(rms(ident_err[mm])))
    if not rec.valid("R3"):
        print("   ** R3 BROKEN — the yaw model does not describe this log; every")
        print("      yaw number below is unsupported. **")
    deadband_report(U, fw)
    pid = PlantID(U, fw)
    pid.fit()
    pid.report()
    rcm = Recommender(U, rec, pid, fw)
    rcm.build()
    rcm.report()
    if emit_params:
        n = rcm.emit_params(emit_params)
        print("\n   wrote %d parameter change(s) to %s" % (n, emit_params))
    targets_report(geo_stats, U, fw, B)

    if csv_dir:
        files = export_csv(csv_dir, U, B, offset)
        print("\n   wrote %d CSV file(s) to %s (see README.txt there)"
              % (len(files), csv_dir))

    if json_path:
        blob = {
            "schema": "analyze_bag_ulog@1",
            "ulog": ulog_path,
            "bag": bag_path,
            "firmware": {
                "ver_sw_base": U.u.msg_info_dict.get("ver_sw"),
                "modelled_as": "fork Vetri2425/PX4-Autopilot @06309e41a7 "
                               "(v1.16.2 + 26-file overlay)",
            },
            "alignment": {"offset_s": offset},
            "reconstruction": {k: {"resid": v.resid, "tol": v.tol,
                                   "n": v.n, "verdict": v.verdict}
                               for k, v in res.items()},
            "plant": {k: v for k, v in pid.out.items()
                      if isinstance(v, (int, float, bool))},
            "deadbands": {
                "heading_floor_deg": math.degrees(fw.heading_deadband_floor_rad),
                "yaw_rate_th_deg_s": math.degrees(fw.rate_th),
                "speed_th_m_s": fw.speed_th,
            },
            "geometry": geo_stats,
            "targets": {"heading_rms_deg": _tracking_heading_rms(U)},
            "recommendations": [
                {"param": r.param, "now": r.now,
                 "recommend": None if not np.isfinite(r.rec) else r.rec,
                 "lo": r.lo, "hi": r.hi, "confidence": r.conf,
                 "model": r.model, "evidence": r.why,
                 "applied_by_emit_params": bool(np.isfinite(r.rec)
                                                and abs(r.rec - r.now) > 1e-6)}
                for r in rcm.recs],
            "params": {k: float(v) for k, v in U.params.items()
                       if k.startswith(("RO_", "RD_", "EKF2_", "GPS_"))},
        }

        def _plain(x):
            if isinstance(x, (np.floating, np.integer)):
                return float(x)
            if isinstance(x, np.bool_):
                return bool(x)
            if isinstance(x, float) and not np.isfinite(x):
                return None
            return x

        with open(json_path, "w") as fh:
            json.dump(blob, fh, indent=1, default=_plain)
        print("\n   wrote machine-readable results to %s" % json_path)

    # ---------------- saturation ----------------
    print("\n9b. SATURATION DETAIL  (as-run values from the ulog)")
    lim = float(U.params.get("RO_YAW_RATE_LIM", float("nan")))
    if np.isfinite(lim) and "S1_cmd_rx" in R:
        v = R["S1_cmd_rx"][drive]
        v = v[np.isfinite(v)]
        if len(v):
            print("  RO_YAW_RATE_LIM %5.1f deg/s : setpoint at the limit %.1f%% of ticks"
                  % (lim, 100 * np.mean(np.abs(v) * R2D >= lim - 0.5)))
    for pname, sig, unit, scale in (("RO_SPEED_LIM", "S3_measured", "m/s", 1.0),):
        if pname in U.params and sc and sig in S:
            v = S[sig][drive2]
            v = v[np.isfinite(v)]
            if len(v):
                print("  %-15s %5.2f %-4s: measured at the limit %.1f%% of ticks"
                      % (pname, U.params[pname], unit,
                         100 * np.mean(v * scale >= U.params[pname] - 0.01)))
    if "S1_cmd_rx" in R and "S2_adjusted" in R:
        m = drive & np.isfinite(R["S1_cmd_rx"]) & np.isfinite(R["S2_adjusted"])
        if m.sum():
            d = np.abs(R["S1_cmd_rx"][m] - R["S2_adjusted"][m])
            print("  slew/limit stage bit on %.1f%% of ticks, max |setpoint-adjusted| "
                  "%.2f deg/s  (RO_YAW_ACCEL_LIM %s / DECEL %s)"
                  % (100 * np.mean(d > 1e-4), R2D * d.max(),
                     U.params.get("RO_YAW_ACCEL_LIM"), U.params.get("RO_YAW_DECEL_LIM")))

    # ---------------- params ----------------
    print("\n15. AS-RUN FCU PARAMS (from the ulog — authoritative; the /mavros/param "
          "mirror freezes at MAVROS init)")
    keys = ["RO_YAW_P", "RO_YAW_RATE_P", "RO_YAW_RATE_I", "RO_YAW_RATE_LIM",
            "RO_YAW_ACCEL_LIM", "RO_YAW_DECEL_LIM", "RO_YAW_RATE_TH",
            "RO_MAX_THR_SPEED", "RO_SPEED_LIM", "RO_SPEED_P", "RO_SPEED_I",
            "RO_ACCEL_LIM", "RO_DECEL_LIM", "RO_SPEED_TH",
            "RD_WHEEL_TRACK", "EKF2_WENC_CTRL", "EKF2_WENC_RAD",
            "GPS_YAW_OFFSET", "EKF2_IMU_POS_X", "EKF2_IMU_POS_Y"]
    line = []
    for k in keys:
        if k in U.params:
            line.append("%s=%g" % (k, U.params[k]))
    for i in range(0, len(line), 4):
        print("  " + "   ".join(line[i:i + 4]))
    print("  %d parameters in the log, %d changed mid-log%s"
          % (len(U.params), len(U.changed_params),
             "" if not U.changed_params else
             " -> " + ", ".join(c[1] for c in U.changed_params[:6])))

    if fig_path:
        _figure(grid, R, drive, fig_path, U)
    return U


def _figure(grid, R, drive, path, U):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    keys = [k for k in ("S0_yaw_rate_cmd", "S1_cmd_rx", "S2_adjusted",
                        "S3_measured", "S4_gyro", "S5_wheels") if k in R]
    fig, ax = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    t = grid - grid[0]
    for k in keys:
        ax[0].plot(t, R2D * R[k], lw=1, label=k)
    ax[0].set_ylabel("yaw rate deg/s")
    ax[0].legend(fontsize=7, ncol=3)
    ax[0].set_title(os.path.basename(U.path) + " — yaw-rate chain")
    if "S1_cmd_rx" in R and "S3_measured" in R:
        ax[1].axhline(0, color="k", lw=.5)
        ax[1].plot(t, R2D * (R["S3_measured"] - R["S1_cmd_rx"]), "C3", lw=1,
                   label="measured - setpoint")
    ax[1].fill_between(t, -1, 1, where=drive, color="C2", alpha=.12,
                       transform=ax[1].get_xaxis_transform(), label="gated")
    ax[1].set_ylabel("tracking error deg/s")
    ax[1].set_xlabel("s")
    ax[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print("\nwrote %s" % path)


# ----------------------------------------------------------------------
def sweep(ulog_dir, bag_root=None, emit_params=None):
    """One row per run, then the cross-run aggregate.

    A band taken from a single run only reflects estimator disagreement inside
    that run. Run-to-run spread is the honest uncertainty on a recommendation,
    so the aggregate here — not the per-run band — is what should be applied.
    """
    ulogs = sorted(glob.glob(os.path.join(ulog_dir, "**", "*.ulg"),
                             recursive=True))
    if not ulogs:
        raise SystemExit("no .ulg under %s" % ulog_dir)
    bags = {}
    if bag_root:
        import yaml                                     # noqa: PLC0415
        for meta in glob.glob(os.path.join(bag_root, "**", "bag", "metadata.yaml"),
                              recursive=True):
            try:
                info = yaml.safe_load(open(meta))["rosbag2_bagfile_information"]
            except Exception:                           # noqa: BLE001
                continue
            t0 = info["starting_time"]["nanoseconds_since_epoch"] / 1e9
            bags[meta.split("/bag/")[0]] = (
                t0, t0 + info["duration"]["nanoseconds"] / 1e9)

    print("=" * 108)
    print("MULTI-RUN SWEEP   %s" % ulog_dir)
    print("=" * 108)
    print("%-12s %-30s %7s %7s %8s %8s %7s %7s"
          % ("ulog", "bag", "G", "A", "R_opt", "K_thr", "hdgRMS", "n"))
    rows = []
    for f in ulogs:
        try:
            U = UlogSide(f)
        except Exception as exc:                        # noqa: BLE001
            print("%-12s  unreadable: %s" % (os.path.basename(f)[:12], exc))
            continue
        name = os.path.basename(f).split("_2026")[0]
        if "OFFBOARD" not in U.mode_summary():
            print("%-12s  %s -> skipped (not an OFFBOARD run)" % (name, U.mode_summary()))
            continue
        pid_run = PlantID(U)
        o = pid_run.fit()
        # Route through the SAME gates as the single-run path. Aggregating raw
        # fits here would let the sweep emit a parameter that every individual
        # run withheld -- which it did for RO_MAX_THR_SPEED until this was fixed.
        rec_run = Reconstruction(U)
        rec_run.run()
        accepted = {r.param for r in Recommender(U, rec_run, pid_run).build()
                    if np.isfinite(r.rec) and abs(r.rec - r.now) > 1e-6}
        bag = ""
        w = U.window_utc()
        if w:
            for b, (a0, a1) in bags.items():
                if min(w[1], a1) - max(w[0], a0) > 1.0:
                    bag = os.path.basename(b)
        hdg = _tracking_heading_rms(U)
        cfg = tuple(round(float(U.params.get(k, float("nan"))), 4)
                    for k in ("RD_MAX_THR_YAW_R", "RO_MAX_THR_SPEED",
                              "RD_WHEEL_TRACK", "RO_YAW_P", "RO_YAW_RATE_P"))
        rows.append(dict(name=name, bag=bag, hdg=hdg, cfg=cfg,
                         accepted=accepted, params=dict(U.params), **o))
        print("%-12s %-30s %7.4f %7.3f %8.3f %8.3f %7s %7d"
              % (name, bag[:30], o.get("G", float("nan")),
                 o.get("A_mid", float("nan")), o.get("R_opt", float("nan")),
                 o.get("K_thr", float("nan")),
                 "%.2f" % hdg if np.isfinite(hdg) else "-",
                 o.get("n_steady", 0)))
    if not rows:
        print("\nno OFFBOARD runs to aggregate")
        return rows

    # Aggregating across runs that flew DIFFERENT parameters is meaningless: the
    # gain being measured is a property of the config. Keep only the largest
    # single-config group and say what was set aside.
    cfgs = {r["cfg"] for r in rows}
    if len(cfgs) > 1:
        # The MOST RECENT config, not the most common one: the question is what
        # to set on the vehicle as it is configured now, and an old config can
        # easily out-number it in a log folder.
        keep_cfg = rows[-1]["cfg"]
        keep = [r for r in rows if r["cfg"] == keep_cfg]
        print("\n   ** %d parameter configurations present in this directory. **"
              % len(cfgs))
        print("      A gain is a property of the config it was flown with, so the")
        print("      aggregate below uses only the MOST RECENT config (%d runs)"
              % len(keep))
        print("      and SETS ASIDE %d run(s) flown with other parameters."
              % (len(rows) - len(keep)))
        rows = keep

    print("\nCROSS-RUN AGGREGATE  (n=%d runs)" % len(rows))
    fw = Firmware(rows[0]["params"])
    agg = {}
    for key, label, cur in (("G", "steady yaw gain G", None),
                            ("A_mid", "plant gain A", None),
                            ("R_opt", "RD_MAX_THR_YAW_R", fw.R_yaw),
                            ("K_thr", "RO_MAX_THR_SPEED", fw.K_spd),
                            ("hdg", "tracking heading RMS (deg)", None)):
        v = np.array([r.get(key, np.nan) for r in rows], float)
        v = v[np.isfinite(v)]
        if len(v) < 2:
            continue
        agg[key] = (v.mean(), v.std(), v.min(), v.max())
        cur_s = "   current %.3f" % cur if cur is not None else ""
        print("  %-26s %7.3f +- %.3f   [%.3f .. %.3f]%s"
              % (label, v.mean(), v.std(), v.min(), v.max(), cur_s))

    if emit_params and "R_opt" in agg:
        with open(emit_params, "w") as fh:
            fh.write("# Cross-run recommended changes from %d OFFBOARD runs\n"
                     % len(rows))
            fh.write("# Source: %s\n" % ulog_dir)
            fh.write("# REVIEW BEFORE LOADING. QGC on the Mac is source of truth.\n")
            fh.write("# vehicle_id\tcomponent_id\tname\tvalue\ttype\n")
            for key, pname, cur in (("R_opt", "RD_MAX_THR_YAW_R", fw.R_yaw),
                                    ("K_thr", "RO_MAX_THR_SPEED", fw.K_spd)):
                if key not in agg:
                    continue
                n_ok = sum(1 for r in rows if pname in r.get("accepted", ()))
                if n_ok < len(rows):
                    fh.write("# %s: WITHHELD -- accepted by only %d of %d runs;\n"
                             "#   see the per-run report for the reason.\n"
                             % (pname, n_ok, len(rows)))
                    print("  %s: WITHHELD from the params file (accepted by %d/%d "
                          "runs)" % (pname, n_ok, len(rows)))
                    continue
                m, sd, lo, hi = agg[key]
                fh.write("# %s: %.3f -> %.3f  (mean of %d runs, sd %.3f, "
                         "range %.3f..%.3f)\n" % (pname, cur, m, len(rows), sd, lo, hi))
                fh.write("1\t1\t%s\t%.6f\t9\n" % (pname, m))
        print("\n  wrote cross-run parameter file to %s" % emit_params)
    return rows


def _tracking_heading_rms(U):
    """Heading error RMS while actually driving a line (deg). NaN if unknowable."""
    if not U.has("rover_attitude_status", "rover_velocity_status"):
        return float("nan")
    ta = U.t("rover_attitude_status")
    a = U.d["rover_attitude_status"]
    e = wrap(a["measured_yaw"] - a["adjusted_yaw_setpoint"])
    sp = _interp_to((U.t("rover_velocity_status"),
                     U.d["rover_velocity_status"]["adjusted_speed_body_x_setpoint"]), ta)
    dyaw = np.gradient(np.unwrap(a["adjusted_yaw_setpoint"]), ta)
    m = (U.mode_mask(ta) & np.isfinite(sp) & (np.abs(sp) > 0.15)
         & (np.abs(dyaw) < 0.10))
    if m.sum() < 30:
        return float("nan")
    return math.degrees(rms(e[m]))


def auto_match(bag_path, ulog_dir):
    """Pick the ulog whose UTC window overlaps this bag the most."""
    B = BagSide(bag_path)
    best, best_ov = None, 0.0
    for f in sorted(glob.glob(os.path.join(ulog_dir, "*.ulg"))):
        try:
            U = UlogSide(f)
        except Exception:
            continue
        w = U.window_utc()
        if not w:
            continue
        ov = min(B.window[1], w[1]) - max(B.window[0], w[0])
        if ov > best_ov:
            best, best_ov = f, ov
    if best is None:
        raise SystemExit("no ulog in %s overlaps this bag's window %s"
                         % (ulog_dir, B.window))
    print("auto-matched %s (overlap %.1f s)\n" % (best, best_ov))
    return best


def selftest():
    """Validate the aligner: inject a known offset and recover it."""
    print("SELFTEST — alignment recovery")
    rng = np.random.default_rng(7)
    t = np.arange(0, 40, 0.02)
    y = (np.sin(2 * np.pi * 0.33 * t) + 0.3 * np.sin(2 * np.pi * 1.1 * t))
    ok = True
    for true_off in (0.0, 0.24, -0.37, 1.05):
        bt = t + 1_000_000.0                       # "epoch" clock
        by = y + rng.normal(0, 0.02, len(t))
        ut = t[::2]                                # ulog at half rate, boot clock
        uy = np.interp(ut + true_off, t, y) + rng.normal(0, 0.02, len(ut))
        coarse = 1_000_000.0 - 0.0
        got, corr, shift = refine_offset(bt, by, ut, uy, coarse)
        # uy(ut) samples the true signal at ut+true_off, so the epoch time of a
        # feature at boot time ut is ut+true_off+1e6 => offset = coarse+true_off
        err = (got - coarse) - true_off
        flag = "ok" if abs(err) <= 0.021 and corr > 0.9 else "FAIL"
        ok &= flag == "ok"
        print("  true %+.2f s -> recovered %+.2f s  (err %+.3f s, r %.3f)  %s"
              % (true_off, got - coarse, err, corr, flag))
    print("  window rejection:", end=" ")
    got, corr, shift = refine_offset(np.arange(0, 10, .02) + 5000,
                                     np.zeros(500),
                                     np.arange(0, 10, .02), np.zeros(500), 0.0)
    print("no-overlap returns coarse unchanged:", got == 0.0)
    print("SELFTEST", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", help="bundle dir (or its bag/ subdir)")
    ap.add_argument("--ulog", help="path to a .ulg")
    ap.add_argument("--ulog-dir", help="directory of .ulg files to auto-match")
    ap.add_argument("--fig", help="write a chain figure here")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--csv-dir", metavar="DIR",
                    help="dump every topic to CSV (native rate, with a\nt_epoch column) plus one time-aligned joint table")
    ap.add_argument("--json", metavar="FILE",
                    help="write the results as JSON for programmatic use")
    ap.add_argument("--bag-dir", help="root of bag bundles, for the sweep")
    ap.add_argument("--sweep", action="store_true",
                    help="one row per run over --ulog-dir, then the aggregate")
    ap.add_argument("--emit-params", metavar="FILE",
                    help="write a QGC .params file of the recommended changes")
    ap.add_argument("--validate-model", action="store_true",
                    help="print only the firmware-model validation section")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    check_deps()
    if a.validate_model:
        if not a.ulog:
            ap.error("--validate-model needs --ulog")
        rec = validate_model_report(UlogSide(a.ulog))
        return 0 if rec.valid("R3") else 1
    if a.sweep:
        if not a.ulog_dir:
            ap.error("--sweep needs --ulog-dir")
        sweep(a.ulog_dir, a.bag_dir, a.emit_params)
        return 0
    if a.bag and a.ulog_dir and not a.ulog:
        a.ulog = auto_match(a.bag, a.ulog_dir)
    if not a.ulog:
        ap.error("need --ulog or (--bag and --ulog-dir)")
    analyse(a.ulog, a.bag, a.fig, a.emit_params, a.json, a.csv_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
