#!/usr/bin/env python3
"""EKF2 fusion-health report from a PX4 .ulg.

Companion to docs/EKF_FUSION_HEALTH_CAPTURE.md. Reads only; changes nothing.

    python3 tools/analyze_ekf_health.py PX4_Logs/log_NN.ulg
    python3 tools/analyze_ekf_health.py log.ulg --segments segments.txt -o report.md

Segment file: one "name  start_s  end_s" per line (seconds from log start, '#' comments).
Without it the whole log is one segment named "ALL".

Test-ratio convention (PX4): ratio = innov^2 / (gate^2 * innov_var).
  <0.5 healthy | 0.5-1.0 marginal | >1.0 REJECTED (measurement discarded).
A low ratio only means something if the noise params are honest -- a sedated
filter also reads green, so ratios are always reported next to fused/rejected
duty cycle, which a sedated filter cannot fake.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    sys.exit("pyulog not installed:  pip3 install pyulog")

# Aid sources this rover can have. gnss_yaw is the whole heading solution
# (EKF2_MAG_TYPE=5 -> no magnetometer fallback); wheel_encoder is fork-only.
AID_SOURCES = [
    "estimator_aid_src_gnss_pos",
    "estimator_aid_src_gnss_vel",
    "estimator_aid_src_gnss_yaw",
    "estimator_aid_src_gnss_hgt",
    "estimator_aid_src_baro_hgt",
    "estimator_aid_src_gravity",
    "estimator_aid_src_wheel_encoder",
]

# Flags worth a timeline. cs_* = control status (what is actually being fused),
# fs_* = fault status. Absence of a transition is as informative as one.
WATCH_FLAGS = [
    "cs_yaw_align", "cs_tilt_align", "cs_gnss_pos", "cs_gnss_vel", "cs_gnss_yaw",
    "cs_gnss_yaw_fault", "cs_inertial_dead_reckoning", "cs_constant_pos",
    "cs_vehicle_at_rest", "cs_in_air", "cs_fake_pos", "cs_valid_fake_pos",
    "fs_bad_hdg", "fs_bad_acc_clipping", "fs_bad_acc_vertical",
    "reject_hor_pos", "reject_hor_vel", "reject_ver_pos", "reject_ver_vel", "reject_yaw",
]

RESET_EVENTS = [
    "reset_pos_to_gps", "reset_vel_to_gps", "reset_hgt_to_gps", "reset_hgt_to_baro",
    "reset_pos_to_last_known", "reset_vel_to_zero", "yaw_aligned_to_imu_gps",
    "starting_gps_fusion", "gps_checks_passed",
]

PARAMS_OF_INTEREST = [
    "SDLOG_PROFILE", "SDLOG_MODE",
    "EKF2_MULTI_IMU", "EKF2_MAG_TYPE", "EKF2_GPS_CTRL", "EKF2_HGT_REF",
    "EKF2_GPS_YAW_OFF", "EKF2_GPS_POS_X", "EKF2_GPS_POS_Y", "EKF2_GPS_POS_Z",
    "EKF2_GPS_P_NOISE", "EKF2_GPS_V_NOISE", "EKF2_HEAD_NOISE",
    "EKF2_WENC_CTRL", "EKF2_WENC_RAD", "EKF2_WENC_NOISE", "EKF2_WENC_LAT_N",
    "EKF2_WENC_GATE",
    "RBCLW_COUNTS_REV", "RD_WHEEL_TRACK", "RO_MAX_THR_SPEED",
]

GYRO_BIAS_THRESHOLD_RAD = 5e-4  # ~3 deg/s equivalent drift over a run


# ---------------------------------------------------------------- helpers


def get(ulog: ULog, name: str, instance: int = 0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == instance:
            return d
    return None


def instances(ulog: ULog, name: str) -> list[int]:
    return sorted(d.multi_id for d in ulog.data_list if d.name == name)


def rel_t(ulog: ULog, dataset, field: str = "timestamp") -> np.ndarray:
    # uORB timestamps are uint64; subtracting in-dtype wraps for samples that
    # predate start_timestamp, so widen first.
    return (dataset.data[field].astype(np.float64) - float(ulog.start_timestamp)) / 1e6


def mask(t: np.ndarray, lo: float, hi: float) -> np.ndarray:
    return (t >= lo) & (t < hi)


def stats(v: np.ndarray) -> dict:
    v = v[np.isfinite(v)]
    if v.size == 0:
        return {}
    return {
        "n": v.size,
        "mean": float(np.mean(v)),
        "p95": float(np.percentile(v, 95)),
        "max": float(np.max(v)),
        "pct_gt_05": 100.0 * float(np.mean(v > 0.5)),
        "pct_gt_10": 100.0 * float(np.mean(v > 1.0)),
    }


def verdict(s: dict) -> str:
    if not s:
        return "no data"
    if s["pct_gt_10"] > 1.0:
        return "REJECTING"
    if s["p95"] > 0.5:
        return "marginal"
    return "healthy"


def wrap180(a: np.ndarray) -> np.ndarray:
    return (a + 180.0) % 360.0 - 180.0


def fld(dataset, base: str) -> list[str]:
    """Field names for a scalar-or-array uORB field, in index order."""
    if base in dataset.data:
        return [base]
    out = []
    i = 0
    while f"{base}[{i}]" in dataset.data:
        out.append(f"{base}[{i}]")
        i += 1
    return out


# ---------------------------------------------------------------- sections


def section_header(ulog: ULog, path: Path, out: list[str]) -> None:
    dur = (ulog.last_timestamp - ulog.start_timestamp) / 1e6
    out.append(f"# EKF2 fusion health — `{path.name}`\n")
    out.append(f"- duration **{dur:.1f} s**, dropouts **{len(ulog.dropouts)}**")

    info = ulog.msg_info_dict
    for k in ("ver_sw", "ver_sw_release", "ver_hw", "sys_uuid"):
        if k in info:
            out.append(f"- `{k}` = `{info[k]}`")
    out.append(
        "  - ⚠ `ver_sw` is the **base checkout** hash. This firmware is built by "
        "CI as stock PX4 v1.16.2 + `cp` overlays (`.github/workflows/build_rover.yml`), "
        "so `ver_sw` does **not** identify which fork patches are in the binary. "
        "Confirm the overlay set separately."
    )

    out.append("\n## Parameters as flown\n")
    p = ulog.initial_parameters
    out.append("| param | value |")
    out.append("|---|---|")
    for k in PARAMS_OF_INTEREST:
        out.append(f"| `{k}` | {p.get(k, '_absent_')} |")

    out.append("\n## Topic presence\n")
    present = {d.name for d in ulog.data_list}
    for name in AID_SOURCES + ["wheel_encoders", "estimator_status", "estimator_status_flags",
                               "estimator_event_flags", "vehicle_imu_status"]:
        n = instances(ulog, name)
        out.append(f"- `{name}`: {'yes, instances ' + str(n) if name in present else '**ABSENT**'}")
    out.append(
        "\n> `estimator_*` topics reach the log through the logger's `strncmp(o_name, \"estimator\", 9)` "
        "glob in `add_default_topics()` (present in stock v1.16.2), which registers them as "
        "**optional** at 2 Hz. Optional topics are skipped when never advertised, so an absent "
        "`estimator_aid_src_*` means EKF2 never published it — i.e. that fuser processed **zero "
        "samples**. It does *not* by itself say why."
    )


def section_driver(ulog: ULog, out: list[str]) -> None:
    """RoboClaw serial health. No wheel_encoders publish => wheel fusion cannot run."""
    out.append("\n## Encoder driver health (root-cause gate for wheel fusion)\n")
    t0 = ulog.start_timestamp
    dur = (ulog.last_timestamp - t0) / 1e6
    buckets = {"Error reading encoders": 0, "Checksum mismatch": 0, "ACK wrong": 0}
    first = last = None
    for m in ulog.logged_messages:
        for key in buckets:
            if key in m.message:
                buckets[key] += 1
                ts = (m.timestamp - t0) / 1e6
                first = ts if first is None else first
                last = ts
    total = sum(buckets.values())
    if total == 0:
        out.append("- no RoboClaw errors logged ✅")
    else:
        out.append(f"| message | count | rate |")
        out.append("|---|---|---|")
        for k, v in buckets.items():
            out.append(f"| `{k}` | {v} | {v / dur:.1f} /s |")
        out.append(f"\n- span **{first:.1f} s → {last:.1f} s** of a {dur:.1f} s log")
        out.append(
            "- `Roboclaw::readEncoder()` returns early on any failed transaction, so "
            "`_wheel_encoders_pub.publish()` is **never reached** on those cycles. A sustained "
            "error rate here means EKF2 receives no `wheel_encoders` samples and "
            "`EKF2_WENC_CTRL=1` is inert regardless of tuning. **Fix the serial link before "
            "drawing any conclusion about wheel-encoder fusion.**"
        )

    we = get(ulog, "wheel_encoders")
    if we is None:
        out.append(
            "- `wheel_encoders` not in this log. It is only logged by firmware carrying the "
            "`logged_topics.cpp` overlay (fork commit `06309e41a7`, 2026-06-08 13:18 IST). "
            "Older binaries cannot show raw encoder data at all."
        )
    else:
        t = rel_t(ulog, we)
        hz = len(t) / (t[-1] - t[0]) if len(t) > 1 else 0.0
        out.append(f"- `wheel_encoders` present: {len(t)} samples, {hz:.1f} Hz ✅")
        for f in fld(we, "wheel_speed"):
            v = we.data[f]
            out.append(f"  - `{f}` range {np.min(v):+.3f} .. {np.max(v):+.3f} rad/s")


def section_status(ulog: ULog, segments, inst: int, out: list[str]) -> None:
    """estimator_status rollup ratios. Note: the heading ratio is hdg_test_ratio."""
    st = get(ulog, "estimator_status", inst)
    out.append(f"\n## `estimator_status` test ratios — instance {inst}\n")
    if st is None:
        out.append("- topic absent")
        return
    t = rel_t(ulog, st)
    ratios = [("vel", "vel_test_ratio"), ("pos", "pos_test_ratio"),
              ("hgt", "hgt_test_ratio"), ("hdg", "hdg_test_ratio")]
    out.append("| segment | ratio | mean | p95 | max | %>0.5 | %>1.0 | verdict |")
    out.append("|---|---|---|---|---|---|---|---|")
    for name, lo, hi in segments:
        m = mask(t, lo, hi)
        for label, f in ratios:
            if f not in st.data:
                continue
            s = stats(st.data[f][m])
            if not s:
                continue
            out.append(
                f"| {name} | {label} | {s['mean']:.3f} | {s['p95']:.3f} | {s['max']:.3f} "
                f"| {s['pct_gt_05']:.1f} | {s['pct_gt_10']:.1f} | {verdict(s)} |"
            )
    out.append(
        "\n> `hdg_test_ratio` — not `mag_test_ratio`, which does not exist in this message — "
        "carries the heading innovation. With `EKF2_MAG_TYPE=5` it is fed entirely by the "
        "UM982 dual-antenna baseline."
    )

    # Reset counters are monotonic; any increase is a filter reset.
    out.append("\n### Filter resets (monotonic counters)\n")
    any_reset = False
    for f in ("reset_count_pos_ne", "reset_count_vel_ne", "reset_count_vel_d",
              "reset_count_quat", "reset_count_pod_d"):
        if f not in st.data:
            continue
        v = st.data[f]
        if v[-1] != v[0]:
            any_reset = True
            idx = np.where(np.diff(v) != 0)[0]
            times = ", ".join(f"{t[i + 1]:.1f}s" for i in idx[:12])
            out.append(f"- `{f}`: {v[0]} → {v[-1]} at {times}")
    if not any_reset:
        out.append("- none ✅")


def section_aid_sources(ulog: ULog, segments, inst: int, out: list[str]) -> None:
    out.append(f"\n## Aid sources — instance {inst}\n")
    out.append("| segment | source | axis | test_ratio p95 | max | %>1.0 | \\|innov\\| mean | "
               "σ_innov mean | fused % | rejected % |")
    out.append("|---|---|---|---|---|---|---|---|---|---|")
    for src in AID_SOURCES:
        d = get(ulog, src, inst)
        if d is None:
            continue
        t = rel_t(ulog, d)
        tr = fld(d, "test_ratio")
        iv = fld(d, "innovation")
        var = fld(d, "innovation_variance")
        for name, lo, hi in segments:
            m = mask(t, lo, hi)
            if not m.any():
                continue
            fused = d.data.get("fused")
            rej = d.data.get("innovation_rejected")
            fu = 100.0 * float(np.mean(fused[m].astype(bool))) if fused is not None else float("nan")
            rj = 100.0 * float(np.mean(rej[m].astype(bool))) if rej is not None else float("nan")
            for ax in range(len(tr)):
                s = stats(d.data[tr[ax]][m])
                if not s:
                    continue
                innov = np.abs(d.data[iv[ax]][m]) if ax < len(iv) else np.array([np.nan])
                sig = np.sqrt(np.abs(d.data[var[ax]][m])) if ax < len(var) else np.array([np.nan])
                out.append(
                    f"| {name} | {src.replace('estimator_aid_src_', '')} | {ax} "
                    f"| {s['p95']:.3f} | {s['max']:.3f} | {s['pct_gt_10']:.1f} "
                    f"| {np.nanmean(innov):.4f} | {np.nanmean(sig):.4f} | {fu:.0f} | {rj:.1f} |"
                )
    missing = [s for s in AID_SOURCES if get(ulog, s, inst) is None]
    if missing:
        out.append("\n**Never published (zero samples processed):** " +
                   ", ".join(f"`{m}`" for m in missing))


def section_flags(ulog: ULog, out: list[str]) -> None:
    out.append("\n## `estimator_status_flags` timeline (instance 0)\n")
    fl = get(ulog, "estimator_status_flags", 0)
    if fl is None:
        out.append("- topic absent")
        return
    t = rel_t(ulog, fl)
    for counter in ("control_status_changes", "fault_status_changes",
                    "innovation_fault_status_changes"):
        c = fl.data.get(counter)
        if c is not None and len(c):
            n = int(np.count_nonzero(np.diff(c.astype(np.int64))))
            out.append(f"- `{counter}` {int(c[0])} → {int(c[-1])} ({n} changes in-log)")
    rows = 0
    for f in WATCH_FLAGS:
        if f not in fl.data:
            continue
        v = fl.data[f].astype(int)
        changes = np.where(np.diff(v) != 0)[0]
        if changes.size == 0:
            out.append(f"- `{f}`: constant **{bool(v[0])}** for the whole log")
        else:
            seq = ", ".join(f"{t[i + 1]:.1f}s→{bool(v[i + 1])}" for i in changes[:16])
            more = "" if changes.size <= 16 else f" (+{changes.size - 16} more)"
            out.append(f"- `{f}`: starts {bool(v[0])}; {seq}{more}")
        rows += 1
    if not rows:
        out.append("- no watched flags present")

    ev = get(ulog, "estimator_event_flags", 0)
    out.append("\n### `estimator_event_flags`\n")
    if ev is None:
        out.append("- topic absent")
        return
    te = rel_t(ulog, ev)
    # The flag fields are LATCHED and the topic is also republished ~1 Hz as a
    # heartbeat, so counting non-zero samples over-reports wildly. The authority on
    # "did anything happen" is the change counter.
    chg = ev.data.get("information_event_changes")
    if chg is not None and len(chg):
        n_new = int(np.count_nonzero(np.diff(chg.astype(np.int64))))
        out.append(f"- `information_event_changes` {int(chg[0])} → {int(chg[-1])} "
                   f"({n_new} change events inside the log)")
        if n_new == 0:
            out.append("  - no new estimator events during the log ✅ "
                       "(flags below are latched from EKF start-up, before logging)")
    for f in RESET_EVENTS:
        if f not in ev.data:
            continue
        v = ev.data[f].astype(int)
        rises = np.where(np.diff(v) > 0)[0]
        if rises.size:
            out.append(f"- `{f}` **transitioned 0→1** at " +
                       ", ".join(f"{te[i + 1]:.1f}s" for i in rises[:10]))
        elif v[0]:
            out.append(f"- `{f}` latched true on entry (no in-log transition)")


def section_bias_and_vibe(ulog: ULog, segments, out: list[str]) -> None:
    out.append("\n## Gyro-bias drift and IMU health\n")
    for inst in instances(ulog, "estimator_states"):
        d = get(ulog, "estimator_states", inst)
        cols = [f"states[{i}]" for i in (10, 11, 12)]
        if not all(c in d.data for c in cols):
            continue
        drift = [float(d.data[c][-1] - d.data[c][0]) for c in cols]
        worst = max(abs(x) for x in drift)
        flag = "⚠ EXCEEDS" if worst > GYRO_BIAS_THRESHOLD_RAD else "ok"
        out.append(
            f"- EKF{inst} gyro-bias Δ over run: "
            + ", ".join(f"{x:+.2e}" for x in drift)
            + f" rad — worst {worst:.2e} vs {GYRO_BIAS_THRESHOLD_RAD:.0e} threshold ({flag})"
        )

    out.append("")
    for inst in instances(ulog, "vehicle_imu_status"):
        d = get(ulog, "vehicle_imu_status", inst)
        av = d.data.get("accel_vibration_metric")
        gv = d.data.get("gyro_vibration_metric")
        clip = 0
        for ax in range(3):
            c = d.data.get(f"accel_clipping[{ax}]")
            if c is not None and len(c):
                clip += int(c[-1] - c[0])
        parts = [f"- IMU{inst}"]
        if av is not None and len(av):
            parts.append(f"accel vibe mean {np.mean(av):.3f} max {np.max(av):.3f}")
        if gv is not None and len(gv):
            parts.append(f"gyro vibe mean {np.mean(gv):.4f}")
        parts.append(f"accel clipping events {clip}" + (" ⚠" if clip else ""))
        out.append("  ".join(parts))
    out.append(
        "\n> Vibration is the common cause when vel + pos + hgt ratios all rise together while "
        "heading stays calm. Clipping counters that advance invalidate every other reading."
    )


def section_cross_instance(ulog: ULog, out: list[str]) -> None:
    out.append("\n## Cross-instance divergence\n")
    inst = instances(ulog, "estimator_local_position")
    if len(inst) < 2:
        out.append("- single EKF instance logged; no cross-check available")
        return
    ref = get(ulog, "estimator_local_position", inst[0])
    tref = rel_t(ulog, ref)
    for i in inst[1:]:
        d = get(ulog, "estimator_local_position", i)
        t = rel_t(ulog, d)
        rows = []
        for ax in ("x", "y", "z"):
            if ax not in ref.data or ax not in d.data:
                continue
            a = np.interp(tref, t, d.data[ax])
            rows.append(f"{ax} max |Δ| {np.max(np.abs(a - ref.data[ax])):.3f} m")
        out.append(f"- EKF{i} vs EKF{inst[0]}: " + ", ".join(rows))
    out.append(
        "\n> Instances share GNSS but not IMU. Large divergence points at the IMU/vibration side; "
        "instances agreeing while ratios are bad points at GNSS or at yaw."
    )


def section_yaw_vs_cog(ulog: ULog, segments, out: list[str]) -> None:
    """Direct EKF2_GPS_YAW_OFF test, self-contained in the log.

    A non-holonomic rover driving forward has course-over-ground == true heading.
    A yaw-offset error delta shows up as a constant (EKF yaw - COG) = delta with the
    SAME sign in both drive directions -- which is what distinguishes it from a
    controller/pure-pursuit artefact, whose path-frame cross-track flips sign.
    """
    out.append("\n## Heading check: EKF yaw vs course-over-ground\n")
    lp = get(ulog, "estimator_local_position", 0)
    att = get(ulog, "estimator_attitude", 0)
    if lp is None or att is None:
        out.append("- `estimator_local_position` or `estimator_attitude` absent")
        return
    need = ("q[0]", "q[1]", "q[2]", "q[3]")
    if not all(k in att.data for k in need):
        out.append("- quaternion fields absent")
        return

    ta = rel_t(ulog, att)
    q0, q1, q2, q3 = (att.data[k] for k in need)
    yaw = np.degrees(np.arctan2(2.0 * (q0 * q3 + q1 * q2),
                                1.0 - 2.0 * (q2 * q2 + q3 * q3)))  # NED yaw, deg

    tl = rel_t(ulog, lp)
    vx, vy = lp.data.get("vx"), lp.data.get("vy")
    if vx is None or vy is None:
        out.append("- velocity fields absent")
        return
    speed = np.hypot(vx, vy)
    cog = np.degrees(np.arctan2(vy, vx))  # NED: x=north, y=east

    cog_i = np.interp(ta, tl, np.unwrap(np.radians(cog)))
    cog_i = np.degrees(cog_i)
    spd_i = np.interp(ta, tl, speed)

    out.append("| segment | n@>0.25 m/s | mean speed | mean(yaw−COG) | sd | median |")
    out.append("|---|---|---|---|---|---|")
    per_seg = {}
    for name, lo, hi in segments:
        m = mask(ta, lo, hi) & (spd_i > 0.25)
        if m.sum() < 5:
            out.append(f"| {name} | {int(m.sum())} | — | _too little motion_ | | |")
            continue
        diff = wrap180(yaw[m] - cog_i[m])
        # Scatter above a few degrees means COG is dominated by GPS velocity noise
        # at low speed, not by a heading error. Do not let it drive a verdict.
        weak = " ⚠ noisy" if (m.sum() < 30 or np.std(diff) > 10.0) else ""
        if not weak:
            per_seg[name] = float(np.median(diff))
        out.append(
            f"| {name} | {int(m.sum())}{weak} | {np.mean(spd_i[m]):.2f} m/s "
            f"| {np.mean(diff):+.2f}° | {np.std(diff):.2f}° | {np.median(diff):+.2f}° |"
        )

    out.append(
        "\n**Reading it:** a constant same-sign offset across out-and-back segments is a "
        "`EKF2_GPS_YAW_OFF` / antenna-baseline error (bug B1). A value that flips sign with "
        "direction is a lateral antenna offset (`EKF2_GPS_POS_Y`, bug B3) or real side-slip, "
        "not a yaw constant. Sub-degree scatter with near-zero mean clears both."
    )
    if len(per_seg) >= 2:
        names = list(per_seg)
        a, b = per_seg[names[0]], per_seg[names[1]]
        same = "SAME sign → yaw-offset candidate" if a * b > 0 else "OPPOSITE sign → not a yaw constant"
        out.append(f"\n- `{names[0]}` {a:+.2f}° vs `{names[1]}` {b:+.2f}° — {same}")


def section_speed_scale(ulog: ULog, segments, out: list[str]) -> None:
    """Segment 1 vs 3 slope test: scale errors grow with speed, fixed biases do not."""
    out.append("\n## Innovation vs speed (scale-error separation)\n")
    lp = get(ulog, "estimator_local_position", 0)
    src = get(ulog, "estimator_aid_src_gnss_vel", 0)
    if lp is None or src is None:
        out.append("- required topics absent")
        return
    tl = rel_t(ulog, lp)
    speed = np.hypot(lp.data["vx"], lp.data["vy"])
    ts = rel_t(ulog, src)
    spd_i = np.interp(ts, tl, speed)
    axes = fld(src, "innovation")
    out.append("| segment | mean speed | mean \\|innov\\| vel N | E | D |")
    out.append("|---|---|---|---|---|")
    for name, lo, hi in segments:
        m = mask(ts, lo, hi) & (spd_i > 0.05)
        if m.sum() < 5:
            continue
        vals = [np.nanmean(np.abs(src.data[a][m])) for a in axes]
        out.append(f"| {name} | {np.mean(spd_i[m]):.2f} m/s | "
                   + " | ".join(f"{v:.4f}" for v in vals) + " |")
    out.append(
        "\n> Compare the slow and fast straight segments. Innovation growing roughly in "
        "proportion to speed indicates a scale error (`EKF2_WENC_RAD`, `RBCLW_COUNTS_REV`); "
        "flat innovation across speeds indicates a fixed bias."
    )


# ---------------------------------------------------------------- driver


def parse_segments(path: Path | None, ulog: ULog):
    dur = (ulog.last_timestamp - ulog.start_timestamp) / 1e6
    if path is None:
        return [("ALL", 0.0, dur + 1.0)]
    segs = []
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 3:
            sys.exit(f"bad segment line: {raw!r} (want: name start_s end_s)")
        segs.append((parts[0], float(parts[1]), float(parts[2])))
    return segs or [("ALL", 0.0, dur + 1.0)]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulg", type=Path)
    ap.add_argument("--segments", type=Path, help="segment definition file")
    ap.add_argument("--instance", type=int, default=0, help="EKF instance for detail tables")
    ap.add_argument("-o", "--out", type=Path, help="write markdown here instead of stdout")
    args = ap.parse_args()

    if not args.ulg.exists():
        sys.exit(f"no such log: {args.ulg}")

    ulog = ULog(str(args.ulg))
    segments = parse_segments(args.segments, ulog)

    out: list[str] = []
    section_header(ulog, args.ulg, out)
    section_driver(ulog, out)
    out.append("\n## Segments\n")
    for name, lo, hi in segments:
        out.append(f"- **{name}**: {lo:.1f} – {hi:.1f} s")
    section_status(ulog, segments, args.instance, out)
    section_aid_sources(ulog, segments, args.instance, out)
    section_flags(ulog, out)
    section_yaw_vs_cog(ulog, segments, out)
    section_speed_scale(ulog, segments, out)
    section_bias_and_vibe(ulog, segments, out)
    section_cross_instance(ulog, out)

    text = "\n".join(out) + "\n"
    if args.out:
        args.out.write_text(text)
        print(f"wrote {args.out}")
    else:
        print(text)


if __name__ == "__main__":
    main()
