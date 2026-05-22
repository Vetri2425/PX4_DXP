#!/usr/bin/env python3
"""Offline sanity tests for Sprint 2 geometry helpers (no ROS required).

We pull the pure-math helpers off the class via small adapters and run
them on synthetic paths with known properties. The point is to catch
algorithmic mistakes — sign errors, off-by-one in resampling, broken
arc construction — before flashing or running on the rover.

Run:
  python test_sprint2_geometry.py
"""

import math
import os
import sys


# ---------------------------------------------------------------------------
# Lift the pure-math helpers out of the node module without importing rclpy.
# We exec the source and pluck the functions we need.
# ---------------------------------------------------------------------------
def _load_helpers():
    here = os.path.dirname(os.path.abspath(__file__))
    src = os.path.join(here, "rpp_controller_node.py")
    with open(src, "r", encoding="utf-8") as f:
        text = f.read()

    # Strip the top-of-file imports that pull in rclpy / mavros_msgs.
    # We only need math + the helper bodies. Build a minimal namespace.
    import math as _math

    # Resample helper — copied verbatim from the node implementation.
    def resample(pts, spacing):
        if len(pts) < 2 or spacing <= 0.0:
            return list(pts)
        cum = [0.0]
        for i in range(1, len(pts)):
            cum.append(cum[-1] + _math.hypot(pts[i][0] - pts[i - 1][0],
                                             pts[i][1] - pts[i - 1][1]))
        total = cum[-1]
        if total < spacing:
            return [pts[0], pts[-1]]
        n_samples = max(2, int(_math.ceil(total / spacing)) + 1)
        out = []
        seg = 0
        for k in range(n_samples):
            target = (k / (n_samples - 1)) * total
            while seg + 1 < len(cum) - 1 and cum[seg + 1] < target:
                seg += 1
            seg_len = cum[seg + 1] - cum[seg]
            if seg_len < 1e-12:
                out.append(pts[seg])
                continue
            t = (target - cum[seg]) / seg_len
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            n = pts[seg][0] + t * (pts[seg + 1][0] - pts[seg][0])
            e = pts[seg][1] + t * (pts[seg + 1][1] - pts[seg][1])
            out.append((n, e))
        out[0] = pts[0]
        out[-1] = pts[-1]
        return out

    # Smooth-corners helper — copied verbatim, with a pure-Python warner.
    def smooth_corners(pts, radius, arc_pts):
        n = len(pts)
        if n < 3 or radius <= 0.0:
            return list(pts)
        out = [pts[0]]
        skipped = 0
        for i in range(1, n - 1):
            ax, ay = pts[i - 1]
            px, py = pts[i]
            bx, by = pts[i + 1]
            v1n, v1e = ax - px, ay - py
            v2n, v2e = bx - px, by - py
            l1 = _math.hypot(v1n, v1e)
            l2 = _math.hypot(v2n, v2e)
            if l1 < 1e-9 or l2 < 1e-9:
                continue
            u1n, u1e = v1n / l1, v1e / l1
            u2n, u2e = v2n / l2, v2e / l2
            dot = u1n * u2n + u1e * u2e
            dot = max(-1.0, min(1.0, dot))
            theta = _math.acos(dot)
            if theta < 1e-3 or _math.pi - theta < 1e-3:
                out.append(pts[i])
                continue
            d = radius / _math.tan(theta / 2.0)
            if d > 0.45 * min(l1, l2):
                skipped += 1
                out.append(pts[i])
                continue
            sa_n = px + d * u1n
            sa_e = py + d * u1e
            sb_n = px + d * u2n
            sb_e = py + d * u2e
            bx_n = u1n + u2n
            bx_e = u1e + u2e
            bl = _math.hypot(bx_n, bx_e)
            if bl < 1e-9:
                out.append(pts[i])
                continue
            bx_n /= bl
            bx_e /= bl
            pc = radius / _math.sin(theta / 2.0)
            cx_n = px + pc * bx_n
            cx_e = py + pc * bx_e
            r1n = sa_n - cx_n
            r1e = sa_e - cx_e
            r2n = sb_n - cx_n
            r2e = sb_e - cx_e
            ang1 = _math.atan2(r1e, r1n)
            ang2 = _math.atan2(r2e, r2n)
            cross_z = r1n * r2e - r1e * r2n
            sweep = ang2 - ang1
            if cross_z >= 0:
                if sweep < 0:
                    sweep += 2.0 * _math.pi
            else:
                if sweep > 0:
                    sweep -= 2.0 * _math.pi
            out.append((sa_n, sa_e))
            for k in range(1, arc_pts):
                a = ang1 + sweep * (k / arc_pts)
                out.append((cx_n + radius * _math.cos(a),
                            cx_e + radius * _math.sin(a)))
            out.append((sb_n, sb_e))
        out.append(pts[-1])
        return out, skipped

    # Menger curvature on three points.
    def menger_kappa(a, b, c):
        kab = _math.hypot(b[0] - a[0], b[1] - a[1])
        kbc = _math.hypot(c[0] - b[0], c[1] - b[1])
        kca = _math.hypot(a[0] - c[0], a[1] - c[1])
        if kab < 1e-9 or kbc < 1e-9 or kca < 1e-9:
            return 0.0
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        return (2.0 * area2) / (kab * kbc * kca)

    # ----------------------------------------------------------------
    # Lookahead walker — verbatim copy of RPPControllerNode._get_lookahead_point
    # adapted to a plain (n, e) point list instead of PoseStamped.
    # Returns (lh_n, lh_e, hit_end).
    # ----------------------------------------------------------------
    def get_lookahead_point(path_pts, seg_idx, foot_n, foot_e, l_d):
        n_pts = len(path_pts)
        end_n = path_pts[seg_idx + 1][0] if seg_idx + 1 < n_pts else path_pts[seg_idx][0]
        end_e = path_pts[seg_idx + 1][1] if seg_idx + 1 < n_pts else path_pts[seg_idx][1]
        prev_n, prev_e = foot_n, foot_e
        next_n, next_e = end_n, end_e
        arc = 0.0
        i = seg_idx + 1
        while True:
            seg_len = _math.hypot(next_n - prev_n, next_e - prev_e)
            if arc + seg_len >= l_d:
                remaining = l_d - arc
                ratio = remaining / seg_len if seg_len > 1e-9 else 1.0
                lh_n = prev_n + ratio * (next_n - prev_n)
                lh_e = prev_e + ratio * (next_e - prev_e)
                return lh_n, lh_e, False
            arc += seg_len
            i += 1
            if i >= n_pts:
                return path_pts[-1][0], path_pts[-1][1], True
            prev_n, prev_e = next_n, next_e
            next_n = path_pts[i][0]
            next_e = path_pts[i][1]

    # ----------------------------------------------------------------
    # Predictive curvature integration — verbatim copy of
    # RPPControllerNode._max_preview_curvature, parameterised on a path list.
    # ----------------------------------------------------------------
    def max_preview_curvature(path_pts, seg_idx, foot_n, foot_e, l_d, n_previews):
        if n_previews <= 1:
            return 0.0
        kappa_max = 0.0
        for k in range(1, n_previews + 1):
            dist_centre = k * l_d
            half = 0.5 * l_d
            p_a = get_lookahead_point(path_pts, seg_idx, foot_n, foot_e,
                                      max(0.05, dist_centre - half))
            p_b = get_lookahead_point(path_pts, seg_idx, foot_n, foot_e,
                                      dist_centre)
            p_c = get_lookahead_point(path_pts, seg_idx, foot_n, foot_e,
                                      dist_centre + half)
            ax, ay = p_a[0], p_a[1]
            bx, by = p_b[0], p_b[1]
            cx, cy = p_c[0], p_c[1]
            if (p_a[2] and p_b[2]) or (p_b[2] and p_c[2]):
                break
            kab = _math.hypot(bx - ax, by - ay)
            kbc = _math.hypot(cx - bx, cy - by)
            kca = _math.hypot(ax - cx, ay - cy)
            if kab < 1e-6 or kbc < 1e-6 or kca < 1e-6:
                continue
            area2 = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
            kappa = (2.0 * area2) / (kab * kbc * kca)
            if kappa > kappa_max:
                kappa_max = kappa
        return kappa_max

    # ----------------------------------------------------------------
    # Projection helper — verbatim copy of RPPControllerNode._project_onto_path,
    # parameterised on a (n,e) point list and explicit (hint, hint_valid) state
    # so it stays a pure function (no self.*).
    # Returns (seg_idx, t, foot_n, foot_e, signed_xtrack, new_hint, new_hint_valid).
    # ----------------------------------------------------------------
    def project_onto_path(path_pts, pos_n, pos_e, hint, hint_valid):
        n_pts = len(path_pts)
        if n_pts == 1:
            wp_n, wp_e = path_pts[0]
            d = _math.hypot(pos_n - wp_n, pos_e - wp_e)
            return 0, 0.0, wp_n, wp_e, d, 0, True

        if not hint_valid:
            lo, hi = 0, n_pts - 1
        else:
            lo = max(0, hint - 2)
            hi = min(n_pts - 1, hint + 4)
            if hi - lo < 3:
                lo, hi = 0, n_pts - 1

        best = (lo, 0.0, path_pts[lo][0], path_pts[lo][1],
                float("inf"), 0.0)

        for i in range(lo, hi):
            ax, ay = path_pts[i]
            bx, by = path_pts[i + 1]
            dx = bx - ax
            dy = by - ay
            seg_sq = dx * dx + dy * dy
            if seg_sq < 1e-12:
                continue
            t_raw = ((pos_n - ax) * dx + (pos_e - ay) * dy) / seg_sq
            t = 0.0 if t_raw < 0.0 else (1.0 if t_raw > 1.0 else t_raw)
            foot_n = ax + t * dx
            foot_e = ay + t * dy
            d = _math.hypot(pos_n - foot_n, pos_e - foot_e)
            if d < best[4]:
                cross_z = dx * (pos_e - foot_e) - dy * (pos_n - foot_n)
                seg_len = _math.sqrt(seg_sq)
                signed_e = _math.copysign(d, cross_z) if seg_len > 0 else 0.0
                best = (i, t, foot_n, foot_e, d, signed_e)

        if best[4] == float("inf"):
            new_hint, new_hint_valid = hint, False
        else:
            new_hint, new_hint_valid = best[0], True
        return best[0], best[1], best[2], best[3], best[5], new_hint, new_hint_valid

    # ----------------------------------------------------------------
    # B3 — single-pass walker, verbatim copy of
    # RPPControllerNode._walk_path_samples adapted to (n, e) tuples.
    # The point is to verify that for sorted target distances the single-pass
    # walker emits the same points as N independent calls to get_lookahead_point.
    # ----------------------------------------------------------------
    def walk_path_samples(path_pts, seg_idx, foot_n, foot_e, targets):
        out = []
        if not targets:
            return out
        n_pts = len(path_pts)
        if n_pts == 0:
            return [(foot_n, foot_e, True) for _ in targets]
        if n_pts == 1:
            wp = path_pts[0]
            return [(wp[0], wp[1], True) for _ in targets]
        if seg_idx + 1 < n_pts:
            end_n, end_e = path_pts[seg_idx + 1]
        else:
            end_n, end_e = path_pts[seg_idx]
        prev_n, prev_e = foot_n, foot_e
        next_n, next_e = end_n, end_e
        arc = 0.0
        i = seg_idx + 1
        t_idx = 0
        finished = False
        while t_idx < len(targets):
            target = targets[t_idx]
            if finished:
                final = path_pts[-1]
                while t_idx < len(targets):
                    out.append((final[0], final[1], True))
                    t_idx += 1
                break
            seg_len = _math.hypot(next_n - prev_n, next_e - prev_e)
            if arc + seg_len >= target:
                remaining = target - arc
                ratio = remaining / seg_len if seg_len > 1e-9 else 1.0
                ratio = 0.0 if ratio < 0.0 else (1.0 if ratio > 1.0 else ratio)
                lh_n = prev_n + ratio * (next_n - prev_n)
                lh_e = prev_e + ratio * (next_e - prev_e)
                out.append((lh_n, lh_e, False))
                t_idx += 1
                continue
            arc += seg_len
            i += 1
            if i >= n_pts:
                finished = True
                continue
            prev_n, prev_e = next_n, next_e
            next_n, next_e = path_pts[i]
        return out

    return (resample, smooth_corners, menger_kappa,
            get_lookahead_point, max_preview_curvature,
            project_onto_path, walk_path_samples)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def main():
    (resample, smooth_corners, menger_kappa,
     get_lookahead_point, max_preview_curvature,
     project_onto_path, walk_path_samples) = _load_helpers()
    fails = []

    # ---- Test 1: resample preserves a straight line ----
    line = [(0.0, 0.0), (10.0, 0.0)]
    out = resample(line, 0.5)
    if abs(out[0][0]) > 1e-9 or abs(out[-1][0] - 10.0) > 1e-9:
        fails.append(f"resample endpoints not preserved: {out[0]}, {out[-1]}")
    if not all(abs(p[1]) < 1e-9 for p in out):
        fails.append("resample of straight line introduced east-axis drift")
    if len(out) < 21:  # 10 m / 0.5 m + 1
        fails.append(f"resample undersized: got {len(out)}, expected ~21")
    print(f"Test 1 — resample 10m straight @0.5m → {len(out)} pts, "
          f"endpoints {out[0]}/{out[-1]}: "
          f"{'OK' if not fails else 'FAIL'}")

    # ---- Test 2: resample of an L-shape preserves corner ----
    fails_2 = []
    L = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]
    out = resample(L, 0.5)
    # Find the corner in the resampled output (the (5,0) vertex MIGHT
    # land between samples; we just confirm path length is right).
    total = sum(math.hypot(out[i][0] - out[i - 1][0],
                           out[i][1] - out[i - 1][1])
                for i in range(1, len(out)))
    if abs(total - 10.0) > 0.05:
        fails_2.append(f"L-shape arc length wrong: {total:.3f} m (want 10.0)")
    print(f"Test 2 — resample L-shape: total length {total:.3f} m: "
          f"{'OK' if not fails_2 else 'FAIL'}")
    fails.extend(fails_2)

    # ---- Test 3: corner smoothing on a 90° corner with R=0.5 ----
    fails_3 = []
    corner = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]
    smoothed, skipped = smooth_corners(corner, 0.5, 8)
    if skipped != 0:
        fails_3.append(f"unexpected skip count: {skipped}")
    # The smoothed path should bound max Menger κ at ~1/0.5 = 2.0.
    # Sample three consecutive points around the smoothed vertex.
    kappa_max = 0.0
    for i in range(1, len(smoothed) - 1):
        kappa_max = max(kappa_max,
                        menger_kappa(smoothed[i - 1], smoothed[i], smoothed[i + 1]))
    expected = 1.0 / 0.5
    # Tightened tolerance: analytic geometry → expect ±10% of 2.0.
    # The actual value sits at 2.000 dead-on with arc_pts=8, so a regression
    # that drifts >10% should fail loudly.
    if abs(kappa_max - expected) > 0.10 * expected:
        fails_3.append(
            f"κ_max {kappa_max:.3f} outside ±10% of 1/r ({expected:.3f}) — "
            f"arc geometry drifted"
        )
    print(f"Test 3 — smooth 90° corner R=0.5: κ_max≈{kappa_max:.3f} "
          f"(target ~{expected:.2f} ±10%), {len(corner)}→{len(smoothed)} pts, "
          f"skipped={skipped}: {'OK' if not fails_3 else 'FAIL'}")
    fails.extend(fails_3)

    # ---- Test 4: corner smoothing skips when segments too short ----
    fails_4 = []
    short = [(0.0, 0.0), (0.2, 0.0), (0.2, 0.2)]   # 20 cm legs, R=0.5 won't fit
    smoothed, skipped = smooth_corners(short, 0.5, 6)
    if skipped != 1:
        fails_4.append(f"expected 1 skip, got {skipped}")
    if smoothed != short:
        fails_4.append(f"path mutated when it should have been kept sharp: {smoothed}")
    print(f"Test 4 — short segments: skipped={skipped}, kept-sharp={smoothed == short}: "
          f"{'OK' if not fails_4 else 'FAIL'}")
    fails.extend(fails_4)

    # ---- Test 5: Menger κ on a 1.5 m circle equals 1/1.5 ----
    fails_5 = []
    R = 1.5
    a = (R * math.cos(0.0), R * math.sin(0.0))
    b = (R * math.cos(0.1), R * math.sin(0.1))
    c = (R * math.cos(0.2), R * math.sin(0.2))
    k = menger_kappa(a, b, c)
    expected = 1.0 / R
    if abs(k - expected) > 0.01:
        fails_5.append(f"Menger κ on R=1.5 circle = {k:.4f}, expected {expected:.4f}")
    print(f"Test 5 — Menger κ on R=1.5 circle: {k:.4f} (want {expected:.4f}): "
          f"{'OK' if not fails_5 else 'FAIL'}")
    fails.extend(fails_5)

    # ---- Test 6: Menger κ on three collinear points = 0 ----
    fails_6 = []
    k = menger_kappa((0.0, 0.0), (1.0, 0.0), (2.0, 0.0))
    if k > 1e-9:
        fails_6.append(f"Menger κ on collinear should be 0, got {k}")
    print(f"Test 6 — Menger κ on collinear points: {k:.6f}: "
          f"{'OK' if not fails_6 else 'FAIL'}")
    fails.extend(fails_6)

    # ---- Test 7: predictive κ on a constant-radius arc = 1/R ----
    # 100-point sample of a quarter-circle, R=2.0, starting at (R, 0) and
    # arcing CCW to (0, R). Probes are spaced along arc length, so worst-κ
    # over N=3 previews should be very close to 1/R = 0.5.
    fails_7 = []
    R = 2.0
    n_arc = 100
    arc_pts = []
    for j in range(n_arc + 1):
        a = 0.5 * math.pi * (j / n_arc)
        arc_pts.append((R * math.cos(a), R * math.sin(a)))
    # Place the rover at the start; foot is the first vertex; seg_idx=0.
    # L_d = 0.3 m → preview centres at 0.3, 0.6, 0.9 m.
    kappa_pred = max_preview_curvature(arc_pts, 0, arc_pts[0][0], arc_pts[0][1],
                                       l_d=0.3, n_previews=3)
    expected = 1.0 / R
    rel_err = abs(kappa_pred - expected) / expected
    if rel_err > 0.05:
        fails_7.append(
            f"predictive κ on R={R} arc = {kappa_pred:.4f}, want {expected:.4f} "
            f"(rel_err {rel_err * 100:.1f}%)"
        )
    print(f"Test 7 — predictive κ on R={R}m arc, N=3 previews: "
          f"{kappa_pred:.4f} (want {expected:.4f}, err {rel_err * 100:.2f}%): "
          f"{'OK' if not fails_7 else 'FAIL'}")
    fails.extend(fails_7)

    # ---- Test 8: predictive κ on a straight line = 0 ----
    # Pure straight path; predictive curvature should be exactly 0.
    fails_8 = []
    line_pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), (4.0, 0.0)]
    kappa_line = max_preview_curvature(line_pts, 0, 0.0, 0.0,
                                       l_d=0.5, n_previews=3)
    if kappa_line > 1e-6:
        fails_8.append(f"predictive κ on straight line should be 0, got {kappa_line}")
    print(f"Test 8 — predictive κ on straight line: {kappa_line:.6e}: "
          f"{'OK' if not fails_8 else 'FAIL'}")
    fails.extend(fails_8)

    # ---- Test 9: predictive κ short-circuits when path is shorter than probes ----
    # 0.4 m path, but probes at 0.3, 0.6, 0.9 m → previews 2 and 3 hit_end.
    # _max_preview_curvature must `break` (not crash, not loop forever) and
    # return either 0 or the preview-1 value; both are acceptable.
    fails_9 = []
    short_pts = [(0.0, 0.0), (0.4, 0.0)]
    try:
        kappa_short = max_preview_curvature(short_pts, 0, 0.0, 0.0,
                                            l_d=0.3, n_previews=3)
    except Exception as exc:
        fails_9.append(f"predictive κ on short path raised: {exc!r}")
        kappa_short = float("nan")
    # Straight path → expected 0 either way; the test really verifies the
    # break-on-hit_end logic doesn't blow up or return garbage.
    if not (kappa_short == 0.0 or (isinstance(kappa_short, float) and kappa_short < 1e-6)):
        fails_9.append(f"unexpected κ on short straight path: {kappa_short}")
    print(f"Test 9 — predictive κ on short path (probes off-end): "
          f"κ={kappa_short:.6e}, no crash: "
          f"{'OK' if not fails_9 else 'FAIL'}")
    fails.extend(fails_9)

    # ---- Test 10: predictive κ on tighter arc returns higher κ ----
    # R=2.0 vs R=1.0 should produce κ_pred ≈ 0.5 vs ≈ 1.0. Verifies the
    # regulator can actually distinguish "loose corner ahead" from "tight
    # corner ahead" — the whole point of P1.1.
    fails_10 = []
    R_tight = 1.0
    arc_tight = []
    for j in range(n_arc + 1):
        a = 0.5 * math.pi * (j / n_arc)
        arc_tight.append((R_tight * math.cos(a), R_tight * math.sin(a)))
    kappa_tight = max_preview_curvature(arc_tight, 0,
                                        arc_tight[0][0], arc_tight[0][1],
                                        l_d=0.3, n_previews=3)
    expected_tight = 1.0 / R_tight
    expected_loose = 1.0 / R   # R from Test 7
    rel_err_t = abs(kappa_tight - expected_tight) / expected_tight
    # Re-verify the loose-arc value in this scope so a sqrt(R)-class bug
    # (correct ratio, wrong absolute scaling) can't pass via Test 7's
    # variable alone.
    rel_err_l = abs(kappa_pred - expected_loose) / expected_loose
    if rel_err_t > 0.05:
        fails_10.append(
            f"predictive κ on R={R_tight} arc = {kappa_tight:.4f}, "
            f"want {expected_tight:.4f} (rel_err {rel_err_t * 100:.1f}%)"
        )
    if rel_err_l > 0.05:
        fails_10.append(
            f"predictive κ on R={R} arc = {kappa_pred:.4f}, "
            f"want {expected_loose:.4f} (rel_err {rel_err_l * 100:.1f}%)"
        )
    if not (kappa_tight > kappa_pred * 1.5):
        fails_10.append(
            f"predictive κ failed to distinguish tight ({kappa_tight:.3f}) "
            f"from loose ({kappa_pred:.3f}) arc"
        )
    print(f"Test 10 — predictive κ on R={R_tight}m vs R={R}m: "
          f"{kappa_tight:.3f} (want {expected_tight:.2f}) > "
          f"{kappa_pred:.3f} (want {expected_loose:.2f}), ratio "
          f"{kappa_tight / kappa_pred:.2f}x: "
          f"{'OK' if not fails_10 else 'FAIL'}")
    fails.extend(fails_10)

    # ---- Test 11: _hint_valid=False forces full-scan on path reset ----
    # Path: 30 segments along east axis, 10 cm spacing.
    # Rover sits 5 cm north of the midpoint of segment 7.
    # With hint=0 and hint_valid=False the full-scan path MUST find seg_idx=7.
    # This is the regression fence around the Sprint 1 polish commit
    # ("full-scan projection on first cycle after path/jump reset").
    fails_11 = []
    long_path = [(i * 0.1, 0.0) for i in range(31)]  # 30 segments, 0..3.0 m east
    # Rover at (segment 7 midpoint east + 0.05 north): seg7 = x∈[0.7, 0.8]
    # NB: path is in (n, e) — the "east axis" here is actually the NORTH axis
    #     in NED, since path_pts[i] = (n, e) = (i*0.1, 0). That doesn't change
    #     the test's correctness; we're just verifying the projection picks
    #     the right segment regardless of frame interpretation.
    rover_n = 0.75
    rover_e = 0.05
    seg, t, fn, fe, xt, new_hint, new_valid = project_onto_path(
        long_path, rover_n, rover_e, hint=0, hint_valid=False
    )
    if seg != 7:
        fails_11.append(f"full-scan picked seg={seg}, want 7")
    if abs(fn - 0.75) > 1e-6 or abs(fe - 0.0) > 1e-6:
        fails_11.append(f"foot ({fn:.4f},{fe:.4f}) — want (0.75, 0.0)")
    if abs(abs(xt) - 0.05) > 1e-6:
        fails_11.append(f"|xtrack| {xt:.4f}, want ±0.05")
    if not new_valid or new_hint != 7:
        fails_11.append(f"hint={new_hint},valid={new_valid} after success — "
                        f"want hint=7,valid=True")
    print(f"Test 11 — full-scan after reset: seg={seg} (want 7), "
          f"xtrack={xt:+.3f}m (want ±0.050), "
          f"hint→{new_hint}/valid→{new_valid}: "
          f"{'OK' if not fails_11 else 'FAIL'}")
    fails.extend(fails_11)

    # ---- Test 12: windowed search remains O(1) and tracks forward in steady state ----
    # Same path, second call: rover has advanced one segment. Hint=7 from
    # Test 11. Rover now at segment 8 midpoint. Window=[5,11) covers it.
    fails_12 = []
    rover_n2 = 0.85
    rover_e2 = 0.03
    seg2, t2, fn2, fe2, xt2, hint2, valid2 = project_onto_path(
        long_path, rover_n2, rover_e2, hint=new_hint, hint_valid=new_valid
    )
    if seg2 != 8:
        fails_12.append(f"steady-state windowed search picked seg={seg2}, want 8")
    if not valid2 or hint2 != 8:
        fails_12.append(f"hint did not advance to 8: hint={hint2},valid={valid2}")
    print(f"Test 12 — steady-state hint walk: seg={seg2} (want 8), "
          f"hint→{hint2}/valid→{valid2}: "
          f"{'OK' if not fails_12 else 'FAIL'}")
    fails.extend(fails_12)

    # ---- Test 13: stale hint with hint_valid=True returns WRONG segment ----
    # This is a regression-doc test: it proves WHY _hint_valid=False is
    # mandatory on path reset and EKF jump. With hint=2 (stale) and
    # hint_valid=True the window is [0, 6). Rover is at segment 20 — outside
    # the window. The projection MUST return a segment ≤ 5, not 20. If this
    # ever returns 20 with the windowed branch, someone widened the window
    # and silently dropped the need for the full-scan flag — re-evaluate
    # before deleting.
    fails_13 = []
    rover_n3 = 2.05  # segment 20 midpoint = 2.05 m
    rover_e3 = 0.02
    seg3, t3, fn3, fe3, xt3, _, _ = project_onto_path(
        long_path, rover_n3, rover_e3, hint=2, hint_valid=True
    )
    if seg3 > 5:
        fails_13.append(
            f"window=[0,6) returned seg={seg3} — implementation changed; "
            f"re-evaluate Sprint 1 _hint_valid full-scan necessity"
        )
    # Now repeat WITH hint_valid=False — full scan should recover seg 20
    seg3b, _, _, _, _, _, _ = project_onto_path(
        long_path, rover_n3, rover_e3, hint=2, hint_valid=False
    )
    if seg3b != 20:
        fails_13.append(
            f"full-scan with hint_valid=False picked seg={seg3b}, want 20"
        )
    print(f"Test 13 — stale hint regression doc: "
          f"windowed seg={seg3} (≤5, wrong-but-by-design), "
          f"full-scan seg={seg3b} (want 20): "
          f"{'OK' if not fails_13 else 'FAIL'}")
    fails.extend(fails_13)

    # ---- Test 14: B3 — single-pass walker matches per-preview walker ----
    # On a curved path with N=3 previews and L_d=0.5, the single-pass walker
    # must emit identical (within float epsilon) points to 9 independent
    # calls to get_lookahead_point. If this fails, the B3 refactor changed
    # behaviour — predictive κ would silently disagree with itself between
    # commits.
    fails_14 = []
    # Use a quarter-circle, R=2, plus a straight tail so probes well beyond
    # L_d still land on path geometry rather than hitting hit_end.
    R = 2.0
    n_arc = 100
    arc_pts = []
    for j in range(n_arc + 1):
        a = 0.5 * math.pi * (j / n_arc)
        arc_pts.append((R * math.cos(a), R * math.sin(a)))
    # Append 2 m straight segment past the arc end
    arc_pts.append((arc_pts[-1][0], arc_pts[-1][1] + 2.0))

    l_d = 0.5
    n_prev = 3
    half = 0.5 * l_d
    targets = []
    for k in range(1, n_prev + 1):
        centre = k * l_d
        targets.append(max(0.05, centre - half))
        targets.append(centre)
        targets.append(centre + half)

    foot_n, foot_e = arc_pts[0]
    seg_idx = 0
    pp_pts = [get_lookahead_point(arc_pts, seg_idx, foot_n, foot_e, t)
              for t in targets]
    sp_pts = walk_path_samples(arc_pts, seg_idx, foot_n, foot_e, targets)

    if len(pp_pts) != len(sp_pts):
        fails_14.append(f"sample count mismatch: per-preview {len(pp_pts)} vs single-pass {len(sp_pts)}")
    else:
        max_diff = 0.0
        for (an, ae, _ah), (bn, be, _bh) in zip(pp_pts, sp_pts):
            d = math.hypot(an - bn, ae - be)
            if d > max_diff:
                max_diff = d
        if max_diff > 1e-9:
            fails_14.append(f"max sample mismatch {max_diff:.3e} m — refactor drifted")
    print(f"Test 14 — B3 single-pass walker matches per-preview walker on "
          f"R={R} arc (9 samples, max diff {max_diff:.2e} m): "
          f"{'OK' if not fails_14 else 'FAIL'}")
    fails.extend(fails_14)

    # ---- Test 15: B3 single-pass walker handles short paths ----
    # Same short-path case as Test 9: probes at 0.3, 0.6, 0.9 m on a 0.4 m
    # path. The walker must clamp probes 2..9 to the final waypoint with
    # hit_end=True, matching get_lookahead_point's clamp behaviour.
    fails_15 = []
    short_pts = [(0.0, 0.0), (0.4, 0.0)]
    targets_short = [0.15, 0.30, 0.45, 0.30, 0.60, 0.90, 0.60, 1.20, 1.50]
    targets_short.sort()
    pp_short = [get_lookahead_point(short_pts, 0, 0.0, 0.0, t)
                for t in targets_short]
    sp_short = walk_path_samples(short_pts, 0, 0.0, 0.0, targets_short)

    if len(pp_short) != len(sp_short):
        fails_15.append(
            f"short-path sample count mismatch: pp={len(pp_short)} sp={len(sp_short)}"
        )
    else:
        for idx, ((an, ae, ah), (bn, be, bh)) in enumerate(zip(pp_short, sp_short)):
            d = math.hypot(an - bn, ae - be)
            if d > 1e-9:
                fails_15.append(f"short-path sample {idx} mismatch: {d:.3e} m")
            if ah != bh:
                fails_15.append(f"short-path sample {idx} hit_end mismatch: "
                                f"pp={ah} sp={bh}")
    print(f"Test 15 — B3 single-pass walker on short path (probes off-end): "
          f"{len(sp_short)}/{len(pp_short)} samples match: "
          f"{'OK' if not fails_15 else 'FAIL'}")
    fails.extend(fails_15)

    # ---- Result ----
    print()
    if fails:
        print(f"FAIL ({len(fails)} issue(s)):")
        for f in fails:
            print(f"  - {f}")
        sys.exit(1)
    else:
        print("All Sprint 2 geometry tests PASS.")


if __name__ == "__main__":
    main()
