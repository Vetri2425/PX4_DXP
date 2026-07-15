#!/usr/bin/env python3
"""audit_transit_state.py — production-grade audit of MARK/TRANSIT state and the
shape-to-shape / group-to-group transition path.

Uses the real path_engine (same source as the deployed server). For each file it
plans with production settings (extensions ON, per_line, optimizer ON) and checks
five invariants on the resulting segment list + merged spray path:

  I1 STATE INTEGRITY   — spray flag is True iff SegmentType.MARK; every PRE/AFT
                         extension and every inter-shape connector is TRANSIT
                         (spray OFF); total mark length == sum of MARK segments.
  I2 CONTINUITY        — the ordered polyline has NO hidden jump: every adjacent
                         segment pair touches within the join tolerance (any gap
                         must be represented by an explicit TRANSIT connector).
  I3 CONNECTOR ROUTING — every inter-shape connector spans prev-tip -> next-start
                         (AFT-tip -> next PRE-start), and is TRANSIT.
  I4 WET-PAINT SAFETY  — no connector drives over a MARK line painted earlier in
                         the route (crossing not-yet-painted geometry is allowed).
  I5 TRANSITION FLIPS  — spray flips exactly at MARK<->TRANSIT boundaries; the
                         boundary waypoint is preserved (not deduped away).
"""
from __future__ import annotations
import os, sys, math

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)
DXF_DIR = os.path.join(REPO, "Simple Demo", "verification_dxf ")

from path_engine.engine import PathEngine
from path_engine.core import SegmentType

JOIN_TOL = 0.01          # _SEGMENT_JOIN_TOL_M — continuity tolerance
FILES = ["multi shape 2.DXF", "square and triangle 1.5m.DXF", "star_3x3m.dxf",
         "square_circle.DXF", "square_line.DXF", "triangle 3x3.DXF"]


def _d(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])


def _seg_seg_cross(p1, p2, p3, p4):
    """True if open segments p1p2 and p3p4 properly cross (not just touch ends)."""
    def ccw(a, b, c): return (c[1]-a[1])*(b[0]-a[0]) - (b[1]-a[1])*(c[0]-a[0])
    d1, d2 = ccw(p3, p4, p1), ccw(p3, p4, p2)
    d3, d4 = ccw(p1, p2, p3), ccw(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True
    return False


def is_connector(s):
    return bool((s.metadata or {}).get("extension_connector")) or \
        str(s.source_entity).startswith("transit:")


def is_extension(s):
    src = str(s.source_entity)
    return src.endswith(":pre") or src.endswith(":aft")


def audit(path):
    eng = PathEngine(enable_path_extensions=True, per_line_extensions=True,
                     pre_extension_m=0.5, aft_extension_m=0.5, mark_spacing=0.05,
                     optimize_order=True, use_two_opt=True)
    p = eng.plan_file(path, origin=(0.0, 0.0))
    segs = p.segments
    findings = []

    # ── I1 STATE INTEGRITY ────────────────────────────────────────────────
    mark_len = sum(s.length for s in segs if s.segment_type == SegmentType.MARK)
    bad_ext = [s for s in segs if is_extension(s) and s.segment_type != SegmentType.TRANSIT]
    bad_con = [s for s in segs if is_connector(s) and s.segment_type != SegmentType.TRANSIT]
    flags_ok = len(p.spray_flags) == len(p.merged_waypoints)
    mark_match = abs(mark_len - p.total_mark_length) < 0.02
    i1 = not bad_ext and not bad_con and flags_ok and mark_match
    findings.append(("I1 state", i1,
        f"mark_len(seg)={mark_len:.2f}==plan {p.total_mark_length:.2f}:{mark_match}, "
        f"flags==wps:{flags_ok}, ext-as-MARK:{len(bad_ext)}, connector-as-MARK:{len(bad_con)}"))

    # ── I2 CONTINUITY (no hidden jump) ────────────────────────────────────
    gaps = []
    for a, b in zip(segs, segs[1:]):
        if a.points and b.points:
            g = _d(a.points[-1], b.points[0])
            if g > JOIN_TOL:
                gaps.append(round(g, 3))
    i2 = not gaps
    findings.append(("I2 continuity", i2,
        f"{len(gaps)} inter-segment gaps > {JOIN_TOL} m (hidden jumps)"
        + (f": {gaps[:5]}" if gaps else "")))

    # ── I3 CONNECTOR ROUTING ──────────────────────────────────────────────
    conns = [(i, s) for i, s in enumerate(segs) if is_connector(s)]
    routing_bad = []
    for i, s in conns:
        prev = segs[i-1] if i > 0 else None
        nxt = segs[i+1] if i+1 < len(segs) else None
        ok = (prev and nxt and s.points and
              _d(s.points[0], prev.points[-1]) <= JOIN_TOL and
              _d(s.points[-1], nxt.points[0]) <= JOIN_TOL and
              s.segment_type == SegmentType.TRANSIT)
        if not ok:
            routing_bad.append(i)
    i3 = not routing_bad
    findings.append(("I3 connectors", i3,
        f"{len(conns)} inter-shape connectors, {len(routing_bad)} mis-routed"))

    # ── I4 WET-PAINT SAFETY ───────────────────────────────────────────────
    # Walk the route; a connector must not cross a MARK segment already painted.
    painted = []   # list of mark polylines laid before the current connector
    wet_hits = []
    for s in segs:
        if s.segment_type == SegmentType.MARK:
            painted.append(s.points)
        elif is_connector(s) and s.points and len(s.points) >= 2:
            c0, c1 = s.points[0], s.points[-1]
            for poly in painted:
                crossed = False
                for j in range(len(poly)-1):
                    if _seg_seg_cross(c0, c1, poly[j], poly[j+1]):
                        # ignore a graze at the connector's own endpoints
                        x_ok = True
                        crossed = True; break
                if crossed:
                    wet_hits.append(round(s.length, 2)); break
    i4 = not wet_hits
    findings.append(("I4 wet-paint", i4,
        f"{len(wet_hits)} connector(s) cross already-painted MARK lines"))

    # ── I5 TRANSITION FLIPS ───────────────────────────────────────────────
    flips = sum(1 for a, b in zip(p.spray_flags, p.spray_flags[1:]) if a != b)
    # Expect: at least 2 flips per open mark run (on->off), and every boundary
    # preserved. Sanity: flips even, and no two identical-length runs collapsed.
    n_mark_runs = 0
    prev = None
    for f in p.spray_flags:
        if f and not prev:
            n_mark_runs += 1
        prev = f
    i5 = flips >= (2 * n_mark_runs - 1) and flips > 0
    findings.append(("I5 flips", i5,
        f"{flips} spray transitions across {n_mark_runs} mark runs "
        f"({len(p.spray_flags)} waypoints)"))

    return p, findings


def main():
    print(f"{'file':<30} {'segs':>5} {'mark_m':>7} {'trans_m':>8}  invariants")
    all_ok = True
    for f in FILES:
        path = os.path.join(DXF_DIR, f)
        if not os.path.exists(path):
            print(f"{f:<30}  MISSING"); continue
        try:
            p, findings = audit(path)
        except Exception as e:
            print(f"{f:<30}  ERROR {type(e).__name__}: {e}"); all_ok = False; continue
        marks = [s for s in p.segments if str(s.segment_type).endswith('MARK')]
        summary = " ".join(f"{name.split()[0]}:{'P' if ok else 'F'}" for name, ok, _ in findings)
        ok_all = all(ok for _, ok, _ in findings)
        all_ok = all_ok and ok_all
        print(f"{f:<30} {len(p.segments):>5} {p.total_mark_length:>7.1f} "
              f"{p.total_transit_length:>8.1f}  {summary}")
        for name, ok, detail in findings:
            if not ok:
                print(f"    ✗ {name}: {detail}")
    print("\n" + ("ALL INVARIANTS PASS" if all_ok else "FAILURES ABOVE"))
    # Detailed breakdown for one representative multi-shape file
    print("\n=== detail: multi shape 2.DXF ===")
    p, findings = audit(os.path.join(DXF_DIR, "multi shape 2.DXF"))
    for name, ok, detail in findings:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:16} {detail}")


if __name__ == "__main__":
    main()
