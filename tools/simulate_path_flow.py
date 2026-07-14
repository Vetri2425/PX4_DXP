#!/usr/bin/env python3
"""Simulate a planned path start-to-end and render the execution order.

Answers "what exactly does the rover do, in order?" for a DXF planned by the backend.
Every number comes from the server's own plan — this does not re-implement the planner.

Produces:
  * a step table: each pass in execution order, with spray state, length, and the
    pivot the rover must make to enter it
  * a flow PNG: the path drawn in execution order, passes numbered, spray-ON in red,
    travel in grey, with the run-up/run-out of each pass marked

Usage:
    python3 tools/simulate_path_flow.py --dxf star_3x3m.dxf --per-line
    python3 tools/simulate_path_flow.py --dxf square_2m.DXF --per-line --no-spray-compensation
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import urllib.parse
import urllib.request
from pathlib import Path

DEFAULT_DXF_DIR = Path("/Users/dyx_a1/Vetri/PX4_DXP/Simple Demo/sample_dxf")


def _api(host, port, method, path, body=None, timeout=90):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f"http://{host}:{port}{path}", data=data, method=method)
    if data:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read() or "{}")


def _upload(host, port, dxf: Path) -> str:
    boundary = "----RoverSimBoundary"
    payload = b"".join([
        f"--{boundary}\r\n".encode(),
        f'Content-Disposition: form-data; name="file"; filename="{dxf.name}"\r\n'.encode(),
        b"Content-Type: application/octet-stream\r\n\r\n",
        dxf.read_bytes(),
        f"\r\n--{boundary}--\r\n".encode(),
    ])
    req = urllib.request.Request(f"http://{host}:{port}/api/path/upload",
                                 data=payload, method="POST")
    req.add_header("Content-Type", f"multipart/form-data; boundary={boundary}")
    with urllib.request.urlopen(req, timeout=90) as r:
        resp = json.loads(r.read() or "{}")
    return resp.get("name") or dxf.name


def _turn_deg(a, b, c) -> float | None:
    """Heading change at b, going a -> b -> c. None if a step is degenerate."""
    if math.dist(a, b) < 1e-9 or math.dist(b, c) < 1e-9:
        return None
    h1 = math.atan2(b[1] - a[1], b[0] - a[0])
    h2 = math.atan2(c[1] - b[1], c[0] - b[0])
    return abs(math.degrees(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dxf", required=True)
    ap.add_argument("--dxf-dir", type=Path, default=DEFAULT_DXF_DIR)
    ap.add_argument("--host", default="192.168.1.102")
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--per-line", action="store_true")
    ap.add_argument("--pre", type=float, default=0.5)
    ap.add_argument("--aft", type=float, default=0.5)
    ap.add_argument("--no-spray-compensation", action="store_true")
    ap.add_argument("--out", type=Path, default=Path("path_previews"))
    args = ap.parse_args()

    dxf = Path(args.dxf)
    if not dxf.is_file():
        dxf = args.dxf_dir / args.dxf
    if not dxf.is_file():
        print(f"no such DXF: {args.dxf}", file=sys.stderr)
        return 2

    name = _upload(args.host, args.port, dxf)
    q = urllib.parse.quote(name, safe="")
    _api(args.host, args.port, "POST", f"/api/path/{q}/extensions", {
        "enabled": True, "pre_extension_m": args.pre,
        "aft_extension_m": args.aft, "per_line": args.per_line,
    })
    plan = _api(args.host, args.port, "POST", "/api/path/plan", {
        "source": name, "line_spacing": 0.05, "transit_spacing": 0.15,
        "marking_speed": 0.35, "transit_speed": 0.50,
        "compensate_spray": not args.no_spray_compensation, "optimize": True,
    })

    wp = [tuple(p) for p in plan["merged_waypoints"]]
    fl = plan["spray_flags"]
    segs = plan["segments"]

    # ── Walk the merged path and slice it into the segments, in execution order.
    # The plan's `segments` carry metadata only, so re-derive each one's waypoint
    # span by walking the flat polyline and matching cumulative length.
    bounds = []
    i = 0
    for s in segs:
        target = s["length_m"]
        start = i
        run = 0.0
        while i < len(wp) - 1 and run < target - 1e-6:
            run += math.dist(wp[i], wp[i + 1])
            i += 1
        bounds.append((start, i))

    # ── Step table ───────────────────────────────────────────────────────────
    print(f"\n{'='*88}")
    print(f"  SIMULATED RUN — {dxf.name}   (per_line={args.per_line}, "
          f"pre={args.pre} m, aft={args.aft} m)")
    print(f"{'='*88}\n")

    mark_speed, transit_speed = 0.35, 0.50
    t = 0.0
    pass_no = 0
    total_pivot = 0.0
    rows = []

    for k, (s, (a, b)) in enumerate(zip(segs, bounds)):
        role = s.get("segment_role") or "-"
        is_mark = s["type"] == "MARK"
        length = s["length_m"]
        speed = mark_speed if is_mark else transit_speed
        dt = length / speed if speed > 0 else 0.0

        # Pivot required to enter this segment.
        pivot = None
        if a > 0 and a < len(wp) - 1:
            pivot = _turn_deg(wp[a - 1], wp[a], wp[a + 1])

        if is_mark:
            pass_no += 1
        if pivot:
            total_pivot += pivot

        rows.append({
            "seg": k, "role": role, "is_mark": is_mark, "len": length,
            "t0": t, "dt": dt, "pivot": pivot, "src": s["source"],
            "a": a, "b": b, "pass": pass_no if is_mark else None,
        })
        t += dt

    print(f"{'t(s)':>7} {'step':>4} {'what':22} {'len':>6} {'spray':5} {'pivot in':>9}  source")
    print("-" * 88)
    for r in rows:
        what = {
            "pre_transit": "  run-up  (PRE)",
            "aft_transit": "  run-out (AFT)",
            "transit": "  travel",
        }.get(r["role"], f"MARK pass #{r['pass']}" if r["is_mark"] else r["role"])
        piv = f"{r['pivot']:6.0f}°" if r["pivot"] and r["pivot"] > 15 else "      -"
        flag = " <<<" if r["pivot"] and r["pivot"] > 170 else ""
        print(f"{r['t0']:7.1f} {r['seg']:4d} {what:22} {r['len']:6.3f} "
              f"{'ON ' if r['is_mark'] else 'off':5} {piv:>9}{flag}  {str(r['src'])[:24]}")

    mark_len = plan["mark_length_m"]
    transit_len = plan["transit_length_m"]
    print("-" * 88)
    print(f"\n  passes (lines marked) : {pass_no}")
    print(f"  marked                : {mark_len:7.3f} m   ({mark_len / (mark_len + transit_len) * 100:.0f}% of travel)")
    print(f"  transit (spray off)   : {transit_len:7.3f} m")
    print(f"  total driven          : {mark_len + transit_len:7.3f} m")
    print(f"  overhead              : {transit_len / mark_len * 100:+7.0f}%")
    print(f"  est. run time         : {t / 60:7.1f} min  (at 0.35 / 0.50 m/s, pivots not counted)")
    print(f"  cumulative pivot      : {total_pivot:7.0f}°")
    revs = sum(1 for r in rows if r["pivot"] and r["pivot"] > 170)
    print(f"  near-reversals (>170°): {revs}   {'<<< RETRACING' if revs else '(none — good)'}")

    # ── Extension-vs-line sanity: a run-up longer than the line it serves ────
    print(f"\n  {'-'*40}")
    print("  EXTENSION PROPORTION (run-up+run-out vs the line they serve)")
    marks = [r for r in rows if r["is_mark"]]
    worst = []
    for r in marks:
        ext = 0.0
        for nb in rows:
            if nb["role"] in ("pre_transit", "aft_transit") and \
               str(nb["src"]).startswith(str(r["src"])):
                ext += nb["len"]
        ratio = ext / r["len"] if r["len"] > 1e-9 else float("inf")
        worst.append((ratio, r["len"], ext, r["src"]))
    worst.sort(reverse=True)
    for ratio, ln, ext, src in worst[:5]:
        warn = "  <<< extension LONGER than the line" if ratio > 1.0 else ""
        print(f"    line {ln:6.3f} m  + {ext:5.3f} m extension  = {ratio:5.1f}x{warn}  {str(src)[:28]}")

    # ── Flow PNG ─────────────────────────────────────────────────────────────
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    fig, ax = plt.subplots(figsize=(12, 12))

    def xy(p):
        return p[1], p[0]  # east=x, north=y

    for i in range(len(wp) - 1):
        x0, y0 = xy(wp[i])
        x1, y1 = xy(wp[i + 1])
        if fl[i]:
            ax.plot([x0, x1], [y0, y1], "-", color="#d62728", lw=3.4,
                    solid_capstyle="round", zorder=3)
        else:
            ax.plot([x0, x1], [y0, y1], "-", color="#8fa8c8", lw=1.2, alpha=0.9, zorder=2)

    # Number each marked pass at its midpoint, and show its direction.
    for r in rows:
        if not r["is_mark"]:
            continue
        a, b = r["a"], r["b"]
        mid = wp[(a + b) // 2]
        mx, my = xy(mid)
        ax.plot(mx, my, "o", ms=15, color="white", mec="#d62728", mew=1.8, zorder=8)
        ax.text(mx, my, str(r["pass"]), ha="center", va="center",
                fontsize=7.5, fontweight="bold", color="#8b0000", zorder=9)
        # direction arrow
        if b - a > 2:
            p0, p1 = wp[a], wp[min(a + 2, b)]
            ax.annotate("", xy=xy(p1), xytext=xy(p0),
                        arrowprops=dict(arrowstyle="-|>", color="#8b0000",
                                        lw=1.6, shrinkA=0, shrinkB=0), zorder=7)

    for r in rows:
        if r["role"] == "pre_transit":
            ax.plot(*xy(wp[r["a"]]), "^", ms=6, color="#2ca02c", zorder=6)
        elif r["role"] == "aft_transit":
            ax.plot(*xy(wp[min(r["b"], len(wp) - 1)]), "v", ms=6, color="#9467bd", zorder=6)
        if r["pivot"] and r["pivot"] > 100:
            ax.plot(*xy(wp[r["a"]]), "x", ms=9, mew=2.0, color="#e6007a", zorder=7)

    ax.plot(*xy(wp[0]), "^", ms=16, color="#2ca02c", zorder=10)
    ax.text(*xy(wp[0]), "  START", fontsize=9, fontweight="bold", va="center", zorder=10)
    ax.plot(*xy(wp[-1]), "s", ms=13, color="#111111", zorder=10)
    ax.text(*xy(wp[-1]), "  END", fontsize=9, fontweight="bold", va="center", zorder=10)

    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, ls=":", alpha=0.35, lw=0.6)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(f"{dxf.name} — execution flow, {pass_no} marked passes\n"
                 f"marked {mark_len:.2f} m · driven {mark_len + transit_len:.2f} m "
                 f"· overhead {transit_len / mark_len * 100:+.0f}% · "
                 f"{revs} reversals",
                 fontsize=12, fontweight="bold")
    ax.legend(handles=[
        Line2D([], [], color="#d62728", lw=3.4, label="MARK — spray ON"),
        Line2D([], [], color="#8fa8c8", lw=1.4, label="travel — spray OFF"),
        Line2D([], [], color="#2ca02c", marker="^", ls="", label="run-up (PRE) start"),
        Line2D([], [], color="#9467bd", marker="v", ls="", label="run-out (AFT) end"),
        Line2D([], [], color="#e6007a", marker="x", ls="", mew=2, label="pivot >100°"),
    ], loc="upper right", fontsize=8, framealpha=0.95)

    args.out.mkdir(parents=True, exist_ok=True)
    png = args.out / f"{dxf.stem.replace(' ', '_')}__flow.png"
    fig.tight_layout()
    fig.savefig(png, dpi=140)
    print(f"\n  -> {png}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
