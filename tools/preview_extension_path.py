#!/usr/bin/env python3
"""Render backend-planned paths to PNG for visual verification.

Everything shown comes from the SERVER, not from a local re-implementation of the
planner. That is the whole point: the picture must be of what the rover would
actually drive, produced by the same endpoints the mobile app calls.

Flow (all via the real API):
    1. POST /api/path/upload                  — upload the DXF
    2. POST /api/path/{name}/extensions       — {enabled: false}   -> RAW
       POST /api/path/plan
    3. POST /api/path/{name}/extensions       — {enabled: true, ...} -> EXTENDED
       POST /api/path/plan
    4. Render both panels side-by-side

Note /api/path/plan deliberately IGNORES the enable_path_extensions /
pre_extension_m / aft_extension_m fields on its own request body (deprecated —
models.py:412). Extensions are a per-file sidecar config, so they must be set via
the /extensions endpoint first. Passing them to /plan silently does nothing; that
is exactly the trap this script avoids.

Usage
-----
    # list what's available
    python3 tools/preview_extension_path.py --list

    # one file, both modes (per-line off = connectivity-aware policy)
    python3 tools/preview_extension_path.py --dxf square_2m.DXF

    # per-line mode (each CAD line gets its own PRE/MARK/AFT)
    python3 tools/preview_extension_path.py --dxf square_2m.DXF --per-line

    # every sample DXF, both extension modes
    python3 tools/preview_extension_path.py --all --per-line

    # against a different backend
    python3 tools/preview_extension_path.py --dxf line.DXF --host 192.168.1.102
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

DEFAULT_DXF_DIR = Path("/Users/dyx_a1/Vetri/PX4_DXP/Simple Demo/sample_dxf")
DEFAULT_HOST = "192.168.1.102"
DEFAULT_PORT = 5001
TIMEOUT_S = 60


# ── HTTP (stdlib only — no requests dependency on the Jetson) ─────────────────

class ApiError(RuntimeError):
    pass


def _api(host: str, port: int, method: str, path: str, token: str | None = None,
         body: dict | None = None) -> dict:
    url = f"http://{host}:{port}{path}"
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    if data is not None:
        req.add_header("Content-Type", "application/json")
    if token:
        req.add_header("X-Rover-Token", token)
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT_S) as r:
            return json.loads(r.read() or "{}")
    except urllib.error.HTTPError as e:
        detail = e.read().decode(errors="replace")[:400]
        raise ApiError(f"{method} {path} -> HTTP {e.code}: {detail}") from None
    except urllib.error.URLError as e:
        raise ApiError(f"{method} {path} -> cannot reach {url}: {e.reason}") from None


def _upload(host: str, port: int, dxf: Path, token: str | None) -> str:
    """POST /api/path/upload (multipart/form-data, hand-rolled)."""
    boundary = "----RoverPreviewBoundary7f3a"
    payload = b"".join([
        f"--{boundary}\r\n".encode(),
        f'Content-Disposition: form-data; name="file"; filename="{dxf.name}"\r\n'.encode(),
        b"Content-Type: application/octet-stream\r\n\r\n",
        dxf.read_bytes(),
        f"\r\n--{boundary}--\r\n".encode(),
    ])
    req = urllib.request.Request(
        f"http://{host}:{port}/api/path/upload", data=payload, method="POST")
    req.add_header("Content-Type", f"multipart/form-data; boundary={boundary}")
    if token:
        req.add_header("X-Rover-Token", token)
    try:
        with urllib.request.urlopen(req, timeout=TIMEOUT_S) as r:
            resp = json.loads(r.read() or "{}")
    except urllib.error.HTTPError as e:
        raise ApiError(f"upload {dxf.name} -> HTTP {e.code}: "
                       f"{e.read().decode(errors='replace')[:300]}") from None
    return resp.get("name") or resp.get("filename") or dxf.name


# ── Backend calls ────────────────────────────────────────────────────────────

def plan_with_extensions(host, port, token, name, *, enabled, pre, aft, per_line,
                         spacing, transit_spacing, mark_speed, transit_speed,
                         compensate_spray):
    """Set the per-file extension config, then plan. Returns the plan response."""
    # Several sample DXFs have spaces in the name ("sct 1.5m.DXF"), which are illegal
    # in a request line. The body still carries the raw name.
    quoted = urllib.parse.quote(name, safe="")
    _api(host, port, "POST", f"/api/path/{quoted}/extensions", token, {
        "enabled": enabled,
        "pre_extension_m": pre,
        "aft_extension_m": aft,
        "per_line": per_line,
    })
    return _api(host, port, "POST", "/api/path/plan", token, {
        "source": name,
        "line_spacing": spacing,
        "transit_spacing": transit_spacing,
        "marking_speed": mark_speed,
        "transit_speed": transit_speed,
        "compensate_spray": compensate_spray,
        "optimize": True,
    })


# ── Metrics computed from the BACKEND's own waypoints ─────────────────────────

def analyse(plan: dict) -> dict:
    wp = [tuple(p) for p in plan["merged_waypoints"]]
    fl = plan["spray_flags"]
    gaps = [math.hypot(wp[i + 1][0] - wp[i][0], wp[i + 1][1] - wp[i][1])
            for i in range(len(wp) - 1)]
    nz = [g for g in gaps if g > 1e-9]

    turns = []
    for i in range(1, len(wp) - 1):
        a = math.atan2(wp[i][1] - wp[i - 1][1], wp[i][0] - wp[i - 1][0])
        b = math.atan2(wp[i + 1][1] - wp[i][1], wp[i + 1][0] - wp[i][0])
        if (math.hypot(wp[i][0] - wp[i - 1][0], wp[i][1] - wp[i - 1][1]) < 1e-9
                or math.hypot(wp[i + 1][0] - wp[i][0], wp[i + 1][1] - wp[i][1]) < 1e-9):
            continue  # zero-length step = deliberate spray-boundary duplicate
        d = abs(math.degrees(math.atan2(math.sin(b - a), math.cos(b - a))))
        if d > 30:
            turns.append((i, d, wp[i]))

    marked = sum(gaps[i] for i in range(len(gaps)) if fl[i])
    driven = sum(gaps)

    # Short gaps are only suspicious AWAY from a spray boundary. Latency compensation
    # legitimately inserts a lead-in point 3.5 cm before the mark starts and trims the
    # end 3.5 mm early, and both land inside a naive "2-4 cm" window. Counting those as
    # double-densify artefacts made every single plan — RAW included — look broken.
    half = 0
    for i, g in enumerate(gaps):
        if not (0.02 < g < 0.04):
            continue
        at_boundary = (
            (i > 0 and fl[i - 1] != fl[i])
            or (i + 1 < len(fl) and fl[i] != fl[i + 1])
        )
        if not at_boundary:
            half += 1

    return {
        "waypoints": len(wp),
        "marked_m": marked,
        "driven_m": driven,
        "overhead_pct": (driven / marked - 1) * 100 if marked > 1e-9 else float("nan"),
        "max_gap_cm": max(nz) * 100 if nz else 0.0,
        "over_spacing": sum(1 for g in nz if g > 0.0501),
        "half_gaps": half,  # double-densify artefact, spray-boundary points excluded
        "turns": turns,
        "hard_turns": [t for t in turns if t[1] > 100],
        "spurs": sum(1 for s in plan.get("segments", [])
                     if "extension_join" in str(s.get("source", s.get("source_entity", "")))),
        "bad_spray": sum(1 for i, f in enumerate(fl) if f and _outside_mark(wp[i], plan)),
    }


def _mark_points(plan) -> set:
    pts = set()
    for s in plan.get("segments", []):
        st = str(s.get("type", s.get("segment_type", ""))).lower()
        if st in ("mark", "0", "segmenttype.mark"):
            for p in s.get("points", []):
                pts.add((round(p[0], 4), round(p[1], 4)))
    return pts


_MARK_CACHE: dict[int, set] = {}


def _outside_mark(pt, plan) -> bool:
    key = id(plan)
    if key not in _MARK_CACHE:
        _MARK_CACHE[key] = _mark_points(plan)
    mk = _MARK_CACHE[key]
    if not mk:
        return False
    return (round(pt[0], 4), round(pt[1], 4)) not in mk


# ── Rendering ────────────────────────────────────────────────────────────────

def _draw(ax, plan, title, meta):
    import matplotlib.pyplot as plt  # noqa: F401

    wp = [tuple(p) for p in plan["merged_waypoints"]]
    fl = plan["spray_flags"]

    # East = x (right), North = y (up) — standard map orientation for the operator.
    def xy(p):
        return p[1], p[0]

    # Draw each consecutive pair coloured by whether spray is ON for that step.
    for i in range(len(wp) - 1):
        x0, y0 = xy(wp[i])
        x1, y1 = xy(wp[i + 1])
        if fl[i]:
            ax.plot([x0, x1], [y0, y1], "-", color="#d62728", lw=3.0,
                    solid_capstyle="round", zorder=3)          # MARK / spray ON
        else:
            ax.plot([x0, x1], [y0, y1], "-", color="#7f9fbf", lw=1.4,
                    alpha=0.95, zorder=2)                       # TRANSIT / spray OFF

    xs = [xy(p)[0] for p in wp]
    ys = [xy(p)[1] for p in wp]
    ax.plot(xs, ys, ".", ms=2.0, color="#333333", alpha=0.55, zorder=4)

    # Flag the geometry the rover struggles with.
    for _, ang, p in meta["turns"]:
        x, y = xy(p)
        if ang > 100:
            ax.plot(x, y, "x", ms=11, mew=2.6, color="#e6007a", zorder=6)
        else:
            ax.plot(x, y, "o", ms=5, mfc="none", mec="#ff7f0e", mew=1.5, zorder=5)

    sx, sy = xy(wp[0])
    ex, ey = xy(wp[-1])
    ax.plot(sx, sy, "^", ms=11, color="#2ca02c", zorder=7)
    ax.plot(ex, ey, "s", ms=9, color="#111111", zorder=7)

    ax.set_title(title, fontsize=11, fontweight="bold", pad=8)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, ls=":", alpha=0.35, lw=0.6)
    ax.set_xlabel("East (m)", fontsize=8)
    ax.set_ylabel("North (m)", fontsize=8)
    ax.tick_params(labelsize=7)

    warn = []
    if meta["spurs"]:
        warn.append(f"{meta['spurs']} spur connector(s)")
    if meta["hard_turns"]:
        warn.append(f"{len(meta['hard_turns'])} turns >100°")
    if meta["half_gaps"]:
        warn.append(f"{meta['half_gaps']} double-densified gaps")
    if meta["over_spacing"]:
        warn.append(f"{meta['over_spacing']} gaps >5cm")
    if meta["bad_spray"]:
        warn.append(f"!! {meta['bad_spray']} spray-ON off-MARK")

    txt = (
        f"waypoints   {meta['waypoints']}\n"
        f"marked      {meta['marked_m']:.3f} m\n"
        f"driven      {meta['driven_m']:.3f} m\n"
        f"overhead    {meta['overhead_pct']:+.0f} %\n"
        f"max gap     {meta['max_gap_cm']:.2f} cm"
    )
    ax.text(0.02, 0.98, txt, transform=ax.transAxes, va="top", ha="left",
            family="monospace", fontsize=8,
            bbox=dict(fc="white", ec="#bbbbbb", alpha=0.9, boxstyle="round,pad=0.4"))

    if warn:
        ax.text(0.98, 0.02, "\n".join("⚠ " + w for w in warn), transform=ax.transAxes,
                va="bottom", ha="right", fontsize=8, color="#a11",
                bbox=dict(fc="#fff4f4", ec="#e0a0a0", alpha=0.95,
                          boxstyle="round,pad=0.4"))


def render(name, raw, raw_m, ext, ext_m, ext_label, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    fig, axes = plt.subplots(1, 2, figsize=(15, 7.6))
    _draw(axes[0], raw, f"RAW  —  no extensions", raw_m)
    _draw(axes[1], ext, f"EXTENDED  —  {ext_label}", ext_m)

    legend = [
        Line2D([], [], color="#d62728", lw=3, label="MARK (spray ON)"),
        Line2D([], [], color="#7f9fbf", lw=1.6, label="TRANSIT (spray OFF)"),
        Line2D([], [], color="#2ca02c", marker="^", ls="", ms=9, label="start"),
        Line2D([], [], color="#111111", marker="s", ls="", ms=8, label="end"),
        Line2D([], [], color="#ff7f0e", marker="o", ls="", mfc="none", mew=1.5,
               ms=7, label="turn >30°"),
        Line2D([], [], color="#e6007a", marker="x", ls="", mew=2.4, ms=9,
               label="turn >100° (hard to track)"),
    ]
    fig.legend(handles=legend, loc="lower center", ncol=6, frameon=False, fontsize=9)
    fig.suptitle(f"{name}   —   planned by the backend (/api/path/plan)",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=(0, 0.05, 1, 0.96))
    fig.savefig(out_png, dpi=135)
    plt.close(fig)


# ── CLI ──────────────────────────────────────────────────────────────────────

def _report(label, m):
    print(f"  {label}")
    print(f"    waypoints {m['waypoints']:4d} | marked {m['marked_m']:6.3f} m | "
          f"driven {m['driven_m']:6.3f} m | overhead {m['overhead_pct']:+5.0f}% | "
          f"max gap {m['max_gap_cm']:5.2f} cm")
    flags = []
    if m["spurs"]:
        flags.append(f"{m['spurs']} SPUR connector(s)")
    if m["hard_turns"]:
        angs = ", ".join(f"{a:.0f}°" for _, a, _ in m["hard_turns"][:6])
        flags.append(f"{len(m['hard_turns'])} turns >100° ({angs})")
    if m["half_gaps"]:
        flags.append(f"{m['half_gaps']} double-densified 2.5cm gaps")
    if m["over_spacing"]:
        flags.append(f"{m['over_spacing']} gaps >5cm")
    if m["bad_spray"]:
        flags.append(f"{m['bad_spray']} spray-ON outside MARK")
    for f in flags:
        print(f"    !! {f}")
    if not flags:
        print("    clean")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dxf", help="DXF filename (in --dxf-dir) or a full path")
    ap.add_argument("--all", action="store_true", help="process every DXF in --dxf-dir")
    ap.add_argument("--list", action="store_true", help="list available DXFs and exit")
    ap.add_argument("--dxf-dir", type=Path, default=DEFAULT_DXF_DIR)
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--token", default=None, help="X-Rover-Token, if auth is enabled")
    ap.add_argument("--out", type=Path, default=Path("path_previews"))
    ap.add_argument("--pre", type=float, default=0.5, help="PRE extension (m)")
    ap.add_argument("--aft", type=float, default=0.5, help="AFT extension (m)")
    ap.add_argument("--per-line", action="store_true",
                    help="per-line mode: every CAD line gets its own PRE/MARK/AFT")
    ap.add_argument("--spacing", type=float, default=0.05)
    ap.add_argument("--transit-spacing", type=float, default=0.15)
    ap.add_argument("--mark-speed", type=float, default=0.35)
    ap.add_argument("--transit-speed", type=float, default=0.50)
    ap.add_argument("--no-spray-compensation", action="store_true",
                    help="disable spray latency compensation (cleaner pure geometry)")
    args = ap.parse_args()

    if not args.dxf_dir.is_dir():
        print(f"error: DXF dir not found: {args.dxf_dir}", file=sys.stderr)
        return 2

    found = sorted(p for p in args.dxf_dir.iterdir()
                   if p.suffix.lower() == ".dxf" and p.is_file())
    if args.list or (not args.dxf and not args.all):
        print(f"DXF files in {args.dxf_dir}:\n")
        for p in found:
            print(f"  {p.name}")
        print(f"\n{len(found)} file(s). Use --dxf <name>, or --all.")
        return 0

    if args.all:
        targets = found
    else:
        cand = Path(args.dxf)
        if not cand.is_file():
            cand = args.dxf_dir / args.dxf
        if not cand.is_file():
            print(f"error: no such DXF: {args.dxf}", file=sys.stderr)
            print(f"  available: {', '.join(p.name for p in found)}", file=sys.stderr)
            return 2
        targets = [cand]

    args.out.mkdir(parents=True, exist_ok=True)
    ext_label = ("per-line: every CAD line gets PRE/MARK/AFT" if args.per_line
                 else "connectivity-aware: chain open ends only")
    print(f"backend  : http://{args.host}:{args.port}")
    print(f"extension: pre={args.pre}m aft={args.aft}m  ({ext_label})")
    print(f"output   : {args.out.resolve()}\n")

    rc = 0
    for dxf in targets:
        print(f"── {dxf.name}")
        try:
            name = _upload(args.host, args.port, dxf, args.token)
            common = dict(spacing=args.spacing, transit_spacing=args.transit_spacing,
                          mark_speed=args.mark_speed, transit_speed=args.transit_speed,
                          compensate_spray=not args.no_spray_compensation)
            raw = plan_with_extensions(args.host, args.port, args.token, name,
                                       enabled=False, pre=0.0, aft=0.0,
                                       per_line=False, **common)
            ext = plan_with_extensions(args.host, args.port, args.token, name,
                                       enabled=True, pre=args.pre, aft=args.aft,
                                       per_line=args.per_line, **common)
        except Exception as e:
            # One bad DXF must not abort the sweep — report and carry on.
            print(f"    FAILED: {type(e).__name__}: {e}\n")
            rc = 1
            continue

        raw_m, ext_m = analyse(raw), analyse(ext)
        _report("RAW      ", raw_m)
        _report("EXTENDED ", ext_m)

        stem = dxf.stem.replace(" ", "_")
        suffix = "perline" if args.per_line else "chainends"
        png = args.out / f"{stem}__{suffix}.png"
        render(dxf.name, raw, raw_m, ext, ext_m, ext_label, png)
        print(f"    -> {png}\n")

    return rc


if __name__ == "__main__":
    sys.exit(main())
