#!/usr/bin/env python3
"""Animate a backend-planned path: watch the rover drive it and lay paint.

Simulates the run in wall-clock time (MARK at marking_speed, TRANSIT at transit_speed)
and renders an MP4. Paint accumulates only where spray is ON, so what you see at the
end IS what the rover would have painted — including any gap, notch, or overrun.

Everything is driven by the server's own plan; the planner is not re-implemented here.

Usage:
    python3 tools/animate_path_flow.py --dxf star_3x3m.dxf --per-line
    python3 tools/animate_path_flow.py --dxf square_2m.DXF --per-line --speed 4
    python3 tools/animate_path_flow.py --all --per-line          # every sample DXF
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
MARK_SPEED = 0.35
TRANSIT_SPEED = 0.50


def _api(host, port, method, path, body=None, timeout=90):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f"http://{host}:{port}{path}", data=data, method=method)
    if data:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read() or "{}")


def _upload(host, port, dxf: Path) -> str:
    boundary = "----RoverAnimBoundary"
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
        return (json.loads(r.read() or "{}")).get("name") or dxf.name


def fetch_plan(host, port, dxf: Path, per_line, pre, aft, compensate):
    name = _upload(host, port, dxf)
    q = urllib.parse.quote(name, safe="")
    _api(host, port, "POST", f"/api/path/{q}/extensions", {
        "enabled": True, "pre_extension_m": pre,
        "aft_extension_m": aft, "per_line": per_line,
    })
    return _api(host, port, "POST", "/api/path/plan", {
        "source": name, "line_spacing": 0.05, "transit_spacing": 0.15,
        "marking_speed": MARK_SPEED, "transit_speed": TRANSIT_SPEED,
        "compensate_spray": compensate, "optimize": True,
    })


def resample_in_time(wp, fl, dt):
    """Walk the polyline at the rover's real speed, sampling every `dt` seconds.

    Returns per-frame (position, spray_on, elapsed_s, distance_m, pass_no). Frame count
    is therefore proportional to how long the run actually takes — a slow marking pass
    occupies more frames than a fast transit, exactly as it would in the field.
    """
    frames = []
    t = 0.0
    dist = 0.0
    pass_no = 0
    prev_spray = False

    for i in range(len(wp) - 1):
        a, b = wp[i], wp[i + 1]
        seg_len = math.dist(a, b)
        spray = fl[i]
        if spray and not prev_spray:
            pass_no += 1
        prev_spray = spray
        if seg_len < 1e-9:
            continue
        speed = MARK_SPEED if spray else TRANSIT_SPEED
        seg_t = seg_len / speed
        n = max(1, int(math.ceil(seg_t / dt)))
        for k in range(n):
            f = k / n
            pos = (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
            frames.append((pos, spray, t + f * seg_t, dist + f * seg_len, pass_no))
        t += seg_t
        dist += seg_len

    frames.append((wp[-1], False, t, dist, pass_no))
    return frames


def animate(dxf: Path, plan, out: Path, speed_x: float, fps: int, per_line: bool):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, FFMpegWriter
    from matplotlib.lines import Line2D

    wp = [tuple(p) for p in plan["merged_waypoints"]]
    fl = plan["spray_flags"]

    dt = speed_x / fps                      # sim seconds per rendered frame
    frames = resample_in_time(wp, fl, dt)
    total_t = frames[-1][2]
    total_d = frames[-1][3]
    n_passes = frames[-1][4]

    xs = [p[1] for p in wp]
    ys = [p[0] for p in wp]
    pad = 0.35
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad

    fig, ax = plt.subplots(figsize=(9, 9.6))
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.grid(True, ls=":", alpha=0.3, lw=0.6)
    ax.set_xlabel("East (m)", fontsize=9)
    ax.set_ylabel("North (m)", fontsize=9)
    ax.tick_params(labelsize=8)

    # Ghost of the full plan, so you can see where the rover is headed.
    ax.plot(xs, ys, "-", color="#e4e8ee", lw=1.0, zorder=1)

    paint_segs = []   # accumulated painted line segments (what ends up on the ground)
    travel_segs = []

    paint_lc = ax.add_collection(
        __import__("matplotlib.collections", fromlist=["LineCollection"])
        .LineCollection([], colors="#d62728", linewidths=4.0, zorder=3,
                        capstyle="round"))
    travel_lc = ax.add_collection(
        __import__("matplotlib.collections", fromlist=["LineCollection"])
        .LineCollection([], colors="#9db4d0", linewidths=1.2, zorder=2, alpha=0.85))

    rover, = ax.plot([], [], "o", ms=13, mfc="#ffffff", mec="#111111", mew=2.0, zorder=6)
    nozzle, = ax.plot([], [], "o", ms=7, zorder=7)
    heading = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                          arrowprops=dict(arrowstyle="-|>", color="#111111", lw=1.8),
                          zorder=6)

    hud = ax.text(0.015, 0.985, "", transform=ax.transAxes, va="top", ha="left",
                  family="monospace", fontsize=9.5,
                  bbox=dict(fc="white", ec="#c8c8c8", alpha=0.94,
                            boxstyle="round,pad=0.5"), zorder=10)
    banner = ax.text(0.5, 0.015, "", transform=ax.transAxes, va="bottom", ha="center",
                     fontsize=11, fontweight="bold", zorder=10)

    mode = "per-line: every CAD line gets PRE/MARK/AFT" if per_line \
        else "chain-ends: continuous run, corners sprayed through"
    ax.set_title(f"{dxf.name}  —  {mode}\n"
                 f"{n_passes} passes · marked {plan['mark_length_m']:.2f} m · "
                 f"driven {total_d:.2f} m · {speed_x:.0f}x real time",
                 fontsize=11, fontweight="bold")
    ax.legend(handles=[
        Line2D([], [], color="#d62728", lw=4, label="paint on the ground"),
        Line2D([], [], color="#9db4d0", lw=1.6, label="travel (spray OFF)"),
    ], loc="upper right", fontsize=8, framealpha=0.95)

    def init():
        paint_lc.set_segments([])
        travel_lc.set_segments([])
        rover.set_data([], [])
        nozzle.set_data([], [])
        return paint_lc, travel_lc, rover, nozzle, hud, banner

    state = {"prev": None}

    def update(i):
        pos, spray, t, d, pno = frames[i]
        prev = state["prev"]
        if prev is not None:
            seg = [(prev[0][1], prev[0][0]), (pos[1], pos[0])]
            # attribute the step to the spray state it was driven under
            (paint_segs if prev[1] else travel_segs).append(seg)
            paint_lc.set_segments(paint_segs)
            travel_lc.set_segments(travel_segs)
            hx, hy = pos[1] - prev[0][1], pos[0] - prev[0][0]
            if math.hypot(hx, hy) > 1e-9:
                k = 0.22 / math.hypot(hx, hy)
                heading.set_position((pos[1], pos[0]))
                heading.xy = (pos[1] + hx * k, pos[0] + hy * k)
        state["prev"] = (pos, spray)

        rover.set_data([pos[1]], [pos[0]])
        nozzle.set_data([pos[1]], [pos[0]])
        nozzle.set_color("#d62728" if spray else "#b8c6d6")

        hud.set_text(
            f"t      {t:6.1f} s\n"
            f"pass   {pno:>3} / {n_passes}\n"
            f"spray  {'ON ' if spray else 'off'}\n"
            f"driven {d:6.2f} m"
        )
        banner.set_text("● SPRAYING" if spray else "travelling")
        banner.set_color("#d62728" if spray else "#7f8c9b")
        return paint_lc, travel_lc, rover, nozzle, hud, banner, heading

    anim = FuncAnimation(fig, update, frames=len(frames), init_func=init,
                         blit=False, interval=1000 / fps)
    out.parent.mkdir(parents=True, exist_ok=True)
    writer = FFMpegWriter(fps=fps, bitrate=2600,
                          metadata={"title": dxf.name, "artist": "PX4_DXP"})
    anim.save(str(out), writer=writer, dpi=110)
    plt.close(fig)
    return len(frames), total_t, total_d, n_passes


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dxf")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--dxf-dir", type=Path, default=DEFAULT_DXF_DIR)
    ap.add_argument("--host", default="192.168.1.102")
    ap.add_argument("--port", type=int, default=5001)
    ap.add_argument("--per-line", action="store_true")
    ap.add_argument("--pre", type=float, default=0.5)
    ap.add_argument("--aft", type=float, default=0.5)
    ap.add_argument("--no-spray-compensation", action="store_true")
    ap.add_argument("--speed", type=float, default=6.0,
                    help="playback speed multiple of real time (default 6x)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--out", type=Path, default=Path("path_videos"))
    args = ap.parse_args()

    if args.all:
        targets = sorted(p for p in args.dxf_dir.iterdir()
                         if p.suffix.lower() == ".dxf" and p.is_file())
    elif args.dxf:
        cand = Path(args.dxf)
        if not cand.is_file():
            cand = args.dxf_dir / args.dxf
        if not cand.is_file():
            print(f"no such DXF: {args.dxf}", file=sys.stderr)
            return 2
        targets = [cand]
    else:
        print("pass --dxf <name> or --all", file=sys.stderr)
        return 2

    suffix = "perline" if args.per_line else "chainends"
    rc = 0
    for dxf in targets:
        print(f"── {dxf.name}")
        try:
            plan = fetch_plan(args.host, args.port, dxf, args.per_line,
                              args.pre, args.aft, not args.no_spray_compensation)
            out = args.out / f"{dxf.stem.replace(' ', '_')}__{suffix}.mp4"
            n, t, d, p = animate(dxf, plan, out, args.speed, args.fps, args.per_line)
            print(f"    {p} passes · {d:.2f} m · {t / 60:.1f} min real "
                  f"→ {n} frames @ {args.speed:.0f}x = {n / args.fps:.0f}s video")
            print(f"    -> {out}")
        except Exception as e:
            print(f"    FAILED: {type(e).__name__}: {e}")
            rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
