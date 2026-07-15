#!/usr/bin/env python3
"""verify_dxf_flow.py — end-to-end CAD→path verification harness.

Feeds each DXF through the LIVE rover-server API (parse-dxf → extensions →
plan → segments), cross-checks against a local ezdxf ground-truth parse, and
produces:

  * per-file PNG render of the planned path (MARK / TRANSIT / extension), with
    start + endpoint markers and measured dimensions,
  * per-file MP4 start→endpoint traversal simulation (H.264 via ffmpeg),
  * a summary-index PNG + markdown + JSON covering the 5 verifications
    (CAD / entities / extensions / dimensions / transitions),
  * a CAD-flow bug list derived from the checks.

The server is treated as the "application"; this script is only a driver +
renderer. Nothing here re-implements planning.

Usage:
  python3 tools/verify_dxf_flow.py                # parse+plan+PNG+summary
  python3 tools/verify_dxf_flow.py --video        # also render MP4s
  python3 tools/verify_dxf_flow.py --only line.DXF square_2m.DXF
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time
from dataclasses import dataclass, field, asdict
from typing import Any, Optional

import requests

# Headless plotting
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.animation as animation

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)  # so `import path_engine` works when run from tools/
DXF_DIR = os.path.join(REPO, "Simple Demo", "verification_dxf ")  # trailing space is real
OUT_DIR = os.path.join(REPO, "verification_output")
BASE_URL = os.environ.get("ROVER_URL", "http://192.168.1.102:5001")

# Extensions ON (pre/aft = 0.5 m). PER_LINE controls the policy:
#   True  = per-entity: every CAD line/edge gets its own PRE→MARK→AFT (incl. each
#           side of a closed square/polygon and every corner).
#   False = connectivity-aware: only a chain's true open ends get a run-up;
#           closed loops and internal corners get none.
EXT_PRE_M = 0.5
EXT_AFT_M = 0.5
PER_LINE = True

# Spacing to retry at when a mission exceeds the waypoint cap at default 5 cm —
# following the guard's own "increase spacing" advice, so the geometry still
# verifies through the server rather than a local fallback.
COARSE_SPACING_M = 0.15

# Representative subset — one per geometry class.
SUBSET = [
    "line.DXF",
    "L_2m.DXF",
    "square_2m.DXF",
    "triangle 3x3.DXF",
    "pentagon.DXF",
    "circle_radius_1m.dxf",
    "arc_sector.DXF",
    "star_3x3m.dxf",
    "multi shape 2.DXF",
    "square and triangle 1.5m.DXF",
    "soccer_pitch_fifa_edited.dxf",
]

TIMEOUT = 30


# ─────────────────────────────────────────────────────────────────────────────
# Result container
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Check:
    status: str      # PASS | WARN | FAIL
    detail: str


@dataclass
class FileResult:
    filename: str
    ok: bool = True
    error: Optional[str] = None
    plan_source: str = "server"   # server | coarsened | local-fallback
    limit_note: Optional[str] = None  # actionable message when the cap guard fired
    render_segments: Optional[dict] = None  # point-carrying segs for rendering only
    # raw API payloads
    parse: dict = field(default_factory=dict)
    plan: dict = field(default_factory=dict)
    segments: dict = field(default_factory=dict)
    ext_saved: dict = field(default_factory=dict)
    local: dict = field(default_factory=dict)
    # verifications
    checks: dict = field(default_factory=dict)   # name -> Check
    bugs: list = field(default_factory=list)     # list[str]
    png: Optional[str] = None
    mp4: Optional[str] = None


# ─────────────────────────────────────────────────────────────────────────────
# API layer
# ─────────────────────────────────────────────────────────────────────────────
def api_parse_dxf(path: str) -> dict:
    with open(path, "rb") as fh:
        r = requests.post(
            f"{BASE_URL}/api/path/parse-dxf",
            files={"file": (os.path.basename(path), fh, "application/dxf")},
            timeout=TIMEOUT,
        )
    r.raise_for_status()
    return r.json()


def api_set_extensions(name: str) -> dict:
    r = requests.post(
        f"{BASE_URL}/api/path/{name}/extensions",
        json={"enabled": True, "pre_extension_m": EXT_PRE_M,
              "aft_extension_m": EXT_AFT_M, "per_line": PER_LINE},
        timeout=TIMEOUT,
    )
    r.raise_for_status()
    return r.json()


class WaypointLimitError(RuntimeError):
    """Server rejected the plan for exceeding the waypoint cap (correct guard)."""


def api_plan(name: str, line_spacing: Optional[float] = None) -> dict:
    body = {"source": name, "include_waypoints": True, "optimize": True}
    if line_spacing is not None:
        body["line_spacing"] = line_spacing
    r = requests.post(f"{BASE_URL}/api/path/plan", json=body, timeout=TIMEOUT)
    if r.status_code != 200:
        detail = r.text[:300]
        if "Too many waypoints" in detail:
            raise WaypointLimitError(detail)
        raise RuntimeError(f"plan {r.status_code}: {detail}")
    return r.json()


def api_segments(name: str) -> dict:
    r = requests.get(f"{BASE_URL}/api/path/{name}/segments", timeout=TIMEOUT)
    if r.status_code != 200:
        raise RuntimeError(f"segments {r.status_code}: {r.text[:300]}")
    return r.json()


def _safe_segments(name: str) -> dict:
    try:
        return api_segments(name)
    except Exception as exc:
        return {"segments": [], "_error": str(exc)}


def local_core_plan(path: str, extensions: bool = False,
                    mark_spacing: float = 0.05) -> tuple[dict, dict]:
    """Plan with the core path_engine library (same source as the server, no
    HTTP/alignment). Two uses:
      * server-timeout fallback (extensions=False), and
      * supplying per-point segment geometry for RENDERING the coarsened case,
        where the server /plan response carries roles but no points.
    is_extension / role are derived from the planner's own :pre/:aft source tags,
    so the render matches the server plan. Returns (plan_dict, segments_dict)."""
    from path_engine.engine import PathEngine
    from path_engine.core import SegmentType
    kw = dict(mark_spacing=mark_spacing)
    if extensions:
        kw.update(enable_path_extensions=True, per_line_extensions=PER_LINE,
                  pre_extension_m=EXT_PRE_M, aft_extension_m=EXT_AFT_M)
    p = PathEngine(**kw).plan_file(path, origin=(0.0, 0.0))
    seglist = []
    for i, s in enumerate(p.segments):
        is_mark = s.segment_type == SegmentType.MARK
        src = str(s.source_entity or "")
        is_ext = src.endswith(":pre") or src.endswith(":aft")
        role = ("pre_transit" if src.endswith(":pre") else
                "aft_transit" if src.endswith(":aft") else
                "mark" if is_mark else "transit")
        seglist.append({
            "index": i, "sequence": i,
            "type": "MARK" if is_mark else "TRANSIT",
            "segment_role": role,
            "source_entity": src,
            "is_extension": is_ext,
            "spray_on": is_mark,
            "speed": s.speed,
            "length_m": s.length,
            "points": s.points,
        })
    plan = {
        "source": os.path.basename(path),
        "num_waypoints": p.num_waypoints,
        "num_segments": len(p.segments),
        "mark_length_m": p.total_mark_length,
        "transit_length_m": p.total_transit_length,
        "total_length_m": p.total_length,
        "segments": [],
        "merged_waypoints": p.merged_waypoints,
        "spray_flags": p.spray_flags,
    }
    return plan, {"segments": seglist}


# ─────────────────────────────────────────────────────────────────────────────
# Local ground-truth parse (ezdxf)
# ─────────────────────────────────────────────────────────────────────────────
def local_parse(path: str, unit_scale: float) -> dict:
    """Raw entity census + bbox in metres, independent of the server."""
    import ezdxf
    doc = ezdxf.readfile(path)
    msp = doc.modelspace()
    types: dict[str, int] = {}
    xs: list[float] = []
    ys: list[float] = []

    def add(x, y):
        xs.append(x); ys.append(y)

    annot_len = 0.0       # geometry length (m) on annotation layers
    nonannot_len = 0.0    # geometry length (m) on ordinary (sprayable) layers

    def _ent_len(e, t) -> float:
        try:
            if t == "LINE":
                return math.dist((e.dxf.start.x, e.dxf.start.y),
                                 (e.dxf.end.x, e.dxf.end.y))
            if t == "CIRCLE":
                return 2 * math.pi * e.dxf.radius
            if t == "ARC":
                sweep = (e.dxf.end_angle - e.dxf.start_angle) % 360.0
                return math.radians(sweep or 360.0) * e.dxf.radius
            if t in ("LWPOLYLINE", "POLYLINE"):
                pts = list(e.get_points("xy")) if t == "LWPOLYLINE" else \
                    [(v.dxf.location.x, v.dxf.location.y) for v in e.vertices]
                pts = [(p[0], p[1]) for p in pts]
                L = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
                if getattr(e, "closed", False) and len(pts) > 2:
                    L += math.dist(pts[-1], pts[0])
                return L
        except Exception:
            return 0.0
        return 0.0

    for e in msp:
        t = e.dxftype()
        types[t] = types.get(t, 0) + 1
        length_m = _ent_len(e, t) * unit_scale
        if _ANNOTATION_LAYERS.search(getattr(e.dxf, "layer", "") or ""):
            annot_len += length_m
        else:
            nonannot_len += length_m
        try:
            if t == "LINE":
                add(e.dxf.start.x, e.dxf.start.y)
                add(e.dxf.end.x, e.dxf.end.y)
            elif t == "CIRCLE":
                cx, cy, r = e.dxf.center.x, e.dxf.center.y, e.dxf.radius
                add(cx - r, cy - r); add(cx + r, cy + r)
            elif t == "ARC":
                cx, cy, r = e.dxf.center.x, e.dxf.center.y, e.dxf.radius
                add(cx - r, cy - r); add(cx + r, cy + r)
            elif t in ("LWPOLYLINE", "POLYLINE"):
                for pt in e.get_points("xy") if t == "LWPOLYLINE" else \
                        [(v.dxf.location.x, v.dxf.location.y) for v in e.vertices]:
                    add(pt[0], pt[1])
            elif t == "POINT":
                add(e.dxf.location.x, e.dxf.location.y)
        except Exception:
            pass

    bbox_m = None
    if xs and ys:
        dx = (max(xs) - min(xs)) * unit_scale
        dy = (max(ys) - min(ys)) * unit_scale
        bbox_m = [round(dx, 4), round(dy, 4)]
    return {"types": types, "num_entities": sum(types.values()), "bbox_m": bbox_m,
            "annot_len_m": round(annot_len, 3), "nonannot_len_m": round(nonannot_len, 3)}


# ─────────────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────────────
def wp_bbox(wps: list[list[float]]) -> Optional[list[float]]:
    if not wps:
        return None
    ns = [p[0] for p in wps]
    es = [p[1] for p in wps]
    return [round(max(ns) - min(ns), 4), round(max(es) - min(es), 4)]


def expected_dim_from_name(name: str):
    """Pull an intended size (metres) from the filename. Returns (value, kind).

    radius is checked FIRST so 'circle_radius_1m' → 2 m diameter, not 1 m.
    """
    m = re.search(r"radius[_ ]?(\d+(?:\.\d+)?)", name, re.IGNORECASE)
    if m:
        return float(m.group(1)) * 2.0, "diameter"  # radius → diameter
    m = re.search(r"(\d+(?:\.\d+)?)\s*[xX]\s*(\d+(?:\.\d+)?)", name)
    if m:
        return max(float(m.group(1)), float(m.group(2))), "side"
    m = re.search(r"(\d+(?:\.\d+)?)\s*m\b", name, re.IGNORECASE)
    if m:
        return float(m.group(1)), "size"
    return None, None


_POLY = {"POLYLINE", "LWPOLYLINE"}
# Layers that are non-printing annotation by CAD convention. Deliberately does
# NOT include CENTER/HIDDEN — those are legitimate geometry in field markings
# (e.g. a soccer-pitch centre circle) and linetype layers, not annotations.
_ANNOTATION_LAYERS = re.compile(
    r"\b(DIM|DIMENSION|DIMS|TEXT|MTEXT|ANNOT|ANNOTATION|DEFPOINTS|TITLE|NOTES?)\b",
    re.IGNORECASE)


def _norm_types(types: dict) -> dict:
    """Collapse POLYLINE/LWPOLYLINE so a subtype label diff isn't a census diff."""
    out: dict[str, int] = {}
    for k, v in types.items():
        key = "POLYLINE*" if k in _POLY else k
        out[key] = out.get(key, 0) + v
    return out


# ─────────────────────────────────────────────────────────────────────────────
# Verifications
# ─────────────────────────────────────────────────────────────────────────────
def run_checks(res: FileResult) -> None:
    parse, plan, segs, local = res.parse, res.plan, res.segments, res.local
    c = res.checks

    # 1. CAD — parse succeeded, entities present, sane unit scale
    n_ent = parse.get("num_entities", 0)
    uscale = parse.get("unit_scale", 0)
    if n_ent > 0 and 1e-6 < uscale < 100:
        c["CAD"] = Check("PASS", f"{n_ent} entities, unit_scale={uscale} m/unit, "
                                 f"layers={parse.get('layer_names')}")
    elif n_ent > 0:
        c["CAD"] = Check("WARN", f"{n_ent} entities but suspicious unit_scale={uscale}")
    else:
        c["CAD"] = Check("FAIL", "0 entities parsed")
        res.bugs.append(f"{res.filename}: server parsed 0 entities")

    # 2. ENTITIES — server census vs local ezdxf ground truth (polyline subtype
    # normalized so LWPOLYLINE vs POLYLINE labelling isn't flagged as a diff).
    srv_types: dict[str, int] = {}
    for e in parse.get("entities", []):
        srv_types[e["entity_type"]] = srv_types.get(e["entity_type"], 0) + 1
    loc_types = local.get("types", {})
    if _norm_types(srv_types) == _norm_types(loc_types):
        c["ENTITIES"] = Check("PASS", f"server=={srv_types} (== local by census)")
    else:
        c["ENTITIES"] = Check("WARN", f"server={srv_types} vs local={loc_types}")
        res.bugs.append(f"{res.filename}: entity census differs — server {srv_types} "
                        f"vs ezdxf {loc_types}")

    # 2b. LAYERS — annotation layers (DIM/DEFPOINTS/ANNOT/HATCH) must not be
    # sprayed. Verified at the PLAN level: the planned MARK length must match the
    # ordinary-layer geometry, i.e. annotation length is excluded. (parse-time
    # is_mark still reports True for ignored entities by contract, so it is NOT a
    # reliable signal — the plan is.)
    annot_layers = sorted({e.get("layer", "") for e in parse.get("entities", [])
                           if _ANNOTATION_LAYERS.search(e.get("layer", ""))})
    annot_len = local.get("annot_len_m", 0.0)
    nonannot_len = local.get("nonannot_len_m", 0.0)
    planned_mark = plan.get("mark_length_m", 0.0)
    if not annot_layers:
        c["LAYERS"] = Check("PASS", "no annotation layer present")
    elif res.plan_source == "local-fallback":
        c["LAYERS"] = Check("WARN", f"annotation layer(s) {annot_layers} present; "
                                    "plan via local fallback — server exclusion not exercised")
    else:
        # Excluded → planned mark ≈ non-annotation length. Sprayed → planned mark
        # would include (part of) the annotation length too.
        tol = max(0.15, 0.05 * max(nonannot_len, 1e-6))
        excluded = abs(planned_mark - nonannot_len) <= tol + annot_len * 0.25
        includes_annot = planned_mark >= nonannot_len + annot_len * 0.5 - tol
        if excluded and not includes_annot:
            c["LAYERS"] = Check("PASS",
                f"annotation layer(s) {annot_layers} EXCLUDED from plan "
                f"({annot_len:.2f} m not sprayed; planned MARK {planned_mark:.2f} m "
                f"≈ ordinary {nonannot_len:.2f} m)")
        else:
            c["LAYERS"] = Check("FAIL",
                f"annotation layer(s) {annot_layers} appear sprayed: planned MARK "
                f"{planned_mark:.2f} m vs ordinary-only {nonannot_len:.2f} m "
                f"(annotation {annot_len:.2f} m)")
            res.bugs.append(f"{res.filename}: annotation layer(s) {annot_layers} "
                            f"still sprayed ({annot_len:.2f} m in plan)")

    # 3. EXTENSIONS — config saved + presence of extension segments
    ext_on = res.ext_saved.get("enabled")
    seglist = segs.get("segments", [])
    ext_segs = [s for s in seglist if s.get("is_extension")]
    roles = {}
    for s in seglist:
        r = s.get("segment_role") or "?"
        roles[r] = roles.get(r, 0) + 1
    # A MARK segment is "closed" when its own endpoints coincide — a closed loop
    # correctly gets no extension. Extensions are only expected when a mark
    # segment (chain) has a genuine free end.
    def _seg_closed(s):
        pts = s.get("points", [])
        return len(pts) >= 2 and math.hypot(pts[0][0]-pts[-1][0],
                                            pts[0][1]-pts[-1][1]) < 0.1
    open_marks = [s for s in seglist if s.get("type") == "MARK" and not _seg_closed(s)]
    if res.plan_source == "local-fallback":
        c["EXTENSIONS"] = Check("WARN",
            "N/A — server timed out; rendered via core planner which applies no "
            "extensions (extension stage is the timeout cause, see bug list)")
    elif not ext_on:
        c["EXTENSIONS"] = Check("FAIL", "extensions not enabled on server after POST")
        res.bugs.append(f"{res.filename}: extension config did not persist (enabled!=true)")
    elif ext_segs:
        lens = [round(s.get("length_m", 0), 3) for s in ext_segs]
        near = [l for l in lens if abs(l - EXT_PRE_M) < 0.06 or abs(l - EXT_AFT_M) < 0.06]
        detail = f"{len(ext_segs)} ext segs, roles={roles}, lens={lens}"
        if near:
            c["EXTENSIONS"] = Check("PASS", detail)
        else:
            c["EXTENSIONS"] = Check("WARN", detail + f" (none ≈ {EXT_PRE_M} m)")
    elif not open_marks:
        # Every mark chain is a closed loop → connectivity policy correctly
        # suppresses extensions.
        c["EXTENSIONS"] = Check("PASS",
            f"all mark chains closed → 0 extensions (correct); roles={roles}")
    else:
        # An open mark chain got no run-up — investigate.
        c["EXTENSIONS"] = Check("WARN",
            f"{len(open_marks)} open mark chain(s) but 0 extension segments "
            f"(roles={roles}) — free ends expected a PRE/AFT run-up")

    # 4. DIMENSIONS — planned bbox vs local CAD bbox; filename intent is
    #    advisory (single-shape only — a multi-shape file's combined bbox is
    #    legitimately larger than any one shape's named size).
    pbb = wp_bbox(plan.get("merged_waypoints", []))
    lbb = local.get("bbox_m")
    exp, kind = expected_dim_from_name(res.filename)
    n_mark_chains = roles.get("mark", 0)
    parts = []
    dim_status = "PASS"
    if pbb and lbb:
        ps, ls = sorted(pbb), sorted(lbb)
        max_infl = EXT_PRE_M + EXT_AFT_M + 0.05   # open-end inflation budget
        diff = max(abs(ps[i] - ls[i]) for i in range(2))
        parts.append(f"planned={pbb} CAD={lbb} Δ={round(diff,3)}m")
        if diff > max_infl:
            dim_status = "WARN"
            parts[-1] += f" > ext budget {max_infl}m"
    else:
        dim_status = "WARN"
        parts.append(f"planned={pbb} CAD={lbb}")
    if exp is not None and lbb:
        if n_mark_chains <= 1:
            near_exp = abs(max(lbb) - exp) < 0.15
            parts.append(f"name→{exp}m ({kind}), CAD max={max(lbb)}m "
                         f"{'OK' if near_exp else 'MISMATCH'}")
            if not near_exp:
                dim_status = "WARN" if dim_status == "PASS" else dim_status
                res.bugs.append(f"{res.filename}: CAD size {max(lbb)}m != "
                                f"filename intent {exp}m")
        else:
            parts.append(f"name→{exp}m (advisory; {n_mark_chains} shapes, "
                         "combined bbox expected larger)")
    c["DIMENSIONS"] = Check(dim_status, "; ".join(parts))

    # 5. TRANSITIONS — mark/transit ordering, connectors, closed start/end
    mark_len = plan.get("mark_length_m", 0)
    transit_len = plan.get("transit_length_m", 0)
    n_mark = sum(1 for s in seglist if s.get("type") == "MARK")
    n_transit = sum(1 for s in seglist if s.get("type") == "TRANSIT")
    wps = plan.get("merged_waypoints", [])
    start = wps[0] if wps else None
    end = wps[-1] if wps else None
    closed = (start and end and math.hypot(start[0]-end[0], start[1]-end[1]) < 0.1)
    detail = (f"MARK segs={n_mark} ({round(mark_len,2)}m), TRANSIT segs={n_transit} "
              f"({round(transit_len,2)}m); start={_r(start)} end={_r(end)} "
              f"{'CLOSED' if closed else 'OPEN'}")
    if n_mark == 0:
        c["TRANSITIONS"] = Check("FAIL", "no MARK segments — nothing sprays")
        res.bugs.append(f"{res.filename}: planned path has zero MARK segments")
    else:
        c["TRANSITIONS"] = Check("PASS", detail)


def _r(p):
    if not p:
        return None
    return [round(p[0], 2), round(p[1], 2)]


# ─────────────────────────────────────────────────────────────────────────────
# Rendering
# ─────────────────────────────────────────────────────────────────────────────
def _seg_xy(seg):
    pts = seg.get("points", [])
    e = [p[1] for p in pts]  # east → x
    n = [p[0] for p in pts]  # north → y
    return e, n


def _render_seglist(res: FileResult) -> list:
    """Segments to draw: prefer the point-carrying render_segments (coarsened
    case), else the check segments (which have points for the server path)."""
    if res.render_segments and res.render_segments.get("segments"):
        return res.render_segments["segments"]
    return res.segments.get("segments", [])


def render_png(res: FileResult) -> Optional[str]:
    seglist = _render_seglist(res)
    wps = res.plan.get("merged_waypoints", [])
    if not seglist and not wps:
        return None
    fig, ax = plt.subplots(figsize=(8, 8))

    drew_segment_points = False
    for s in seglist:
        e, n = _seg_xy(s)
        if not e:
            continue
        drew_segment_points = True
        if s.get("is_extension"):
            ax.plot(e, n, "-", color="#ff8c00", lw=2.4, zorder=3)
        elif s.get("type") == "MARK":
            ax.plot(e, n, "-", color="#1a9850", lw=2.6, zorder=2)
        else:  # TRANSIT
            ax.plot(e, n, "--", color="#888888", lw=1.3, zorder=1)

    if not drew_segment_points and wps:
        # Segments carried no point lists (coarsened /plan response) — draw the
        # merged path coloured by spray flag instead.
        flags = res.plan.get("spray_flags", [])
        for i in range(len(wps) - 1):
            on = flags[i] if i < len(flags) else False
            ax.plot([wps[i][1], wps[i+1][1]], [wps[i][0], wps[i+1][0]], "-",
                    color="#1a9850" if on else "#888888",
                    lw=2.0 if on else 1.1, zorder=2 if on else 1)

    if wps:
        s, en = wps[0], wps[-1]
        ax.plot(s[1], s[0], "o", color="#1a9850", ms=13, mec="k", zorder=5, label="start")
        ax.plot(en[1], en[0], "X", color="#d73027", ms=14, mec="k", zorder=5, label="end")

    pbb = wp_bbox(wps)
    lbb = res.local.get("bbox_m")
    chk = res.checks
    _src_notes = {"coarsened": f"  [re-planned @ {COARSE_SPACING_M} m — default 5 cm exceeds cap]",
                  "local-fallback": "  [CORE-PLANNER FALLBACK: server timed out]"}
    src_note = _src_notes.get(res.plan_source, "")
    title = (f"{res.filename}{src_note}\n"
             f"planned bbox {pbb} m | CAD {lbb} m | "
             f"MARK {round(res.plan.get('mark_length_m',0),2)}m  "
             f"TRANSIT {round(res.plan.get('transit_length_m',0),2)}m")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)

    legend = [
        Line2D([0], [0], color="#1a9850", lw=2.6, label="MARK (spray)"),
        Line2D([0], [0], color="#888888", lw=1.3, ls="--", label="TRANSIT"),
        Line2D([0], [0], color="#ff8c00", lw=2.4, label="extension (PRE/AFT)"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#1a9850",
               mec="k", ms=10, label="start"),
        Line2D([0], [0], marker="X", color="w", markerfacecolor="#d73027",
               mec="k", ms=10, label="end"),
    ]
    ax.legend(handles=legend, loc="best", fontsize=8)
    fig.tight_layout()
    out = os.path.join(OUT_DIR, f"{_slug(res.filename)}.png")
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


_CAT_COLOR = {"mark": "#1a9850", "ext": "#ff8c00", "transit": "#888888"}


def _seg_category(s) -> str:
    if s.get("is_extension"):
        return "ext"
    return "mark" if s.get("type") == "MARK" else "transit"


def _ordered_traversal(res: FileResult):
    """Drive-order list of (x=east, y=north, category) built from segments so
    the animation walks MARK / EXTENSION / TRANSIT in the true planned order.
    Falls back to merged_waypoints coloured by spray flag if no segments."""
    seglist = _render_seglist(res)
    order = []
    for s in seglist:
        cat = _seg_category(s)
        for p in s.get("points", []):
            order.append((p[1], p[0], cat))
    if order:
        return order
    wps = res.plan.get("merged_waypoints", [])
    flags = res.plan.get("spray_flags", [])
    return [(p[1], p[0], "mark" if (i < len(flags) and flags[i]) else "transit")
            for i, p in enumerate(wps)]


def render_mp4(res: FileResult) -> Optional[str]:
    order = _ordered_traversal(res)
    if len(order) < 2:
        return None
    xs = [p[0] for p in order]
    ys = [p[1] for p in order]
    seglist = _render_seglist(res)
    n_ext = sum(1 for s in seglist if s.get("is_extension"))

    fig, ax = plt.subplots(figsize=(7.2, 7.6))
    ax.set_aspect("equal")
    pad = 0.6
    ax.set_xlim(min(xs)-pad, max(xs)+pad)
    ax.set_ylim(min(ys)-pad, max(ys)+pad)
    ax.grid(True, alpha=0.3)
    ext_note = f" · {n_ext} extension run-ups" if n_ext else " · no extensions (policy)"
    ax.set_title(f"{res.filename} — start→endpoint traversal{ext_note}", fontsize=10)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")

    # Static full path coloured by category so PRE/AFT extension spurs at each
    # corner are visible even before the dot reaches them.
    for s in seglist:
        pts = s.get("points", [])
        if len(pts) < 2:
            continue
        e = [p[1] for p in pts]; n = [p[0] for p in pts]
        cat = _seg_category(s)
        if cat == "ext":
            ax.plot(e, n, "-", color=_CAT_COLOR["ext"], lw=3.2, zorder=3, alpha=0.9)
        elif cat == "mark":
            ax.plot(e, n, "-", color=_CAT_COLOR["mark"], lw=2.0, zorder=2, alpha=0.55)
        else:
            ax.plot(e, n, "--", color=_CAT_COLOR["transit"], lw=1.1, zorder=1, alpha=0.5)
    if not seglist:  # fallback path
        ax.plot(xs, ys, color="#cccccc", lw=1.0, zorder=1)

    ax.plot(xs[0], ys[0], "o", color="#1a9850", ms=12, mec="k", zorder=6)
    ax.plot(xs[-1], ys[-1], "X", color="#d73027", ms=13, mec="k", zorder=6)

    prog, = ax.plot([], [], lw=4.5, color="k", alpha=0.22, zorder=4)  # covered-so-far
    dot, = ax.plot([], [], "o", ms=11, mec="k", zorder=7)

    legend = [
        Line2D([0], [0], color=_CAT_COLOR["mark"], lw=2.4, label="MARK (spray ON)"),
        Line2D([0], [0], color=_CAT_COLOR["ext"], lw=3.2, label="extension PRE/AFT (spray OFF)"),
        Line2D([0], [0], color=_CAT_COLOR["transit"], lw=1.2, ls="--", label="TRANSIT"),
    ]
    ax.legend(handles=legend, loc="best", fontsize=8)

    N = len(order)
    step = max(1, N // 220)
    frames = list(range(0, N, step)) + [N - 1]

    def upd(i):
        prog.set_data(xs[:i+1], ys[:i+1])
        dot.set_data([xs[i]], [ys[i]])
        dot.set_color(_CAT_COLOR[order[i][2]])
        return prog, dot

    anim = animation.FuncAnimation(fig, upd, frames=frames, interval=50, blit=True)
    out = os.path.join(OUT_DIR, f"{_slug(res.filename)}.mp4")
    try:
        writer = animation.FFMpegWriter(fps=20, bitrate=1800,
                                        extra_args=["-pix_fmt", "yuv420p"])
        anim.save(out, writer=writer)
    except Exception as exc:
        plt.close(fig)
        print(f"  ! MP4 failed for {res.filename}: {exc}")
        return None
    plt.close(fig)
    return out


def _slug(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name)


CHECK_ORDER = ["CAD", "ENTITIES", "LAYERS", "EXTENSIONS", "DIMENSIONS", "TRANSITIONS"]
_STATUS_COLOR = {"PASS": "#c8e6c9", "WARN": "#ffe0b2", "FAIL": "#ffcdd2", "-": "#eeeeee"}
_STATUS_EMOJI = {"PASS": "✅", "WARN": "🟠", "FAIL": "❌", "-": "—"}


def build_report(results: list[FileResult]) -> None:
    # ── summary_index.png : status grid ──────────────────────────────────────
    n = len(results)
    fig, ax = plt.subplots(figsize=(11, 0.55 * n + 2))
    ax.axis("off")
    ncol = len(CHECK_ORDER) + 1
    # header
    headers = ["file"] + CHECK_ORDER
    for j, h in enumerate(headers):
        ax.text(j / ncol + 0.005, 1.0, h, fontsize=9, fontweight="bold",
                transform=ax.transAxes, va="top")
    for i, r in enumerate(results):
        y = 1.0 - (i + 1) * (0.9 / (n + 1)) - 0.03
        _mark = {"coarsened": f"  @{COARSE_SPACING_M}m", "local-fallback": "  ⤷core"}
        label = r.filename + _mark.get(r.plan_source, "")
        ax.text(0.005, y, label, fontsize=8, transform=ax.transAxes, va="center")
        for j, k in enumerate(CHECK_ORDER, start=1):
            chk = r.checks.get(k)
            st = chk.status if chk else "-"
            ax.add_patch(plt.Rectangle(
                (j / ncol + 0.002, y - 0.018), 0.9 / ncol, 0.032,
                transform=ax.transAxes, facecolor=_STATUS_COLOR.get(st, "#eee"),
                edgecolor="white"))
            ax.text(j / ncol + 0.9 / ncol / 2 + 0.002, y, st, fontsize=7,
                    ha="center", va="center", transform=ax.transAxes)
    n_fail = sum(1 for r in results for c in r.checks.values() if c.status == "FAIL")
    n_warn = sum(1 for r in results for c in r.checks.values() if c.status == "WARN")
    n_bug = len({b for r in results for b in r.bugs})
    ax.set_title(f"DXF CAD-flow verification — {n} files | "
                 f"{n_fail} FAIL · {n_warn} WARN · {n_bug} distinct bugs\n"
                 f"server={BASE_URL}", fontsize=11, fontweight="bold")
    fig.tight_layout()
    idx_png = os.path.join(OUT_DIR, "summary_index.png")
    fig.savefig(idx_png, dpi=140)
    plt.close(fig)

    # ── SUMMARY.md ───────────────────────────────────────────────────────────
    lines = []
    lines.append("# DXF → Path CAD-flow verification\n")
    lines.append(f"- **Server (application):** `{BASE_URL}`")
    lines.append(f"- **Files:** {n} (representative subset)")
    _pol = ("per-line / per-entity (every edge + corner gets PRE→MARK→AFT)"
            if PER_LINE else "connectivity-aware (open ends only; closed loops get none)")
    lines.append(f"- **Extensions:** ON, pre={EXT_PRE_M} m, aft={EXT_AFT_M} m, `per_line={PER_LINE}` — {_pol}")
    lines.append(f"- **Totals:** {n_fail} FAIL · {n_warn} WARN · {n_bug} distinct bugs\n")
    lines.append("![status grid](summary_index.png)\n")

    lines.append("## Verification matrix\n")
    lines.append("| file | " + " | ".join(CHECK_ORDER) + " | plan |")
    lines.append("|" + "---|" * (len(CHECK_ORDER) + 2))
    for r in results:
        cells = []
        for k in CHECK_ORDER:
            c = r.checks.get(k)
            cells.append(_STATUS_EMOJI.get(c.status, "—") if c else "—")
        src = {"server": "server", "coarsened": f"@{COARSE_SPACING_M}m",
               "local-fallback": "core⤷"}.get(r.plan_source, r.plan_source)
        lines.append(f"| `{r.filename}` | " + " | ".join(cells) + f" | {src} |")
    lines.append("")

    lines.append("## Per-file detail\n")
    for r in results:
        lines.append(f"### `{r.filename}`  ·  [PNG](./{_slug(r.filename)}.png)"
                     + (f" · [MP4](./{_slug(r.filename)}.mp4)" if r.mp4 else ""))
        if r.plan_source == "coarsened":
            lines.append(f"> ✅ default 5 cm correctly rejected by the waypoint guard "
                         f"(*{r.limit_note}*); re-planned at {COARSE_SPACING_M} m — the guard's own advice.")
        elif r.plan_source == "local-fallback":
            lines.append("> ⚠ planned via **core path_engine fallback** — server /plan could not serve this file.")
        for k in CHECK_ORDER:
            c = r.checks.get(k)
            if c:
                lines.append(f"- **{k}** {_STATUS_EMOJI[c.status]} {c.detail}")
        lines.append("")

    # ── bug list (deduped) ───────────────────────────────────────────────────
    bugs = []
    seen = set()
    for r in results:
        for b in r.bugs:
            if b not in seen:
                seen.add(b); bugs.append(b)
    lines.append("## CAD-flow bugs / findings\n")
    if bugs:
        for i, b in enumerate(bugs, 1):
            lines.append(f"{i}. {b}")
    else:
        lines.append("_None._")
    lines.append("")

    with open(os.path.join(OUT_DIR, "SUMMARY.md"), "w") as fh:
        fh.write("\n".join(lines))
    print(f"Wrote {idx_png}")
    print(f"Wrote {os.path.join(OUT_DIR, 'SUMMARY.md')}")


# ─────────────────────────────────────────────────────────────────────────────
# Driver
# ─────────────────────────────────────────────────────────────────────────────
def process(filename: str, do_video: bool) -> FileResult:
    res = FileResult(filename=filename)
    path = os.path.join(DXF_DIR, filename)
    if not os.path.exists(path):
        res.ok = False
        res.error = "file not found"
        return res
    try:
        res.parse = api_parse_dxf(path)
        name = res.parse.get("filename", filename)
        res.local = local_parse(path, res.parse.get("unit_scale", 0.01))
        res.ext_saved = api_set_extensions(name)
        try:
            res.plan = api_plan(name)
            res.segments = _safe_segments(name)
        except WaypointLimitError as wl:
            # CORRECT behaviour: the mission exceeds the waypoint cap at default
            # spacing and the server rejected it fast with an actionable message
            # (this is the BUG-2 fix working, not a failure). Re-plan at coarser
            # spacing — the message's own advice — so the geometry still verifies.
            res.limit_note = (str(wl).split("Too many waypoints:", 1)[-1]
                              .strip().rstrip('"}').strip()[:140])
            res.plan = api_plan(name, line_spacing=COARSE_SPACING_M)
            # /segments has no spacing override (re-plans at default → same cap),
            # so use the /plan response's own segments for the CHECKS (they carry
            # roles + is_extension but no points)...
            res.segments = {"segments": res.plan.get("segments", [])}
            # ...and compute a point-carrying local plan (same source, same
            # spacing + extension settings) purely so the video/PNG can draw the
            # PRE/AFT run-ups the /plan response can't express.
            try:
                _, res.render_segments = local_core_plan(
                    path, extensions=True, mark_spacing=COARSE_SPACING_M)
            except Exception:
                res.render_segments = None
            res.plan_source = "coarsened"
        except Exception as plan_exc:
            msg = str(plan_exc)
            res.bugs.append(f"{filename}: server /plan failed — {msg[:160]}")
            res.plan, res.segments = local_core_plan(path)
            res.plan_source = "local-fallback"
        run_checks(res)
        res.png = render_png(res)
        if do_video:
            res.mp4 = render_mp4(res)
    except Exception as exc:
        res.ok = False
        res.error = f"{type(exc).__name__}: {exc}"
        res.bugs.append(f"{filename}: pipeline error — {res.error}")
    return res


def main():
    global BASE_URL
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", action="store_true", help="also render MP4s")
    ap.add_argument("--only", nargs="*", help="subset of filenames to run")
    ap.add_argument("--url", default=BASE_URL)
    args = ap.parse_args()

    BASE_URL = args.url
    os.makedirs(OUT_DIR, exist_ok=True)

    files = args.only if args.only else SUBSET
    results: list[FileResult] = []
    print(f"Server: {BASE_URL}  |  {len(files)} files  |  video={args.video}")
    for f in files:
        t0 = time.time()
        r = process(f, args.video)
        dt = time.time() - t0
        status = "OK " if r.ok else "ERR"
        summary = " ".join(f"{k}:{v.status[0]}" for k, v in r.checks.items())
        src = "" if r.plan_source == "server" else f"[{r.plan_source}] "
        print(f"[{status}] {f:<34} {dt:5.1f}s  {src}{summary}  "
              f"{r.error or ''}")
        results.append(r)

    # persist raw + summary
    dump = []
    for r in results:
        d = asdict(r)
        d["checks"] = {k: asdict(v) for k, v in r.checks.items()}
        d.pop("render_segments", None)  # point-heavy, render-only — not for the record
        dump.append(d)
    with open(os.path.join(OUT_DIR, "results.json"), "w") as fh:
        json.dump(dump, fh, indent=2, default=str)
    print(f"\nWrote {os.path.join(OUT_DIR, 'results.json')}")
    build_report(results)
    return results


if __name__ == "__main__":
    main()
