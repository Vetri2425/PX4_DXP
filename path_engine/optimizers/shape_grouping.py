"""Shape grouping — chain connected MARK line primitives into ordered runs.

A multi-shape DXF (e.g. a triangle + a square + a circle) is parsed as bare
LINE/CIRCLE primitives with no shape identity. Feeding those primitives
straight into the nearest-neighbour optimizer lets it reorder and reverse
individual lines, so "triangle then square" stops being a coherent shape-level
traversal: square and triangle edges interleave and the shared-edge handoff is
arbitrary.

This pass groups connected line-like MARK segments into composite, continuously
ordered MARK segments BEFORE optimization. The optimizer then orders whole
shapes (and reverses them as a unit) instead of loose primitives, and TRANSIT
links are only inserted between shapes — the natural boundaries. Curved MARK
entities (arc/circle/ellipse/spline) and TRANSIT segments are never merged: a
circle stays its own group so it keeps the smooth profile and its closed-loop
completion guard downstream.

Connectivity is the only shape signal available (the parser emits no group id),
so segments are chained by shared endpoints within a tolerance. At a junction
of degree > 2 (e.g. the vertex where a shared edge meets two shapes) the walk
continues into whichever neighbour keeps the heading straightest, which keeps a
shape's perimeter coherent; any edges left over after one walk are emitted as
additional chains rather than dropped.
"""

from __future__ import annotations

import math

from ..core import PathSegment, SegmentType

_CURVED_GEOMETRY = {"ARC", "CIRCLE", "ELLIPSE", "SPLINE", "LWPOLYLINE_BULGE"}
_CURVED_PREFIXES = ("ARC_", "CIRCLE_", "ELLIPSE_", "SPLINE_")


def _is_chainable(seg: PathSegment) -> bool:
    """True for straight, line-like MARK segments that may be chained.

    Curved entities and TRANSIT segments are excluded — merging a circle into a
    line chain would corrupt its smooth-profile classification downstream.
    """
    if seg.segment_type != SegmentType.MARK:
        return False
    if len(seg.points) < 2:
        return False
    geom = str(seg.metadata.get("geometry_type", "")).upper()
    if geom in _CURVED_GEOMETRY:
        return False
    if seg.source_entity.startswith(_CURVED_PREFIXES):
        return False
    return True


class _NodeIndex:
    """Clusters near-coincident endpoints into shared node ids (O(n^2), n small)."""

    def __init__(self, tol: float):
        self.tol = tol
        self.pts: list[tuple[float, float]] = []

    def get(self, pt: tuple[float, float]) -> int:
        for i, q in enumerate(self.pts):
            if math.hypot(pt[0] - q[0], pt[1] - q[1]) <= self.tol:
                return i
        self.pts.append(pt)
        return len(self.pts) - 1


def _unit(dn: float, de: float) -> tuple[float, float]:
    mag = math.hypot(dn, de)
    if mag < 1e-12:
        return (0.0, 0.0)
    return (dn / mag, de / mag)


def _oriented_from(seg: PathSegment, node_a: int, start_node: int) -> PathSegment:
    """Return seg with its points oriented to begin at ``start_node``."""
    if node_a == start_node:
        return seg
    return PathSegment(
        segment_type=seg.segment_type,
        points=list(reversed(seg.points)),
        speed=seg.speed,
        segment_id=seg.segment_id,
        source_entity=seg.source_entity,
        metadata=dict(seg.metadata),
    )


def _chain_component(
    comp_segs: list[int],
    segs: list[PathSegment],
    endpoints: dict[int, tuple[int, int]],
    tol: float,
) -> list[list[PathSegment]]:
    """Walk one connected component into one or more oriented segment chains."""
    unused = set(comp_segs)
    # node -> list of seg indices incident to it (within this component)
    incident: dict[int, list[int]] = {}
    for si in comp_segs:
        a, b = endpoints[si]
        incident.setdefault(a, []).append(si)
        incident.setdefault(b, []).append(si)

    def degree(node: int) -> int:
        return sum(1 for si in incident.get(node, ()) if si in unused)

    chains: list[list[PathSegment]] = []
    while unused:
        # Prefer starting at an open end (odd/degree-1 node) so an open polyline
        # is traced end-to-end; fall back to any node still carrying an edge.
        start_node = None
        for node in incident:
            if degree(node) == 1:
                start_node = node
                break
        if start_node is None:
            for si in comp_segs:
                if si in unused:
                    start_node = endpoints[si][0]
                    break
        chain: list[PathSegment] = []
        current = start_node
        incoming = None  # travel direction entering ``current``
        while True:
            candidates = [si for si in incident.get(current, ()) if si in unused]
            if not candidates:
                break
            # Choose the neighbour that keeps the heading straightest.
            best_si = None
            best_score = -2.0
            for si in candidates:
                a, b = endpoints[si]
                oriented = _oriented_from(segs[si], a, current)
                p0, p1 = oriented.points[0], oriented.points[1]
                out_dir = _unit(p1[0] - p0[0], p1[1] - p0[1])
                score = 1.0 if incoming is None else (
                    incoming[0] * out_dir[0] + incoming[1] * out_dir[1]
                )
                if score > best_score:
                    best_score = score
                    best_si = si
                    best_oriented = oriented
            si = best_si
            oriented = best_oriented
            a, b = endpoints[si]
            chain.append(oriented)
            unused.discard(si)
            pn, pm = oriented.points[-1], oriented.points[-2]
            incoming = _unit(pn[0] - pm[0], pn[1] - pm[1])
            current = b if a == current else a
        if chain:
            chains.append(chain)
    return chains


def _merge_chain(chain: list[PathSegment], tol: float) -> PathSegment:
    """Concatenate an oriented chain into one composite MARK segment."""
    if len(chain) == 1:
        return chain[0]
    pts: list[tuple[float, float]] = []
    sources: list[str] = []
    for seg in chain:
        if seg.source_entity:
            sources.append(seg.source_entity)
        for pt in seg.points:
            if pts and math.hypot(pt[0] - pts[-1][0], pt[1] - pts[-1][1]) <= tol:
                continue
            pts.append(pt)
    head = chain[0]
    meta = {
        k: v for k, v in head.metadata.items()
        # Tangents/geometry belonged to the first primitive only — dropping them
        # lets the composite be re-tagged below as a whole line chain.
        if k not in ("start_tangent", "end_tangent", "geometry_type",
                     "direction", "reversed")
    }
    meta["grouped_from"] = sources
    meta["geometry_type"] = "LINE_CHAIN"
    meta["line_like"] = True
    label = sources[0] if sources else head.source_entity
    return PathSegment(
        segment_type=SegmentType.MARK,
        points=pts,
        speed=head.speed,
        segment_id=head.segment_id,
        source_entity=f"group:{label}+{len(chain) - 1}",
        metadata=meta,
    )


def group_connected_segments(
    segments: list[PathSegment],
    tol: float = 0.05,
) -> list[PathSegment]:
    """Chain connected line-like MARK segments into composite ordered runs.

    Non-chainable segments (TRANSIT, curved MARK, single-point) and lone
    chainable segments pass through unchanged, so the output is byte-identical
    to the input whenever there is nothing to group. Overall ordering is
    preserved: each connected component's chain(s) are emitted at the position
    of the component's first segment in the input.

    Args:
        segments: Densified segments (MARK + TRANSIT).
        tol: Endpoint-coincidence tolerance in metres.

    Returns:
        New segment list with connected line-like MARK runs merged.
    """
    nodes = _NodeIndex(tol)
    endpoints: dict[int, tuple[int, int]] = {}
    chainable: list[int] = []
    for i, seg in enumerate(segments):
        if _is_chainable(seg):
            a = nodes.get(seg.points[0])
            b = nodes.get(seg.points[-1])
            endpoints[i] = (a, b)
            chainable.append(i)

    if len(chainable) < 2:
        return list(segments)

    # Union-find over chainable segments sharing a node.
    parent = {i: i for i in chainable}

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(x: int, y: int) -> None:
        parent[find(x)] = find(y)

    node_to_seg: dict[int, int] = {}
    for i in chainable:
        for node in endpoints[i]:
            if node in node_to_seg:
                union(i, node_to_seg[node])
            else:
                node_to_seg[node] = i

    components: dict[int, list[int]] = {}
    for i in chainable:
        components.setdefault(find(i), []).append(i)

    # Pre-compute each component's chains.
    comp_chains: dict[int, list[PathSegment]] = {}
    for root, comp_segs in components.items():
        if len(comp_segs) == 1:
            comp_chains[root] = [segments[comp_segs[0]]]
            continue
        chains = _chain_component(comp_segs, segments, endpoints, tol)
        comp_chains[root] = [_merge_chain(c, tol) for c in chains]

    # Emit in input order; each component appears at its first segment's slot.
    out: list[PathSegment] = []
    emitted: set[int] = set()
    for i, seg in enumerate(segments):
        if i not in endpoints:
            out.append(seg)
            continue
        root = find(i)
        if root in emitted:
            continue
        emitted.add(root)
        out.extend(comp_chains[root])
    return out
