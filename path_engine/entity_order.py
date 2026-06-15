"""Shared entity-order helper.

Used by both the API preview layer (GET /api/path/{name}/entities)
and the planning pipeline (PathManager.plan_path) so the rover
executes entities in exactly the same order the UI displays.

Rules
-----
- If *saved_order* is empty, return parser order unchanged.
- Entities present in *saved_order* appear in that order.
- Entities **not** in *saved_order* are appended at the end in parser order
  (handles newly-added DXF entities that post-date the saved sidecar).
- Saved IDs that no longer exist in the DXF are silently ignored
  (handles deleted / renamed entities).
- No duplicates are emitted.
"""

from __future__ import annotations

from typing import TypeVar

T = TypeVar("T")


def apply_entity_order(entities: list[T], saved_order: list[str]) -> list[T]:
    """Reorder *entities* according to *saved_order*.

    Each entity must expose an ``entity_id`` attribute (all DXFEntity objects do).

    Args:
        entities:    Parsed entity list in DXF / parser order.
        saved_order: Ordered list of entity IDs from the sidecar JSON.

    Returns:
        A new list.  When *saved_order* is empty the original parser order is
        returned (as a fresh list so callers can always mutate freely).
    """
    if not saved_order:
        return list(entities)

    by_id: dict[str, T] = {ent.entity_id: ent for ent in entities}  # type: ignore[attr-defined]
    used: set[str] = set()
    ordered: list[T] = []

    for entity_id in saved_order:
        ent = by_id.get(entity_id)
        if ent is not None and entity_id not in used:
            ordered.append(ent)
            used.add(entity_id)

    # Append any entities that were not covered by the saved order
    for ent in entities:
        eid: str = ent.entity_id  # type: ignore[attr-defined]
        if eid not in used:
            ordered.append(ent)
            used.add(eid)

    return ordered
