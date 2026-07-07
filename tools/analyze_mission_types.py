#!/usr/bin/env python3
"""Classify mission-debug bundles by mission/shape type (line, circle, square, ...).

Reads bundle metadata (manifest.json identity.source_filename) with filename fallback.
Scans Complete/ and Incomplete/ subfolders when present, else flat bundle dirs.

Usage:
    python3 tools/analyze_mission_types.py bags/6-7-2026/new_6-7-2026
    python3 tools/analyze_mission_types.py bags/6-7-2026/new_6-7-2026 --organize
"""
from __future__ import annotations

import argparse
import json
import re
import shutil
import sys
from collections import defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path

# Longer / more specific tokens first to avoid false matches.
SHAPE_RULES: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("triangle", ("triangle", "tri_")),
    ("square", ("square", "sq_")),
    ("circle", ("circle", "circ_")),
    ("arc", ("arc_", "_arc", "arc.")),
    ("line", ("line.", "line_", "_line", "line.dxf")),
    ("lshape", ("lshape", "l_shape", "l-shape")),
    ("uturn", ("uturn", "u_turn", "u-turn")),
    ("multishape", ("multishape", "multi_shape", "sct_")),
    ("text", ("text_", "_text", "text.")),
)


@dataclass
class MissionTypeRecord:
    bundle: str
    folder: str  # Complete | Incomplete | root
    mission_type: str
    source_filename: str | None
    staged_mission_id: str | None
    stop_reason: str | None
    mission_state: str | None


def _read_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def infer_mission_type(*texts: str | None) -> str:
    joined = " ".join(t for t in texts if t).lower()
    if not joined:
        return "unknown"
    for shape, tokens in SHAPE_RULES:
        for tok in tokens:
            if tok in joined:
                return shape
    return "unknown"


def classify_bundle(bundle: Path, folder: str) -> MissionTypeRecord:
    manifest = _read_json(bundle / "manifest.json") or {}
    identity = manifest.get("identity") or {}
    outcome = manifest.get("outcome") or {}
    final_status = outcome.get("final_mission_status") or {}

    source = identity.get("source_filename")
    mission_type = infer_mission_type(source, bundle.name)

    return MissionTypeRecord(
        bundle=bundle.name,
        folder=folder,
        mission_type=mission_type,
        source_filename=source,
        staged_mission_id=identity.get("staged_mission_id"),
        stop_reason=outcome.get("stop_reason"),
        mission_state=final_status.get("state"),
    )


def iter_bundle_dirs(root: Path) -> list[tuple[Path, str]]:
    complete = root / "Complete"
    incomplete = root / "Incomplete"
    if complete.is_dir() or incomplete.is_dir():
        out: list[tuple[Path, str]] = []
        for label, parent in (("Complete", complete), ("Incomplete", incomplete)):
            if not parent.is_dir():
                continue
            for child in sorted(parent.iterdir()):
                if child.is_dir() and (child / "manifest.json").is_file():
                    out.append((child, label))
        return out

    return [
        (child, "root")
        for child in sorted(root.iterdir())
        if child.is_dir()
        and child.name not in {"Complete", "Incomplete"}
        and (child / "manifest.json").is_file()
    ]


def build_report(root: Path) -> dict:
    records = [classify_bundle(b, folder) for b, folder in iter_bundle_dirs(root)]

    by_type: dict[str, list[dict]] = defaultdict(list)
    by_type_outcome: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    for rec in records:
        by_type[rec.mission_type].append(asdict(rec))
        by_type_outcome[rec.mission_type][rec.folder] += 1

    summary_rows = []
    for mtype in sorted(by_type_outcome.keys()):
        counts = by_type_outcome[mtype]
        summary_rows.append({
            "mission_type": mtype,
            "complete": counts.get("Complete", 0),
            "incomplete": counts.get("Incomplete", 0),
            "total": sum(counts.values()),
        })

    return {
        "source": str(root.resolve()),
        "total_bundles": len(records),
        "summary_by_type": summary_rows,
        "bundles": [asdict(r) for r in records],
    }


def render_markdown(report: dict) -> str:
    lines = [
        "# Mission Type Analysis",
        "",
        f"**Source:** `{report['source']}`",
        f"**Total bundles:** {report['total_bundles']}",
        "",
        "## Summary by type",
        "",
        "| Type | Complete | Incomplete | Total |",
        "|------|----------|------------|-------|",
    ]
    for row in report["summary_by_type"]:
        lines.append(
            f"| {row['mission_type']} | {row['complete']} | {row['incomplete']} | {row['total']} |"
        )

    lines.extend(["", "## Bundles by type", ""])
    grouped: dict[str, list[dict]] = defaultdict(list)
    for b in report["bundles"]:
        grouped[b["mission_type"]].append(b)

    for mtype in sorted(grouped.keys()):
        lines.append(f"### {mtype} ({len(grouped[mtype])})")
        lines.append("")
        for b in grouped[mtype]:
            ts = b["bundle"].split("_IST_", 1)[0] if "_IST_" in b["bundle"] else b["bundle"][:19]
            lines.append(
                f"- **{b['folder']}** `{ts}` — `{b['source_filename']}` "
                f"({b['stop_reason'] or '?'})"
            )
        lines.append("")

    return "\n".join(lines)


def organize_by_type(root: Path, records: list[MissionTypeRecord], *, dry_run: bool) -> None:
    for rec in records:
        src = root / rec.folder / rec.bundle
        if not src.is_dir():
            continue
        dest = root / rec.folder / rec.mission_type / rec.bundle
        if dest.exists():
            print(f"SKIP (exists): {dest.relative_to(root)}", file=sys.stderr)
            continue
        if dry_run:
            print(f"MOVE {src.relative_to(root)} -> {dest.relative_to(root)}")
            continue
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(src), str(dest))
        print(f"moved -> {dest.relative_to(root)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze mission types in debug bundles.")
    parser.add_argument("source_dir", type=Path)
    parser.add_argument("--organize", action="store_true", help="Move bundles into type subfolders")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    root = args.source_dir
    if not root.is_dir():
        print(f"not found: {root}", file=sys.stderr)
        return 1

    report = build_report(root)
    md = render_markdown(report)

    json_path = root / "mission_type_report.json"
    md_path = root / "mission_type_report.md"
    if not args.dry_run:
        json_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        md_path.write_text(md, encoding="utf-8")
        print(f"Wrote {json_path}")
        print(f"Wrote {md_path}")

    print()
    print(md)

    if args.organize:
        records = [classify_bundle(b, folder) for b, folder in iter_bundle_dirs(root)]
        print("\n--- Organizing by type ---")
        organize_by_type(root, records, dry_run=args.dry_run)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
