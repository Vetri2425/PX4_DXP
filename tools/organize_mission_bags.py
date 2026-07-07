#!/usr/bin/env python3
"""Classify mission-debug bundles as completed vs incomplete and organize them.

Mission-debug bundles are the directories produced by bag_autorecord (each contains
manifest.json, rosbag/, server/, etc.). Completion is determined from bundle
metadata — not from the INCOMPLETE marker file, which only flags recording
integrity (e.g. a missing topic), not whether the rover finished the mission.

Classification (first match wins):
  1. manifest outcome.stop_reason == "mission_completed"
  2. server/terminal.json reason == "mission_completed"
  3. manifest outcome.final_mission_status.state == "completed"
  4. rosbag fallback: last /rpp/debug state_code == 3 (DONE)

Usage:
    python3 tools/organize_mission_bags.py <source_dir> [--dry-run]
    python3 tools/organize_mission_bags.py <source_dir> --copy
"""
from __future__ import annotations

import argparse
import json
import shutil
import sqlite3
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

RPP_STATE_DONE = 3
COMPLETE_DIR = "Complete"
INCOMPLETE_DIR = "Incomplete"
SKIP_DIR_NAMES = {COMPLETE_DIR, INCOMPLETE_DIR}


@dataclass
class BundleVerdict:
    name: str
    category: str  # "complete" | "incomplete"
    reason: str
    stop_reason: str | None = None
    mission_state: str | None = None
    terminal_reason: str | None = None
    integrity: str | None = None
    rpp_final_state: int | None = None


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def _find_db3(bundle: Path) -> Path | None:
    rosbag_dir = bundle / "rosbag"
    if not rosbag_dir.is_dir():
        return None
    hits = sorted(rosbag_dir.glob("*.db3"))
    return hits[0] if hits else None


def _rpp_final_state(bundle: Path) -> int | None:
    db3 = _find_db3(bundle)
    if db3 is None:
        return None
    try:
        con = sqlite3.connect(f"file:{db3}?mode=ro", uri=True)
        cur = con.cursor()
        row = cur.execute(
            """
            SELECT m.data
            FROM messages m
            JOIN topics t ON t.id = m.topic_id
            WHERE t.name = '/rpp/debug'
            ORDER BY m.timestamp DESC
            LIMIT 1
            """
        ).fetchone()
        con.close()
    except sqlite3.Error:
        return None
    if not row:
        return None
    try:
        from rosbags.typesys import Stores, get_typestore

        ts = get_typestore(Stores.ROS2_HUMBLE)
        msg = ts.deserialize_cdr(row[0], "std_msgs/msg/Float32MultiArray")
        if len(msg.data) > 7:
            return int(msg.data[7])
    except Exception:
        return None
    return None


def classify_bundle(bundle: Path) -> BundleVerdict:
    name = bundle.name
    manifest = _read_json(bundle / "manifest.json") or {}
    outcome = manifest.get("outcome") or {}
    final_status = outcome.get("final_mission_status") or {}

    stop_reason = outcome.get("stop_reason")
    mission_state = final_status.get("state")
    integrity = outcome.get("integrity")

    terminal = _read_json(bundle / "server" / "terminal.json") or {}
    terminal_reason = terminal.get("reason")

    if stop_reason == "mission_completed":
        return BundleVerdict(
            name, "complete", "manifest.stop_reason=mission_completed",
            stop_reason, mission_state, terminal_reason, integrity,
        )
    if terminal_reason == "mission_completed":
        return BundleVerdict(
            name, "complete", "terminal.json.reason=mission_completed",
            stop_reason, mission_state, terminal_reason, integrity,
        )
    if mission_state == "completed":
        return BundleVerdict(
            name, "complete", "final_mission_status.state=completed",
            stop_reason, mission_state, terminal_reason, integrity,
        )

    rpp_state = _rpp_final_state(bundle)
    if rpp_state == RPP_STATE_DONE:
        return BundleVerdict(
            name, "complete", "rosbag /rpp/debug final state=DONE(3)",
            stop_reason, mission_state, terminal_reason, integrity, rpp_state,
        )

    detail_parts = []
    if stop_reason:
        detail_parts.append(f"stop_reason={stop_reason}")
    if mission_state:
        detail_parts.append(f"state={mission_state}")
    if terminal_reason:
        detail_parts.append(f"terminal={terminal_reason}")
    if rpp_state is not None:
        detail_parts.append(f"rpp_final={rpp_state}")
    detail = ", ".join(detail_parts) if detail_parts else "no completion signals"

    return BundleVerdict(
        name, "incomplete", detail,
        stop_reason, mission_state, terminal_reason, integrity, rpp_state,
    )


def _date_prefixes(dates: list[str]) -> tuple[str, ...]:
    """Turn '2026-07-06' or '6-7-2026' into bundle-name prefixes."""
    prefixes: list[str] = []
    for raw in dates:
        raw = raw.strip()
        if not raw:
            continue
        parts = raw.replace("/", "-").split("-")
        if len(parts) == 3 and len(parts[0]) == 4:
            y, m, d = parts
            prefixes.append(f"{y}-{int(m):02d}-{int(d):02d}_")
        elif len(parts) == 3:
            m, d, y = parts
            prefixes.append(f"{y}-{int(m):02d}-{int(d):02d}_")
    return tuple(prefixes)


def _matches_dates(name: str, prefixes: tuple[str, ...]) -> bool:
    return not prefixes or name.startswith(prefixes)


def iter_bundles(source: Path, *, date_prefixes: tuple[str, ...] = ()) -> list[Path]:
    bundles: list[Path] = []
    for child in sorted(source.iterdir()):
        if not child.is_dir():
            continue
        if child.name in SKIP_DIR_NAMES:
            continue
        if not _matches_dates(child.name, date_prefixes):
            continue
        if (child / "manifest.json").is_file() or (child / "rosbag").is_dir():
            bundles.append(child)
    return bundles


def prune_older_bundles(
    source: Path,
    *,
    keep_dates: list[str],
    dry_run: bool = False,
) -> list[str]:
    """Delete bundles in Complete/Incomplete that don't match keep_dates."""
    prefixes = _date_prefixes(keep_dates)
    if not prefixes:
        return []
    deleted: list[str] = []
    for folder in (COMPLETE_DIR, INCOMPLETE_DIR):
        parent = source / folder
        if not parent.is_dir():
            continue
        for bundle in sorted(parent.iterdir()):
            if not bundle.is_dir() or _matches_dates(bundle.name, prefixes):
                continue
            rel = f"{folder}/{bundle.name}"
            if dry_run:
                print(f"DELETE {rel}")
            else:
                shutil.rmtree(bundle)
                print(f"deleted {rel}")
            deleted.append(rel)
    return deleted


def organize(
    source: Path,
    *,
    dry_run: bool = False,
    copy: bool = False,
    keep_dates: list[str] | None = None,
    prune_older: bool = False,
) -> list[BundleVerdict]:
    if not source.is_dir():
        raise FileNotFoundError(f"source directory not found: {source}")

    date_prefixes = _date_prefixes(keep_dates or [])
    verdicts: list[BundleVerdict] = []
    for bundle in iter_bundles(source, date_prefixes=date_prefixes):
        verdicts.append(classify_bundle(bundle))

    complete_dir = source / COMPLETE_DIR
    incomplete_dir = source / INCOMPLETE_DIR

    if not dry_run:
        complete_dir.mkdir(exist_ok=True)
        incomplete_dir.mkdir(exist_ok=True)

    for verdict in verdicts:
        src = source / verdict.name
        dest_parent = complete_dir if verdict.category == "complete" else incomplete_dir
        dest = dest_parent / verdict.name
        if dest.exists():
            print(f"SKIP (already placed): {verdict.name}", file=sys.stderr)
            continue
        action = "COPY" if copy else "MOVE"
        if dry_run:
            print(f"{action:4s} {verdict.category:10s}  {verdict.name}  ({verdict.reason})")
            continue
        if copy:
            shutil.copytree(src, dest)
            print(f"copied  -> {dest.relative_to(source)}  ({verdict.reason})")
        else:
            shutil.move(str(src), str(dest))
            print(f"moved   -> {dest.relative_to(source)}  ({verdict.reason})")

    if prune_older and keep_dates:
        prune_older_bundles(source, keep_dates=keep_dates, dry_run=dry_run)

    report = {
        "source": str(source.resolve()),
        "complete_count": sum(1 for v in verdicts if v.category == "complete"),
        "incomplete_count": sum(1 for v in verdicts if v.category == "incomplete"),
        "bundles": [asdict(v) for v in verdicts],
    }
    if keep_dates:
        report["date_filter"] = keep_dates
    report_path = source / "classification_report.json"
    if not dry_run:
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"\nWrote {report_path}")

    return verdicts


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Sort mission-debug bundles into Complete/ and Incomplete/ folders.",
    )
    parser.add_argument("source_dir", type=Path, help="Directory containing bundle folders")
    parser.add_argument("--dry-run", action="store_true", help="Print actions without moving")
    parser.add_argument(
        "--copy", action="store_true",
        help="Copy bundles instead of moving (keeps originals in place)",
    )
    parser.add_argument(
        "--keep-dates", nargs="+", metavar="DATE",
        help="Only include bundles from these dates (e.g. 2026-07-06 2026-07-07 or 6-7-2026)",
    )
    parser.add_argument(
        "--prune-older", action="store_true",
        help="After organizing, delete bundles in Complete/Incomplete outside --keep-dates",
    )
    parser.add_argument(
        "--prune-only", action="store_true",
        help="Only prune older bundles from existing Complete/Incomplete folders",
    )
    args = parser.parse_args()

    if args.prune_only:
        if not args.keep_dates:
            parser.error("--prune-only requires --keep-dates")
        deleted = prune_older_bundles(
            args.source_dir, keep_dates=args.keep_dates, dry_run=args.dry_run,
        )
        print(f"\nPruned {len(deleted)} older bundle(s)")
        return 0

    verdicts = organize(
        args.source_dir,
        dry_run=args.dry_run,
        copy=args.copy,
        keep_dates=args.keep_dates,
        prune_older=args.prune_older,
    )
    complete = sum(1 for v in verdicts if v.category == "complete")
    incomplete = len(verdicts) - complete
    print(f"\nSummary: {complete} complete, {incomplete} incomplete ({len(verdicts)} total)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
