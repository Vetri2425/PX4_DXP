#!/usr/bin/env python3
"""Mission-progress + spray-handshake contract (design doc §3, §4).

Shared, **pure, no-rclpy** vocabulary for the RPP↔spray progress channel:
the `MissionPhase` enum, milestone event names, the four channel payloads
(with JSON (de)serialization), the topic names, and the QoS *specs* as plain
data. Same philosophy as `spray_modes.py` / `spray_flow_model.py`: no ROS
import, so it is unit-testable off-robot (`test_mission_progress.py` runs on
the Mac, which has no rclpy). The nodes own the ROS glue and build real
`QoSProfile`s from the specs here.

Why a shared module instead of duplicating constants (as the spray node does
for `CORNER_ALIGN` at spray_controller_node.py:70)? That duplication avoids a
dependency on the **controller** module specifically. This module is neutral —
neither node's control logic lives here — so both nodes may import it, exactly
as they already import `spray_modes` etc. Single source of truth, no cross-node
controller coupling.

**Design decision (open-Q #1, RESOLVED 2026-07-23): `MissionPhase` is a
SEPARATE enum on a SEPARATE topic (`/rpp/progress`), NOT appended to
`SegmentStateCode` / `/rpp/segment_debug`.** Tracking-state and mission-phase
are orthogonal, co-occurring axes (e.g. MARK_TRACKING ∧ PRE_CORNER_SLOWDOWN),
so one enum slot cannot hold both without losing information; and keeping them
apart makes the frozen `/rpp/segment_debug` contract *structurally* untouched.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from enum import IntEnum


# ---------------------------------------------------------------------------
# Topic names (single source of truth for both nodes + the server)
# ---------------------------------------------------------------------------
TOPIC_PROGRESS = "/rpp/progress"        # RPP → spray, high-rate state
TOPIC_MILESTONE = "/rpp/milestone"      # RPP → spray/server, discrete events
TOPIC_POINT_DONE = "/spray/point_done"  # spray → RPP, point completion
TOPIC_ADVANCE = "/point/advance"        # server → RPP, manual "next point"


# ---------------------------------------------------------------------------
# MissionPhase — the mission-progress axis (design §3). SEPARATE from
# SegmentStateCode; published only on /rpp/progress.
# ---------------------------------------------------------------------------
class MissionPhase(IntEnum):
    IDLE = 0            # no path / no pose
    TRANSIT = 1         # driving a no-spray connector (spray OFF)
    PRE_EXT = 2         # inside a PRE extension leg (approaching a mark, OFF)
    APPROACH_MARK = 3   # within lead distance of a MARK start (may lead ON)
    MARK_TRACKING = 4   # inside a MARK region, tracking the line
    MARK_END = 5        # within lead distance of the MARK end (may lead OFF)
    AFT_EXT = 6         # inside an AFT extension leg (spray OFF)
    APPROACH_POINT = 7  # point mode: closing on must-hit point i
    AT_POINT = 8        # point mode: precise-stopped & confirmed on point i
    DWELL_HOLD = 9      # point mode: holding while spray dwells point i
    WAIT_OPERATOR = 10  # point mode (manual): dwell done, awaiting /point/advance
    REACHED_END = 11    # final waypoint reached, mission complete


# ---------------------------------------------------------------------------
# Milestone event names (design §4.2). Discrete, once-per-transition, RELIABLE.
# ---------------------------------------------------------------------------
class MilestoneEvent:
    MARK_START = "MARK_START"
    MARK_STOP = "MARK_STOP"
    PRE_START = "PRE_START"
    AFT_STOP = "AFT_STOP"
    AT_POINT = "AT_POINT"
    DWELL_DONE_RPP = "DWELL_DONE_RPP"
    REACHED_END = "REACHED_END"

    ALL = frozenset({
        MARK_START, MARK_STOP, PRE_START, AFT_STOP,
        AT_POINT, DWELL_DONE_RPP, REACHED_END,
    })


# ---------------------------------------------------------------------------
# QoS specs — plain data (no rclpy). Each node maps these to a real
# QoSProfile. Mirrors existing choices: high-rate telemetry = BEST_EFFORT
# (like /rpp/segment_debug); must-arrive commands/events = RELIABLE VOLATILE
# (like /spray/manual) — never TRANSIENT_LOCAL, so a restart cannot replay a
# stale "advance"/"done".
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class QoSSpec:
    reliability: str  # "BEST_EFFORT" | "RELIABLE"
    durability: str   # "VOLATILE" | "TRANSIENT_LOCAL"
    depth: int


PROGRESS_QOS = QoSSpec("BEST_EFFORT", "VOLATILE", 1)    # §4.1 — 50 Hz, loss-tolerant
MILESTONE_QOS = QoSSpec("RELIABLE", "VOLATILE", 10)     # §4.2 — must not drop
POINT_DONE_QOS = QoSSpec("RELIABLE", "VOLATILE", 10)    # §4.3 — must not drop
ADVANCE_QOS = QoSSpec("RELIABLE", "VOLATILE", 1)        # §4.4 — must not drop


# ---------------------------------------------------------------------------
# Channel payloads. `to_json()` is strict (always emits every field);
# `from_json()` is LENIENT (missing/garbled fields fall back to safe defaults)
# so a consumer never crashes on a malformed tick — it degrades, per the
# "proof augments, never strands" principle (design §2).
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ProgressMsg:
    """`/rpp/progress` — continuous state (RPP → spray). Design §4.1."""

    phase: MissionPhase = MissionPhase.IDLE
    segment_index: int = -1
    point_index: int = -1
    dist_to_next_boundary_m: float = float("nan")
    next_boundary: str = ""          # name of the next phase transition, or ""
    speed_mps: float = 0.0
    stopped: bool = False            # confirmed physically stopped
    xtrack_m: float = float("nan")   # signed cross-track, observability only

    def to_json(self) -> str:
        return json.dumps({
            "phase": int(self.phase),
            "phase_name": MissionPhase(self.phase).name,
            "segment_index": self.segment_index,
            "point_index": self.point_index,
            "dist_to_next_boundary_m": _num(self.dist_to_next_boundary_m),
            "next_boundary": self.next_boundary,
            "speed_mps": _num(self.speed_mps),
            "stopped": bool(self.stopped),
            "xtrack_m": _num(self.xtrack_m),
        })

    @classmethod
    def from_json(cls, raw: str) -> "ProgressMsg":
        d = _loads(raw)
        return cls(
            phase=_as_phase(d.get("phase")),
            segment_index=_as_int(d.get("segment_index"), -1),
            point_index=_as_int(d.get("point_index"), -1),
            dist_to_next_boundary_m=_as_float(d.get("dist_to_next_boundary_m")),
            next_boundary=str(d.get("next_boundary", "")),
            speed_mps=_as_float(d.get("speed_mps"), 0.0),
            stopped=bool(d.get("stopped", False)),
            xtrack_m=_as_float(d.get("xtrack_m")),
        )


@dataclass(frozen=True)
class MilestoneMsg:
    """`/rpp/milestone` — discrete event (RPP → spray/server). Design §4.2."""

    event: str
    seq: int
    index: int = -1            # point/segment index the event refers to, else -1
    stamp_ns: int = 0

    def to_json(self) -> str:
        return json.dumps({
            "event": self.event,
            "index": self.index,
            "seq": self.seq,
            "stamp_ns": self.stamp_ns,
        })

    @classmethod
    def from_json(cls, raw: str) -> "MilestoneMsg":
        d = _loads(raw)
        return cls(
            event=str(d.get("event", "")),
            seq=_as_int(d.get("seq"), -1),
            index=_as_int(d.get("index"), -1),
            stamp_ns=_as_int(d.get("stamp_ns"), 0),
        )


@dataclass(frozen=True)
class PointDoneMsg:
    """`/spray/point_done` — completion (spray → RPP). Design §4.3."""

    point_index: int
    seq: int
    done: bool = True
    reason: str = "dwell_complete"

    def to_json(self) -> str:
        return json.dumps({
            "point_index": self.point_index,
            "done": bool(self.done),
            "seq": self.seq,
            "reason": self.reason,
        })

    @classmethod
    def from_json(cls, raw: str) -> "PointDoneMsg":
        d = _loads(raw)
        return cls(
            point_index=_as_int(d.get("point_index"), -1),
            seq=_as_int(d.get("seq"), -1),
            done=bool(d.get("done", False)),
            reason=str(d.get("reason", "")),
        )


@dataclass(frozen=True)
class AdvanceMsg:
    """`/point/advance` — operator command (server → RPP). Design §4.4.

    `expect_index` guards against stale double-taps: the RPP rejects an
    advance whose `expect_index` does not match the point it is holding at.
    """

    advance: bool = True
    expect_index: int = -1

    def to_json(self) -> str:
        return json.dumps({
            "advance": bool(self.advance),
            "expect_index": self.expect_index,
        })

    @classmethod
    def from_json(cls, raw: str) -> "AdvanceMsg":
        d = _loads(raw)
        return cls(
            advance=bool(d.get("advance", False)),
            expect_index=_as_int(d.get("expect_index"), -1),
        )


# ---------------------------------------------------------------------------
# Lenient parse helpers — never raise on malformed input; degrade to defaults.
# ---------------------------------------------------------------------------
def _loads(raw: str) -> dict:
    try:
        d = json.loads(raw)
        return d if isinstance(d, dict) else {}
    except (ValueError, TypeError):
        return {}


def _num(x: float) -> float | None:
    """JSON has no NaN/Inf — emit null so the wire stays valid JSON."""
    try:
        xf = float(x)
    except (ValueError, TypeError):
        return None
    if xf != xf or xf in (float("inf"), float("-inf")):
        return None
    return xf


def _as_int(x, default: int) -> int:
    try:
        return int(x)
    except (ValueError, TypeError):
        return default


def _as_float(x, default: float = float("nan")) -> float:
    if x is None:
        return default
    try:
        return float(x)
    except (ValueError, TypeError):
        return default


def _as_phase(x) -> MissionPhase:
    try:
        return MissionPhase(int(x))
    except (ValueError, TypeError):
        return MissionPhase.IDLE
