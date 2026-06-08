#!/usr/bin/env python3
"""Capture live Socket.IO events emitted by the rover server.

Usage:
    python3 tools/capture_telemetry.py               # 1 telemetry frame
    python3 tools/capture_telemetry.py -n 5          # 5 telemetry frames
    python3 tools/capture_telemetry.py -n 0          # continuous (Ctrl-C to stop)
    python3 tools/capture_telemetry.py -t 5          # all events for 5 seconds
    python3 tools/capture_telemetry.py -t 5 --all    # same (--all is implicit with -t)
    python3 tools/capture_telemetry.py --host 192.168.1.102 --port 5001

Output: one JSON object per line (NDJSON), printed to stdout.
Each object has:
  _event        — Socket.IO event name
  _captured_at  — UTC ISO-8601 timestamp
  ...data fields from the server payload

Errors / connection status go to stderr so stdout stays pipe-clean.

All server events: telemetry, mission_status, mission_status_update,
  mission_loaded, mission_error, rover_disconnected, arm_result, estop_result,
  mode_result, params_result, socket_error, unauthorised, message
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import signal
import sys
from datetime import datetime, timezone

import socketio

# All events the server can emit to the frontend.
ALL_EVENTS = [
    "telemetry",
    "mission_status",
    "mission_status_update",
    "mission_loaded",
    "mission_error",
    "rover_disconnected",
    "arm_result",
    "estop_result",
    "mode_result",
    "params_result",
    "socket_error",
    "unauthorised",
    "message",
]


def _ts() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _clean(v):
    """Recursively replace NaN/Inf with None so output is valid JSON."""
    if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
        return None
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, list):
        return [_clean(x) for x in v]
    return v


def _dumps(event: str, data) -> str:
    if isinstance(data, dict):
        frame = {"_event": event, "_captured_at": _ts(), **data}
    else:
        frame = {"_event": event, "_captured_at": _ts(), "data": data}
    return json.dumps(_clean(frame))


async def capture(host: str, port: int, count: int, duration: float, events: list) -> None:
    url = f"http://{host}:{port}"
    received = 0
    stop = asyncio.Event()

    sio = socketio.AsyncClient(reconnection=False, logger=False, engineio_logger=False)

    @sio.event
    async def connect():
        print(f"[{_ts()}] connected to {url}  events={events}", file=sys.stderr)

    @sio.event
    async def connect_error(data):
        print(f"[{_ts()}] connection error: {data}", file=sys.stderr)
        stop.set()

    @sio.event
    async def disconnect():
        print(f"[{_ts()}] disconnected", file=sys.stderr)
        stop.set()

    def make_handler(event_name: str):
        async def handler(data):
            nonlocal received
            print(_dumps(event_name, data), flush=True)
            received += 1
            if count > 0 and received >= count:
                stop.set()
        return handler

    for ev in events:
        sio.on(ev, make_handler(ev))

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    timer: asyncio.TimerHandle | None = None
    try:
        await sio.connect(url, transports=["websocket"])
        if duration > 0:
            timer = loop.call_later(duration, stop.set)
        await stop.wait()
    except Exception as exc:
        print(f"[{_ts()}] fatal: {exc}", file=sys.stderr)
    finally:
        if timer:
            timer.cancel()
        if sio.connected:
            await sio.disconnect()
        print(f"[{_ts()}] captured {received} frame(s)", file=sys.stderr)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Capture rover WebSocket events",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  # all events for 5 s (typical debug session)\n"
            "  python3 tools/capture_telemetry.py -t 5\n\n"
            "  # 10 telemetry-only frames\n"
            "  python3 tools/capture_telemetry.py -n 10\n\n"
            "  # pipe to jq for GPS fields\n"
            "  python3 tools/capture_telemetry.py -t 5 2>/dev/null"
            " | jq 'select(._event==\"telemetry\") | {gps_fix_name,hrms,vrms}'\n"
        ),
    )
    parser.add_argument("--host", default="192.168.1.102", help="Rover server host")
    parser.add_argument("--port", type=int, default=5001, help="Rover server port")
    parser.add_argument(
        "-n", "--count", type=int, default=0,
        metavar="N",
        help="Stop after N frames (0 = unlimited, default 0 when -t given, else 1)",
    )
    parser.add_argument(
        "-t", "--time", type=float, default=0.0,
        metavar="SEC",
        help="Stop after SEC seconds (captures ALL event types)",
    )
    parser.add_argument(
        "--all", action="store_true",
        help="Listen to all event types (implicit when -t is used)",
    )
    args = parser.parse_args()

    # Determine event scope and count defaults.
    capture_all = args.all or args.time > 0
    events = ALL_EVENTS if capture_all else ["telemetry"]

    # count default: 1 for telemetry-only, unlimited for timed/all-events
    count = args.count
    if count == 0 and args.time == 0 and not capture_all:
        count = 1  # plain invocation with no flags → one telemetry frame

    asyncio.run(capture(args.host, args.port, count, args.time, events))


if __name__ == "__main__":
    main()
