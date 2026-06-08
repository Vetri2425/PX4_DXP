#!/usr/bin/env python3
"""Capture live Socket.IO telemetry from the rover server.

Usage:
    python3 tools/capture_telemetry.py            # once
    python3 tools/capture_telemetry.py -n 3       # three samples
    python3 tools/capture_telemetry.py -n 5       # five samples
    python3 tools/capture_telemetry.py -n 0       # continuous (Ctrl-C to stop)
    python3 tools/capture_telemetry.py --host 192.168.1.102 --port 5001

Output: one JSON object per line (NDJSON), printed to stdout.
Errors / connection status go to stderr so stdout stays pipe-clean.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import signal
import sys
from datetime import datetime, timezone

import socketio


def _ts() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _dumps(obj) -> str:
    """JSON serialise; NaN/Inf → null (JSON spec has no NaN literal)."""
    import math

    def _default(v):
        if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
            return None
        raise TypeError(type(v))

    # walk dict/list to replace nan before encoding
    def _clean(v):
        if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
            return None
        if isinstance(v, dict):
            return {k: _clean(x) for k, x in v.items()}
        if isinstance(v, list):
            return [_clean(x) for x in v]
        return v

    return json.dumps(_clean(obj))


async def capture(host: str, port: int, count: int) -> None:
    """Connect and collect `count` telemetry frames (0 = unlimited)."""
    url = f"http://{host}:{port}"
    received = 0
    stop = asyncio.Event()

    sio = socketio.AsyncClient(reconnection=False, logger=False, engineio_logger=False)

    @sio.event
    async def connect():
        print(f"[{_ts()}] connected to {url}", file=sys.stderr)

    @sio.event
    async def connect_error(data):
        print(f"[{_ts()}] connection error: {data}", file=sys.stderr)
        stop.set()

    @sio.event
    async def disconnect():
        print(f"[{_ts()}] disconnected", file=sys.stderr)
        stop.set()

    @sio.on("telemetry")
    async def on_telemetry(data):
        nonlocal received
        frame = {"_captured_at": _ts(), **data}
        print(_dumps(frame), flush=True)
        received += 1
        if count > 0 and received >= count:
            stop.set()

    # Allow Ctrl-C to break the wait loop cleanly.
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    try:
        await sio.connect(url, transports=["websocket"])
        await stop.wait()
    except Exception as exc:
        print(f"[{_ts()}] fatal: {exc}", file=sys.stderr)
    finally:
        if sio.connected:
            await sio.disconnect()
        print(f"[{_ts()}] captured {received} frame(s)", file=sys.stderr)


def main() -> None:
    parser = argparse.ArgumentParser(description="Capture rover WebSocket telemetry")
    parser.add_argument("--host", default="192.168.1.102", help="Rover server host")
    parser.add_argument("--port", type=int, default=5001, help="Rover server port")
    parser.add_argument(
        "-n", "--count", type=int, default=1,
        metavar="N",
        help="Number of frames to capture (0 = continuous, default 1)",
    )
    args = parser.parse_args()
    asyncio.run(capture(args.host, args.port, args.count))


if __name__ == "__main__":
    main()
