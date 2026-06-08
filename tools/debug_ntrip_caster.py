#!/usr/bin/env python3
"""Standalone NTRIP caster health probe (Mac-local, no ROS/MAVROS deps).

Proves whether the caster authenticates and streams real RTCM3 bytes,
independent of the project code. Reads config from environment variables:

    NTRIP_HOST       (default: caster.emlid.com)
    NTRIP_PORT       (default: 2101)
    NTRIP_MOUNTPT    (required)
    NTRIP_USER       (required)
    NTRIP_PASS       (required)
    NTRIP_DURATION   (seconds to read stream, default: 10)
    NTRIP_SEND_GGA   (1 to send a static GGA after connect, default: 0)

Never logs the password. Exits 0 if RTCM3-like bytes are received, else 1.
"""

import base64
import os
import socket
import sys
import time


def build_gga() -> str:
    # Static, plausible fix (Chennai approx) for VRS casters that need a GGA.
    body = "GPGGA,000000.00,1300.0000,N,08015.0000,E,1,08,1.0,10.0,M,0.0,M,,"
    chk = 0
    for c in body:
        chk ^= ord(c)
    return f"${body}*{chk:02X}\r\n"


def main() -> int:
    host = os.environ.get("NTRIP_HOST", "caster.emlid.com")
    port = int(os.environ.get("NTRIP_PORT", "2101"))
    mount = os.environ.get("NTRIP_MOUNTPT")
    user = os.environ.get("NTRIP_USER")
    pw = os.environ.get("NTRIP_PASS")
    duration = float(os.environ.get("NTRIP_DURATION", "10"))
    send_gga = os.environ.get("NTRIP_SEND_GGA", "0") == "1"

    if not (mount and user and pw):
        print("ERROR: NTRIP_MOUNTPT, NTRIP_USER, NTRIP_PASS are required.")
        return 2

    print(f"Target : {host}:{port}/{mount}  (user={user}, pass=***)")

    # --- DNS + TCP ---------------------------------------------------------
    try:
        ip = socket.gethostbyname(host)
        print(f"DNS    : {host} -> {ip}")
    except Exception as e:
        print(f"DNS    : FAILED ({e})")
        return 1

    creds = base64.b64encode(f"{user}:{pw}".encode()).decode()
    req = (
        f"GET /{mount} HTTP/1.0\r\n"
        f"Host: {host}\r\n"
        f"Ntrip-Version: Ntrip/2.0\r\n"
        f"User-Agent: NTRIP local-debug\r\n"
        f"Authorization: Basic {creds}\r\n"
        f"\r\n"
    )

    try:
        s = socket.create_connection((host, port), timeout=10)
    except Exception as e:
        print(f"TCP    : connect FAILED ({e})")
        return 1
    print("TCP    : connected")

    s.sendall(req.encode())

    # --- Read response header ---------------------------------------------
    s.settimeout(10)
    resp = b""
    leftover = b""
    try:
        while True:
            chunk = s.recv(256)
            if not chunk:
                break
            resp += chunk
            if b"\r\n\r\n" in resp:
                header, leftover = resp.split(b"\r\n\r\n", 1)
                break
            if resp.startswith(b"ICY 200 OK") and b"\r\n" in resp:
                line, leftover = resp.split(b"\r\n", 1)
                header = line
                break
            if len(resp) > 4096:
                header = resp
                break
    except socket.timeout:
        print("HEADER : TIMEOUT waiting for response")
        s.close()
        return 1

    header_txt = header.decode(errors="ignore")
    status_line = header_txt.splitlines()[0] if header_txt else "(empty)"
    print(f"STATUS : {status_line}")

    ok = ("200" in status_line)
    auth_ok = ok and "401" not in header_txt and "Unauthorized" not in header_txt
    print(f"AUTH   : {'accepted' if auth_ok else 'REJECTED/uncertain'}")

    if not ok:
        # Likely a sourcetable (mountpoint wrong) or auth failure.
        snippet = header_txt[:200].replace("\r", " ").replace("\n", " ")
        print(f"BODY   : {snippet}")
        print("RESULT : caster did NOT return a 200 stream — see status above.")
        s.close()
        return 1

    if send_gga:
        try:
            s.sendall(build_gga().encode())
            print("GGA    : static GGA sent")
        except Exception as e:
            print(f"GGA    : send failed ({e})")

    # --- Read stream -------------------------------------------------------
    s.settimeout(duration + 5)
    total = bytearray(leftover)
    start = time.time()
    try:
        while time.time() - start < duration:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                print("STREAM : caster closed connection")
                break
            total += chunk
    finally:
        s.close()

    elapsed = max(time.time() - start, 1e-6)
    n = len(total)
    first32 = bytes(total[:32]).hex(" ")
    rtcm_like = (0xD3 in total[:512]) if n else False
    rate = n / elapsed

    print(f"BYTES  : {n} in {elapsed:.1f}s  (~{rate:.0f} B/s)")
    print(f"HEX32  : {first32}")
    print(f"RTCM3? : {'YES (0xD3 preamble seen)' if rtcm_like else 'NO 0xD3 preamble found'}")

    if n > 0 and rtcm_like:
        print("RESULT : CASTER HEALTHY — real RTCM3 stream confirmed.")
        return 0
    print("RESULT : NO valid RTCM stream — treat caster/config as suspect.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
