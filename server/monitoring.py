"""Read-only rover monitoring helpers.

The helpers in this module never command ROS, systemd, networking, or the FCU.
Every platform probe is best-effort and bounded by a short timeout so endpoints
degrade to explicit unavailable/unknown fields on developer machines.
"""

from __future__ import annotations

import datetime as dt
import json
import re
import shutil
import socket
import subprocess
from pathlib import Path
from typing import Callable

EXPECTED_ROS_NODES = (
    "/fastapi_bridge",
    "/mavros",
    "/rpp_controller",
    "/spray_controller",
    "/twist_to_setpoint",
)

CommandRunner = Callable[[list[str], float], subprocess.CompletedProcess[str]]


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds").replace(
        "+00:00", "Z"
    )


def _run_command(cmd: list[str], timeout_s: float) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        capture_output=True,
        check=False,
        text=True,
        timeout=timeout_s,
    )


def _fq_name(namespace: str, name: str) -> str:
    ns = namespace or "/"
    if not ns.startswith("/"):
        ns = f"/{ns}"
    if ns != "/" and ns.endswith("/"):
        ns = ns[:-1]
    node = name[1:] if name.startswith("/") else name
    return f"/{node}" if ns == "/" else f"{ns}/{node}"


def _split_fq_name(value: str) -> tuple[str, str]:
    if not value.startswith("/"):
        value = f"/{value}"
    namespace, _, name = value.rpartition("/")
    return name, namespace or "/"


def collect_nodes_status(ros_node=None, runner: CommandRunner = _run_command) -> dict:
    errors: list[str] = []
    found: set[str] = set()
    source = "unavailable"

    if ros_node is not None and hasattr(ros_node, "get_node_names_and_namespaces"):
        try:
            found = {
                _fq_name(namespace, name)
                for name, namespace in ros_node.get_node_names_and_namespaces()
            }
            source = "ros_graph"
        except Exception as exc:
            errors.append(f"ros graph unavailable: {exc}")

    if not found and shutil.which("ros2"):
        try:
            proc = runner(["ros2", "node", "list"], 1.5)
            if proc.returncode == 0:
                found = {
                    line.strip()
                    for line in proc.stdout.splitlines()
                    if line.strip().startswith("/")
                }
                source = "ros2_cli"
            else:
                detail = (proc.stderr or proc.stdout).strip()
                errors.append(f"ros2 node list failed: {detail or proc.returncode}")
        except (OSError, subprocess.TimeoutExpired) as exc:
            errors.append(f"ros2 node list unavailable: {exc}")

    expected = []
    for fq in EXPECTED_ROS_NODES:
        name, namespace = _split_fq_name(fq)
        status = "present" if fq in found else ("missing" if source != "unavailable" else "unknown")
        expected.append(
            {
                "name": name,
                "namespace": namespace,
                "fully_qualified_name": fq,
                "expected": True,
                "status": status,
            }
        )

    nodes = []
    for fq in sorted(found):
        name, namespace = _split_fq_name(fq)
        nodes.append(
            {
                "name": name,
                "namespace": namespace,
                "fully_qualified_name": fq,
                "expected": fq in EXPECTED_ROS_NODES,
                "status": "present",
            }
        )

    return {
        "ok": source != "unavailable" and all(item["status"] == "present" for item in expected),
        "source": source,
        "nodes": nodes,
        "expected_nodes": expected,
        "errors": errors,
    }


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return None


def _sysfs_interfaces(sys_class_net: Path = Path("/sys/class/net")) -> list[dict]:
    interfaces = []
    try:
        entries = sorted(sys_class_net.iterdir(), key=lambda item: item.name)
    except OSError:
        return interfaces
    for entry in entries:
        interfaces.append(
            {
                "name": entry.name,
                "operstate": _read_text(entry / "operstate"),
                "mac": _read_text(entry / "address"),
                "addresses": [],
            }
        )
    return interfaces


def _interfaces_from_ip(data: list[dict], sys_class_net: Path) -> list[dict]:
    sysfs_by_name = {item["name"]: item for item in _sysfs_interfaces(sys_class_net)}
    interfaces = []
    for item in data:
        name = str(item.get("ifname") or "")
        if not name:
            continue
        sysfs = sysfs_by_name.get(name, {})
        addresses = []
        for addr in item.get("addr_info") or []:
            family = addr.get("family")
            local = addr.get("local")
            if family and local:
                addresses.append(
                    {
                        "family": str(family),
                        "address": str(local),
                        "prefixlen": addr.get("prefixlen"),
                    }
                )
        interfaces.append(
            {
                "name": name,
                "operstate": item.get("operstate") or sysfs.get("operstate"),
                "mac": item.get("address") or sysfs.get("mac"),
                "addresses": addresses,
            }
        )
    return interfaces


def _default_routes_from_ip(data: list[dict]) -> list[dict]:
    routes = []
    for item in data:
        dst = item.get("dst", "default")
        if dst != "default":
            continue
        routes.append(
            {
                "interface": item.get("dev"),
                "gateway": item.get("gateway"),
                "destination": "default",
            }
        )
    return routes


def _parse_iw_dev(stdout: str) -> list[str]:
    interfaces = []
    for line in stdout.splitlines():
        stripped = line.strip()
        if stripped.startswith("Interface "):
            interfaces.append(stripped.split(None, 1)[1])
    return interfaces


def _parse_iw_link(interface: str, stdout: str) -> dict:
    payload = {"interface": interface, "connected": False}
    if "Not connected" in stdout:
        return payload
    payload["connected"] = True
    for line in stdout.splitlines():
        stripped = line.strip()
        if stripped.startswith("SSID:"):
            payload["ssid"] = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("freq:"):
            try:
                payload["frequency_mhz"] = int(stripped.split(":", 1)[1].strip())
            except ValueError:
                pass
        elif stripped.startswith("signal:"):
            match = re.search(r"signal:\s*(-?\d+(?:\.\d+)?)", stripped)
            if match:
                payload["signal_dbm"] = float(match.group(1))
        elif stripped.startswith("tx bitrate:"):
            payload["tx_bitrate"] = stripped.split(":", 1)[1].strip()
    return payload


def _wifi_status(runner: CommandRunner) -> dict:
    errors: list[str] = []
    if not shutil.which("iw"):
        return {"available": False, "interfaces": [], "errors": ["iw unavailable"]}
    try:
        proc = runner(["iw", "dev"], 1.0)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"available": False, "interfaces": [], "errors": [f"iw dev unavailable: {exc}"]}
    if proc.returncode != 0:
        detail = (proc.stderr or proc.stdout).strip()
        return {
            "available": False,
            "interfaces": [],
            "errors": [f"iw dev failed: {detail or proc.returncode}"],
        }
    interfaces = []
    for name in _parse_iw_dev(proc.stdout):
        try:
            link = runner(["iw", "dev", name, "link"], 1.0)
        except (OSError, subprocess.TimeoutExpired) as exc:
            errors.append(f"iw link {name} unavailable: {exc}")
            interfaces.append({"interface": name, "connected": None})
            continue
        if link.returncode != 0:
            errors.append(f"iw link {name} failed: {(link.stderr or link.stdout).strip()}")
            interfaces.append({"interface": name, "connected": None})
            continue
        interfaces.append(_parse_iw_link(name, link.stdout))
    return {"available": True, "interfaces": interfaces, "errors": errors}


def collect_network_telemetry(
    runner: CommandRunner = _run_command,
    sys_class_net: Path = Path("/sys/class/net"),
) -> dict:
    errors: list[str] = []
    source = "unavailable"
    interfaces: list[dict] = []
    default_routes: list[dict] = []

    if shutil.which("ip"):
        try:
            addr_proc = runner(["ip", "-j", "addr", "show"], 1.0)
            if addr_proc.returncode == 0:
                interfaces = _interfaces_from_ip(json.loads(addr_proc.stdout), sys_class_net)
                source = "iproute2"
            else:
                errors.append(f"ip addr failed: {(addr_proc.stderr or addr_proc.stdout).strip()}")
        except (OSError, subprocess.TimeoutExpired, json.JSONDecodeError) as exc:
            errors.append(f"ip addr unavailable: {exc}")

        try:
            route_proc = runner(["ip", "-j", "route", "show", "default"], 1.0)
            if route_proc.returncode == 0:
                default_routes = _default_routes_from_ip(json.loads(route_proc.stdout))
            else:
                errors.append(
                    f"ip route failed: {(route_proc.stderr or route_proc.stdout).strip()}"
                )
        except (OSError, subprocess.TimeoutExpired, json.JSONDecodeError) as exc:
            errors.append(f"ip route unavailable: {exc}")

    if not interfaces:
        interfaces = _sysfs_interfaces(sys_class_net)
        if interfaces:
            source = "sysfs"
        elif source == "unavailable":
            errors.append("network interfaces unavailable")

    return {
        "timestamp": _utc_now(),
        "hostname": socket.gethostname(),
        "source": source,
        "interfaces": interfaces,
        "default_routes": default_routes,
        "wifi": _wifi_status(runner),
        "errors": errors,
    }
