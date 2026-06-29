import asyncio
import subprocess

import auth
import main
import monitoring
from routes import system


def setup_function():
    auth.reset_for_tests()
    main.ros_node = None


def test_monitoring_routes_are_registered_under_api_prefix():
    paths = {f"/api{getattr(route, 'path', '')}" for route in system.router.routes}

    assert "/api/nodes" in paths
    assert "/api/network" in paths


def test_nodes_api_reports_ros_graph_without_commands():
    class FakeRosNode:
        def get_node_names_and_namespaces(self):
            return [
                ("fastapi_bridge", "/"),
                ("rpp_controller", "/"),
                ("extra_monitor", "/diagnostics"),
            ]

    main.ros_node = FakeRosNode()
    payload = asyncio.run(system.nodes())

    assert payload["source"] == "ros_graph"
    assert payload["ok"] is False
    assert "/diagnostics/extra_monitor" in {
        item["fully_qualified_name"] for item in payload["nodes"]
    }
    expected = {item["fully_qualified_name"]: item for item in payload["expected_nodes"]}
    assert expected["/fastapi_bridge"]["status"] == "present"
    assert expected["/spray_controller"]["status"] == "missing"


def test_collect_nodes_status_degrades_when_ros_graph_and_cli_are_unavailable(monkeypatch):
    monkeypatch.setattr(monitoring.shutil, "which", lambda _name: None)

    payload = monitoring.collect_nodes_status(ros_node=None)

    assert payload["source"] == "unavailable"
    assert payload["ok"] is False
    assert {item["status"] for item in payload["expected_nodes"]} == {"unknown"}


def test_collect_network_telemetry_degrades_to_sysfs(monkeypatch, tmp_path):
    sys_net = tmp_path / "net"
    iface = sys_net / "eno1"
    iface.mkdir(parents=True)
    (iface / "operstate").write_text("up\n", encoding="utf-8")
    (iface / "address").write_text("00:11:22:33:44:55\n", encoding="utf-8")
    monkeypatch.setattr(monitoring.shutil, "which", lambda _name: None)

    payload = monitoring.collect_network_telemetry(sys_class_net=sys_net)

    assert payload["source"] == "sysfs"
    assert payload["interfaces"][0]["name"] == "eno1"
    assert payload["interfaces"][0]["operstate"] == "up"
    assert payload["wifi"]["available"] is False
    assert "iw unavailable" in payload["wifi"]["errors"]


def test_collect_network_telemetry_parses_ip_and_iw(monkeypatch, tmp_path):
    sys_net = tmp_path / "net"
    sys_net.mkdir()

    def fake_which(name):
        return f"/usr/sbin/{name}" if name in {"ip", "iw"} else None

    def fake_runner(cmd, _timeout):
        joined = " ".join(cmd)
        if joined == "ip -j addr show":
            return subprocess.CompletedProcess(
                cmd,
                0,
                '[{"ifname":"wlan0","operstate":"UP","address":"aa:bb",'
                '"addr_info":[{"family":"inet","local":"192.168.1.102","prefixlen":24}]}]',
                "",
            )
        if joined == "ip -j route show default":
            return subprocess.CompletedProcess(
                cmd, 0, '[{"dst":"default","dev":"wlan0","gateway":"192.168.1.1"}]', ""
            )
        if joined == "iw dev":
            return subprocess.CompletedProcess(cmd, 0, "phy#0\n\tInterface wlan0\n", "")
        if joined == "iw dev wlan0 link":
            return subprocess.CompletedProcess(
                cmd,
                0,
                "Connected to aa:bb\n\tSSID: rover-net\n\tfreq: 2412\n"
                "\tsignal: -48 dBm\n\ttx bitrate: 72.2 MBit/s\n",
                "",
            )
        raise AssertionError(cmd)

    monkeypatch.setattr(monitoring.shutil, "which", fake_which)

    payload = monitoring.collect_network_telemetry(
        runner=fake_runner,
        sys_class_net=sys_net,
    )

    assert payload["source"] == "iproute2"
    assert payload["interfaces"][0]["addresses"][0]["address"] == "192.168.1.102"
    assert payload["default_routes"][0]["gateway"] == "192.168.1.1"
    assert payload["wifi"]["interfaces"][0]["ssid"] == "rover-net"
    assert payload["wifi"]["interfaces"][0]["signal_dbm"] == -48.0
