"""Regression tests for spray parameter service handling and load safety."""

from __future__ import annotations

import asyncio
import inspect
import json
import os
import sys
import types
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from fastapi import HTTPException


def _install_ros_stubs() -> None:
    if "rclpy.callback_groups" in sys.modules:
        return

    rclpy = sys.modules.get("rclpy", types.ModuleType("rclpy"))
    rclpy.init = lambda *a, **k: None
    rclpy.ok = lambda: True
    sys.modules["rclpy"] = rclpy

    cbg = types.ModuleType("rclpy.callback_groups")
    cbg.ReentrantCallbackGroup = object
    cbg.MutuallyExclusiveCallbackGroup = object
    sys.modules["rclpy.callback_groups"] = cbg

    rclpy_exec = types.ModuleType("rclpy.executors")
    rclpy_exec.MultiThreadedExecutor = object
    sys.modules["rclpy.executors"] = rclpy_exec

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = object
    sys.modules["rclpy.node"] = rclpy_node

    rclpy_qos = types.ModuleType("rclpy.qos")

    class _Enum:
        BEST_EFFORT = RELIABLE = VOLATILE = TRANSIENT_LOCAL = KEEP_LAST = 1

    rclpy_qos.QoSProfile = lambda *a, **k: None
    rclpy_qos.ReliabilityPolicy = _Enum
    rclpy_qos.DurabilityPolicy = _Enum
    rclpy_qos.HistoryPolicy = _Enum
    sys.modules["rclpy.qos"] = rclpy_qos

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msg = types.ModuleType("geometry_msgs.msg")

    class _PoseStamped:
        pass

    class _TwistStamped:
        pass

    class _Vector3Stamped:
        pass

    geometry_msg.PoseStamped = _PoseStamped
    geometry_msg.TwistStamped = _TwistStamped
    geometry_msg.Vector3Stamped = _Vector3Stamped
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msg

    nav_msgs = types.ModuleType("nav_msgs")
    nav_msg = types.ModuleType("nav_msgs.msg")
    nav_msg.Path = type("Path", (), {})
    sys.modules["nav_msgs"] = nav_msgs
    sys.modules["nav_msgs.msg"] = nav_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.Bool = type("Bool", (), {"__init__": lambda self: setattr(self, "data", False)})
    std_msg.Float32MultiArray = type(
        "Float32MultiArray", (), {"__init__": lambda self: setattr(self, "data", [])}
    )
    std_msg.String = type("String", (), {"__init__": lambda self: setattr(self, "data", "")})
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msg

    rcl_interfaces = types.ModuleType("rcl_interfaces")
    rcl_srv = types.ModuleType("rcl_interfaces.srv")
    rcl_msg = types.ModuleType("rcl_interfaces.msg")

    class _SetParameters:
        class Request:
            def __init__(self):
                self.parameters = []

    class _GetParameters:
        class Request:
            pass

    class _ListParameters:
        class Request:
            depth = 0

    class _Parameter:
        def __init__(self):
            self.name = ""
            self.value = None

    class _ParameterValue:
        pass

    class _ParameterType:
        pass

    rcl_srv.SetParameters = _SetParameters
    rcl_srv.GetParameters = _GetParameters
    rcl_srv.ListParameters = _ListParameters
    rcl_msg.Parameter = _Parameter
    rcl_msg.ParameterValue = _ParameterValue
    rcl_msg.ParameterType = _ParameterType
    sys.modules["rcl_interfaces"] = rcl_interfaces
    sys.modules["rcl_interfaces.srv"] = rcl_srv
    sys.modules["rcl_interfaces.msg"] = rcl_msg

    std_srvs = types.ModuleType("std_srvs")
    std_srv = types.ModuleType("std_srvs.srv")

    class _Trigger:
        class Request:
            pass

        class Response:
            success = False
            message = ""

    std_srv.Trigger = _Trigger
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = std_srv


_install_ros_stubs()
sys.path.insert(0, os.path.dirname(__file__))

import main
import routes.path as path_routes
from models import LoadMissionRequest, MissionState
from ros_node import RosBridgeNode
from spray_mission_config import apply_spray_mission_config


class _FakeSetParamResult:
    def __init__(self, successful: bool, reason: str = ""):
        self.successful = successful
        self.reason = reason


class _FakeSetParamResponse:
    def __init__(self, results):
        self.results = results


class _FakeSprayParamClient:
    def __init__(self, *, wait_result=True, wait_error=None, call_error=None):
        self._wait_result = wait_result
        self._wait_error = wait_error
        self._call_error = call_error
        self.wait_calls = []
        self.call_requests = []

    def wait_for_service(self, timeout_sec=None):
        self.wait_calls.append(timeout_sec)
        if self._wait_error is not None:
            raise self._wait_error
        return self._wait_result

    def call_async(self, req):
        self.call_requests.append(req)
        if self._call_error is not None:
            raise self._call_error
        return MagicMock(name="ros_future")


def _spray_node(
    client=None,
    *,
    await_impl=None,
) -> RosBridgeNode:
    node = object.__new__(RosBridgeNode)
    node._spray_param_set_cli = client
    if await_impl is not None:
        node._await_ros_future = await_impl
    else:
        node._await_ros_future = RosBridgeNode._await_ros_future.__get__(node, RosBridgeNode)
    return node


@pytest.fixture
def spray_req():
    return SimpleNamespace(parameters=[])


@pytest.mark.anyio
async def test_spray_param_client_missing(spray_req):
    node = _spray_node(client=None)
    ok, results, msg = await node._call_spray_set_param(spray_req)
    assert ok is False
    assert results == []
    assert msg == "Spray parameter client is not initialized"


@pytest.mark.anyio
async def test_spray_param_service_unavailable(spray_req):
    client = _FakeSprayParamClient(wait_result=False)
    node = _spray_node(client)
    ok, results, msg = await node._call_spray_set_param(
        spray_req, service_wait_timeout_s=2.0
    )
    assert ok is False
    assert results == []
    assert msg == "Spray parameter service unavailable after 2.0s"
    assert client.wait_calls == [2.0]


@pytest.mark.anyio
async def test_spray_param_service_check_exception(spray_req):
    client = _FakeSprayParamClient(wait_error=RuntimeError("discovery broken"))
    node = _spray_node(client)
    ok, results, msg = await node._call_spray_set_param(spray_req)
    assert ok is False
    assert results == []
    assert msg == "Spray parameter service check failed: discovery broken"


@pytest.mark.anyio
async def test_spray_param_response_timeout(spray_req):
    client = _FakeSprayParamClient(wait_result=True)

    async def _timeout(_future, timeout):
        raise asyncio.TimeoutError()

    node = _spray_node(client, await_impl=_timeout)
    ok, results, msg = await node._call_spray_set_param(
        spray_req, response_timeout_s=8.0
    )
    assert ok is False
    assert results == []
    assert msg == "Spray parameter response timed out after 8.0s"
    assert len(client.call_requests) == 1


@pytest.mark.anyio
async def test_spray_param_call_exception(spray_req):
    client = _FakeSprayParamClient(
        wait_result=True, call_error=RuntimeError("transport down")
    )
    node = _spray_node(client)
    ok, results, msg = await node._call_spray_set_param(spray_req)
    assert ok is False
    assert results == []
    assert msg == "Spray parameter call failed: transport down"


@pytest.mark.anyio
async def test_spray_param_single_rejection(spray_req):
    client = _FakeSprayParamClient(wait_result=True)

    async def _reject(_future, timeout):
        return _FakeSetParamResponse(
            [_FakeSetParamResult(False, reason="bad dash_on_distance_m")]
        )

    node = _spray_node(client, await_impl=_reject)
    ok, results, msg = await node._call_spray_set_param(spray_req)
    assert ok is False
    assert len(results) == 1
    assert msg == "bad dash_on_distance_m"


@pytest.mark.anyio
async def test_spray_param_multiple_rejections_joined(spray_req):
    client = _FakeSprayParamClient(wait_result=True)

    async def _reject_many(_future, timeout):
        return _FakeSetParamResponse(
            [
                _FakeSetParamResult(False, reason="bad on distance"),
                _FakeSetParamResult(True),
                _FakeSetParamResult(False, reason="bad phase reset"),
            ]
        )

    node = _spray_node(client, await_impl=_reject_many)
    ok, results, msg = await node._call_spray_set_param(spray_req)
    assert ok is False
    assert len(results) == 3
    assert msg == "bad on distance; bad phase reset"


@pytest.mark.anyio
async def test_spray_param_successful_bulk_apply(spray_req):
    client = _FakeSprayParamClient(wait_result=True)

    async def _ok(_future, timeout):
        return _FakeSetParamResponse(
            [
                _FakeSetParamResult(True),
                _FakeSetParamResult(True),
            ]
        )

    node = _spray_node(client, await_impl=_ok)
    ok, results, msg = await node._call_spray_set_param(spray_req)
    assert ok is True
    assert len(results) == 2
    assert msg == ""


class _Controller:
    state = MissionState.IDLE

    def __init__(self):
        self.loaded = None

    def load_path(self, points, **kwargs):
        self.loaded = (list(points), kwargs)


def _write_staged(tmp_path, mission_id, *, mode="continuous", extra=None):
    staged = {
        "mission_id": mission_id,
        "waypoints": [[0.0, 0.0], [1.0, 0.0]],
        "spray_flags": [True, True],
        "configuration_revision": 1,
        "path_fingerprint": f"fp-{mission_id}",
    }
    if mode is not None:
        staged["spray_mode"] = mode
    if mode == "dash":
        staged["dash_on_distance_m"] = 0.3
        staged["dash_off_distance_m"] = 0.3
    if extra:
        staged.update(extra)
    path = tmp_path / f"{mission_id}.json"
    path.write_text(json.dumps(staged), encoding="utf-8")
    return staged


class _FailingRos:
    def __init__(self, why: str):
        self.why = why

    async def set_spray_params_bulk_async(self, params):
        return False, [], self.why

    async def trigger_spray_apply_mission_config_async(self):
        return True, "ok"


@pytest.mark.anyio
@pytest.mark.parametrize(
    "mode,why",
    [
        ("dash", "Spray parameter response timed out after 8.0s"),
        ("point", "Spray parameter service unavailable after 2.0s"),
    ],
)
async def test_dependent_mode_load_503_before_controller_mutation(
    monkeypatch, tmp_path, mode, why
):
    mission_id = f"{mode}_blocked"
    extra = None
    if mode == "point":
        extra = {
            "point_mission_points": [
                {"north_m": 0.0, "east_m": 0.0, "dwell_s": 1.0, "source_index": 0}
            ],
            "point_source_frame": "LOCAL_NED",
        }
    _write_staged(tmp_path, mission_id, mode=mode, extra=extra)
    ctrl = _Controller()
    monkeypatch.setattr(path_routes, "STAGING_DIR", str(tmp_path))
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "ros_node", _FailingRos(why))
    monkeypatch.setattr(main, "point_mission", None)

    with pytest.raises(HTTPException) as exc:
        await path_routes.load_mission_to_controller(
            LoadMissionRequest(mission_id=mission_id)
        )
    assert exc.value.status_code == 503
    assert exc.value.detail == f"Spray controller dependency unavailable: {why}"
    assert ctrl.loaded is None


@pytest.mark.anyio
@pytest.mark.parametrize("mode", [None, "continuous"])
async def test_continuous_mode_degraded_fallback_unchanged(
    monkeypatch, tmp_path, mode
):
    mission_id = "legacy" if mode is None else "continuous"
    _write_staged(tmp_path, mission_id, mode=mode)
    ctrl = _Controller()
    monkeypatch.setattr(path_routes, "STAGING_DIR", str(tmp_path))
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(
        main,
        "ros_node",
        _FailingRos("Spray parameter response timed out after 8.0s"),
    )
    monkeypatch.setattr(main, "point_mission", None)

    response = await path_routes.load_mission_to_controller(
        LoadMissionRequest(mission_id=mission_id)
    )
    assert response["spray_config_applied"] is False
    assert (
        response["spray_config_degraded_reason"]
        == "Spray parameter response timed out after 8.0s"
    )
    assert ctrl.loaded is not None
    assert ctrl.loaded[1]["spray_flags"] == [False, False]


@pytest.mark.anyio
async def test_retry_succeeds_after_spray_service_recovers(monkeypatch, tmp_path):
    mission_id = "dash_retry"
    _write_staged(tmp_path, mission_id, mode="dash")
    ctrl = _Controller()

    class RecoveringRos:
        def __init__(self):
            self.attempts = 0

        async def set_spray_params_bulk_async(self, params):
            self.attempts += 1
            if self.attempts == 1:
                return False, [], "Spray parameter service unavailable after 2.0s"
            return True, [True] * len(params), ""

        async def trigger_spray_apply_mission_config_async(self):
            return True, "ok"

    ros = RecoveringRos()
    monkeypatch.setattr(path_routes, "STAGING_DIR", str(tmp_path))
    monkeypatch.setattr(main, "offboard_ctrl", ctrl)
    monkeypatch.setattr(main, "ros_node", ros)
    monkeypatch.setattr(main, "point_mission", None)

    with pytest.raises(HTTPException) as first_exc:
        await path_routes.load_mission_to_controller(
            LoadMissionRequest(mission_id=mission_id)
        )
    assert first_exc.value.status_code == 503
    assert ctrl.loaded is None

    response = await path_routes.load_mission_to_controller(
        LoadMissionRequest(mission_id=mission_id)
    )
    assert response["status"] == "success"
    assert response["spray_config_applied"] is True
    assert ctrl.loaded is not None


def test_list_paths_offloaded_from_event_loop():
    """GET /api/paths already runs path_mgr.list_paths() in a worker thread."""
    source = inspect.getsource(path_routes.list_paths)
    assert "asyncio.to_thread" in source
    assert "path_mgr.list_paths" in source


@pytest.mark.anyio
async def test_apply_spray_mission_config_success(monkeypatch):
    class Ros:
        async def set_spray_params_bulk_async(self, params):
            return True, [True] * len(params), ""

        async def trigger_spray_apply_mission_config_async(self):
            return True, "ok"

    ok, why, config = await apply_spray_mission_config(
        Ros(),
        {
            "mission_id": "dash-1",
            "spray_mode": "dash",
            "dash_on_distance_m": 0.3,
            "dash_off_distance_m": 0.3,
            "configuration_revision": 2,
            "path_fingerprint": "fp-dash-1",
        },
    )
    assert ok is True
    assert why == "applied"
    assert config is not None
    assert config.mode.value == "dash"