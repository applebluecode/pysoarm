"""The ROS 2 node, exercised without ROS by stubbing ``rclpy`` and the message packages.

The node is a separate ament package, so it is not on ``sys.path`` for the test run: it is
loaded from its file, with fake ``rclpy`` / ``sensor_msgs`` / ``std_msgs`` / ``std_srvs``
modules in place. That covers the wiring — parameters, endpoints, unit conversion, command
routing, stale-command rejection and torque services — which is exactly the part that cannot
be tested by ``FakeBus`` alone.
"""

from __future__ import annotations

import importlib.util
import math
import sys
import types
from pathlib import Path
from typing import Any

import pytest

from pysoarm import SOArm
from pysoarm.bus.fake import FakeBus

NODE_PATH = Path(__file__).parent.parent / "ros2" / "pysoarm_ros2" / "pysoarm_ros2" / "node.py"


# --------------------------------------------------------------------------- #
# Minimal ROS stand-ins
# --------------------------------------------------------------------------- #
class FakeLogger:
    """Records log calls instead of printing them."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, str]] = []

    def _record(self, level: str, message: str) -> None:
        self.calls.append((level, message))

    def info(self, message: str) -> None:
        self._record("info", message)

    def warn(self, message: str) -> None:
        self._record("warn", message)

    def error(self, message: str) -> None:
        self._record("error", message)

    def debug(self, message: str) -> None:
        self._record("debug", message)


class FakePublisher:
    def __init__(self) -> None:
        self.messages: list[Any] = []

    def publish(self, message: Any) -> None:
        self.messages.append(message)


class FakeSubscription:
    def __init__(self, callback: Any) -> None:
        self.callback = callback


class FakeService:
    def __init__(self, callback: Any) -> None:
        self.callback = callback


class FakeTimer:
    def __init__(self, period: float, callback: Any) -> None:
        self.period = period
        self.callback = callback


class FakeParameter:
    def __init__(self, value: Any) -> None:
        self.value = value


class FakeNode:
    """The subset of ``rclpy.node.Node`` the pysoarm node uses."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.parameters: dict[str, Any] = {}
        self.publishers: dict[str, FakePublisher] = {}
        self.subscriptions: dict[str, FakeSubscription] = {}
        self.services: dict[str, FakeService] = {}
        self.timers: list[FakeTimer] = []
        self.logger = FakeLogger()
        self.destroyed = False

    def declare_parameter(self, name: str, default: Any) -> FakeParameter:
        self.parameters[name] = default
        return FakeParameter(default)

    def create_publisher(self, message_type: Any, topic: str, qos: Any) -> FakePublisher:
        del message_type, qos
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(
        self, message_type: Any, topic: str, callback: Any, qos: Any
    ) -> FakeSubscription:
        del message_type, qos
        subscription = FakeSubscription(callback)
        self.subscriptions[topic] = subscription
        return subscription

    def create_service(self, service_type: Any, name: str, callback: Any) -> FakeService:
        del service_type
        service = FakeService(callback)
        self.services[name] = service
        return service

    def create_timer(self, period: float, callback: Any) -> FakeTimer:
        timer = FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def get_logger(self) -> FakeLogger:
        return self.logger

    def get_clock(self) -> Any:
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: "stamp"))

    def destroy_node(self) -> bool:
        self.destroyed = True
        return True


class Header:
    def __init__(self) -> None:
        self.stamp: Any = None
        self.frame_id = ""


class JointState:
    def __init__(self) -> None:
        self.header = Header()
        self.name: list[str] = []
        self.position: list[float] = []
        self.velocity: list[float] = []


class Float64MultiArray:
    def __init__(self, data: list[float] | None = None) -> None:
        self.data = data or []


class TriggerResponse:
    def __init__(self) -> None:
        self.success = False
        self.message = ""


class TriggerRequest:
    pass


class Trigger:
    Request = TriggerRequest
    Response = TriggerResponse


@pytest.fixture
def ros_modules(monkeypatch: pytest.MonkeyPatch) -> None:
    """Install the fake ROS modules for the duration of one test."""
    node_module: Any = types.ModuleType("rclpy.node")
    node_module.Node = FakeNode
    rclpy: Any = types.ModuleType("rclpy")
    rclpy.node = node_module
    rclpy.init = lambda args=None: None
    rclpy.spin = lambda node: None
    rclpy.try_shutdown = lambda: None

    sensor_msgs: Any = types.ModuleType("sensor_msgs")
    sensor_msgs_msg: Any = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = JointState
    sensor_msgs.msg = sensor_msgs_msg

    std_msgs: Any = types.ModuleType("std_msgs")
    std_msgs_msg: Any = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Float64MultiArray = Float64MultiArray
    std_msgs.msg = std_msgs_msg

    std_srvs: Any = types.ModuleType("std_srvs")
    std_srvs_srv: Any = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = Trigger
    std_srvs.srv = std_srvs_srv

    for name, module in {
        "rclpy": rclpy,
        "rclpy.node": node_module,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)


@pytest.fixture
def node_module(ros_modules: None) -> Any:
    """Load the node module from its file (it is not installed in this environment)."""
    spec = importlib.util.spec_from_file_location("pysoarm_ros2_node_under_test", NODE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_bus() -> FakeBus:
    bus = FakeBus()
    for mid in bus.motor_ids:
        bus.set_register(mid, "min_position_limit", 800)
        bus.set_register(mid, "max_position_limit", 3200)
    return bus


@pytest.fixture
def node(node_module: Any, monkeypatch: pytest.MonkeyPatch) -> Any:
    """An instance wired to an SOArm on the fake bus instead of a serial port."""
    bus = make_bus()

    def fake_arm(*args: Any, **kwargs: Any) -> SOArm:
        del args, kwargs
        return SOArm(bus=bus)

    monkeypatch.setattr(node_module, "SOArm", fake_arm)
    instance = node_module.SOArmNode()
    instance._test_bus = bus
    return instance


# --------------------------------------------------------------------------- #
# Tests
# --------------------------------------------------------------------------- #
def test_node_declares_parameters_endpoints_and_timer(node: Any) -> None:
    assert {
        "port",
        "model",
        "backend",
        "calibration",
        "rate_hz",
        "max_step_deg",
        "command_timeout_s",
        "frame_id",
    } <= set(node.parameters)
    assert "joint_states" in node.publishers
    assert "joint_commands" in node.subscriptions
    assert {"enable", "disable"} <= set(node.services)
    assert node.timers and node.timers[0].period == pytest.approx(1.0 / 50.0)


def test_timer_publishes_joint_states(node: Any) -> None:
    node.timers[0].callback()
    (message,) = node.publishers["joint_states"].messages
    assert isinstance(message, JointState)
    assert len(message.name) == 6
    assert message.name[0] == "shoulder_pan"


def test_timer_converts_degrees_to_radians(node: Any) -> None:
    frame = node._bridge.read(health=False)
    node.timers[0].callback()
    (message,) = node.publishers["joint_states"].messages
    for published, degrees in zip(message.position, frame.positions_deg, strict=True):
        assert published == pytest.approx(math.radians(degrees))


def test_command_topic_writes_a_goal(node: Any) -> None:
    node._bridge.read(health=False)
    before = node._test_bus.get_register(1, "goal_position")
    node.subscriptions["joint_commands"].callback(Float64MultiArray([0.0] * 6))
    after = node._test_bus.get_register(1, "goal_position")
    assert after != before
    assert not [call for call in node.logger.calls if call[0] == "error"]


def test_rejected_command_is_logged_not_raised(node: Any) -> None:
    node.subscriptions["joint_commands"].callback(Float64MultiArray([1.0, 2.0]))  # wrong size
    warns = [message for level, message in node.logger.calls if level == "warn"]
    assert any("command rejected" in message for message in warns)


def test_enable_and_disable_services(node: Any) -> None:
    node._test_bus.disable_torque()
    enable = Trigger.Response()
    enable = node.services["enable"].callback(Trigger.Request(), enable)
    assert enable.success is True
    assert node._test_bus.get_register(1, "torque_enable") == 1

    disable = Trigger.Response()
    disable = node.services["disable"].callback(Trigger.Request(), disable)
    assert disable.success is True
    assert node._test_bus.get_register(1, "torque_enable") == 0


def test_destroy_node_releases_torque(node: Any) -> None:
    node._test_bus.enable_torque()
    assert node.destroy_node() is True
    assert node._test_bus.get_register(1, "torque_enable") == 0
    assert node.destroyed


def test_main_starts_and_stops(node_module: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    bus = make_bus()

    def fake_arm(*args: Any, **kwargs: Any) -> SOArm:
        del args, kwargs
        return SOArm(bus=bus)

    monkeypatch.setattr(node_module, "SOArm", fake_arm)
    node_module.main([])  # stub rclpy: init -> construct -> spin -> destroy
    assert bus.get_register(1, "torque_enable") == 0  # released on shutdown
