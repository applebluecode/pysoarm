"""The optional ``rustypot`` backend, exercised against a stub of the pyo3 module."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import pytest

import pysoarm.bus as bus_pkg
from pysoarm.bus import make_bus
from pysoarm.bus.fake import FakeBus
from pysoarm.bus.rustypot import RustypotBus, require_rustypot
from pysoarm.bus.scs import ScsBus
from pysoarm.exceptions import BusConnectionError, CommunicationError, ConfigurationError

STUB_MODEL_NUMBER = 777
STUB_DEFINITION = object()


class StubRustypotBus:
    """Mimics ``rustypot.Bus`` on top of :class:`~pysoarm.bus.fake.FakeBus`."""

    def __init__(
        self, port: str, baudrate: int, timeout_s: float, definitions: Mapping[int, Any]
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.definitions = definitions
        self.inner = FakeBus(tuple(definitions))
        self.inner.open()
        self.is_open = True
        self.calls: list[tuple[object, ...]] = []
        self.status: dict[int, int] = {}
        self.raise_on_next: str | None = None

    def _maybe_raise(self) -> None:
        if self.raise_on_next is not None:
            message, self.raise_on_next = self.raise_on_next, None
            raise RuntimeError(message)

    def read_register_with_error(self, mid: int, name: str, retries: int = 0) -> tuple[int, int]:
        self.calls.append(("read", mid, name, retries))
        self._maybe_raise()
        return self.inner.read(mid, name), self.status.get(mid, 0)

    def write_register(self, mid: int, name: str, value: int, retries: int = 0) -> None:
        self.calls.append(("write", mid, name, value, retries))
        self._maybe_raise()
        self.inner.write(mid, name, value)

    def sync_read_register_with_error(
        self, ids: Sequence[int], name: str, retries: int = 0
    ) -> list[tuple[int, int]]:
        self.calls.append(("sync_read", tuple(ids), name, retries))
        self._maybe_raise()
        return [(self.inner.read(mid, name), self.status.get(mid, 0)) for mid in ids]

    def sync_write_register(
        self, ids: Sequence[int], name: str, values: Sequence[int], retries: int = 0
    ) -> None:
        self.calls.append(("sync_write", tuple(ids), name, tuple(values), retries))
        self._maybe_raise()
        for mid, value in zip(ids, values, strict=True):
            self.inner.write(mid, name, value)

    def set_torque(self, ids: Sequence[int], enabled: bool, retries: int = 0) -> dict[int, str]:
        self.calls.append(("torque", tuple(ids), enabled, retries))
        self._maybe_raise()
        for mid in ids:
            self.inner.write(mid, "torque_enable", 1 if enabled else 0)
        return {}

    def scan(self, definition: Any, ids: Sequence[int] | None = None) -> dict[int, int]:
        self.calls.append(("scan", definition, tuple(ids or ())))
        return self.inner.scan(ids)

    def close(self) -> None:
        self.is_open = False


class StubModule:
    """Stands in for the ``rustypot`` module."""

    def __init__(self) -> None:
        self.buses: list[StubRustypotBus] = []

    def find_model(self, name: str) -> tuple[int, Any] | None:
        return (STUB_MODEL_NUMBER, STUB_DEFINITION) if name == "sts3215" else None

    def Bus(  # noqa: N802 - mirrors the pyo3 API name
        self,
        port: str,
        baudrate: int,
        timeout_s: float,
        definitions: Mapping[int, Any],
    ) -> StubRustypotBus:
        bus = StubRustypotBus(port, baudrate, timeout_s, definitions)
        self.buses.append(bus)
        return bus


@pytest.fixture
def stub() -> StubModule:
    return StubModule()


@pytest.fixture
def bus(stub: StubModule) -> RustypotBus:
    bus = RustypotBus("/dev/fake", rustypot_module=stub)
    bus.open()
    return bus


def test_read_and_write_go_through_rustypot(bus: RustypotBus, stub: StubModule) -> None:
    assert bus.read(1, "present_position") == 2048
    bus.write(1, "goal_position", 1234)
    inner = stub.buses[0]
    assert inner.inner.get_register(1, "goal_position") == 1234
    assert [call[0] for call in inner.calls] == ["read", "write"]


def test_retries_happen_at_the_adapter_layer(bus: RustypotBus, stub: StubModule) -> None:
    stub.buses[0].raise_on_next = "Operation timed out"
    assert bus.read(1, "present_position") == 2048  # retried once, then succeeded
    reads = [call for call in stub.buses[0].calls if call[0] == "read"]
    assert len(reads) == 2
    # rustypot's own retries stay disabled so the attempt count is not squared
    assert all(call[-1] == 0 for call in reads)


def test_sync_read_returns_a_mapping_in_id_order(bus: RustypotBus) -> None:
    assert bus.sync_read((3, 1, 2), "present_position") == {3: 2048, 1: 2048, 2: 2048}


def test_sync_write_writes_every_value(bus: RustypotBus, stub: StubModule) -> None:
    bus.sync_write("goal_position", {1: 1500, 2: 1600})
    assert stub.buses[0].inner.sync_read((1, 2), "present_position") == {1: 1500, 2: 1600}


def test_torque_uses_rustypot_set_torque(bus: RustypotBus, stub: StubModule) -> None:
    bus.disable_torque()
    assert stub.buses[0].inner.get_register(1, "torque_enable") == 0
    bus.enable_torque((1, 2))
    assert stub.buses[0].inner.get_register(1, "torque_enable") == 1


def test_scan_reports_model_numbers(bus: RustypotBus) -> None:
    assert bus.scan() == dict.fromkeys(range(1, 7), STUB_MODEL_NUMBER)


def test_read_only_registers_are_rejected_locally(bus: RustypotBus, stub: StubModule) -> None:
    with pytest.raises(ConfigurationError, match="read-only"):
        bus.write(1, "present_position", 1)
    assert stub.buses[0].inner.get_register(1, "present_position") == 2048


def test_unknown_ids_and_registers_are_configuration_errors(bus: RustypotBus) -> None:
    with pytest.raises(ConfigurationError, match="not part of this bus"):
        bus.read(99, "present_position")
    with pytest.raises(ConfigurationError, match="Unknown register"):
        bus.read(1, "no_such_register")


def test_status_error_bits_become_communication_errors(bus: RustypotBus, stub: StubModule) -> None:
    stub.buses[0].status[2] = 0x08
    with pytest.raises(CommunicationError, match="status 0x08"):
        bus.read(2, "present_position")
    with pytest.raises(CommunicationError, match="status 0x08"):
        bus.sync_read((1, 2), "present_position")


def test_bus_failures_become_communication_errors(stub: StubModule) -> None:
    bus = RustypotBus("/dev/fake", retries=0, rustypot_module=stub)
    bus.open()
    stub.buses[0].raise_on_next = "Operation timed out"
    with pytest.raises(CommunicationError, match="Operation timed out"):
        bus.read(1, "present_position")


def test_open_failure_is_a_connection_error(stub: StubModule) -> None:
    def exploding(*args: Any, **kwargs: Any) -> Any:
        raise OSError("Device or resource busy")

    stub.Bus = exploding  # type: ignore[method-assign]
    bus = RustypotBus("/dev/fake", rustypot_module=stub)
    with pytest.raises(BusConnectionError, match="Device or resource busy"):
        bus.open()
    assert not bus.is_open


def test_double_open_and_idempotent_close(bus: RustypotBus) -> None:
    with pytest.raises(BusConnectionError, match="already open"):
        bus.open()
    bus.close()
    bus.close()
    assert not bus.is_open


def test_missing_dependency_reports_the_install_hint(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def absent() -> Any:
        raise ModuleNotFoundError("No module named 'rustypot'")

    monkeypatch.setattr("pysoarm.bus.rustypot._import_rustypot", absent)
    with pytest.raises(ConfigurationError, match=r"pysoarm\[rustypot\]"):
        require_rustypot()


def test_unknown_servo_model_is_rejected(stub: StubModule) -> None:
    with pytest.raises(ConfigurationError, match="no servo model named"):
        RustypotBus("/dev/fake", servo_model="sts9999", rustypot_module=stub)


def test_factory_selects_the_backend(monkeypatch: pytest.MonkeyPatch) -> None:
    scs = make_bus("/dev/fake", (1, 2), backend="scs")
    assert isinstance(scs, ScsBus)
    with pytest.raises(ConfigurationError, match="Unknown backend"):
        make_bus("/dev/fake", (1, 2), backend="dynamixel")  # type: ignore[arg-type]


def test_factory_auto_falls_back_to_scs(monkeypatch: pytest.MonkeyPatch, stub: StubModule) -> None:
    monkeypatch.setattr(bus_pkg, "_rustypot_available", lambda: False)
    assert isinstance(make_bus("/dev/fake", (1, 2), backend="auto"), ScsBus)

    monkeypatch.setattr(bus_pkg, "_rustypot_available", lambda: True)
    monkeypatch.setattr("pysoarm.bus.rustypot.require_rustypot", lambda module=None: stub)
    assert isinstance(make_bus("/dev/fake", (1, 2), backend="auto"), RustypotBus)


def test_status_errors_are_retried_like_the_scs_backend(bus: RustypotBus, stub: StubModule) -> None:
    stub.buses[0].status[2] = 0x04
    with pytest.raises(CommunicationError) as excinfo:
        bus.read(2, "present_position")
    error = excinfo.value
    assert error.status == 0x04
    assert error.flags == frozenset({"overheat"})
    assert error.attempts == 3  # the default retries=2 applies to status alarms too
    assert len([call for call in stub.buses[0].calls if call[0] == "read"]) == 3


def test_status_retry_can_be_disabled(stub: StubModule) -> None:
    bus = RustypotBus("/dev/fake", retries=2, retry_status=False, rustypot_module=stub)
    bus.open()
    stub.buses[0].status[2] = 0x04
    with pytest.raises(CommunicationError):
        bus.read(2, "present_position")
    assert len([call for call in stub.buses[0].calls if call[0] == "read"]) == 1


def test_per_call_retry_override_is_honoured(stub: StubModule) -> None:
    bus = RustypotBus("/dev/fake", retries=0, rustypot_module=stub)
    bus.open()
    stub.buses[0].raise_on_next = "Operation timed out"
    assert bus.read(1, "present_position", retries=1) == 2048
    assert len([call for call in stub.buses[0].calls if call[0] == "read"]) == 2
