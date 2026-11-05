"""SCS wire protocol: frame layout, checksums, retries and the sync-write broadcast.

A wire-level emulator stands in for the servos, so the transport is tested end to end without
hardware: the bus builds frames, the emulator parses them, answers with real SCS packets, and
the bus decodes the replies.
"""

from __future__ import annotations

from typing import Any

import pytest

from pysoarm.bus.fake import FakeBus
from pysoarm.bus.registers import REGISTERS
from pysoarm.bus.scs import ScsBus
from pysoarm.exceptions import BusConnectionError, CommunicationError, ConfigurationError

HEADER = b"\xff\xff"
INSTR_PING = 0x01
INSTR_READ = 0x02
INSTR_WRITE = 0x03
INSTR_SYNC_WRITE = 0x83
BY_ADDR = {register.addr: register for register in REGISTERS.values()}


def checksum(body: bytes) -> int:
    return ~sum(body) & 0xFF


class FakeSerialPeer:
    """Emulates one or more STS3215 servos on the wire (mirrors serial + protocol)."""

    """Emulates one or more STS3215 servos on the wire."""

    def __init__(
        self, motor_ids: tuple[int, ...] = (1, 2, 3, 4, 5, 6), *, model_number: int = 777
    ) -> None:
        self.inner = FakeBus(motor_ids, model_number=model_number)
        self.inner.open()
        self.requests: list[bytes] = []
        self.is_open = True
        self.silent = False
        self.drop_reads = 0
        self.corrupt_checksum = False
        self.force_status = 0
        self.drop_ids: set[int] = set()
        self.reply_id_override: int | None = None
        self.bad_len = False
        self.truncate = False
        self.short_data = False
        self.prefix_garbage = b""
        self.collide_on_pending_reply = True
        self._out = b""

    # -- serial.Serial subset --------------------------------------------- #
    def write(self, data: bytes) -> int:
        self.requests.append(bytes(data))
        self._handle(data)
        return len(data)

    def read(self, size: int = 1) -> bytes:
        chunk, self._out = self._out[:size], self._out[size:]
        return chunk

    def reset_input_buffer(self) -> None:
        self._out = b""

    def close(self) -> None:
        self.is_open = False

    # -- protocol emulation ------------------------------------------------ #
    def _reply(self, motor_id: int, data: bytes) -> None:
        if self.collide_on_pending_reply and self._out:
            # half-duplex: transmitting while a reply is still on the wire corrupts both
            self._out = b"\xff\x00collision"
            return
        if self.short_data and data:
            data = data[:1]
        reply_id = motor_id if self.reply_id_override is None else self.reply_id_override
        length = 1 if self.bad_len else len(data) + 2
        body = bytes([reply_id, length, self.force_status]) + data
        frame = HEADER + body + bytes([checksum(body)])
        if self.corrupt_checksum:
            frame = frame[:-1] + bytes([frame[-1] ^ 0xFF])
        if self.truncate:
            frame = HEADER
        self._out += self.prefix_garbage + frame

    def _handle(self, frame: bytes) -> None:  # noqa: PLR0911
        if not frame.startswith(HEADER) or len(frame) < 6:
            return
        motor_id, length, instruction = frame[2], frame[3], frame[4]
        params, received = frame[5:-1], frame[-1]
        if (sum(frame[2:-1]) + received) & 0xFF != 0xFF:
            return  # corrupt request: stay silent, like real hardware
        if self.silent:
            return
        if instruction == INSTR_SYNC_WRITE:
            register = BY_ADDR[params[0]]
            size = params[1]
            payload = params[2:]
            for index in range(0, len(payload), size + 1):
                target = payload[index]
                raw = payload[index + 1 : index + 1 + size]
                self.inner.write(target, register.name, register.decode(raw))
            return
        if motor_id not in self.inner.motor_ids:
            return
        if instruction == INSTR_READ:
            if motor_id in self.drop_ids:
                return
            if self.drop_reads > 0:
                self.drop_reads -= 1
                return
            register = BY_ADDR[params[0]]
            value = self.inner.read(motor_id, register.name)
            self._reply(motor_id, register.encode(value))
        elif instruction == INSTR_WRITE:
            register = BY_ADDR[params[0]]
            self.inner.write(motor_id, register.name, register.decode(params[1:]))
            self._reply(motor_id, b"")
        elif instruction == INSTR_PING:
            self._reply(motor_id, b"")
        del length


def make_bus(peer: FakeSerialPeer, **kwargs: Any) -> ScsBus:
    defaults: dict[str, Any] = {"timeout_s": 0.01, "retries": 0}
    return ScsBus(
        "/dev/fake",
        (1, 2, 3, 4, 5, 6),
        serial_factory=lambda *args, **kw: peer,
        **{**defaults, **kwargs},
    )


def test_read_builds_the_documented_frame() -> None:
    peer = FakeSerialPeer()
    bus = make_bus(peer)
    bus.open()
    assert bus.read(1, "present_position") == 2048
    (request,) = peer.requests
    # FF FF | ID | LEN = params + 2 | READ | addr | size | checksum
    assert request[:5] == bytes([0xFF, 0xFF, 0x01, 0x04, INSTR_READ])
    assert request[5:7] == bytes([0x38, 0x02])
    assert request[7] == checksum(request[2:7])


def test_write_is_acknowledged_and_applied() -> None:
    peer = FakeSerialPeer()
    bus = make_bus(peer)
    bus.open()
    bus.write(1, "goal_position", 1234)
    assert peer.inner.get_register(1, "goal_position") == 1234
    (request,) = peer.requests
    assert request[4] == INSTR_WRITE
    assert request[3] == len(request[5:-1]) + 2


def test_sync_read_sends_one_request_per_motor_and_collects_all() -> None:
    peer = FakeSerialPeer()
    bus = make_bus(peer)
    bus.open()
    values = bus.sync_read((1, 3, 5), "present_position")
    assert values == {1: 2048, 3: 2048, 5: 2048}
    assert len(peer.requests) == 3


def test_sync_write_is_a_single_broadcast_frame() -> None:
    peer = FakeSerialPeer()
    peer.inner.sync_write("goal_position", dict.fromkeys(peer.inner.motor_ids, 2048))
    bus = make_bus(peer)
    bus.open()
    peer.requests.clear()
    goals = {motor_id: 1000 + motor_id for motor_id in peer.inner.motor_ids}
    bus.sync_write("goal_position", goals)
    (request,) = peer.requests
    assert request[2] == 0xFE  # broadcast id
    assert request[4] == INSTR_SYNC_WRITE
    # LEN = 2 params (addr, size) + 2 + n * (1 id + 2 data)
    assert request[3] == 4 + len(goals) * 3
    for motor_id, goal in goals.items():
        assert peer.inner.get_register(motor_id, "goal_position") == goal


def test_retry_recovers_from_one_lost_reply() -> None:
    peer = FakeSerialPeer()
    peer.drop_reads = 1
    bus = make_bus(peer, retries=2)
    bus.open()
    assert bus.read(1, "present_position") == 2048
    assert len(peer.requests) == 2


def test_retries_exhausted_reports_no_reply() -> None:
    peer = FakeSerialPeer()
    peer.drop_reads = 10
    bus = make_bus(peer, retries=1)
    bus.open()
    with pytest.raises(CommunicationError, match="No reply from motor 1"):
        bus.read(1, "present_position")
    assert len(peer.requests) == 2


def test_silent_bus_times_out_quickly() -> None:
    peer = FakeSerialPeer()
    peer.silent = True
    bus = make_bus(peer, retries=0, timeout_s=0.01)
    bus.open()
    with pytest.raises(CommunicationError, match="No reply"):
        bus.sync_read((1, 2), "present_position")


def test_corrupt_checksum_is_detected() -> None:
    peer = FakeSerialPeer()
    peer.corrupt_checksum = True
    bus = make_bus(peer)
    bus.open()
    with pytest.raises(CommunicationError, match="Checksum mismatch"):
        bus.read(1, "present_position")


def test_status_error_bits_are_surfaced() -> None:
    peer = FakeSerialPeer()
    peer.force_status = 0x08
    bus = make_bus(peer)
    bus.open()
    with pytest.raises(CommunicationError, match="status 0x08"):
        bus.read(1, "present_position")


def test_scan_reports_model_numbers() -> None:
    peer = FakeSerialPeer()
    bus = make_bus(peer)
    bus.open()
    assert bus.scan() == dict.fromkeys((1, 2, 3, 4, 5, 6), 777)
    assert bus.scan((2, 9)) == {2: 777}


def test_read_only_register_is_rejected_before_touching_the_bus() -> None:
    peer = FakeSerialPeer()
    bus = make_bus(peer)
    bus.open()
    with pytest.raises(ConfigurationError, match="read-only"):
        bus.write(1, "present_position", 1)
    assert peer.requests == []


def test_double_open_and_factory_failure_are_connection_errors() -> None:
    peer = FakeSerialPeer()
    bus = make_bus(peer)
    bus.open()
    with pytest.raises(BusConnectionError, match="already open"):
        bus.open()
    bus.close()
    bus.close()  # idempotent
    assert not bus.is_open

    def exploding_factory(*args: Any, **kwargs: Any) -> FakeSerialPeer:
        raise OSError("Device or resource busy")

    failing = ScsBus("/dev/fake", serial_factory=exploding_factory)
    with pytest.raises(BusConnectionError, match="Device or resource busy"):
        failing.open()


def test_reply_from_the_wrong_motor_is_rejected() -> None:
    peer = FakeSerialPeer()
    peer.reply_id_override = 5
    bus = make_bus(peer)
    bus.open()
    with pytest.raises(CommunicationError, match="Reply from motor 5"):
        bus.read(1, "present_position")


def test_invalid_length_byte_is_rejected() -> None:
    peer = FakeSerialPeer()
    peer.bad_len = True
    bus = make_bus(peer)
    bus.open()
    with pytest.raises(CommunicationError, match="Invalid LEN=1"):
        bus.read(1, "present_position")


def test_truncated_reply_is_reported() -> None:
    peer = FakeSerialPeer()
    peer.truncate = True
    bus = make_bus(peer)
    bus.open()
    with pytest.raises(CommunicationError, match="Truncated status packet"):
        bus.read(1, "present_position")


def test_short_payload_is_reported() -> None:
    peer = FakeSerialPeer()
    peer.short_data = True
    bus = make_bus(peer)
    bus.open()
    with pytest.raises(CommunicationError, match="returned 1 byte"):
        bus.read(1, "present_position")


def test_receiver_resynchronises_after_garbage() -> None:
    peer = FakeSerialPeer()
    peer.prefix_garbage = b"\x00\x13noise"
    bus = make_bus(peer)
    bus.open()
    assert bus.read(1, "present_position") == 2048


def test_sync_read_reports_the_missing_motors() -> None:
    peer = FakeSerialPeer()
    peer.drop_ids = {3}
    bus = make_bus(peer, retries=1)
    bus.open()
    with pytest.raises(CommunicationError, match=r"No reply from motor\(s\) \[3\]"):
        bus.sync_read((1, 2, 3), "present_position")


def test_status_errors_carry_structured_fields_and_are_retried() -> None:
    peer = FakeSerialPeer()
    peer.force_status = 0x08
    bus = make_bus(peer, retries=2)
    bus.open()
    with pytest.raises(CommunicationError) as excinfo:
        bus.read(1, "present_position")
    error = excinfo.value
    assert error.status == 0x08
    assert error.flags == frozenset({"overcurrent"})
    assert error.transient is True
    assert error.attempts == 3  # one initial attempt + two retries
    assert len(peer.requests) == 3


def test_per_call_retry_override_is_honoured() -> None:
    peer = FakeSerialPeer()
    peer.drop_reads = 1
    bus = make_bus(peer, retries=0)
    bus.open()
    # The bus defaults to no retries, but the hot path asks for two attempts.
    assert bus.read(1, "present_position", retries=1) == 2048
    assert len(peer.requests) == 2
