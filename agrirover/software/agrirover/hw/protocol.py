"""Python mirror of firmware/core/protocol.[ch] (COBS + CRC-16/CCITT-FALSE framing)."""
from __future__ import annotations

import struct
from dataclasses import dataclass

MSG_CMD_WHEELS, MSG_CMD_MODE, MSG_CMD_GAINS, MSG_STATE, MSG_INFO = 0x01, 0x02, 0x03, 0x81, 0x82
REQ_DISABLE, REQ_ENABLE, REQ_CLEAR_FAULT, REQ_INFO = 0, 1, 2, 3
STATE_NAMES = ["INIT", "IDLE", "ACTIVE", "STOPPING", "ESTOP", "FAULT"]
FAULT_BITS = {0: "ESTOP", 1: "BUMPER", 2: "OVERCURRENT", 3: "DRIVER", 4: "LOW_BATTERY", 5: "LINK_LOST", 6: "RAIL"}
_STATE = struct.Struct("<IiifffhhHhHBH")
_INFO = struct.Struct("<BBBH")


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out, block = bytearray(), bytearray()
    for b in data:
        if b == 0:
            out.append(len(block) + 1)
            out += block
            block = bytearray()
        else:
            block.append(b)
            if len(block) == 254:
                out.append(255)
                out += block
                block = bytearray()
    out.append(len(block) + 1)
    out += block
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out, i = bytearray(), 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            raise ValueError("bad COBS data")
        out += data[i + 1:i + code]
        i += code
        if code != 255 and i < len(data):
            out.append(0)
    return bytes(out)


def encode_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    raw = bytes([msg_type, seq & 0xFF]) + payload
    return cobs_encode(raw + struct.pack("<H", crc16(raw))) + b"\x00"


def cmd_wheels(seq: int, w_left: float, w_right: float) -> bytes:
    return encode_frame(MSG_CMD_WHEELS, seq, struct.pack("<ff", w_left, w_right))


def cmd_mode(seq: int, request: int) -> bytes:
    return encode_frame(MSG_CMD_MODE, seq, bytes([request]))


def cmd_gains(seq: int, kp: float, ki: float, kff: float) -> bytes:
    return encode_frame(MSG_CMD_GAINS, seq, struct.pack("<fff", kp, ki, kff))


@dataclass
class BaseState:
    t_ms: int
    enc_left: int
    enc_right: int
    w_left: float
    w_right: float
    gyro_z: float
    i_left_ma: int
    i_right_ma: int
    vbat_mv: int
    ibat_ma: int
    vmot_mv: int
    state: int
    faults: int

    @property
    def state_name(self) -> str:
        return STATE_NAMES[self.state] if self.state < len(STATE_NAMES) else f"?{self.state}"

    @property
    def fault_names(self) -> list[str]:
        return [n for b, n in FAULT_BITS.items() if self.faults & (1 << b)]


def decode_state(payload: bytes) -> BaseState:
    return BaseState(*_STATE.unpack(payload))


def decode_info(payload: bytes) -> dict:
    v, major, minor, cpr = _INFO.unpack(payload)
    return dict(proto_version=v, fw=f"{major}.{minor}", enc_cpr=cpr)


class FrameParser:
    """Incremental parser: feed bytes, get (type, seq, payload) tuples."""

    def __init__(self):
        self._buf = bytearray()
        self.rx_ok = self.rx_err = 0

    def feed(self, data: bytes):
        for b in data:
            if b != 0:
                if len(self._buf) < 256:
                    self._buf.append(b)
                continue
            chunk, self._buf = bytes(self._buf), bytearray()
            if not chunk:
                continue
            try:
                raw = cobs_decode(chunk)
                if len(raw) < 4 or struct.unpack("<H", raw[-2:])[0] != crc16(raw[:-2]):
                    raise ValueError("crc")
            except ValueError:
                self.rx_err += 1
                continue
            self.rx_ok += 1
            yield raw[0], raw[1], raw[2:-2]
