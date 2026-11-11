"""Serial link to the base controller (STM32). Thread-free: call poll() from your loop."""
from __future__ import annotations

import time

from . import protocol as P


class BaseDriver:
    def __init__(self, port: str = "/dev/ttyAMA0", baud: int = 115200, serial_obj=None):
        if serial_obj is None:
            import serial  # pyserial
            serial_obj = serial.Serial(port, baud, timeout=0)
        self.ser = serial_obj
        self.parser = P.FrameParser()
        self.seq = 0
        self.state: P.BaseState | None = None
        self.info: dict | None = None
        self.t_state = 0.0

    def _send(self, frame: bytes):
        self.ser.write(frame)
        self.seq = (self.seq + 1) & 0xFF

    def set_wheels(self, w_left: float, w_right: float):
        self._send(P.cmd_wheels(self.seq, w_left, w_right))

    def enable(self, on: bool = True):
        self._send(P.cmd_mode(self.seq, P.REQ_ENABLE if on else P.REQ_DISABLE))

    def clear_fault(self):
        self._send(P.cmd_mode(self.seq, P.REQ_CLEAR_FAULT))

    def request_info(self):
        self._send(P.cmd_mode(self.seq, P.REQ_INFO))

    def set_gains(self, kp: float, ki: float, kff: float):
        self._send(P.cmd_gains(self.seq, kp, ki, kff))

    def poll(self) -> P.BaseState | None:
        """Read pending bytes; returns the newest state frame received in this call, if any."""
        data = self.ser.read(4096)
        newest = None
        for mtype, _seq, payload in self.parser.feed(data or b""):
            if mtype == P.MSG_STATE and len(payload) == 37:
                newest = self.state = P.decode_state(payload)
                self.t_state = time.monotonic()
            elif mtype == P.MSG_INFO and len(payload) == 5:
                self.info = P.decode_info(payload)
        return newest

    def link_ok(self, max_age_s: float = 0.5) -> bool:
        return self.state is not None and (time.monotonic() - self.t_state) < max_age_s
