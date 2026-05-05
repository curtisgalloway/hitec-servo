# Copyright 2025 Curtis Galloway
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Serial transport layer for Hitec DPC programmers.

Two concrete transports:

  RawTransport  — DPC-11 (CP210x): raw 4-byte packets at 19200 baud.
                  The CP210x appears as a standard serial port on macOS/Linux.

  DPC20Transport — DPC-20 / DPC-485: STX/ETX framed packets over any COM port.
                   Handles the KSO3 wrapper, handshake, and kRs3 response parsing.
"""

from __future__ import annotations

import time
from abc import ABC, abstractmethod

import serial

from . import protocol as P

_RETRIES = 3
_INTER_PACKET_SLEEP = 0.005  # 5 ms between retransmissions


class Transport(ABC):
    @abstractmethod
    def send(self, packet: bytes, expect_reply: bool = True) -> bytes | None:
        """Send a 4-byte servo packet; return raw response bytes or None on timeout."""

    @abstractmethod
    def close(self) -> None: ...

    def __enter__(self) -> "Transport":
        return self

    def __exit__(self, *_) -> None:
        self.close()


class RawTransport(Transport):
    """
    DPC-11 mode: raw 4-byte command packets over the CP210x serial port.

    The servo's response format in raw mode is not fully confirmed without
    hardware. Based on the source, return_p[4] holds the 8-bit read value and
    return_p[4:6] holds 16-bit values. Bytes 0-3 are assumed to be a header
    (possibly the kRs3 ASCII key, matching the DPC-20 response layout).
    """

    _RESPONSE_LEN = 6

    def __init__(self, port: str, baud: int = 19200, timeout: float = 0.1) -> None:
        self._ser = serial.Serial(port, baud, timeout=timeout)

    def send(self, packet: bytes, expect_reply: bool = True) -> bytes | None:
        for _ in range(_RETRIES):
            self._ser.reset_input_buffer()
            self._ser.write(packet)
            time.sleep(_INTER_PACKET_SLEEP)
            if not expect_reply:
                return b""
            resp = self._ser.read(self._RESPONSE_LEN)
            if len(resp) >= 5:
                return resp
        return None

    def close(self) -> None:
        self._ser.close()


class DPC20Transport(Transport):
    """
    DPC-20 / DPC-485 mode: STX/ETX framed packets over a standard COM port.

    Sends commands wrapped in the KSO3 outer frame (mux=7); reads responses
    looking for STX/ETX frames with the kRs3 payload prefix (mux=11).
    """

    _READ_TIMEOUT = 0.1   # seconds per read attempt
    _MAX_READ     = 256

    def __init__(
        self,
        port: str,
        baud: int = 19200,
        four_pin: bool = False,
        series: str = "57",
    ) -> None:
        self._ser = serial.Serial(port, baud, timeout=self._READ_TIMEOUT)
        self._four_pin = four_pin
        self._series = series
        self._connect()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def send(self, packet: bytes, expect_reply: bool = True) -> bytes | None:
        inner = P.kso_wrapper(packet, four_pin=self._four_pin, want_reply=expect_reply)
        frame = P.stxetx_frame(inner, P.MUX_COMMAND)
        for _ in range(_RETRIES):
            self._ser.reset_input_buffer()
            self._write(frame)
            time.sleep(_INTER_PACKET_SLEEP)
            if not expect_reply:
                return b""
            raw = self._ser.read(self._MAX_READ)
            payload = P.parse_stxetx(raw, P.MUX_RESPONSE)
            if payload is not None and payload[:4] == P.KEY_RS3:
                return payload  # caller extracts data at offsets 4 and 5
        return None

    def close(self) -> None:
        self._ser.close()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _write(self, data: bytes) -> None:
        for b in data:
            self._ser.write(bytes([b]))
        n_bits = len(data) * 11
        wait_us = n_bits * 1_000_000 // self._ser.baudrate + 1000
        time.sleep(wait_us / 1_000_000)

    def _send_message(self, msg: bytes) -> None:
        frame = P.stxetx_frame(msg, P.MUX_COMMAND)
        self._write(frame)

    def _connect(self) -> None:
        # Probe the adapter
        self._send_message(P.MSG_PROBE)
        time.sleep(0.010)
        raw = self._ser.read(self._MAX_READ)

        payload = P.parse_stxetx(raw, P.MUX_HANDSHAKE)
        if payload and payload[:4] in (P.KEY_IBR, P.KEY_IBW, P.KEY_IBU):
            # Adapter is stuck in firmware-update mode; reset it
            self._ser.write(P.MSG_RESET)
            self._ser.dtr = False
            self._ser.close()
            time.sleep(0.5)
            self._ser.open()
            self._ser.dtr = True
            time.sleep(0.010)

        # Select pin/series mode
        if self._four_pin:
            mode_msg = P.MSG_4PIN
            zero_follow = False
        elif self._series == "9":
            mode_msg = P.MSG_3PIN_9
            zero_follow = True
        elif self._series == "57":
            mode_msg = P.MSG_3PIN_57
            zero_follow = True
        else:
            mode_msg = P.MSG_3PIN
            zero_follow = False

        self._send_message(mode_msg)
        if zero_follow:
            zero_pkt = P.stxetx_frame(
                P.kso_wrapper(bytes([0x00]), four_pin=False, want_reply=False),
                P.MUX_COMMAND,
            )
            self._write(zero_pkt)
