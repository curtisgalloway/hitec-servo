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
    looking for STX/ETX frames with the kRs3/kVs3 payload prefix.

    Older DPC-20 firmware responds with mux=11 and key kRs3.
    Newer AT32-based firmware (2024+) responds with mux=9 and key kVs3.
    Both are tried on every read.
    """

    _READ_TIMEOUT = (
        2.0  # seconds per read attempt (AT32 firmware can take >500 ms to reply)
    )
    _MAX_READ = 256

    # Response mux values to try, in preference order
    _RESPONSE_MUXES = (9, P.MUX_RESPONSE)
    _RESPONSE_KEYS = (P.KEY_VS3, P.KEY_RS3)

    def __init__(
        self,
        port: str,
        baud: int = 19200,
        four_pin: bool = False,
        series: str = "57",
    ) -> None:
        self._ser = serial.Serial(
            port, baud, timeout=self._READ_TIMEOUT, dsrdtr=False, rtscts=False
        )
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
            # Windows DPC sends KP3I00 twice before every read; the AT32
            # firmware requires recent heartbeats to stay in read mode.
            if expect_reply and self._new_firmware:
                self._send_message(P.MSG_KP3I)
                self._send_message(P.MSG_KP3I)
            self._write(frame)
            time.sleep(_INTER_PACKET_SLEEP)
            if not expect_reply:
                return b""
            raw = self._ser.read(self._MAX_READ)
            for mux in self._RESPONSE_MUXES:
                payload = P.parse_stxetx(raw, mux)
                if payload is not None and payload[:4] in self._RESPONSE_KEYS:
                    return payload[4:]  # strip key prefix; caller gets servo bytes
        return None

    def close(self) -> None:
        self._ser.close()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _write(self, data: bytes) -> None:
        self._ser.write(data)
        n_bits = len(data) * 11
        wait_us = n_bits * 1_000_000 // self._ser.baudrate + 1000
        time.sleep(wait_us / 1_000_000)

    def _send_message(self, msg: bytes) -> None:
        frame = P.stxetx_frame(msg, P.MUX_COMMAND)
        self._write(frame)

    def _connect(self) -> None:
        # Toggle DTR to wake the AT32 firmware (new DPC-20 hardware requires this).
        # C# SerialPort opens with DTR=False by default; pyserial opens with DTR=True,
        # so the firmware never sees the rising edge it needs unless we assert it here.
        # RTS is also cleared — raw tests confirmed this combination is required.
        self._ser.dtr = False
        self._ser.rts = False
        time.sleep(0.05)
        self._ser.dtr = True
        time.sleep(0.5)

        # Probe the adapter.
        # Windows DPC always sends KP3S immediately before KWAU; the AT32
        # firmware requires this pairing to produce a kAPP response.
        # Legacy firmware also accepts this sequence.
        # Try up to 30 cycles (Windows does the same) to give the adapter
        # time to initialise its servo bus after a cold start.
        probe_payload = None
        new_firmware = True
        for _ in range(30):
            self._send_message(P.MSG_3PIN)  # KP3S — mode hint before probe
            self._send_message(P.MSG_PROBE)  # KWAU — probe
            time.sleep(0.015)
            raw = self._ser.read(self._MAX_READ)
            for mux in (9, P.MUX_HANDSHAKE):
                probe_payload = P.parse_stxetx(raw, mux)
                if probe_payload:
                    new_firmware = mux == 9
                    break
            if probe_payload and probe_payload[:4] == P.KEY_APP:
                break  # adapter is ready

        self._new_firmware = new_firmware

        if probe_payload and probe_payload[:4] in (P.KEY_IBR, P.KEY_IBW, P.KEY_IBU):
            # Adapter is stuck in firmware-update mode; reset it
            self._ser.write(P.MSG_RESET)
            self._ser.dtr = False
            self._ser.close()
            time.sleep(0.5)
            self._ser.open()
            self._ser.dtr = True
            time.sleep(0.010)

        if new_firmware:
            # AT32 firmware: the KP3S+KWAU loop above already selected generic
            # 3-pin mode.  Just send KP3I00 to enter read mode.
            self._send_message(P.MSG_KP3I)
            time.sleep(0.05)
        else:
            # Legacy firmware: send the series-specific mode select and optional
            # zero-follow packet (required for 57-series and 9-series legacy).
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
