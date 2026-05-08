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

"""High-level HitecServo interface."""

from __future__ import annotations

from . import protocol as P
from . import registers as R
from .transport import DPC20Transport, RawTransport, Transport


class HitecServo:
    """
    Communicate with a Hitec DPC-programmable servo.

    Parameters
    ----------
    port:
        Serial port path (e.g. ``/dev/ttyUSB0``, ``/dev/cu.usbserial-XXXX``).
    baud:
        Baud rate. Default 19200 matches the DPC-11 USB adapter.
    mode:
        ``"raw"``   — DPC-11 (CP210x): 4-byte raw packets, no framing.
        ``"dpc20"`` — DPC-20 / DPC-485: STX/ETX framed packets.
    series:
        ``"57"`` (HS-5/7XXX, default) or ``"9"`` (HSB-9XXX).
        Only relevant for DPC-20 mode; selects the KP3Sx handshake message.
    four_pin:
        True for 4-pin DPC-20 adapters (selects KP4S mode).

    Usage
    -----
    ::

        with HitecServo("/dev/ttyUSB0") as servo:
            params = servo.read_all()
            servo.write_register("deadband", 5)
            print(servo.get_model())
    """

    def __init__(
        self,
        port: str,
        baud: int = 19200,
        mode: str = "raw",
        series: str = "57",
        four_pin: bool = False,
    ) -> None:
        if mode == "raw":
            self._transport: Transport = RawTransport(port, baud)
        elif mode == "dpc20":
            self._transport = DPC20Transport(
                port, baud, four_pin=four_pin, series=series
            )
        else:
            raise ValueError(f"Unknown mode {mode!r}; choose 'raw' or 'dpc20'")
        self._mode = mode

    # ------------------------------------------------------------------
    # Register access
    # ------------------------------------------------------------------

    def read_register(self, name: str) -> int:
        """
        Read a named register and return its raw integer value.

        For 16-bit registers the value is assembled big-endian from two
        consecutive 8-bit reads. To get engineering units multiply by
        ``registers.get(name).scale``.
        """
        reg = R.get(name)
        if reg.is_16bit:
            hi = self._read8(reg.sram_addr)
            lo = self._read8(reg.sram_addr + 1)
            return (hi << 8) | lo
        return self._read8(reg.sram_addr)

    def write_register(self, name: str, value: int, persist: bool = True) -> None:
        """
        Write *value* to a named register.

        Parameters
        ----------
        persist:
            If True (default) and the register has an EEPROM mirror, the value
            is also written to the EEPROM address so it survives power cycling.
            Set to False for transient changes.
        """
        reg = R.get(name)
        if reg.is_16bit:
            hi = (value >> 8) & 0xFF
            lo = value & 0xFF
            self._write_sram(reg.sram_addr, hi)
            self._write_sram(reg.sram_addr + 1, lo)
            if persist and reg.ee_addr is not None:
                self._write_ee(reg.ee_addr, hi)
                self._write_ee(reg.ee_addr + 1, lo)
        else:
            self._write_sram(reg.sram_addr, value)
            if persist and reg.ee_addr is not None:
                self._write_ee(reg.ee_addr, value)

    def read_all(self) -> dict[str, int]:
        """Read all known registers and return a name→raw-value dict."""
        return {name: self.read_register(name) for name in R.all_registers()}

    # ------------------------------------------------------------------
    # Convenience helpers
    # ------------------------------------------------------------------

    def get_model(self) -> str:
        """Read the model name string (ASCII, up to 21 chars) from addr 96–116."""
        chars = []
        for addr in range(R.MODEL_NAME_START, R.MODEL_NAME_END + 1):
            byte = self._read8(addr)
            if 32 <= byte < 128:
                chars.append(chr(byte))
        return "".join(chars).strip()

    def get_version(self) -> int:
        """Read firmware version byte."""
        return self._read_raw(P.CMD_READ_VER, addr=0, data=0, offset=self._data_offset)

    def set_position(self, microseconds: float) -> None:
        """Send a position command. *microseconds* is the PWM pulse width (e.g. 1500)."""
        pwm_raw = round(microseconds * 4)
        packet = P.position_packet(pwm_raw)
        self._transport.send(packet, expect_reply=False)

    # ------------------------------------------------------------------
    # Transport helpers
    # ------------------------------------------------------------------

    @property
    def _data_offset(self) -> int:
        # DPC-20 transport strips the kVs3/kRs3 prefix; remaining bytes are
        # [cmd_echo, addr_echo, VALUE, csum] → value at index 2.
        # DPC-11 raw mode response has a 4-byte header before the value → index 4.
        return 2 if self._mode == "dpc20" else 4

    def _read8(self, addr: int) -> int:
        return self._read_raw(P.CMD_READ_8, addr=addr, data=0, offset=self._data_offset)

    def _write_sram(self, addr: int, value: int) -> None:
        for _ in range(3):
            self._transport.send(
                P.servo_packet(P.CMD_WRITE_8, addr, value), expect_reply=False
            )

    def _write_ee(self, addr: int, value: int) -> None:
        for _ in range(3):
            self._transport.send(
                P.servo_packet(P.CMD_WRITE_EE, addr, value), expect_reply=False
            )

    def _read_raw(self, cmd: int, addr: int, data: int, offset: int) -> int:
        packet = P.servo_packet(cmd, addr, data)
        resp = self._transport.send(packet, expect_reply=True)
        if resp is None or len(resp) <= offset:
            raise TimeoutError(f"No response from servo (cmd=0x{cmd:02X} addr={addr})")
        return resp[offset]

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def close(self) -> None:
        self._transport.close()

    def __enter__(self) -> "HitecServo":
        return self

    def __exit__(self, *_) -> None:
        self.close()
