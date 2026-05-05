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
Hitec D-series servo direct protocol for Raspberry Pi Pico (CircuitPython).

Copy this file to your Pico's filesystem alongside code.py.  It has no
dependencies outside CircuitPython's standard libraries plus adafruit_pioasm
and rp2pio (both ship with CircuitPython 8+).

Physical layer: 115200 baud, 8N1, INVERTED polarity, half-duplex on ONE wire.
No hardware inverter needed — the PIO state machines handle signal inversion
in software.

Wiring:
  - Servo signal wire → 1 kΩ resistor → GP15 (or any GPIO you choose)
  - Pull the GPIO up to 3V3 via the same 1 kΩ (the resistor doubles as pullup)
  - Servo power (red/brown): external 5–7.4 V supply, NOT the Pico's 3V3/5V pin

Usage::

    from pico_dseries import DServoComm

    servo = DServoComm(signal_pin=board.GP15)
    value, err = servo.read_register(0x0C)   # position (HD_REG_CURRENT_APV)
    if err:
        print("error:", err)
    else:
        print("position APV:", value)

    servo.write_register(0x4E, 1)   # set servo ID to 1
    servo.save_config()             # persist to flash

All register addresses match the REGS dict in dseries.py.  Use integer
addresses directly — there is no name lookup on the Pico to keep RAM lean.
"""

import time

import adafruit_pioasm
import board
import rp2pio

# ── Wire constants ────────────────────────────────────────────────────────────

_HDR_QUERY = 0x96
_HDR_REPLY = 0x69
_OP_READ   = 0x00
_OP_WRITE  = 0x02

_BAUD = 115200
_PIO_FREQ = _BAUD * 8          # 8× oversample; 1 bit = 8 PIO cycles
_WRITE_SETTLE_S = 0.005        # 5 ms after write command (no servo ACK)

# ── PIO programs ──────────────────────────────────────────────────────────────
#
# Inverted UART TX: idle = LOW, start bit = HIGH, data bits inverted, stop = LOW.
# Bytes are pre-inverted in software before handing to the TX SM so the SM just
# shifts bits out normally.
#
_TX_ASM = """
.program hitec_tx
.side_set 1 opt
.wrap_target
    pull          side 0 [7]    ; stop bit LOW (and idle); stalls when FIFO empty.
    set x, 7      side 1 [7]    ; start bit HIGH; load loop counter.
bit_loop:
    out pins, 1        [6]      ; one data bit (LSB first); 7 cycles.
    jmp x-- bit_loop            ; 1 cycle → 8 cycles per bit.
.wrap
"""

# Inverted UART RX: wait for servo idle LOW, then catch start bit HIGH.
# Auto-push fires after 8 bits → one byte per FIFO entry.
_RX_ASM = """
.program hitec_rx
.wrap_target
    wait 0 pin 0                ; servo asserts idle (line LOW)
    wait 1 pin 0                ; start bit (line HIGH)
    set x, 7      [11]          ; 1+11=12 cycles → sample at centre of bit 0
bit_loop:
    in pins, 1    [6]           ; sample one bit (LSB first); 7 cycles.
    jmp x-- bit_loop            ; 1 cycle → 8 cycles per bit.
.wrap
"""

_TX_PROGRAM = adafruit_pioasm.Program(_TX_ASM)
_RX_PROGRAM = adafruit_pioasm.Program(_RX_ASM)


# ── Protocol helpers ──────────────────────────────────────────────────────────

def decode_model_name(product_no, product_version):
    """Decode the servo model name from product_no (addr 0) and product_version (addr 2)."""
    text = ""
    text2 = ""
    text3 = ""
    flag = False
    num2 = product_version & 0xF000
    num3 = product_version & 0x00F0
    if num2 != 0:
        text3 = "_"
    if num2 == 0x2000 and num3 == 0x30:
        flag = True
    num5 = product_no // 10000
    num6 = product_no % 10000
    flag2 = False
    if num5 == 0:
        text = "D"
        text2 = str(product_no)
    elif num5 == 1:
        text = "MD"
        text2 = str(num6)
    elif num5 == 2:
        text = "SERVO"
        text2 = chr(num6) if 0 < num6 < 0x10000 else ""
    elif num5 == 3:
        num7 = num6 // 1000
        text2 = str(num6 % 1000)
        text3 = "_"
        text = {0: "DB", 1: "X", 2: "MDB", 3: "D", 4: "D",
                5: "MD", 6: "D", 7: "MD"}.get(num7, "")
    elif num5 == 4:
        text = "SG"
        text2 = str(num6)
        flag2 = True
    elif num5 == 5 and num6 == 1:
        text = "HLS"
        text2 = "12"
    if flag and not flag2 and text and not text.endswith("B"):
        text += "B"
    result = text + text2 + text3
    return result if result else "Unknown(%d)" % product_no


def _invert(data):
    return bytes(b ^ 0xFF for b in data)


def _build_read_query(servo_id, addr):
    cs = (servo_id + addr) & 0xFF
    return bytes([_HDR_QUERY, servo_id, addr, _OP_READ, cs])


def _build_write_cmd(servo_id, addr, value):
    # Little-endian: byte[4] = LSB, byte[5] = MSB (matches read response order).
    low  = value & 0xFF
    high = (value >> 8) & 0xFF
    cs = (servo_id + addr + _OP_WRITE + low + high) & 0xFF
    return bytes([_HDR_QUERY, servo_id, addr, _OP_WRITE, low, high, cs])


def _parse_read_response(addr, data):
    if len(data) != 7:
        return None, "short response: %d bytes" % len(data)
    hdr, mystery, addr2, op, low, high, cs = data
    if hdr != _HDR_REPLY:
        return None, "bad header: 0x%02X" % hdr
    if addr2 != addr:
        return None, "addr echo mismatch: 0x%02X" % addr2
    if op != _OP_WRITE:
        return None, "bad op byte: 0x%02X" % op
    expected_cs = (mystery + addr2 + op + low + high) & 0xFF
    if cs != expected_cs:
        return None, "checksum mismatch: 0x%02X" % cs
    return low | (high << 8), None


# ── State-machine helpers ─────────────────────────────────────────────────────

def _make_tx_sm(signal_pin):
    return rp2pio.StateMachine(
        _TX_PROGRAM.assembled,
        frequency=_PIO_FREQ,
        first_out_pin=signal_pin,
        first_sideset_pin=signal_pin,
        initial_out_pin_direction=1,
        initial_sideset_pin_direction=1,
        initial_out_pin_state=0,
        initial_sideset_pin_state=0,
        out_shift_right=True,
        auto_pull=False,
        pull_threshold=8,
        **_TX_PROGRAM.pio_kwargs,
    )


def _make_rx_sm(signal_pin):
    return rp2pio.StateMachine(
        _RX_PROGRAM.assembled,
        frequency=_PIO_FREQ,
        first_in_pin=signal_pin,
        in_shift_right=True,
        auto_push=True,
        push_threshold=8,
        **_RX_PROGRAM.pio_kwargs,
    )


# ── Public class ──────────────────────────────────────────────────────────────

class DServoComm:
    """
    Communicate with a Hitec D-series servo directly from a Pico.

    Parameters
    ----------
    signal_pin:
        The GPIO pin connected to the servo signal wire (e.g. ``board.GP15``).
    servo_id:
        Servo bus ID (0–254).  Factory default is 0.

    All D-series register addresses are available as ``dseries.REGS`` on the
    host side.  On the Pico, pass integer addresses directly.
    """

    def __init__(self, signal_pin=None, servo_id: int = 0) -> None:
        if signal_pin is None:
            signal_pin = board.GP15
        self._pin = signal_pin
        self._servo_id = servo_id

    def read_register(self, addr: int) -> tuple:
        """
        Send a read query for *addr* and return ``(value, None)`` on success
        or ``(None, error_string)`` on failure.
        """
        query = _build_read_query(self._servo_id, addr)

        tx_sm = _make_tx_sm(self._pin)
        try:
            tx_sm.write(_invert(query))
        finally:
            tx_sm.deinit()

        # Start RX immediately: servo responds in ~2 ms.
        # The RX PIO's `wait 0 / wait 1` sequence synchronises to the start bit.
        rx_sm = _make_rx_sm(self._pin)
        try:
            raw = bytearray(7)
            rx_sm.readinto(raw)
        finally:
            rx_sm.deinit()

        return _parse_read_response(addr, _invert(raw))

    def write_register(self, addr: int, value: int) -> None:
        """
        Write a 16-bit *value* to register *addr*.  Fire-and-forget; the
        servo does not acknowledge write commands.
        """
        cmd = _build_write_cmd(self._servo_id, addr, value)

        tx_sm = _make_tx_sm(self._pin)
        try:
            tx_sm.write(_invert(cmd))
        finally:
            tx_sm.deinit()

        time.sleep(_WRITE_SETTLE_S)

    def send_position(self, value: int) -> None:
        """Command the servo to move to *value* (write POSITION_NEW, addr 30)."""
        self.write_register(30, value)

    def read_model_name(self):
        """Return ``(model_name_str, None)`` or ``(None, error_string)``."""
        pno, err = self.read_register(0)
        if err:
            return None, err
        pver, err = self.read_register(2)
        if err:
            return None, err
        return decode_model_name(pno, pver), None

    def save_config(self) -> None:
        """Persist current register values to the servo's flash memory."""
        self.write_register(112, 1)   # ADDR_CONFIG_SAVE

    def restore_factory_defaults(self) -> None:
        """Restore factory defaults (irreversible until manually reconfigured)."""
        self.write_register(110, 1)   # ADDR_FACTORY_DEFAULT
