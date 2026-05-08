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
Hitec DPC packet building and parsing.

All wire format knowledge comes from static analysis of dpc_57.exe.
See docs/protocol.md for the full specification.
"""

from .crc import crc8

# Command codes (ASCII letters)
CMD_READ_8 = 0x61  # 'a' — read 8-bit SRAM register,  response at offset 4
CMD_WRITE_8 = 0x62  # 'b' — write 8-bit SRAM register  (addr 0-127)
CMD_WRITE_EE = 0x64  # 'd' — write 8-bit EEPROM register (addr 128-255)
CMD_READ_16 = 0x65  # 'e' — read 16-bit register,      response at offsets 4-5
CMD_SET_POS = 0x66  # 'f' — set servo position (16-bit PWM value)
CMD_READ_VER = 0x67  # 'g' — read firmware version,     response at offset 4

# STX/ETX mux values
MUX_HANDSHAKE = 0  # STX=0x02, ETX=0x03 — probe packets
MUX_COMMAND = 7  # STX=0x09, ETX=0x0A — host→adapter
MUX_RESPONSE = 11  # STX=0x0D, ETX=0x0E — adapter→host


def servo_packet(cmd: int, addr: int, data: int) -> bytes:
    """Build a 4-byte servo command packet: [CMD, ADDR, DATA, CSUM]."""
    csum = (256 - (cmd + addr + data) % 256) % 256
    return bytes([cmd, addr, data, csum])


def position_packet(pwm_raw: int) -> bytes:
    """Build a position-control packet. pwm_raw = microseconds * 4."""
    hi = (pwm_raw >> 8) & 0xFF
    lo = pwm_raw & 0xFF
    csum = (256 - (CMD_SET_POS + hi + lo) % 256) % 256
    return bytes([CMD_SET_POS, hi, lo, csum])


def stxetx_frame(payload: bytes, mux: int) -> bytes:
    """Wrap payload in STX/ETX frame: [STX, payload..., CRC8, len, ETX]."""
    stx = (2 + mux) & 0xFF
    etx = (3 + mux) & 0xFF
    checksum = crc8(payload)
    return bytes([stx]) + payload + bytes([checksum, len(payload), etx])


def parse_stxetx(data: bytes, mux: int) -> bytes | None:
    """Scan data for a valid STX/ETX frame and return its payload, or None."""
    stx = (2 + mux) & 0xFF
    etx = (3 + mux) & 0xFF
    for i in range(4, len(data)):
        if data[i] != etx:
            continue
        length = data[i - 1]
        checksum = data[i - 2]
        stx_pos = i - 3 - length  # i=ETX, i-1=LEN, i-2=CRC, i-3-len=STX
        if stx_pos < 0 or data[stx_pos] != stx:
            continue
        payload = data[stx_pos + 1 : stx_pos + 1 + length]
        if len(payload) == length and crc8(payload) == checksum:
            return payload
    return None


def kso_wrapper(
    servo_bytes: bytes, four_pin: bool = False, want_reply: bool = True
) -> bytes:
    """
    Build the DPC-20 KSO3/KSO4 wrapper used by send_serial_packet().

    Servo bytes are bit-inverted inside the wrapper. The returned bytes
    are then passed to stxetx_frame(..., mux=MUX_COMMAND) before sending.
    """
    pin_byte = 0x34 if four_pin else 0x33  # '4' or '3'
    inverted = bytes(~b & 0xFF for b in servo_bytes)
    ret = bytes([10, 10, 20]) if want_reply else bytes([0, 0, 0])
    return bytes([0x4B, 0x53, 0x4F, pin_byte, len(servo_bytes)]) + inverted + ret


# Key strings in DPC-20 responses
KEY_APP = b"kAPP"  # adapter ready after probe
KEY_IBR = b"kIBR"  # adapter in firmware-update mode (IBus Read)
KEY_IBW = b"kIBW"  # adapter in firmware-update mode (IBus Write)
KEY_IBU = b"kIBU"  # adapter in firmware-update mode (IBus Upgrade)
KEY_RS3 = b"kRs3"  # servo response payload prefix (legacy firmware)
KEY_VS3 = b"kVs3"  # servo response payload prefix (new AT32 firmware)

# DPC-20 handshake sequence message strings
MSG_PROBE = b"KWAU"  # probe: sent to detect adapter
MSG_RESET = b":A:A:A"  # ASCII reset string sent when adapter is in FW-update mode
MSG_3PIN_57 = b"KP3S5"  # select HS-5/7XXX 3-pin mode
MSG_3PIN_9 = b"KP3S9"  # select HSB-9XXX 3-pin mode
MSG_3PIN = b"KP3S"  # select generic 3-pin mode
MSG_4PIN = b"KP4S"  # select 4-pin mode
