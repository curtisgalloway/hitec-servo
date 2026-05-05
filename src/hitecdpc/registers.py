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

"""Register map for HS-5/7XXX series servos (dpc_57.exe).

Addresses and encodings confirmed from frmWin5xxx7xxx.cs decompilation.
"""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Register:
    sram_addr: int
    ee_addr: int | None        # EEPROM mirror address; None = no persistent copy
    is_16bit: bool = False     # True → read two consecutive bytes, assemble big-endian
    scale: float = 1.0         # multiply raw value by scale to get meaningful units
    unit: str = ""             # human-readable unit string
    description: str = ""


# fmt: off
_REGS: dict[str, Register] = {
    "kp_min":      Register(sram_addr=0,   ee_addr=129,  description="Kp gain minimum"),
    "kp_max":      Register(sram_addr=1,   ee_addr=130,  description="Kp gain maximum"),
    "deadband":    Register(sram_addr=2,   ee_addr=131,  description="Dead band (1–16)"),
    "kd":          Register(sram_addr=3,   ee_addr=132,  description="Kd gain"),
    "kdj":         Register(sram_addr=4,   ee_addr=133,  description="Kd gain join"),
    "asccnt":      Register(sram_addr=5,   ee_addr=134,  description="Acceleration count"),
    "speed":       Register(sram_addr=6,   ee_addr=135,  description="Speed (1=10%…64=100%)"),
    "ioffset":     Register(sram_addr=7,   ee_addr=136,  is_16bit=True, scale=0.25, unit="µs",
                            description="Position offset"),
    "smin":        Register(sram_addr=9,   ee_addr=None, is_16bit=True, scale=0.25, unit="µs",
                            description="Minimum position"),
    "smax":        Register(sram_addr=11,  ee_addr=None, is_16bit=True, scale=0.25, unit="µs",
                            description="Maximum position"),
    "imin":        Register(sram_addr=13,  ee_addr=None, is_16bit=True,
                            description="Input minimum"),
    "imax":        Register(sram_addr=15,  ee_addr=None, is_16bit=True,
                            description="Input maximum"),
    "sref":        Register(sram_addr=17,  ee_addr=148,  is_16bit=True, scale=0.25, unit="µs",
                            description="Center / reference position"),
    "apos":        Register(sram_addr=19,  ee_addr=150,  description="Angle limit (+)"),
    "aneg":        Register(sram_addr=20,  ee_addr=151,  description="Angle limit (−)"),
    "fs_time":     Register(sram_addr=21,  ee_addr=None, is_16bit=True, unit="ms",
                            description="Failsafe delay"),
    "ms_time":     Register(sram_addr=23,  ee_addr=None, is_16bit=True,
                            description="Multiplex signal time"),
    "fpos":        Register(sram_addr=25,  ee_addr=None, is_16bit=True, scale=0.25, unit="µs",
                            description="Failsafe position"),
    "status":      Register(sram_addr=27,  ee_addr=None,
                            description="Flags: bit0=direction (0=CCW,1=CW), bit1+=failsafe"),
    "type":        Register(sram_addr=29,  ee_addr=None, description="Servo type code"),
    "diss_limit":  Register(sram_addr=32,  ee_addr=162,  description="Dissipation limit"),
    "stell_limit": Register(sram_addr=33,  ee_addr=164,
                            description="Stall/OLP limit (% or ≥100=off)"),
    "high_res":    Register(sram_addr=34,  ee_addr=None, description="High-resolution mode flag"),
    "stretch":     Register(sram_addr=35,  ee_addr=None, description="Stretch"),
}
# fmt: on

# Alternative names users might pass
_ALIASES: dict[str, str] = {
    "dead_band":  "deadband",
    "Dead Band":  "deadband",
    "kp-min":     "kp_min",
    "kp-max":     "kp_max",
}

# SRAM addr 96-116 = model name (21 ASCII bytes), read one byte at a time with cmd 'a'
MODEL_NAME_START = 96
MODEL_NAME_END   = 116


def get(name: str) -> Register:
    """Return the Register for *name*, resolving aliases. Raises KeyError if unknown."""
    canonical = _ALIASES.get(name, name)
    try:
        return _REGS[canonical]
    except KeyError:
        raise KeyError(
            f"Unknown register {name!r}. Available: {sorted(_REGS)}"
        ) from None


def all_registers() -> dict[str, Register]:
    return dict(_REGS)
