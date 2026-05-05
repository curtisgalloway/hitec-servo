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
Hitec D-series servo direct protocol — no DPC adapter needed.

Physical layer: 115200 baud, 8N1, INVERTED polarity, half-duplex on one wire.
Hardware options:
  - USB-serial adapter + single-transistor inverter (Mac / Linux / Raspberry Pi)
  - Raspberry Pi built-in UART + transistor inverter (/dev/serial0 at 115200)
  - Pico: use pico_dseries.py (PIO state machines, no extra hardware)

All D-series registers are 16-bit.  Both read responses and write commands use
little-endian byte order: byte[4] = LSB, byte[5] = MSB.
"""

from __future__ import annotations

import time

import serial

# ── Wire constants ────────────────────────────────────────────────────────────

HDR_QUERY = 0x96  # first byte of every host→servo packet
HDR_REPLY = 0x69  # first byte of every servo→host packet
OP_READ   = 0x00  # operation byte for read commands
OP_WRITE  = 0x02  # operation byte for write commands (same in responses)

BAUD = 115200

# ── D-series register addresses (all 16-bit) ──────────────────────────────────
#
# Source: frmWinDxxx.cs private const byte ADDR_* declarations.
# All addresses are even; registers are 16-bit little-endian.
# R = read-only telemetry, R/W = configurable, W = write-triggers-action.

REGS: dict[str, int] = {
    # ── Read-only identity / telemetry ────────────────────────────────────────
    "product_no":               0,   # R  — model code; decode with decode_model_name()
    "product_version":          2,   # R  — hardware variant flags
    "firmware_version":         4,   # R  — firmware version number
    "ic_serial_sub":            6,   # R
    "ic_serial_main":           8,   # R
    "status":                  10,   # R  — error/status flags (see AGENTS.md bit map)
    "position":                12,   # R  — current APV (HD_REG_CURRENT_APV)
    "velocity":                14,   # R  — current velocity
    "torque":                  16,   # R  — current torque
    "voltage":                 18,   # R  — supply voltage
    "temperature":             20,   # R  — internal temperature
    "current":                 26,   # R  — current draw
    "position_new":            30,   # R/W — last commanded / target position
    "position_in_new":        234,   # R  — target position feedback

    # ── Parameter version ─────────────────────────────────────────────────────
    "param_version_1":         32,   # R  — firmware feature version (≥35 → SmartSense)
    "param_version_2":         36,   # R

    # ── User data registers ───────────────────────────────────────────────────
    "user_1":                  38,   # R
    "user_2":                  40,   # R
    "user_3":                  42,   # R

    # ── Identity / bus config (saved to flash) ────────────────────────────────
    "servo_type":              48,   # R/W
    "servo_id":                50,   # R/W — bus ID 0–254; factory default 0
    "baudrate":                52,   # R/W — baud rate code 0–8; see BAUDRATE_CODES
    "signal_mode":             54,   # R/W
    "simple_return_delay":     56,   # R/W
    "normal_return_delay":     58,   # R/W

    # ── SmartSense vibration detection (firmware ≥ v35) ───────────────────────
    "vib_sign_change_margin":  60,   # R/W
    "vib_min_max_margin":      62,   # R/W
    "vib_good_check_no":       64,   # R/W
    "vib_speed_check_no":      66,   # R/W
    "vib_d_gain_min":          68,   # R/W

    # ── Power / safety ────────────────────────────────────────────────────────
    "power_config":            70,   # R/W
    "emergency":               72,   # R/W
    "action_mode":             74,   # R/W
    "failsafe":                76,   # R/W — failsafe position / mode

    # ── Motion limits ─────────────────────────────────────────────────────────
    "deadband":                78,   # R/W
    "position_max":            80,   # R/W
    "position_min":            82,   # R/W
    "velocity_max":            84,   # R/W
    "torque_max":              86,   # R/W
    "voltage_max":             88,   # R/W
    "voltage_min":             90,   # R/W
    "temperature_max":         92,   # R/W
    "direction":               94,   # R/W — 0=CCW, 1=CW

    # ── Motion tuning ─────────────────────────────────────────────────────────
    "start_speed":             96,   # R/W
    "power_down_time":         98,   # R/W
    "position_slope":         100,   # R/W
    "vib_deadband_min":       102,   # R/W — SmartSense
    "vib_deadband_max":       104,   # R/W — SmartSense
    "vib_deadband_delay":     106,   # R/W — SmartSense
    "vib_deadband_p_gain":    108,   # R/W — SmartSense

    # ── Control (write triggers action) ───────────────────────────────────────
    "factory_default":        110,   # W — write any value → restore factory defaults
    "config_save":            112,   # W — write any value → save to flash
    "lock":                   114,   # R/W
    "firmware_upgrade":       120,   # W — firmware update trigger

    # ── Protocol timing ───────────────────────────────────────────────────────
    "rx_byte_interval":       122,   # R/W
    "simple_return_limit_time":124,  # R/W
    "pwm_ext_delay_time":     126,   # R/W
    "pwm_normal_ack_min":     128,   # R/W
    "pwm_normal_hold_limit":  130,   # R/W

    # ── Motor hardware parameters ─────────────────────────────────────────────
    "motor_turn_direction":   132,   # R/W
    "motor_pwm_period":       134,   # R/W
    "motor_pwm_deadtime":     136,   # R/W

    # ── PID controller ────────────────────────────────────────────────────────
    "pid_p":                  138,   # R/W — proportional gain
    "pid_d":                  140,   # R/W — derivative gain
    "pid_i":                  142,   # R/W — integral gain
    "pid_deadband":           144,   # R/W — PID dead band
    "pos_min_max_margin":     146,   # R/W

    # ── Calibration ───────────────────────────────────────────────────────────
    "position_4095":          148,   # R/W — position at encoder 4095
    "position_0":             150,   # R/W — position at encoder 0

    # ── Position / torque / temperature lock ──────────────────────────────────
    "pos_lock_limit":         152,   # R/W
    "pos_lock_time":          154,   # R/W
    "pos_lock_ratio":         156,   # R/W
    "torque_lock_time":       158,   # R/W
    "temper_lock_limit":      160,   # R/W
    "temper_lock_time":       162,   # R/W

    # ── Sampling / ADC ────────────────────────────────────────────────────────
    "pid_sampling_time":      164,   # R/W
    "velocity_sampling_time": 166,   # R/W
    "hum_sampling_time":      168,   # R/W
    "adc_sampling_time":      170,   # R/W

    # ── Temperature calibration ───────────────────────────────────────────────
    "temper_25_deg":          172,   # R/W
    "temper_50_deg":          174,   # R/W

    # ── EPA (End Point Adjustment) calibration ────────────────────────────────
    "pos_pwm_max":            176,   # R/W — PWM endpoint → position (right)
    "pos_pwm_min":            178,   # R/W — PWM endpoint → position (left)
    "pos_virtual_bit":        180,   # R/W
    "pos_oversampling_bit":   182,   # R/W
    "pid_pos_para_p_gain":    184,   # R/W
    "motor_deadband_offset":  186,   # R/W
    "i_comp_max":             188,   # R/W
    "motor_pwm_prescaler":    192,   # R/W
    "pos_pwm_mid":            194,   # R/W — PWM center → position (mid)
    "pwm_in_signal_range":    196,   # R/W
    "sys_config":             198,   # R/W

    # ── Extended identity / SmartSense VIB ───────────────────────────────────
    "serial_sub":             204,   # R/W
    "vib_check_max_no":       206,   # R/W
    "vib_pwm_in_deadband":    208,   # R/W
    "vib_pwm_good_no":        210,   # R/W
    "pid_p_vib":              212,   # R/W
    "pid_d_vib":              214,   # R/W

    # ── Target position limits ────────────────────────────────────────────────
    "pos_target_limit_max":   216,   # R/W
    "pos_target_limit_min":   218,   # R/W
    "rx_packet_interval":     220,   # R/W

    # ── Advanced / miscellaneous ──────────────────────────────────────────────
    "torque_min":             232,   # R/W
    "pid_gain":               236,   # R/W
    "time_run":               238,   # R/W
    "pid_config":             242,   # R/W
    "pz":                     244,   # R/W
    "fw_ver":                 246,   # R
    "ppm_conf_2":             248,   # R/W
    "speed_conf":             250,   # R/W
    "sys_config_2":           252,   # R/W
    "ppm_conf":               254,   # R/W
}

# Convenience alias
REG = REGS

# ── Baudrate register encoding ────────────────────────────────────────────────
#
# Values written to the `baudrate` register (addr 52).
# Source: ComboBox2.Items in frmWinDxxx.cs (9-entry list, index 0–8).

BAUDRATE_CODES: dict[int, int] = {
    0: 9_600,
    1: 14_400,
    2: 19_200,
    3: 38_400,
    4: 57_600,
    5: 115_200,   # factory default
    6: 229_800,
    7: 459_700,
    8: 930_200,
}

# Reverse map: baud rate → register code
BAUDRATE_TO_CODE: dict[int, int] = {v: k for k, v in BAUDRATE_CODES.items()}


def get_reg(name: str) -> int:
    """Return the register address for *name*, raising KeyError on unknown names."""
    try:
        return REGS[name.lower().replace(" ", "_")]
    except KeyError:
        raise KeyError(f"Unknown D-series register: {name!r}") from None


# ── Model name decoding ───────────────────────────────────────────────────────

def decode_model_name(product_no: int, product_version: int) -> str:
    """
    Reconstruct the servo's marketing model name from two identity registers.

    Parameters
    ----------
    product_no:
        Value read from register address 0 (``ADDR_PRODUCT_NO``).
    product_version:
        Value read from register address 2 (``ADDR_PRODUCT_VERSION``).

    Returns
    -------
    str
        E.g. ``"D646_"`` for a D646WP.  The trailing ``_`` indicates a variant
        suffix (WP = waterproof, B = brushless, etc.) that is a physical label
        only — not encoded further in firmware registers.

    Notes
    -----
    Algorithm reverse-engineered from ``frmWinDxxx.cs``, lines 11956–12062.
    """
    text = ""
    text2 = ""
    text3 = ""
    flag = False

    # product_version encodes the variant suffix flag and B-series indicator.
    num2 = product_version & 0xF000
    num3 = product_version & 0x00F0
    if num2 != 0:
        text3 = "_"
    if num2 == 0x2000 and num3 == 0x30:
        flag = True  # B-series (brush variant)

    # product_no encodes the series prefix and model number.
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
    return result if result else f"Unknown({product_no})"


# ── Pure protocol functions ───────────────────────────────────────────────────

def build_read_query(servo_id: int, addr: int) -> bytes:
    """Build a 5-byte read query packet."""
    cs = (servo_id + addr) & 0xFF
    return bytes([HDR_QUERY, servo_id, addr, OP_READ, cs])


def build_write_cmd(servo_id: int, addr: int, value: int) -> bytes:
    """
    Build a 7-byte write command packet.  *value* is a 16-bit unsigned int.

    Both reads and writes use little-endian byte order: byte[4] = LSB,
    byte[5] = MSB.  (The DPC firmware names these 'high' and 'low' respectively,
    which is backwards from the conventional meaning.)
    """
    low  = value & 0xFF          # byte[4] — LSB
    high = (value >> 8) & 0xFF   # byte[5] — MSB
    cs = (servo_id + addr + OP_WRITE + low + high) & 0xFF
    return bytes([HDR_QUERY, servo_id, addr, OP_WRITE, low, high, cs])


def parse_read_response(addr: int, data: bytes) -> tuple[int | None, str | None]:
    """
    Validate a 7-byte read response and return (value, None) or (None, error).

    Response layout: [0x69, mystery, addr, 0x02, low, high, checksum]
    Value is little-endian: low | (high << 8).
    """
    if len(data) != 7:
        return None, f"short response: {len(data)} bytes (expected 7)"
    hdr, mystery, addr2, op, low, high, cs = data
    if hdr != HDR_REPLY:
        return None, f"bad header 0x{hdr:02X} (expected 0x{HDR_REPLY:02X})"
    if addr2 != addr:
        return None, f"addr echo mismatch: 0x{addr2:02X} (expected 0x{addr:02X})"
    if op != OP_WRITE:
        return None, f"bad op byte 0x{op:02X} (expected 0x{OP_WRITE:02X})"
    expected_cs = (mystery + addr2 + op + low + high) & 0xFF
    if cs != expected_cs:
        return None, f"checksum mismatch: 0x{cs:02X} (expected 0x{expected_cs:02X})"
    return low | (high << 8), None


# ── CPython transport (pyserial + hardware inverter) ──────────────────────────

class DSeriesTransport:
    """
    Direct D-series servo transport for Mac / Linux / Raspberry Pi.

    Requires a USB-serial TTL adapter and a single-transistor signal inverter
    between the adapter's TX/RX lines and the servo signal wire.  The transistor
    inverts the polarity so that pyserial can use standard (non-inverted) UART.

    Half-duplex echo: because TX and RX share the same physical wire, every byte
    we transmit is echoed back on RX.  ``echo_cancel=True`` (default) reads and
    discards those echo bytes before waiting for the servo response.  Set it to
    ``False`` only if your adapter hardware suppresses its own TX echo.

    Parameters
    ----------
    port:
        Serial port device (e.g. ``/dev/ttyUSB0``, ``/dev/cu.usbserial-XXXX``).
    servo_id:
        Servo bus ID (0–254).  0 is the factory default.
    timeout:
        Per-read timeout in seconds.
    echo_cancel:
        Discard transmitted bytes from the RX buffer before reading the response.
    """

    _WRITE_SETTLE_S = 0.005   # 5 ms after write (mirrors Thread.Sleep(5) in C#)

    def __init__(
        self,
        port: str,
        servo_id: int = 0,
        timeout: float = 0.1,
        echo_cancel: bool = True,
    ) -> None:
        self._ser = serial.Serial(port, BAUD, timeout=timeout)
        self._servo_id = servo_id
        self._echo_cancel = echo_cancel

    # ── Public API ────────────────────────────────────────────────────────────

    def read_register(self, name_or_addr: str | int) -> tuple[int | None, str | None]:
        """
        Read a register by name or address. Returns ``(value, None)`` on
        success or ``(None, error_string)`` on failure.
        """
        addr = name_or_addr if isinstance(name_or_addr, int) else get_reg(name_or_addr)
        query = build_read_query(self._servo_id, addr)
        self._ser.reset_input_buffer()
        self._ser.write(query)
        if self._echo_cancel:
            self._ser.read(len(query))
        raw = self._ser.read(7)
        return parse_read_response(addr, raw)

    def write_register(self, name_or_addr: str | int, value: int) -> None:
        """
        Write *value* to a register by name or address.  Fire-and-forget; the
        servo does not send a response to write commands.
        """
        addr = name_or_addr if isinstance(name_or_addr, int) else get_reg(name_or_addr)
        cmd = build_write_cmd(self._servo_id, addr, value)
        self._ser.reset_input_buffer()
        self._ser.write(cmd)
        time.sleep(self._WRITE_SETTLE_S)

    def send_position(self, value: int) -> None:
        """
        Command the servo to move to *value* (write POSITION_NEW, addr 30).

        The position range depends on the servo model; for D-series servos the
        typical range is 0–4095 (12-bit encoder).  Values outside the servo's
        configured position_min / position_max limits are clamped in firmware.
        """
        self.write_register("position_new", value)

    def read_model_name(self) -> tuple[str | None, str | None]:
        """
        Read product_no and product_version and return the decoded model name.

        Returns ``(name, None)`` on success or ``(None, error_string)`` on failure.
        """
        pno, err = self.read_register("product_no")
        if err:
            return None, err
        pver, err = self.read_register("product_version")
        if err:
            return None, err
        return decode_model_name(pno, pver), None

    def save_config(self) -> None:
        """Persist current settings to flash (writes ADDR_CONFIG_SAVE)."""
        self.write_register("config_save", 1)

    def restore_factory_defaults(self) -> None:
        """Restore factory defaults (writes ADDR_FACTORY_DEFAULT)."""
        self.write_register("factory_default", 1)

    def close(self) -> None:
        self._ser.close()

    def __enter__(self) -> "DSeriesTransport":
        return self

    def __exit__(self, *_) -> None:
        self.close()
