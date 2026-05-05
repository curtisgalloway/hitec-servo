// Copyright 2025 Curtis Galloway
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//! Hitec D-series servo direct protocol — no DPC adapter needed.
//!
//! Physical layer: 115200 baud, 8N1, **inverted** polarity, half-duplex on one wire.
//! Hardware: USB-serial adapter + single-transistor signal inverter.
//!
//! All D-series registers are 16-bit.  Both read responses and write commands use
//! little-endian byte order: byte\[4\] = LSB, byte\[5\] = MSB.

// ── Wire constants ────────────────────────────────────────────────────────────

/// First byte of every host→servo packet.
pub const HDR_QUERY: u8 = 0x96;
/// First byte of every servo→host response.
pub const HDR_REPLY: u8 = 0x69;
/// Operation byte: read.
pub const OP_READ: u8 = 0x00;
/// Operation byte: write (also appears in read responses).
pub const OP_WRITE: u8 = 0x02;
/// Default baud rate.
pub const BAUD: u32 = 115_200;

// ── Register addresses ────────────────────────────────────────────────────────

/// D-series register addresses.  All registers are 16-bit (addresses are even).
///
/// Source: `frmWinDxxx.cs` `private const byte ADDR_*` declarations.
pub mod regs {
    // ── Read-only identity / telemetry ────────────────────────────────────────
    /// Model code; decode with [`super::decode_model_name`].
    pub const PRODUCT_NO: u8 = 0;
    /// Hardware variant flags; used with PRODUCT_NO for model name decoding.
    pub const PRODUCT_VERSION: u8 = 2;
    pub const FIRMWARE_VERSION: u8 = 4;
    pub const IC_SERIAL_SUB: u8 = 6;
    pub const IC_SERIAL_MAIN: u8 = 8;
    /// Error/status flags (see AGENTS.md bit map).
    pub const STATUS: u8 = 10;
    /// Current position (APV) — `HD_REG_CURRENT_APV`.
    pub const POSITION: u8 = 12;
    pub const VELOCITY: u8 = 14;
    pub const TORQUE: u8 = 16;
    pub const VOLTAGE: u8 = 18;
    pub const TEMPERATURE: u8 = 20;
    pub const CURRENT: u8 = 26;
    /// Last commanded / target position.  Write to command movement.
    pub const POSITION_NEW: u8 = 30;
    /// Target position feedback.
    pub const POSITION_IN_NEW: u8 = 234;

    // ── Parameter version ─────────────────────────────────────────────────────
    /// Firmware feature version (≥ 35 enables SmartSense).
    pub const PARAM_VERSION_1: u8 = 32;
    pub const PARAM_VERSION_2: u8 = 36;

    // ── User data registers ───────────────────────────────────────────────────
    pub const USER_1: u8 = 38;
    pub const USER_2: u8 = 40;
    pub const USER_3: u8 = 42;

    // ── Identity / bus config (saved to flash) ────────────────────────────────
    pub const SERVO_TYPE: u8 = 48;
    /// Bus ID (0–254); factory default 0.
    pub const SERVO_ID: u8 = 50;
    /// Baud rate code 0–8; see `BAUDRATE_CODES`.
    pub const BAUDRATE: u8 = 52;
    pub const SIGNAL_MODE: u8 = 54;
    pub const SIMPLE_RETURN_DELAY: u8 = 56;
    pub const NORMAL_RETURN_DELAY: u8 = 58;

    // ── SmartSense vibration detection (firmware ≥ v35) ───────────────────────
    pub const VIB_SIGN_CHANGE_MARGIN: u8 = 60;
    pub const VIB_MIN_MAX_MARGIN: u8 = 62;
    pub const VIB_GOOD_CHECK_NO: u8 = 64;
    pub const VIB_SPEED_CHECK_NO: u8 = 66;
    pub const VIB_D_GAIN_MIN: u8 = 68;

    // ── Power / safety ────────────────────────────────────────────────────────
    pub const POWER_CONFIG: u8 = 70;
    pub const EMERGENCY: u8 = 72;
    pub const ACTION_MODE: u8 = 74;
    pub const FAILSAFE: u8 = 76;

    // ── Motion limits ─────────────────────────────────────────────────────────
    pub const DEADBAND: u8 = 78;
    pub const POSITION_MAX: u8 = 80;
    pub const POSITION_MIN: u8 = 82;
    pub const VELOCITY_MAX: u8 = 84;
    pub const TORQUE_MAX: u8 = 86;
    pub const VOLTAGE_MAX: u8 = 88;
    pub const VOLTAGE_MIN: u8 = 90;
    pub const TEMPERATURE_MAX: u8 = 92;
    /// Direction: 0 = CCW, 1 = CW.
    pub const DIRECTION: u8 = 94;

    // ── Motion tuning ─────────────────────────────────────────────────────────
    pub const START_SPEED: u8 = 96;
    pub const POWER_DOWN_TIME: u8 = 98;
    pub const POSITION_SLOPE: u8 = 100;
    pub const VIB_DEADBAND_MIN: u8 = 102;
    pub const VIB_DEADBAND_MAX: u8 = 104;
    pub const VIB_DEADBAND_DELAY: u8 = 106;
    pub const VIB_DEADBAND_P_GAIN: u8 = 108;

    // ── Control (write triggers action) ───────────────────────────────────────
    /// Write any value → restore factory defaults.
    pub const FACTORY_DEFAULT: u8 = 110;
    /// Write any value → save current config to flash.
    pub const CONFIG_SAVE: u8 = 112;
    pub const LOCK: u8 = 114;
    /// Firmware update trigger.
    pub const FIRMWARE_UPGRADE: u8 = 120;

    // ── Protocol timing ───────────────────────────────────────────────────────
    pub const RX_BYTE_INTERVAL: u8 = 122;
    pub const SIMPLE_RETURN_LIMIT_TIME: u8 = 124;
    pub const PWM_EXT_DELAY_TIME: u8 = 126;
    pub const PWM_NORMAL_ACK_MIN: u8 = 128;
    pub const PWM_NORMAL_HOLD_LIMIT: u8 = 130;

    // ── Motor hardware parameters ─────────────────────────────────────────────
    pub const MOTOR_TURN_DIRECTION: u8 = 132;
    pub const MOTOR_PWM_PERIOD: u8 = 134;
    pub const MOTOR_PWM_DEADTIME: u8 = 136;

    // ── PID controller ────────────────────────────────────────────────────────
    pub const PID_P: u8 = 138;
    pub const PID_D: u8 = 140;
    pub const PID_I: u8 = 142;
    pub const PID_DEADBAND: u8 = 144;
    pub const POS_MIN_MAX_MARGIN: u8 = 146;

    // ── Calibration ───────────────────────────────────────────────────────────
    /// Calibration: position at encoder value 4095.
    pub const POSITION_4095: u8 = 148;
    /// Calibration: position at encoder value 0.
    pub const POSITION_0: u8 = 150;

    // ── Position / torque / temperature lock ──────────────────────────────────
    pub const POS_LOCK_LIMIT: u8 = 152;
    pub const POS_LOCK_TIME: u8 = 154;
    pub const POS_LOCK_RATIO: u8 = 156;
    pub const TORQUE_LOCK_TIME: u8 = 158;
    pub const TEMPER_LOCK_LIMIT: u8 = 160;
    pub const TEMPER_LOCK_TIME: u8 = 162;

    // ── Sampling / ADC ────────────────────────────────────────────────────────
    pub const PID_SAMPLING_TIME: u8 = 164;
    pub const VELOCITY_SAMPLING_TIME: u8 = 166;
    pub const HUM_SAMPLING_TIME: u8 = 168;
    pub const ADC_SAMPLING_TIME: u8 = 170;

    // ── Temperature calibration ───────────────────────────────────────────────
    pub const TEMPER_25_DEG: u8 = 172;
    pub const TEMPER_50_DEG: u8 = 174;

    // ── EPA (End Point Adjustment) calibration ────────────────────────────────
    /// PWM endpoint → position (right).
    pub const POS_PWM_MAX: u8 = 176;
    /// PWM endpoint → position (left).
    pub const POS_PWM_MIN: u8 = 178;
    pub const POS_VIRTUAL_BIT: u8 = 180;
    pub const POS_OVERSAMPLING_BIT: u8 = 182;
    pub const PID_POS_PARA_P_GAIN: u8 = 184;
    pub const MOTOR_DEADBAND_OFFSET: u8 = 186;
    pub const I_COMP_MAX: u8 = 188;
    pub const MOTOR_PWM_PRESCALER: u8 = 192;
    /// PWM center → position (mid-point).
    pub const POS_PWM_MID: u8 = 194;
    pub const PWM_IN_SIGNAL_RANGE: u8 = 196;
    pub const SYS_CONFIG: u8 = 198;

    // ── Extended identity / SmartSense VIB ────────────────────────────────────
    pub const SERIAL_SUB: u8 = 204;
    pub const VIB_CHECK_MAX_NO: u8 = 206;
    pub const VIB_PWM_IN_DEADBAND: u8 = 208;
    pub const VIB_PWM_GOOD_NO: u8 = 210;
    pub const PID_P_VIB: u8 = 212;
    pub const PID_D_VIB: u8 = 214;

    // ── Target position limits ────────────────────────────────────────────────
    pub const POS_TARGET_LIMIT_MAX: u8 = 216;
    pub const POS_TARGET_LIMIT_MIN: u8 = 218;
    pub const RX_PACKET_INTERVAL: u8 = 220;

    // ── Advanced / miscellaneous ──────────────────────────────────────────────
    pub const TORQUE_MIN: u8 = 232;
    pub const PID_GAIN: u8 = 236;
    pub const TIME_RUN: u8 = 238;
    pub const PID_CONFIG: u8 = 242;
    pub const PZ: u8 = 244;
    pub const FW_VER: u8 = 246;
    pub const PPM_CONF_2: u8 = 248;
    pub const SPEED_CONF: u8 = 250;
    pub const SYS_CONFIG_2: u8 = 252;
    pub const PPM_CONF: u8 = 254;
}

// ── Baudrate register encoding ────────────────────────────────────────────────

/// Baud rate values corresponding to codes 0–8 in the `baudrate` register.
///
/// Source: `ComboBox2.Items` in `frmWinDxxx.cs`.
pub const BAUDRATE_CODES: &[(u8, u32)] = &[
    (0, 9_600),
    (1, 14_400),
    (2, 19_200),
    (3, 38_400),
    (4, 57_600),
    (5, 115_200), // factory default
    (6, 229_800),
    (7, 459_700),
    (8, 930_200),
];

/// Decode a baudrate register value (0–8) to the actual baud rate in bits/s.
pub fn decode_baudrate(code: u16) -> Option<u32> {
    BAUDRATE_CODES.iter().find(|(c, _)| *c as u16 == code).map(|(_, b)| *b)
}

/// Encode a baud rate in bits/s to the register code, if supported.
pub fn encode_baudrate(baud: u32) -> Option<u8> {
    BAUDRATE_CODES.iter().find(|(_, b)| *b == baud).map(|(c, _)| *c)
}

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors that can occur when communicating with a D-series servo.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum Error {
    #[error("bad response header: 0x{got:02X} (expected 0x{expected:02X})")]
    BadHeader { got: u8, expected: u8 },

    #[error("address echo mismatch: 0x{got:02X} (expected 0x{expected:02X})")]
    AddrMismatch { got: u8, expected: u8 },

    #[error("checksum mismatch: 0x{got:02X} (expected 0x{expected:02X})")]
    ChecksumMismatch { got: u8, expected: u8 },

    #[error("short response: {got} bytes (expected {expected})")]
    ShortResponse { got: usize, expected: usize },

    #[error("timeout: no response from servo")]
    Timeout,

    #[cfg(feature = "transport")]
    #[error("serial port error: {0}")]
    Serial(#[from] serialport::Error),

    #[cfg(feature = "transport")]
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
}

// ── Model name decoding ───────────────────────────────────────────────────────

/// Reconstruct the servo's marketing model name from two identity registers.
///
/// * `product_no`      — value read from register address 0 (`ADDR_PRODUCT_NO`).
/// * `product_version` — value read from register address 2 (`ADDR_PRODUCT_VERSION`).
///
/// Returns e.g. `"D646_"` for a D646WP.  The trailing `_` indicates a variant
/// suffix (WP = waterproof, B = brushless, etc.) that is a physical product
/// label only — it is not encoded further in firmware registers.
///
/// Algorithm reverse-engineered from `frmWinDxxx.cs`, lines 11956–12062.
pub fn decode_model_name(product_no: u16, product_version: u16) -> String {
    let mut text = "";
    let mut text2 = String::new();
    let mut text3 = "";
    let mut flag = false;

    let num2 = product_version & 0xF000;
    let num3 = product_version & 0x00F0;
    if num2 != 0 {
        text3 = "_";
    }
    if num2 == 0x2000 && num3 == 0x30 {
        flag = true;
    }

    let num5 = product_no / 10000;
    let num6 = product_no % 10000;
    let mut flag2 = false;

    match num5 {
        0 => {
            text = "D";
            text2 = product_no.to_string();
        }
        1 => {
            text = "MD";
            text2 = num6.to_string();
        }
        2 => {
            text = "SERVO";
            text2 = char::from_u32(num6 as u32)
                .map(|c| c.to_string())
                .unwrap_or_default();
        }
        3 => {
            let num7 = num6 / 1000;
            text2 = (num6 % 1000).to_string();
            text3 = "_";
            text = match num7 {
                0 => "DB",
                1 => "X",
                2 => "MDB",
                3 | 4 | 6 => "D",
                5 | 7 => "MD",
                _ => "",
            };
        }
        4 => {
            text = "SG";
            text2 = num6.to_string();
            flag2 = true;
        }
        5 if num6 == 1 => {
            text = "HLS";
            text2 = "12".to_string();
        }
        _ => {}
    }

    let prefix = if flag && !flag2 && !text.is_empty() && !text.ends_with('B') {
        format!("{text}B")
    } else {
        text.to_string()
    };

    let result = format!("{prefix}{text2}{text3}");
    if result.is_empty() {
        format!("Unknown({product_no})")
    } else {
        result
    }
}

// ── Pure protocol functions ───────────────────────────────────────────────────

/// Build a 5-byte read query: `[0x96, servo_id, addr, 0x00, checksum]`.
pub fn build_read_query(servo_id: u8, addr: u8) -> [u8; 5] {
    let cs = servo_id.wrapping_add(addr);
    [HDR_QUERY, servo_id, addr, OP_READ, cs]
}

/// Build a 7-byte write command: `[0x96, servo_id, addr, 0x02, lsb, msb, checksum]`.
///
/// Both reads and writes use little-endian byte order: byte\[4\] = LSB,
/// byte\[5\] = MSB.  (The DPC firmware calls these "high" and "low" respectively,
/// which is backwards from conventional terminology.)
pub fn build_write_cmd(servo_id: u8, addr: u8, value: u16) -> [u8; 7] {
    let lsb = value as u8;
    let msb = (value >> 8) as u8;
    let cs = servo_id
        .wrapping_add(addr)
        .wrapping_add(OP_WRITE)
        .wrapping_add(lsb)
        .wrapping_add(msb);
    [HDR_QUERY, servo_id, addr, OP_WRITE, lsb, msb, cs]
}

/// Validate a 7-byte read response and return the 16-bit register value.
///
/// Response layout: `[0x69, mystery, addr, 0x02, lsb, msb, checksum]`.
/// The value is little-endian: `lsb | (msb << 8)`.
pub fn parse_read_response(addr: u8, data: &[u8]) -> Result<u16, Error> {
    if data.len() < 7 {
        return Err(Error::ShortResponse { got: data.len(), expected: 7 });
    }
    let (hdr, mystery, addr2, _op, lsb, msb, cs) =
        (data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
    if hdr != HDR_REPLY {
        return Err(Error::BadHeader { got: hdr, expected: HDR_REPLY });
    }
    if addr2 != addr {
        return Err(Error::AddrMismatch { got: addr2, expected: addr });
    }
    let expected_cs = mystery
        .wrapping_add(addr2)
        .wrapping_add(OP_WRITE)
        .wrapping_add(lsb)
        .wrapping_add(msb);
    if cs != expected_cs {
        return Err(Error::ChecksumMismatch { got: cs, expected: expected_cs });
    }
    Ok((lsb as u16) | ((msb as u16) << 8))
}

// ── Transport (requires feature "transport") ──────────────────────────────────

#[cfg(feature = "transport")]
mod transport_impl {
    use super::*;
    use serialport::SerialPort;
    use std::io::{Read, Write};
    use std::time::Duration;

    /// Direct D-series servo transport for Mac / Linux / Raspberry Pi.
    ///
    /// Requires a USB-serial TTL adapter and a single-transistor signal
    /// inverter between the adapter's TX/RX lines and the servo signal wire.
    ///
    /// # Echo cancellation
    ///
    /// Because TX and RX share the same physical wire, every byte transmitted
    /// is echoed back on RX.  [`DSeriesTransport`] reads and discards the echo
    /// bytes by default (`echo_cancel = true`).  Disable it only if your
    /// adapter suppresses its own TX echo in hardware.
    pub struct DSeriesTransport {
        port: Box<dyn SerialPort>,
        servo_id: u8,
        echo_cancel: bool,
    }

    impl DSeriesTransport {
        /// Open a serial port and return a transport ready to use.
        ///
        /// `servo_id` is the servo bus address (factory default: 0).
        pub fn new(path: &str, servo_id: u8) -> Result<Self, Error> {
            let port = serialport::new(path, BAUD)
                .timeout(Duration::from_millis(100))
                .open()?;
            Ok(Self { port, servo_id, echo_cancel: true })
        }

        /// Override the echo-cancel setting (builder style).
        pub fn with_echo_cancel(mut self, echo_cancel: bool) -> Self {
            self.echo_cancel = echo_cancel;
            self
        }

        /// Read register `addr` and return its 16-bit value.
        pub fn read_register(&mut self, addr: u8) -> Result<u16, Error> {
            let query = build_read_query(self.servo_id, addr);
            self.port.clear(serialport::ClearBuffer::All)?;
            self.port.write_all(&query)?;
            self.port.flush()?;
            if self.echo_cancel {
                let mut echo = [0u8; 5];
                self.read_exact_timeout(&mut echo)?;
            }
            let mut buf = [0u8; 7];
            self.read_exact_timeout(&mut buf)?;
            parse_read_response(addr, &buf)
        }

        /// Write a 16-bit `value` to register `addr`.
        ///
        /// Write commands are fire-and-forget; the servo does not acknowledge them.
        pub fn write_register(&mut self, addr: u8, value: u16) -> Result<(), Error> {
            let cmd = build_write_cmd(self.servo_id, addr, value);
            self.port.clear(serialport::ClearBuffer::All)?;
            self.port.write_all(&cmd)?;
            self.port.flush()?;
            std::thread::sleep(Duration::from_millis(5));
            Ok(())
        }

        /// Command the servo to move to `value` (write `POSITION_NEW`, addr 30).
        ///
        /// The position range depends on the servo model; typical D-series range
        /// is 0–4095 (12-bit encoder).  Values outside configured position limits
        /// are clamped in servo firmware.
        pub fn send_position(&mut self, value: u16) -> Result<(), Error> {
            self.write_register(regs::POSITION_NEW, value)
        }

        /// Read `product_no` and `product_version` and decode the model name.
        pub fn read_model_name(&mut self) -> Result<String, Error> {
            let pno = self.read_register(regs::PRODUCT_NO)?;
            let pver = self.read_register(regs::PRODUCT_VERSION)?;
            Ok(decode_model_name(pno, pver))
        }

        /// Persist the current configuration to the servo's flash memory.
        pub fn save_config(&mut self) -> Result<(), Error> {
            self.write_register(regs::CONFIG_SAVE, 1)
        }

        /// Restore all settings to factory defaults (irreversible).
        pub fn restore_factory_defaults(&mut self) -> Result<(), Error> {
            self.write_register(regs::FACTORY_DEFAULT, 1)
        }

        fn read_exact_timeout(&mut self, buf: &mut [u8]) -> Result<(), Error> {
            let mut total = 0;
            while total < buf.len() {
                match self.port.read(&mut buf[total..]) {
                    Ok(0) => return Err(Error::Timeout),
                    Ok(n) => total += n,
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                        return Err(Error::Timeout)
                    }
                    Err(e) => return Err(Error::Io(e)),
                }
            }
            Ok(())
        }
    }
}

#[cfg(feature = "transport")]
pub use transport_impl::DSeriesTransport;

// ── Named register table ──────────────────────────────────────────────────────

/// All D-series registers in display order, as `(name, address)` pairs.
///
/// Used by the CLI for `dump` output and register-name lookup.
pub const ALL_REGS: &[(&str, u8)] = &[
    // Telemetry (read-only)
    ("product_no",              regs::PRODUCT_NO),
    ("product_version",         regs::PRODUCT_VERSION),
    ("firmware_version",        regs::FIRMWARE_VERSION),
    ("ic_serial_sub",           regs::IC_SERIAL_SUB),
    ("ic_serial_main",          regs::IC_SERIAL_MAIN),
    ("status",                  regs::STATUS),
    ("position",                regs::POSITION),
    ("velocity",                regs::VELOCITY),
    ("torque",                  regs::TORQUE),
    ("voltage",                 regs::VOLTAGE),
    ("temperature",             regs::TEMPERATURE),
    ("current",                 regs::CURRENT),
    ("position_new",            regs::POSITION_NEW),
    ("position_in_new",         regs::POSITION_IN_NEW),
    // Parameter version
    ("param_version_1",         regs::PARAM_VERSION_1),
    ("param_version_2",         regs::PARAM_VERSION_2),
    // User data
    ("user_1",                  regs::USER_1),
    ("user_2",                  regs::USER_2),
    ("user_3",                  regs::USER_3),
    // Identity / bus config
    ("servo_type",              regs::SERVO_TYPE),
    ("servo_id",                regs::SERVO_ID),
    ("baudrate",                regs::BAUDRATE),
    ("signal_mode",             regs::SIGNAL_MODE),
    ("simple_return_delay",     regs::SIMPLE_RETURN_DELAY),
    ("normal_return_delay",     regs::NORMAL_RETURN_DELAY),
    // SmartSense
    ("vib_sign_change_margin",  regs::VIB_SIGN_CHANGE_MARGIN),
    ("vib_min_max_margin",      regs::VIB_MIN_MAX_MARGIN),
    ("vib_good_check_no",       regs::VIB_GOOD_CHECK_NO),
    ("vib_speed_check_no",      regs::VIB_SPEED_CHECK_NO),
    ("vib_d_gain_min",          regs::VIB_D_GAIN_MIN),
    // Power / safety
    ("power_config",            regs::POWER_CONFIG),
    ("emergency",               regs::EMERGENCY),
    ("action_mode",             regs::ACTION_MODE),
    ("failsafe",                regs::FAILSAFE),
    // Motion limits
    ("deadband",                regs::DEADBAND),
    ("position_max",            regs::POSITION_MAX),
    ("position_min",            regs::POSITION_MIN),
    ("velocity_max",            regs::VELOCITY_MAX),
    ("torque_max",              regs::TORQUE_MAX),
    ("voltage_max",             regs::VOLTAGE_MAX),
    ("voltage_min",             regs::VOLTAGE_MIN),
    ("temperature_max",         regs::TEMPERATURE_MAX),
    ("direction",               regs::DIRECTION),
    // Motion tuning
    ("start_speed",             regs::START_SPEED),
    ("power_down_time",         regs::POWER_DOWN_TIME),
    ("position_slope",          regs::POSITION_SLOPE),
    ("vib_deadband_min",        regs::VIB_DEADBAND_MIN),
    ("vib_deadband_max",        regs::VIB_DEADBAND_MAX),
    ("vib_deadband_delay",      regs::VIB_DEADBAND_DELAY),
    ("vib_deadband_p_gain",     regs::VIB_DEADBAND_P_GAIN),
    // Control
    ("factory_default",         regs::FACTORY_DEFAULT),
    ("config_save",             regs::CONFIG_SAVE),
    ("lock",                    regs::LOCK),
    // Protocol timing
    ("rx_byte_interval",        regs::RX_BYTE_INTERVAL),
    ("simple_return_limit_time",regs::SIMPLE_RETURN_LIMIT_TIME),
    ("pwm_ext_delay_time",      regs::PWM_EXT_DELAY_TIME),
    ("pwm_normal_ack_min",      regs::PWM_NORMAL_ACK_MIN),
    ("pwm_normal_hold_limit",   regs::PWM_NORMAL_HOLD_LIMIT),
    // Motor hardware
    ("motor_turn_direction",    regs::MOTOR_TURN_DIRECTION),
    ("motor_pwm_period",        regs::MOTOR_PWM_PERIOD),
    ("motor_pwm_deadtime",      regs::MOTOR_PWM_DEADTIME),
    // PID
    ("pid_p",                   regs::PID_P),
    ("pid_d",                   regs::PID_D),
    ("pid_i",                   regs::PID_I),
    ("pid_deadband",            regs::PID_DEADBAND),
    ("pos_min_max_margin",      regs::POS_MIN_MAX_MARGIN),
    // Calibration
    ("position_4095",           regs::POSITION_4095),
    ("position_0",              regs::POSITION_0),
    // Lock
    ("pos_lock_limit",          regs::POS_LOCK_LIMIT),
    ("pos_lock_time",           regs::POS_LOCK_TIME),
    ("pos_lock_ratio",          regs::POS_LOCK_RATIO),
    ("torque_lock_time",        regs::TORQUE_LOCK_TIME),
    ("temper_lock_limit",       regs::TEMPER_LOCK_LIMIT),
    ("temper_lock_time",        regs::TEMPER_LOCK_TIME),
    // Sampling / ADC
    ("pid_sampling_time",       regs::PID_SAMPLING_TIME),
    ("velocity_sampling_time",  regs::VELOCITY_SAMPLING_TIME),
    ("hum_sampling_time",       regs::HUM_SAMPLING_TIME),
    ("adc_sampling_time",       regs::ADC_SAMPLING_TIME),
    // Temperature calibration
    ("temper_25_deg",           regs::TEMPER_25_DEG),
    ("temper_50_deg",           regs::TEMPER_50_DEG),
    // EPA calibration
    ("pos_pwm_max",             regs::POS_PWM_MAX),
    ("pos_pwm_min",             regs::POS_PWM_MIN),
    ("pos_virtual_bit",         regs::POS_VIRTUAL_BIT),
    ("pos_oversampling_bit",    regs::POS_OVERSAMPLING_BIT),
    ("pid_pos_para_p_gain",     regs::PID_POS_PARA_P_GAIN),
    ("motor_deadband_offset",   regs::MOTOR_DEADBAND_OFFSET),
    ("i_comp_max",              regs::I_COMP_MAX),
    ("motor_pwm_prescaler",     regs::MOTOR_PWM_PRESCALER),
    ("pos_pwm_mid",             regs::POS_PWM_MID),
    ("pwm_in_signal_range",     regs::PWM_IN_SIGNAL_RANGE),
    ("sys_config",              regs::SYS_CONFIG),
    // Extended
    ("serial_sub",              regs::SERIAL_SUB),
    ("vib_check_max_no",        regs::VIB_CHECK_MAX_NO),
    ("vib_pwm_in_deadband",     regs::VIB_PWM_IN_DEADBAND),
    ("vib_pwm_good_no",         regs::VIB_PWM_GOOD_NO),
    ("pid_p_vib",               regs::PID_P_VIB),
    ("pid_d_vib",               regs::PID_D_VIB),
    // Target position limits
    ("pos_target_limit_max",    regs::POS_TARGET_LIMIT_MAX),
    ("pos_target_limit_min",    regs::POS_TARGET_LIMIT_MIN),
    ("rx_packet_interval",      regs::RX_PACKET_INTERVAL),
    // Advanced / miscellaneous
    ("torque_min",              regs::TORQUE_MIN),
    ("pid_gain",                regs::PID_GAIN),
    ("time_run",                regs::TIME_RUN),
    ("pid_config",              regs::PID_CONFIG),
    ("pz",                      regs::PZ),
    ("fw_ver",                  regs::FW_VER),
    ("ppm_conf_2",              regs::PPM_CONF_2),
    ("speed_conf",              regs::SPEED_CONF),
    ("sys_config_2",            regs::SYS_CONFIG_2),
    ("ppm_conf",                regs::PPM_CONF),
];

/// Resolve a name (or decimal / `0x`-hex address) to `(name, addr)`.
pub fn resolve_reg(token: &str) -> Result<(&'static str, u8), String> {
    let normalised = token.to_ascii_lowercase().replace('-', "_");
    if let Some(&(name, addr)) = ALL_REGS.iter().find(|(n, _)| *n == normalised) {
        return Ok((name, addr));
    }
    let addr = if let Some(hex) = token.strip_prefix("0x").or_else(|| token.strip_prefix("0X")) {
        u8::from_str_radix(hex, 16).ok()
    } else {
        token.parse::<u8>().ok()
    };
    if let Some(addr) = addr {
        let name = ALL_REGS.iter().find(|(_, a)| *a == addr).map(|(n, _)| *n).unwrap_or("(raw)");
        return Ok((name, addr));
    }
    Err(format!(
        "Unknown register {token:?}. Known names: {}",
        ALL_REGS.iter().map(|(n, _)| *n).collect::<Vec<_>>().join(", ")
    ))
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── build_read_query ──────────────────────────────────────────────────────

    #[test]
    fn read_query_header_and_op() {
        let q = build_read_query(0, 0x0C);
        assert_eq!(q[0], HDR_QUERY);
        assert_eq!(q[3], OP_READ);
    }

    #[test]
    fn read_query_length() {
        assert_eq!(build_read_query(0, 0).len(), 5);
    }

    #[test]
    fn read_query_checksum() {
        let q = build_read_query(0, 0x0C);
        assert_eq!(q[4], 0x0C);
    }

    #[test]
    fn read_query_matches_known_vector() {
        assert_eq!(build_read_query(0, 0x0C), [0x96, 0x00, 0x0C, 0x00, 0x0C]);
    }

    #[test]
    fn read_query_includes_servo_id_in_checksum() {
        let q0 = build_read_query(0, 0x0C);
        let q1 = build_read_query(1, 0x0C);
        assert_ne!(q0[4], q1[4]);
    }

    // ── build_write_cmd ───────────────────────────────────────────────────────

    #[test]
    fn write_cmd_length() {
        assert_eq!(build_write_cmd(0, 0, 0).len(), 7);
    }

    #[test]
    fn write_cmd_little_endian() {
        // byte[4] = LSB, byte[5] = MSB
        let c = build_write_cmd(0, 0x0A, 0x0102);
        assert_eq!(c[4], 0x02, "byte[4] should be LSB");
        assert_eq!(c[5], 0x01, "byte[5] should be MSB");
    }

    #[test]
    fn write_cmd_checksum() {
        let c = build_write_cmd(0, 0x0A, 0x0102);
        // cs = (servo_id + addr + OP_WRITE + lsb + msb) & 0xFF
        let expected = 0u8
            .wrapping_add(0x0A)
            .wrapping_add(OP_WRITE)
            .wrapping_add(0x02) // lsb
            .wrapping_add(0x01); // msb
        assert_eq!(c[6], expected);
    }

    #[test]
    fn write_cmd_small_value() {
        // value=2: lsb=2, msb=0
        let c = build_write_cmd(0, 78, 2);
        assert_eq!(c[4], 2);
        assert_eq!(c[5], 0);
    }

    #[test]
    fn write_cmd_large_value() {
        // value=0x0BB8 (3000): lsb=0xB8, msb=0x0B
        let c = build_write_cmd(0, 30, 0x0BB8);
        assert_eq!(c[4], 0xB8);
        assert_eq!(c[5], 0x0B);
    }

    // ── parse_read_response ───────────────────────────────────────────────────

    fn make_response(addr: u8, value: u16) -> [u8; 7] {
        let lsb = value as u8;
        let msb = (value >> 8) as u8;
        let mystery: u8 = 0;
        let cs = mystery
            .wrapping_add(addr)
            .wrapping_add(OP_WRITE)
            .wrapping_add(lsb)
            .wrapping_add(msb);
        [HDR_REPLY, mystery, addr, OP_WRITE, lsb, msb, cs]
    }

    #[test]
    fn parse_valid_zero() {
        let resp = make_response(0x0C, 0);
        assert_eq!(parse_read_response(0x0C, &resp).unwrap(), 0);
    }

    #[test]
    fn parse_valid_nonzero() {
        let resp = make_response(0x0C, 2048);
        assert_eq!(parse_read_response(0x0C, &resp).unwrap(), 2048);
    }

    #[test]
    fn parse_little_endian() {
        let resp = make_response(0x0C, 0x0102);
        assert_eq!(parse_read_response(0x0C, &resp).unwrap(), 0x0102);
    }

    #[test]
    fn parse_bad_header() {
        let mut resp = make_response(0x0C, 100);
        resp[0] = 0x00;
        assert!(matches!(
            parse_read_response(0x0C, &resp),
            Err(Error::BadHeader { .. })
        ));
    }

    #[test]
    fn parse_short_response() {
        assert!(matches!(
            parse_read_response(0x0C, &[0x69, 0x00, 0x0C]),
            Err(Error::ShortResponse { .. })
        ));
    }

    #[test]
    fn parse_addr_mismatch() {
        let resp = make_response(0x0C, 100);
        assert!(matches!(
            parse_read_response(0x0E, &resp),
            Err(Error::AddrMismatch { .. })
        ));
    }

    #[test]
    fn parse_bad_checksum() {
        let mut resp = make_response(0x0C, 100);
        resp[6] ^= 0xFF;
        assert!(matches!(
            parse_read_response(0x0C, &resp),
            Err(Error::ChecksumMismatch { .. })
        ));
    }

    // ── decode_model_name ─────────────────────────────────────────────────────

    #[test]
    fn model_name_d646wp() {
        // product_no=34646 (0x8756), product_version=4097 (0x1001)
        // num5=3, num6=4646, num7=4 → "D" + "646" + "_"
        assert_eq!(decode_model_name(34646, 4097), "D646_");
    }

    #[test]
    fn model_name_variant_flag_sets_suffix() {
        // product_version high nibble nonzero → text3 = "_"
        // product_no=10646 → num5=1 → "MD" + "646"; text3="_"
        assert_eq!(decode_model_name(10646, 0x1000), "MD646_");
    }

    #[test]
    fn model_name_b_series() {
        // flag (B-series): product_version & 0xF000 == 0x2000 and & 0xF0 == 0x30
        // product_no=10646 → "MD" → flag adds "B" → "MDB646_"
        assert_eq!(decode_model_name(10646, 0x2030), "MDB646_");
    }

    #[test]
    fn model_name_unknown() {
        // num5=6 falls through all match arms → empty result → "Unknown(...)"
        assert_eq!(decode_model_name(60001, 0), "Unknown(60001)");
    }

    // ── baudrate helpers ──────────────────────────────────────────────────────

    #[test]
    fn decode_baudrate_default() {
        assert_eq!(decode_baudrate(5), Some(115_200));
    }

    #[test]
    fn encode_baudrate_roundtrip() {
        for &(code, baud) in BAUDRATE_CODES {
            assert_eq!(encode_baudrate(baud), Some(code));
            assert_eq!(decode_baudrate(code as u16), Some(baud));
        }
    }

    // ── register addresses ────────────────────────────────────────────────────

    #[test]
    fn position_register_address() {
        assert_eq!(regs::POSITION, 12);
    }

    #[test]
    fn all_reg_addrs_even() {
        for &(name, addr) in ALL_REGS {
            assert_eq!(addr % 2, 0, "register {name} (0x{addr:02X}) is not even");
        }
    }

    #[test]
    fn all_reg_names_unique() {
        let mut names: Vec<&str> = ALL_REGS.iter().map(|(n, _)| *n).collect();
        let before = names.len();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), before, "duplicate register names in ALL_REGS");
    }

    #[test]
    fn all_reg_addrs_unique() {
        let mut addrs: Vec<u8> = ALL_REGS.iter().map(|(_, a)| *a).collect();
        let before = addrs.len();
        addrs.sort_unstable();
        addrs.dedup();
        assert_eq!(addrs.len(), before, "duplicate register addresses in ALL_REGS");
    }
}
