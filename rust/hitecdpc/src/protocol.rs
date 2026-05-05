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

//! HS-5/7XXX packet framing — host ↔ DPC adapter protocol.
//!
//! This layer sits between the host PC and the DPC-11/20 adapter.
//! The adapter in turn forwards commands to the servo.
//!
//! # Packet anatomy
//!
//! Every exchange uses a 4-byte servo command packet:
//! ```text
//! [CMD, ADDR, DATA, CSUM]   where CSUM = (256 − (CMD+ADDR+DATA) % 256) % 256
//! ```
//!
//! In DPC-20 mode the packet is wrapped in a KSO frame and then in an
//! STX/ETX frame before being sent over the wire.

use crate::crc::crc8;

// ── Command codes ─────────────────────────────────────────────────────────────

/// Read 8-bit SRAM register; response data at offset 4 of the reply.
pub const CMD_READ_8: u8 = 0x61; // 'a'
/// Write 8-bit SRAM register (addr 0–127).
pub const CMD_WRITE_8: u8 = 0x62; // 'b'
/// Write 8-bit EEPROM register (addr 128–255).
pub const CMD_WRITE_EE: u8 = 0x64; // 'd'
/// Read 16-bit register; response at offsets 4–5.
pub const CMD_READ_16: u8 = 0x65; // 'e'
/// Set servo position (16-bit PWM value).
pub const CMD_SET_POS: u8 = 0x66; // 'f'
/// Read firmware version; response at offset 4.
pub const CMD_READ_VER: u8 = 0x67; // 'g'

// ── STX/ETX mux values ───────────────────────────────────────────────────────

/// Probe / handshake packets.
pub const MUX_HANDSHAKE: u8 = 0;
/// Host → adapter command packets.
pub const MUX_COMMAND: u8 = 7;
/// Adapter → host response packets.
pub const MUX_RESPONSE: u8 = 11;

// ── DPC-20 message constants ──────────────────────────────────────────────────

pub const MSG_PROBE: &[u8] = b"KWAU";
pub const MSG_RESET: &[u8] = b":A:A:A";
pub const MSG_3PIN_57: &[u8] = b"KP3S5";
pub const MSG_3PIN_9: &[u8] = b"KP3S9";
pub const MSG_3PIN: &[u8] = b"KP3S";
pub const MSG_4PIN: &[u8] = b"KP4S";

pub const KEY_APP: &[u8] = b"kAPP";
pub const KEY_RS3: &[u8] = b"kRs3";
pub const KEY_IBR: &[u8] = b"kIBR";
pub const KEY_IBW: &[u8] = b"kIBW";
pub const KEY_IBU: &[u8] = b"kIBU";

// ── Pure packet functions ─────────────────────────────────────────────────────

/// Build a 4-byte servo command packet `[CMD, ADDR, DATA, CSUM]`.
pub fn servo_packet(cmd: u8, addr: u8, data: u8) -> [u8; 4] {
    let sum = (cmd as u16) + (addr as u16) + (data as u16);
    let csum = ((256 - sum % 256) % 256) as u8;
    [cmd, addr, data, csum]
}

/// Build a position-control packet. `pwm_raw` = microseconds × 4.
pub fn position_packet(pwm_raw: u16) -> [u8; 4] {
    let hi = (pwm_raw >> 8) as u8;
    let lo = pwm_raw as u8;
    let sum = (CMD_SET_POS as u16) + (hi as u16) + (lo as u16);
    let csum = ((256 - sum % 256) % 256) as u8;
    [CMD_SET_POS, hi, lo, csum]
}

/// Wrap `payload` in an STX/ETX frame: `[STX, payload…, CRC8, len, ETX]`.
pub fn stxetx_frame(payload: &[u8], mux: u8) -> Vec<u8> {
    let stx = 2u8.wrapping_add(mux);
    let etx = 3u8.wrapping_add(mux);
    let checksum = crc8(payload);
    let len = payload.len() as u8;
    let mut frame = Vec::with_capacity(payload.len() + 4);
    frame.push(stx);
    frame.extend_from_slice(payload);
    frame.push(checksum);
    frame.push(len);
    frame.push(etx);
    frame
}

/// Scan `data` for a valid STX/ETX frame and return its payload, or `None`.
pub fn parse_stxetx(data: &[u8], mux: u8) -> Option<Vec<u8>> {
    let stx = 2u8.wrapping_add(mux);
    let etx = 3u8.wrapping_add(mux);
    for i in 4..data.len() {
        if data[i] != etx {
            continue;
        }
        let length = data[i - 1] as usize;
        let checksum = data[i - 2];
        let stx_pos = i.checked_sub(3 + length)?;
        if data[stx_pos] != stx {
            continue;
        }
        let payload = &data[stx_pos + 1..stx_pos + 1 + length];
        if payload.len() == length && crc8(payload) == checksum {
            return Some(payload.to_vec());
        }
    }
    None
}

/// Build the DPC-20 KSO3/KSO4 wrapper used by `send_serial_packet`.
///
/// Servo bytes are bit-inverted inside the wrapper. The returned bytes
/// are then passed to [`stxetx_frame`] with `mux = MUX_COMMAND`.
pub fn kso_wrapper(servo_bytes: &[u8], four_pin: bool, want_reply: bool) -> Vec<u8> {
    let pin_byte: u8 = if four_pin { 0x34 } else { 0x33 }; // '4' or '3'
    let ret: [u8; 3] = if want_reply { [10, 10, 20] } else { [0, 0, 0] };
    let mut out = Vec::with_capacity(5 + servo_bytes.len() + 3);
    out.extend_from_slice(&[0x4B, 0x53, 0x4F, pin_byte, servo_bytes.len() as u8]);
    out.extend(servo_bytes.iter().map(|&b| !b));
    out.extend_from_slice(&ret);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn servo_packet_checksum_valid() {
        let pkt = servo_packet(0x61, 0x02, 0x00);
        let sum: u16 = pkt.iter().map(|&b| b as u16).sum();
        assert_eq!(sum % 256, 0, "packet bytes must sum to 0 mod 256");
    }

    #[test]
    fn position_packet_checksum_valid() {
        let pkt = position_packet(6000);
        let sum: u16 = pkt.iter().map(|&b| b as u16).sum();
        assert_eq!(sum % 256, 0);
    }

    #[test]
    fn stxetx_round_trip_mux0() {
        let payload = b"hello";
        let frame = stxetx_frame(payload, MUX_HANDSHAKE);
        let recovered = parse_stxetx(&frame, MUX_HANDSHAKE).unwrap();
        assert_eq!(recovered, payload);
    }

    #[test]
    fn stxetx_round_trip_mux7() {
        let payload = servo_packet(0x61, 0x02, 0x00);
        let frame = stxetx_frame(&payload, MUX_COMMAND);
        let recovered = parse_stxetx(&frame, MUX_COMMAND).unwrap();
        assert_eq!(recovered, payload);
    }

    #[test]
    fn parse_stxetx_finds_frame_in_larger_buffer() {
        let payload = b"test";
        let frame = stxetx_frame(payload, MUX_RESPONSE);
        let mut buf = vec![0xDE, 0xAD, 0xBE, 0xEF];
        buf.extend_from_slice(&frame);
        buf.extend_from_slice(&[0xFF, 0xFF]);
        let recovered = parse_stxetx(&buf, MUX_RESPONSE).unwrap();
        assert_eq!(recovered, payload);
    }

    #[test]
    fn parse_stxetx_wrong_mux_returns_none() {
        let frame = stxetx_frame(b"data", MUX_COMMAND);
        assert!(parse_stxetx(&frame, MUX_RESPONSE).is_none());
    }

    #[test]
    fn kso_wrapper_inverts_bytes() {
        let pkt = servo_packet(0x61, 0x02, 0x00);
        let wrapped = kso_wrapper(&pkt, false, true);
        // Bytes 5..5+len are the inverted servo bytes
        for (i, &orig) in pkt.iter().enumerate() {
            assert_eq!(wrapped[5 + i], !orig);
        }
    }

    #[test]
    fn kso_wrapper_pin_byte() {
        let w3 = kso_wrapper(b"\x00", false, true);
        let w4 = kso_wrapper(b"\x00", true, true);
        assert_eq!(w3[3], 0x33); // '3'
        assert_eq!(w4[3], 0x34); // '4'
    }
}
