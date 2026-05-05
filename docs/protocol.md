# Hitec DPC Serial Protocol Specification

Recovered from static analysis of `dpc_57.exe` (HS-5/7XXX programmer, v3.2.1).
All fields are confirmed from decompiled C# source. Entries marked **inferred** are
deduced from context; all others are directly read from the source.

---

## 1. Transport

### 1.1 DPC-11 (USB direct, CP210x)

The DPC-11 hardware is a Silicon Labs CP210x USB-to-UART bridge.
The host communicates via `SiUSBXp.DLL`:

| DLL function | Purpose |
|---|---|
| `SI_GetNumDevices` | Enumerate attached DPC-11 units |
| `SI_GetProductString` | Verify product string (`hitec_bl_servo_interface_model_#_001`) |
| `SI_Open` / `SI_Close` | Open/close device handle |
| `SI_SetBaudRate(handle, 19200)` | Configure UART baud rate |
| `SI_SetFlowControl` | RTS/CTS/DTR/DSR flow control |
| `SI_Write(handle, buf, 4, ...)` | Write 4-byte command packet |
| `SI_CheckRXQueue` | Poll for received bytes |
| `SI_Read(handle, buf, n, ...)` | Read response bytes |
| `SI_FlushBuffers` | Flush TX/RX buffers |

**Baud rate:** 19200 bps (the "57" in `dpc_57.exe` refers to the HS-5/7XXX servo
family, not the baud rate).

**Packet mode:** Raw 4-byte packets — no STX/ETX framing — sent directly via
`SI_Write`. Responses arrive as raw bytes via `SI_Read`.

### 1.2 DPC-20 / DPC-485 (Serial port)

These adapters use a standard COM port via `System.IO.Ports.SerialPort`. All
packets are wrapped in the STX/ETX frame described in §2.

---

## 2. STX/ETX Frame Format (DPC-20 / DPC-485)

```
Offset  Field
  0     STX  = (2 + mux) & 0xFF
  1..N  payload bytes
  N+1   CRC8 over bytes [1..N], init=0xFF, poly=0x8C
  N+2   N (payload length, 1 byte)
  N+3   ETX  = (3 + mux) & 0xFF
```

`mux` is a framing mode selector. Three values appear in the codebase:

| mux | STX  | ETX  | Usage |
|-----|------|------|-------|
|  0  | 0x02 | 0x03 | Handshake/probe packets (raw command strings) |
|  7  | 0x09 | 0x0A | Host→adapter command packets |
| 11  | 0x0D | 0x0E | Adapter→host servo response packets |

### CRC-8 algorithm

```
crc_8(crc, byte):
    for bit in 0..7:
        xor_bit = (crc ^ byte) & 1
        crc >>= 1
        if xor_bit: crc ^= 0x8C
        byte >>= 1
    return crc
```

This is the reflected Dallas/Maxim 1-Wire CRC-8 (polynomial x⁸+x⁵+x⁴+1).
Initial value: `0xFF`.

---

## 3. Servo Command Packet

The 4-byte inner payload sent to the servo for every register operation:

```
Byte  Field     Description
  0   CMD       Command code (ASCII letter, see §4)
  1   ADDR      Register address (0–255)
  2   DATA      Data value (0–255)
  3   CSUM      Checksum = (256 - ((CMD + ADDR + DATA) % 256)) % 256
```

The packet is retransmitted **3 times** per operation (i = 0, 1, 2) with a
5 ms inter-packet sleep.

For DPC-11: sent raw via `SI_Write(handle, write, 4)`.
For DPC-20: wrapped in a `KSO3` / `KSO4` wrapper (§5) then STX/ETX framed (mux=7).

---

## 4. Command Codes

| Byte | ASCII | Operation | Bank | Response |
|------|-------|-----------|------|----------|
| 0x61 | `a`   | Read 8-bit register   | SRAM (addr 0–127) | return_p[4] = value |
| 0x62 | `b`   | Write 8-bit register  | SRAM (addr 0–127) | ACK only |
| 0x64 | `d`   | Write 8-bit register  | EEPROM (addr 128–255) | ACK only |
| 0x65 | `e`   | Read 16-bit register  | SRAM              | (return_p[4]×256 + return_p[5]) |
| 0x66 | `f`   | Set servo position    | —                 | ACK only |
| 0x67 | `g`   | Read firmware version | —                 | return_p[4] = version |

**Position command (0x66):** DATA field is unused; instead `write[1]` and
`write[2]` hold the high and low bytes of the 16-bit PWM value (raw ×4 of
microseconds). Checksum covers all three bytes in the usual way.

**Write after read:** Every register write is preceded by reading all registers
with cmd `a` (0x61) to populate the local cache. After write, some registers
are written twice — once to the SRAM address (cmd `b`) and once to the
corresponding EEPROM address (cmd `d`). See §6 for the address mapping.

---

## 5. DPC-20 Serial Packet Wrapper

When the adapter is a DPC-20 (3-pin or 4-pin), the 4-byte servo packet is
embedded in an outer `KSO3`/`KSO4` wrapper before STX/ETX framing:

```
Byte     Value
  0      'K'  (0x4B)
  1      'S'  (0x53)
  2      'O'  (0x4F)          — or 'X' (0x58) for HSB-9xxx (send9)
  3      '3'  (0x33) 3-pin   — or '4' (0x34) for 4-pin
  4      count (number of servo bytes that follow)
  5..    servo bytes, each BIT-INVERTED (~b)
  +0     f_return ? 10 : 0   (return-data timeout hint)
  +1     f_return ? 10 : 0
  +2     f_return ? 20 : 0
```

This is then wrapped in STX/ETX with mux=7, giving wire bytes:
```
0x09  [KSO3 payload]  CRC8  LEN  0x0A
```

---

## 6. DPC-20 Connection / Handshake Sequence

1. **Probe:** Send `KWAU` (0x4B 0x57 0x41 0x55) in STX/ETX frame (mux=7).
2. **Parse response:**
   - Payload starts with `kAPP` → adapter ready, proceed to step 3.
   - Payload starts with `kIBR`, `kIBW`, or `kIBU` → adapter is in firmware-
     update mode. Send ASCII string `:A:A:A` on the raw serial port, close,
     then re-open and retry.
3. **Pin mode selection** (payload key sent via `send_message_packet`, mux=7):
   | Series      | Key sent |
   |-------------|----------|
   | HS-5/7XXX   | `KP3S5`  |
   | HSB-9XXX    | `KP3S9`  |
   | Generic 3-pin | `KP3S` |
   | 4-pin       | `KP4S`   |
4. After `KP3S5` or `KP3S9`, send a zero data packet:
   `send_serial_packet([0x00], count=1, f_return=false)`.

---

## 7. Response Packet (DPC-20)

Servo responses arrive framed with mux=11 (STX=0x0D, ETX=0x0E). Inside the
frame the payload begins with the 4-byte ASCII key `kRs3`, followed by the
response data:

```
Offset  Content
0–3     "kRs3"  (0x6B 0x52 0x73 0x33)
4       value_high (or 8-bit value for cmd 'a'/'g')
5       value_low  (for cmd 'e'; high×256 + low = 16-bit value)
```

The host reads responses via `read_serial_data(key="kRs3", more=11)`.

For DPC-11 (raw mode), the response arrives as raw bytes in `return_p[]`
without the `kRs3` wrapper; the value is still at `return_p[4]` (8-bit) or
`return_p[4:5]` (16-bit), consistent with the DPC-20 layout.

---

## 8. Register Map — HS-5/7XXX Series

### 8.1 SRAM Bank (addr 0–127)

Read with cmd `a` (0x61); write with cmd `b` (0x62).
Populated at connect by `read_servo()`, which loops addr 0–36.

| Addr | Name | Description | Encoding |
|------|------|-------------|----------|
| 0 | kp_min | Kp gain minimum | u8 |
| 1 | kp_max | Kp gain maximum | u8 |
| 2 | deadband | Dead band (Kpj) | u8, 1–16 |
| 3 | kd | Kd gain | u8 |
| 4 | kdj | Kd gain join | u8 |
| 5 | asccnt | Acceleration count | u8 |
| 6 | speed | Speed (Kv) | u8, 1=10%…64=100% |
| 7–8 | ioffset | Position offset | u16 BE, µs = value / 4 |
| 9–10 | smin | Minimum position | u16 BE, µs = value / 4 |
| 11–12 | smax | Maximum position | u16 BE, µs = value / 4 |
| 13–14 | imin | Input min | u16 BE |
| 15–16 | imax | Input max | u16 BE |
| 17–18 | sref | Center / reference | u16 BE, µs = value / 4 |
| 19 | apos | Angle limit (+) | u8 |
| 20 | aneg | Angle limit (−) | u8 |
| 21–22 | fs_time | Failsafe delay | u16 BE, ms |
| 23–24 | ms_time | Multiplex signal time | u16 BE |
| 25–26 | fpos | Failsafe position | u16 BE, µs = value / 4 |
| 27 | status | Flags | bit0=direction (0=CCW,1=CW), bit1+=failsafe |
| 29 | type | Servo type code | u8 |
| 32 | diss_limit | Dissipation limit | u8 |
| 33 | stell_limit | Stall/OLP limit | u8, ≥100=off, else % |
| 34 | high_reg | High-resolution mode flag | u8 |
| 35 | stretch | Stretch | u8 |
| 96–116 | model_name | Model name string | 21×ASCII, read with cmd `a` |

> The 16-bit registers at 7–8, 9–10, etc. are read as two consecutive 8-bit
> reads (addr N then addr N+1) and assembled big-endian by the host.

### 8.2 EEPROM Bank (addr 128–255)

Write only with cmd `d` (0x64). Values are persisted across power cycles.
Most registers mirror a SRAM register; the SRAM register is typically written
first, then the EEPROM copy.

| EEPROM addr | SRAM mirror | Name |
|-------------|-------------|------|
| 128 | 1 | kp_max (alt path) |
| 129 | 0 | kp_min |
| 130 | 1 | kp_max |
| 131 | 2 | deadband |
| 132 | 3 | kd |
| 133 | 4 | kdj |
| 134 | 5 | asccnt |
| 135 | 6 | speed |
| 136–137 | 7–8 | ioffset |
| 138–141 | — | (reserved / zero) |
| 146–147 | — | |
| 148–149 | 17–18 | sref (center) |
| 150 | 19 | apos |
| 151 | 20 | aneg |
| 162 | 32 | diss_limit |
| 164 | 33 | stell_limit |

---

## 9. CRC-16 (firmware upload — not register reads)

Two CRC-16 implementations exist in `modCRC16`:

### 9.1 CRC-16/MODBUS (`ComputeCrc`, table-based)

Polynomial: 0x8005 reflected. 256-entry lookup table. Used for bulk data / 
firmware upload. Not used in normal register read/write operations.

### 9.2 CRC-16/CCITT (`crc16_array` / `crc16_byte`)

Polynomial: 0x1021. Bit-by-bit. Role not yet determined from register-read code
alone; likely used for a different packet type.

---

## 10. Open Questions

- Exact framing of DPC-11 raw responses (what are `return_p[0..3]` in the
  raw response? Are they echoed command bytes, or a header?)
- Confirm which CRC-16 variant is used where (firmware upload path needs live
  capture or deeper trace of the `WriteRead_Value_w_file` methods).
- Register addresses 55–56 and 67–68 appear in write sequences but have not
  been correlated to a UI control name.
- HSB-9XXX and D-series register maps — separate forms (`frmWin9xxx`,
  `frmWinDxxx`) with similar but distinct register layouts.

---

## 11. Source Files

All findings confirmed from decompiled source at `decompiled/HitecDservoGUI/`:

| File | Contents |
|------|----------|
| `modDPC20.cs` | Frame builder, handshake, STX/ETX parser, serial I/O |
| `modCRC8.cs` | CRC-8 implementation |
| `modCRC16.cs` | CRC-16 MODBUS and CCITT implementations |
| `Module1.cs` | CP210x DLL imports, global state, com_types enum |
| `frmWin5xxx7xxx.cs` | HS-5/7XXX UI: `read_servo()`, `read_write_eep()`, all register accesses |
| `mod9xxx.cs` | HSB-9XXX servo default configuration table |
