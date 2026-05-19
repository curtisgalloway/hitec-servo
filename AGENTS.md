<!--
Copyright 2025 Curtis Galloway

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-->

# AGENTS.md — Hitec Servo Tools

## What this repo does

This project provides tools to read and configure Hitec digital servos from
any platform, without the official Windows DPC software:

- **`hitecdpc` CLI** (Rust) — the primary interface; produces structured JSON
  output suitable for agent use.  See [`docs/cli.md`](docs/cli.md).
- **Python library** (`src/hitecdpc/`) — `DSeriesTransport` for D-series
  direct connection.  DPC-20 transport is also implemented for future HS-5/7XXX
  support (`transport.py`), but the high-level servo API for those series has not
  yet been written.
- **CircuitPython module** (`src/hitecdpc/pico_dseries.py`) — runs on a
  Raspberry Pi Pico with no extra inverter hardware.

**Supported series:**

| Series | Adapter needed? | CLI support | Python support |
|--------|----------------|-------------|----------------|
| D-series (D485HW, D646WP, …) | No | Yes | Yes |
| HS-5/7XXX | DPC-11 or DPC-20 | Not yet | Not yet |
| HSB-9XXX | DPC-11 or DPC-20 | Not yet | Not yet |

---

## Hardware requirements

### D-series (direct connection)

Connect a USB-serial TTL adapter to the servo signal wire through a
**single-transistor inverter** (the D-series uses inverted-polarity UART).
See [`docs/cli.md#hardware-setup`](docs/cli.md#hardware-setup) for the
schematic — it is a 2N3904 NPN transistor and two resistors.

**Alternative:** A Raspberry Pi Pico running `pico_dseries.py` handles the
inversion in PIO firmware; no extra hardware needed beyond a 1 kΩ resistor.

### HS-5/7XXX and HSB-9XXX

A Hitec DPC-11 or DPC-20 adapter is required.  Connect it via USB; a
`/dev/ttyUSB*` or `/dev/cu.usbserial-*` port will appear.

---

## Quick start — Rust CLI

```bash
cd rust
cargo build --release
# binary is rust/target/release/hitecdpc
# or install globally:
cargo install --path hitecdpc-cli
```

```bash
# Read all registers (structured JSON)
hitecdpc --port /dev/ttyUSB0 --json dump

# Read one register
hitecdpc --port /dev/ttyUSB0 --json read position

# Write a register (in-memory)
hitecdpc --port /dev/ttyUSB0 --json write deadband 2

# Persist all settings to flash
hitecdpc --port /dev/ttyUSB0 --json save

# Stream telemetry (Ctrl-C to stop)
hitecdpc --port /dev/ttyUSB0 --json monitor --interval 0.5

# Restore factory defaults
hitecdpc --port /dev/ttyUSB0 --json factory-reset
```

**Exit codes:** 0 = success, 1 = comm error, 2 = usage error.
**JSON errors** appear as `{"error": "..."}` on stdout; human errors go to
stderr.  Always check the exit code.

See [`docs/cli.md`](docs/cli.md) for the complete reference including all
register names, JSON schema, and agent usage patterns.

---

## Quick start — Python library

```bash
uv sync     # or: pip install -e .
```

```python
from hitecdpc import DSeriesTransport

with DSeriesTransport("/dev/ttyUSB0", servo_id=0) as t:
    value, err = t.read_register("position")
    if err:
        raise RuntimeError(err)
    print("position:", value)
    t.write_register("deadband", 2)
    t.save_config()
```

Run tests:

```bash
uv run --with pyserial --with pytest pytest
```

---

## Quick start — Raspberry Pi Pico

Copy `src/hitecdpc/pico_dseries.py` to the Pico root as `pico_dseries.py`.

```python
import board
from pico_dseries import DServoComm

servo = DServoComm(signal_pin=board.GP15, servo_id=0)
value, err = servo.read_register(0x0C)   # position
servo.write_register(0x4E, 2)            # deadband
servo.save_config()
```

To identify a servo from the host, run:

```bash
uv run --with pyserial python3 scripts/identify_servo.py [PORT]
```

`PORT` defaults to `/dev/cu.usbmodem1101`.

---

## Key technical facts

### D-series wire protocol

```
Query  (host → servo, 5 bytes):  [0x96, servo_id, addr, 0x00, checksum]
  checksum = (servo_id + addr) & 0xFF

Response (servo → host, 7 bytes): [0x69, ?,  addr, 0x02, low, high, checksum]
  value    = low | (high << 8)   ← little-endian
  checksum = (? + addr + 0x02 + low + high) & 0xFF

Write   (host → servo, 7 bytes): [0x96, servo_id, addr, 0x02, high, low, checksum]
  checksum = (servo_id + addr + 0x02 + high + low) & 0xFF
  No ACK from servo; wait 5 ms before next command.
```

Physical layer: 115200 baud, 8N1, **inverted polarity** (idle LOW), half-duplex.

### Protocol variants

| Series | Baud (servo) | Adapter baud | Interface | Direct? |
|--------|-------------|-------------|-----------|---------|
| HS-5XXX / HS-7XXX | 57600 | 19200 | TTL UART via CP210x | No |
| HSB-9XXX | 9600 | 19200 | TTL UART via CP210x | No |
| D-Series / MD-Series | 115200 | — | Direct UART, inverted | **Yes** |

### Error / status word (16-bit)

```
Bit  Meaning
 0   Signal Address Error
 1   Signal Checksum Error
 2   Signal Interval Error
 3   Signal Limit Error
 4   Signal Format Error
 5   Signal Send Error
 8   Position Min Over
 9   Position Max Over
10   Temperature Over
11   Torque Over
12   Voltage Min Over
13   Voltage Max Over
15   Boot Loader active
```

### Servo addressing

- ID range 0–254; 255 is broadcast.
- Setting ID to 0 triggers EPA reset.

---

## File layout

```
src/hitecdpc/          Python library
  __init__.py          exports DSeriesTransport
  crc.py               CRC-8
  protocol.py          DPC packet framing (STX/ETX, KSO3, CRC)
  dseries.py           D-series protocol + CPython transport
  pico_dseries.py      Standalone CircuitPython module (copy to Pico)
  registers.py         HS-5/7XXX register map (metadata, no transport)
  transport.py         RawTransport, DPC20Transport

rust/                  Rust workspace
  hitecdpc/            Library crate
    src/crc.rs         CRC-8 and CRC-16
    src/protocol.rs    HS-5/7XXX framing
    src/dseries.rs     D-series protocol + register table + transport
  hitecdpc-cli/        CLI binary crate
    src/main.rs        hitecdpc binary (clap + serde_json)

tests/                 Python tests (pytest)
docs/
  protocol.md          Full wire protocol specification
  cli.md               CLI reference
scripts/
  identify_servo.py    Read D-series identity via Pico REPL
```

---

## Known limitations

- **HS-5/7XXX and HSB-9XXX are not yet implemented.** The DPC-20 transport
  (`DPC20Transport`) and HS-5xxx register metadata (`registers.py`) are in place;
  a high-level servo API still needs to be written and tested against hardware.
- **D-series requires a signal inverter** (or a Pico) — a plain USB-serial adapter will not work.
- **Servo ID 0 is the factory default** for all D-series servos.  If multiple servos are on the same bus, assign each a unique ID before connecting them together.
- **DPC-11 raw response format** — the `kRs3` header on raw DPC-11 responses is inferred from the DPC-20 path; not yet confirmed against live hardware.

---

## References

- [`docs/cli.md`](docs/cli.md) — full CLI reference
- [`docs/protocol.md`](docs/protocol.md) — wire protocol specification
- Tim Maxwell's HiTEC D-servo Arduino library — independently confirms the 0x96/0x69 wire format
