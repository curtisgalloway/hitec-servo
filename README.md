# hitec-servo

Cross-platform tools for reading and configuring Hitec digital servos without
the official Windows software.

**Status:** D-series (D485HW, D646WP, …) fully working — read, write, and
monitor servos directly without any DPC adapter.  HS-5/7XXX and HSB-9XXX
support is implemented in the Python library (requires a DPC-11 or DPC-20
adapter) but not yet in the Rust CLI.

The wire protocol was recovered from static analysis of the official DPC
software.  This project is an independent implementation; no Hitec code is
included or redistributed.

---

## Quick start — D-series servo

**Hardware you need:**

- USB-serial TTL adapter (CP2102, FT232, etc.)
- Single-transistor inverter circuit between the adapter and the servo signal
  wire (see [docs/cli.md](docs/cli.md#hardware-setup) for the schematic)
- External 5–7.4 V power supply for the servo

**Build and install the CLI:**

```bash
cd rust
cargo build --release
# or install to ~/.cargo/bin/
cargo install --path hitecdpc-cli
```

**Use it:**

```bash
# Read all registers
hitecdpc --port /dev/ttyUSB0 dump

# Read one register
hitecdpc --port /dev/ttyUSB0 read position

# Write a register and persist to flash
hitecdpc --port /dev/ttyUSB0 write deadband 2
hitecdpc --port /dev/ttyUSB0 save

# Stream live telemetry
hitecdpc --port /dev/ttyUSB0 monitor --interval 0.5

# Machine-readable JSON (for scripts and agents)
hitecdpc --port /dev/ttyUSB0 --json dump
```

See [docs/cli.md](docs/cli.md) for the full command reference, JSON output
format, and all 40+ register names.

---

## Raspberry Pi Pico (no adapter, no inverter)

Copy `src/hitecdpc/pico_dseries.py` to the Pico's filesystem.  It uses PIO
state machines to handle the inverted UART in firmware — no extra hardware
beyond a 1 kΩ resistor.

```python
from pico_dseries import DServoComm

servo = DServoComm(signal_pin=board.GP15)
value, err = servo.read_register(0x0C)   # position
servo.write_register(0x4E, 2)            # deadband
servo.save_config()
```

---

## Python library

```bash
pip install -e .   # or: uv sync
```

```python
from hitecdpc import DSeriesTransport

with DSeriesTransport("/dev/ttyUSB0", servo_id=0) as t:
    value, err = t.read_register("position")
    t.write_register("deadband", 2)
    t.save_config()
```

For HS-5/7XXX via a DPC-11 or DPC-20 adapter:

```python
from hitecdpc import HitecServo

with HitecServo("/dev/ttyUSB0", mode="raw", series="57") as servo:
    print(servo.get_model())
    servo.write_register("deadband", 5)
```

Run tests:

```bash
uv run --with pyserial --with pytest pytest
```

---

## Rust crate

```toml
# Cargo.toml
hitecdpc = { path = "rust/hitecdpc" }
```

```rust
use hitecdpc::dseries::{DSeriesTransport, regs};

let mut servo = DSeriesTransport::new("/dev/ttyUSB0", 0)?;
let position = servo.read_register(regs::POSITION)?;
servo.write_register(regs::DEADBAND, 2)?;
servo.save_config()?;
```

Run tests:

```bash
cd rust && cargo test --workspace
```

---

## How it works

The D-series wire protocol was recovered from static analysis of Hitec's DPC
software.  Key findings:

- **No adapter required.** D-series servos speak a simple binary protocol
  directly on the signal wire.  The DPC adapter only handles TTL↔USB bridging
  for other series.
- **Inverted UART.** Idle is LOW, start bit is HIGH — opposite of standard
  RS-232.  A single NPN transistor or 74HC04 gate inverts the signal.
- **Simple framing.** 5-byte query / 7-byte response, all registers 16-bit.

```
Query  → [0x96, id, addr, 0x00, checksum]
Response ← [0x69,  ?, addr, 0x02, low, high, checksum]
Write  → [0x96, id, addr, 0x02, high, low, checksum]
```

See [docs/protocol.md](docs/protocol.md) for the full specification including
HS-5/7XXX STX/ETX framing and CRC algorithms.

---

## Repository layout

```
src/hitecdpc/        Python library
  dseries.py         D-series protocol + transport
  pico_dseries.py    Standalone CircuitPython module
  servo.py           HitecServo (HS-5/7XXX, HSB-9XXX)
  protocol.py        HS-5/7XXX packet framing
  registers.py       HS-5/7XXX register map
  crc.py             CRC-8

rust/
  hitecdpc/          Rust library crate
  hitecdpc-cli/      hitecdpc CLI binary

docs/
  protocol.md        Wire protocol specification
  cli.md             CLI reference

tests/               Python tests (pytest)
scripts/
  identify_servo.py  Identify a D-series servo via Pico REPL
```

---

## License

Apache 2.0 — see [LICENSE](LICENSE).

This project is an independent reverse engineering effort and is not affiliated
with Hitec RCD.  "Hitec", "DPC", and servo model numbers are trademarks of
Hitec RCD.
