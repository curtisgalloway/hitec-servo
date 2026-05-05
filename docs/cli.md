# `hitecdpc` CLI Reference

The `hitecdpc` binary is a Rust command-line tool for reading, writing, and
monitoring Hitec digital servos directly from any host computer — no official
Windows software required.

**Source:** `rust/hitecdpc-cli/`  
**Build:** `cargo build --release` inside `rust/`  
**Install:** `cargo install --path rust/hitecdpc-cli`

---

## Synopsis

```
hitecdpc --port <PORT> [--series d] [--id N] [--json] <COMMAND> [ARGS]
```

## Global options

| Flag | Default | Description |
|------|---------|-------------|
| `--port PORT` | *(required)* | Serial port (`/dev/ttyUSB0`, `/dev/cu.usbserial-XXXX`, `COM3`) |
| `--series {d,57,9}` | `d` | Servo series (only `d` implemented; 57/9 need a DPC adapter and are not yet ported to Rust) |
| `--id N` | `0` | Servo bus ID, 0–254.  Factory default is 0. |
| `--json` | off | Emit machine-readable JSON on stdout (one line per event) |

## Exit codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | Communication error — no response, checksum failure, timeout |
| 2 | Usage error — bad port, unknown register, invalid value |

---

## Commands

### `dump`

Read all known registers and print a summary.

```
hitecdpc --port /dev/ttyUSB0 dump
hitecdpc --port /dev/ttyUSB0 --json dump
```

Human output:
```
REGISTER                 ADDR    VALUE       HEX
------------------------------------------------
product_no                  0        1  0x00000001
firmware_version            4      128  0x00000080
position                   12     2048  0x00000800
deadband                   78        2  0x00000002
...
```

JSON output (single line):
```json
{
  "command": "dump",
  "series": "d",
  "registers": {
    "position":         {"addr": 12,  "value": 2048, "error": null},
    "velocity":         {"addr": 14,  "value": 0,    "error": null},
    "deadband":         {"addr": 78,  "value": 2,    "error": null},
    "firmware_version": {"addr": 4,   "value": 128,  "error": null}
  }
}
```

Registers that time out (some are model-specific) have `"value": null` and a
non-null `"error"` string.  The exit code is still 0 if at least one read
succeeded.

---

### `read <register>`

Read a single register.  `<register>` is a name from the table below, a
decimal address, or a `0x`-prefixed hex address.

```
hitecdpc --port /dev/ttyUSB0 read position
hitecdpc --port /dev/ttyUSB0 read 12          # decimal address
hitecdpc --port /dev/ttyUSB0 read 0x0C        # hex address
hitecdpc --port /dev/ttyUSB0 --json read deadband
```

Human output:
```
position  addr=0x0C  value=2048  (0x0800)
```

JSON output:
```json
{"command": "read", "register": "position", "addr": 12, "value": 2048, "error": null}
```

---

### `write <register> <value>`

Write a 16-bit value to a register.  `<value>` is decimal or `0x` hex.
Write commands are fire-and-forget; the servo does not send an ACK.

```
hitecdpc --port /dev/ttyUSB0 write deadband 2
hitecdpc --port /dev/ttyUSB0 write deadband 0x02
hitecdpc --port /dev/ttyUSB0 --json write deadband 2
```

Human output:
```
wrote deadband (addr=0x4E) = 2
```

JSON output:
```json
{"command": "write", "register": "deadband", "addr": 78, "value": 2, "error": null}
```

**Flag:** `--no-delay` skips the 5 ms post-write settling pause.  Use with
caution — the servo needs time to apply the new value before the next command.

---

### `save`

Persist all current register values to the servo's internal flash.  Without
this, writes survive only until the servo is power-cycled.

```
hitecdpc --port /dev/ttyUSB0 save
hitecdpc --port /dev/ttyUSB0 --json save
```

JSON output:
```json
{"command": "save", "error": null}
```

---

### `factory-reset`

Restore all settings to factory defaults.  **Irreversible without a backup.**

```
hitecdpc --port /dev/ttyUSB0 factory-reset
```

JSON output:
```json
{"command": "factory-reset", "error": null}
```

---

### `monitor [--interval N] [REGISTERS...]`

Stream live telemetry at a regular interval.  Prints one line per sample.
Press Ctrl-C to stop.

```bash
# Default registers (position velocity torque voltage temperature), 1 s interval
hitecdpc --port /dev/ttyUSB0 monitor

# Custom registers, 0.5 s interval
hitecdpc --port /dev/ttyUSB0 monitor --interval 0.5 position torque

# Machine-readable stream
hitecdpc --port /dev/ttyUSB0 --json monitor --interval 0.1 position
```

Human output (one line per sample):
```
ts=1746316800.123  position=2048  velocity=0  torque=150  voltage=6000  temperature=35
```

JSON output (one line per sample, flushed immediately):
```json
{"command": "monitor", "ts": 1746316800.123, "data": {"position": 2048, "velocity": 0, "torque": 150, "voltage": 6000, "temperature": 35}}
```

Registers that fail for a given sample appear as `null` in JSON or `ERR` in
human mode; the stream continues.

---

## Error output

All errors go to **stderr** with exit code 1 or 2, regardless of `--json`.
When `--json` is set, a JSON error object is also printed to **stdout** before
exiting:

```json
{"error": "timeout: no response from servo"}
```

This lets agents detect failure by checking either the exit code or the `error`
key in the last stdout line.

---

## D-series register reference

All D-series registers are 16-bit.  Addresses are even integers.

| Name | Addr | R/W | Notes |
|------|-----:|-----|-------|
| `product_no` | 0 | R | Product number |
| `product_version` | 2 | R | Product version |
| `firmware_version` | 4 | R | Firmware version |
| `ic_serial_sub` | 6 | R | |
| `ic_serial_main` | 8 | R | |
| `status` | 10 | R | Status flags (see protocol.md for bit map) |
| `position` | 12 | R | Current position APV — `HD_REG_CURRENT_APV` |
| `velocity` | 14 | R | Current velocity |
| `torque` | 16 | R | Current torque |
| `voltage` | 18 | R | Supply voltage |
| `temperature` | 20 | R | Internal temperature |
| `current` | 26 | R | Current draw |
| `position_new` | 30 | R | Last commanded position |
| `servo_type` | 48 | R/W | Servo type code |
| `servo_id` | 50 | R/W | Bus ID (0–254); factory default 0 |
| `baudrate` | 52 | R/W | Baud rate code |
| `signal_mode` | 54 | R/W | |
| `simple_return_delay` | 56 | R/W | |
| `normal_return_delay` | 58 | R/W | |
| `deadband` | 78 | R/W | Dead band |
| `position_max` | 80 | R/W | Maximum allowed position |
| `position_min` | 82 | R/W | Minimum allowed position |
| `velocity_max` | 84 | R/W | Maximum velocity |
| `torque_max` | 86 | R/W | Maximum torque |
| `voltage_max` | 88 | R/W | Maximum voltage |
| `voltage_min` | 90 | R/W | Minimum voltage |
| `temperature_max` | 92 | R/W | Over-temperature threshold |
| `direction` | 94 | R/W | 0 = CCW, 1 = CW |
| `start_speed` | 96 | R/W | |
| `power_down_time` | 98 | R/W | |
| `position_slope` | 100 | R/W | |
| `factory_default` | 110 | W | Write any value → restore factory defaults |
| `config_save` | 112 | W | Write any value → save to flash |
| `lock` | 114 | R/W | |
| `pid_p` | 138 | R/W | PID proportional gain |
| `pid_d` | 140 | R/W | PID derivative gain |
| `pid_i` | 142 | R/W | PID integral gain |
| `pid_deadband` | 144 | R/W | PID dead band |
| `position_4095` | 148 | R/W | Calibration: position at encoder 4095 |
| `position_0` | 150 | R/W | Calibration: position at encoder 0 |

---

## Hardware setup (D-series direct)

The D-series uses 115200 baud 8N1 **inverted-polarity** UART on a single
half-duplex wire.  A USB-serial adapter alone is not enough — you need a
**signal inverter** between the adapter and the servo.

**Minimal inverter circuit:**

```
                      3.3 V
                        │
                       10 kΩ
                        │
USB-serial TX ──1 kΩ──┤B  NPN (e.g. 2N3904)
                       │E
                       GND
                       │C ──── servo signal line
                              (also tied to USB-serial RX via 1 kΩ)
```

Because TX and RX share the signal wire, the driver receives its own echo.
`hitecdpc` discards those echo bytes automatically (echo-cancel is on by
default).

**Alternative (no extra parts):** Use a Raspberry Pi Pico running
`src/hitecdpc/pico_dseries.py` — the PIO state machines handle inverted
UART in firmware.

---

## Agent usage patterns

### Read a register, check for error

```bash
out=$(hitecdpc --port /dev/ttyUSB0 --json read position)
if [ $? -ne 0 ]; then
  echo "comm failure: $(echo "$out" | jq -r .error)" >&2
  exit 1
fi
value=$(echo "$out" | jq .value)
```

### Tune a parameter and persist

```bash
hitecdpc --port /dev/ttyUSB0 --json write deadband 3 && \
hitecdpc --port /dev/ttyUSB0 --json save
```

### Dump all registers to a JSON file

```bash
hitecdpc --port /dev/ttyUSB0 --json dump > servo_snapshot.json
```

### Restore from snapshot (write every register back)

```bash
jq -r '.registers | to_entries[] | select(.value.error == null) |
       "\(.key) \(.value.value)"' servo_snapshot.json |
while read name value; do
  hitecdpc --port /dev/ttyUSB0 --json write "$name" "$value"
done
hitecdpc --port /dev/ttyUSB0 --json save
```

### Stream position to a file for 30 seconds

```bash
timeout 30 hitecdpc --port /dev/ttyUSB0 --json monitor \
  --interval 0.1 position >> position_log.jsonl
```
