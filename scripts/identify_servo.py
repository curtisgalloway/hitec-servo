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
Read D-series servo identity through a Pico's CircuitPython REPL.

Usage:
    uv run --with pyserial python3 scripts/identify_servo.py [PORT]

PORT defaults to /dev/cu.usbmodem1101.
"""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem1101"

# Code to send via paste mode — reads identity registers and prints results.
CODE = """\
import board
from pico_dseries import DServoComm
servo = DServoComm(signal_pin=board.GP15, servo_id=0)
regs = [
    ('product_no',        0),
    ('product_version',   2),
    ('firmware_version',  4),
    ('ic_serial_main',    8),
    ('servo_type',       48),
    ('servo_id',         50),
    ('status',           10),
]
print('--- servo identity ---')
for name, addr in regs:
    v, e = servo.read_register(addr)
    if e:
        print(f'  {name:<20} ERR: {e}')
    else:
        print(f'  {name:<20} {v}  (0x{v:04X})')
print('--- done ---')
"""

print(f"Connecting to {PORT} …")
with serial.Serial(PORT, 115200, timeout=0.5) as port:
    # Interrupt whatever is running
    port.write(b"\x03\x03")
    time.sleep(0.8)
    port.reset_input_buffer()

    # Enter CircuitPython paste mode (Ctrl-E)
    port.write(b"\x05")
    time.sleep(0.3)

    # Send code one line at a time — paste mode accepts raw newlines
    for line in CODE.splitlines(keepends=True):
        port.write(line.encode())
        time.sleep(0.02)

    # Execute (Ctrl-D exits paste mode and runs the code)
    port.write(b"\x04")

    # Collect output until we see "--- done ---" or time out
    output = ""
    deadline = time.time() + 15.0
    while time.time() < deadline:
        chunk = port.read(port.in_waiting or 1)
        if chunk:
            output += chunk.decode("utf-8", errors="replace")
            # The sentinel also appears in the paste-mode echo of print('--- done ---'),
            # so wait for the second occurrence — the real executed output.
            if output.count("--- done ---") >= 2:
                break
        time.sleep(0.05)

# Print just the interesting part — use rfind to skip the paste-mode echo
start = output.rfind("--- servo identity ---")
if start != -1:
    print(output[start:].split("--- done ---")[0] + "--- done ---")
else:
    print("No response — check wiring and servo power.")
    print("Raw output:")
    print(repr(output[-500:]))
