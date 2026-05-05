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

use clap::{Parser, Subcommand};
use hitecdpc::dseries::{self, DSeriesTransport};
use serde_json::json;
use std::process;
use std::time::{SystemTime, UNIX_EPOCH};

// ── CLI definition ────────────────────────────────────────────────────────────

#[derive(Parser)]
#[command(
    name = "hitecdpc",
    about = "Read, write, and monitor Hitec digital servos",
    long_about = "\
Read, write, and monitor Hitec digital servos.\n\
\n\
Series d  — D-series (D485HW, D646WP, …): direct, no adapter needed.\n\
Series 57 — HS-5/7XXX via DPC-11/20 adapter (not yet implemented).\n\
Series 9  — HSB-9XXX via DPC-11/20 adapter (not yet implemented).\n\
\n\
Exit codes: 0 = success, 1 = communication error, 2 = usage error."
)]
struct Cli {
    /// Serial port device (e.g. /dev/ttyUSB0, /dev/cu.usbserial-XXXX, COM3)
    #[arg(long)]
    port: String,

    /// Servo series
    #[arg(long, default_value = "d")]
    series: String,

    /// Servo bus ID (0–254; factory default is 0)
    #[arg(long, default_value = "0")]
    id: u8,

    /// Emit machine-readable JSON on stdout (errors still go to stderr)
    #[arg(long)]
    json: bool,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Read all known registers and print a summary table
    Dump,

    /// Read one register by name or address (decimal or 0x hex)
    Read {
        register: String,
    },

    /// Write a 16-bit value to a register by name or address
    Write {
        register: String,
        /// Value to write (decimal or 0x hex)
        value: String,
        /// Skip the 5 ms post-write delay (use with caution)
        #[arg(long)]
        no_delay: bool,
    },

    /// Persist the current configuration to the servo's flash memory
    Save,

    /// Restore all settings to factory defaults (irreversible)
    #[command(name = "factory-reset")]
    FactoryReset,

    /// Decode and print the servo model name (reads product_no and product_version)
    #[command(name = "model-name")]
    ModelName,

    /// Stream live telemetry at a regular interval (Ctrl-C to stop)
    Monitor {
        /// Poll interval in seconds
        #[arg(long, default_value = "1.0")]
        interval: f64,

        /// Registers to poll (default: position velocity torque voltage temperature)
        registers: Vec<String>,
    },
}

// ── Output helpers ────────────────────────────────────────────────────────────

fn die(msg: impl std::fmt::Display, use_json: bool) -> ! {
    if use_json {
        println!("{}", json!({"error": msg.to_string()}));
    } else {
        eprintln!("error: {msg}");
    }
    process::exit(1);
}

fn die2(msg: impl std::fmt::Display, use_json: bool) -> ! {
    if use_json {
        println!("{}", json!({"error": msg.to_string()}));
    } else {
        eprintln!("error: {msg}");
    }
    process::exit(2);
}

fn ts() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

// ── Command handlers ──────────────────────────────────────────────────────────

fn cmd_dump(t: &mut DSeriesTransport, use_json: bool) {
    let mut results = serde_json::Map::new();

    if !use_json {
        println!("{:<24} {:>4}  {:>7}  {:>8}", "REGISTER", "ADDR", "VALUE", "HEX");
        println!("{}", "-".repeat(48));
    }

    for &(name, addr) in dseries::ALL_REGS {
        match t.read_register(addr) {
            Ok(value) => {
                if use_json {
                    results.insert(
                        name.to_string(),
                        json!({"addr": addr, "value": value, "error": null}),
                    );
                } else {
                    println!("{:<24} {:>4}  {:>7}  {:#010X}", name, addr, value, value);
                }
            }
            Err(e) => {
                if use_json {
                    results.insert(
                        name.to_string(),
                        json!({"addr": addr, "value": null, "error": e.to_string()}),
                    );
                } else {
                    println!("{:<24} {:>4}  {:>7}", name, addr, "ERROR");
                }
            }
        }
    }

    if use_json {
        println!("{}", json!({"command": "dump", "series": "d", "registers": results}));
    }
}

fn cmd_read(t: &mut DSeriesTransport, token: &str, use_json: bool) {
    let (name, addr) = match dseries::resolve_reg(token) {
        Ok(r) => r,
        Err(e) => die2(e, use_json),
    };
    match t.read_register(addr) {
        Ok(value) => {
            if use_json {
                println!(
                    "{}",
                    json!({"command":"read","register":name,"addr":addr,"value":value,"error":null})
                );
            } else {
                println!("{name}  addr=0x{addr:02X}  value={value}  (0x{value:04X})");
            }
        }
        Err(e) => die(e, use_json),
    }
}

fn cmd_write(t: &mut DSeriesTransport, token: &str, raw_value: &str, use_json: bool) {
    let (name, addr) = match dseries::resolve_reg(token) {
        Ok(r) => r,
        Err(e) => die2(e, use_json),
    };
    let value: u16 = {
        let s = raw_value;
        let parsed = if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
            u16::from_str_radix(hex, 16).ok()
        } else {
            s.parse::<u16>().ok()
        };
        match parsed {
            Some(v) => v,
            None => die2(format!("Invalid value {raw_value:?} — expected u16 (decimal or 0x hex)"), use_json),
        }
    };
    match t.write_register(addr, value) {
        Ok(()) => {
            if use_json {
                println!(
                    "{}",
                    json!({"command":"write","register":name,"addr":addr,"value":value,"error":null})
                );
            } else {
                println!("wrote {name} (addr=0x{addr:02X}) = {value}");
            }
        }
        Err(e) => die(e, use_json),
    }
}

fn cmd_save(t: &mut DSeriesTransport, use_json: bool) {
    match t.save_config() {
        Ok(()) => {
            if use_json {
                println!("{}", json!({"command":"save","error":null}));
            } else {
                println!("config saved to flash");
            }
        }
        Err(e) => die(e, use_json),
    }
}

fn cmd_model_name(t: &mut DSeriesTransport, use_json: bool) {
    match t.read_model_name() {
        Ok(name) => {
            if use_json {
                println!("{}", json!({"command": "model-name", "model": name, "error": null}));
            } else {
                println!("model: {name}");
            }
        }
        Err(e) => die(e, use_json),
    }
}

fn cmd_factory_reset(t: &mut DSeriesTransport, use_json: bool) {
    match t.restore_factory_defaults() {
        Ok(()) => {
            if use_json {
                println!("{}", json!({"command":"factory-reset","error":null}));
            } else {
                println!("factory defaults restored");
            }
        }
        Err(e) => die(e, use_json),
    }
}

const DEFAULT_MONITOR_REGS: &[&str] =
    &["position", "velocity", "torque", "voltage", "temperature"];

fn cmd_monitor(t: &mut DSeriesTransport, interval: f64, reg_tokens: &[String], use_json: bool) {
    let regs: Vec<(&str, u8)> = if reg_tokens.is_empty() {
        DEFAULT_MONITOR_REGS
            .iter()
            .map(|&s| dseries::resolve_reg(s).unwrap())
            .collect()
    } else {
        reg_tokens
            .iter()
            .map(|s| match dseries::resolve_reg(s) {
                Ok(r) => r,
                Err(e) => die2(e, use_json),
            })
            .collect()
    };

    let sleep = std::time::Duration::from_secs_f64(interval);

    loop {
        let timestamp = ts();
        let mut row = serde_json::Map::new();
        let mut parts = vec![format!("ts={timestamp:.3}")];

        for &(name, addr) in &regs {
            match t.read_register(addr) {
                Ok(v) => {
                    row.insert(name.to_string(), json!(v));
                    parts.push(format!("{name}={v}"));
                }
                Err(_) => {
                    row.insert(name.to_string(), json!(null));
                    parts.push(format!("{name}=ERR"));
                }
            }
        }

        if use_json {
            println!("{}", json!({"command":"monitor","ts":timestamp,"data":row}));
        } else {
            println!("{}", parts.join("  "));
        }

        std::thread::sleep(sleep);
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────

fn main() {
    let cli = Cli::parse();
    let use_json = cli.json;

    if cli.series != "d" {
        die2(
            format!(
                "--series {} is not yet implemented in the Rust CLI; \
                 use the Python CLI (`hitecdpc`) for series 57/9",
                cli.series
            ),
            use_json,
        );
    }

    let mut transport = match DSeriesTransport::new(&cli.port, cli.id) {
        Ok(t) => t,
        Err(e) => die2(format!("Failed to open {}: {e}", cli.port), use_json),
    };

    match &cli.command {
        Command::Dump => cmd_dump(&mut transport, use_json),
        Command::Read { register } => cmd_read(&mut transport, register, use_json),
        Command::Write { register, value, .. } => {
            cmd_write(&mut transport, register, value, use_json)
        }
        Command::Save => cmd_save(&mut transport, use_json),
        Command::FactoryReset => cmd_factory_reset(&mut transport, use_json),
        Command::ModelName => cmd_model_name(&mut transport, use_json),
        Command::Monitor { interval, registers } => {
            cmd_monitor(&mut transport, *interval, registers, use_json)
        }
    }
}
