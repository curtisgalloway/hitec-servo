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

//! `hitecdpc` — cross-platform Rust library for Hitec DPC digital servo programmers.
//!
//! # Modules
//!
//! - [`crc`] — CRC-8 (Dallas/Maxim 1-Wire) and CRC-16 (MODBUS) implementations.
//! - [`protocol`] — HS-5/7XXX packet framing (STX/ETX, KSO wrapper, servo packets).
//! - [`dseries`] — D-series servo direct protocol (no adapter needed).
//!
//! # Quick start — D-series (D485HW, D646WP, …)
//!
//! ```no_run
//! use hitecdpc::dseries::{DSeriesTransport, regs};
//!
//! let mut servo = DSeriesTransport::new("/dev/ttyUSB0", 0).unwrap();
//! let position = servo.read_register(regs::POSITION).unwrap();
//! println!("position APV: {position}");
//! servo.write_register(regs::DEADBAND, 2).unwrap();
//! servo.save_config().unwrap();
//! ```

pub mod crc;
pub mod dseries;
pub mod protocol;

#[cfg(feature = "transport")]
pub use dseries::DSeriesTransport;
pub use dseries::Error as DSeriesError;
