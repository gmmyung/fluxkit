#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
//! Deterministic `no_std` field-oriented control engine for Fluxkit.
//!
//! `fluxkit_core` owns pure control logic only. It consumes validated loop
//! inputs, runs synchronous control math, and emits structured duty commands
//! and status snapshots without owning hardware resources or executor state.

pub mod config;
pub mod control;
pub mod error;
pub mod io;
pub mod mode;
pub mod motor;
pub mod params;
pub mod state;
pub mod status;
pub mod util;
pub mod validation;

pub use config::CurrentLoopConfig;
pub use error::Error;
pub use io::{FastLoopInput, FastLoopOutput, RotorEstimate};
pub use mode::ControlMode;
pub use motor::MotorController;
pub use params::{InverterParams, MotorParams};
pub use state::MotorState;
pub use status::MotorStatus;
