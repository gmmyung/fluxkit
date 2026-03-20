#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
//! Workspace entry point for the `fluxkit` multi-crate library.
//!
//! The core crates live in [`fluxkit_core`], [`fluxkit_hal`], and
//! [`fluxkit_math`], and are re-exported here so downstream users can depend on
//! `fluxkit` as the umbrella crate.

pub use fluxkit_core as core;
pub use fluxkit_core::{
    AngleSource, ControlMode, CurrentLoopConfig, FastLoopInput, FastLoopOutput, FaultKind,
    InverterParams, MotorCommand, MotorController, MotorParams, MotorState, MotorStatus,
    RotorEstimate,
};
pub use fluxkit_hal as hal;
pub use fluxkit_hal::{
    BusVoltageSensor, CurrentSampleValidity, CurrentSampler, FaultInput, GateDriver,
    GateDriverFault, MonotonicMicros, PhaseCurrentSample, PhasePwm, RotorReading, RotorSensor,
    RotorSensorKind, TemperatureSensor, centered_phase_duty,
};
pub use fluxkit_math as math;
pub use fluxkit_math::*;
