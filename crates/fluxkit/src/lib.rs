#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
//! Workspace entry point for the `fluxkit` multi-crate library.
//!
//! The core crates live in [`fluxkit_core`], [`fluxkit_hal`], and
//! [`fluxkit_math`], and are re-exported here so downstream users can depend on
//! `fluxkit` as the umbrella crate.

pub mod calibration;
pub mod system;

pub use calibration::{
    ActuatorCalibrationLimits, ActuatorCalibrationRequest, ActuatorCalibrationResult,
    ActuatorCalibrationSystem, ActuatorCalibrationSystemError, MotorCalibrationLimits,
    MotorCalibrationRequest, MotorCalibrationResult, MotorCalibrationSystem,
    MotorCalibrationSystemError,
};
pub use fluxkit_core as core;
pub use fluxkit_core::{
    ActuatorCompensationConfig, ActuatorCompensationTelemetry, ActuatorEstimate, ActuatorLimits,
    ActuatorModel, ActuatorParams, CalibrationError, ControlMode, CurrentLoopConfig, Error,
    FastLoopInput, FastLoopOutput, FrictionCompensation, InverterParams, MotorController,
    MotorLimits, MotorModel, MotorParams, MotorState, MotorStatus, RotorEstimate, TickSchedule,
};
pub use fluxkit_hal as hal;
pub use fluxkit_hal::{
    BusVoltageSensor, CurrentSampleValidity, CurrentSampler, FaultInput, GateDriver,
    GateDriverFault, MonotonicMicros, OutputReading, OutputSensor, PhaseCurrentSample, PhasePwm,
    RotorReading, RotorSensor, TemperatureSensor, centered_phase_duty,
};
pub use fluxkit_math as math;
pub use fluxkit_math::*;
pub use system::{MotorCommand, MotorHandle, MotorRuntimeConfig, MotorRuntimeStatus};
pub use system::{MotorHardware, MotorSystem, MotorSystemError};
