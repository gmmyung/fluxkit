#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
//! Project-facing entry point for the Fluxkit motor-control workspace.
//!
//! `fluxkit` is the crate you use when integrating a real project:
//!
//! - define board-specific HAL implementations
//! - run motor calibration
//! - run actuator calibration
//! - construct [`MotorSystem`]
//! - drive it from a fixed-period interrupt with [`MotorSystem::tick`]
//! - interact from non-IRQ code through [`MotorHandle`]
//!
//! The lower-level crates are re-exported here:
//!
//! - [`fluxkit_core`]: pure deterministic control logic
//! - [`fluxkit_hal`]: narrow hardware contracts
//! - [`fluxkit_math`]: units, transforms, modulation, estimators

pub mod calibration;
pub mod system;

pub use calibration::{
    ActuatorCalibrationLimits, ActuatorCalibrationPhase, ActuatorCalibrationRequest,
    ActuatorCalibrationResult, ActuatorCalibrationSystem, ActuatorCalibrationSystemError,
    MotorCalibrationLimits, MotorCalibrationPhase, MotorCalibrationRequest, MotorCalibrationResult,
    MotorCalibrationSystem, MotorCalibrationSystemError,
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
pub use system::{MotorCommand, MotorHandle, MotorRuntimeStatus};
pub use system::{MotorHardware, MotorSystem, MotorSystemError};
