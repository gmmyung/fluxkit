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
    ActuatorCalibrationSystem, ActuatorCalibrationSystemError, CalibrationTickResult,
    MotorCalibrationHardware, MotorCalibrationSystem, MotorCalibrationSystemError,
};
pub use fluxkit_core as core;
pub use fluxkit_core::{
    ActuatorBlendBandCalibrationCommand, ActuatorBlendBandCalibrationConfig,
    ActuatorBlendBandCalibrationInput, ActuatorBlendBandCalibrationResult,
    ActuatorBlendBandCalibrationState, ActuatorBlendBandCalibrator,
    ActuatorBreakawayCalibrationCommand, ActuatorBreakawayCalibrationConfig,
    ActuatorBreakawayCalibrationInput, ActuatorBreakawayCalibrationResult,
    ActuatorBreakawayCalibrationState, ActuatorBreakawayCalibrator, ActuatorCalibration,
    ActuatorCompensationConfig, ActuatorCompensationTelemetry, ActuatorEstimate,
    ActuatorFrictionCalibrationCommand, ActuatorFrictionCalibrationConfig,
    ActuatorFrictionCalibrationInput, ActuatorFrictionCalibrationResult,
    ActuatorFrictionCalibrationState, ActuatorFrictionCalibrator, ActuatorParams, CalibrationError,
    ControlMode, CurrentLoopConfig, Error, FastLoopInput, FastLoopOutput,
    FluxLinkageCalibrationConfig, FluxLinkageCalibrationInput, FluxLinkageCalibrationResult,
    FluxLinkageCalibrationState, FluxLinkageCalibrator, FrictionCompensation, InverterParams,
    MotorCalibration, MotorController, MotorParams, MotorState, MotorStatus,
    PhaseInductanceCalibrationConfig, PhaseInductanceCalibrationInput,
    PhaseInductanceCalibrationResult, PhaseInductanceCalibrationState, PhaseInductanceCalibrator,
    PhaseResistanceCalibrationConfig, PhaseResistanceCalibrationInput,
    PhaseResistanceCalibrationResult, PhaseResistanceCalibrationState, PhaseResistanceCalibrator,
    PolePairsAndOffsetCalibrationConfig, PolePairsAndOffsetCalibrationInput,
    PolePairsAndOffsetCalibrationResult, PolePairsAndOffsetCalibrationState,
    PolePairsAndOffsetCalibrator, RotorEstimate, TickSchedule,
};
pub use fluxkit_hal as hal;
pub use fluxkit_hal::{
    BusVoltageSensor, CurrentSampleValidity, CurrentSampler, FaultInput, GateDriver,
    GateDriverFault, MonotonicMicros, OutputReading, OutputSensor, PhaseCurrentSample, PhasePwm,
    RotorReading, RotorSensor, TemperatureSensor, centered_phase_duty,
};
pub use fluxkit_math as math;
pub use fluxkit_math::*;
pub use system::{MotorHardware, MotorSystem, MotorSystemError};
