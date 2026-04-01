#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
//! Deterministic `no_std` field-oriented control engine for Fluxkit.
//!
//! `fluxkit_core` owns pure control logic only. It consumes validated loop
//! inputs, runs synchronous control math, and emits structured duty commands
//! and status snapshots without owning hardware resources or executor state.
//!
//! In the full workspace layering:
//!
//! - `fluxkit_math` provides units, transforms, modulation, and estimator primitives
//! - `fluxkit_core` provides the deterministic engine and pure calibration procedures
//! - `fluxkit_hal` provides synchronous hardware traits
//! - `fluxkit` provides the normal application-facing runtime and calibration wrappers
//!
//! Most application code should start at `fluxkit`, not at `fluxkit_core`.
//!
//! # Reference plot
//!
//! Representative closed-loop current response of the controller against the
//! ideal PMSM plant model used in integration tests.
#![doc = "\n"]
#![doc = include_str!("../docs/plots/closed_loop_current.svg")]

#[macro_use]
mod logging;

pub mod actuator;
pub mod calibration;
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

pub use actuator::{
    ActuatorCompensationConfig, ActuatorCompensationTelemetry, ActuatorEstimate, ActuatorLimits,
    ActuatorModel, ActuatorParams, FrictionCompensation,
};
pub use calibration::{
    ActuatorBlendBandCalibrationCommand, ActuatorBlendBandCalibrationConfig,
    ActuatorBlendBandCalibrationInput, ActuatorBlendBandCalibrationResult,
    ActuatorBlendBandCalibrator, ActuatorBreakawayCalibrationCommand,
    ActuatorBreakawayCalibrationConfig, ActuatorBreakawayCalibrationInput,
    ActuatorBreakawayCalibrationResult, ActuatorBreakawayCalibrator, ActuatorCalibration,
    ActuatorCalibrationRoutine, ActuatorFrictionCalibration, ActuatorFrictionCalibrationCommand,
    ActuatorFrictionCalibrationConfig, ActuatorFrictionCalibrationInput,
    ActuatorFrictionCalibrationResult, ActuatorFrictionCalibrator,
    ActuatorGearRatioCalibrationCommand, ActuatorGearRatioCalibrationConfig,
    ActuatorGearRatioCalibrationInput, ActuatorGearRatioCalibrationResult,
    ActuatorGearRatioCalibrator, CalibrationError, FluxLinkageCalibrationConfig,
    FluxLinkageCalibrationInput, FluxLinkageCalibrationResult, FluxLinkageCalibrator,
    MotorCalibration, MotorCalibrationRoutine, PhaseInductanceCalibrationConfig,
    PhaseInductanceCalibrationInput, PhaseInductanceCalibrationResult, PhaseInductanceCalibrator,
    PhaseResistanceCalibrationConfig, PhaseResistanceCalibrationInput,
    PhaseResistanceCalibrationResult, PhaseResistanceCalibrator,
    PolePairsAndOffsetCalibrationConfig, PolePairsAndOffsetCalibrationInput,
    PolePairsAndOffsetCalibrationResult, PolePairsAndOffsetCalibrator,
};
pub use config::{CurrentLoopConfig, CurrentLoopConfigBuilder};
pub use control::current::{
    CurrentEstimator, CurrentReference, LpfCurrentEstimator, LpfCurrentEstimatorConfig,
    PassThroughCurrentEstimator,
};
pub use error::Error;
pub use io::{FastLoopInput, FastLoopOutput, RotorEstimate};
pub use mode::ControlMode;
pub use motor::MotorController;
pub use params::{InverterParams, MotorLimits, MotorModel, MotorParams};
pub use state::MotorState;
pub use status::MotorStatus;
