#![no_std]
#![forbid(unsafe_code)]
#![deny(rust_2018_idioms)]
#![warn(missing_docs, missing_debug_implementations)]
#![doc = include_str!("../README.md")]

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
