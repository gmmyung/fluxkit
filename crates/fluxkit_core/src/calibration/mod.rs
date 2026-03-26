//! Calibration procedures and persisted calibration records.
//!
//! Calibration logic in `fluxkit-core` stays pure and synchronous. Procedures
//! consume measured signals, emit explicit drive requests, and return typed
//! calibration results without owning hardware.
//!
//! Current implemented flow:
//!
//! - motor electrical:
//!   - pole pairs + electrical angle offset
//!   - phase resistance normalized to `25°C`
//!   - phase inductance
//!   - flux linkage
//! - actuator friction:
//!   - gear ratio
//!   - Coulomb + viscous friction
//!   - breakaway torque
//!   - zero-velocity blend band
//!
//! The result records are intentionally incremental:
//!
//! - [`MotorCalibration`]
//! - [`ActuatorCalibration`]
//!
//! That lets board/runtime code run multiple procedures, merge the completed
//! results, then apply the accumulated values onto parameter structs.
//!
//! The intended usage model is deliberately explicit:
//!
//! 1. construct one routine
//! 2. tick it until it produces a calibration delta
//! 3. merge that delta into the persisted record
//! 4. apply the merged record onto the live parameter struct

pub mod actuator;
pub mod motor;
pub mod shared;

pub use actuator::{
    ActuatorBlendBandCalibrationCommand, ActuatorBlendBandCalibrationConfig,
    ActuatorBlendBandCalibrationInput, ActuatorBlendBandCalibrationResult,
    ActuatorBlendBandCalibrator, ActuatorBreakawayCalibrationCommand,
    ActuatorBreakawayCalibrationConfig, ActuatorBreakawayCalibrationInput,
    ActuatorBreakawayCalibrationResult, ActuatorBreakawayCalibrator, ActuatorCalibration,
    ActuatorFrictionCalibration, ActuatorFrictionCalibrationCommand,
    ActuatorFrictionCalibrationConfig, ActuatorFrictionCalibrationInput,
    ActuatorFrictionCalibrationResult, ActuatorFrictionCalibrator,
    ActuatorGearRatioCalibrationCommand, ActuatorGearRatioCalibrationConfig,
    ActuatorGearRatioCalibrationInput, ActuatorGearRatioCalibrationResult,
    ActuatorGearRatioCalibrator,
};
pub use motor::{
    FluxLinkageCalibrationConfig, FluxLinkageCalibrationInput, FluxLinkageCalibrationResult,
    FluxLinkageCalibrator, MotorCalibration, PhaseInductanceCalibrationConfig,
    PhaseInductanceCalibrationInput, PhaseInductanceCalibrationResult, PhaseInductanceCalibrator,
    PhaseResistanceCalibrationConfig, PhaseResistanceCalibrationInput,
    PhaseResistanceCalibrationResult, PhaseResistanceCalibrator,
    PolePairsAndOffsetCalibrationConfig, PolePairsAndOffsetCalibrationInput,
    PolePairsAndOffsetCalibrationResult, PolePairsAndOffsetCalibrator,
};
pub use shared::{ActuatorCalibrationRoutine, CalibrationError, MotorCalibrationRoutine};
