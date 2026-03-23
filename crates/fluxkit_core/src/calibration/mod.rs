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
//!   - phase resistance
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

pub mod actuator_blend_band;
pub mod actuator_breakaway;
pub mod actuator_friction;
pub mod actuator_gear_ratio;
pub mod actuator_result;
pub mod error;
pub mod flux_linkage;
pub mod phase_inductance;
pub mod phase_resistance;
pub mod pole_pairs_and_offset;
pub mod result;
pub mod routine;

pub use actuator_blend_band::{
    ActuatorBlendBandCalibrationCommand, ActuatorBlendBandCalibrationConfig,
    ActuatorBlendBandCalibrationInput, ActuatorBlendBandCalibrationResult,
    ActuatorBlendBandCalibrationState, ActuatorBlendBandCalibrator,
};
pub use actuator_breakaway::{
    ActuatorBreakawayCalibrationCommand, ActuatorBreakawayCalibrationConfig,
    ActuatorBreakawayCalibrationInput, ActuatorBreakawayCalibrationResult,
    ActuatorBreakawayCalibrationState, ActuatorBreakawayCalibrator,
};
pub use actuator_friction::{
    ActuatorFrictionCalibrationCommand, ActuatorFrictionCalibrationConfig,
    ActuatorFrictionCalibrationInput, ActuatorFrictionCalibrationResult,
    ActuatorFrictionCalibrationState, ActuatorFrictionCalibrator,
};
pub use actuator_gear_ratio::{
    ActuatorGearRatioCalibrationCommand, ActuatorGearRatioCalibrationConfig,
    ActuatorGearRatioCalibrationInput, ActuatorGearRatioCalibrationResult,
    ActuatorGearRatioCalibrationState, ActuatorGearRatioCalibrator,
};
pub use actuator_result::ActuatorCalibration;
pub use error::CalibrationError;
pub use flux_linkage::{
    FluxLinkageCalibrationConfig, FluxLinkageCalibrationInput, FluxLinkageCalibrationResult,
    FluxLinkageCalibrationState, FluxLinkageCalibrator,
};
pub use phase_inductance::{
    PhaseInductanceCalibrationConfig, PhaseInductanceCalibrationInput,
    PhaseInductanceCalibrationResult, PhaseInductanceCalibrationState, PhaseInductanceCalibrator,
};
pub use phase_resistance::{
    PhaseResistanceCalibrationConfig, PhaseResistanceCalibrationInput,
    PhaseResistanceCalibrationResult, PhaseResistanceCalibrationState, PhaseResistanceCalibrator,
};
pub use pole_pairs_and_offset::{
    PolePairsAndOffsetCalibrationConfig, PolePairsAndOffsetCalibrationInput,
    PolePairsAndOffsetCalibrationResult, PolePairsAndOffsetCalibrationState,
    PolePairsAndOffsetCalibrator,
};
pub use result::MotorCalibration;
pub use routine::{
    ActuatorCalibrationRoutine, ActuatorCalibrationRoutineResult, MotorCalibrationRoutine,
    MotorCalibrationRoutineResult,
};
